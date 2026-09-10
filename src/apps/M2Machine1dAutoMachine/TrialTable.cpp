#include "TrialTable.h"

#include <spdlog/spdlog.h>

#include <cctype>
#include <cmath>
#include <fstream>
#include <sstream>

namespace {

//! Split a CSV line on commas. Tolerates surrounding whitespace.
std::vector<std::string> splitCsv(const std::string &line) {
    std::vector<std::string> cells;
    std::stringstream ss(line);
    std::string cell;
    while (std::getline(ss, cell, ',')) {
        size_t b = cell.find_first_not_of(" \t\r\n");
        size_t e = cell.find_last_not_of(" \t\r\n");
        cells.push_back(b == std::string::npos ? "" : cell.substr(b, e - b + 1));
    }
    return cells;
}

bool looksNumeric(const std::string &s) {
    if (s.empty()) return false;
    return std::isdigit(static_cast<unsigned char>(s[0])) || s[0] == '-' || s[0] == '+' || s[0] == '.';
}

}  // namespace

bool TrialTable::load(const std::string &path, double unitScale, double minTarget,
                      double maxTarget, double home, double direction) {
    trials_.clear();
    formulation_ = "unknown";

    std::ifstream f(path);
    if (!f.is_open()) {
        spdlog::critical("TrialTable: cannot open '{}'. Place it next to the executable.", path);
        return false;
    }

    std::string line;
    size_t lineNo = 0;
    size_t skipped = 0;
    int matchFitts = 0, matchShannon = 0, matchNeither = 0;
    bool reachable = true;

    while (std::getline(f, line)) {
        lineNo++;
        if (line.empty()) continue;
        if (!line.empty() && line.back() == '\r') line.pop_back();

        std::vector<std::string> c = splitCsv(line);
        if (c.size() < 3) {
            spdlog::warn("TrialTable: line {} has {} columns, expected >=3. Skipped.", lineNo, c.size());
            skipped++;
            continue;
        }
        if (!looksNumeric(c[0])) {  // header row
            spdlog::debug("TrialTable: treating line {} as header.", lineNo);
            continue;
        }

        Trial t;
        try {
            t.index  = std::stoi(c[0]);
            t.A      = std::stod(c[1]) * unitScale;
            t.W      = std::stod(c[2]) * unitScale;
            t.IDfile = (c.size() > 3 && looksNumeric(c[3])) ? std::stod(c[3]) : 0.0;
        } catch (const std::exception &) {
            spdlog::warn("TrialTable: line {} unparsable. Skipped.", lineNo);
            skipped++;
            continue;
        }

        if (t.A <= 0.0 || t.W <= 0.0) {
            spdlog::error("TrialTable: trial {} has non-positive A ({}) or W ({}).", t.index, t.A, t.W);
            skipped++;
            continue;
        }

        t.IDfitts   = std::log2(2.0 * t.A / t.W);
        t.IDshannon = std::log2(t.A / t.W + 1.0);

        if (t.IDfile > 0.0) {
            if (std::fabs(t.IDfile - t.IDfitts) < 0.02)
                matchFitts++;
            else if (std::fabs(t.IDfile - t.IDshannon) < 0.02)
                matchShannon++;
            else
                matchNeither++;
        }

        // Reachability of the derived target.
        const double target = home + direction * t.A;
        if (target < minTarget || target > maxTarget) {
            spdlog::critical(
                "TrialTable: trial {} target {:.4f} m is outside [{:.4f}, {:.4f}]. "
                "Move HOME or check UNIT_SCALE.",
                t.index, target, minTarget, maxTarget);
            reachable = false;
        }

        trials_.push_back(t);
    }

    if (trials_.empty()) {
        spdlog::critical("TrialTable: no usable trials in '{}'.", path);
        return false;
    }

    // Report which ID formulation the file column matches.
    if (matchFitts + matchShannon + matchNeither == 0) {
        formulation_ = "no ID column";
    } else if (matchFitts >= matchShannon && matchFitts > matchNeither) {
        formulation_ = "Fitts log2(2A/W)";
    } else if (matchShannon > matchFitts && matchShannon > matchNeither) {
        formulation_ = "Shannon log2(A/W+1)";
    } else {
        formulation_ = "NEITHER";
        spdlog::error(
            "TrialTable: the ID column matches neither log2(2A/W) nor log2(A/W+1) "
            "on {} of {} rows. Check for transposed A/W columns.",
            matchNeither, trials_.size());
    }

    double aMin = trials_[0].A, aMax = trials_[0].A, wMin = trials_[0].W, wMax = trials_[0].W;
    double idMin = trials_[0].IDfitts, idMax = trials_[0].IDfitts;
    for (const Trial &t : trials_) {
        aMin = std::min(aMin, t.A); aMax = std::max(aMax, t.A);
        wMin = std::min(wMin, t.W); wMax = std::max(wMax, t.W);
        idMin = std::min(idMin, t.IDfitts); idMax = std::max(idMax, t.IDfitts);
    }

    spdlog::info("TrialTable: loaded {} trials from '{}' ({} skipped).", trials_.size(), path, skipped);
    spdlog::info("TrialTable: A {:.4f}-{:.4f} m, W {:.4f}-{:.4f} m, ID(Fitts) {:.2f}-{:.2f} bits.",
                 aMin, aMax, wMin, wMax, idMin, idMax);
    spdlog::info("TrialTable: CSV ID column matches: {}", formulation_);

    if (!reachable) {
        spdlog::critical("TrialTable: one or more targets unreachable. Refusing to start.");
        return false;
    }
    return true;
}