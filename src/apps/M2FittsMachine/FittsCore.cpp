/**
 * \file FittsCore.cpp
 * \brief Implementation of the hardware-independent core of M2FittsMachine (see FittsCore.h).
 *
 * Provenance: drafted with AI assistance (Claude, Anthropic), September 2026. Licence: Apache-2.0.
 */
#include "FittsCore.h"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <iterator>
#include <sstream>
#include <stdexcept>

namespace fitts {

namespace {

const double kPi = 3.14159265358979323846;

std::string trimWs(const std::string &s) {
    const char *ws = " \t\r\n";
    const std::size_t b = s.find_first_not_of(ws);
    if (b == std::string::npos) return "";
    const std::size_t e = s.find_last_not_of(ws);
    return s.substr(b, e - b + 1);
}

/** Split a CSV line on 'delim', honouring double-quoted fields ("" inside quotes = literal quote). */
std::vector<std::string> splitCsvLine(const std::string &line, char delim) {
    std::vector<std::string> out;
    std::string cur;
    bool in_quotes = false;
    for (std::size_t i = 0; i < line.size(); ++i) {
        const char c = line[i];
        if (c == '"') {
            if (in_quotes && i + 1 < line.size() && line[i + 1] == '"') {
                cur += '"';
                ++i;
            } else {
                in_quotes = !in_quotes;
            }
        } else if (c == delim && !in_quotes) {
            out.push_back(trimWs(cur));
            cur.clear();
        } else {
            cur += c;
        }
    }
    out.push_back(trimWs(cur));
    return out;
}

/** ';' or TAB if present (locale-specific spreadsheet exports), otherwise ','. */
char detectDelimiter(const std::string &line) {
    if (line.find(';') != std::string::npos) return ';';
    if (line.find('\t') != std::string::npos) return '\t';
    return ',';
}

bool parseNumber(std::string s, char delim, double &out) {
    s = trimWs(s);
    if (s.empty()) return false;
    if (s.find(',') != std::string::npos) {
        if (delim == ',') return false;              // ambiguous: refuse rather than guess
        std::replace(s.begin(), s.end(), ',', '.');  // decimal comma in ';' or TAB separated files
    }
    errno = 0;
    char *end = nullptr;
    const double v = std::strtod(s.c_str(), &end);
    if (end == s.c_str() || *end != '\0' || errno == ERANGE || !std::isfinite(v)) return false;
    out = v;
    return true;
}

std::uint64_t fnv1a64(const std::string &bytes) {
    std::uint64_t h = 1469598103934665603ULL;
    for (unsigned char c : bytes) {
        h ^= c;
        h *= 1099511628211ULL;
    }
    return h;
}

std::string num(double v, int precision = 6) {
    if (!std::isfinite(v)) return "NaN";
    std::ostringstream os;
    os << std::fixed << std::setprecision(precision) << v;
    return os.str();
}

std::string sanitise(std::string s) {
    for (char &c : s)
        if (c == ',' || c == '"' || c == '\n' || c == '\r') c = ';';
    return s;
}

}  // namespace

int columnLetterToIndex(const std::string &letters) {
    const std::string s = trimWs(letters);
    if (s.empty() || s.size() > 3) return -1;
    int idx = 0;
    for (char c : s) {
        if (c >= 'a' && c <= 'z') c = static_cast<char>(c - 'a' + 'A');
        if (c < 'A' || c > 'Z') return -1;
        idx = idx * 26 + (c - 'A' + 1);
    }
    return idx - 1;
}

std::vector<Trial> loadTrialsCsv(const std::string &path, int d_col, int w_col, int expected_rows, CsvInfo *info) {
    if (d_col < 0 || w_col < 0) throw std::runtime_error("Invalid D/W column specification.");
    if (d_col == w_col) throw std::runtime_error("D and W columns must be different.");

    std::ifstream f(path, std::ios::binary);
    if (!f) throw std::runtime_error("Cannot open trial file '" + path + "'.");
    std::string content((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());

    CsvInfo local;
    local.path = path;
    local.fnv1a64 = fnv1a64(content);
    if (content.size() >= 3 && static_cast<unsigned char>(content[0]) == 0xEF &&
        static_cast<unsigned char>(content[1]) == 0xBB && static_cast<unsigned char>(content[2]) == 0xBF) {
        content.erase(0, 3);  // UTF-8 byte-order mark (e.g. Excel "CSV UTF-8")
    }

    std::istringstream in(content);
    std::vector<Trial> trials;
    std::string line;
    int line_no = 0;
    bool delimiter_set = false, first_row = true;
    const std::size_t needed = static_cast<std::size_t>(std::max(d_col, w_col)) + 1;

    while (std::getline(in, line)) {
        ++line_no;
        if (!line.empty() && line.back() == '\r') line.pop_back();  // CRLF
        if (trimWs(line).empty()) continue;
        if (!delimiter_set) {
            local.delimiter = detectDelimiter(line);
            delimiter_set = true;
        }
        const std::vector<std::string> fields = splitCsvLine(line, local.delimiter);
        if (std::all_of(fields.begin(), fields.end(), [](const std::string &s) { return s.empty(); })) continue;

        double D = 0., W = 0.;
        const bool okD = fields.size() > static_cast<std::size_t>(d_col) && parseNumber(fields[d_col], local.delimiter, D);
        const bool okW = fields.size() > static_cast<std::size_t>(w_col) && parseNumber(fields[w_col], local.delimiter, W);

        if (first_row) {
            first_row = false;
            if (!okD && !okW) {  // header row
                local.header = line;
                continue;
            }
        }
        const std::string where = "Trial file '" + path + "', line " + std::to_string(line_no) + ": ";
        if (fields.size() < needed)
            throw std::runtime_error(where + "expected at least " + std::to_string(needed) + " columns, found " +
                                     std::to_string(fields.size()) + ".");
        if (!okD || !okW)
            throw std::runtime_error(where + "non-numeric value (D field '" + fields[d_col] + "', W field '" +
                                     fields[w_col] + "').");
        if (D <= 0. || W <= 0.) throw std::runtime_error(where + "D and W must both be > 0.");

        Trial t;
        t.index = static_cast<int>(trials.size()) + 1;
        t.csv_line = line_no;
        t.D = D;
        t.W = W;
        trials.push_back(t);
    }
    if (trials.empty()) throw std::runtime_error("Trial file '" + path + "' contains no trials.");
    if (expected_rows > 0 && static_cast<int>(trials.size()) != expected_rows)
        throw std::runtime_error("Trial file '" + path + "' contains " + std::to_string(trials.size()) +
                                 " trials but " + std::to_string(expected_rows) + " were expected.");
    if (info) *info = local;
    return trials;
}

double targetX(const Trial &t, const FittsConfig &c) { return c.home(0) + c.direction * t.D * c.units_to_m; }

double targetTolerance(const Trial &t, const FittsConfig &c) { return c.tolerance_factor * t.W * c.units_to_m; }

std::vector<std::string> validateConfig(const FittsConfig &c) {
    std::vector<std::string> e;
    auto req = [&e](bool ok, const std::string &msg) {
        if (!ok) e.push_back(msg);
    };
    const int dc = columnLetterToIndex(c.d_column), wc = columnLetterToIndex(c.w_column);
    req(dc >= 0, "trials.d_column must be a spreadsheet column letter (e.g. A)");
    req(wc >= 0, "trials.w_column must be a spreadsheet column letter (e.g. B)");
    req(dc != wc, "trials.d_column and trials.w_column must differ");
    req(c.units_to_m > 0., "trials.units_to_m must be > 0");
    req(c.expected_trials >= 0, "trials.expected_rows must be >= 0");
    req(c.start_trial >= 1, "trials.start_trial must be >= 1");
    req(c.direction == -1 || c.direction == 1, "task.direction must be -1 or +1");
    req(c.tolerance_factor > 0., "task.tolerance_factor must be > 0");
    req(c.dwell_time >= 0., "task.dwell_time must be >= 0");
    req(c.acquisition_timeout > 0., "task.acquisition_timeout must be > 0");
    req(c.onset_speed > 0., "task.onset_speed must be > 0");
    req(c.home_tolerance > 0. && c.rest_speed > 0. && c.rest_time >= 0. && c.settle_timeout > 0.,
        "home_settling: tolerance, rest_speed, settle_timeout must be > 0 and rest_time >= 0");
    req(c.return_ref_speed >= 0., "home_settling.return_ref_speed must be >= 0");
    req(c.homing_ref_speed > 0., "home_settling.homing_ref_speed must be > 0 (a step from the calibration corner is unsafe)");
    req((c.pd.kp.array() > 0.).all() && (c.pd.kd.array() >= 0.).all(), "pd.kp must be > 0 and pd.kd >= 0");
    req(c.pd.f_max > 0., "pd.f_max must be > 0");
    req(c.pd.vel_filter_hz >= 0., "pd.velocity_filter_hz must be >= 0");
    req((c.pd.stiction_comp.array() >= 0.).all() && c.pd.stiction_rest_speed > 0. && c.pd.stiction_deadband >= 0.,
        "pd.stiction_comp must be >= 0, pd.stiction_rest_speed > 0 and pd.stiction_deadband >= 0");
    req(c.max_speed > 0., "safety.max_speed must be > 0");
    req(c.brake_damping >= 0., "safety.brake_damping must be >= 0");
    req(c.workspace_x(0) < c.workspace_x(1) && c.workspace_y(0) < c.workspace_y(1), "safety.workspace ranges must be [min, max]");
    req(c.workspace_tolerance >= 0. && c.target_margin >= 0., "safety tolerances must be >= 0");
    req(c.home(0) >= c.workspace_x(0) + c.target_margin && c.home(0) <= c.workspace_x(1) - c.target_margin &&
            c.home(1) >= c.workspace_y(0) + c.target_margin && c.home(1) <= c.workspace_y(1) - c.target_margin,
        "task.home must lie inside the workspace minus target_margin");
    req(c.calib_force > 0. && c.calib_damping >= 0. && c.calib_still_speed > 0. && c.calib_still_time > 0.,
        "calibration parameters must be positive");
    req(!c.log_folder.empty(), "logging.folder must not be empty");
    return e;
}

std::vector<std::string> validateTrials(const std::vector<Trial> &trials, const FittsConfig &c) {
    std::vector<std::string> e;
    if (c.start_trial > static_cast<int>(trials.size()))
        e.push_back("trials.start_trial (" + std::to_string(c.start_trial) + ") exceeds the number of trials (" +
                    std::to_string(trials.size()) + ")");
    const double lo = c.workspace_x(0) + c.target_margin, hi = c.workspace_x(1) - c.target_margin;
    int n_bad = 0;
    for (const Trial &t : trials) {
        const double xt = targetX(t, c), tol = targetTolerance(t, c), Dm = t.D * c.units_to_m;
        std::string why;
        if (xt - tol < lo || xt + tol > hi)
            why = "target band [" + num(xt - tol, 4) + ", " + num(xt + tol, 4) + "] m leaves the allowed x range [" +
                  num(lo, 4) + ", " + num(hi, 4) + "] m";
        else if (Dm - tol <= c.home_tolerance)
            why = "home lies inside/too close to the target band (need D - tolerance_factor*W > home_tolerance)";
        if (!why.empty() && ++n_bad <= 10)
            e.push_back("trial " + std::to_string(t.index) + " (CSV line " + std::to_string(t.csv_line) +
                        ", D=" + num(t.D, 3) + ", W=" + num(t.W, 3) + "): " + why);
    }
    if (n_bad > 10) e.push_back("... and " + std::to_string(n_bad - 10) + " more invalid trials");
    return e;
}

std::string describeConfig(const FittsConfig &c) {
    std::ostringstream o;
    o << "trials.file: " << c.trials_file << "\n"
      << "trials.d_column: " << c.d_column << "\n"
      << "trials.w_column: " << c.w_column << "\n"
      << "trials.units_to_m: " << c.units_to_m << "\n"
      << "trials.expected_rows: " << c.expected_trials << "\n"
      << "trials.start_trial: " << c.start_trial << "\n"
      << "task.home_m: [" << c.home(0) << ", " << c.home(1) << "]\n"
      << "task.direction: " << c.direction << "\n"
      << "task.tolerance_factor: " << c.tolerance_factor << "\n"
      << "task.dwell_time_s: " << c.dwell_time << "\n"
      << "task.acquisition_timeout_s: " << c.acquisition_timeout << "\n"
      << "task.onset_speed_mps: " << c.onset_speed << "\n"
      << "home_settling.tolerance_m: " << c.home_tolerance << "\n"
      << "home_settling.rest_speed_mps: " << c.rest_speed << "\n"
      << "home_settling.rest_time_s: " << c.rest_time << "\n"
      << "home_settling.settle_timeout_s: " << c.settle_timeout << "\n"
      << "home_settling.return_ref_speed_mps: " << c.return_ref_speed << "\n"
      << "home_settling.homing_ref_speed_mps: " << c.homing_ref_speed << "\n"
      << "pd.kp_N_per_m: [" << c.pd.kp(0) << ", " << c.pd.kp(1) << "]\n"
      << "pd.kd_Ns_per_m: [" << c.pd.kd(0) << ", " << c.pd.kd(1) << "]\n"
      << "pd.f_max_N: " << c.pd.f_max << "\n"
      << "pd.velocity_filter_hz: " << c.pd.vel_filter_hz << "\n"
      << "pd.stiction_comp_N: [" << c.pd.stiction_comp(0) << ", " << c.pd.stiction_comp(1) << "] (below "
      << c.pd.stiction_rest_speed << " m/s, outside " << c.pd.stiction_deadband << " m of the set-point)\n"
      << "pd.friction_compensation: " << (c.friction_compensation ? "true" : "false") << "\n"
      << "safety.max_speed_mps: " << c.max_speed << "\n"
      << "safety.workspace_x_m: [" << c.workspace_x(0) << ", " << c.workspace_x(1) << "]\n"
      << "safety.workspace_y_m: [" << c.workspace_y(0) << ", " << c.workspace_y(1) << "]\n"
      << "safety.workspace_tolerance_m: " << c.workspace_tolerance << "\n"
      << "safety.target_margin_m: " << c.target_margin << "\n"
      << "safety.brake_damping_Ns_per_m: " << c.brake_damping << "\n"
      << "calibration.force_N: " << c.calib_force << "\n"
      << "calibration.damping_Ns_per_m: " << c.calib_damping << "\n"
      << "calibration.still_speed_mps: " << c.calib_still_speed << "\n"
      << "calibration.still_time_s: " << c.calib_still_time << "\n"
      << "logging.folder: " << c.log_folder << "\n";
    return o.str();
}

std::string localTimestamp(const char *fmt, bool with_ms) {
    using namespace std::chrono;
    const auto now = system_clock::now();
    const std::time_t tt = system_clock::to_time_t(now);
    std::tm tm_local{};
    localtime_r(&tt, &tm_local);
    char buf[64];
    std::strftime(buf, sizeof(buf), fmt, &tm_local);
    if (!with_ms) return buf;
    const long long ms = duration_cast<milliseconds>(now.time_since_epoch()).count() % 1000;
    char out[80];
    std::snprintf(out, sizeof(out), "%s.%03lld", buf, ms);
    return out;
}

// ------------------------------------------------------------------------------------------------ PD
Vec2 PDController::compute(const Vec2 &x_ref, const Vec2 &v_ref, const Vec2 &x, const Vec2 &v, double dt, bool *saturated) {
    if (!x.allFinite() || !v.allFinite() || !x_ref.allFinite() || !v_ref.allFinite()) {
        if (saturated) *saturated = true;
        return Vec2::Zero();  // never command a force computed from invalid data
    }
    // Velocity used by the D term: raw, or 1st-order low-pass filtered (exact discretisation)
    if (!initialised_ || g_.vel_filter_hz <= 0.) {
        v_filt_ = v;
        initialised_ = true;
    } else if (dt > 0.) {
        const double tau = 1. / (2. * kPi * g_.vel_filter_hz);
        const double a = 1. - std::exp(-dt / tau);
        v_filt_ += a * (v - v_filt_);
    }
    Vec2 F = g_.kp.cwiseProduct(x_ref - x) + g_.kd.cwiseProduct(v_ref - v_filt_);
    bool sat = false;
    for (int i = 0; i < 2; ++i) {
        if (F(i) > g_.f_max) {
            F(i) = g_.f_max;
            sat = true;
        } else if (F(i) < -g_.f_max) {
            F(i) = -g_.f_max;
            sat = true;
        }
    }
    // Stiction compensation while the handle is stuck away from the set-point: RobotM2 compensates friction only
    // above its 5 cm/s velocity threshold, so near rest the unsaturated spring can stall short of a small target.
    // Same term as fsc::SharedControlLaw in M2FittsRobotHumanMachine, where it is scaled by alpha.
    for (int i = 0; i < 2; ++i)
        if (g_.stiction_comp(i) > 0. && std::fabs(v(i)) < g_.stiction_rest_speed &&
            std::fabs(x_ref(i) - x(i)) > g_.stiction_deadband && F(i) != 0.)
            F(i) += (F(i) > 0.) ? g_.stiction_comp(i) : -g_.stiction_comp(i);
    if (saturated) *saturated = sat;
    return F;
}

// --------------------------------------------------------------------------------- Reference generator
void RateLimitedReference::reset(const Vec2 &start, const Vec2 &goal, double max_speed) {
    ref_ = start;
    goal_ = goal;
    vmax_ = max_speed;
    done_ = false;
}

Vec2 RateLimitedReference::update(double dt, Vec2 &v_ref) {
    v_ref.setZero();
    if (done_) return ref_;
    const Vec2 d = goal_ - ref_;
    const double dist = d.norm();
    if (vmax_ <= 0. || dist < 1e-9) {
        ref_ = goal_;
        done_ = true;
        return ref_;
    }
    const double step = vmax_ * std::min(std::max(dt, 0.), 0.01);  // dt clamped: no jump after a loop overrun
    if (dist <= step) {
        ref_ = goal_;
        done_ = true;
        return ref_;
    }
    ref_ += (d / dist) * step;
    v_ref = (d / dist) * vmax_;
    return ref_;
}

// ------------------------------------------------------------------------------------- Trial monitor
const char *outcomeName(TrialOutcome o) {
    switch (o) {
        case TrialOutcome::Success: return "success";
        case TrialOutcome::Timeout: return "timeout";
        case TrialOutcome::Aborted: return "aborted";
        default: return "running";
    }
}

void TrialMonitor::start(const Trial &trial, const FittsConfig &cfg, const Vec2 &x_start, const std::string &wall_clock) {
    result_ = TrialResult();
    result_.trial = trial;
    result_.wall_clock = wall_clock;
    result_.x_home = cfg.home(0);
    result_.y_home = cfg.home(1);
    result_.x_target = targetX(trial, cfg);
    result_.tolerance = targetTolerance(trial, cfg);
    result_.x_start = x_start(0);
    result_.D_effective = cfg.direction * (result_.x_target - x_start(0));
    dwell_time_ = cfg.dwell_time;
    timeout_ = cfg.acquisition_timeout;
    onset_speed_ = cfg.onset_speed;
    direction_ = cfg.direction;
    in_target_ = false;
    entry_time_ = kNaN;
    t_prev_ = kNaN;
    dt_sum_ = 0.;
    sat_samples_ = 0;
}

bool TrialMonitor::update(double t, const Vec2 &x, const Vec2 &v, bool saturated) {
    if (finished()) return true;
    TrialResult &r = result_;

    ++r.samples;
    if (std::isfinite(t_prev_)) {
        const double dt = t - t_prev_;
        dt_sum_ += dt;
        r.dt_max = std::isfinite(r.dt_max) ? std::max(r.dt_max, dt) : dt;
        r.dt_mean = dt_sum_ / static_cast<double>(r.samples - 1);
    }
    t_prev_ = t;
    if (saturated) ++sat_samples_;
    r.sat_fraction = static_cast<double>(sat_samples_) / static_cast<double>(r.samples);

    const double err = direction_ * (x(0) - r.x_target);  // > 0 beyond the target, < 0 short of it
    const bool in = std::abs(err) <= r.tolerance;
    const double speed_x = std::abs(v(0));

    if (!std::isfinite(r.rt_kinematic) && speed_x >= onset_speed_) r.rt_kinematic = t;
    if (speed_x > r.peak_speed_x) {
        r.peak_speed_x = speed_x;
        r.t_peak_speed_x = t;
    }
    r.overshoot = std::max(r.overshoot, err);
    r.y_max_dev = std::max(r.y_max_dev, std::abs(x(1) - r.y_home));

    if (in && !in_target_) {  // entry into the band
        ++r.n_entries;
        if (!std::isfinite(r.mt_first_entry)) r.mt_first_entry = t;
        entry_time_ = t;
    }
    in_target_ = in;

    if (in && (t - entry_time_) >= dwell_time_) {
        r.outcome = TrialOutcome::Success;
        r.mt_final_entry = entry_time_;
        r.task_time = t;
        r.endpoint_error = err;
    } else if (!in && t >= timeout_) {
        r.outcome = TrialOutcome::Timeout;
        r.task_time = t;
        r.endpoint_error = err;
        r.note = "not in target at acquisition_timeout";
    }
    if (finished()) finalise();
    return finished();
}

void TrialMonitor::abort(double t, const std::string &reason) {
    if (finished()) return;
    result_.outcome = TrialOutcome::Aborted;
    result_.task_time = t;
    result_.note = reason;
    finalise();
}

void TrialMonitor::finalise() {
    TrialResult &r = result_;
    if (std::isfinite(r.rt_kinematic)) {
        if (std::isfinite(r.mt_first_entry)) r.mt_first_entry_kin = r.mt_first_entry - r.rt_kinematic;
        if (std::isfinite(r.mt_final_entry)) r.mt_final_entry_kin = r.mt_final_entry - r.rt_kinematic;
    }
    in_target_ = false;
}

// ------------------------------------------------------------------------------------ Results writer
std::string ResultsWriter::header() {
    return "trial,csv_line,D_file,W_file,ID_fitts_bits,ID_shannon_bits,x_home_m,x_target_m,tolerance_m,"
           "x_start_m,D_effective_m,outcome,success,rt_kinematic_s,mt_first_entry_s,mt_final_entry_s,"
           "mt_first_entry_kin_s,mt_final_entry_kin_s,task_time_s,n_entries,peak_speed_x_mps,t_peak_speed_x_s,"
           "overshoot_m,endpoint_error_m,y_max_dev_m,sat_fraction,dt_mean_ms,dt_max_ms,samples,wall_clock,note";
}

bool ResultsWriter::open(const std::string &path) {
    close();
    path_ = path;
    f_.open(path, std::ios::out | std::ios::trunc);
    if (!f_) return false;
    f_ << header() << "\n";
    f_.flush();
    return f_.good();
}

bool ResultsWriter::write(const TrialResult &r) {
    if (!f_.is_open()) return false;
    const double D = r.trial.D, W = r.trial.W;
    f_ << r.trial.index << ',' << r.trial.csv_line << ',' << num(D) << ',' << num(W) << ','
       << num(std::log2(2. * D / W)) << ',' << num(std::log2(D / W + 1.)) << ',' << num(r.x_home) << ','
       << num(r.x_target) << ',' << num(r.tolerance) << ',' << num(r.x_start) << ',' << num(r.D_effective) << ','
       << outcomeName(r.outcome) << ',' << (r.outcome == TrialOutcome::Success ? 1 : 0) << ','
       << num(r.rt_kinematic) << ',' << num(r.mt_first_entry) << ',' << num(r.mt_final_entry) << ','
       << num(r.mt_first_entry_kin) << ',' << num(r.mt_final_entry_kin) << ',' << num(r.task_time) << ','
       << r.n_entries << ',' << num(r.peak_speed_x) << ',' << num(r.t_peak_speed_x) << ',' << num(r.overshoot) << ','
       << num(r.endpoint_error) << ',' << num(r.y_max_dev) << ',' << num(r.sat_fraction, 4) << ','
       << num(r.dt_mean * 1e3, 3) << ',' << num(r.dt_max * 1e3, 3) << ',' << r.samples << ',' << sanitise(r.wall_clock)
       << ',' << sanitise(r.note) << "\n";
    f_.flush();  // one row per trial: nothing is lost if the app is interrupted
    return f_.good();
}

void ResultsWriter::close() {
    if (f_.is_open()) f_.close();
}

}  // namespace fitts