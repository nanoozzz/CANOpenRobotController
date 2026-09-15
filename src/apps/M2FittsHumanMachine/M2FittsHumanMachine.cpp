#include "M2FittsHumanMachine.h"

#include <algorithm>
#include <cmath>
#include <csignal>
#include <ctime>
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <sstream>

/******************************************************************************
 * Default warm-up table
 *
 * Protocol: "2 rounds x 5 ID = 10 reps (each ID is shown once using the exact same
 * condition)". NOTE: this built-in table is a fallback only. It realises the five IDs with
 * *different* (A,W) pairs than schedule/warmup.csv shipped with the Unity project, and warm-up
 * amplitude exposure is not neutral with respect to Block 1. Always provide warmup.csv in the
 * trials directory so that a single table is used, and check the log line printed by loadWarmup().
 ******************************************************************************/
static const double kDefaultWarmup[5][3] = {
    // A (cm),  W (cm),     ID (bits)
    {12.0, 2.000000, 2.807},
    {12.0, 1.333333, 3.322},
    {18.0, 1.333333, 3.858},
    {20.25, 1.000000, 4.409},
    {30.375, 1.000000, 4.972},
};

//! Config file locations tried in order (the first one that opens wins).
static const char *kConfigCandidates[] = {
    "config/M2FittsHuman.conf",
    "config/M2FittsHumanMachine.cfg",
    "../config/M2FittsHuman.conf",
    "../config/M2FittsHumanMachine.cfg",
};

/******************************************************************************
 * Small text helpers (csv / config parsing)
 ******************************************************************************/
static std::string trimStr(const std::string &s) {
    const std::string ws = " \t\r\n";
    size_t b = s.find_first_not_of(ws);
    if (b == std::string::npos) return "";
    size_t e = s.find_last_not_of(ws);
    return s.substr(b, e - b + 1);
}

static std::vector<std::string> splitStr(const std::string &s, char sep) {
    std::vector<std::string> out;
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, sep)) out.push_back(trimStr(item));
    return out;
}

//! Lower-case and keep only alphanumeric characters (to match csv header names robustly)
static std::string normaliseName(const std::string &s) {
    std::string out;
    for (char c : s)
        if (std::isalnum(static_cast<unsigned char>(c))) out += std::tolower(static_cast<unsigned char>(c));
    return out;
}

static bool toDouble(const std::string &s, double &v) {
    if (s.empty()) return false;
    try {
        size_t used = 0;
        v = std::stod(s, &used);
        return used == s.size();
    } catch (...) {
        return false;
    }
}

//! Fixed precision number, with empty measures written as NaN (read as NA by R/pandas)
static std::string num(double v, int prec = 3) {
    if (std::isnan(v)) return "NaN";
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.*f", prec, v);
    return std::string(buf);
}

/******************************************************************************
 * Transition conditions
 ******************************************************************************/
static bool calibrationDone(StateMachine &SM) {
    return SM.state<M2FittsCalibState>("CalibState")->isCalibDone();
}

static bool uiAvailable(StateMachine &SM) {
    return SM.state<M2FittsWaitUIState>("WaitUIState")->isUIReady();
}

static bool readyToStartTrial(StateMachine &SM) {
    M2FittsHumanMachine &sm = static_cast<M2FittsHumanMachine &>(SM);
    return sm.state<M2FittsReadyState>("ReadyState")->isReady() && !sm.sessionFinished();
}

static bool trialOver(StateMachine &SM) {
    return SM.state<M2FittsReachState>("ReachState")->isTrialDone();
}

static bool returnedAndSessionOver(StateMachine &SM) {
    M2FittsHumanMachine &sm = static_cast<M2FittsHumanMachine &>(SM);
    return sm.state<M2FittsReturnState>("ReturnState")->isReturnDone() && sm.sessionFinished();
}

static bool returnedAndBreakDue(StateMachine &SM) {
    M2FittsHumanMachine &sm = static_cast<M2FittsHumanMachine &>(SM);
    return sm.state<M2FittsReturnState>("ReturnState")->isReturnDone() && sm.breakDue();
}

static bool returned(StateMachine &SM) {
    return SM.state<M2FittsReturnState>("ReturnState")->isReturnDone();
}

static bool breakOver(StateMachine &SM) {
    return SM.state<M2FittsBreakState>("BreakState")->isBreakOver();
}

static bool abortRequested(StateMachine &SM) {
    return static_cast<M2FittsHumanMachine &>(SM).abortSignal();
}

/******************************************************************************
 * Construction
 ******************************************************************************/
M2FittsHumanMachine::M2FittsHumanMachine() {
    //Optional run-time configuration (participant, paths, geometry, timings...)
    loadConfig();
    applyConfig();

    //Create an M2 Robot and set it to generic state machine
    setRobot(std::make_unique<RobotM2>(cfgStr("robot_name", "M2_MELB")));

    //Create state instances and add to the State Machine
    addState("CalibState", std::make_shared<M2FittsCalibState>(robot(), this));
    addState("WaitUIState", std::make_shared<M2FittsWaitUIState>(robot(), this));
    addState("StandbyState", std::make_shared<M2FittsStandbyState>(robot(), this));
    addState("ReadyState", std::make_shared<M2FittsReadyState>(robot(), this));
    addState("ReachState", std::make_shared<M2FittsReachState>(robot(), this));
    addState("ReturnState", std::make_shared<M2FittsReturnState>(robot(), this));
    addState("BreakState", std::make_shared<M2FittsBreakState>(robot(), this));
    addState("EndState", std::make_shared<M2FittsEndState>(robot(), this));

    //Define transitions between states. Order matters: the first active transition of a state is used.
    //Calibration -> wait for the Unity client -> protocol. The wait state is transparent and is
    //skipped immediately when ui_required = false (mouse-free bench testing without the UI).
    addTransition("CalibState", &calibrationDone, "WaitUIState");
    addTransition("WaitUIState", &uiAvailable, "ReadyState");
    addTransition("ReadyState", &readyToStartTrial, "ReachState");
    addTransition("ReachState", &trialOver, "ReturnState");
    addTransition("ReturnState", &returnedAndSessionOver, "EndState");
    addTransition("ReturnState", &returnedAndBreakDue, "BreakState");
    addTransition("ReturnState", &returned, "ReachState");
    addTransition("BreakState", &breakOver, "ReadyState");
    //Manual abort (keyboard 'x' or UI ABRT) from any state: registered last so that it never
    //pre-empts a normal transition
    addTransitionFromAny(&abortRequested, "StandbyState");

    setInitState("CalibState");
}

M2FittsHumanMachine::~M2FittsHumanMachine() {
    if (resultsFile_.is_open()) resultsFile_.close();
}

/******************************************************************************
 * Configuration
 ******************************************************************************/
bool M2FittsHumanMachine::loadConfigFile(const std::string &file) {
    std::ifstream f(file);
    if (!f.is_open()) return false;

    std::string line;
    while (std::getline(f, line)) {
        line = trimStr(line);
        if (line.empty() || line[0] == '#' || line[0] == ';') continue;
        size_t eq = line.find('=');
        if (eq == std::string::npos) continue;
        //Strip any trailing in-line comment from the value
        std::string value = trimStr(line.substr(eq + 1));
        size_t hash = value.find('#');
        if (hash != std::string::npos) value = trimStr(value.substr(0, hash));
        cfg_[normaliseName(line.substr(0, eq))] = value;
    }
    spdlog::info("M2FittsHuman: loaded {} parameters from {}.", cfg_.size(), file);
    return true;
}

bool M2FittsHumanMachine::loadConfig() {
    for (const char *candidate : kConfigCandidates) {
        if (loadConfigFile(candidate)) return true;
    }
    spdlog::warn("M2FittsHuman: no config file found (tried config/M2FittsHuman.conf and "
                 "config/M2FittsHumanMachine.cfg, also one level up). Using built-in defaults.");
    return false;
}

std::string M2FittsHumanMachine::cfgStr(const std::string &key, const std::string &def) const {
    auto it = cfg_.find(normaliseName(key));
    return (it == cfg_.end()) ? def : it->second;
}
double M2FittsHumanMachine::cfgDbl(const std::string &key, double def) const {
    double v;
    return toDouble(cfgStr(key, ""), v) ? v : def;
}
int M2FittsHumanMachine::cfgInt(const std::string &key, int def) const {
    return static_cast<int>(std::lround(cfgDbl(key, def)));
}
bool M2FittsHumanMachine::cfgBool(const std::string &key, bool def) const {
    std::string v = normaliseName(cfgStr(key, ""));
    if (v == "true" || v == "1" || v == "yes" || v == "on") return true;
    if (v == "false" || v == "0" || v == "no" || v == "off") return false;
    return def;
}

void M2FittsHumanMachine::applyConfig() {
    participant_ = cfgStr("participant", participant_);
    block_ = cfgInt("block", block_);
    trialsDir_ = cfgStr("trials_dir", trialsDir_);
    logDir_ = cfgStr("results_dir", logDir_);
    serverIP_ = cfgStr("ip", serverIP_);
    serverPort_ = cfgInt("port", serverPort_);

    uiRequired_ = cfgBool("ui_required", uiRequired_);
    uiDivider_ = std::max(1, cfgInt("ui_stream_divider", uiDivider_));
    displayGain_ = cfgDbl("display_gain", displayGain_);

    params_.originX = cfgDbl("origin_x", params_.originX);
    params_.originY = cfgDbl("origin_y", params_.originY);
    params_.taskDirection = (cfgDbl("task_direction", params_.taskDirection) < 0) ? -1. : 1.;
    params_.useYChannel = cfgBool("use_y_channel", params_.useYChannel);
    params_.channelK = cfgDbl("channel_k", params_.channelK);
    params_.channelD = cfgDbl("channel_d", params_.channelD);

    params_.dwellTime = cfgDbl("dwell_time", params_.dwellTime);
    params_.homeExitRadius = cfgDbl("home_exit_radius", params_.homeExitRadius);
    params_.maxTrialTime = cfgDbl("max_trial_time", params_.maxTrialTime);

    params_.returnOffset = cfgDbl("return_offset", params_.returnOffset);
    params_.returnSpeed = cfgDbl("return_speed", params_.returnSpeed);
    params_.returnMinTime = cfgDbl("return_min_time", params_.returnMinTime);
    params_.originTolerance = cfgDbl("origin_tolerance", params_.originTolerance);
    params_.maxReturnTime = cfgDbl("max_return_time", params_.maxReturnTime);
    params_.originSnapTime = cfgDbl("origin_snap_time", params_.originSnapTime);
    params_.originHoldTime = cfgDbl("origin_hold_time", params_.originHoldTime);

    params_.roundBreakTime = cfgDbl("round_break_time", params_.roundBreakTime);
    params_.warmupRestTime = cfgDbl("warmup_rest_time", params_.warmupRestTime);
    params_.readyHoldTime = cfgDbl("ready_hold_time", params_.readyHoldTime);

    params_.kPosVel = cfgDbl("k_pos_vel", params_.kPosVel);
    params_.forceLimit = cfgDbl("force_limit", params_.forceLimit);

    params_.nRounds = cfgInt("n_rounds", params_.nRounds);
    params_.trialsPerRound = cfgInt("trials_per_round", params_.trialsPerRound);
    params_.warmupRepeats = cfgInt("warmup_repeats", params_.warmupRepeats);

    if (std::fabs(displayGain_ - 1.0) > 1e-9)
        spdlog::warn("M2FittsHuman: display_gain = {} (non-unity). The visual amplitude no longer "
                     "equals the movement amplitude, so the *visual* index of difficulty differs "
                     "from the mechanical one. Only set this if a gain manipulation is intended.",
                     displayGain_);

    //Session tag: <participant>_B<block>_<yyyymmdd-HHMMSS>, used for both output files
    std::time_t t = std::time(nullptr);
    char stamp[32];
    std::strftime(stamp, sizeof(stamp), "%Y%m%d-%H%M%S", std::localtime(&t));
    sessionTag_ = participant_ + "_B" + std::to_string(block_) + "_" + stamp;
}

/******************************************************************************
 * Trial table
 ******************************************************************************/
bool M2FittsHumanMachine::readTrialCsv(const std::string &file, std::vector<FittsTrial> &trials) const {
    std::ifstream f(file);
    if (!f.is_open()) {
        spdlog::critical("M2FittsHuman: cannot open trial file {}.", file);
        return false;
    }

    int colIndex = -1, colA = -1, colW = -1, colID = -1;
    bool columnsKnown = false;
    std::string line;
    int lineNb = 0;

    while (std::getline(f, line)) {
        lineNb++;
        line = trimStr(line);
        if (line.empty()) continue;
        std::vector<std::string> fields = splitStr(line, ',');
        if (fields.size() < 3) {
            spdlog::warn("M2FittsHuman: {} line {} ignored (less than 3 columns).", file, lineNb);
            continue;
        }

        if (!columnsKnown) {
            //Header line? (i.e. at least one non numerical field)
            double dummy;
            bool isHeader = false;
            for (const auto &s : fields)
                if (!toDouble(s, dummy)) isHeader = true;

            if (isHeader) {
                for (size_t i = 0; i < fields.size(); i++) {
                    std::string n = normaliseName(fields[i]);
                    if (n == "index" || n == "trial" || n == "trialnb" || n == "no") colIndex = i;
                    else if (n == "a" || n == "acm" || n == "amplitude") colA = i;
                    else if (n == "w" || n == "wcm" || n == "width") colW = i;
                    else if (n == "id" || n == "idbits") colID = i;
                }
                if (colA < 0 || colW < 0) {
                    spdlog::critical("M2FittsHuman: {} header does not contain A and W columns.", file);
                    return false;
                }
                columnsKnown = true;
                continue;  //header consumed
            } else {
                //No header: assume the documented order index,A,W,ID
                colIndex = 0; colA = 1; colW = 2; colID = 3;
                columnsKnown = true;
            }
        }

        FittsTrial t;
        double v;
        if (colIndex >= 0 && colIndex < (int)fields.size() && toDouble(fields[colIndex], v)) t.index = (int)std::lround(v);
        else t.index = (int)trials.size() + 1;

        if (colA >= (int)fields.size() || !toDouble(fields[colA], t.A_cm)) {
            spdlog::critical("M2FittsHuman: {} line {}: invalid A.", file, lineNb);
            return false;
        }
        if (colW >= (int)fields.size() || !toDouble(fields[colW], t.W_cm) || t.W_cm <= 0) {
            spdlog::critical("M2FittsHuman: {} line {}: invalid W.", file, lineNb);
            return false;
        }
        if (colID >= 0 && colID < (int)fields.size() && toDouble(fields[colID], v)) t.ID_bits = v;
        else t.ID_bits = std::log2(t.A_cm / t.W_cm + 1.);  //Shannon formulation, as used in the design

        trials.push_back(t);
    }

    if (trials.empty()) {
        spdlog::critical("M2FittsHuman: no trial read from {}.", file);
        return false;
    }
    return true;
}

bool M2FittsHumanMachine::loadTrialTable() {
    blockTrials_.clear();
    for (int g = 1; g <= params_.nRounds; g++) {
        std::vector<FittsTrial> round;
        std::string file = trialsDir_ + "/bal_group_" + std::to_string(g) + ".csv";
        if (!readTrialCsv(file, round)) return false;
        if ((int)round.size() != params_.trialsPerRound)
            spdlog::warn("M2FittsHuman: {} holds {} trials ({} expected).", file, round.size(), params_.trialsPerRound);
        for (size_t i = 0; i < round.size(); i++) {
            round[i].round = g;
            round[i].inRound = (int)i + 1;
            blockTrials_.push_back(round[i]);
        }
        spdlog::info("M2FittsHuman: round {} loaded ({} trials) from {}.", g, round.size(), file);
    }
    return !blockTrials_.empty();
}

bool M2FittsHumanMachine::loadWarmup() {
    warmupTrials_.clear();
    std::vector<FittsTrial> conditions;
    std::string file = trialsDir_ + "/warmup.csv";
    std::ifstream test(file);
    if (test.is_open()) {
        test.close();
        if (!readTrialCsv(file, conditions)) return false;
        spdlog::info("M2FittsHuman: warm-up conditions loaded from {}.", file);
    } else {
        for (const auto &c : kDefaultWarmup) {
            FittsTrial t;
            t.A_cm = c[0];
            t.W_cm = c[1];
            t.ID_bits = c[2];
            conditions.push_back(t);
        }
        spdlog::warn("M2FittsHuman: {} not found - falling back on the BUILT-IN warm-up table "
                     "({} conditions). This table does not match the one shipped with the Unity "
                     "project; provide warmup.csv so that a single table is used.",
                     file, conditions.size());
    }

    //warmupRepeats rounds, each presenting every warm-up condition once (in increasing ID order)
    std::sort(conditions.begin(), conditions.end(),
              [](const FittsTrial &a, const FittsTrial &b) { return a.ID_bits < b.ID_bits; });
    int n = 1;
    for (int r = 1; r <= params_.warmupRepeats; r++) {
        for (size_t i = 0; i < conditions.size(); i++) {
            FittsTrial t = conditions[i];
            t.round = r;
            t.inRound = (int)i + 1;
            t.index = n++;
            warmupTrials_.push_back(t);
        }
    }
    return !warmupTrials_.empty();
}

/**
 * \brief Sanity checks on the loaded table.
 * \return false only for a fatal problem (target outside the reachable workspace), which
 *         must stop the session; ID formulation and balance issues are warnings.
 */
bool M2FittsHumanMachine::checkTrialTable() {
    bool ok = true;  //fatal problems only
    //1. Index of difficulty consistency: ID = log2(A/W + 1) (Shannon formulation)
    int nIDMismatch = 0;
    for (const auto &t : blockTrials_) {
        double expected = std::log2(t.A_cm / t.W_cm + 1.);
        if (std::fabs(expected - t.ID_bits) > 1e-3) nIDMismatch++;
    }
    if (nIDMismatch > 0)  //warning only: another ID formulation may legitimately be in use
        spdlog::warn("M2FittsHuman: {} trials whose ID column differs from log2(A/W+1) by more than 1e-3.", nIDMismatch);

    //2. Workspace: the far edge of the widest/farthest target and the off-origin return point must be reachable
    double maxA = 0., maxHalfW = 0.;
    for (const auto &t : blockTrials_) {
        maxA = std::max(maxA, t.A());
        maxHalfW = std::max(maxHalfW, t.halfW());
    }
    double xFar = params_.originX + params_.taskDirection * (maxA + maxHalfW);
    double xNear = params_.originX;
    const double xMin = 0.02, xMax = 0.605;  //M2 x travel is [0, 0.625]: keep 2 cm clear of the stops
    if (xFar > xMax || xFar < xMin || xNear > xMax || xNear < xMin) {
        spdlog::critical("M2FittsHuman: workspace violation - origin {:.3f} m, farthest target edge {:.3f} m (allowed [{:.2f}, {:.2f}] m). Adjust origin_x.",
                         xNear, xFar, xMin, xMax);
        ok = false;
    }
    //The off-origin return point must also be reachable
    double xAway = params_.originX + params_.taskDirection * params_.returnOffset;
    if (xAway > xMax || xAway < xMin) {
        spdlog::critical("M2FittsHuman: return position {:.3f} m is outside the usable travel.", xAway);
        ok = false;
    }
    if (params_.originY < 0.02 || params_.originY > 0.42)
        spdlog::warn("M2FittsHuman: origin_y = {:.3f} m is close to the y stops ([0, 0.440] m).", params_.originY);

    //3. Design balance (informative): occurrences of each (A,W) condition
    std::map<std::string, int> counts;
    for (const auto &t : blockTrials_) {
        char key[64];
        std::snprintf(key, sizeof(key), "A=%.4f/W=%.4f", t.A_cm, t.W_cm);
        counts[key]++;
    }
    spdlog::info("M2FittsHuman: {} block trials over {} conditions ({} repetitions each if balanced).",
                 blockTrials_.size(), counts.size(), counts.empty() ? 0 : (int)(blockTrials_.size() / counts.size()));
    for (const auto &c : counts)
        if (counts.begin()->second != c.second)
            spdlog::warn("M2FittsHuman: unbalanced design - {} appears {} times.", c.first, c.second);

    return ok;
}

/******************************************************************************
 * Results file
 ******************************************************************************/
bool M2FittsHumanMachine::openResultsFile() {
    std::string file = logDir_ + "/M2FittsHuman_" + sessionTag_ + "_trials.csv";
    resultsFile_.open(file, std::ios::out | std::ios::trunc);
    if (!resultsFile_.is_open()) {
        spdlog::critical("M2FittsHuman: cannot open results file {} (does the {} folder exist?).", file, logDir_);
        return false;
    }
    resultsFile_ << "participant,block,phase,round,trial_in_round,trial_index,"
                    "A_cm,W_cm,ID_bits,"
                    "MT_s,RT_s,MT_move_s,t_first_entry_s,n_entries,"
                    "x_entry_cm,x_sel_cm,v_peak_ms,success,"
                    "return_time_s,return_timeout,return_abort,"
                    "dwell_s,t_onset_s,t_end_s,ui_connected\n";
    resultsFile_.flush();
    spdlog::info("M2FittsHuman: trial results -> {}", file);
    return true;
}

void M2FittsHumanMachine::writeResultRow(const FittsTrialResult &r) {
    if (!resultsFile_.is_open()) return;
    resultsFile_ << participant_ << "," << block_ << ","
                 << (r.phase == PHASE_WARMUP ? "warmup" : "block") << ","
                 << r.trial.round << "," << r.trial.inRound << "," << r.trial.index << ","
                 << num(r.trial.A_cm, 4) << "," << num(r.trial.W_cm, 6) << "," << num(r.trial.ID_bits, 4) << ","
                 << num(r.MT, 4) << "," << num(r.RT, 4) << "," << num(r.MT_move, 4) << ","
                 << num(r.t_first_entry, 4) << "," << r.nEntries << ","
                 << num(r.x_entry_cm, 4) << "," << num(r.x_sel_cm, 4) << "," << num(r.v_peak, 4) << ","
                 << (r.success ? 1 : 0) << ","
                 << num(r.returnTime, 4) << "," << (r.returnTimeout ? 1 : 0) << "," << (r.returnAbort ? 1 : 0) << ","
                 << num(params_.dwellTime, 3) << "," << num(r.t_onset, 4) << "," << num(r.t_end, 4) << ","
                 << (ui.connected() ? 1 : 0) << "\n";
    resultsFile_.flush();  //written trial by trial: an interrupted session keeps all completed trials
}

/******************************************************************************
 * Protocol sequencing
 ******************************************************************************/
void M2FittsHumanMachine::setCurrentTrial() {
    const std::vector<FittsTrial> &list = (phase_ == PHASE_WARMUP) ? warmupTrials_ : blockTrials_;
    if (phase_ != PHASE_DONE && trialIdx_ < list.size()) currentTrial_ = list[trialIdx_];
}

void M2FittsHumanMachine::recordReach(const FittsTrialResult &r) {
    pending_ = r;
}

void M2FittsHumanMachine::finaliseTrial(double returnTime, bool timedOut, bool aborted) {
    pending_.returnTime = returnTime;
    pending_.returnTimeout = timedOut;
    pending_.returnAbort = aborted;
    writeResultRow(pending_);
    results_.push_back(pending_);
    trialsDone_++;

    //Advance to the next trial and schedule the breaks
    trialIdx_++;
    if (phase_ == PHASE_WARMUP) {
        if (trialIdx_ >= warmupTrials_.size()) {
            phase_ = PHASE_BLOCK;
            trialIdx_ = 0;
            breakDue_ = true;
            breakDuration_ = params_.warmupRestTime;
            spdlog::info("M2FittsHuman: warm-up completed ({} reps). Rest, then Block {}.", warmupTrials_.size(), block_);
        }
    } else if (phase_ == PHASE_BLOCK) {
        if (trialIdx_ >= blockTrials_.size()) {
            phase_ = PHASE_DONE;
            spdlog::info("M2FittsHuman: Block {} completed ({} trials).", block_, blockTrials_.size());
        } else if (blockTrials_[trialIdx_].round != blockTrials_[trialIdx_ - 1].round) {
            breakDue_ = true;
            breakDuration_ = params_.roundBreakTime;
            spdlog::info("M2FittsHuman: round {} completed. Break, then round {}.",
                         blockTrials_[trialIdx_ - 1].round, blockTrials_[trialIdx_].round);
        }
    }
    setCurrentTrial();
}

void M2FittsHumanMachine::printSummary() {
    if (results_.empty()) return;
    std::map<double, std::pair<int, double>> perID;  //ID -> (n, sum MT) over successful block trials
    int nSuccess = 0, nBlock = 0;
    for (const auto &r : results_) {
        if (r.phase != PHASE_BLOCK) continue;
        nBlock++;
        if (!r.success || std::isnan(r.MT)) continue;
        nSuccess++;
        perID[r.trial.ID_bits].first++;
        perID[r.trial.ID_bits].second += r.MT;
    }
    std::cout << "\n----------------- Block " << block_ << " summary (" << participant_ << ") -----------------\n"
              << nSuccess << "/" << nBlock << " trials validated\n"
              << "   ID (bits)      n     mean MT (s)\n";
    for (const auto &e : perID)
        std::cout << "   " << std::fixed << std::setprecision(4) << std::setw(8) << e.first
                  << std::setw(8) << e.second.first
                  << std::setw(14) << std::setprecision(3) << (e.second.second / e.second.first) << "\n";
    std::cout << "(MT excludes the " << params_.dwellTime << " s validation dwell)\n"
              << "-------------------------------------------------------------\n" << std::endl;
}

/******************************************************************************
 * UI and inputs
 ******************************************************************************/
std::vector<double> M2FittsHumanMachine::sessionDescriptor() const {
    //Field order is part of the wire protocol: see M2FittsHumanMachine.h and FittsProtocol.cs.
    return {(double)M2FITTS_PROTOCOL_VERSION,
            (double)block_,
            (double)params_.nRounds,
            (double)params_.trialsPerRound,
            (double)warmupTrials_.size(),
            params_.originX,
            params_.originY,
            params_.taskDirection,
            params_.dwellTime,
            params_.maxTrialTime,
            params_.originTolerance,
            params_.returnOffset,
            params_.useYChannel ? 1. : 0.,
            displayGain_};
}

void M2FittsHumanMachine::sendUI(const std::string &cmd, const std::vector<double> &params) {
    ui.send(cmd, params);
}

void M2FittsHumanMachine::sendUIContext(const std::string &cmd, const std::vector<double> &params) {
    ui.sendContext(cmd, params);
}

bool M2FittsHumanMachine::goSignal() {
    if (robot()->keyboard->getS() || robot()->keyboard->getNb() == 1) return true;
    if (robot()->joystick->isButtonPressed(1)) return true;
    return ui.consumeGo();
}

bool M2FittsHumanMachine::abortSignal() {
    return robot()->keyboard->getX() || ui.consumeAbort();
}

/******************************************************************************
 * CORC interface
 ******************************************************************************/
void M2FittsHumanMachine::init() {
    spdlog::debug("M2FittsHumanMachine::init()");

    if (!loadWarmup() || !loadTrialTable()) {
        spdlog::critical("M2FittsHuman: trial table could not be loaded from {}. Exiting...", trialsDir_);
        std::raise(SIGTERM);
        return;
    }
    if (!checkTrialTable()) {  //fatal (unreachable targets): stop before the participant is set up
        spdlog::critical("M2FittsHuman: trial table incompatible with the workspace. Exiting...");
        std::raise(SIGTERM);
        return;
    }
    if (!openResultsFile()) {
        std::raise(SIGTERM);
        return;
    }
    phase_ = PHASE_WARMUP;
    trialIdx_ = 0;
    setCurrentTrial();

    spdlog::info("M2FittsHuman: participant {}, block {}: {} warm-up reps then {} trials in {} rounds.",
                 participant_, block_, warmupTrials_.size(), blockTrials_.size(), params_.nRounds);

    if (robot()->initialise()) {
        logHelper.initLogger("M2FittsHumanLog", logDir_ + "/M2FittsHuman_" + sessionTag_ + "_raw.csv", LogFormat::CSV, true);
        logHelper.add(runningTime(), "Time (s)");
        logHelper.add(robot()->getEndEffPosition(), "Position (m)");
        logHelper.add(robot()->getEndEffVelocity(), "Velocity (m/s)");
        logHelper.add(robot()->getEndEffForce(), "Force (N)");
        logHelper.add(robot()->getInteractionForce(), "InteractionForce (N)");
        logHelper.add(logState_, "StateCode");
        logHelper.add(logPhase_, "Phase");
        logHelper.add(logTrialNb_, "Trial");
        logHelper.add(logTargetX_, "TargetX (m)");
        logHelper.add(logHalfWidth_, "TargetHalfWidth (m)");
        logHelper.add(logDwell_, "DwellProgress");
        //Display-side markers, mirrored from the link in hwStateUpdate() so that UI events can be
        //related to the kinematics offline. They must be non-const lvalues: see the note in the header.
        logHelper.add(logMarkCode_, "UIMarkCode");
        logHelper.add(logMarkTime_, "UIMarkClientTime (s)");
        logHelper.add(logMarkServerTime_, "UIMarkServerTime (s)");
        logHelper.startLogger();

        UIserver = std::make_shared<FLNLHelper>(*robot(), "0.0.0.0");

        //---------------------------------------------------------------- UI link
        //IMPORTANT: the state vector is registered explicitly here rather than through the
        //FLNLHelper(RobotM2&,...) convenience constructor, because that constructor streams the
        //helper's OWN clock (started at helper construction, i.e. after robot initialisation).
        //That clock is offset by an unknown few hundred ms from the state machine clock used for
        //t_onset/t_end in the results csv and for the Time column of the raw log, which makes
        //offline alignment of display frames and kinematics impossible. Streaming runningTime()
        //puts every record on one clock.
        ui.setClock(&runningTime());
        //ui.init(serverIP_, serverPort_, uiDivider_, uiRequired_);
        //ui.registerState(runningTime());                    // [0]      t          (s)
        //ui.registerState(robot()->getEndEffPosition());     // [1,2]    x, y       (m)
        //ui.registerState(robot()->getEndEffVelocity());     // [3,4]    dx, dy     (m/s)
        //ui.registerState(robot()->getInteractionForce());   // [5,6]    Fx, Fy     (N)
        //ui.registerState(logState_);                        // [7]      state code
        //ui.registerState(logPhase_);                        // [8]      phase
        //ui.registerState(logTrialNb_);                      // [9]      trial index
        //ui.registerState(logTargetX_);                      // [10]     target x   (m)
        //ui.registerState(logHalfWidth_);                    // [11]     half width (m)
        //ui.registerState(logDwell_);                        // [12]     dwell progress 0..1
        if (!ui.init(
            UIserver,
            uiDivider_,
            uiRequired_)) {

            spdlog::critical(
            "M2FittsHuman/UI: failed to initialise UI link.");

            std::raise(SIGTERM);
            return;
        }
        ui.setSession(sessionDescriptor());
        //spdlog::info("Registered 13 UI state values");
        spdlog::info(
            "M2FittsHuman/UI: using existing UIserver.");

        spdlog::info(
            "M2FittsHuman/UI: expected state vector = 13 values.");
    } else {
        spdlog::critical("Failed robot initialisation. Exiting...");
        std::raise(SIGTERM);  //Clean exit
    }
}

void M2FittsHumanMachine::end() {
    if (resultsFile_.is_open()) {
        resultsFile_.flush();
        resultsFile_.close();
    }
    printSummary();
    if (running()) ui.close();
    StateMachine::end();
}

void M2FittsHumanMachine::hwStateUpdate(void) {
    StateMachine::hwStateUpdate();
    //Values streamed in the continuous log and to the UI (updated here so that every logged
    //sample and every streamed frame is tagged consistently)
    logPhase_ = (double)phase_;
    logTrialNb_ = (double)currentTrial_.index;
    //Mirror the latest display marker so that it is captured by the logger (which holds plain
    //double pointers, hence the copy rather than a reference into the link)
    logMarkCode_ = ui.markCode();
    logMarkTime_ = ui.markTime();
    logMarkServerTime_ = ui.markServerTime();

    auto now = std::chrono::steady_clock::now();
    static auto lastCheck = std::chrono::steady_clock::now();
    static bool connected = false;

    if (UIserver && std::chrono::duration<double, std::milli>(now - lastCheck).count() > 1000.0) {
        connected = UIserver->isConnected();
        if (!connected) {
            spdlog::warn("UI disconnected. Waiting for Unity...");
            UIserver->reconnect(); 
            connected = UIserver->isConnected();
        }
        lastCheck = now;
    }

    StateMachine::hwStateUpdate();

    static auto lastSend = std::chrono::steady_clock::now();
    if (connected && std::chrono::duration<double, std::milli>(now - lastSend).count() >= 25.0) {
        UIserver->sendState();
        lastSend = now;
    }
    //Connection monitoring, inbound commands, context replay and (decimated) state stream
    ui.update();
}