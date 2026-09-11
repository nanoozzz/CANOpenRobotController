/**
 * \file M2FittsMachine.cpp
 * \brief Implementation of M2FittsMachine (see M2FittsMachine.h).
 *
 * Provenance: drafted with AI assistance (Claude, Anthropic), September 2026. Review before use.
 * Licence: Apache-2.0 (same as CORC).
 */
#include "M2FittsMachine.h"

#include <sys/stat.h>

#include <algorithm>
#include <cerrno>
#include <csignal>
#include <cstring>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace {

M2FittsMachine &fittsSM(StateMachine &sm) { return static_cast<M2FittsMachine &>(sm); }

// ------------------------------------------------------------------------------ Transition conditions
bool faultRaised(StateMachine &sm) { return fittsSM(sm).session()->fault; }

bool calibDone(StateMachine &sm) { return sm.state<M2FittsCalibState>("CalibState")->isDone(); }

bool homingFinished(StateMachine &sm) {
    const auto st = sm.state<M2FittsHomingState>("GoHomeState");
    return st->isSettled() || st->timedOut();
}

bool noTrialsLeft(StateMachine &sm) {
    const auto s = fittsSM(sm).session();
    return s->next_trial >= s->trials.size();
}

bool startRequested(StateMachine &sm) { return fittsSM(sm).robot()->keyboard->getS(); }

bool trialFinished(StateMachine &sm) { return sm.state<M2FittsReachState>("ReachState")->isFinished(); }

bool returnTimedOut(StateMachine &sm) { return sm.state<M2FittsHomingState>("ReturnState")->timedOut(); }

bool returnedBlockDone(StateMachine &sm) {
    return sm.state<M2FittsHomingState>("ReturnState")->isSettled() && noTrialsLeft(sm);
}

bool returnedPaused(StateMachine &sm) {
    return sm.state<M2FittsHomingState>("ReturnState")->isSettled() && fittsSM(sm).session()->pause_requested;
}

bool returnedNextTrial(StateMachine &sm) { return sm.state<M2FittsHomingState>("ReturnState")->isSettled(); }

// ------------------------------------------------------------------------------ YAML helpers
/** Reject unknown keys so that a typo never silently falls back to a default value. */
void checkKeys(const YAML::Node &n, const std::string &section, const std::vector<std::string> &known) {
    if (!n) return;
    if (!n.IsMap()) throw std::runtime_error("'" + section + "' must be a map of key: value");
    for (auto it = n.begin(); it != n.end(); ++it) {
        const std::string key = it->first.as<std::string>();
        if (std::find(known.begin(), known.end(), key) == known.end())
            throw std::runtime_error("unknown configuration key '" + section + "." + key + "' (typo?)");
    }
}

template <typename T>
void readValue(const YAML::Node &n, const std::string &section, const char *key, T &out) {
    if (!n || !n[key]) return;  // keep default
    try {
        out = n[key].as<T>();
    } catch (const std::exception &) {
        throw std::runtime_error("configuration key '" + section + "." + key + "' has an invalid value");
    }
}

void readVec2(const YAML::Node &n, const std::string &section, const char *key, VM2 &out) {
    std::vector<double> v;
    readValue(n, section, key, v);
    if (!n || !n[key]) return;
    if (v.size() != 2) throw std::runtime_error("configuration key '" + section + "." + key + "' needs 2 values");
    out = VM2(v[0], v[1]);
}

void makeDirectory(const std::string &path) {
    if (mkdir(path.c_str(), 0775) != 0 && errno != EEXIST)
        throw std::runtime_error("cannot create folder '" + path + "': " + std::strerror(errno));
}

std::string joinLines(const std::vector<std::string> &v) {
    std::string out;
    for (const auto &s : v) out += "\n  - " + s;
    return out;
}

}  // namespace

// ================================================================================== Construction
M2FittsMachine::M2FittsMachine() : session_(std::make_shared<FittsSession>()) {
    setRobot(std::make_unique<RobotM2>("M2_MELB"));

    // Configuration and trial table are read first. On failure nothing is driven (see init()).
    loadSetup();
    const fitts::FittsConfig &c = session_->cfg;

    addState("CalibState", std::make_shared<M2FittsCalibState>(robot(), session_));
    addState("GoHomeState", std::make_shared<M2FittsHomingState>(robot(), session_, "M2Fitts Go Home", PHASE_GO_HOME, c.homing_ref_speed));
    addState("ReadyState", std::make_shared<M2FittsReadyState>(robot(), session_));
    addState("ReachState", std::make_shared<M2FittsReachState>(robot(), session_));
    addState("ReturnState", std::make_shared<M2FittsHomingState>(robot(), session_, "M2Fitts Return Home", PHASE_RETURN, c.return_ref_speed));
    addState("EndState", std::make_shared<M2FittsEndState>(robot(), session_));
    addState("FaultState", std::make_shared<M2FittsFaultState>(robot(), session_));

    // Registered first, so it is evaluated before any other transition of every state.
    addTransitionFromAny(&faultRaised, "FaultState");

    addTransition("CalibState", &calibDone, "GoHomeState");
    addTransition("GoHomeState", &homingFinished, "ReadyState");
    addTransition("ReadyState", &noTrialsLeft, "EndState");
    addTransition("ReadyState", &startRequested, "ReachState");
    addTransition("ReachState", &trialFinished, "ReturnState");
    // Order matters: the first active transition is taken.
    addTransition("ReturnState", &returnTimedOut, "ReadyState");
    addTransition("ReturnState", &returnedBlockDone, "EndState");
    addTransition("ReturnState", &returnedPaused, "ReadyState");
    addTransition("ReturnState", &returnedNextTrial, "ReachState");

    setInitState("CalibState");
}

M2FittsMachine::~M2FittsMachine() {}

void M2FittsMachine::loadSetup() {
    FittsSession &s = *session_;
    fitts::FittsConfig &c = s.cfg;
    const std::string base = XSTR(BASE_DIRECTORY);
    config_path_ = base + "/config/M2FittsMachine.yaml";
    try {
        const YAML::Node root = YAML::LoadFile(config_path_);
        const YAML::Node app = root["M2FittsMachine"];
        if (!app) throw std::runtime_error("section 'M2FittsMachine' not found");
        checkKeys(app, "M2FittsMachine", {"trials", "task", "home_settling", "pd", "safety", "calibration", "logging"});

        const YAML::Node tr = app["trials"];
        checkKeys(tr, "trials", {"file", "d_column", "w_column", "units_to_m", "expected_rows", "start_trial"});
        readValue(tr, "trials", "file", c.trials_file);
        readValue(tr, "trials", "d_column", c.d_column);
        readValue(tr, "trials", "w_column", c.w_column);
        readValue(tr, "trials", "units_to_m", c.units_to_m);
        readValue(tr, "trials", "expected_rows", c.expected_trials);
        readValue(tr, "trials", "start_trial", c.start_trial);

        const YAML::Node task = app["task"];
        checkKeys(task, "task", {"home", "direction", "tolerance_factor", "dwell_time", "acquisition_timeout", "onset_speed"});
        readVec2(task, "task", "home", c.home);
        readValue(task, "task", "direction", c.direction);
        readValue(task, "task", "tolerance_factor", c.tolerance_factor);
        readValue(task, "task", "dwell_time", c.dwell_time);
        readValue(task, "task", "acquisition_timeout", c.acquisition_timeout);
        readValue(task, "task", "onset_speed", c.onset_speed);

        const YAML::Node hs = app["home_settling"];
        checkKeys(hs, "home_settling", {"tolerance", "rest_speed", "rest_time", "settle_timeout", "return_ref_speed", "homing_ref_speed"});
        readValue(hs, "home_settling", "tolerance", c.home_tolerance);
        readValue(hs, "home_settling", "rest_speed", c.rest_speed);
        readValue(hs, "home_settling", "rest_time", c.rest_time);
        readValue(hs, "home_settling", "settle_timeout", c.settle_timeout);
        readValue(hs, "home_settling", "return_ref_speed", c.return_ref_speed);
        readValue(hs, "home_settling", "homing_ref_speed", c.homing_ref_speed);

        const YAML::Node pd = app["pd"];
        checkKeys(pd, "pd", {"kp", "kd", "f_max", "velocity_filter_hz", "friction_compensation"});
        readVec2(pd, "pd", "kp", c.pd.kp);
        readVec2(pd, "pd", "kd", c.pd.kd);
        readValue(pd, "pd", "f_max", c.pd.f_max);
        readValue(pd, "pd", "velocity_filter_hz", c.pd.vel_filter_hz);
        readValue(pd, "pd", "friction_compensation", c.friction_compensation);

        const YAML::Node sf = app["safety"];
        checkKeys(sf, "safety", {"max_speed", "workspace_x", "workspace_y", "workspace_tolerance", "target_margin", "brake_damping"});
        readValue(sf, "safety", "max_speed", c.max_speed);
        readVec2(sf, "safety", "workspace_x", c.workspace_x);
        readVec2(sf, "safety", "workspace_y", c.workspace_y);
        readValue(sf, "safety", "workspace_tolerance", c.workspace_tolerance);
        readValue(sf, "safety", "target_margin", c.target_margin);
        readValue(sf, "safety", "brake_damping", c.brake_damping);

        const YAML::Node cal = app["calibration"];
        checkKeys(cal, "calibration", {"force", "damping", "still_speed", "still_time"});
        readValue(cal, "calibration", "force", c.calib_force);
        readValue(cal, "calibration", "damping", c.calib_damping);
        readValue(cal, "calibration", "still_speed", c.calib_still_speed);
        readValue(cal, "calibration", "still_time", c.calib_still_time);

        const YAML::Node lg = app["logging"];
        checkKeys(lg, "logging", {"folder"});
        readValue(lg, "logging", "folder", c.log_folder);

        const std::vector<std::string> cfg_errors = fitts::validateConfig(c);
        if (!cfg_errors.empty()) throw std::runtime_error("invalid configuration:" + joinLines(cfg_errors));

        const std::string csv_path =
            (!c.trials_file.empty() && c.trials_file[0] == '/') ? c.trials_file : base + "/config/" + c.trials_file;
        s.trials = fitts::loadTrialsCsv(csv_path, fitts::columnLetterToIndex(c.d_column),
                                        fitts::columnLetterToIndex(c.w_column), c.expected_trials, &s.csv);
        const std::vector<std::string> trial_errors = fitts::validateTrials(s.trials, c);
        if (!trial_errors.empty()) throw std::runtime_error("invalid trials:" + joinLines(trial_errors));

        s.next_trial = static_cast<std::size_t>(c.start_trial - 1);
        s.setup_ok = true;
    } catch (const std::exception &e) {
        s.setup_ok = false;
        s.setup_error = std::string(e.what()) + "\n  (configuration file: " + config_path_ + ")";
    }
}

// ================================================================================== init / end
void M2FittsMachine::init() {
    spdlog::debug("M2FittsMachine::init()");
    FittsSession &s = *session_;
    if (!s.setup_ok) {
        spdlog::critical("M2Fitts setup failed - the robot will NOT be driven:\n  {}", s.setup_error);
        std::raise(SIGTERM);  // clean exit (same pattern as CORC demo apps)
        return;
    }
    if (!robot()->initialise()) {
        s.setup_ok = false;
        spdlog::critical("M2Fitts: failed robot initialisation. Exiting...");
        std::raise(SIGTERM);
        return;
    }

    try {
        makeDirectory(s.cfg.log_folder);
        const std::string stem = s.cfg.log_folder + "/M2Fitts_" + fitts::localTimestamp("%Y%m%d_%H%M%S", false);
        if (!s.results.open(stem + "_trials.csv")) throw std::runtime_error("cannot create '" + stem + "_trials.csv'");
        writeParametersFile(stem + "_parameters.txt");

        // Time series recorded at every control loop by StateMachine::update(); started by activate().
        logHelper.initLogger("M2FittsTimeSeries", stem + "_timeseries.csv", LogFormat::CSV, true);
        logHelper.add(runningTime(), "time_s");
        logHelper.add(s.log_trial, "trial");
        logHelper.add(s.log_phase, "phase");
        logHelper.add(robot()->getEndEffPosition(), "x");        // x_1 = X, x_2 = Y [m]
        logHelper.add(robot()->getEndEffVelocity(), "dx");       // [m/s]
        logHelper.add(s.log_x_ref, "xref");                      // PD set-point [m]
        logHelper.add(s.log_force, "Fpd");                       // PD (or brake) force command [N]
        logHelper.add(robot()->getEndEffForce(), "Fmotor");      // force from measured motor torque [N]
        logHelper.add(s.log_in_target, "in_target");
        logHelper.add(s.log_saturated, "saturated");
        spdlog::info("M2Fitts: output files {}_trials.csv, {}_timeseries.csv, {}_parameters.txt", stem, stem, stem);
    } catch (const std::exception &e) {
        s.setup_ok = false;
        spdlog::critical("M2Fitts: cannot create output files: {}", e.what());
        std::raise(SIGTERM);
        return;
    }

    // Echo what was read so the column mapping can be checked before the block starts.
    spdlog::info("M2Fitts: {} trials read from {} (header: '{}'). Block starts at trial {}.", s.trials.size(),
                 s.csv.path, s.csv.header, s.cfg.start_trial);
    for (std::size_t i = 0; i < std::min<std::size_t>(3, s.trials.size()); ++i) {
        const fitts::Trial &t = s.trials[i];
        spdlog::info("  trial {}: D = {}, W = {} -> target x = {:.4f} m, tolerance +/-{:.4f} m", t.index, t.D, t.W,
                     fitts::targetX(t, s.cfg), fitts::targetTolerance(t, s.cfg));
    }
}

void M2FittsMachine::end() {
    StateMachine::end();  // exits current state (an interrupted trial is written as 'aborted'), disables the robot
    if (session_->results.isOpen()) {
        session_->results.close();
        spdlog::info("M2Fitts: trial results saved in {}", session_->results.path());
    }
}

void M2FittsMachine::hwStateUpdate() {
    StateMachine::hwStateUpdate();
    if (robot()->keyboard->getX() && !session_->pause_requested && session_->log_phase != PHASE_READY) {
        session_->pause_requested = true;
        spdlog::warn("M2Fitts: pause requested - the robot will wait at home after the current trial (S resumes).");
    }
}

void M2FittsMachine::writeParametersFile(const std::string &path) const {
    const FittsSession &s = *session_;
    std::ofstream p(path);
    if (!p) throw std::runtime_error("cannot create '" + path + "'");
    std::ostringstream hash;
    hash << std::hex << s.csv.fnv1a64;
    p << "# M2FittsMachine run parameters (effective values after YAML overrides)\n"
      << "created: " << fitts::localTimestamp("%Y-%m-%dT%H:%M:%S", false) << "\n"
      << "app_build: " << __DATE__ << " " << __TIME__ << "\n"
      << "config_file: " << config_path_ << "\n"
      << "trials_csv: " << s.csv.path << "\n"
      << "trials_csv_fnv1a64: " << hash.str() << "\n"
      << "trials_csv_header: " << s.csv.header << "\n"
      << "trials_count: " << s.trials.size() << "\n"
      << fitts::describeConfig(s.cfg);
}
