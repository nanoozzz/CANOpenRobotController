/**
 * \file M2FittsRobotHumanMachine.cpp
 * \brief Block 2 (human-robot shared control) of the robot-guidance Fitts' law experiment on the M2.
 *        See M2FittsRobotHumanMachine.h. Protocol code is that of M2FittsHumanMachine (Block 1) unless stated.
 */
#include "M2FittsRobotHumanMachine.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <ctime>
#include <initializer_list>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>

#include "yaml-cpp/yaml.h"

/******************************************************************************
 * Defaults and search paths
 ******************************************************************************/
//! Config file locations tried in order (the first one that opens wins).
static const char *kConfigCandidates[] = {
    "config/M2FittsRobotHuman.conf",
    "config/M2FittsRobotHumanMachine.cfg",
    "../config/M2FittsRobotHuman.conf",
    "../config/M2FittsRobotHumanMachine.cfg",
    XSTR(BASE_DIRECTORY) "/config/M2FittsRobotHumanMachine.cfg",
    "/home/nle/Fitts/CANOpenRobotController/config/M2FittsRobotHumanMachine.cfg"};

//! Robot-alone configuration (source of the PD gains), tried in order unless robot_config is set.
static const char *kRobotConfigCandidates[] = {
    "config/M2FittsMachine.yaml",
    "../config/M2FittsMachine.yaml",
    XSTR(BASE_DIRECTORY) "/config/M2FittsMachine.yaml",
    "/home/nle/Fitts/CANOpenRobotController/config/M2FittsMachine.yaml"};

//! Two IDs closer than this are the same condition when alpha is looked up by ID [bits].
static const double kIdMatchTolerance = 0.005;
//! Two alpha values closer than this are equal (csv formatting differences).
static const double kAlphaMatchTolerance = 1e-6;

/******************************************************************************
 * Small text helpers (csv / config parsing), as in Block 1
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
    M2FittsRobotHumanMachine &sm = static_cast<M2FittsRobotHumanMachine &>(SM);
    return sm.state<M2FittsReadyState>("ReadyState")->isReady() && !sm.sessionFinished();
}

static bool trialOver(StateMachine &SM) {
    return SM.state<M2FittsSharedReachState>("ReachState")->isTrialDone();
}

static bool returnedAndSessionOver(StateMachine &SM) {
    M2FittsRobotHumanMachine &sm = static_cast<M2FittsRobotHumanMachine &>(SM);
    return sm.state<M2FittsReturnState>("ReturnState")->isReturnDone() && sm.sessionFinished();
}

static bool returnedAndBreakDue(StateMachine &SM) {
    M2FittsRobotHumanMachine &sm = static_cast<M2FittsRobotHumanMachine &>(SM);
    return sm.state<M2FittsReturnState>("ReturnState")->isReturnDone() && sm.breakDue();
}

static bool returned(StateMachine &SM) {
    return SM.state<M2FittsReturnState>("ReturnState")->isReturnDone();
}

static bool breakOver(StateMachine &SM) {
    return SM.state<M2FittsBreakState>("BreakState")->isBreakOver();
}

static bool abortRequested(StateMachine &SM) {
    return static_cast<M2FittsRobotHumanMachine &>(SM).abortSignal();
}

static bool faultRaisedCond(StateMachine &SM) {
    return static_cast<M2FittsRobotHumanMachine &>(SM).faultPending();
}

/******************************************************************************
 * Construction
 ******************************************************************************/
M2FittsRobotHumanMachine::M2FittsRobotHumanMachine() {
    loadConfig();
    applyConfig();

    setRobot(std::make_unique<RobotM2>(cfgStr("robot_name", "M2_MELB")));

    addState("CalibState", std::make_shared<M2FittsCalibState>(robot(), this));
    addState("WaitUIState", std::make_shared<M2FittsWaitUIState>(robot(), this));
    addState("StandbyState", std::make_shared<M2FittsStandbyState>(robot(), this));
    addState("ReadyState", std::make_shared<M2FittsReadyState>(robot(), this));
    addState("ReachState", std::make_shared<M2FittsSharedReachState>(robot(), this));
    addState("ReturnState", std::make_shared<M2FittsReturnState>(robot(), this));
    addState("BreakState", std::make_shared<M2FittsBreakState>(robot(), this));
    addState("EndState", std::make_shared<M2FittsEndState>(robot(), this));
    addState("FaultState", std::make_shared<M2FittsFaultState>(robot(), this));

    //Safety fault raised by the shared reach: registered first, so that it pre-empts every other transition
    addTransitionFromAny(&faultRaisedCond, "FaultState");
    //Protocol: identical to Block 1
    addTransition("CalibState", &calibrationDone, "WaitUIState");
    addTransition("WaitUIState", &uiAvailable, "ReadyState");
    addTransition("ReadyState", &readyToStartTrial, "ReachState");
    addTransition("ReachState", &trialOver, "ReturnState");
    addTransition("ReturnState", &returnedAndSessionOver, "EndState");
    addTransition("ReturnState", &returnedAndBreakDue, "BreakState");
    addTransition("ReturnState", &returned, "ReachState");
    addTransition("BreakState", &breakOver, "ReadyState");
    //Manual abort from any state: registered last so that it never pre-empts a normal transition
    addTransitionFromAny(&abortRequested, "StandbyState");

    setInitState("CalibState");
}

M2FittsRobotHumanMachine::~M2FittsRobotHumanMachine() {
    if (resultsFile_.is_open()) resultsFile_.close();
}

/******************************************************************************
 * Configuration
 ******************************************************************************/
bool M2FittsRobotHumanMachine::loadConfigFile(const std::string &file) {
    std::ifstream f(file);
    if (!f.is_open()) return false;

    std::string line;
    while (std::getline(f, line)) {
        line = trimStr(line);
        if (line.empty() || line[0] == '#' || line[0] == ';') continue;
        size_t eq = line.find('=');
        if (eq == std::string::npos) continue;
        //In-line comments start at '#', '//' or ';'. (Block 1 stripped '#' only, so its lines written as
        //"value; //!< ..." did not parse and silently kept their defaults.)
        std::string value = line.substr(eq + 1);
        for (const char *marker : {"#", "//", ";"}) {
            size_t pos = value.find(marker);
            if (pos != std::string::npos) value = value.substr(0, pos);
        }
        cfg_[normaliseName(line.substr(0, eq))] = trimStr(value);
    }
    configFile_ = file;
    spdlog::info("M2FittsRobotHuman: loaded {} parameters from {}.", cfg_.size(), file);
    return true;
}

bool M2FittsRobotHumanMachine::loadConfig() {
    for (const char *candidate : kConfigCandidates) {
        if (loadConfigFile(candidate)) return true;
    }
    spdlog::warn("M2FittsRobotHuman: no config file found (config/M2FittsRobotHumanMachine.cfg, one level up, or in "
                 "the CORC source folder). Using built-in defaults.");
    return false;
}

std::string M2FittsRobotHumanMachine::cfgStr(const std::string &key, const std::string &def) const {
    auto it = cfg_.find(normaliseName(key));
    return (it == cfg_.end()) ? def : it->second;
}
double M2FittsRobotHumanMachine::cfgDbl(const std::string &key, double def) const {
    double v;
    return toDouble(cfgStr(key, ""), v) ? v : def;
}
int M2FittsRobotHumanMachine::cfgInt(const std::string &key, int def) const {
    return static_cast<int>(std::lround(cfgDbl(key, def)));
}
bool M2FittsRobotHumanMachine::cfgBool(const std::string &key, bool def) const {
    std::string v = normaliseName(cfgStr(key, ""));
    if (v == "true" || v == "1" || v == "yes" || v == "on") return true;
    if (v == "false" || v == "0" || v == "no" || v == "off") return false;
    return def;
}

std::string M2FittsRobotHumanMachine::expandPath(const std::string &s) const {
    std::string out = s;
    const std::string key = "{participant}";
    size_t pos;
    while ((pos = out.find(key)) != std::string::npos) out.replace(pos, key.size(), participant_);
    return out;
}

void M2FittsRobotHumanMachine::applyConfig() {
    //---- session
    participant_ = cfgStr("participant", participant_);
    block_ = cfgInt("block", block_);
    trialsDir_ = expandPath(cfgStr("trials_dir", trialsDir_));
    trialsPrefix_ = cfgStr("trials_prefix", trialsPrefix_);
    trialsFile_ = expandPath(cfgStr("trials_file", ""));
    warmupFile_ = expandPath(cfgStr("warmup_file", ""));
    if (warmupFile_.empty()) warmupFile_ = trialsDir_ + "/warmup.csv";
    cooldownFile_ = expandPath(cfgStr("cooldown_file", ""));
    if (cooldownFile_.empty()) cooldownFile_ = trialsDir_ + "/cooldown.csv";
    logDir_ = expandPath(cfgStr("results_dir", logDir_));
    bindIP_ = cfgStr("bind_ip", bindIP_);
    serverPort_ = cfgInt("port", serverPort_);
    robotConfigSetting_ = expandPath(cfgStr("robot_config", ""));

    //---- UI
    uiRequired_ = cfgBool("ui_required", uiRequired_);
    uiDivider_ = std::max(1, cfgInt("ui_stream_divider", uiDivider_));
    displayGain_ = cfgDbl("display_gain", displayGain_);

    //---- protocol: Block 1 keys and meanings
    params_.originX = cfgDbl("origin_x", params_.originX);
    params_.originY = cfgDbl("origin_y", params_.originY);
    params_.taskDirection = (cfgDbl("task_direction", params_.taskDirection) < 0) ? -1. : 1.;
    params_.useYChannel = cfgBool("use_y_channel", params_.useYChannel);
    params_.channelK = cfgDbl("channel_k", params_.channelK);
    params_.channelD = cfgDbl("channel_d", params_.channelD);
    params_.channelFMax = cfgDbl("channel_f_max", params_.channelFMax);

    params_.dwellTime = cfgDbl("dwell_time", params_.dwellTime);
    params_.homeExitRadius = cfgDbl("home_exit_radius", params_.homeExitRadius);
    params_.maxTrialTime = cfgDbl("max_trial_time", params_.maxTrialTime);
    params_.successHoldTime = cfgDbl("success_hold_time", params_.successHoldTime);

    params_.returnOffset = cfgDbl("return_offset", params_.returnOffset);
    params_.returnSpeed = cfgDbl("return_speed", params_.returnSpeed);
    params_.returnMinTime = cfgDbl("return_min_time", params_.returnMinTime);
    params_.originTolerance = cfgDbl("origin_tolerance", params_.originTolerance);
    params_.maxReturnTime = cfgDbl("max_return_time", params_.maxReturnTime);
    params_.originSnapTime = cfgDbl("origin_snap_time", params_.originSnapTime);
    params_.originHoldTime = cfgDbl("origin_hold_time", params_.originHoldTime);
    params_.originSettleSpeed = cfgDbl("origin_settle_speed", params_.originSettleSpeed);
    params_.originSettleTime = cfgDbl("origin_settle_time", params_.originSettleTime);
    params_.forceLimitTime = cfgDbl("force_limit_time", params_.forceLimitTime);
    params_.forceGraceTime = cfgDbl("force_grace_time", params_.forceGraceTime);

    params_.roundBreakTime = cfgDbl("round_break_time", params_.roundBreakTime);
    params_.roundBreakMinTime = cfgDbl("round_break_min_time", params_.roundBreakMinTime);
    params_.warmupRestTime = cfgDbl("warmup_rest_time", params_.warmupRestTime);
    params_.cooldownRestTime = cfgDbl("cooldown_rest_time", params_.cooldownRestTime);
    params_.readyHoldTime = cfgDbl("ready_hold_time", params_.readyHoldTime);

    params_.kPosVel = cfgDbl("k_pos_vel", params_.kPosVel);
    params_.forceLimit = cfgDbl("force_limit", params_.forceLimit);
    params_.kHold = cfgDbl("k_hold", params_.kHold);
    params_.awayTolerance = cfgDbl("away_tolerance", params_.awayTolerance);
    params_.awaySettleSpeed = cfgDbl("away_settle_speed", params_.awaySettleSpeed);
    params_.maxMoveExtraTime = cfgDbl("max_move_extra_time", params_.maxMoveExtraTime);

    params_.nRounds = cfgInt("n_rounds", params_.nRounds);
    params_.trialsPerRound = cfgInt("trials_per_round", params_.trialsPerRound);
    params_.warmupRepeats = cfgInt("warmup_repeats", params_.warmupRepeats);
    params_.cooldownRepeats = cfgInt("cooldown_repeats", params_.cooldownRepeats);
    params_.warmupRequireGo = cfgBool("warmup_require_go", params_.warmupRequireGo);
    params_.cooldownRequireGo = cfgBool("cooldown_require_go", params_.cooldownRequireGo);
    params_.requireGoEachSubBlock = cfgBool("require_go_each_sub_block", params_.requireGoEachSubBlock);

    //---- Block 2: safety of the shared reach
    params_.reachForceLimit = cfgDbl("reach_force_limit", params_.reachForceLimit);
    params_.maxSpeed = cfgDbl("max_speed", params_.maxSpeed);
    params_.workspaceTolerance = cfgDbl("workspace_tolerance", params_.workspaceTolerance);
    params_.brakeDamping = cfgDbl("brake_damping", params_.brakeDamping);
    params_.brakeFMax = cfgDbl("brake_f_max", params_.brakeFMax);
    params_.conflictThreshold = cfgDbl("conflict_threshold", params_.conflictThreshold);

    //---- Block 2: shared-control law
    alphaOverride_ = cfgDbl("alpha_override", alphaOverride_);
    blend_.humanForceSign = (cfgDbl("human_force_sign", blend_.humanForceSign) < 0) ? -1. : 1.;
    humanForceFilterHz_ = cfgDbl("human_force_filter_hz", humanForceFilterHz_);
    blend_.platformAssistGain = cfgDbl("platform_assist_gain", blend_.platformAssistGain);
    blend_.platformForceThreshold = cfgDbl("platform_assist_force_threshold", blend_.platformForceThreshold);
    blend_.platformVelThreshold = cfgDbl("platform_assist_velocity_threshold", blend_.platformVelThreshold);
    blend_.cancelFMax = cfgDbl("cancel_f_max", blend_.cancelFMax);
    blend_.cmdFMax = cfgDbl("command_f_max", blend_.cmdFMax);
    blend_.channelSaturate = cfgBool("channel_saturate", blend_.channelSaturate);
    //The y channel is the Block 1 task constraint: same parameters as the rest of the protocol
    blend_.useYChannel = params_.useYChannel;
    blend_.channelK = params_.channelK;
    blend_.channelD = params_.channelD;
    blend_.channelFMax = params_.channelFMax;

    if (std::fabs(displayGain_ - 1.0) > 1e-9)
        spdlog::warn("M2FittsRobotHuman: display_gain = {} (non-unity): the visual ID differs from the mechanical one.", displayGain_);

    //---- session tag and output files
    std::time_t t = std::time(nullptr);
    char stamp[32];
    std::strftime(stamp, sizeof(stamp), "%Y%m%d-%H%M%S", std::localtime(&t));
    sessionTag_ = participant_ + "_B" + std::to_string(block_) + (alphaOverride_ >= 0. ? "_alphaOVR" : "") + "_" + stamp;
    resultsPath_ = logDir_ + "/M2FittsRobotHuman_" + sessionTag_ + "_trials.csv";
    rawPath_ = logDir_ + "/M2FittsRobotHuman_" + sessionTag_ + "_raw.csv";
    paramsPath_ = logDir_ + "/M2FittsRobotHuman_" + sessionTag_ + "_parameters.txt";
}

/******************************************************************************
 * Robot PD (u_r): read from the robot-alone configuration, so that Block 2 runs the calibrated controller
 ******************************************************************************/
bool M2FittsRobotHumanMachine::loadRobotController() {
    std::vector<std::string> candidates;
    if (!robotConfigSetting_.empty())
        candidates.push_back(robotConfigSetting_);
    else
        for (const char *c : kRobotConfigCandidates) candidates.push_back(c);

    for (const std::string &file : candidates) {
        std::ifstream test(file);
        if (!test.is_open()) continue;
        test.close();
        try {
            const YAML::Node root = YAML::LoadFile(file);
            const YAML::Node app = root["M2FittsMachine"];
            if (!app) throw std::runtime_error("section 'M2FittsMachine' not found");
            const YAML::Node pd = app["pd"];
            if (!pd) throw std::runtime_error("section 'M2FittsMachine.pd' not found");

            auto vec2 = [&pd](const char *key) {
                if (!pd[key]) throw std::runtime_error(std::string("pd.") + key + " is missing");
                const std::vector<double> v = pd[key].as<std::vector<double>>();
                if (v.size() != 2) throw std::runtime_error(std::string("pd.") + key + " needs 2 values [x, y]");
                return VM2(v[0], v[1]);
            };
            pdGains_.kp = vec2("kp");
            pdGains_.kd = vec2("kd");
            if (!pd["f_max"]) throw std::runtime_error("pd.f_max is missing");
            pdGains_.f_max = pd["f_max"].as<double>();
            //Optional keys: M2FittsMachine's in-code defaults when absent (FittsCore.h)
            pdGains_.vel_filter_hz = pd["velocity_filter_hz"] ? pd["velocity_filter_hz"].as<double>() : 0.;
            const bool robotFrictionComp = pd["friction_compensation"] ? pd["friction_compensation"].as<bool>() : false;
            //Stiction compensation: part of u_r, same values as M2FittsMachine (x only here, scaled by alpha). Off if absent.
            if (pd["stiction_comp"]) blend_.robotStictionComp = vec2("stiction_comp")(0);
            if (pd["stiction_rest_speed"]) blend_.stictionRestSpeed = pd["stiction_rest_speed"].as<double>();
            if (pd["stiction_deadband"]) blend_.stictionDeadband = pd["stiction_deadband"].as<double>();
            if (blend_.robotStictionComp < 0. || !(blend_.stictionRestSpeed > 0.) || blend_.stictionDeadband < 0.)
                throw std::runtime_error("invalid pd.stiction_* values (comp >= 0, rest speed > 0, deadband >= 0 required)");

            if (!(pdGains_.kp.array() > 0.).all() || !(pdGains_.kd.array() >= 0.).all() || !(pdGains_.f_max > 0.) ||
                pdGains_.vel_filter_hz < 0.)
                throw std::runtime_error("invalid PD values (kp > 0, kd >= 0, f_max > 0, velocity_filter_hz >= 0 required)");

            robotConfigFile_ = file;
            spdlog::info("M2FittsRobotHuman: robot PD (u_r) from {}: kp = [{}, {}] N/m, kd = [{}, {}] N.s/m, "
                         "f_max = {} N, velocity filter = {} Hz.",
                         file, pdGains_.kp(0), pdGains_.kp(1), pdGains_.kd(0), pdGains_.kd(1), pdGains_.f_max,
                         pdGains_.vel_filter_hz);
            spdlog::info("M2FittsRobotHuman: robot stiction compensation {} N (x alpha) while slower than {} m/s and more than "
                         "{} mm from the target centre{}.",
                         blend_.robotStictionComp, blend_.stictionRestSpeed, 1000. * blend_.stictionDeadband,
                         blend_.robotStictionComp > 0. ? "" : " (off)");
            if (!robotFrictionComp)
                spdlog::warn("M2FittsRobotHuman: {} has pd.friction_compensation = false, whereas Block 1 and Block 2 run "
                             "RobotM2's friction compensation: the alpha = 1 end of Block 2 then differs from the "
                             "robot-alone calibration by that compensation.",
                             file);

            //Geometry cross-check (information only: MT depends on A and W, not on where the targets lie)
            const YAML::Node task = app["task"];
            if (task && task["home"]) {
                const std::vector<double> h = task["home"].as<std::vector<double>>();
                if (h.size() == 2 && (std::fabs(h[0] - params_.originX) > 1e-3 || std::fabs(h[1] - params_.originY) > 1e-3))
                    spdlog::warn("M2FittsRobotHuman: robot-alone home [{}, {}] m differs from the origin [{}, {}] m used here.",
                                 h[0], h[1], params_.originX, params_.originY);
            }
            if (task && task["direction"]) {
                const double d = task["direction"].as<double>();
                if ((d < 0) != (params_.taskDirection < 0))
                    spdlog::warn("M2FittsRobotHuman: robot-alone task direction ({}) differs from task_direction ({}) used here.",
                                 d, params_.taskDirection);
            }
            return true;
        } catch (const std::exception &e) {
            spdlog::critical("M2FittsRobotHuman: cannot read the robot PD from {}: {}", file, e.what());
            return false;
        }
    }
    spdlog::critical("M2FittsRobotHuman: robot-alone configuration M2FittsMachine.yaml not found{}.",
                     robotConfigSetting_.empty() ? std::string(" in config/, ../config/ or the CORC source folder")
                                                 : " at robot_config = " + robotConfigSetting_);
    return false;
}

/******************************************************************************
 * Trial tables (Block 1 format + alpha column)
 ******************************************************************************/
bool M2FittsRobotHumanMachine::readTrialCsv(const std::string &file, std::vector<FittsTrial> &trials, bool &hasAlpha) const {
    hasAlpha = false;
    std::ifstream f(file);
    if (!f.is_open()) {
        spdlog::critical("M2FittsRobotHuman: cannot open trial file {}.", file);
        return false;
    }

    int colIndex = -1, colA = -1, colW = -1, colID = -1, colAlpha = -1;
    bool columnsKnown = false;
    std::string line;
    int lineNb = 0;

    while (std::getline(f, line)) {
        lineNb++;
        line = trimStr(line);
        if (line.empty()) continue;
        std::vector<std::string> fields = splitStr(line, ',');
        if (fields.size() < 3) {
            spdlog::warn("M2FittsRobotHuman: {} line {} ignored (less than 3 columns).", file, lineNb);
            continue;
        }

        if (!columnsKnown) {
            double dummy;
            bool isHeader = false;
            for (const auto &s : fields)
                if (!toDouble(s, dummy)) isHeader = true;

            if (isHeader) {
                for (size_t i = 0; i < fields.size(); i++) {
                    std::string n = normaliseName(fields[i]);
                    if (n == "index" || n == "trial" || n == "trialnb" || n == "no") colIndex = (int)i;
                    else if (n == "a" || n == "acm" || n == "amplitude") colA = (int)i;
                    else if (n == "w" || n == "wcm" || n == "width") colW = (int)i;
                    else if (n == "id" || n == "idbits") colID = (int)i;
                    else if (n == "alpha" || n == "autonomy" || n == "autonomylevel") colAlpha = (int)i;
                }
                if (colA < 0 || colW < 0) {
                    spdlog::critical("M2FittsRobotHuman: {} header does not contain A and W columns.", file);
                    return false;
                }
                columnsKnown = true;
                continue;
            } else {
                //No header: documented order index,A,W,ID[,alpha]
                colIndex = 0;
                colA = 1;
                colW = 2;
                colID = 3;
                colAlpha = (fields.size() >= 5) ? 4 : -1;
                columnsKnown = true;
            }
        }

        FittsTrial t;
        double v;
        if (colIndex >= 0 && colIndex < (int)fields.size() && toDouble(fields[colIndex], v)) t.index = (int)std::lround(v);
        else t.index = (int)trials.size() + 1;

        if (colA >= (int)fields.size() || !toDouble(fields[colA], t.A_cm)) {
            spdlog::critical("M2FittsRobotHuman: {} line {}: invalid A.", file, lineNb);
            return false;
        }
        if (colW >= (int)fields.size() || !toDouble(fields[colW], t.W_cm) || t.W_cm <= 0) {
            spdlog::critical("M2FittsRobotHuman: {} line {}: invalid W.", file, lineNb);
            return false;
        }
        if (colID >= 0 && colID < (int)fields.size() && toDouble(fields[colID], v)) t.ID_bits = v;
        else t.ID_bits = std::log2(t.A_cm / t.W_cm + 1.);

        if (colAlpha >= 0) {
            if (colAlpha >= (int)fields.size() || !toDouble(fields[colAlpha], v)) {
                spdlog::critical("M2FittsRobotHuman: {} line {}: missing or invalid alpha.", file, lineNb);
                return false;
            }
            t.alpha = v;
        }
        trials.push_back(t);
    }

    hasAlpha = (colAlpha >= 0);
    if (trials.empty()) {
        spdlog::critical("M2FittsRobotHuman: no trial read from {}.", file);
        return false;
    }
    return true;
}

bool M2FittsRobotHumanMachine::loadTrialTable() {
    blockTrials_.clear();

    //One file holding every trial (trials_file), rows in presentation order: round r is made of rows
    //(r-1)*trials_per_round+1 .. r*trials_per_round - the layout of schedule/bal_group_all.csv
    if (!trialsFile_.empty()) {
        std::vector<FittsTrial> all;
        bool hasAlpha = false;
        if (!readTrialCsv(trialsFile_, all, hasAlpha)) return false;
        if (!hasAlpha) {
            spdlog::critical("M2FittsRobotHuman: {} has no 'alpha' column. Block 2 needs the autonomy level of every trial.",
                             trialsFile_);
            return false;
        }
        const size_t expected = (size_t)std::max(0, params_.nRounds) * (size_t)std::max(0, params_.trialsPerRound);
        if (params_.trialsPerRound <= 0 || all.size() != expected) {
            spdlog::critical("M2FittsRobotHuman: {} holds {} trials, but n_rounds x trials_per_round = {} x {} = {}. Rounds "
                             "are cut from consecutive rows, so the counts must match exactly.",
                             trialsFile_, all.size(), params_.nRounds, params_.trialsPerRound, expected);
            return false;
        }
        for (size_t i = 0; i < all.size(); i++) {
            all[i].round = (int)(i / params_.trialsPerRound) + 1;
            all[i].inRound = (int)(i % params_.trialsPerRound) + 1;
            blockTrials_.push_back(all[i]);
        }
        spdlog::info("M2FittsRobotHuman: {} trials loaded from {} ({} rounds of {} consecutive rows).", all.size(),
                     trialsFile_, params_.nRounds, params_.trialsPerRound);
        return true;
    }

    //Otherwise one file per round, as in Block 1: <trials_dir>/<trials_prefix><n>.csv
    for (int g = 1; g <= 1; g++) {
        std::vector<FittsTrial> round;
        bool hasAlpha = false;
        std::string file = trialsDir_ + "/" + "bal_group_all_alpha" + ".csv";
        if (!readTrialCsv(file, round, hasAlpha)) return false;
        if (!hasAlpha) {
            spdlog::critical("M2FittsRobotHuman: {} has no 'alpha' column. Block 2 needs the autonomy level of every "
                             "trial; a Block 1 table cannot be used as is.",
                             file);
            return false;
        }
        if ((int)round.size() != params_.trialsPerRound)
            spdlog::warn("M2FittsRobotHuman: {} holds {} trials ({} expected).", file, round.size(), params_.trialsPerRound);
        for (size_t i = 0; i < round.size(); i++) {
            round[i].round = g;
            round[i].inRound = (int)i + 1;
            blockTrials_.push_back(round[i]);
        }
        spdlog::info("M2FittsRobotHuman: round {} loaded ({} trials) from {}.", g, round.size(), file);
    }
    return !blockTrials_.empty();
}

bool M2FittsRobotHumanMachine::lookupAlpha(double id, double &alpha) const {
    for (const auto &e : alphaTable_) {
        if (std::fabs(e.first - id) < kIdMatchTolerance) {
            alpha = e.second;
            return true;
        }
    }
    return false;
}

bool M2FittsRobotHumanMachine::buildAlphaTable() {
    alphaTable_.clear();
    for (const auto &t : blockTrials_) {
        if (!std::isfinite(t.alpha) || t.alpha < 0. || t.alpha > 1.) {
            spdlog::critical("M2FittsRobotHuman: round {} trial {} (index {}): alpha = {} is not in [0, 1].",
                             t.round, t.inRound, t.index, t.alpha);
            return false;
        }
        double known;
        if (lookupAlpha(t.ID_bits, known)) {
            if (std::fabs(known - t.alpha) > kAlphaMatchTolerance) {
                spdlog::critical("M2FittsRobotHuman: ID {:.4f} bits carries two alpha values ({} and {}, round {} trial {}). "
                                 "The design assigns one alpha per ID: check the tables.",
                                 t.ID_bits, known, t.alpha, t.round, t.inRound);
                return false;
            }
        } else {
            alphaTable_.push_back(std::make_pair(t.ID_bits, t.alpha));
        }
    }
    std::sort(alphaTable_.begin(), alphaTable_.end());
    spdlog::info("M2FittsRobotHuman: autonomy level per ID ({} IDs, participant {}):", alphaTable_.size(), participant_);
    for (const auto &e : alphaTable_) spdlog::info("   ID {:.4f} bits -> alpha = {:.4f}", e.first, e.second);
    return !alphaTable_.empty();
}

/**
 * \brief Loads a sub-block (warm-up or cool-down) from its own csv: every row is a trial, in file order.
 *
 * The file has the columns of the main trial table. Its 'alpha' column is taken as it stands, because a
 * sub-block deliberately uses several alpha values at one ID; it is therefore NOT checked against the block's
 * ID -> alpha table. Rows without an alpha column fall back to that table.
 */
bool M2FittsRobotHumanMachine::loadSubBlock(const std::string &file, int repeats, std::vector<FittsTrial> &trials,
                                            const char *label) {
    trials.clear();
    if (repeats <= 0) {
        spdlog::info("M2FittsRobotHuman: no {} trials (repeats = {}).", label, repeats);
        return true;
    }

    std::vector<FittsTrial> rows;
    bool hasAlpha = false;
    if (!readTrialCsv(file, rows, hasAlpha)) {
        spdlog::critical("M2FittsRobotHuman: {} table {} could not be read. Set its repeats to 0 to run without it.",
                         label, file);
        return false;
    }
    for (auto &t : rows) {
        if (hasAlpha) {
            if (!std::isfinite(t.alpha) || t.alpha < 0. || t.alpha > 1.) {
                spdlog::critical("M2FittsRobotHuman: {} table {}: alpha = {} is not in [0, 1].", label, file, t.alpha);
                return false;
            }
        } else if (!lookupAlpha(t.ID_bits, t.alpha)) {
            spdlog::critical("M2FittsRobotHuman: {}: ID {:.4f} bits has no alpha - {} has no alpha column and that ID "
                             "does not occur in the block table.",
                             label, t.ID_bits, file);
            return false;
        }
    }

    int n = 1;
    for (int r = 1; r <= repeats; r++) {
        for (size_t i = 0; i < rows.size(); i++) {
            FittsTrial t = rows[i];
            t.round = r;
            t.inRound = (int)i + 1;
            t.index = n++;
            trials.push_back(t);
        }
    }
    std::string alphas;
    for (size_t i = 0; i < rows.size(); i++) alphas += (i ? ", " : "") + num(rows[i].alpha, 2);
    spdlog::info("M2FittsRobotHuman: {}: {} trials from {} ({} rows x {} pass(es)); alpha order: {}.", label,
                 trials.size(), file, rows.size(), repeats, alphas);
    return !trials.empty();
}

bool M2FittsRobotHumanMachine::checkTrialTable() {
    bool ok = true;
    //1. Index of difficulty consistency: ID = log2(A/W + 1)
    int nIDMismatch = 0;
    for (const auto &t : blockTrials_) {
        double expected = std::log2(t.A_cm / t.W_cm + 1.);
        if (std::fabs(expected - t.ID_bits) > 1e-3) nIDMismatch++;
    }
    if (nIDMismatch > 0)
        spdlog::warn("M2FittsRobotHuman: {} trials whose ID column differs from log2(A/W+1) by more than 1e-3.", nIDMismatch);

    //2. Workspace: far edge of the farthest target and the off-origin return point must be reachable
    double maxA = 0., maxHalfW = 0.;
    for (const auto &t : blockTrials_) {
        maxA = std::max(maxA, t.A());
        maxHalfW = std::max(maxHalfW, t.halfW());
    }
    double xFar = params_.originX + params_.taskDirection * (maxA + maxHalfW);
    double xNear = params_.originX;
    const double xMin = 0.02, xMax = 0.605;
    if (xFar > xMax || xFar < xMin || xNear > xMax || xNear < xMin) {
        spdlog::critical("M2FittsRobotHuman: workspace violation - origin {:.3f} m, farthest target edge {:.3f} m "
                         "(allowed [{:.2f}, {:.2f}] m). Adjust origin_x.",
                         xNear, xFar, xMin, xMax);
        ok = false;
    }
    double xAway = params_.originX + params_.taskDirection * params_.returnOffset;
    if (xAway > xMax || xAway < xMin) {
        spdlog::critical("M2FittsRobotHuman: return position {:.3f} m is outside the usable travel.", xAway);
        ok = false;
    }
    if (params_.originY < 0.02 || params_.originY > 0.42)
        spdlog::warn("M2FittsRobotHuman: origin_y = {:.3f} m is close to the y stops ([0, 0.440] m).", params_.originY);

    //3. Design balance (informative)
    std::map<std::string, int> counts;
    for (const auto &t : blockTrials_) {
        char key[64];
        std::snprintf(key, sizeof(key), "A=%.4f/W=%.4f", t.A_cm, t.W_cm);
        counts[key]++;
    }
    spdlog::info("M2FittsRobotHuman: {} block trials over {} conditions ({} repetitions each if balanced).",
                 blockTrials_.size(), counts.size(), counts.empty() ? 0 : (int)(blockTrials_.size() / counts.size()));
    for (const auto &c : counts)
        if (counts.begin()->second != c.second)
            spdlog::warn("M2FittsRobotHuman: unbalanced design - {} appears {} times.", c.first, c.second);

    return ok;
}

/******************************************************************************
 * Output files
 ******************************************************************************/
bool M2FittsRobotHumanMachine::openResultsFile() {
    resultsFile_.open(resultsPath_, std::ios::out | std::ios::trunc);
    if (!resultsFile_.is_open()) {
        spdlog::critical("M2FittsRobotHuman: cannot open results file {} (does the {} folder exist?).", resultsPath_, logDir_);
        return false;
    }
    //Block 1 columns in the Block 1 order, Block 2 columns appended
    resultsFile_ << "participant,block,phase,round,trial_in_round,trial_index,"
                    "A_cm,W_cm,ID_bits,x_start_cm,"
                    "MT_s,RT_s,MT_move_s,t_first_entry_s,n_entries,"
                    "x_entry_cm,x_sel_cm,v_peak_ms,success,"
                    "return_time_s,return_timeout,return_abort,"
                    "dwell_s,t_onset_s,t_end_s,ui_connected,"
                    "alpha,imp_h_Ns,imp_r_Ns,Fh_peak_N,conflict_frac,blend_sat_frac,reach_abort\n";
    resultsFile_.flush();
    spdlog::info("M2FittsRobotHuman: trial results -> {}", resultsPath_);
    return true;
}

void M2FittsRobotHumanMachine::writeResultRow(const FittsTrialResult &r) {
    if (!resultsFile_.is_open()) return;
    resultsFile_ << participant_ << "," << block_ << ","
                 << (r.phase == PHASE_WARMUP ? "warmup" : (r.phase == PHASE_COOLDOWN ? "cooldown" : "block")) << ","
                 << r.trial.round << "," << r.trial.inRound << "," << r.trial.index << ","
                 << num(r.trial.A_cm, 4) << "," << num(r.trial.W_cm, 6) << "," << num(r.trial.ID_bits, 4) << "," << num(r.x_start_cm, 4) << ","
                 << num(r.MT, 4) << "," << num(r.RT, 4) << "," << num(r.MT_move, 4) << ","
                 << num(r.t_first_entry, 4) << "," << r.nEntries << ","
                 << num(r.x_entry_cm, 4) << "," << num(r.x_sel_cm, 4) << "," << num(r.v_peak, 4) << ","
                 << (r.success ? 1 : 0) << ","
                 << num(r.returnTime, 4) << "," << (r.returnTimeout ? 1 : 0) << "," << (r.returnAbort ? 1 : 0) << ","
                 << num(params_.dwellTime, 3) << "," << num(r.t_onset, 4) << "," << num(r.t_end, 4) << ","
                 << (ui.connected() ? 1 : 0) << ","
                 << num(r.alpha, 4) << "," << num(r.impH, 4) << "," << num(r.impR, 4) << "," << num(r.FhPeak, 3) << ","
                 << num(r.conflictFrac, 4) << "," << num(r.satFrac, 4) << "," << r.reachAbort << "\n";
    resultsFile_.flush();  //trial by trial: an interrupted session keeps all completed trials
}

bool M2FittsRobotHumanMachine::writeParametersFile(const std::string &file) const {
    std::ofstream f(file, std::ios::out | std::ios::trunc);
    if (!f.is_open()) return false;
    const FittsParams &p = params_;
    const fsc::BlendParams &b = blend_;
    std::time_t t = std::time(nullptr);
    char stamp[32];
    std::strftime(stamp, sizeof(stamp), "%Y-%m-%dT%H:%M:%S", std::localtime(&t));

    f << "# M2FittsRobotHumanMachine - effective parameters of this session (Block 2, shared control)\n"
      << "created: " << stamp << "\n"
      << "app_build: " << __DATE__ << " " << __TIME__ << "\n"
      << "config_file: " << (configFile_.empty() ? std::string("(none - built-in defaults)") : configFile_) << "\n"
      << "participant: " << participant_ << "\n"
      << "block: " << block_ << "\n"
      << "trials_dir: " << trialsDir_ << "\n"
      << "trials_prefix: " << trialsPrefix_ << "\n"
      << "trials_file: " << (trialsFile_.empty() ? std::string("(none - one file per round)") : trialsFile_) << "\n"
      << "warmup_table: " << (params_.warmupRepeats > 0 ? warmupFile_ : std::string("(none)")) << "\n"
      << "cooldown_table: " << (params_.cooldownRepeats > 0 ? cooldownFile_ : std::string("(none)")) << "\n"
      << "results_file: " << resultsPath_ << "\n"
      << "raw_log_file: " << rawPath_ << "\n"
      << "\n# Task axis x: u = alpha*u_r + (1-alpha)*u_h, realised as F_cmd,x = alpha*F_pd,x - alpha*k_h*F_h,x\n"
      << "# F_h = human_force_sign*F_int ; k_h = 1 - platform_assist_gain*human_force_sign while RobotM2's assist is active\n"
      << "# y: Block 1 virtual channel (alpha-independent)\n"
      << "robot_config_file: " << robotConfigFile_ << "\n"
      << "pd.kp_N_per_m: [" << pdGains_.kp(0) << ", " << pdGains_.kp(1) << "]\n"
      << "pd.kd_Ns_per_m: [" << pdGains_.kd(0) << ", " << pdGains_.kd(1) << "]\n"
      << "pd.f_max_N: " << pdGains_.f_max << "\n"
      << "pd.velocity_filter_hz: " << pdGains_.vel_filter_hz << "\n"
      << "pd.stiction_comp_x_N: " << blend_.robotStictionComp << " (x alpha, below " << blend_.stictionRestSpeed
      << " m/s, outside " << blend_.stictionDeadband << " m of the target centre)\n"
      << "robotm2_friction_compensation: " << (frictionComp_ ? "true" : "false") << "\n"
      << "alpha_override: " << (alphaOverride_ >= 0. ? num(alphaOverride_, 4) : std::string("off")) << "\n"
      << "human_force_sign: " << b.humanForceSign << "\n"
      << "human_force_filter_hz: " << humanForceFilterHz_ << "\n"
      << "platform_assist_gain: " << b.platformAssistGain << "\n"
      << "platform_assist_force_threshold_N: " << b.platformForceThreshold << "\n"
      << "platform_assist_velocity_threshold_mps: " << b.platformVelThreshold << "\n"
      << "cancel_f_max_N: " << b.cancelFMax << "\n"
      << "command_f_max_N: " << b.cmdFMax << "\n"
      << "channel: use_y_channel=" << (b.useYChannel ? "true" : "false") << " k=" << b.channelK << " d=" << b.channelD
      << " f_max=" << b.channelFMax << " saturate=" << (b.channelSaturate ? "true" : "false") << "\n"
      << "reach_force_limit_N: " << p.reachForceLimit << "\n"
      << "force_limit_time_s: " << p.forceLimitTime << "\n"
      << "max_speed_mps: " << p.maxSpeed << "\n"
      << "workspace_tolerance_m: " << p.workspaceTolerance << "\n"
      << "brake_damping_Ns_per_m: " << p.brakeDamping << "\n"
      << "brake_f_max_N: " << p.brakeFMax << "\n"
      << "conflict_threshold_N: " << p.conflictThreshold << "\n"
      << "\n# Protocol (Block 1 meaning)\n"
      << "origin_m: [" << p.originX << ", " << p.originY << "]\n"
      << "task_direction: " << p.taskDirection << "\n"
      << "dwell_time_s: " << p.dwellTime << "\n"
      << "home_exit_radius_m: " << p.homeExitRadius << "\n"
      << "max_trial_time_s: " << p.maxTrialTime << "\n"
      << "success_hold_time_s: " << p.successHoldTime << "\n"
      << "return: offset=" << p.returnOffset << " speed=" << p.returnSpeed << " min_time=" << p.returnMinTime
      << " origin_tolerance=" << p.originTolerance << " max_return_time=" << p.maxReturnTime
      << " snap_time=" << p.originSnapTime << " hold_time=" << p.originHoldTime
      << " settle_speed=" << p.originSettleSpeed << " settle_time=" << p.originSettleTime << "\n"
      << "driven_moves: k_pos_vel=" << p.kPosVel << " k_hold=" << p.kHold << " force_limit=" << p.forceLimit
      << " force_grace_time=" << p.forceGraceTime << " away_tolerance=" << p.awayTolerance
      << " away_settle_speed=" << p.awaySettleSpeed << " max_move_extra_time=" << p.maxMoveExtraTime << "\n"
      << "breaks: every " << p.trialsPerRound << " trials, " << p.roundBreakTime << " s (a go cannot end one before "
      << p.roundBreakMinTime << " s), ready_hold=" << p.readyHoldTime << "\n"
      << "structure: n_rounds=" << p.nRounds << " trials_per_round=" << p.trialsPerRound
      << " warmup_trials=" << warmupTrials_.size() << " cooldown_trials=" << cooldownTrials_.size()
      << " warmup_rest=" << p.warmupRestTime << " cooldown_rest=" << p.cooldownRestTime
      << " warmup_require_go=" << (p.warmupRequireGo ? "true" : "false")
      << " cooldown_require_go=" << (p.cooldownRequireGo ? "true" : "false")
      << " require_go_each_sub_block=" << (p.requireGoEachSubBlock ? "true" : "false") << "\n"
      << "ui: bind_ip=" << bindIP_ << " port=" << serverPort_ << " required=" << (uiRequired_ ? "true" : "false")
      << " divider=" << uiDivider_ << " display_gain=" << displayGain_ << "\n"
      << "\n# Autonomy level per ID (block tables)\n";
    for (const auto &e : alphaTable_) f << "alpha_ID_" << num(e.first, 4) << ": " << num(e.second, 6) << "\n";
    for (int k = 0; k < 2; k++) {
        const std::vector<FittsTrial> &list = (k == 0) ? warmupTrials_ : cooldownTrials_;
        const char *label = (k == 0) ? "warmup" : "cooldown";
        if (list.empty()) continue;
        f << "\n# " << label << " sequence: A_cm, W_cm, ID_bits, alpha\n";
        for (const auto &t : list)
            f << label << ": " << num(t.A_cm, 4) << ", " << num(t.W_cm, 6) << ", " << num(t.ID_bits, 4) << ", "
              << num(t.alpha, 4) << "\n";
    }
    return f.good();
}

/******************************************************************************
 * Protocol sequencing (Block 1)
 ******************************************************************************/
void M2FittsRobotHumanMachine::setCurrentTrial() {
    const std::vector<FittsTrial> &list = (phase_ == PHASE_WARMUP)     ? warmupTrials_
                                          : (phase_ == PHASE_COOLDOWN) ? cooldownTrials_
                                                                       : blockTrials_;
    if (phase_ != PHASE_DONE && trialIdx_ < list.size()) currentTrial_ = list[trialIdx_];
}

void M2FittsRobotHumanMachine::recordReach(const FittsTrialResult &r) {
    pending_ = r;
}

void M2FittsRobotHumanMachine::finaliseTrial(double returnTime, bool timedOut, bool aborted) {
    pending_.returnTime = returnTime;
    pending_.returnTimeout = timedOut;
    pending_.returnAbort = aborted;
    writeResultRow(pending_);
    results_.push_back(pending_);
    trialsDone_++;

    trialIdx_++;
    if (phase_ == PHASE_WARMUP) {
        if (trialIdx_ >= warmupTrials_.size()) {                //warm-up over: rest, then the block
            phase_ = PHASE_BLOCK;
            trialIdx_ = 0;
            breakDue_ = true;
            breakDuration_ = params_.warmupRestTime;
            spdlog::info("M2FittsRobotHuman: warm-up completed ({} trials). Rest (up to {} s), then Block {}.",
                         warmupTrials_.size(), params_.warmupRestTime, block_);
        }
    } else if (phase_ == PHASE_COOLDOWN) {
        if (trialIdx_ >= cooldownTrials_.size()) {
            phase_ = PHASE_DONE;
            spdlog::info("M2FittsRobotHuman: cool-down completed ({} trials).", cooldownTrials_.size());
        }
    } else if (phase_ == PHASE_BLOCK) {
        if (trialIdx_ >= blockTrials_.size()) {
            spdlog::info("M2FittsRobotHuman: Block {} completed ({} trials).", block_, blockTrials_.size());
            if (!cooldownTrials_.empty()) {                     //rest, then the cool-down sub-block
                phase_ = PHASE_COOLDOWN;
                trialIdx_ = 0;
                breakDue_ = true;
                breakDuration_ = params_.cooldownRestTime;
                spdlog::info("M2FittsRobotHuman: rest (up to {} s), then the cool-down ({} trials).",
                             params_.cooldownRestTime, cooldownTrials_.size());
            } else {
                phase_ = PHASE_DONE;
            }
        } else if (params_.trialsPerRound > 0 && trialIdx_ % (size_t)params_.trialsPerRound == 0) {
            //A break after every trials_per_round finished trials (Block 1's 45-trial rounds), whatever the file layout
            breakDue_ = true;
            breakDuration_ = params_.roundBreakTime;
            spdlog::info("M2FittsRobotHuman: {} trials done - break (up to {} s), then trials {}-{}.", trialIdx_,
                         params_.roundBreakTime, trialIdx_ + 1,
                         std::min(blockTrials_.size(), trialIdx_ + (size_t)params_.trialsPerRound));
        }
    }
    setCurrentTrial();
}

void M2FittsRobotHumanMachine::printSummary() {
    summaryPrinted_ = true;
    if (results_.empty()) return;
    struct PerID {
        int n = 0;
        double sumMT = 0.;
        double alpha = fittsNaN();
    };
    std::map<double, PerID> perID;  //ID -> successful block trials
    int nSuccess = 0, nBlock = 0;
    for (const auto &r : results_) {
        if (r.phase != PHASE_BLOCK) continue;
        nBlock++;
        PerID &e = perID[r.trial.ID_bits];
        e.alpha = r.alpha;
        if (!r.success || std::isnan(r.MT)) continue;
        nSuccess++;
        e.n++;
        e.sumMT += r.MT;
    }
    std::cout << "\n----------------- Block " << block_ << " summary (" << participant_ << ") -----------------\n"
              << nSuccess << "/" << nBlock << " trials validated\n"
              << "   ID (bits)   alpha       n     mean MT (s)\n";
    for (const auto &e : perID)
        std::cout << "   " << std::fixed << std::setprecision(4) << std::setw(8) << e.first
                  << std::setw(8) << std::setprecision(3) << e.second.alpha
                  << std::setw(8) << e.second.n
                  << std::setw(14) << std::setprecision(3)
                  << (e.second.n > 0 ? e.second.sumMT / e.second.n : fittsNaN()) << "\n";
    std::cout << "(MT excludes the " << params_.dwellTime << " s validation dwell)\n"
              << "-------------------------------------------------------------\n" << std::endl;
}

/******************************************************************************
 * UI and inputs (Block 1)
 ******************************************************************************/
std::vector<double> M2FittsRobotHumanMachine::sessionDescriptor() const {
    //Field order is part of the wire protocol (FittsProtocol.cs, SessionInfo): unchanged from Block 1
    return {(double)M2FITTS_PROTOCOL_VERSION,
            (double)block_,
            (double)params_.nRounds,
            (double)params_.trialsPerRound,
            (double)warmupTrials_.size(),  //shown by the display in its start-up instruction
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

void M2FittsRobotHumanMachine::sendUI(const std::string &cmd, const std::vector<double> &params) {
    ui.send(cmd, params);
}

void M2FittsRobotHumanMachine::sendUIContext(const std::string &cmd, const std::vector<double> &params) {
    ui.sendContext(cmd, params);
}

const char *M2FittsRobotHumanMachine::goSource() {
    if (robot()->keyboard->getS()) return "keyboard 's'";
    if (robot()->keyboard->getNb() == 1) return "keyboard '1'";
    //Press edge, not level: a held or stuck button 1 (Block 1 used isButtonPressed) cannot fire on every cycle
    if (robot()->joystick->isButtonTransition(1) > 0) return "joystick button 1";
    if (ui.consumeGo()) return "display (SPACE)";
    return nullptr;
}

bool M2FittsRobotHumanMachine::goSignal() {
    return goSource() != nullptr;
}

bool M2FittsRobotHumanMachine::abortSignal() {
    return robot()->keyboard->getX() || ui.consumeAbort();
}

void M2FittsRobotHumanMachine::raiseFault(const std::string &why) {
    if (faultRaised_) return;
    faultRaised_ = true;
    faultReason_ = why;
    spdlog::critical("M2FittsRobotHuman: safety fault - {}.", why);
}

/******************************************************************************
 * CORC interface
 ******************************************************************************/
void M2FittsRobotHumanMachine::init() {
    spdlog::debug("M2FittsRobotHumanMachine::init()");
    setupOk_ = false;

    if (alphaOverride_ > 1.) {
        spdlog::critical("M2FittsRobotHuman: alpha_override = {} is outside [0, 1]. Exiting...", alphaOverride_);
        std::raise(SIGTERM);
        return;
    }
    if (!loadRobotController()) {
        spdlog::critical("M2FittsRobotHuman: the robot PD (u_r) could not be loaded. Exiting...");
        std::raise(SIGTERM);
        return;
    }
    //Saturations that would let part of the participant's force through at alpha = 1 (the robot then stops short)
    if (blend_.cancelFMax > 0. && params_.reachForceLimit > 0. && blend_.cancelFMax < params_.reachForceLimit)
        spdlog::warn("M2FittsRobotHuman: cancel_f_max ({} N) < reach_force_limit ({} N): at alpha = 1 a participant force "
                     "between the two is only partly cancelled and the trial is not ended. Set cancel_f_max >= "
                     "reach_force_limit.",
                     blend_.cancelFMax, params_.reachForceLimit);
    if (blend_.cmdFMax > 0. && blend_.cmdFMax < pdGains_.f_max + blend_.cancelFMax + blend_.robotStictionComp)
        spdlog::warn("M2FittsRobotHuman: command_f_max ({} N) < f_max + cancel_f_max + stiction_comp ({} N): the command "
                     "cap can clip the blend.",
                     blend_.cmdFMax, pdGains_.f_max + blend_.cancelFMax + blend_.robotStictionComp);
    if (!loadTrialTable() || !buildAlphaTable() ||
        !loadSubBlock(warmupFile_, params_.warmupRepeats, warmupTrials_, "warm-up") ||
        !loadSubBlock(cooldownFile_, params_.cooldownRepeats, cooldownTrials_, "cool-down")) {
        spdlog::critical("M2FittsRobotHuman: trial/alpha tables could not be loaded ({}). Exiting...",
                         trialsFile_.empty() ? "trials_dir = " + trialsDir_ : "trials_file = " + trialsFile_);
        std::raise(SIGTERM);
        return;
    }
    if (!checkTrialTable()) {
        spdlog::critical("M2FittsRobotHuman: trial table incompatible with the workspace. Exiting...");
        std::raise(SIGTERM);
        return;
    }
    if (!openResultsFile()) {
        std::raise(SIGTERM);
        return;
    }
    if (!writeParametersFile(paramsPath_))
        spdlog::warn("M2FittsRobotHuman: could not write {}.", paramsPath_);
    else
        spdlog::info("M2FittsRobotHuman: parameters -> {}", paramsPath_);

    phase_ = warmupTrials_.empty() ? PHASE_BLOCK : PHASE_WARMUP;
    trialIdx_ = 0;
    setCurrentTrial();
    fhFilter_.setCutoff(humanForceFilterHz_);

    spdlog::info("M2FittsRobotHuman: participant {}, block {}: {} warm-up + {} block trials ({} rounds) + {} cool-down trials (shared control).",
                 participant_, block_, warmupTrials_.size(), blockTrials_.size(), params_.nRounds,
                 cooldownTrials_.size());
    {
        const size_t every = (size_t)std::max(1, params_.trialsPerRound);
        std::string at;
        for (size_t k = every; k < blockTrials_.size(); k += every) at += (at.empty() ? "" : ", ") + std::to_string(k);
        spdlog::info("M2FittsRobotHuman: breaks after trials {} ({} s each; a go ends one early, not before {} s).",
                     at.empty() ? std::string("none") : at, params_.roundBreakTime, params_.roundBreakMinTime);
    }
    spdlog::info("M2FittsRobotHuman: human force = {} x F_int, cancellation filter {} Hz, RobotM2 assist mirror g_p = {}.",
                 blend_.humanForceSign, humanForceFilterHz_, blend_.platformAssistGain);
    if (alphaOverride_ >= 0.)
        spdlog::warn("M2FittsRobotHuman: alpha_override = {}: EVERY trial uses this value and the csv alpha is ignored "
                     "(commissioning only). Output files are tagged _alphaOVR.",
                     alphaOverride_);

    if (robot()->initialise()) {
        logHelper.initLogger("M2FittsRobotHumanLog", rawPath_, LogFormat::CSV, true);
        //Block 1 columns, in the Block 1 order
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
        logHelper.add(logMarkCode_, "UIMarkCode");
        logHelper.add(logMarkTime_, "UIMarkClientTime (s)");
        logHelper.add(logMarkServerTime_, "UIMarkServerTime (s)");
        //Block 2 columns
        logHelper.add(logAlpha_, "Alpha");
        logHelper.add(logFpd_, "Fpd (N)");
        logHelper.add(logFh_, "FhumanEst (N)");
        logHelper.add(logFcmd_, "Fcmd (N)");
        logHelper.add(logBlendSat_, "BlendSaturated");
        //Not started here: StateMachine::activate() starts it. (Block 1 also started it in init(), which wrote
        //the header line twice.)

        //Same server as Block 1 (the M2 constructor registers the 7-value state stream the Unity client expects)
        UIserver = std::make_shared<FLNLHelper>(*robot(), bindIP_, serverPort_);
        ui.setClock(&runningTime());
        if (!ui.init(UIserver, uiDivider_, uiRequired_)) {
            spdlog::critical("M2FittsRobotHuman/UI: failed to initialise the UI link.");
            std::raise(SIGTERM);
            return;
        }
        ui.setSession(sessionDescriptor());
        setupOk_ = true;
    } else {
        spdlog::critical("Failed robot initialisation. Exiting...");
        std::raise(SIGTERM);
    }
}

void M2FittsRobotHumanMachine::end() {
    const bool wasRunning = running();
    //Exit the current state first, so that a reach interrupted by Ctrl-C is still written (reach_abort = 2)
    StateMachine::end();
    if (resultsFile_.is_open()) {
        resultsFile_.flush();
        resultsFile_.close();
    }
    if (!summaryPrinted_) printSummary();
    if (wasRunning) ui.close();
}

void M2FittsRobotHumanMachine::hwStateUpdate(void) {
    //Once per cycle. (Block 1 calls StateMachine::hwStateUpdate() twice: Keyboard::updateInput() clears the key
    //states on every call, so the second call erased the key read by the first and 's'/'x' were lost.)
    StateMachine::hwStateUpdate();

    const double t = runningTime();
    const double dt = (lastHwTime_ < 0.) ? 0. : t - lastHwTime_;
    lastHwTime_ = t;

    //Human force on the handle, robot frame: estimated every cycle, so that it is also logged outside the reach
    VM2 Fint = robot()->getInteractionForce();
    FhHat_ = blend_.humanForceSign * fhFilter_.update(Fint, dt);
    logFh_ = FhHat_;

    //Shared-control log values: NaN unless the reach state writes them during this cycle
    logFpd_ = VM2::Constant(fittsNaN());
    logFcmd_ = VM2::Constant(fittsNaN());
    logBlendSat_ = fittsNaN();
    logAlpha_ = appliedAlpha(currentTrial_);

    //Values streamed in the continuous log (Block 1)
    logPhase_ = (double)phase_;
    logTrialNb_ = (double)currentTrial_.index;
    logMarkCode_ = ui.markCode();
    logMarkTime_ = ui.markTime();
    logMarkServerTime_ = ui.markServerTime();

    //Keep the FLNL server accepting a (re)connection: non-blocking, restarts libFLNL's accepting thread if idle
    if (UIserver && (t - lastConnCheck_) >= 1.0) {
        lastConnCheck_ = t;
        if (!UIserver->isConnected()) UIserver->reconnect();
    }
    //Inbound commands, context replay and decimated state stream (Block 1)
    ui.update();
}