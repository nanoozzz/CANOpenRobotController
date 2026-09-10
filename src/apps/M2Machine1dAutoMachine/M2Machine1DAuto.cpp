#include "M2Machine1DAuto.h"

#include <csignal>
#include <iomanip>

/* ---------------------------------------------------------------- events -- */
static bool calibFinished(StateMachine &sm) {
    return sm.state<Calib1D>("Calib")->isCalibDone();
}

static bool homeReached(StateMachine &sm) {
    return sm.state<Home1D>("Home")->isReady();
}

static bool moveFinished(StateMachine &sm) {
    return sm.state<Move1D>("Move")->isDone();
}

//! Registered FIRST so it is evaluated before the loop-back transition.
static bool itiFinishedLast(StateMachine &sm) {
    auto &m = static_cast<M2Machine1DAuto &>(sm);
    return sm.state<ITI1D>("ITI")->isDone() && m.isExhausted();
}

static bool itiFinishedMore(StateMachine &sm) {
    auto &m = static_cast<M2Machine1DAuto &>(sm);
    return sm.state<ITI1D>("ITI")->isDone() && !m.isExhausted();
}

/* ----------------------------------------------------------- construction -- */
M2Machine1DAuto::M2Machine1DAuto() {
    setRobot(std::make_unique<RobotM2>("M2_MELB"));

    logPos   = Eigen::VectorXd::Zero(2);
    logVel   = Eigen::VectorXd::Zero(2);
    logForce = Eigen::VectorXd::Zero(2);

    addState("Calib", std::make_shared<Calib1D>(robot(), this));
    addState("Home",  std::make_shared<Home1D>(robot(), this));
    addState("Move",  std::make_shared<Move1D>(robot(), this));
    addState("ITI",   std::make_shared<ITI1D>(robot(), this));
    addState("End",   std::make_shared<End1D>(robot(), this));

    addTransition("Calib", &calibFinished, "Home");
    addTransition("Home",  &homeReached,   "Move");
    addTransition("Move",  &moveFinished,  "ITI");
    addTransition("ITI",   &itiFinishedLast, "End");   // must precede the loop-back
    addTransition("ITI",   &itiFinishedMore, "Home");

    setInitState("Calib");
}

M2Machine1DAuto::~M2Machine1DAuto() { closeRecords(); }

/* ------------------------------------------------------------------ init -- */
void M2Machine1DAuto::init() {
    spdlog::debug("M2Machine1DAuto::init()");

    if (!robot()->initialise()) {
        spdlog::critical("Failed robot initialisation. Exiting...");
        std::raise(SIGTERM);
        return;
    }

    const double home     = exp1d::HOME[exp1d::TASK_AXIS];
    const double axisMin  = exp1d::AXIS_MIN[exp1d::TASK_AXIS];
    const double axisMax  = exp1d::AXIS_MAX[exp1d::TASK_AXIS];

    if (!trials_.load(exp1d::TRIAL_FILE, exp1d::UNIT_SCALE, axisMin, axisMax,
                      home, exp1d::DIRECTION)) {
        spdlog::critical("Trial table invalid. Exiting before the robot moves.");
        std::raise(SIGTERM);
        return;
    }

    // Report the predicted MT-vs-ID slope so it can be checked against the data.
    const double wn = std::sqrt(exp1d::K_TASK / exp1d::M_EFF);
    spdlog::info("Controller: K={:.1f} N/m, B={:.2f} Ns/m, w_n={:.2f} rad/s (zeta=1).",
                 exp1d::K_TASK, exp1d::B_TASK, wn);
    spdlog::info("Predicted MT-vs-ID slope on the order of ln2/w_n = {:.0f} ms/bit.",
                 1000.0 * std::log(2.0) / wn);

    records_.open(exp1d::RECORD_FILE);
    if (!records_.is_open()) {
        spdlog::critical("Cannot open '{}'. Does the logs/ folder exist?", exp1d::RECORD_FILE);
        std::raise(SIGTERM);
        return;
    }
    records_ << std::fixed << std::setprecision(6);
    records_ << "trial,A,W,ID_file,ID_fitts,ID_shannon,target,MT,t_onset,peak_v,"
                "t_peak_v,final_error,max_off_axis,band_entries,sat_fraction,"
                "n_samples,timed_out\n";

    logHelper.initLogger("M2_1DAuto", "logs/M2_1DAuto_traj.csv", LogFormat::CSV, true);
    logHelper.add(runningTime(), "t");
    logHelper.add(logPhase,      "phase");
    logHelper.add(logTrialIdx,   "trial");
    logHelper.add(logTargetPos,  "target");
    logHelper.add(logW,          "W");
    logHelper.add(logPos,        "P");
    logHelper.add(logVel,        "V");
    logHelper.add(logForce,      "F");
    // NOTE: no startLogger() here. StateMachine::activate() starts it, and
    // calling both produces two "Starting logger" lines and a duplicate header.

    spdlog::info("M2Machine1DAuto ready: {} trials queued.", trials_.size());
}

/* ------------------------------------------------------- per-cycle update -- */
void M2Machine1DAuto::hwStateUpdate() {
    StateMachine::hwStateUpdate();

    logPos   = robot()->getEndEffPosition();
    logVel   = robot()->getEndEffVelocity();
    logForce = robot()->getInteractionForce();
    logTrialIdx = exhausted_ ? -1.0 : static_cast<double>(currentTrial().index);
}

/* ------------------------------------------------------------- sequencing -- */
void M2Machine1DAuto::advanceTrial() {
    if (trialIdx_ + 1 < trials_.size())
        trialIdx_++;
    else
        exhausted_ = true;
}

/* ---------------------------------------------------------------- records -- */
void M2Machine1DAuto::writeTrialRecord(const Move1D &mv) {
    if (!records_.is_open()) return;
    const Trial &t = currentTrial();
    const double satFraction =
        mv.nSamples > 0 ? static_cast<double>(mv.satSamples) / mv.nSamples : 0.0;

    records_ << t.index << ',' << t.A << ',' << t.W << ',' << t.IDfile << ','
             << t.IDfitts << ',' << t.IDshannon << ',' << mv.target << ','
             << mv.MT << ',' << mv.tOnset << ',' << mv.peakV << ','
             << mv.tPeakV << ',' << mv.finalError << ',' << mv.maxOffAxis << ','
             << mv.bandEntries << ',' << satFraction << ',' << mv.nSamples << ','
             << (mv.timedOut ? 1 : 0) << '\n';
    records_.flush();  // survive a mid-session abort

    if (mv.timedOut) nTimedOut_++;

    spdlog::info("Trial {:>3}/{}  ID={:.2f}  MT={:.3f}s  peakV={:.3f}m/s  sat={:.0f}%{}",
                 t.index, trials_.size(), t.IDfitts, mv.MT, mv.peakV,
                 100.0 * satFraction, mv.timedOut ? "  [TIMEOUT]" : "");
}

void M2Machine1DAuto::closeRecords() {
    if (records_.is_open()) {
        records_.close();
        spdlog::info("Records written to '{}' ({} timed out).", exp1d::RECORD_FILE, nTimedOut_);
    }
}

/* -------------------------------------------------------------------- end -- */
void M2Machine1DAuto::end() {
    closeRecords();
    StateMachine::end();
}