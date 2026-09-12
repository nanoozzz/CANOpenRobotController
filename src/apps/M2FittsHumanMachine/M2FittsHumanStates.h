/**
 * \file M2FittsHumanStates.h
 * \brief States of the M2FittsHumanMachine app: human (unassisted) Fitts' law
 *        reaching on the M2, Block 1 of the robot-guidance protocol.
 *
 * Trial cycle implemented here (one CORC state per phase of the cycle):
 *
 *   ReadyState  --go-->  ReachState  --trial over-->  ReturnState  --+--> ReachState   (next trial)
 *                             ^                                      |
 *                             |                                      +--> BreakState --> ReadyState
 *                             |                                      |
 *                             +--------------------------------------+--> EndState     (session over)
 *
 * All protocol data (trial table, indices, parameters, result logging) lives in
 * the M2FittsHumanMachine object; states only read/write it through the `sm` pointer.
 *
 * \version 0.1
 * \date 2026-09-12
 */

#ifndef M2FITTSHUMANSTATES_H_DEF
#define M2FITTSHUMANSTATES_H_DEF

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "RobotM2.h"
#include "State.h"

class M2FittsHumanMachine;  //!< Forward declaration: each state holds a pointer to its owner machine.

/** \brief Not-a-number shorthand used for measures that do not exist on a given trial. */
inline double fittsNaN() { return std::numeric_limits<double>::quiet_NaN(); }

/** \brief Session phase. Block 2 (robot support) is NOT implemented in this app. */
enum FittsPhase {
    PHASE_WARMUP = 0,  //!< 2 rounds x 5 IDs = 10 familiarisation reps
    PHASE_BLOCK = 1,   //!< Block 1: 4 rounds x 45 trials = 180 trials
    PHASE_DONE = 2     //!< Session complete
};

/** \brief Numeric state code streamed in the continuous (control-loop rate) log and to the UI. */
enum FittsStateCode {
    ST_CALIB = 0,
    ST_STANDBY = 1,
    ST_READY = 2,
    ST_REACH = 3,
    ST_RETURN = 4,
    ST_BREAK = 5,
    ST_END = 6
};

/**
 * \brief One Fitts condition/trial, as read from a bal_group_<n>.csv line.
 *
 * A and W are stored in cm (as in the csv) and converted to m (robot units) on demand,
 * so that what is logged is exactly what was read from the trial table.
 */
struct FittsTrial {
    int index = 0;       //!< Trial number as given in the csv 'index' column (1..180 across the block)
    int round = 0;       //!< Round (=csv group) the trial belongs to: 1..4 (0 for warm-up)
    int inRound = 0;     //!< Position of the trial within its round: 1..45 (1..5 for warm-up)
    double A_cm = 0.;    //!< Target amplitude [cm]
    double W_cm = 0.;    //!< Target width [cm]
    double ID_bits = 0.; //!< Index of difficulty [bits], as given in the csv

    double A() const { return A_cm / 100.; }      //!< Amplitude [m]
    double W() const { return W_cm / 100.; }      //!< Width [m]
    double halfW() const { return W_cm / 200.; }  //!< Half width (target tolerance) [m]
};

/**
 * \brief Everything measured on a single trial: one row of the results csv.
 *
 * MT follows the definition set in the protocol: time from target onset to the
 * *final* (i.e. dwell-completing) entry into the target, so that the 1 s dwell
 * used to validate the selection is excluded: MT = t_trial_end - t_onset - dwell.
 */
struct FittsTrialResult {
    int phase = PHASE_WARMUP;  //!< PHASE_WARMUP or PHASE_BLOCK
    FittsTrial trial;          //!< Condition of that trial

    double t_onset = 0.;               //!< Machine time at target onset [s] (to align with the continuous log)
    double t_end = 0.;                 //!< Machine time at the end of the validating dwell [s]
    double MT = fittsNaN();            //!< Final entry time re. target onset [s] -- the requested MT
    double RT = fittsNaN();            //!< Onset -> handle leaves the home region [s]
    double MT_move = fittsNaN();       //!< Home exit -> final entry [s] (MT without reaction time)
    double t_first_entry = fittsNaN(); //!< Onset -> first entry in target [s] (== MT if no re-entry)
    int nEntries = 0;                  //!< Number of target entries (1 = clean capture, >1 = dwell broken)
    double x_entry_cm = fittsNaN();    //!< Signed distance to target centre at final entry [cm] (for effective width)
    double x_sel_cm = fittsNaN();      //!< Signed distance to target centre at end of dwell [cm] (selection point)
    double v_peak = 0.;                //!< Peak speed along the task axis [m/s]
    bool success = false;              //!< true if the dwell was completed before the trial time-out

    double returnTime = fittsNaN();    //!< Duration of the participant-driven drag back to the origin [s]
    bool returnTimeout = false;        //!< true if the drag-back cap was reached and the robot returned on its own
    bool returnAbort = false;          //!< true if a robot-driven move was aborted on excessive interaction force
};

/**
 * \brief All tunable protocol/hardware parameters, with the values used for Block 1.
 *
 * Every field can be overriden at run time (without recompiling) through the optional
 * config file, see M2FittsHumanMachine::loadConfig(). Distances are in m, times in s.
 */
struct FittsParams {
    // --- Task geometry (robot frame; M2 workspace is x in [0, 0.625], y in [0, 0.440]) ---
    double originX = 0.40;        //!< Home/start position, x [m]
    double originY = 0.20;        //!< Home/start position, y [m]
    double taskDirection = -1;   //!< +1: targets at increasing x; -1: decreasing x
    bool useYChannel = true;      //!< Virtual channel constraining off-axis (y) motion: makes the task 1D.
                                  //!< This is a task constraint, NOT assistance: it must be identical in Blocks 1 and 2.
    double channelK = 800.;       //!< Channel stiffness [N/m]
    double channelD = 10.;        //!< Channel damping [N/(m/s)]

    // --- Trial (reaching phase) ---
    double dwellTime = 1.0;       //!< Time the cursor must stay inside the target to validate the trial [s]
    double homeExitRadius = 0.005;//!< Distance from origin beyond which the movement is considered started [m]
    double maxTrialTime = 10.0;   //!< Trial time-out [s]: trial is aborted (success=0) and the cycle continues

    // --- Post-trial return to the origin (protocol note of the flow chart) ---
    double returnOffset = 0.03;   //!< The robot drives the handle to this distance from the origin [m]
    double returnSpeed = 0.20;    //!< Mean speed of the robot-driven move back [m/s]
    double returnMinTime = 0.8;   //!< Minimum duration of the robot-driven move back [s]
    double originTolerance = 0.005;//!< The participant must bring the handle within this distance of the origin [m]
    double maxReturnTime = 5.0;   //!< Participant-chosen inter-trial rest cap [s]: after this the robot returns on its own
    double originSnapTime = 0.4;  //!< Duration of the final robot-driven move onto the exact origin [s]
    double originHoldTime = 0.25; //!< Hold at the origin before the next target onset [s]

    // --- Breaks ---
    double roundBreakTime = 60.;  //!< Break after each round of 45 trials [s] (participant can end it early)
    double warmupRestTime = 60.;  //!< Rest between warm-up and Block 1 [s]
    double readyHoldTime = 2.0;   //!< Hold at origin before auto-starting the next round [s]

    // --- Control / safety ---
    double kPosVel = 1.0;         //!< Proportional gain of the position-over-velocity loop [1/s]
    double forceLimit = 30.;      //!< Interaction force above which a robot-driven move is aborted [N]

    // --- Structure ---
    int nRounds = 4;              //!< Number of rounds (csv groups) in the block
    int trialsPerRound = 45;      //!< Expected number of trials per round (checked at load)
    int warmupRepeats = 2;        //!< Warm-up rounds (each covering the 5 IDs once)
};

/**
 * \brief Base class of all M2FittsHuman states: gives access to the robot and to the owner machine.
 *
 * Note: unlike M2DemoStates, no banner is printed on entry/exit (the trial cycle enters
 * and exits states ~540 times per session; CORC already logs transitions through spdlog).
 */
class M2FittsState : public State {
   protected:
    RobotM2 *robot;           //!< Pointer to the state machine's robot object
    M2FittsHumanMachine *sm;  //!< Pointer to the owner state machine (protocol data, UI, results log)

    M2FittsState(RobotM2 *M2, M2FittsHumanMachine *machine, const char *name = "") : State(name), robot(M2), sm(machine){};

   private:
    void entry(void) final { entryCode(); };
    void during(void) final { duringCode(); };
    void exit(void) final { exitCode(); };

   public:
    virtual void entryCode(){};
    virtual void duringCode(){};
    virtual void exitCode(){};
};

/**
 * \brief Absolute position calibration: pushes against the bottom-left stops at constant torque.
 *        Same procedure as M2DemoMachine (required before any position-referenced task).
 */
class M2FittsCalibState : public M2FittsState {
   public:
    M2FittsCalibState(RobotM2 *M2, M2FittsHumanMachine *machine, const char *name = "M2 Fitts Calibration") : M2FittsState(M2, machine, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);

    bool isCalibDone() { return calibDone; }

   private:
    VM2 stop_reached_time;
    bool at_stop[2];
    bool calibDone = false;
};

/**
 * \brief Transparent standby: mass/friction compensated, no task. Entered on abort (key 'x') and after the session.
 */
class M2FittsStandbyState : public M2FittsState {
   public:
    M2FittsStandbyState(RobotM2 *M2, M2FittsHumanMachine *machine, const char *name = "M2 Fitts Standby") : M2FittsState(M2, machine, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);
};

/**
 * \brief Brings the handle to the origin and waits for the go signal.
 *
 * Used once at the start of the session (explicit go required: key 's', joystick button 1
 * or UI command) and after every break (auto-start once the handle has been held at the
 * origin for readyHoldTime, so the session flows without experimenter input).
 */
class M2FittsReadyState : public M2FittsState {
   public:
    M2FittsReadyState(RobotM2 *M2, M2FittsHumanMachine *machine, const char *name = "M2 Fitts Ready") : M2FittsState(M2, machine, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);

    bool isReady() { return ready_; }

   private:
    VM2 Xi_;               //!< Position at state entry
    VM2 Xorigin_;          //!< Origin (home) position
    double T_ = 1.;        //!< Duration of the min-jerk move to the origin [s]
    bool requireGo_ = true;//!< If true, an explicit go signal is required (first start of the session)
    bool atOrigin_ = false;
    double tAtOrigin_ = 0.;
    bool ready_ = false;
};

/**
 * \brief One reaching trial: target onset, movement, dwell-validated capture.
 *
 * The robot is transparent (mass + friction compensation, zero assistance) -- this is the
 * "no robot support" condition of Block 1. If useYChannel is set, a PD virtual channel holds
 * y at the origin so that the movement is one-dimensional along the task axis.
 */
class M2FittsReachState : public M2FittsState {
   public:
    M2FittsReachState(RobotM2 *M2, M2FittsHumanMachine *machine, const char *name = "M2 Fitts Reach") : M2FittsState(M2, machine, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);

    bool isTrialDone() { return trialDone_; }

   private:
    FittsTrial trial_;          //!< Condition of the current trial
    VM2 Xorigin_, Xtarget_;     //!< Home and target positions
    double halfW_ = 0.;         //!< Target tolerance [m]
    bool trialDone_ = false;

    bool leftHome_ = false;     //!< Movement started?
    bool inTarget_ = false;     //!< Cursor inside target at the previous iteration?
    double tEntry_ = 0.;        //!< Time of the current (possibly final) entry into the target [s]
    double xEntry_ = 0.;        //!< Signed error at the current entry [m]
    FittsTrialResult res_;      //!< Accumulated trial measures
};

/**
 * \brief Post-trial return to the origin, in four phases (see the protocol note of the flow chart):
 *
 *  1. MOVE_AWAY: the robot drives the handle to returnOffset (3 cm) from the origin (min-jerk).
 *  2. DRAG:      transparent; the participant manually drags the handle to within originTolerance
 *                (0.5 cm) of the origin. This is the self-paced inter-trial rest, capped at maxReturnTime (5 s).
 *  3. SNAP:      the robot positions the handle on the exact origin (min-jerk).
 *  4. HOLD:      the handle is held on the origin for originHoldTime before the next target onset.
 *
 *  Robot-driven phases are aborted (-> DRAG) if the interaction force exceeds forceLimit.
 */
class M2FittsReturnState : public M2FittsState {
   public:
    M2FittsReturnState(RobotM2 *M2, M2FittsHumanMachine *machine, const char *name = "M2 Fitts Return") : M2FittsState(M2, machine, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);

    bool isReturnDone() { return returnDone_; }

   private:
    enum ReturnPhase { MOVE_AWAY = 0, DRAG = 1, SNAP = 2, HOLD = 3 };

    void gotoPhase(ReturnPhase p);

    ReturnPhase phase_ = MOVE_AWAY;
    double tPhase_ = 0.;      //!< State time at which the current phase started [s]
    VM2 Xi_;                  //!< Position at the start of the current robot-driven move
    VM2 Xaway_, Xorigin_;     //!< Off-origin waiting position and origin
    double T_ = 1.;           //!< Duration of the current robot-driven move [s]
    double dragTime_ = 0.;    //!< Measured duration of the participant-driven drag back [s]
    int snapRetries_ = 0;     //!< Number of aborted final repositioning attempts (bounded to avoid a loop)
    bool timedOut_ = false;
    bool aborted_ = false;
    bool returnDone_ = false;
};

/**
 * \brief Break between rounds (and rest between warm-up and Block 1). Transparent; ends on
 *        time-out or early on a go signal. Duration is set by the machine before entry.
 */
class M2FittsBreakState : public M2FittsState {
   public:
    M2FittsBreakState(RobotM2 *M2, M2FittsHumanMachine *machine, const char *name = "M2 Fitts Break") : M2FittsState(M2, machine, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);

    bool isBreakOver() { return over_; }

   private:
    double duration_ = 60.;
    bool over_ = false;
};

/**
 * \brief End of Block 1: closes the results file, prints a per-ID summary, robot left transparent.
 */
class M2FittsEndState : public M2FittsState {
   public:
    M2FittsEndState(RobotM2 *M2, M2FittsHumanMachine *machine, const char *name = "M2 Fitts End") : M2FittsState(M2, machine, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);
};

/**
 * \brief Minimum jerk interpolation between X0 and Xf over T, evaluated at t.
 *
 * Same helper as M2DemoStates, with the velocity expression differentiated w.r.t. T rather
 * than t: mathematically identical but defined at t=0 (the demo version returns NaN there).
 * \return normalised time (1 when the movement is complete)
 */
double JerkIt(VM2 X0, VM2 Xf, double T, double t, VM2 &Xd, VM2 &dXd);

#endif  // M2FITTSHUMANSTATES_H_DEF