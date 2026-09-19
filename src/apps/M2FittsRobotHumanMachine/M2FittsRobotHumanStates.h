/**
 * \file M2FittsRobotHumanStates.h
 * \brief States of the M2FittsRobotHumanMachine app: Block 2 (human-robot shared control) of the
 *        robot-guidance Fitts' law protocol on the M2.
 *
 * Trial cycle - identical to Block 1 (M2FittsHumanStates.h); only the Reach state differs:
 *
 *   Calib -> WaitUI -> ReadyState --go--> ReachState --trial over--> ReturnState --+--> ReachState  (next trial)
 *                           ^                                                      |
 *                           |                                                      +--> BreakState --> ReadyState
 *                           |                                                      |
 *                           +------------------------------------------------------+--> EndState    (session over)
 *
 *   any state --'x' / UI ABRT------------------------> StandbyState (transparent; session stopped)
 *   any state --safety fault raised in the reach-----> FaultState   (damping brake; session stopped)
 *
 * In the Reach state the robot applies, on the task axis,
 *      F_cmd,x = alpha * F_pd,x - alpha * k_h * F_h,x            (FittsSharedControl.h)
 * with alpha read per trial from the trial csv. Display protocol, timing, detection and every Block 1
 * measure are unchanged; force-sharing measures are added. Block 2 has no warm-up: the session starts
 * with round 1 after the go signal.
 *
 * \version 1.0
 * \date 2026-09-18
 */

#ifndef M2FITTSROBOTHUMANSTATES_H_DEF
#define M2FITTSROBOTHUMANSTATES_H_DEF

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "FittsSharedControl.h"
#include "RobotM2.h"
#include "State.h"

class M2FittsRobotHumanMachine;  //!< Forward declaration: each state holds a pointer to its owner machine.

/** \brief Not-a-number shorthand used for measures that do not exist on a given trial. */
inline double fittsNaN() { return std::numeric_limits<double>::quiet_NaN(); }

/** \brief Session phase. */
enum FittsPhase {
    PHASE_WARMUP = 0,  //!< Not used in Block 2 (no warm-up); kept because the value is part of the display protocol
    PHASE_BLOCK = 1,   //!< Block 2: nRounds x trialsPerRound trials
    PHASE_DONE = 2     //!< Session complete
};

/** \brief State code of the continuous log. 0-7 as in Block 1 (FittsStateCode in the Unity client).
 *  ST_FAULT exists only in this app and only in the raw log: it is never streamed to the display. */
enum FittsStateCode {
    ST_CALIB = 0,
    ST_STANDBY = 1,
    ST_READY = 2,
    ST_REACH = 3,
    ST_RETURN = 4,
    ST_BREAK = 5,
    ST_END = 6,
    ST_WAITUI = 7,
    ST_FAULT = 9
};

/** \brief Value of the reach_abort results column. */
enum ReachAbortCode {
    REACH_OK = 0,           //!< Reach ended normally (validated capture or time-out)
    REACH_FORCE_LIMIT = 1,  //!< Interaction force above reach_force_limit: robot released, trial scored as a miss
    REACH_INTERRUPTED = 2   //!< Session stopped during the reach (abort key/UI, safety fault or Ctrl-C)
};

/**
 * \brief One Fitts condition/trial, as read from a <trials_prefix><n>.csv line: the Block 1 columns plus alpha.
 *
 * A and W are stored in cm (as in the csv) and converted to m (robot units) on demand.
 */
struct FittsTrial {
    int index = 0;              //!< Trial number as given in the csv 'index' column
    int round = 0;              //!< Round (= csv group) the trial belongs to
    int inRound = 0;            //!< Position of the trial within its round
    double A_cm = 0.;           //!< Target amplitude [cm]
    double W_cm = 0.;           //!< Target width [cm]
    double ID_bits = 0.;        //!< Index of difficulty [bits], as given in the csv
    double alpha = fittsNaN();  //!< Autonomy level in [0, 1] ('alpha' csv column)

    double A() const { return A_cm / 100.; }      //!< Amplitude [m]
    double W() const { return W_cm / 100.; }      //!< Width [m]
    double halfW() const { return W_cm / 200.; }  //!< Half width (target tolerance) [m]
};

/**
 * \brief Everything measured on one trial: one row of the results csv.
 *
 * Block 1 measures keep their Block 1 definitions (MT = final entry time re. target onset, dwell excluded).
 * Force-sharing measures cover [onset, final entry] (the MT window) on validated trials and the whole reach
 * otherwise; "towards the targets" is the task direction.
 */
struct FittsTrialResult {
    int phase = PHASE_BLOCK;   //!< Always PHASE_BLOCK in Block 2
    FittsTrial trial;          //!< Condition of that trial

    double t_onset = 0.;               //!< Machine time at target onset [s]
    double t_end = 0.;                 //!< Machine time at the end of the trial [s]
    double MT = fittsNaN();            //!< Final entry time re. target onset [s]
    double RT = fittsNaN();            //!< Onset -> handle leaves the home region [s]
    double MT_move = fittsNaN();       //!< Home exit -> final entry [s]
    double t_first_entry = fittsNaN(); //!< Onset -> first entry in target [s]
    int nEntries = 0;                  //!< Number of target entries
    double x_entry_cm = fittsNaN();    //!< Signed distance to target centre at final entry [cm]
    double x_sel_cm = fittsNaN();      //!< Signed distance to target centre at end of dwell [cm]
    double v_peak = 0.;                //!< Peak speed along the task axis [m/s]
    bool success = false;              //!< true if the dwell was completed before the time-out
    double x_start_cm = fittsNaN();    //!< Signed distance to the origin at target onset [cm]

    double returnTime = fittsNaN();    //!< Duration of the participant-driven drag back to the origin [s]
    bool returnTimeout = false;
    bool returnAbort = false;

    //--- Block 2
    double alpha = fittsNaN();         //!< Autonomy level applied on this trial
    double impH = fittsNaN();          //!< Impulse of the measured human force towards the targets [N.s]
    double impR = fittsNaN();          //!< Impulse of the robot term alpha * F_pd towards the targets [N.s]
    double FhPeak = fittsNaN();        //!< Peak |human force| along the task axis [N]
    double conflictFrac = fittsNaN();  //!< Share of samples with human and robot pushing in opposite directions
    double satFrac = fittsNaN();       //!< Share of samples on which the blend was not realised (saturation)
    int reachAbort = REACH_OK;         //!< See ReachAbortCode
};

/** \brief Running sums behind the force-sharing measures. */
struct SharedAccum {
    double impH = 0.;
    double impR = 0.;
    double FhPeak = 0.;
    long n = 0;
    long nConflict = 0;
    long nSat = 0;
};

/**
 * \brief Tunable protocol/hardware parameters. Block 1 fields keep the Block 1 meaning and defaults
 *        (M2FittsHumanStates.h); the Block 2 fields concern the safety of the shared reach.
 *        Every field can be overridden in the config file (see M2FittsRobotHumanMachine::applyConfig()).
 */
struct FittsParams {
    // --- Task geometry (robot frame; M2 workspace is x in [0, 0.625], y in [0, 0.440]) ---
    double originX = 0.40;          //!< Home/start position, x [m]
    double originY = 0.20;          //!< Home/start position, y [m]
    double taskDirection = -1;      //!< +1: targets at increasing x; -1: decreasing x
    bool useYChannel = true;        //!< Virtual channel on y (task constraint, identical in Blocks 1 and 2)
    double channelK = 800.;         //!< Channel stiffness [N/m]
    double channelD = 10.;          //!< Channel damping [N/(m/s)]
    double channelFMax = 30.;       //!< Channel force saturation [N] (see channel_saturate)

    // --- Trial (reaching phase) ---
    double dwellTime = 1.0;         //!< Dwell validating the capture [s]
    double homeExitRadius = 0.005;  //!< Movement onset threshold [m]
    double maxTrialTime = 15.0;     //!< Trial time-out [s]
    double successHoldTime = 0.5;   //!< Hold at the target before the return starts [s]

    // --- Post-trial return to the origin ---
    double returnOffset = 0.03;
    double returnSpeed = 0.20;
    double returnMinTime = 0.8;
    double originTolerance = 0.005;
    double maxReturnTime = 5.0;
    double originSnapTime = 0.4;
    double originHoldTime = 0.25;
    double originSettleSpeed = 0.03;
    double originSettleTime = 0.15;
    double forceLimitTime = 0.10;   //!< |F| must exceed a force limit continuously for this long [s] (also used in the reach)
    double forceGraceTime = 0.15;

    // --- Breaks ---
    double roundBreakTime = 60.;
    double readyHoldTime = 2.0;

    // --- Control / safety of robot-driven moves ---
    double kPosVel = 1.0;
    double forceLimit = 30.;
    double kHold = 8.;
    double awayTolerance = 0.002;
    double awaySettleSpeed = 0.02;
    double maxMoveExtraTime = 1.0;

    // --- Structure ---
    int nRounds = 4;
    int trialsPerRound = 45;

    // --- Block 2: safety of the shared reach ---
    double reachForceLimit = 40.;      //!< |interaction force| above this for forceLimitTime ends the reach [N]; 0 = off
    double maxSpeed = 1.5;             //!< Speed above this during the shared reach -> FaultState [m/s]
    double workspaceTolerance = 0.01;  //!< Excursion beyond the M2 travel during the shared reach -> FaultState [m]
    double brakeDamping = 40.;         //!< FaultState damping [N/(m/s)]
    double brakeFMax = 30.;            //!< FaultState force saturation [N]
    double conflictThreshold = 1.0;    //!< Force above which a push counts in conflict_frac [N]
};

/**
 * \brief Base class of all states: gives access to the robot and to the owner machine.
 */
class M2FittsState : public State {
   protected:
    RobotM2 *robot;                //!< Pointer to the state machine's robot object
    M2FittsRobotHumanMachine *sm;  //!< Pointer to the owner state machine (protocol data, UI, results)

    M2FittsState(RobotM2 *M2, M2FittsRobotHumanMachine *machine, const char *name = "") : State(name), robot(M2), sm(machine){};

   private:
    void entry(void) final { entryCode(); };
    void during(void) final { duringCode(); };
    void exit(void) final { exitCode(); };

   public:
    virtual void entryCode(){};
    virtual void duringCode(){};
    virtual void exitCode(){};
};

/** \brief Absolute position calibration against the stops (Block 1). Does nothing if the setup failed. */
class M2FittsCalibState : public M2FittsState {
   public:
    M2FittsCalibState(RobotM2 *M2, M2FittsRobotHumanMachine *machine, const char *name = "M2 Fitts Calibration") : M2FittsState(M2, machine, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);

    bool isCalibDone() { return calibDone; }

   private:
    VM2 stop_reached_time;
    bool at_stop[2];
    bool calibDone = false;
};

/** \brief Transparent hold until the Unity client has connected and completed the handshake (Block 1). */
class M2FittsWaitUIState : public M2FittsState {
   public:
    M2FittsWaitUIState(RobotM2 *M2, M2FittsRobotHumanMachine *machine, const char *name = "M2 Fitts Wait UI") : M2FittsState(M2, machine, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);

    bool isUIReady() { return ready_; }

   private:
    bool ready_ = false;
    bool announced_ = false;
};

/** \brief Transparent standby after a manual abort (Block 1). Terminal. */
class M2FittsStandbyState : public M2FittsState {
   public:
    M2FittsStandbyState(RobotM2 *M2, M2FittsRobotHumanMachine *machine, const char *name = "M2 Fitts Standby") : M2FittsState(M2, machine, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);
};

/** \brief Brings the handle to the origin and waits for the go signal (Block 1). */
class M2FittsReadyState : public M2FittsState {
   public:
    M2FittsReadyState(RobotM2 *M2, M2FittsRobotHumanMachine *machine, const char *name = "M2 Fitts Ready") : M2FittsState(M2, machine, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);

    bool isReady() { return ready_; }

   private:
    VM2 Xi_;
    VM2 Xorigin_;
    double T_ = 1.;
    bool requireGo_ = true;
    bool atOrigin_ = false;
    double tAtOrigin_ = 0.;
    bool ready_ = false;
};

/**
 * \brief One shared-control reaching trial: target onset, shared movement, dwell-validated capture.
 *
 * Target onset, entry/dwell detection, MT and every Block 1 measure are those of Block 1. The robot applies
 * F_cmd,x = alpha * F_pd,x - alpha * k_h * F_h,x (PD set-point stepped to the target centre at onset, as in
 * M2FittsMachine) and the Block 1 channel on y. The reach ends early, scored as a miss, if the interaction
 * force stays above reach_force_limit; a safety fault (non-finite reading, overspeed, workspace) hands over
 * to FaultState.
 */
class M2FittsSharedReachState : public M2FittsState {
   public:
    M2FittsSharedReachState(RobotM2 *M2, M2FittsRobotHumanMachine *machine, const char *name = "M2 Fitts Shared Reach") : M2FittsState(M2, machine, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);

    bool isTrialDone() { return trialDone_; }

   private:
    bool applySharedControl(double dt);             //!< Computes and sends the blended command; false if a fault was raised
    void releaseRobot();                            //!< Block 1 command (transparent + channel), once the robot must stop acting
    bool safetyOk(const VM2 &X, const VM2 &dX);     //!< Raises a machine fault and returns false if the reach is unsafe
    void accumulate(double dt);                     //!< Adds this sample to the force-sharing sums
    void storeMeasures(const SharedAccum &a);       //!< Copies the force-sharing measures into the result

    FittsTrial trial_;
    double alpha_ = 0.;
    VM2 Xorigin_, Xtarget_, Xref_;
    double halfW_ = 0.;
    bool trialDone_ = false;

    bool leftHome_ = false;
    bool inTarget_ = false;
    double tEntry_ = 0.;
    double xEntry_ = 0.;
    double overForceTime_ = 0.;
    FittsTrialResult res_;

    fsc::SharedControlLaw law_;
    fsc::BlendOutput out_;
    SharedAccum acc_;         //!< Sums since onset
    SharedAccum accAtEntry_;  //!< Sums at the latest target entry (MT window if that entry is the final one)
};

/** \brief Post-trial return to the origin (Block 1): CONFIRM, MOVE_AWAY, DRAG, SNAP, HOLD. */
class M2FittsReturnState : public M2FittsState {
   public:
    M2FittsReturnState(RobotM2 *M2, M2FittsRobotHumanMachine *machine, const char *name = "M2 Fitts Return") : M2FittsState(M2, machine, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);

    bool isReturnDone() { return returnDone_; }

   private:
    enum ReturnPhase { CONFIRM = 0, MOVE_AWAY = 1, DRAG = 2, SNAP = 3, HOLD = 4 };

    void gotoPhase(ReturnPhase p);
    bool forceAbort(double t);

    ReturnPhase phase_ = MOVE_AWAY;
    double tPhase_ = 0.;
    VM2 Xi_;
    VM2 Xaway_, Xorigin_;
    double T_ = 1.;
    double dragTime_ = 0.;
    int snapRetries_ = 0;
    double overForceTime_ = 0.;
    bool abortedLatched_ = false;
    bool timedOut_ = false;
    bool aborted_ = false;
    bool returnDone_ = false;
    double settleTime_ = 0.;
};

/** \brief Break between rounds (Block 1). */
class M2FittsBreakState : public M2FittsState {
   public:
    M2FittsBreakState(RobotM2 *M2, M2FittsRobotHumanMachine *machine, const char *name = "M2 Fitts Break") : M2FittsState(M2, machine, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);

    bool isBreakOver() { return over_; }

   private:
    double duration_ = 60.;
    bool over_ = false;
};

/** \brief End of the block (Block 1): robot transparent, summary printed. */
class M2FittsEndState : public M2FittsState {
   public:
    M2FittsEndState(RobotM2 *M2, M2FittsRobotHumanMachine *machine, const char *name = "M2 Fitts End") : M2FittsState(M2, machine, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);
};

/** \brief Safety fault raised during the shared reach: damping brake until the app is stopped. Terminal. */
class M2FittsFaultState : public M2FittsState {
   public:
    M2FittsFaultState(RobotM2 *M2, M2FittsRobotHumanMachine *machine, const char *name = "M2 Fitts Fault") : M2FittsState(M2, machine, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);

   private:
    void brake();
};

/**
 * \brief Minimum jerk interpolation between X0 and Xf over T, evaluated at t.
 * \return normalised time (1 when the movement is complete)
 */
double JerkIt(VM2 X0, VM2 Xf, double T, double t, VM2 &Xd, VM2 &dXd);

#endif  // M2FITTSROBOTHUMANSTATES_H_DEF