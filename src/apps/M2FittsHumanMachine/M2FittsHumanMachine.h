/**
 * \file M2FittsHumanMachine.h
 * \brief Block 1 (no robot support) of the robot-guidance Fitts' law experiment on the M2,
 *        with the Unity front-end driven over libFLNL.
 *
 * The app runs, for one participant:
 *      Wait for UI -> Warm-up (warmupRepeats x 5 IDs) -> Rest -> Block 1 (4 rounds x 45 trials) -> End
 * and writes one row per trial (trial #, A, W, ID, MT, ...) to a results csv, plus the usual
 * CORC control-loop-rate log of position/velocity/force.
 *
 * -----------------------------------------------------------------------------------------
 * AUTHORITY
 * -----------------------------------------------------------------------------------------
 * CORC is the sole authority on the protocol AND on every measurement: trial order, target
 * onset, entry/dwell detection, movement time, the 1D channel and all robot-driven motion are
 * decided here, on the control loop. Unity renders what it is told and provides the participant
 * interface; it performs no detection and no timing that enters the analysis. Any measurement
 * made on the Unity side is sampled at frame rate (60-144 Hz, with vsync jitter) and carries
 * display latency, which would bias movement time in a way that scales with the index of
 * difficulty - i.e. it would contaminate the very slope the experiment estimates.
 *
 * -----------------------------------------------------------------------------------------
 * UNITY COMMUNICATION (libFLNL, see M2FittsUILink)
 * -----------------------------------------------------------------------------------------
 * Continuous state stream, 13 doubles in this fixed order (registered in init()):
 *      [t, x, y, dx, dy, Fx, Fy, stateCode, phase, trialIndex, targetX, halfWidth, dwellProgress]
 * `t` is the state machine running time - the SAME clock as t_onset/t_end in the results csv
 * and as the Time column of the raw log, so display frames and kinematics can be aligned.
 *
 * Commands sent to the UI (4 characters + double parameters):
 *   "SESS" [version, block, nRounds, trialsPerRound, nWarmup, originX, originY, taskDirection,
 *           dwellTime, maxTrialTime, originTolerance, returnOffset, useYChannel, displayGain]
 *   "TRIA" [phase, round, trialInRound, trialIndex, A_cm, W_cm, ID, targetX, targetY,
 *           halfWidth, originX, originY, dwellTime, maxTrialTime]   target onset
 *   "HITT" [trialIndex, MT, nEntries, x_sel_cm]     trial validated (dwell completed)
 *   "MISS" [trialIndex]                             trial timed out
 *   "RETN" [awayX, awayY]                           robot is driving the handle off the origin
 *   "DRAG" [originX, originY, tol, maxTime]         participant must drag the handle back
 *   "ORIG" [originX, originY]                       handle on the origin, next trial imminent
 *   "REST" [duration, trialsDone, phase]            break started
 *   "RDY!" [requireGo, originX, originY]            waiting at the origin for the go signal
 *   "WAIT" []                                       CORC is waiting for the client
 *   "ENDE" [nTrials]                                end of Block 1
 *   "PONG" [clientStamp, serverTime]                reply to PING
 *   "VERR" [serverVersion]                          protocol version mismatch
 *   "OK"                                            acknowledgement
 *
 * Commands accepted from the UI:
 *   "HELO" [version, clientStamp]   handshake; triggers SESS + replay of the current context
 *   "GTNS" / "STRT" / "SKIP"        go / skip the current wait
 *   "ABRT"                          abort to transparent standby
 *   "PING" [clientStamp]            latency and clock-offset measurement
 *   "RTTR" [rtt_ms]                 client-measured round trip, logged for the record
 *   "MARK" [code, clientTime]       display-side event marker, written into the raw log
 *
 * Keyboard: 's' (or joystick button 1) = go/skip, 'x' = abort to transparent standby.
 *
 * \version 1.0
 * \date 2026-09-15
 */

#ifndef M2FITTSHUMANMACHINE_H
#define M2FITTSHUMANMACHINE_H

#include <fstream>
#include <map>
#include <string>
#include <vector>

#include "M2FittsUILink.h"
#include "RobotM2.h"
#include "StateMachine.h"

#include "M2FittsHumanStates.h"

/**
 * \brief State machine of the human (unassisted) Fitts' task, Block 1.
 *
 * Holds the protocol: trial table, current phase/round/trial, break scheduling and the
 * trial-by-trial results file. States read it through their `sm` pointer.
 */
class M2FittsHumanMachine : public StateMachine {
   public:
    M2FittsHumanMachine();
    ~M2FittsHumanMachine();

    void init();
    void end();
    void hwStateUpdate();

    RobotM2 *robot() { return static_cast<RobotM2 *>(_robot.get()); }  //!< Robot getter with specialised type

    //---------------------------------------------------------------- protocol
    const FittsParams &p() const { return params_; }                  //!< Protocol parameters
    FittsPhase phase() const { return phase_; }                       //!< Current phase (warm-up / block / done)
    const FittsTrial &currentTrial() const { return currentTrial_; }  //!< Condition of the trial about to be (or being) performed
    bool sessionFinished() const { return phase_ == PHASE_DONE; }
    bool breakDue() const { return breakDue_; }                       //!< A break is owed before the next trial
    double breakDuration() const { return breakDuration_; }
    void clearBreakDue() { breakDue_ = false; }
    bool requireGoSignal() const { return trialsDone_ == 0; }         //!< Explicit go needed only for the very first trial
    int trialsDone() const { return trialsDone_; }
    int blockNb() const { return block_; }
    size_t nWarmupTrials() const { return warmupTrials_.size(); }
    size_t nBlockTrials() const { return blockTrials_.size(); }
    const std::string &participant() const { return participant_; }

    VM2 originPosition() const { return VM2(params_.originX, params_.originY); }
    VM2 targetPosition(const FittsTrial &t) const {
        return VM2(params_.originX + params_.taskDirection * t.A(), params_.originY);
    }
    VM2 returnPosition() const {
        return VM2(params_.originX + params_.taskDirection * params_.returnOffset, params_.originY);
    }
    //! Signed coordinate along the task axis, positive towards the targets.
    double taskCoord(const VX &X) const { return params_.taskDirection * X(0); }
    double taskCoord(const VM2 &X) const { return params_.taskDirection * X(0); }

    void recordReach(const FittsTrialResult &r);  //!< Called by the reach state when a trial ends (success or time-out)
    void finaliseTrial(double returnTime, bool timedOut, bool aborted);  //!< Called by the return state: writes the row, advances the trial counter

    //---------------------------------------------------------------- UI / inputs
    //! Fire-and-forget UI event.
    void sendUI(const std::string &cmd, const std::vector<double> &params = {});
    //! UI event that also defines what should be on screen; replayed on (re)connection.
    void sendUIContext(const std::string &cmd, const std::vector<double> &params = {});

    bool goSignal();     //!< Go/skip requested (UI command, keyboard 's'/'1' or joystick button 1)?
    bool abortSignal();  //!< Abort requested (keyboard 'x' or UI ABRT)?

    //! Is the client requirement satisfied (either not required, or connected and handshaked)?
    bool uiReady() const { return !ui.required() || (ui.connected() && ui.handshakeDone()); }

    //---------------------------------------------------------------- continuous log
    void setLoggedState(double stateCode) { logState_ = stateCode; }
    void setLoggedTarget(double targetX, double halfWidth) { logTargetX_ = targetX; logHalfWidth_ = halfWidth; }
    void setLoggedDwell(double progress) { logDwell_ = progress; }  //!< 0..1, authoritative dwell progress for the UI ring

    void printSummary();  //!< Per-ID trial count and mean MT of the block (printed at the end of the session)

    M2FittsUILink ui;  //!< Communication layer (Unity client)

   private:
    //---- setup
    bool loadConfig();  //!< Optional 'key = value' config file; tries several conventional paths
    bool loadConfigFile(const std::string &file);
    std::string cfgStr(const std::string &key, const std::string &def) const;
    double cfgDbl(const std::string &key, double def) const;
    int cfgInt(const std::string &key, int def) const;
    bool cfgBool(const std::string &key, bool def) const;
    void applyConfig();

    bool readTrialCsv(const std::string &file, std::vector<FittsTrial> &trials) const;
    bool loadTrialTable();   //!< Loads bal_group_1..n.csv into blockTrials_
    bool loadWarmup();       //!< Loads warmup.csv if present, otherwise builds the default warm-up list
    bool checkTrialTable();  //!< ID consistency (ID = log2(A/W+1)), balance and workspace checks
    bool openResultsFile();
    void writeResultRow(const FittsTrialResult &r);
    void setCurrentTrial();
    std::vector<double> sessionDescriptor() const;  //!< Parameters of the SESS command

    //---- protocol state
    FittsParams params_;
    std::vector<FittsTrial> warmupTrials_;
    std::vector<FittsTrial> blockTrials_;
    FittsTrial currentTrial_;
    FittsPhase phase_ = PHASE_WARMUP;
    size_t trialIdx_ = 0;      //!< Index within the current phase list
    int trialsDone_ = 0;       //!< Total number of trials finalised (warm-up included)
    bool breakDue_ = false;
    double breakDuration_ = 60.;
    FittsTrialResult pending_; //!< Reach outcome of the trial being closed (completed by the return phase)

    //---- session/config
    std::map<std::string, std::string> cfg_;
    std::string participant_ = "P00";
    std::string trialsDir_ = "../schedule";
    std::string logDir_ = "../logs";
    std::string sessionTag_;   //!< <participant>_B<block>_<date-time>, used for both output files
    std::shared_ptr<FLNLHelper> UIserver = nullptr;
    std::string sessionId = "UNSET";
    std::string serverIP_ = "169.254.105.2";
    int serverPort_ = 2048;
    int block_ = 1;

    //---- UI
    bool uiRequired_ = true;   //!< Do not start the protocol until a client has connected
    int uiDivider_ = 2;        //!< Stream one frame every N control cycles (2 -> 250 Hz at 500 Hz loop)
    double displayGain_ = 1.0; //!< Robot metre -> scene metre. MUST be 1 unless a gain manipulation is intended.

    //---- results
    std::ofstream resultsFile_;
    std::vector<FittsTrialResult> results_;

    //---- values streamed in the continuous log and to the UI (must outlive the logger/link)
    //LogHelper::add() deduces its template parameter from the argument, so it must be given a
    //NON-const lvalue: a const double& deduces LogElement<const double>, whose const scalar member
    //cannot be default-initialised. (A const Eigen vector is fine - it has a default constructor -
    //which is why the position/velocity/force lines below compile with their const accessors.)
    //The UI markers therefore live here as plain doubles, mirrored from the link once per cycle,
    //consistent with the other logged values.
    double logState_ = 0.;
    double logPhase_ = 0.;
    double logTrialNb_ = 0.;
    double logTargetX_ = 0.;
    double logHalfWidth_ = 0.;
    double logDwell_ = 0.;
    double logMarkCode_ = 0.;
    double logMarkTime_ = 0.;
    double logMarkServerTime_ = 0.;
};

#endif  // M2FITTSHUMANMACHINE_H