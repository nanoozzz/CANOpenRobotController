/**
 * \file M2FittsHumanMachine.h
 * \brief Block 1 (no robot support) of the robot-guidance Fitts' law experiment on the M2.
 *
 * The app runs, for one participant:
 *      Warm-up (warmupRepeats x 5 IDs)  ->  Rest  ->  Block 1 (4 rounds x 45 trials, from csv)  ->  End
 * and writes one row per trial (trial #, A, W, ID, MT, ...) to a results csv, plus the usual
 * CORC control-loop-rate log of position/velocity/force.
 *
 * Trial list: data/M2FittsHuman/bal_group_1.csv ... bal_group_4.csv (columns index,A,W,ID; A and W in cm).
 * Warm-up list: data/M2FittsHuman/warmup.csv if present, otherwise the built-in default table
 *               (one canonical (A,W) per ID, see M2FittsHumanMachine.cpp).
 *
 * Unity communication (libFLNL, see FLNLHelper):
 *   - continuous state stream: [time, x, y, dx, dy, Fx, Fy] (registered by the FLNLHelper M2 constructor)
 *   - commands sent to the UI (4 characters + parameters):
 *       "TRIA" [phase, round, trial#, A_cm, W_cm, ID, targetX, targetY, halfWidth, originX, originY]  target onset
 *       "HITT" [trial#, MT]              trial validated (dwell completed)
 *       "MISS" [trial#]                  trial timed out
 *       "RETN" [awayX, awayY]            robot is driving the handle off the origin
 *       "DRAG" [originX, originY, tol, maxTime]   participant must drag the handle back
 *       "ORIG" [originX, originY]        handle on the origin, next trial imminent
 *       "REST" [duration, roundDone]     break started
 *       "RDY!" [requireGo]               waiting at the origin for the go signal
 *       "ENDE" [nTrials]                 end of Block 1
 *   - commands accepted from the UI: "GTNS", "STRT" or "SKIP" (go / skip the current wait), acknowledged with "OK"
 *
 * Keyboard: 's' (or joystick button 1) = go/skip, 'x' = abort to transparent standby.
 *
 * \version 0.1
 * \date 2026-09-12
 */

#ifndef M2FITTSHUMANMACHINE_H
#define M2FITTSHUMANMACHINE_H

#include <fstream>
#include <map>
#include <string>
#include <vector>

#include "FLNLHelper.h"
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
    void sendUI(const std::string &cmd, const std::vector<double> &params = {});
    bool goSignal();    //!< Go/skip requested (UI command, keyboard 's'/'1' or joystick button 1)?
    bool abortSignal(); //!< Abort requested (keyboard 'x')?

    //---------------------------------------------------------------- continuous log
    void setLoggedState(double stateCode) { logState_ = stateCode; }
    void setLoggedTarget(double targetX, double halfWidth) { logTargetX_ = targetX; logHalfWidth_ = halfWidth; }

    void printSummary();  //!< Per-ID trial count and mean MT of the block (printed at the end of the session)

    std::shared_ptr<FLNLHelper> UIserver = nullptr;  //!< Communication server (Unity client)

   private:
    //---- setup
    bool loadConfig(const std::string &file);  //!< Optional 'key = value' config file; returns false if absent
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
    std::string serverIP_ = "169.254.105.3";
    int serverPort_ = 2048;
    int block_ = 1;

    //---- results
    std::ofstream resultsFile_;
    std::vector<FittsTrialResult> results_;

    //---- values streamed in the continuous log (must outlive the logger)
    double logState_ = 0.;
    double logPhase_ = 0.;
    double logTrialNb_ = 0.;
    double logTargetX_ = 0.;
    double logHalfWidth_ = 0.;
};

#endif  // M2FITTSHUMANMACHINE_H