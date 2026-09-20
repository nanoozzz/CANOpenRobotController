/**
 * \file M2FittsRobotHumanMachine.h
 * \brief Block 2 (human-robot shared control) of the robot-guidance Fitts' law experiment on the M2,
 *        with the Unity front-end driven over libFLNL exactly as in Block 1.
 *
 * Session: Wait for UI -> Block 2 (nRounds x trialsPerRound, breaks between rounds) -> End: the Block 1
 * procedure (M2FittsHumanMachine) without its warm-up and the rest that followed it. The reach runs under
 *
 *          u = alpha * u_r + (1 - alpha) * u_h                         (task axis; see FittsSharedControl.h)
 *
 * u_r is the PD of the robot-alone calibration (M2FittsMachine). Its gains are read from the same
 * M2FittsMachine.yaml, never from this app's configuration. u_h is the Block 1 human input. alpha is read
 * per trial from the 'alpha' column of the trial csv files; one value per ID is required and checked at start-up.
 *
 * -----------------------------------------------------------------------------------------
 * UNITY COMMUNICATION: the Block 1 protocol, unchanged (version 1, same 7-value state stream from
 * FLNLHelper(RobotM2&), same commands and field orders). SESS carries block = 2. alpha is not sent.
 * -----------------------------------------------------------------------------------------
 * OUTPUT (results_dir), <tag> = <participant>_B<block>[_alphaOVR]_<yyyymmdd-HHMMSS>:
 *   M2FittsRobotHuman_<tag>_trials.csv      Block 1 columns in the Block 1 order, then
 *                                           alpha, imp_h_Ns, imp_r_Ns, Fh_peak_N, conflict_frac,
 *                                           blend_sat_frac, reach_abort
 *   M2FittsRobotHuman_<tag>_raw.csv         Block 1 raw columns in the Block 1 order, then
 *                                           Alpha, Fpd (N)_1/_2, FhumanEst (N)_1/_2, Fcmd (N)_1/_2, BlendSaturated
 *   M2FittsRobotHuman_<tag>_parameters.txt  effective parameters, PD source file, alpha table
 *
 * Keyboard: 's' (or joystick button 1) = go/skip, 'x' = abort to transparent standby.
 *
 * \version 1.0
 * \date 2026-09-18
 */

#ifndef M2FITTSROBOTHUMANMACHINE_H
#define M2FITTSROBOTHUMANMACHINE_H

#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "FittsSharedControl.h"
#include "M2FittsUILink.h"
#include "RobotM2.h"
#include "StateMachine.h"

#include "M2FittsRobotHumanStates.h"

/**
 * \brief State machine of the shared-control Fitts' task, Block 2.
 *
 * Holds the protocol (trial table with alpha, phase/round/trial, breaks, results file), the robot PD gains,
 * the blend parameters and the human-force estimate. States read it through their `sm` pointer.
 */
class M2FittsRobotHumanMachine : public StateMachine {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    M2FittsRobotHumanMachine();
    ~M2FittsRobotHumanMachine();

    void init();
    void end();
    void hwStateUpdate();

    RobotM2 *robot() { return static_cast<RobotM2 *>(_robot.get()); }  //!< Robot getter with specialised type

    //---------------------------------------------------------------- protocol (as Block 1)
    const FittsParams &p() const { return params_; }
    FittsPhase phase() const { return phase_; }
    const FittsTrial &currentTrial() const { return currentTrial_; }
    bool sessionFinished() const { return phase_ == PHASE_DONE; }
    bool breakDue() const { return breakDue_; }
    double breakDuration() const { return breakDuration_; }
    void clearBreakDue() { breakDue_ = false; }
    bool requireGoSignal() const { return trialsDone_ == 0; }
    int trialsDone() const { return trialsDone_; }
    int blockNb() const { return block_; }
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

    void recordReach(const FittsTrialResult &r);
    void finaliseTrial(double returnTime, bool timedOut, bool aborted);

    //---------------------------------------------------------------- shared control (Block 2)
    //! alpha applied on trial t: the csv value, or alpha_override when set (commissioning only)
    double appliedAlpha(const FittsTrial &t) const { return (alphaOverride_ >= 0.) ? alphaOverride_ : t.alpha; }
    const fsc::PDGains &pdGains() const { return pdGains_; }        //!< u_r gains, from M2FittsMachine.yaml
    const fsc::BlendParams &blend() const { return blend_; }
    bool frictionCompensation() const { return frictionComp_; }     //!< RobotM2 compensation flag (Block 1: true)
    const VM2 &humanForce() const { return FhHat_; }                //!< Human force on the handle, robot frame [N], updated every cycle
    bool setupOk() const { return setupOk_; }

    void raiseFault(const std::string &why);                         //!< Latches a safety fault (-> FaultState)
    bool faultPending() const { return faultRaised_ && !faultAcknowledged_; }
    void acknowledgeFault() { faultAcknowledged_ = true; }
    const std::string &faultReason() const { return faultReason_; }

    //---------------------------------------------------------------- UI / inputs (as Block 1)
    void sendUI(const std::string &cmd, const std::vector<double> &params = {});
    void sendUIContext(const std::string &cmd, const std::vector<double> &params = {});
    bool goSignal();
    const char *goSource();  //!< Input that gave a go on this cycle (nullptr: none)
    bool abortSignal();
    bool uiReady() const { return !ui.required() || (ui.connected() && ui.handshakeDone()); }

    //---------------------------------------------------------------- continuous log
    void setLoggedState(double stateCode) { logState_ = stateCode; }
    void setLoggedTarget(double targetX, double halfWidth) { logTargetX_ = targetX; logHalfWidth_ = halfWidth; }
    void setLoggedDwell(double progress) { logDwell_ = progress; }
    void setLoggedControl(const VM2 &Fpd, const VM2 &Fcmd, double saturated) {
        logFpd_ = Fpd;
        logFcmd_ = Fcmd;
        logBlendSat_ = saturated;
    }

    void printSummary();  //!< Per-ID alpha, trial count and mean MT of the block

    M2FittsUILink ui;  //!< Communication layer (Unity client)

   private:
    //---- setup
    bool loadConfig();
    bool loadConfigFile(const std::string &file);
    std::string cfgStr(const std::string &key, const std::string &def) const;
    double cfgDbl(const std::string &key, double def) const;
    int cfgInt(const std::string &key, int def) const;
    bool cfgBool(const std::string &key, bool def) const;
    std::string expandPath(const std::string &s) const;  //!< Replaces {participant}
    void applyConfig();

    bool loadRobotController();  //!< PD gains of the robot-alone calibration (M2FittsMachine.yaml)
    bool readTrialCsv(const std::string &file, std::vector<FittsTrial> &trials, bool &hasAlpha) const;
    bool loadTrialTable();       //!< trials_file, or <trials_dir>/<trials_prefix>1..n.csv (alpha column required)
    bool buildAlphaTable();      //!< ID -> alpha, checks range and one alpha per ID
    bool lookupAlpha(double id, double &alpha) const;
    bool checkTrialTable();
    bool openResultsFile();
    void writeResultRow(const FittsTrialResult &r);
    bool writeParametersFile(const std::string &file) const;
    void setCurrentTrial();
    std::vector<double> sessionDescriptor() const;

    //---- protocol state
    FittsParams params_;
    std::vector<FittsTrial> blockTrials_;
    FittsTrial currentTrial_;
    FittsPhase phase_ = PHASE_BLOCK;
    size_t trialIdx_ = 0;
    int trialsDone_ = 0;
    bool breakDue_ = false;
    double breakDuration_ = 60.;
    FittsTrialResult pending_;

    //---- shared control
    fsc::PDGains pdGains_;
    fsc::BlendParams blend_;
    bool frictionComp_ = true;
    double alphaOverride_ = -1.;
    std::vector<std::pair<double, double>> alphaTable_;  //!< (ID bits, alpha), sorted by ID
    std::string robotConfigSetting_;                     //!< robot_config key ("" = search the usual places)
    std::string robotConfigFile_;                        //!< YAML file the PD was actually read from
    fsc::LowPass2 fhFilter_;
    double humanForceFilterHz_ = 0.;
    VM2 FhHat_ = VM2::Zero();
    double lastHwTime_ = -1.;
    bool setupOk_ = false;
    bool faultRaised_ = false;
    bool faultAcknowledged_ = false;
    std::string faultReason_;

    //---- session/config
    std::map<std::string, std::string> cfg_;
    std::string configFile_;
    std::string participant_ = "P00";
    std::string trialsDir_ = "../schedule";
    std::string trialsPrefix_ = "bal_group_";
    std::string trialsFile_;  //!< trials_file: one csv holding every trial ("" = one file per round)
    std::string logDir_ = "../logs";
    std::string sessionTag_;
    std::string resultsPath_, rawPath_, paramsPath_;
    std::shared_ptr<FLNLHelper> UIserver = nullptr;
    std::string bindIP_ = "0.0.0.0";
    int serverPort_ = 2048;
    int block_ = 2;
    double lastConnCheck_ = -1e9;

    //---- UI
    bool uiRequired_ = true;
    int uiDivider_ = 2;
    double displayGain_ = 1.0;

    //---- results
    std::ofstream resultsFile_;
    std::vector<FittsTrialResult> results_;
    bool summaryPrinted_ = false;

    //---- values in the continuous log / UI stream (must outlive the logger; non-const lvalues, see Block 1)
    double logState_ = 0.;
    double logPhase_ = 0.;
    double logTrialNb_ = 0.;
    double logTargetX_ = 0.;
    double logHalfWidth_ = 0.;
    double logDwell_ = 0.;
    double logMarkCode_ = 0.;
    double logMarkTime_ = 0.;
    double logMarkServerTime_ = 0.;
    double logAlpha_ = fittsNaN();
    VM2 logFpd_ = VM2::Zero();
    VM2 logFh_ = VM2::Zero();
    VM2 logFcmd_ = VM2::Zero();
    double logBlendSat_ = fittsNaN();
};

#endif  // M2FITTSROBOTHUMANMACHINE_H