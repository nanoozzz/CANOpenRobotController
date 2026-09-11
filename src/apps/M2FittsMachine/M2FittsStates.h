/**
 * \file M2FittsStates.h
 * \brief States of the M2FittsMachine CORC app (ArmMotus M2, 1D reaching along x with a PD controller).
 *
 *  CalibState -> GoHomeState -> ReadyState --[S]--> ReachState -> ReturnState -> ReachState ... -> EndState
 *  Any state -> FaultState when an app-level safety check fails.
 *
 * Provenance: drafted with AI assistance (Claude, Anthropic), September 2026. Review before use.
 * Licence: Apache-2.0 (same as CORC).
 */
#ifndef M2FITTS_STATES_H
#define M2FITTS_STATES_H

#include <memory>
#include <string>
#include <vector>

#include "FittsCore.h"
#include "RobotM2.h"
#include "State.h"

/** Phase codes written in the time-series log. */
enum FittsPhase : int {
    PHASE_CALIB = 0,
    PHASE_GO_HOME = 1,
    PHASE_READY = 2,
    PHASE_REACH = 3,
    PHASE_RETURN = 4,
    PHASE_END = 5,
    PHASE_FAULT = 9
};

/** Data shared by the state machine and all states (configuration, trial table, flags, logged variables). */
struct FittsSession {
    fitts::FittsConfig cfg;
    std::vector<fitts::Trial> trials;
    fitts::CsvInfo csv;
    std::size_t next_trial = 0;  //!< 0-based index of the next trial to run
    fitts::ResultsWriter results;

    bool setup_ok = false;  //!< false: configuration/trial file invalid -> motors are never driven
    std::string setup_error;
    bool fault = false;
    std::string fault_reason;
    bool pause_requested = false;
    int n_success = 0, n_timeout = 0, n_aborted = 0;

    // Variables recorded at every control loop by the StateMachine LogHelper
    int log_trial = 0;
    int log_phase = PHASE_CALIB;
    int log_in_target = 0;
    int log_saturated = 0;
    VM2 log_x_ref = VM2::Zero();
    VM2 log_force = VM2::Zero();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/** Common base: robot access, PD application and app-level safety checks. */
class M2FittsBaseState : public State {
   public:
    M2FittsBaseState(RobotM2 *robot, std::shared_ptr<FittsSession> session, const std::string &name)
        : State(name), robot_(robot), s_(std::move(session)) {
        // Gains from the YAML (loaded before the states are constructed). Fix 11 Sep 2026: this call was
        // missing, so the controllers silently ran the in-code placeholder gains (400/80 N, f_max 30 N).
        pd_.setGains(s_->cfg.pd);
    }
    const fitts::PDGains &pdGains() const { return pd_.gains(); }  //!< Gains actually used by this state's controller

   protected:
    VM2 position() { return robot_->getEndEffPosition(); }
    VM2 velocity() { return robot_->getEndEffVelocity(); }
    /** Compute the PD force towards (x_ref, v_ref), send it to the robot, update logged variables. Returns true if saturated. */
    bool applyPD(const VM2 &x_ref, const VM2 &v_ref);
    /** Returns false (and raises the session fault flag) if speed, workspace or sensor readings are not acceptable. */
    bool safetyOk();
    /** Pure damping force (no position term), saturated; applied at joint level so it also works if uncalibrated. */
    void brake();

    RobotM2 *robot_;
    std::shared_ptr<FittsSession> s_;
    fitts::PDController pd_;
};

/** Absolute position calibration: push both axes against their lower mechanical stops, then set q = (0, 0). */
class M2FittsCalibState : public M2FittsBaseState {
   public:
    M2FittsCalibState(RobotM2 *robot, std::shared_ptr<FittsSession> session)
        : M2FittsBaseState(robot, std::move(session), "M2Fitts Calibration") {}
    bool isDone() const { return done_; }

   protected:
    void entry() override;
    void during() override;
    void exit() override;

   private:
    VM2 still_time_ = VM2::Zero();
    bool done_ = false;
};

/** Move to home along a rate-limited reference, then wait until settled (used for initial homing and returns). */
class M2FittsHomingState : public M2FittsBaseState {
   public:
    M2FittsHomingState(RobotM2 *robot, std::shared_ptr<FittsSession> session, const std::string &name,
                       FittsPhase phase, double ref_speed)
        : M2FittsBaseState(robot, std::move(session), name), phase_(phase), ref_speed_(ref_speed) {}
    bool isSettled() const { return settled_; }
    bool timedOut() const { return timed_out_; }

   protected:
    void entry() override;
    void during() override;
    void exit() override {}

   private:
    FittsPhase phase_;
    double ref_speed_;
    fitts::RateLimitedReference ref_;
    double rest_time_ = 0., timeout_at_ = 0.;
    bool settled_ = false, timed_out_ = false;
};

/** Hold home and wait for the operator (key S) before starting or resuming the block. */
class M2FittsReadyState : public M2FittsBaseState {
   public:
    M2FittsReadyState(RobotM2 *robot, std::shared_ptr<FittsSession> session)
        : M2FittsBaseState(robot, std::move(session), "M2Fitts Ready") {}

   protected:
    void entry() override;
    void during() override;
    void exit() override;
};

/** One trial: step set-point to the target, PD control, event detection; the trial row is written on exit. */
class M2FittsReachState : public M2FittsBaseState {
   public:
    M2FittsReachState(RobotM2 *robot, std::shared_ptr<FittsSession> session)
        : M2FittsBaseState(robot, std::move(session), "M2Fitts Reach") {}
    bool isFinished() const { return monitor_.finished(); }

   protected:
    void entry() override;
    void during() override;
    void exit() override;

   private:
    fitts::TrialMonitor monitor_;
    VM2 target_ = VM2::Zero();
    bool started_ = false;
};

/** Block completed: hold home and report. */
class M2FittsEndState : public M2FittsBaseState {
   public:
    M2FittsEndState(RobotM2 *robot, std::shared_ptr<FittsSession> session)
        : M2FittsBaseState(robot, std::move(session), "M2Fitts End") {}

   protected:
    void entry() override;
    void during() override;
    void exit() override {}
};

/** Safety fault: brake with pure damping until the app is stopped (Ctrl+C). */
class M2FittsFaultState : public M2FittsBaseState {
   public:
    M2FittsFaultState(RobotM2 *robot, std::shared_ptr<FittsSession> session)
        : M2FittsBaseState(robot, std::move(session), "M2Fitts Fault") {}

   protected:
    void entry() override;
    void during() override;
    void exit() override;
};

#endif  // M2FITTS_STATES_H
