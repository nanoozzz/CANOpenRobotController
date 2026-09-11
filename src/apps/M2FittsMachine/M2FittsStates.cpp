/**
 * \file M2FittsStates.cpp
 * \brief Implementation of the M2FittsMachine states (see M2FittsStates.h).
 *
 * Provenance: drafted with AI assistance (Claude, Anthropic), September 2026. Review before use.
 * Licence: Apache-2.0 (same as CORC).
 */
#include "M2FittsStates.h"

#include <algorithm>
#include <cmath>
#include <sstream>

// ================================================================================== Base state helpers
bool M2FittsBaseState::applyPD(const VM2 &x_ref, const VM2 &v_ref) {
    bool saturated = false;
    const VM2 F = pd_.compute(x_ref, v_ref, position(), velocity(), dt(), &saturated);
    // friction_compensation = false -> pure PD (setJointTorque(J^T F) with J = I for the M2).
    // friction_compensation = true  -> PD + RobotM2's model-based friction feedforward.
    robot_->setEndEffForceWithCompensation(F, s_->cfg.friction_compensation);
    s_->log_x_ref = x_ref;
    s_->log_force = F;
    s_->log_saturated = saturated ? 1 : 0;
    return saturated;
}

bool M2FittsBaseState::safetyOk() {
    if (s_->fault) return false;
    const fitts::FittsConfig &c = s_->cfg;
    const VM2 x = position(), v = velocity();
    std::ostringstream why;
    if (!robot_->isCalibrated()) {
        why << "robot is not calibrated";
    } else if (!x.allFinite() || !v.allFinite()) {
        why << "non-finite position or velocity reading";
    } else if (v.norm() > c.max_speed) {
        why << "end-effector speed " << v.norm() << " m/s exceeds safety.max_speed (" << c.max_speed << " m/s)";
    } else if (x(0) < c.workspace_x(0) - c.workspace_tolerance || x(0) > c.workspace_x(1) + c.workspace_tolerance ||
               x(1) < c.workspace_y(0) - c.workspace_tolerance || x(1) > c.workspace_y(1) + c.workspace_tolerance) {
        why << "position (" << x(0) << ", " << x(1) << ") m is outside the workspace";
    }
    const std::string reason = why.str();
    if (reason.empty()) return true;
    s_->fault = true;
    s_->fault_reason = reason;
    spdlog::critical("M2Fitts safety check failed: {}", reason);
    return false;
}

void M2FittsBaseState::brake() {
    VM2 v = velocity();
    if (!v.allFinite()) v.setZero();
    VM2 F = -s_->cfg.brake_damping * v;
    for (int i = 0; i < 2; ++i) F(i) = std::max(-s_->cfg.pd.f_max, std::min(s_->cfg.pd.f_max, F(i)));
    robot_->setJointTorque(F);  // M2 is Cartesian (J = I): joint force == end-effector force
    s_->log_force = F;
    s_->log_saturated = 0;
}

// ================================================================================== Calibration
void M2FittsCalibState::entry() {
    done_ = false;
    still_time_.setZero();
    s_->log_phase = PHASE_CALIB;
    if (!s_->setup_ok) return;  // invalid configuration or trial file: never drive the motors
    robot_->decalibrate();
    // Torque mode is initialised once, here, and never re-initialised afterwards:
    // RobotM2::initTorqueControl() exchanges SDOs and sleeps, which would perturb trial timing.
    robot_->initTorqueControl();
    spdlog::info("M2Fitts: calibrating against the lower mechanical stops - keep the workspace clear.");
}

void M2FittsCalibState::during() {
    if (!s_->setup_ok) return;
    const fitts::FittsConfig &c = s_->cfg;
    if (robot_->isCalibrated()) {
        robot_->setJointTorque(VM2::Zero());
        done_ = true;
        return;
    }
    const VM2 v = robot_->getVelocity();
    VM2 tau = VM2::Zero();
    for (int i = 0; i < 2; ++i) {
        // Push each axis towards its lower stop with a constant force, reduced when moving away from it.
        // This force law and its default values follow M2CalibState in CORC's M2DemoMachine (Apache-2.0).
        tau(i) = -std::min(std::max(c.calib_force - c.calib_damping * v(i), 0.), c.calib_force);
        // Here the axis must be continuously still (the timer resets on motion).
        still_time_(i) = (std::abs(v(i)) < c.calib_still_speed) ? still_time_(i) + dt() : 0.;
    }
    if (still_time_(0) >= c.calib_still_time && still_time_(1) >= c.calib_still_time) {
        robot_->applyCalibration();  // sets q = (0, 0) at the stops (RobotM2::qCalibration)
        spdlog::info("M2Fitts: calibration done.");
    } else {
        robot_->setJointTorque(tau);
    }
}

void M2FittsCalibState::exit() {
    if (s_->setup_ok) robot_->setJointTorque(VM2::Zero());
}

// ================================================================================== Homing / return
void M2FittsHomingState::entry() {
    settled_ = false;
    timed_out_ = false;
    rest_time_ = 0.;
    s_->log_phase = phase_;
    s_->log_in_target = 0;
    if (phase_ == PHASE_GO_HOME) s_->log_trial = 0;  // during returns the last trial number is kept in the log

    const VM2 x = position();
    ref_.reset(x, s_->cfg.home, ref_speed_);
    const double travel_time = (ref_speed_ > 0.) ? (s_->cfg.home - x).norm() / ref_speed_ : 0.;
    timeout_at_ = travel_time + s_->cfg.settle_timeout;

    pd_.reset();
    VM2 v_ref;
    const VM2 x_ref = ref_.update(0., v_ref);
    applyPD(x_ref, v_ref);
}

void M2FittsHomingState::during() {
    if (!safetyOk()) {
        brake();
        return;
    }
    VM2 v_ref;
    const VM2 x_ref = ref_.update(dt(), v_ref);
    applyPD(x_ref, v_ref);

    const fitts::FittsConfig &c = s_->cfg;
    const double home_error = (position() - c.home).norm();
    const bool at_rest_home = ref_.done() && home_error <= c.home_tolerance && velocity().norm() <= c.rest_speed;
    rest_time_ = at_rest_home ? rest_time_ + dt() : 0.;
    if (!settled_ && rest_time_ >= c.rest_time) settled_ = true;
    if (!settled_ && !timed_out_ && running() > timeout_at_) {
        timed_out_ = true;
        spdlog::warn("M2Fitts: not settled within {} m of home after {:.1f} s (error {:.4f} m). "
                     "Pausing - check gains/friction, then press S to continue.",
                     c.home_tolerance, running(), home_error);
    }
}

// ================================================================================== Ready (operator)
void M2FittsReadyState::entry() {
    s_->log_phase = PHASE_READY;
    s_->log_in_target = 0;
    s_->pause_requested = false;
    pd_.reset();
    applyPD(s_->cfg.home, VM2::Zero());
    const fitts::PDGains &g = pd_.gains();  // read back from the controller itself, not from the configuration
    spdlog::info("M2Fitts PD in use: kp = [{}, {}] N/m, kd = [{}, {}] N.s/m, f_max = {} N, velocity filter = {} Hz",
                 g.kp(0), g.kp(1), g.kd(0), g.kd(1), g.f_max, g.vel_filter_hz);
    if (s_->next_trial < s_->trials.size())
        spdlog::info("M2Fitts READY at home. Next trial {}/{}. Press S to start/resume, X to pause after the "
                     "current trial, Ctrl+C to stop.",
                     s_->next_trial + 1, s_->trials.size());
}

void M2FittsReadyState::during() {
    if (!safetyOk()) {
        brake();
        return;
    }
    applyPD(s_->cfg.home, VM2::Zero());
}

void M2FittsReadyState::exit() { s_->pause_requested = false; }

// ================================================================================== Reach (one trial)
void M2FittsReachState::entry() {
    started_ = false;
    if (s_->next_trial >= s_->trials.size()) {  // defensive: transitions should prevent this
        s_->fault = true;
        s_->fault_reason = "internal error: ReachState entered with no trial left";
        return;
    }
    const fitts::FittsConfig &c = s_->cfg;
    const fitts::Trial &tr = s_->trials[s_->next_trial];
    target_ = c.home;
    target_(0) = fitts::targetX(tr, c);  // y set-point stays at home y

    // Trial onset (t = 0 of State::running()): the PD set-point steps from home to the target.
    pd_.reset();
    monitor_.start(tr, c, position(), fitts::localTimestamp("%Y-%m-%dT%H:%M:%S", true));
    applyPD(target_, VM2::Zero());
    started_ = true;

    s_->log_trial = tr.index;
    s_->log_phase = PHASE_REACH;
    s_->log_in_target = 0;
    spdlog::info("Trial {}/{} (CSV line {}): D = {}, W = {} -> x_target = {:.4f} m, tolerance = +/-{:.4f} m", tr.index,
                 s_->trials.size(), tr.csv_line, tr.D, tr.W, target_(0), fitts::targetTolerance(tr, c));
}

void M2FittsReachState::during() {
    if (!safetyOk()) {
        brake();
        return;
    }
    const bool saturated = applyPD(target_, VM2::Zero());
    if (!monitor_.finished()) {
        monitor_.update(running(), position(), velocity(), saturated);
        s_->log_in_target = monitor_.inTarget() ? 1 : 0;
    }
}

void M2FittsReachState::exit() {
    s_->log_in_target = 0;
    if (!started_) return;
    if (!monitor_.finished())
        monitor_.abort(running(), s_->fault ? "fault: " + s_->fault_reason : "interrupted before completion");
    const fitts::TrialResult &r = monitor_.result();
    if (!s_->results.write(r))
        spdlog::error("M2Fitts: could not write trial {} to '{}'", r.trial.index, s_->results.path());
    switch (r.outcome) {
        case fitts::TrialOutcome::Success: ++s_->n_success; break;
        case fitts::TrialOutcome::Timeout: ++s_->n_timeout; break;
        default: ++s_->n_aborted; break;
    }
    spdlog::info("Trial {} {}: MT first entry = {:.3f} s, MT final entry = {:.3f} s, entries = {}", r.trial.index,
                 fitts::outcomeName(r.outcome), r.mt_first_entry, r.mt_final_entry, r.n_entries);
    ++s_->next_trial;
}

// ================================================================================== End of block
void M2FittsEndState::entry() {
    s_->log_phase = PHASE_END;
    s_->log_trial = 0;
    s_->log_in_target = 0;
    pd_.reset();
    applyPD(s_->cfg.home, VM2::Zero());
    const std::string path = s_->results.path();
    s_->results.close();
    spdlog::info("M2Fitts: block finished - success {}, timeout {}, aborted {}. Results: {}", s_->n_success,
                 s_->n_timeout, s_->n_aborted, path);
    spdlog::info("M2Fitts: holding home. Press Ctrl+C to stop the app.");
}

void M2FittsEndState::during() {
    if (!safetyOk()) {
        brake();
        return;
    }
    applyPD(s_->cfg.home, VM2::Zero());
}

// ================================================================================== Fault
void M2FittsFaultState::entry() {
    s_->log_phase = PHASE_FAULT;
    s_->log_in_target = 0;
    brake();
    spdlog::critical("M2Fitts FAULT: {}. Braking with damping only. Stop the app with Ctrl+C (restart required).",
                     s_->fault_reason);
}

void M2FittsFaultState::during() { brake(); }

void M2FittsFaultState::exit() {
    if (s_->setup_ok) robot_->setJointTorque(VM2::Zero());
}
