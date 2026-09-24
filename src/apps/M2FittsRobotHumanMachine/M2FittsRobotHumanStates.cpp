/**
 * \file M2FittsRobotHumanStates.cpp
 * \brief States of M2FittsRobotHumanMachine (see M2FittsRobotHumanStates.h). Every state except the Reach
 *        and the Fault state is the Block 1 state (M2FittsHumanStates.cpp); the y-channel force is computed
 *        by fsc::channelForce(), which equals the Block 1 expression while channel_saturate = false.
 */
#include "M2FittsRobotHumanStates.h"

#include <algorithm>
#include <cstdio>
#include <iomanip>
#include <iostream>

#include "M2FittsRobotHumanMachine.h"

//! M2 travel (RobotM2 joint limits) [m]
static const double kTravelX = 0.625;
static const double kTravelY = 0.440;

/**
 * \brief Minimum jerk interpolation (see header).
 */
double JerkIt(VM2 X0, VM2 Xf, double T, double t, VM2 &Xd, VM2 &dXd) {
    if (T <= 0) {
        Xd = Xf;
        dXd = VM2::Zero();
        return 1.;
    }
    t = std::max(std::min(t, T), .0);                //Bound time
    double tn = std::max(std::min(t / T, 1.0), .0);  //Normalised time bounded 0-1
    double tn2 = tn * tn;
    double tn3 = tn2 * tn;
    double tn4 = tn * tn3;
    double tn5 = tn * tn4;
    Xd = X0 + ((X0 - Xf) * (15. * tn4 - 6. * tn5 - 10. * tn3));
    dXd = (X0 - Xf) * (60. * tn3 - 30. * tn4 - 30. * tn2) / T;
    return tn;
}

//! True if |interaction force| exceeds limit (limit <= 0 disables the test).
static bool interactionForceAbove(RobotM2 *robot, double limit) {
    if (limit <= 0) return false;
    VM2 F = robot->getInteractionForce();
    return F.norm() > limit;
}

//! Block 1 command outside robot-driven moves: transparent along x, virtual channel on y.
static VM2 channelOnly(M2FittsRobotHumanMachine *sm, const VM2 &X, const VM2 &dX, double yCentre) {
    VM2 F(0., fsc::channelForce(sm->blend(), yCentre, X(1), dX(1)));
    if (!F.allFinite()) F.setZero();
    return F;
}

/******************************************************************************
 * Calibration (Block 1; nothing is driven if the setup failed)
 ******************************************************************************/
void M2FittsCalibState::entryCode(void) {
    sm->setLoggedState(ST_CALIB);
    sm->setLoggedDwell(0.);
    calibDone = false;
    for (unsigned int i = 0; i < 2; i++) {
        stop_reached_time[i] = .0;
        at_stop[i] = false;
    }
    if (!sm->setupOk()) {
        spdlog::critical("M2FittsRobotHuman: setup failed - the robot is not driven.");
        return;
    }
    robot->decalibrate();
    robot->initTorqueControl();
    robot->printJointStatus();
    std::cout << "Calibrating (keep clear)..." << std::flush;
}

void M2FittsCalibState::duringCode(void) {
    if (!sm->setupOk()) return;
    VM2 tau(0, 0);

    //Apply constant torque (with damping) unless stop has been detected for more than 1s
    VM2 vel = robot->getVelocity();
    double b = 3;
    for (unsigned int i = 0; i < vel.size(); i++) {
        tau(i) = -std::min(std::max(20 - b * vel(i), .0), 20.);
        if (stop_reached_time(i) > 1) at_stop[i] = true;
        if (std::abs(vel(i)) < 0.005) stop_reached_time(i) += dt();
    }

    //Switch to transparent control when done
    if (robot->isCalibrated()) {
        robot->setEndEffForceWithCompensation(VM2::Zero(), false);
        calibDone = true;
    } else {
        if (at_stop[0] && at_stop[1]) {
            robot->applyCalibration();
            std::cout << "OK." << std::endl;
        } else {
            robot->setJointTorque(tau);
            if (iterations() % 100 == 1) std::cout << "." << std::flush;
        }
    }
}

void M2FittsCalibState::exitCode(void) {
    if (!sm->setupOk()) return;
    robot->setEndEffForceWithCompensation(VM2::Zero());
}

/******************************************************************************
 * Wait for the Unity client (Block 1)
 ******************************************************************************/
void M2FittsWaitUIState::entryCode(void) {
    sm->setLoggedState(ST_WAITUI);
    robot->initTorqueControl();
    robot->setEndEffForceWithCompensation(VM2::Zero(), true);
    ready_ = false;
    announced_ = false;

    if (!sm->ui.required()) {
        spdlog::info("M2FittsRobotHuman: ui_required = false - starting without a display client.");
        ready_ = true;
        return;
    }
    std::cout << "\nWaiting for the Unity client to connect (and to send HELO)...\n"
              << "The robot is transparent; nothing is recorded until the display is up." << std::endl;
}

void M2FittsWaitUIState::duringCode(void) {
    robot->setEndEffForceWithCompensation(VM2::Zero(), true);

    if (sm->uiReady()) {
        if (!announced_) {
            announced_ = true;
            sm->sendUIContext("WAIT");
            std::cout << "Client connected." << std::endl;
        }
        ready_ = true;
        return;
    }

    if (iterations() % 2500 == 1)  //~ every 5 s at a 2 ms control period
        std::cout << "   still waiting for the client..." << std::endl;
}

void M2FittsWaitUIState::exitCode(void) {
    robot->setEndEffForceWithCompensation(VM2::Zero());
}

/******************************************************************************
 * Standby: transparent, no task (abort target, Block 1)
 ******************************************************************************/
void M2FittsStandbyState::entryCode(void) {
    sm->setLoggedState(ST_STANDBY);
    sm->setLoggedDwell(0.);
    robot->initTorqueControl();
    sm->sendUIContext("STBY");
    spdlog::warn("M2FittsRobotHuman: session interrupted - robot transparent. Completed trials are saved.");
}

void M2FittsStandbyState::duringCode(void) {
    robot->setEndEffForceWithCompensation(VM2::Zero(), true);
}

void M2FittsStandbyState::exitCode(void) {
    robot->setEndEffForceWithCompensation(VM2::Zero());
}

/******************************************************************************
 * Ready: bring the handle to the origin and wait for the go signal (Block 1)
 ******************************************************************************/
void M2FittsReadyState::entryCode(void) {
    sm->setLoggedState(ST_READY);
    sm->setLoggedDwell(0.);
    robot->initVelocityControl();
    robot->setEndEffVelocity(VM2::Zero());

    Xi_ = robot->getEndEffPosition();
    Xorigin_ = sm->originPosition();
    T_ = std::max(sm->p().returnMinTime, (Xorigin_ - Xi_).norm() / sm->p().returnSpeed);

    requireGo_ = sm->requireGoSignal();
    atOrigin_ = false;
    tAtOrigin_ = 0.;
    ready_ = false;
    //Same for the ready screen: start (or resume) only on a go given from now on
    if (sm->ui.consumeGo()) spdlog::info("M2FittsRobotHuman: a go pressed before the ready screen was discarded.");

    sm->sendUIContext("RDY!", {requireGo_ ? 1. : 0., Xorigin_(0), Xorigin_(1)});
    if (requireGo_) {
        const char *what = (sm->phase() == PHASE_WARMUP)     ? "warm-up trial"
                           : (sm->phase() == PHASE_COOLDOWN) ? "cool-down trial (ask for the autonomy rating first)"
                                                             : "trial";
        std::cout << "\nNext: " << what << " " << sm->currentTrial().index << " of "
                  << (sm->phase() == PHASE_WARMUP ? sm->nWarmupTrials()
                      : sm->phase() == PHASE_COOLDOWN ? sm->nCooldownTrials() : sm->nBlockTrials())
                  << " (alpha " << sm->appliedAlpha(sm->currentTrial()) << "). Hold the handle. "
                  << "Press 's' (or joystick button 1, or the start control in the UI) to begin.\n" << std::endl;
    }
    else
        std::cout << "Resuming in " << sm->p().readyHoldTime << " s (hold the handle at the start position)..." << std::endl;
}

void M2FittsReadyState::duringCode(void) {
    VM2 X = robot->getEndEffPosition();
    VM2 Xd, dXd;
    double status = JerkIt(Xi_, Xorigin_, T_, running(), Xd, dXd);
    robot->setEndEffVelocity(dXd + sm->p().kPosVel * (Xd - X));

    if (status >= 1. && !atOrigin_) {
        atOrigin_ = true;
        tAtOrigin_ = running();
        sm->sendUIContext("ORIG", {Xorigin_(0), Xorigin_(1)});
    }

    if (sm->goSignal())
        ready_ = true;
    else if (atOrigin_ && !requireGo_ && (running() - tAtOrigin_) >= sm->p().readyHoldTime)
        ready_ = true;
}

void M2FittsReadyState::exitCode(void) {
    robot->setEndEffVelocity(VM2::Zero());
}

/******************************************************************************
 * Shared reach: one trial under u = alpha*u_r + (1-alpha)*u_h
 ******************************************************************************/
void M2FittsSharedReachState::entryCode(void) {
    sm->setLoggedState(ST_REACH);
    sm->setLoggedDwell(0.);

    trial_ = sm->currentTrial();
    alpha_ = sm->appliedAlpha(trial_);
    Xorigin_ = sm->originPosition();
    Xtarget_ = sm->targetPosition(trial_);
    Xref_ = VM2(Xtarget_(0), Xorigin_(1));  //x: PD set-point = target centre (as M2FittsMachine); y: channel centre
    halfW_ = trial_.halfW();
    sm->setLoggedTarget(Xtarget_(0), halfW_);

    //Torque mode, as in Block 1 (the handle arrives from the velocity-controlled hold on the origin)
    robot->initTorqueControl();
    law_.setGains(sm->pdGains());
    law_.setParams(sm->blend());
    law_.reset();  //PD and its velocity filter restart at onset, as in M2FittsMachine

    trialDone_ = false;
    leftHome_ = false;
    inTarget_ = false;
    tEntry_ = 0.;
    xEntry_ = 0.;
    overForceTime_ = 0.;
    acc_ = SharedAccum();
    accAtEntry_ = SharedAccum();

    res_ = FittsTrialResult();
    res_.phase = sm->phase();
    res_.trial = trial_;
    res_.alpha = alpha_;
    res_.t_onset = sm->runningTime();
    res_.x_start_cm = (sm->taskCoord(robot->getEndEffPosition()) - sm->taskCoord(Xorigin_)) * 100.;

    //Target onset: the PD set-point steps to the target on this same cycle
    applySharedControl(0.);

    //Same TRIA message as Block 1 (alpha is not sent to the display)
    sm->sendUIContext("TRIA", {sm->uiPhase(), (double)trial_.round, (double)trial_.inRound,
                               (double)trial_.index, trial_.A_cm, trial_.W_cm, trial_.ID_bits,
                               Xtarget_(0), Xtarget_(1), halfW_, Xorigin_(0), Xorigin_(1),
                               sm->p().dwellTime, sm->p().maxTrialTime});
}

bool M2FittsSharedReachState::safetyOk(const VM2 &X, const VM2 &dX) {
    const FittsParams &p = sm->p();
    const double tol = p.workspaceTolerance;
    char why[200] = "";
    if (!X.allFinite() || !dX.allFinite())
        std::snprintf(why, sizeof(why), "non-finite position or velocity reading");
    else if (!std::isfinite(sm->humanForce()(0)))
        std::snprintf(why, sizeof(why), "interaction force not available (sensors not zeroed?) - shared control needs it");
    else if (p.maxSpeed > 0. && dX.norm() > p.maxSpeed)
        std::snprintf(why, sizeof(why), "end-effector speed %.2f m/s above max_speed (%.2f m/s)", dX.norm(), p.maxSpeed);
    else if (X(0) < -tol || X(0) > kTravelX + tol || X(1) < -tol || X(1) > kTravelY + tol)
        std::snprintf(why, sizeof(why), "handle outside the M2 travel (x = %.3f m, y = %.3f m)", X(0), X(1));
    if (why[0] == '\0') return true;
    sm->raiseFault(why);
    return false;
}

bool M2FittsSharedReachState::applySharedControl(double dt) {
    VM2 X = robot->getEndEffPosition();
    VM2 dX = robot->getEndEffVelocity();
    if (!safetyOk(X, dX)) {
        robot->setEndEffForceWithCompensation(VM2::Zero(), sm->frictionCompensation());
        return false;
    }
    VM2 Fint = robot->getInteractionForce();
    VM2 dq = robot->getVelocity();
    out_ = law_.compute(alpha_, Xref_, X, dX, sm->humanForce(), Fint, dq, dt);
    if (!out_.valid) {
        sm->raiseFault("shared-control law received an invalid input (alpha = " + std::to_string(alpha_) + ")");
        robot->setEndEffForceWithCompensation(VM2::Zero(), sm->frictionCompensation());
        return false;
    }
    //Same RobotM2 call (and so the same friction compensation) as Block 1 and the robot-alone run
    robot->setEndEffForceWithCompensation(out_.F_cmd, sm->frictionCompensation());
    sm->setLoggedControl(out_.F_pd, out_.F_cmd, (out_.cancelSaturated || out_.cmdSaturated) ? 1. : 0.);
    return true;
}

void M2FittsSharedReachState::releaseRobot() {
    VM2 X = robot->getEndEffPosition();
    VM2 dX = robot->getEndEffVelocity();
    VM2 F = channelOnly(sm, X, dX, Xorigin_(1));
    robot->setEndEffForceWithCompensation(F, sm->frictionCompensation());
    sm->setLoggedControl(VM2::Constant(fittsNaN()), F, 0.);
}

void M2FittsSharedReachState::accumulate(double dt) {
    const double dir = sm->p().taskDirection;
    const double fH = dir * sm->humanForce()(0);   //measured human force towards the targets [N]
    const double fR = dir * alpha_ * out_.F_pd(0);  //robot term towards the targets [N]
    acc_.impH += fH * dt;
    acc_.impR += fR * dt;
    acc_.FhPeak = std::max(acc_.FhPeak, std::fabs(fH));
    acc_.n++;
    const double thr = sm->p().conflictThreshold;
    if (std::fabs(fH) > thr && std::fabs(fR) > thr && fH * fR < 0.) acc_.nConflict++;
    if (out_.cancelSaturated || out_.cmdSaturated) acc_.nSat++;
}

void M2FittsSharedReachState::storeMeasures(const SharedAccum &a) {
    res_.impH = a.impH;
    res_.impR = a.impR;
    res_.FhPeak = (a.n > 0) ? a.FhPeak : fittsNaN();
    res_.conflictFrac = (a.n > 0) ? (double)a.nConflict / (double)a.n : fittsNaN();
    res_.satFrac = (a.n > 0) ? (double)a.nSat / (double)a.n : fittsNaN();
}

void M2FittsSharedReachState::duringCode(void) {
    if (trialDone_) return;                  //the transition to ReturnState happens at the next cycle
    if (!applySharedControl(dt())) return;   //safety fault raised: FaultState takes over at the next cycle

    VM2 X = robot->getEndEffPosition();
    VM2 dX = robot->getEndEffVelocity();

    //Kinematics along the task axis (Block 1)
    double x = sm->taskCoord(X);
    double xTarget = sm->taskCoord(Xtarget_);
    double v = sm->p().taskDirection * dX(0);
    res_.v_peak = std::max(res_.v_peak, std::fabs(v));

    //Movement onset: handle leaves the home region
    if (!leftHome_ && (X - Xorigin_).norm() > sm->p().homeExitRadius) {
        leftHome_ = true;
        res_.RT = running();
    }

    //Force sharing on this sample
    accumulate(dt());

    //Inside the target? (1D band if the movement is channel constrained, radial otherwise)
    double err = x - xTarget;
    bool inside = sm->p().useYChannel ? (std::fabs(err) <= halfW_) : ((X - Xtarget_).norm() <= halfW_);

    if (inside && !inTarget_) {  //(re)entry into the target: restarts the dwell
        tEntry_ = running();
        xEntry_ = err;
        res_.nEntries++;
        if (std::isnan(res_.t_first_entry)) res_.t_first_entry = tEntry_;
        accAtEntry_ = acc_;  //sums over [onset, this entry]: the MT window if this entry is the final one
    }
    inTarget_ = inside;

    const double dwell = sm->p().dwellTime;
    sm->setLoggedDwell(inside && dwell > 0. ? std::min(1., (running() - tEntry_) / dwell) : 0.);

    //Debounced interaction-force limit: the robot must not keep pushing against a resisting participant
    overForceTime_ = interactionForceAbove(robot, sm->p().reachForceLimit) ? overForceTime_ + dt() : 0.;

    if (inside && (running() - tEntry_) >= dwell) {
        //Trial validated: MT is the time of the final entry (Block 1 definition)
        res_.success = true;
        res_.MT = tEntry_;
        res_.MT_move = std::isnan(res_.RT) ? fittsNaN() : tEntry_ - res_.RT;
        res_.x_entry_cm = xEntry_ * 100.;
        res_.x_sel_cm = err * 100.;
        res_.t_end = sm->runningTime();
        storeMeasures(accAtEntry_);
        trialDone_ = true;

        sm->recordReach(res_);
        sm->sendUI("HITT", {(double)trial_.index, res_.MT, (double)res_.nEntries, res_.x_sel_cm});
        std::cout << (res_.phase == PHASE_WARMUP ? "[warm-up] " : res_.phase == PHASE_COOLDOWN ? "[cool-dn] " : "[block]   ")
                  << "trial " << std::setw(4) << trial_.index
                  << " | A=" << std::setw(8) << std::fixed << std::setprecision(4) << trial_.A_cm
                  << " W=" << std::setw(8) << trial_.W_cm
                  << " ID=" << std::setw(7) << std::setprecision(3) << trial_.ID_bits
                  << " alpha=" << std::setw(5) << std::setprecision(2) << alpha_
                  << " | MT=" << std::setw(6) << std::setprecision(3) << res_.MT << " s"
                  << (res_.nEntries > 1 ? "  (" + std::to_string(res_.nEntries) + " entries)" : "")
                  << std::endl;
    } else if (sm->p().reachForceLimit > 0. && overForceTime_ >= sm->p().forceLimitTime) {
        //Participant resisting the robot: release it at once, score the trial as a miss, continue the cycle
        releaseRobot();
        res_.success = false;
        res_.MT = fittsNaN();
        res_.t_end = sm->runningTime();
        res_.reachAbort = REACH_FORCE_LIMIT;
        storeMeasures(acc_);
        trialDone_ = true;

        sm->recordReach(res_);
        sm->sendUI("MISS", {(double)trial_.index});
        spdlog::warn("M2FittsRobotHuman: trial {} ended - interaction force above {} N for {} s. Robot released, "
                     "trial scored as a miss (reach_abort = 1).",
                     trial_.index, sm->p().reachForceLimit, sm->p().forceLimitTime);
    } else if (running() > sm->p().maxTrialTime) {
        //Time-out (Block 1): trial kept as unsuccessful, the cycle continues
        res_.success = false;
        res_.MT = fittsNaN();
        res_.t_end = sm->runningTime();
        storeMeasures(acc_);
        trialDone_ = true;

        sm->recordReach(res_);
        sm->sendUI("MISS", {(double)trial_.index});
        spdlog::warn("M2FittsRobotHuman: trial {} timed out after {} s (no validated capture).", trial_.index, sm->p().maxTrialTime);
    }
}

void M2FittsSharedReachState::exitCode(void) {
    sm->setLoggedDwell(0.);
    if (!trialDone_) {
        //Left before the trial ended (abort, safety fault or Ctrl-C): keep a row for the record
        res_.success = false;
        res_.MT = fittsNaN();
        res_.t_end = sm->runningTime();
        res_.reachAbort = REACH_INTERRUPTED;
        storeMeasures(acc_);
        trialDone_ = true;
        sm->recordReach(res_);
        sm->finaliseTrial(fittsNaN(), false, false);
    }
    robot->setEndEffForceWithCompensation(VM2::Zero(), sm->frictionCompensation());
}

/******************************************************************************
 * Return: robot moves off the origin, participant drags the handle back (Block 1)
 ******************************************************************************/
void M2FittsReturnState::entryCode(void) {
    sm->setLoggedState(ST_RETURN);
    sm->setLoggedDwell(0.);
    Xorigin_ = sm->originPosition();
    Xaway_ = sm->returnPosition();
    dragTime_ = fittsNaN();
    timedOut_ = false;
    aborted_ = false;
    returnDone_ = false;
    snapRetries_ = 0;
    overForceTime_ = 0.;
    abortedLatched_ = false;
    gotoPhase(CONFIRM);
}

bool M2FittsReturnState::forceAbort(double t) {
    if (t < sm->p().forceGraceTime) {
        overForceTime_ = 0.;
        return false;
    }
    overForceTime_ = interactionForceAbove(robot, sm->p().forceLimit) ? overForceTime_ + dt() : 0.;
    return overForceTime_ >= sm->p().forceLimitTime;
}

void M2FittsReturnState::gotoPhase(ReturnPhase p) {
    phase_ = p;
    tPhase_ = running();
    VM2 X = robot->getEndEffPosition();

    switch (p) {
        case CONFIRM:  //wait before the robot-driven move (participant can release the handle)
            robot->initTorqueControl();
            robot->setEndEffForceWithCompensation(VM2::Zero(), true);
            break;

        case MOVE_AWAY:  //robot drives the handle to returnOffset from the origin
            robot->initVelocityControl();
            Xi_ = X;
            T_ = std::max(sm->p().returnMinTime, (Xaway_ - Xi_).norm() / sm->p().returnSpeed);
            sm->sendUIContext("RETN", {Xaway_(0), Xaway_(1)});
            overForceTime_ = 0.;
            break;

        case DRAG:  //participant drags the handle back: self-paced inter-trial rest
            robot->initTorqueControl();
            robot->setEndEffForceWithCompensation(VM2::Zero(), true);
            settleTime_ = 0.;
            sm->sendUIContext("DRAG", {Xorigin_(0), Xorigin_(1), sm->p().originTolerance, sm->p().maxReturnTime, X(0), X(1)});
            break;

        case SNAP:  //robot positions the handle on the exact origin
            robot->initVelocityControl();
            Xi_ = X;
            T_ = std::max(sm->p().originSnapTime, (Xorigin_ - Xi_).norm() / sm->p().returnSpeed);
            break;

        case HOLD:  //held on the origin until the next target onset
            sm->sendUIContext("ORIG", {Xorigin_(0), Xorigin_(1)});
            break;
    }
}

void M2FittsReturnState::duringCode(void) {
    VM2 X = robot->getEndEffPosition();
    VM2 dX = robot->getEndEffVelocity();
    double t = running() - tPhase_;
    VM2 Xd, dXd;

    switch (phase_) {
        case CONFIRM: {
            robot->setEndEffForceWithCompensation(channelOnly(sm, X, dX, Xorigin_(1)), true);
            if (t >= sm->p().successHoldTime) gotoPhase(MOVE_AWAY);
            break;
        }

        case MOVE_AWAY: {
            double status = JerkIt(Xi_, Xaway_, T_, t, Xd, dXd);
            double k = (status >= 1.) ? sm->p().kHold : sm->p().kPosVel;
            robot->setEndEffVelocity(dXd + k * (Xd - X));

            if (forceAbort(t)) {
                aborted_ = true;
                abortedLatched_ = true;
                spdlog::warn("M2FittsRobotHuman: MOVE_AWAY aborted at x={:.4f} m, |F|={:.1f}.", X(0), robot->getInteractionForce().norm());
                gotoPhase(DRAG);
            } else {
                bool arrived = std::fabs(sm->taskCoord(X) - sm->taskCoord(Xaway_)) <= sm->p().awayTolerance;
                bool slow = std::fabs(dX(0)) <= sm->p().awaySettleSpeed;
                if (status >= 1. && arrived && slow) {
                    spdlog::info("M2FittsRobotHuman: MOVE_AWAY converged {:+.2f} mm from the away point after {:.2f} s.",
                                 (sm->taskCoord(X) - sm->taskCoord(Xaway_)) * 1000., t);
                    gotoPhase(DRAG);
                } else if (t > T_ + sm->p().maxMoveExtraTime) {
                    spdlog::warn("M2FittsRobotHuman: MOVE_AWAY did not converge - stopped {:.1f} mm from the away point "
                                 "(x={:.4f} m, target {:.4f} m).",
                                 (sm->taskCoord(X) - sm->taskCoord(Xaway_)) * 1000., X(0), Xaway_(0));
                    gotoPhase(DRAG);
                }
            }
            break;
        }

        case DRAG: {
            robot->setEndEffForceWithCompensation(channelOnly(sm, X, dX, Xorigin_(1)), true);

            double d = sm->p().useYChannel ? std::fabs(sm->taskCoord(X) - sm->taskCoord(Xorigin_)) : (X - Xorigin_).norm();
            double speed = sm->p().useYChannel ? std::fabs(dX(0)) : dX.norm();

            if (d <= sm->p().originTolerance && speed <= sm->p().originSettleSpeed) {
                settleTime_ += dt();
                if (settleTime_ >= sm->p().originSettleTime) {
                    dragTime_ = t;
                    aborted_ = false;
                    gotoPhase(SNAP);
                }
            } else {
                settleTime_ = 0.;
                aborted_ = false;
            }
            break;
        }

        case SNAP: {
            double status = JerkIt(Xi_, Xorigin_, T_, t, Xd, dXd);
            double k = (status >= 1.) ? sm->p().kHold : sm->p().kPosVel;
            robot->setEndEffVelocity(dXd + k * (Xd - X));

            if (forceAbort(t) && snapRetries_ < 2) {
                aborted_ = true;
                abortedLatched_ = true;
                snapRetries_++;
                spdlog::warn("M2FittsRobotHuman: repositioning aborted (interaction force > {} N), retry {}.", sm->p().forceLimit, snapRetries_);
                gotoPhase(DRAG);
            } else {
                bool arrived = (X - Xorigin_).norm() <= sm->p().awayTolerance;
                bool slow = dX.norm() <= sm->p().awaySettleSpeed;
                if (status >= 1. && arrived && slow) {
                    gotoPhase(HOLD);
                } else if (t > T_ + sm->p().maxMoveExtraTime) {
                    spdlog::warn("M2FittsRobotHuman: SNAP did not converge - trial will start {:.1f} mm off the origin.",
                                 (X - Xorigin_).norm() * 1000.);
                    gotoPhase(HOLD);
                }
            }
            break;
        }

        case HOLD: {
            if (aborted_)
                robot->setEndEffVelocity(VM2::Zero());
            else
                robot->setEndEffVelocity(sm->p().kPosVel * (Xorigin_ - X));

            if (t >= sm->p().originHoldTime) {
                sm->finaliseTrial(dragTime_, timedOut_, abortedLatched_);
                returnDone_ = true;
            }
            break;
        }
    }
}

void M2FittsReturnState::exitCode(void) {
    robot->setEndEffVelocity(VM2::Zero());
}

/******************************************************************************
 * Break between rounds (Block 1)
 ******************************************************************************/
void M2FittsBreakState::entryCode(void) {
    sm->setLoggedState(ST_BREAK);
    sm->setLoggedDwell(0.);
    duration_ = sm->breakDuration();
    sm->clearBreakDue();
    over_ = false;
    //A go pressed earlier (e.g. SPACE on the display during the round) stays latched in the UI link until it is
    //read, and would end this break on its first cycle. Discard it: only a go given during the break counts.
    if (sm->ui.consumeGo()) spdlog::info("M2FittsRobotHuman: a go pressed before the break was discarded.");

    robot->initTorqueControl();
    robot->setEndEffForceWithCompensation(VM2::Zero(), true);
    sm->sendUIContext("REST", {duration_, (double)sm->trialsDone(), sm->uiPhase()});
    std::cout << "\n--- Break: up to " << duration_ << " s. Press 's' (or the skip control in the UI) to continue earlier. ---" << std::endl;
}

void M2FittsBreakState::duringCode(void) {
    robot->setEndEffForceWithCompensation(VM2::Zero(), true);

    if (iterations() % 2500 == 1)
        std::cout << "   " << std::fixed << std::setprecision(0) << std::max(0., duration_ - running()) << " s left..." << std::endl;

    if (running() >= duration_) {
        over_ = true;
        spdlog::info("M2FittsRobotHuman: break over ({:.0f} s).", running());
    } else if (running() >= sm->p().roundBreakMinTime) {
        if (const char *src = sm->goSource()) {
            over_ = true;
            spdlog::info("M2FittsRobotHuman: break ended early, after {:.1f} s, by {}.", running(), src);
        }
    } else {
        sm->ui.consumeGo();  //a go given before round_break_min_time is ignored
    }
}

void M2FittsBreakState::exitCode(void) {
    robot->setEndEffForceWithCompensation(VM2::Zero());
}

/******************************************************************************
 * End of block (Block 1)
 ******************************************************************************/
void M2FittsEndState::entryCode(void) {
    sm->setLoggedState(ST_END);
    sm->setLoggedDwell(0.);
    robot->initTorqueControl();
    sm->sendUIContext("ENDE", {(double)sm->trialsDone()});
    sm->printSummary();
    std::cout << "Block " << sm->blockNb() << " finished: " << sm->trialsDone()
              << " trials recorded. The robot is transparent - you can stop CORC (Ctrl-C)." << std::endl;
}

void M2FittsEndState::duringCode(void) {
    robot->setEndEffForceWithCompensation(VM2::Zero(), true);
}

void M2FittsEndState::exitCode(void) {
    robot->setEndEffForceWithCompensation(VM2::Zero());
}

/******************************************************************************
 * Safety fault (Block 2): damping brake until the app is stopped
 ******************************************************************************/
void M2FittsFaultState::brake() {
    VM2 v = robot->getEndEffVelocity();
    if (!v.allFinite()) v.setZero();
    VM2 F = -sm->p().brakeDamping * v;
    for (int i = 0; i < 2; ++i) F(i) = std::max(-sm->p().brakeFMax, std::min(sm->p().brakeFMax, F(i)));
    robot->setJointTorque(F);  //M2: J = I, so joint force = end-effector force (works even if calibration is lost)
}

void M2FittsFaultState::entryCode(void) {
    sm->setLoggedState(ST_FAULT);
    sm->setLoggedDwell(0.);
    sm->acknowledgeFault();
    robot->initTorqueControl();
    brake();
    sm->sendUIContext("STBY");
    spdlog::critical("M2FittsRobotHuman FAULT: {}. Damping brake on; completed trials are saved. "
                     "Stop CORC (Ctrl-C) and restart to continue ('x' switches to transparent standby).",
                     sm->faultReason());
}

void M2FittsFaultState::duringCode(void) {
    brake();
}

void M2FittsFaultState::exitCode(void) {
    robot->setEndEffForceWithCompensation(VM2::Zero(), sm->frictionCompensation());
}