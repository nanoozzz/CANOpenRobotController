#include "M2FittsHumanStates.h"

#include <algorithm>
#include <iomanip>
#include <iostream>

#include "M2FittsHumanMachine.h"

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
    //Derivative w.r.t. time: d/dt = (d/dtn)/T -- defined at t=0, unlike a division by t
    dXd = (X0 - Xf) * (60. * tn3 - 30. * tn4 - 30. * tn2) / T;
    return tn;
}

/**
 * \brief True if the measured interaction force exceeds the safety limit (0 disables the check).
 */
static bool interactionForceTooHigh(RobotM2 *robot, const FittsParams &p) {
    if (p.forceLimit <= 0) return false;
    VM2 F = robot->getInteractionForce();
    return F.norm() > p.forceLimit;
}

/******************************************************************************
 * Calibration (same procedure as M2DemoMachine: constant torque onto the stops)
 ******************************************************************************/
void M2FittsCalibState::entryCode(void) {
    sm->setLoggedState(ST_CALIB);
    calibDone = false;
    for (unsigned int i = 0; i < 2; i++) {
        stop_reached_time[i] = .0;
        at_stop[i] = false;
    }
    robot->decalibrate();
    robot->initTorqueControl();
    robot->printJointStatus();
    std::cout << "Calibrating (keep clear)..." << std::flush;
}

void M2FittsCalibState::duringCode(void) {
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
        calibDone = true;  //Trigger event
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
    robot->setEndEffForceWithCompensation(VM2::Zero());
}

/******************************************************************************
 * Standby: transparent, no task (abort target)
 ******************************************************************************/
void M2FittsStandbyState::entryCode(void) {
    sm->setLoggedState(ST_STANDBY);
    robot->initTorqueControl();
    spdlog::warn("M2FittsHuman: session interrupted - robot transparent. Completed trials are saved.");
}

void M2FittsStandbyState::duringCode(void) {
    robot->setEndEffForceWithCompensation(VM2::Zero(), true);
}

void M2FittsStandbyState::exitCode(void) {
    robot->setEndEffForceWithCompensation(VM2::Zero());
}

/******************************************************************************
 * Ready: bring the handle to the origin and wait for the go signal
 ******************************************************************************/
void M2FittsReadyState::entryCode(void) {
    sm->setLoggedState(ST_READY);
    robot->initVelocityControl();
    robot->setEndEffVelocity(VM2::Zero());

    Xi_ = robot->getEndEffPosition();
    Xorigin_ = sm->originPosition();
    T_ = std::max(sm->p().returnMinTime, (Xorigin_ - Xi_).norm() / sm->p().returnSpeed);

    requireGo_ = sm->requireGoSignal();
    atOrigin_ = false;
    tAtOrigin_ = 0.;
    ready_ = false;

    sm->sendUI("RDY!", {requireGo_ ? 1. : 0., Xorigin_(0), Xorigin_(1)});
    if (requireGo_)
        std::cout << "\nHold the handle. Press 's' (or joystick button 1, or send GTNS from the UI) to start.\n" << std::endl;
    else
        std::cout << "Resuming in " << sm->p().readyHoldTime << " s (hold the handle at the start position)..." << std::endl;
}

void M2FittsReadyState::duringCode(void) {
    //Min jerk move to the origin, then hold on it (position over velocity loop)
    VM2 X = robot->getEndEffPosition();
    VM2 Xd, dXd;
    double status = JerkIt(Xi_, Xorigin_, T_, running(), Xd, dXd);
    robot->setEndEffVelocity(dXd + sm->p().kPosVel * (Xd - X));

    if (status >= 1. && !atOrigin_) {
        atOrigin_ = true;
        tAtOrigin_ = running();
        sm->sendUI("ORIG", {Xorigin_(0), Xorigin_(1)});
    }

    //Go on explicit signal, or automatically once held at the origin (between rounds)
    if (sm->goSignal())
        ready_ = true;
    else if (atOrigin_ && !requireGo_ && (running() - tAtOrigin_) >= sm->p().readyHoldTime)
        ready_ = true;
}

void M2FittsReadyState::exitCode(void) {
    robot->setEndEffVelocity(VM2::Zero());
}

/******************************************************************************
 * Reach: one trial (target onset -> dwell validated capture)
 ******************************************************************************/
void M2FittsReachState::entryCode(void) {
    sm->setLoggedState(ST_REACH);

    trial_ = sm->currentTrial();
    Xorigin_ = sm->originPosition();
    Xtarget_ = sm->targetPosition(trial_);
    halfW_ = trial_.halfW();
    sm->setLoggedTarget(Xtarget_(0), halfW_);

    //"No robot support": transparent (mass and friction compensation only)
    robot->initTorqueControl();
    robot->setEndEffForceWithCompensation(VM2::Zero(), true);

    trialDone_ = false;
    leftHome_ = false;
    inTarget_ = false;
    tEntry_ = 0.;
    xEntry_ = 0.;

    res_ = FittsTrialResult();
    res_.phase = sm->phase();
    res_.trial = trial_;
    res_.t_onset = sm->runningTime();

    //Target onset (the UI draws the target from these parameters)
    sm->sendUI("TRIA", {(double)sm->phase(), (double)trial_.round, (double)trial_.index,
                        trial_.A_cm, trial_.W_cm, trial_.ID_bits,
                        Xtarget_(0), Xtarget_(1), halfW_, Xorigin_(0), Xorigin_(1)});
}

void M2FittsReachState::duringCode(void) {
    VM2 X = robot->getEndEffPosition();
    VM2 dX = robot->getEndEffVelocity();

    //Transparent along the task axis; optional virtual channel holding y (1D task constraint)
    VM2 F = VM2::Zero();
    if (sm->p().useYChannel) F(1) = sm->p().channelK * (Xorigin_(1) - X(1)) - sm->p().channelD * dX(1);
    robot->setEndEffForceWithCompensation(F, true);

    //Kinematics along the task axis
    double x = sm->taskCoord(X);
    double xTarget = sm->taskCoord(Xtarget_);
    double v = sm->p().taskDirection * dX(0);
    res_.v_peak = std::max(res_.v_peak, std::fabs(v));

    //Movement onset: handle leaves the home region
    if (!leftHome_ && (X - Xorigin_).norm() > sm->p().homeExitRadius) {
        leftHome_ = true;
        res_.RT = running();
    }

    //Inside the target? (1D band if the movement is channel constrained, radial otherwise)
    double err = x - xTarget;  //signed error re. target centre, positive beyond the centre
    bool inside = sm->p().useYChannel ? (std::fabs(err) <= halfW_) : ((X - Xtarget_).norm() <= halfW_);

    if (inside && !inTarget_) {  //(re)entry into the target: restarts the dwell
        tEntry_ = running();
        xEntry_ = err;
        res_.nEntries++;
        if (std::isnan(res_.t_first_entry)) res_.t_first_entry = tEntry_;
    }
    inTarget_ = inside;

    if (inside && (running() - tEntry_) >= sm->p().dwellTime) {
        //Trial validated: MT is the time of the *final* entry, i.e. trial duration minus the dwell
        res_.success = true;
        res_.MT = tEntry_;
        res_.MT_move = std::isnan(res_.RT) ? fittsNaN() : tEntry_ - res_.RT;
        res_.x_entry_cm = xEntry_ * 100.;
        res_.x_sel_cm = err * 100.;
        res_.t_end = sm->runningTime();
        trialDone_ = true;

        sm->recordReach(res_);
        sm->sendUI("HITT", {(double)trial_.index, res_.MT});
        std::cout << (res_.phase == PHASE_WARMUP ? "[warm-up] " : "[block]   ")
                  << "trial " << std::setw(4) << trial_.index
                  << " | A=" << std::setw(8) << std::fixed << std::setprecision(4) << trial_.A_cm
                  << " W=" << std::setw(8) << trial_.W_cm
                  << " ID=" << std::setw(7) << std::setprecision(3) << trial_.ID_bits
                  << " | MT=" << std::setw(6) << std::setprecision(3) << res_.MT << " s"
                  << (res_.nEntries > 1 ? "  (" + std::to_string(res_.nEntries) + " entries)" : "")
                  << std::endl;
    } else if (running() > sm->p().maxTrialTime) {
        //Time-out: trial kept in the log as unsuccessful, the cycle continues
        res_.success = false;
        res_.MT = fittsNaN();
        res_.t_end = sm->runningTime();
        trialDone_ = true;

        sm->recordReach(res_);
        sm->sendUI("MISS", {(double)trial_.index});
        spdlog::warn("M2FittsHuman: trial {} timed out after {} s (no validated capture).", trial_.index, sm->p().maxTrialTime);
    }
}

void M2FittsReachState::exitCode(void) {
    robot->setEndEffForceWithCompensation(VM2::Zero());
}

/******************************************************************************
 * Return: robot moves off the origin, participant drags the handle back
 ******************************************************************************/
void M2FittsReturnState::entryCode(void) {
    sm->setLoggedState(ST_RETURN);
    Xorigin_ = sm->originPosition();
    Xaway_ = sm->returnPosition();
    dragTime_ = fittsNaN();
    timedOut_ = false;
    aborted_ = false;
    returnDone_ = false;
    snapRetries_ = 0;
    gotoPhase(MOVE_AWAY);
}

void M2FittsReturnState::gotoPhase(ReturnPhase p) {
    phase_ = p;
    tPhase_ = running();
    VM2 X = robot->getEndEffPosition();

    switch (p) {
        case MOVE_AWAY:  //robot drives the handle to returnOffset from the origin
            robot->initVelocityControl();
            Xi_ = X;
            T_ = std::max(sm->p().returnMinTime, (Xaway_ - Xi_).norm() / sm->p().returnSpeed);
            sm->sendUI("RETN", {Xaway_(0), Xaway_(1)});
            break;

        case DRAG:  //participant drags the handle back: self-paced inter-trial rest
            robot->initTorqueControl();
            robot->setEndEffForceWithCompensation(VM2::Zero(), true);
            sm->sendUI("DRAG", {Xorigin_(0), Xorigin_(1), sm->p().originTolerance, sm->p().maxReturnTime});
            break;

        case SNAP:  //robot positions the handle on the exact origin
            robot->initVelocityControl();
            Xi_ = X;
            T_ = std::max(sm->p().originSnapTime, (Xorigin_ - Xi_).norm() / sm->p().returnSpeed);
            break;

        case HOLD:  //held on the origin until the next target onset
            sm->sendUI("ORIG", {Xorigin_(0), Xorigin_(1)});
            break;
    }
}

void M2FittsReturnState::duringCode(void) {
    VM2 X = robot->getEndEffPosition();
    VM2 dX = robot->getEndEffVelocity();
    double t = running() - tPhase_;
    VM2 Xd, dXd;

    switch (phase_) {
        case MOVE_AWAY: {
            double status = JerkIt(Xi_, Xaway_, T_, t, Xd, dXd);
            robot->setEndEffVelocity(dXd + sm->p().kPosVel * (Xd - X));
            if (interactionForceTooHigh(robot, sm->p())) {
                //The participant is resisting: stop driving and let them bring the handle back
                aborted_ = true;
                spdlog::warn("M2FittsHuman: robot-driven move aborted (interaction force > {} N).", sm->p().forceLimit);
                gotoPhase(DRAG);
            } else if (status >= 1.) {
                gotoPhase(DRAG);
            }
            break;
        }

        case DRAG: {
            VM2 F = VM2::Zero();
            if (sm->p().useYChannel) F(1) = sm->p().channelK * (Xorigin_(1) - X(1)) - sm->p().channelD * dX(1);
            robot->setEndEffForceWithCompensation(F, true);

            double d = sm->p().useYChannel ? std::fabs(sm->taskCoord(X) - sm->taskCoord(Xorigin_)) : (X - Xorigin_).norm();
            if (d <= sm->p().originTolerance) {  //back within tolerance: the robot takes over for the last mm
                dragTime_ = t;
                gotoPhase(SNAP);
            } else if (t >= sm->p().maxReturnTime) {  //rest cap reached: the robot returns the handle itself
                dragTime_ = t;
                timedOut_ = true;
                spdlog::warn("M2FittsHuman: handle not returned within {} s - robot repositioning.", sm->p().maxReturnTime);
                gotoPhase(SNAP);
            }
            break;
        }

        case SNAP: {
            double status = JerkIt(Xi_, Xorigin_, T_, t, Xd, dXd);
            robot->setEndEffVelocity(dXd + sm->p().kPosVel * (Xd - X));
            if (interactionForceTooHigh(robot, sm->p()) && snapRetries_ < 2) {
                aborted_ = true;
                snapRetries_++;
                spdlog::warn("M2FittsHuman: repositioning aborted (interaction force > {} N), retry {}.", sm->p().forceLimit, snapRetries_);
                gotoPhase(DRAG);
            } else if (status >= 1.) {
                gotoPhase(HOLD);
            }
            break;
        }

        case HOLD: {
            if (aborted_)
                robot->setEndEffVelocity(VM2::Zero());  //do not fight the participant after a force abort
            else
                robot->setEndEffVelocity(sm->p().kPosVel * (Xorigin_ - X));

            if (t >= sm->p().originHoldTime) {
                //Trial cycle complete: write the row and move the protocol on
                sm->finaliseTrial(dragTime_, timedOut_, aborted_);
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
 * Break between rounds (and rest between warm-up and block)
 ******************************************************************************/
void M2FittsBreakState::entryCode(void) {
    sm->setLoggedState(ST_BREAK);
    duration_ = sm->breakDuration();
    sm->clearBreakDue();
    over_ = false;

    robot->initTorqueControl();
    robot->setEndEffForceWithCompensation(VM2::Zero(), true);
    sm->sendUI("REST", {duration_, (double)sm->trialsDone()});
    std::cout << "\n--- Break: up to " << duration_ << " s. Press 's' (or send SKIP) to continue earlier. ---" << std::endl;
}

void M2FittsBreakState::duringCode(void) {
    robot->setEndEffForceWithCompensation(VM2::Zero(), true);

    if (iterations() % 2500 == 1)  //~ every 5 s at a 2 ms control period
        std::cout << "   " << std::fixed << std::setprecision(0) << std::max(0., duration_ - running()) << " s left..." << std::endl;

    if (running() >= duration_ || sm->goSignal()) over_ = true;
}

void M2FittsBreakState::exitCode(void) {
    robot->setEndEffForceWithCompensation(VM2::Zero());
}

/******************************************************************************
 * End of block
 ******************************************************************************/
void M2FittsEndState::entryCode(void) {
    sm->setLoggedState(ST_END);
    robot->initTorqueControl();
    sm->sendUI("ENDE", {(double)sm->trialsDone()});
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