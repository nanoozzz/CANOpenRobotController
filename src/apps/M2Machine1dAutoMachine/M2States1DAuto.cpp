#include "M2States1DAuto.h"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <csignal>

#include "M2Machine1DAuto.h"

namespace {

//! Fifth-order minimum-jerk interpolation. dXd is the exact time derivative of Xd.
double minJerk(const VM2 &X0, const VM2 &Xf, double T, double t, VM2 &Xd, VM2 &dXd) {
    if (t <= 0.0) { Xd = X0; dXd = VM2::Zero(); return 0.0; }
    if (t >= T)   { Xd = Xf; dXd = VM2::Zero(); return 1.0; }
    const double s = t / T, s2 = s * s, s3 = s2 * s, s4 = s3 * s, s5 = s4 * s;
    Xd  = X0 + (Xf - X0) * (10.0 * s3 - 15.0 * s4 + 6.0 * s5);
    dXd = (Xf - X0) * ((30.0 * s2 - 60.0 * s3 + 30.0 * s4) / T);
    return s;
}

//! Hold the off-axis on the home line with a stiff virtual constraint.
double offAxisForce(const VM2 &X, const VM2 &dX) {
    const double e = exp1d::HOME[exp1d::OFF_AXIS] - X(exp1d::OFF_AXIS);
    double F = exp1d::K_OFF * e - exp1d::B_OFF * dX(exp1d::OFF_AXIS);
    return std::clamp(F, -exp1d::F_MAX_OFF, exp1d::F_MAX_OFF);
}

}  // namespace

/* ===================================================================== */
/* CALIBRATION                                                            */
/* ===================================================================== */
Calib1D::Calib1D(RobotM2 *robot, StateMachine *machine)
    : State("Calib"), robot(robot), machine(machine) {}

void Calib1D::entry() {
    calibDone = false;
    stopTimer[0] = stopTimer[1] = 0.0;
    robot->initTorqueControl();
    static_cast<M2Machine1DAuto *>(machine)->logPhase = exp1d::PHASE_CALIB;
    spdlog::info("Calib1D: pushing to stops. Keep the workspace clear.");
}

void Calib1D::during() {
    VM2 vel = robot->getVelocity();
    VM2 tau;
    for (int i = 0; i < 2; i++) {
        tau(i) = -std::clamp(2.0 - 0.1 * vel(i), -2.0, 2.0);
        if (std::fabs(vel(i)) < 0.005)
            stopTimer[i] += dt();
        else
            stopTimer[i] = 0.0;
    }
    if (stopTimer[0] > 1.0 && stopTimer[1] > 1.0) {
        robot->applyCalibration();
        calibDone = true;
        spdlog::info("Calib1D: calibration applied after {:.2f} s.", running());
    } else {
        robot->setJointTorque(tau);
    }
}

void Calib1D::exit() { robot->setJointTorque(VM2::Zero()); }

/* ===================================================================== */
/* HOME                                                                   */
/* ===================================================================== */
Home1D::Home1D(RobotM2 *robot, StateMachine *machine)
    : State("Home"), robot(robot), machine(machine) {}

void Home1D::entry() {
    Xi = robot->getEndEffPosition();
    ready = false;
    // Control mode is set HERE and nowhere downstream: initTorqueControl blocks
    // for ~20 ms, and calling it in Move1D::entry would inject that delay
    // immediately before the timed movement and bias MT.
    robot->initTorqueControl();
    auto *m = static_cast<M2Machine1DAuto *>(machine);
    m->logPhase = exp1d::PHASE_HOME;
    spdlog::debug("Home1D: returning to home for trial {}.", m->currentTrial().index);
}

void Home1D::during() {
    const VM2 X  = robot->getEndEffPosition();
    const VM2 dX = robot->getEndEffVelocity();

    VM2 Xhome;
    Xhome(0) = exp1d::HOME[0];
    Xhome(1) = exp1d::HOME[1];

    VM2 Xd, dXd;
    const double progress = minJerk(Xi, Xhome, exp1d::HOME_DURATION, running(), Xd, dXd);

    VM2 F = exp1d::K_TASK * (Xd - X) + exp1d::B_TASK * (dXd - dX);
    if (F.norm() > exp1d::F_MAX) F = F.normalized() * exp1d::F_MAX;
    robot->setEndEffForceWithCompensation(F);

    const bool settled = (X - Xhome).norm() < exp1d::HOME_TOL && dX.norm() < exp1d::HOME_V_TOL;
    if (progress >= 1.0 && settled) {
        ready = true;
    } else if (running() > exp1d::HOME_TIMEOUT) {
        spdlog::warn("Home1D: timeout, {:.1f} mm from home. Continuing anyway.",
                     1000.0 * (X - Xhome).norm());
        ready = true;
    }
}

void Home1D::exit() {}

/* ===================================================================== */
/* TIMED MOVEMENT                                                         */
/* ===================================================================== */
Move1D::Move1D(RobotM2 *robot, StateMachine *machine)
    : State("Move"), robot(robot), machine(machine) {}

void Move1D::entry() {
    auto *m = static_cast<M2Machine1DAuto *>(machine);
    const Trial &tr = m->currentTrial();

    target = exp1d::HOME[exp1d::TASK_AXIS] + exp1d::DIRECTION * tr.A;
    W      = tr.W;

    MT = -1.0; tOnset = -1.0; peakV = 0.0; tPeakV = -1.0;
    finalError = 0.0; maxOffAxis = 0.0;
    bandEntries = 0; satSamples = 0; nSamples = 0;
    done = false; onsetDetected = false; wasInBand = false; timedOut = false;
    settleStart = -1.0;

    m->logPhase     = exp1d::PHASE_MOVE;
    m->logTargetPos = target;
    m->logW         = W;

    spdlog::debug("Move1D: trial {} A={:.4f} W={:.4f} ID={:.2f} target={:.4f}",
                  tr.index, tr.A, tr.W, tr.IDfitts, target);
}

void Move1D::during() {
    const VM2 X  = robot->getEndEffPosition();
    const VM2 dX = robot->getEndEffVelocity();
    const double x = X(exp1d::TASK_AXIS);
    const double v = dX(exp1d::TASK_AXIS);
    const double t = running();

    // ---- control: step target, critically damped PD, saturated ----
    const double e = target - x;
    double F = exp1d::K_TASK * e - exp1d::B_TASK * v;
    if (std::fabs(F) > exp1d::F_MAX) {
        F = std::copysign(exp1d::F_MAX, F);
        satSamples++;
    }

    VM2 Fcmd;
    Fcmd(exp1d::TASK_AXIS) = F;
    Fcmd(exp1d::OFF_AXIS)  = offAxisForce(X, dX);
    robot->setEndEffForceWithCompensation(Fcmd);

    // ---- measurement ----
    nSamples++;
    if (std::fabs(v) > peakV) { peakV = std::fabs(v); tPeakV = t; }
    const double offErr = std::fabs(X(exp1d::OFF_AXIS) - exp1d::HOME[exp1d::OFF_AXIS]);
    if (offErr > maxOffAxis) maxOffAxis = offErr;

    const bool inBand = std::fabs(e) <= 0.5 * W;

    if (!onsetDetected) {
        if (std::fabs(v) >= exp1d::V_ONSET) {
            onsetDetected = true;
            tOnset = t;
        }
    } else {
        if (inBand && !wasInBand) bandEntries++;
        if (inBand && std::fabs(v) <= exp1d::V_OFFSET) {
            if (settleStart < 0.0) settleStart = t;
            if (t - settleStart >= exp1d::DWELL) {
                MT = settleStart - tOnset;  // offset = moment the settled window began
                finalError = e;
                done = true;
            }
        } else {
            settleStart = -1.0;
        }
    }
    wasInBand = inBand;

    if (!done && t >= exp1d::TIMEOUT) {
        timedOut = true;
        MT = -1.0;
        finalError = e;
        done = true;
        spdlog::warn("Move1D: trial {} timed out {:.1f} mm from target (onset {}).",
                     static_cast<M2Machine1DAuto *>(machine)->currentTrial().index,
                     1000.0 * std::fabs(e), onsetDetected ? "detected" : "NEVER DETECTED");
    }
}

void Move1D::exit() {}

/* ===================================================================== */
/* INTER-TRIAL INTERVAL                                                   */
/* ===================================================================== */
ITI1D::ITI1D(RobotM2 *robot, StateMachine *machine)
    : State("ITI"), robot(robot), machine(machine) {}

void ITI1D::entry() {
    done = false;
    auto *m  = static_cast<M2Machine1DAuto *>(machine);
    auto mv  = m->state<Move1D>("Move");
    holdTarget = mv->target;
    m->logPhase = exp1d::PHASE_ITI;

    m->writeTrialRecord(*mv);
    m->advanceTrial();
}

void ITI1D::during() {
    const VM2 X  = robot->getEndEffPosition();
    const VM2 dX = robot->getEndEffVelocity();

    double F = exp1d::K_TASK * (holdTarget - X(exp1d::TASK_AXIS)) -
               exp1d::B_TASK * dX(exp1d::TASK_AXIS);
    F = std::clamp(F, -exp1d::F_MAX, exp1d::F_MAX);

    VM2 Fcmd;
    Fcmd(exp1d::TASK_AXIS) = F;
    Fcmd(exp1d::OFF_AXIS)  = offAxisForce(X, dX);
    robot->setEndEffForceWithCompensation(Fcmd);

    if (running() >= exp1d::ITI) done = true;
}

void ITI1D::exit() {}

/* ===================================================================== */
/* END                                                                    */
/* ===================================================================== */
End1D::End1D(RobotM2 *robot, StateMachine *machine)
    : State("End"), robot(robot), machine(machine) {}

void End1D::entry() {
    robot->setEndEffForceWithCompensation(VM2::Zero());
    auto *m = static_cast<M2Machine1DAuto *>(machine);
    m->logPhase = exp1d::PHASE_END;
    m->closeRecords();
    spdlog::info("End1D: all trials complete. Requesting shutdown.");
    std::raise(SIGTERM);
}

void End1D::during() { robot->setEndEffForceWithCompensation(VM2::Zero()); }

void End1D::exit() {}