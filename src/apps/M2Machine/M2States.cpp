#include "M2States.h"
#include "M2Machine.h"
#include <spdlog/spdlog.h>
#include <algorithm> 
#include <cmath>    

static double MinJerk(const VM2& X0, const VM2& Xf, double T, double t, VM2& Xd, VM2& dXd) {
    if (t <= 0) { Xd = X0; dXd = VM2::Zero(); return 0.0; }
    if (t >= T) { Xd = Xf; dXd = VM2::Zero(); return 1.0; }
    
    double s = t / T;
    double s2 = s*s;
    double s3 = s2*s;
    double s4 = s3*s;
    double s5 = s4*s;
    
    Xd  = X0 + (Xf - X0) * (10.0*s3 - 15.0*s4 + 6.0*s5);
    dXd = (Xf - X0) * ((30.0*s2 - 60.0*s3 + 30.0*s4) / T);
    
    return s;
}

/* =====================================================================
 * CALIBRATION STATE
 * ===================================================================== */
M2CalibState::M2CalibState(RobotM2 *robot, StateMachine *machine) 
    : State("CalibState"), robot(robot), machine(machine) {}

void M2CalibState::entry() {
    calibDone = false;
    robot->initTorqueControl();
    spdlog::info("M2CalibState: Starting Calibration...");
    
    stop_timer[0] = 0.0;
    stop_timer[1] = 0.0;
}

void M2CalibState::during() {
    VM2 vel = robot->getVelocity(); 
    VM2 tau;
    
    for(int i=0; i<2; i++) {
        tau(i) = -std::clamp(2.0 - 0.1 * vel(i), -2.0, 2.0); 
        
        if(std::abs(vel(i)) < 0.005) {
            stop_timer[i] += dt(); 
        } else {
            stop_timer[i] = 0.0;
        }
    }

    if(stop_timer[0] > 1.0 && stop_timer[1] > 1.0) {
        robot->applyCalibration();
        calibDone = true;
        spdlog::info("Calibration Complete.");
    } else {
        robot->setJointTorque(tau);
    }
}

void M2CalibState::exit() {
    robot->setJointTorque(VM2::Zero());
}

/* =====================================================================
 * STANDBY STATE
 * ===================================================================== */
M2StandbyState::M2StandbyState(RobotM2 *robot, StateMachine *machine) 
    : State("StandbyState"), robot(robot), machine(machine) {}

void M2StandbyState::entry() {
    robot->initTorqueControl();
    robot->setEndEffForceWithCompensation(VM2::Zero()); 
    goToReset = false;
    spdlog::info("Entering Standby: Robot is transparent.");
}

void M2StandbyState::during() {
    robot->setEndEffForceWithCompensation(VM2::Zero());

    M2Machine* m = static_cast<M2Machine*>(machine);
    if (m->UIserver && m->UIserver->isCmd()) {
        std::string cmd; std::vector<double> args;
        m->UIserver->getCmd(cmd, args);
        
        if (cmd == "RSTA") {
            spdlog::info("Received RSTA -> Transitioning to Reset");
            goToReset = true; 
        }
        m->UIserver->clearCmd();
    }
}

void M2StandbyState::exit() {}

/* =====================================================================
 * RESET STATE
 * ===================================================================== */
M2ResetState::M2ResetState(RobotM2 *robot, StateMachine *machine) 
    : State("ResetState"), robot(robot), machine(machine) {}

void M2ResetState::entry() {
    Xi = robot->getEndEffPosition(); 
    finished = false;
    spdlog::info("ResetState: Homing to workspace center...");
}

void M2ResetState::during() {
    VM2 X = robot->getEndEffPosition();
    VM2 dX = robot->getEndEffVelocity();
    VM2 targetCenter = {0.32, 0.20}; 
    VM2 Xd, dXd; 

    double t = running();
    double duration = 2.0;
    double progress = MinJerk(Xi, targetCenter, duration, t, Xd, dXd);

    VM2 F_cmd = 400.0 * (Xd - X) + 20.0 * (dXd - dX);
    
    double maxF = 40.0;
    if (F_cmd.norm() > maxF) F_cmd = F_cmd.normalized() * maxF;

    robot->setEndEffForceWithCompensation(F_cmd);

    if (progress >= 1.0 && (X - targetCenter).norm() < 0.005 && dX.norm() < 0.02) {
        finished = true;
    }
}

void M2ResetState::exit() {
    robot->setEndEffForceWithCompensation(VM2::Zero());
    spdlog::info("ResetState: Center Reached.");
}