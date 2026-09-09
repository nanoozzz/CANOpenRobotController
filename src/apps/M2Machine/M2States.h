#ifndef M2STATES_H
#define M2STATES_H

#include "State.h"
#include "RobotM2.h"

class M2Machine;
class StateMachine;

class M2CalibState : public State {
public:
    M2CalibState(RobotM2 *robot, StateMachine *machine);
    void entry() override; // version mismatch, changed from *Code()
    void during() override;
    void exit() override;
    bool isCalibDone() { return calibDone; }
private:
    RobotM2 *robot;
    bool calibDone = false;
    double stop_timer[2] = {0, 0};
    StateMachine *machine; 
};

class M2StandbyState : public State {
public:
    M2StandbyState(RobotM2 *robot, StateMachine *machine);
    void entry() override;
    void during() override;
    void exit() override;
    bool shouldGoToReset() { return goToReset; }
private:
    RobotM2 *robot;
    StateMachine *machine; 
    bool goToReset = false;
};

class M2ResetState : public State {
public:
    M2ResetState(RobotM2 *robot, StateMachine *machine);
    void entry() override;
    void during() override;
    void exit() override;
    bool isResetFinished() { return finished; }
private:
    RobotM2 *robot;
    VM2 Xi; 
    bool finished = false;
    StateMachine *machine; 
};

#endif