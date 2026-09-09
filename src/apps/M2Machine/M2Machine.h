#ifndef M2_MACHINE_H
#define M2_MACHINE_H

#include "RobotM2.h"
#include "StateMachine.h"
#include "FLNLHelper.h"
#include "LogHelper.h" 

class M2Machine : public StateMachine {
public:
    M2Machine();
    virtual ~M2Machine();

    void init();
    void end();
    void hwStateUpdate();

    RobotM2* robot() { return static_cast<RobotM2*>(_robot.get()); }

    std::shared_ptr<FLNLHelper> UIserver = nullptr;
    std::string sessionId = "UNSET";

};

#endif