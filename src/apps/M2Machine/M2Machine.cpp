#include "M2Machine.h"
#include "M2States.h" // added by Nguyen

static bool endCalib(StateMachine& sm) {
    return (sm.state<M2CalibState>("CalibState"))->isCalibDone();
}

static bool toResetTriggered(StateMachine& SM) {
    auto& sm = static_cast<M2Machine&>(SM);
    return sm.state<M2StandbyState>("StandbyState")->shouldGoToReset();
}

static bool resetFinished(StateMachine& sm) {
    return sm.state<M2ResetState>("ResetState")->isResetFinished();
}

M2Machine::M2Machine() {
    setRobot(std::make_unique<RobotM2>("M2_MELB"));

    addState("CalibState",   std::make_shared<M2CalibState>(robot(), this));
    addState("StandbyState", std::make_shared<M2StandbyState>(robot(), this));
    addState("ResetState",   std::make_shared<M2ResetState>(robot(), this));

    addTransition("CalibState", &endCalib, "StandbyState");

    addTransition("StandbyState", &toResetTriggered, "ResetState");

    addTransition("ResetState", &resetFinished, "StandbyState");

    setInitState("CalibState");
}

M2Machine::~M2Machine() {}

void M2Machine::init() {
    spdlog::debug("M2Machine::init()");
    if (robot()->initialise()) {
        // Setup central logging
        logHelper.initLogger("M2MachineLog", "logs/M2Machine.csv", LogFormat::CSV, true);
        logHelper.add(runningTime(), "Time (s)");
        logHelper.add(robot()->getEndEffPosition(), "Position");
        logHelper.add(robot()->getEndEffVelocity(), "Velocity");
        logHelper.add(robot()->getEndEffForce(), "Force");
        logHelper.startLogger();

        UIserver = std::make_shared<FLNLHelper>(*robot(),"0.0.0.0");
        
    } else {
        spdlog::critical("Failed robot initialisation. Exiting...");
        std::raise(SIGTERM);
    }
}

/*void M2Machine::hwStateUpdate() {
    auto now = std::chrono::steady_clock::now();
    static auto lastCheck = std::chrono::steady_clock::now();
    static bool connected = false;

    if (UIserver && std::chrono::duration<double, std::milli>(now - lastCheck).count() > 1000.0) {
        connected = UIserver->isConnected();
        if (!connected) {
            spdlog::warn("UI disconnected. Waiting for Unity...");
            UIserver->reconnect(); 
            connected = UIserver->isConnected();
        }
        lastCheck = now;
    }

    StateMachine::hwStateUpdate();

    static auto lastSend = std::chrono::steady_clock::now();
    if (connected && std::chrono::duration<double, std::milli>(now - lastSend).count() >= 25.0) {
        UIserver->sendState();
        lastSend = now;
    }
}*/

void M2Machine::hwStateUpdate() {
    StateMachine::hwStateUpdate();

    auto now = std::chrono::steady_clock::now();
    static auto lastSend = now;
    if (UIserver && std::chrono::duration<double, std::milli>(now - lastSend).count() >= 25.0) {
        UIserver->sendState();
        lastSend = now;
    }
}

void M2Machine::end() {
    if (UIserver) UIserver->closeConnection();
    StateMachine::end();
}