/**
 * \file M2FittsMachine.h
 * \brief CORC application running a 1D Fitts'-type reaching block on the ArmMotus M2 with a PD controller.
 *
 * Trials are read from a CSV file (D and W per row, cm by default). For each trial the PD set-point steps
 * from home (default x = 0.40 m, y = 0.20 m) to x_home - D; the trial succeeds when the end-effector stays
 * within tolerance_factor * W (default W/2) of the target for dwell_time (default 1 s); the robot then
 * returns home. Movement-time variables are written per trial; kinematics are logged at every control loop.
 *
 * Provenance: drafted with AI assistance (Claude, Anthropic), September 2026. Review before use.
 * Licence: Apache-2.0 (same as CORC).
 */
#ifndef M2FITTS_MACHINE_H
#define M2FITTS_MACHINE_H

#include <memory>
#include <string>

#include "M2FittsStates.h"
#include "RobotM2.h"
#include "StateMachine.h"

class M2FittsMachine : public StateMachine {
   public:
    M2FittsMachine();
    ~M2FittsMachine();

    void init() override;
    void end() override;
    void hwStateUpdate() override;

    RobotM2 *robot() { return static_cast<RobotM2 *>(_robot.get()); }  //!< Robot getter with specialised type
    std::shared_ptr<FittsSession> session() { return session_; }

   private:
    void loadSetup();                                       //!< Read YAML configuration and trial CSV (no motion)
    void writeParametersFile(const std::string &path) const;  //!< Provenance record next to the data files

    std::shared_ptr<FittsSession> session_;
    std::string config_path_;
};

#endif  // M2FITTS_MACHINE_H
