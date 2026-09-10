#ifndef EXPCONFIG_H
#define EXPCONFIG_H

#include <cmath>

/**
 * \brief All experiment parameters for the 1D automatic Fitts-characterisation app.
 *
 * Everything you are expected to change lives in this file. Constants marked
 * [MEASURE] must be established on your own hardware before the first real run:
 * the defaults are placeholders, not recommendations.
 */
namespace exp1d {

// ---------------------------------------------------------------- geometry --
constexpr int    TASK_AXIS = 0;              //!< 0 = x, 1 = y. The axis movements occur along.
constexpr int    OFF_AXIS  = 1 - TASK_AXIS;  //!< The axis held by a virtual constraint.
constexpr double DIRECTION = +1.0;           //!< target = HOME[TASK_AXIS] + DIRECTION*A

//! Home position, metres. [MEASURE] Place it so HOME + DIRECTION*A_max stays
//! inside the workspace with margin, i.e. near one end of travel, NOT the centre.
constexpr double HOME[2] = {0.32, 0.20};

//! Hard workspace limits, metres. [MEASURE] Read these off RobotM2.h or by
//! driving the robot by hand and logging the extremes. Startup aborts if any
//! target falls outside.
constexpr double AXIS_MIN[2] = {0.10, 0.05};
constexpr double AXIS_MAX[2] = {0.55, 0.40};

// -------------------------------------------------------------------- trials --
constexpr const char* TRIAL_FILE  = "trials.csv";      //!< columns: index,A,W,ID
constexpr const char* RECORD_FILE = "logs/trials_out.csv";

//! Multiply CSV A and W by this to get metres. 0.001 if your CSV is in mm, 1.0 if metres.
constexpr double UNIT_SCALE = 0.001;

// ---------------------------------------------------------------- controller --
//! Effective end-effector mass, kg. [MEASURE] See README for the identification
//! procedure. Damping is derived from it, so a wrong value here means a wrong
//! damping ratio and a contaminated settling time.
constexpr double M_EFF = 2.0;

//! Task-axis stiffness, N/m. Sets the closed-loop rate w_n = sqrt(K/M_EFF) and
//! therefore the predicted slope of MT vs ID (~ln2/w_n seconds per bit).
//! Pick K so your MT range is comfortably measurable, e.g. 0.2-2 s.
constexpr double K_TASK = 200.0;

//! Critically damped by construction: zeta = 1 gives monotone approach to the
//! tolerance band and the cleanest logarithmic settling law.
inline const double B_TASK = 2.0 * std::sqrt(K_TASK * M_EFF);

//! Off-axis virtual constraint. Start LOW and raise while watching for buzzing:
//! a stiff wall at 500 Hz can go unstable.
constexpr double K_OFF = 800.0;
inline const double B_OFF = 2.0 * std::sqrt(K_OFF * M_EFF);

constexpr double F_MAX      = 40.0;  //!< Task-axis force cap, N. Safety AND a data quality issue: see README.
constexpr double F_MAX_OFF  = 60.0;  //!< Off-axis force cap, N.

// -------------------------------------------------------- MT detection rules --
constexpr double V_ONSET  = 0.02;   //!< m/s. Speed at which the movement is declared started.
constexpr double V_OFFSET = 0.02;   //!< m/s. Speed below which it may be declared settled.
constexpr double DWELL    = 0.05;   //!< s. Time it must stay inside +-W/2 below V_OFFSET.
constexpr double TIMEOUT  = 5.0;    //!< s. Abort the trial and flag it.

// ------------------------------------------------------------------- timing --
constexpr double HOME_DURATION = 1.5;    //!< s, min-jerk return to home.
constexpr double HOME_TOL      = 0.003;  //!< m, position tolerance to declare home reached.
constexpr double HOME_V_TOL    = 0.02;   //!< m/s.
constexpr double HOME_TIMEOUT  = 4.0;    //!< s.
constexpr double ITI           = 0.5;    //!< s, hold at target after each trial.

// ------------------------------------------------------------ phase codes ----
// Written to the continuous log so you can slice trajectories by phase.
constexpr double PHASE_CALIB = 0.0;
constexpr double PHASE_HOME  = 1.0;
constexpr double PHASE_MOVE  = 2.0;
constexpr double PHASE_ITI   = 3.0;
constexpr double PHASE_END   = 4.0;

}  // namespace exp1d

#endif  // EXPCONFIG_H