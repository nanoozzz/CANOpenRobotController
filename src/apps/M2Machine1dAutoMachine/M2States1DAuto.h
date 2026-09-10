#ifndef M2STATES1DAUTO_H
#define M2STATES1DAUTO_H

#include "ExpConfig.h"
#include "RobotM2.h"
#include "State.h"

class StateMachine;
class M2Machine1DAuto;

/**
 * \brief Push both axes into their stops and zero the encoders.
 *
 * Carried over unchanged from the working M2Machine, including the joint
 * velocity accessor, which is known to compile in this tree.
 */
class Calib1D : public State {
   public:
    Calib1D(RobotM2 *robot, StateMachine *machine);
    void entry() override;
    void during() override;
    void exit() override;
    bool isCalibDone() const { return calibDone; }

   private:
    RobotM2 *robot;
    StateMachine *machine;
    bool calibDone = false;
    double stopTimer[2] = {0.0, 0.0};
};

/**
 * \brief Minimum-jerk return to the fixed home position. Never timed.
 *
 * Deliberately separate from Move1D so the return movement cannot contaminate
 * the measured trial.
 */
class Home1D : public State {
   public:
    Home1D(RobotM2 *robot, StateMachine *machine);
    void entry() override;
    void during() override;
    void exit() override;
    bool isReady() const { return ready; }

   private:
    RobotM2 *robot;
    StateMachine *machine;
    VM2 Xi;
    bool ready = false;
};

/**
 * \brief The timed movement: settle-to-tolerance under a critically damped PD.
 *
 * MT is an outcome, not a parameter. The state ends when the end-effector has
 * held position inside +-W/2 below V_OFFSET for DWELL seconds, or on timeout.
 */
class Move1D : public State {
   public:
    Move1D(RobotM2 *robot, StateMachine *machine);
    void entry() override;
    void during() override;
    void exit() override;
    bool isDone() const { return done; }

    // --- per-trial measurements, consumed by ITI1D ---
    double MT          = -1.0;  //!< s, from onset to start of the settled window
    double tOnset      = -1.0;  //!< s since state entry
    double peakV       = 0.0;   //!< m/s
    double tPeakV      = -1.0;  //!< s since state entry
    double finalError  = 0.0;   //!< m, signed, at settle or timeout
    double maxOffAxis  = 0.0;   //!< m, largest excursion off the task axis
    int    bandEntries = 0;     //!< times it entered +-W/2 (>1 implies corrections)
    int    satSamples  = 0;     //!< samples where the task-axis force was clipped
    int    nSamples    = 0;
    bool   timedOut    = false;
    double target      = 0.0;   //!< m, task-axis target for this trial
    double W           = 0.0;   //!< m, tolerance for this trial

   private:
    RobotM2 *robot;
    StateMachine *machine;
    bool done          = false;
    bool onsetDetected = false;
    bool wasInBand     = false;
    double settleStart = -1.0;
};

/**
 * \brief Inter-trial interval: hold at the target, write the record, advance.
 */
class ITI1D : public State {
   public:
    ITI1D(RobotM2 *robot, StateMachine *machine);
    void entry() override;
    void during() override;
    void exit() override;
    bool isDone() const { return done; }

   private:
    RobotM2 *robot;
    StateMachine *machine;
    bool done = false;
    double holdTarget = 0.0;
};

/**
 * \brief Release the robot, close the files, request a clean shutdown.
 */
class End1D : public State {
   public:
    End1D(RobotM2 *robot, StateMachine *machine);
    void entry() override;
    void during() override;
    void exit() override;

   private:
    RobotM2 *robot;
    StateMachine *machine;
};

#endif  // M2STATES1DAUTO_H