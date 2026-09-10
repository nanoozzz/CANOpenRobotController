#ifndef M2MACHINE1DAUTO_H
#define M2MACHINE1DAUTO_H

#include <Eigen/Dense>
#include <fstream>

#include "ExpConfig.h"
#include "LogHelper.h"
#include "M2States1DAuto.h"
#include "RobotM2.h"
#include "StateMachine.h"
#include "TrialTable.h"

/**
 * \brief Autonomous 1D reaching app for characterising the robot's own
 *        movement-time / index-of-difficulty relationship.
 *
 * The robot returns to a fixed home before every trial, then moves to a target
 * derived from the trial amplitude and settles inside +-W/2. Movement time is
 * measured, not commanded.
 *
 * Two outputs are produced:
 *   - a continuous trajectory log at loop rate (LogHelper)
 *   - one summary row per trial (own ofstream) -- this is the experimental data
 */
class M2Machine1DAuto : public StateMachine {
   public:
    M2Machine1DAuto();
    virtual ~M2Machine1DAuto();

    void init() override;
    void end() override;
    void hwStateUpdate() override;

    RobotM2 *robot() { return static_cast<RobotM2 *>(_robot.get()); }

    // --- trial sequencing ---
    const Trial &currentTrial() const { return trials_.at(trialIdx_); }
    bool isExhausted() const { return exhausted_; }
    void advanceTrial();

    // --- records ---
    void writeTrialRecord(const Move1D &mv);
    void closeRecords();

    /**
     * Mirrors bound into LogHelper. These are Eigen::VectorXd (dynamic size)
     * rather than VM2 on purpose: LogHelper::registerState stores a raw address
     * taken from a const Eigen::VectorXd& parameter, so passing a fixed-size VM2
     * would bind to a temporary and leave a dangling pointer.
     */
    Eigen::VectorXd logPos, logVel, logForce;
    double logPhase = 0.0, logTrialIdx = 0.0, logTargetPos = 0.0, logW = 0.0;

   private:
    TrialTable trials_;
    size_t trialIdx_ = 0;
    bool exhausted_ = false;
    std::ofstream records_;
    int nTimedOut_ = 0;
};

#endif  // M2MACHINE1DAUTO_H