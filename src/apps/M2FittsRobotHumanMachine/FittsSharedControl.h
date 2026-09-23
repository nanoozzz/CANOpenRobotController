/**
 * \file FittsSharedControl.h
 * \brief Hardware-independent shared-control law of M2FittsRobotHumanMachine (Block 2).
 *
 * Block 2 of the robot-guidance Fitts' protocol runs every reach under
 *
 *          u = alpha * u_r + (1 - alpha) * u_h                      (task axis x of the M2)
 *
 *  u_r  robot input: the PD force of M2FittsMachine (robot-alone calibration), with the same law,
 *       gains and discretisation; the set-point steps to the target centre at target onset.
 *  u_h  human input: the human force channel as it acted on the handle in Block 1
 *       (M2FittsHumanMachine), i.e. the human force F_h plus the force-proportional assist that
 *       RobotM2::setEndEffForceWithCompensation(F, true) adds while the handle moves.
 *
 * Both inputs are forces on the same handle. The human force is physical and acts whatever the robot
 * commands, so its share can only be brought down to (1 - alpha) by the robot cancelling alpha of it,
 * as measured by the handle force sensors (F_int = s * F_h):
 *
 *     F_cmd,x = alpha * F_pd,x - alpha * k_h * F_h,x
 *     net_x   = k_h * F_h,x + F_cmd,x = alpha * F_pd,x + (1 - alpha) * k_h * F_h,x
 *
 *     k_h = 1 - g_p * s   while the RobotM2 assist is active (|F_int,x| > thr_f and |dq_x| > thr_v)
 *     k_h = 1             otherwise
 *
 * (RobotM2 adds -g_p * F_int = -g_p * s * F_h, with g_p = gamma * blend = 0.4 * 0.5 at commit 017f81d.)
 * The velocity-only part of RobotM2's friction feed-forward is plant compensation, left to RobotM2.
 * The y axis keeps the Block 1 virtual channel: a task constraint, independent of alpha.
 *
 * alpha = 0 gives F_cmd,x = 0 exactly (Block 1); alpha = 1 gives net_x = F_pd,x (robot alone). The
 * cancellation and the total command are saturated for safety, and every saturated sample is flagged
 * so that trials on which the blend could not be realised can be identified.
 *
 * Only Eigen and the standard library are used, so the law can be exercised offline.
 *
 * \version 1.0
 * \date 2026-09-18
 */
#ifndef FITTS_SHARED_CONTROL_H
#define FITTS_SHARED_CONTROL_H

#include <Eigen/Dense>

namespace fsc {

typedef Eigen::Vector2d Vec2;

/** PD gains in end-effector space [x, y]: same fields and meaning as fitts::PDGains (M2FittsMachine). */
struct PDGains {
    Vec2 kp = Vec2(400., 400.);  //!< Proportional gain [N/m]
    Vec2 kd = Vec2(80., 80.);    //!< Derivative gain [N.s/m]
    double f_max = 30.;          //!< Per-axis force saturation [N]
    double vel_filter_hz = 0.;   //!< First-order low-pass cut-off on the velocity used by the D term [Hz]; <= 0: off
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * \brief PD law of M2FittsMachine (fitts::PDController), per axis:
 *            F = Kp (x_ref - x) + Kd (v_ref - v_f),   then |F_i| <= f_max
 *        The derivative acts on the (optionally filtered) measured velocity, so stepping the set-point
 *        produces no derivative kick.
 */
class PDController {
   public:
    void setGains(const PDGains &g) { g_ = g; }
    const PDGains &gains() const { return g_; }
    void reset() { initialised_ = false; }
    Vec2 compute(const Vec2 &x_ref, const Vec2 &v_ref, const Vec2 &x, const Vec2 &v, double dt,
                 bool *saturatedX = nullptr, bool *saturatedY = nullptr);
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

   private:
    PDGains g_;
    Vec2 v_filt_ = Vec2::Zero();
    bool initialised_ = false;
};

/** \brief First-order low-pass filter on a 2D signal (exact discretisation). Cut-off <= 0: pass-through. */
class LowPass2 {
   public:
    void setCutoff(double hz) { hz_ = hz; }
    double cutoff() const { return hz_; }
    void reset() { init_ = false; }
    //! A non-finite input (e.g. force sensors not zeroed yet) is passed through and restarts the filter.
    Vec2 update(const Vec2 &u, double dt);
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

   private:
    double hz_ = 0.;
    Vec2 y_ = Vec2::Zero();
    bool init_ = false;
};

/** \brief Parameters of the blend. Defaults mirror RobotM2 (commit 017f81d) and Block 1. */
struct BlendParams {
    // Human force channel
    double humanForceSign = -1.;           //!< s in F_h = s * F_int (F_int = RobotM2::getInteractionForce())
    double platformAssistGain = 0.2;       //!< g_p = gamma * blend in RobotM2::setEndEffForceWithCompensation
    double platformForceThreshold = 0.05;  //!< threshold_f of that function [N]
    double platformVelThreshold = 0.05;    //!< threshold_v of that function [m/s]
    // Safety saturations (<= 0 disables)
    double cancelFMax = 40.;  //!< Cap on |alpha * k_h * F_h,x| [N]; keep >= reach_force_limit (else a participant force
                              //!< between the two leaks through at alpha = 1 without ending the trial)
    double cmdFMax = 75.;     //!< Cap on |F_cmd,x| [N]; keep >= f_max + cancelFMax + stiction compensation
    // Robot stiction compensation (part of u_r; from M2FittsMachine.yaml pd.stiction_*; <= 0 disables)
    double robotStictionComp = 0.;     //!< Breakaway force added to the robot term (x alpha) while stuck [N]
    double stictionRestSpeed = 0.01;   //!< Below this speed the handle counts as stuck [m/s]
    double stictionDeadband = 0.0005;  //!< No compensation within this distance of the target centre [m]
    // Block 1 virtual channel on y (task constraint, alpha-independent)
    bool useYChannel = true;
    double channelK = 800.;         //!< [N/m]
    double channelD = 10.;          //!< [N/(m/s)]
    double channelFMax = 30.;       //!< [N], applied only if channelSaturate
    bool channelSaturate = false;   //!< Block 1 never applied channelFMax: keep false for identical blocks
};

/** \brief One evaluation of the law. */
struct BlendOutput {
    Vec2 F_cmd = Vec2::Zero();  //!< Force to send with RobotM2::setEndEffForceWithCompensation(F_cmd, true) [N]
    Vec2 F_pd = Vec2::Zero();   //!< u_r: PD force (x is used; y is computed but not applied) [N]
    double F_cancel = 0.;       //!< -alpha * k_h * F_h,x after saturation [N]
    double F_stiction = 0.;     //!< Robot stiction compensation included in F_cmd,x [N]
    double k_h = 1.;            //!< Gain of the Block 1 human channel on this sample
    bool pdSaturated = false;   //!< PD saturated on x (part of u_r's definition, as in the robot-alone run)
    bool cancelSaturated = false;
    bool cmdSaturated = false;
    bool valid = false;         //!< false: non-finite input or alpha outside [0, 1]; F_cmd is then zero
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

//! Block 1 virtual channel force on y.
double channelForce(const BlendParams &p, double yCentre, double y, double dy);

//! k_h from the raw interaction force and the joint velocity on x (mirrors RobotM2's activation test).
double humanChannelGain(const BlendParams &p, double fIntRawX, double dqX);

class SharedControlLaw {
   public:
    void setGains(const PDGains &g) { pd_.setGains(g); }
    void setParams(const BlendParams &p) { p_ = p; }
    const PDGains &gains() const { return pd_.gains(); }
    const BlendParams &params() const { return p_; }
    void reset() { pd_.reset(); }  //!< Call at target onset, as M2FittsMachine resets its PD

    /**
     * \param alpha    autonomy level of the trial, in [0, 1]
     * \param xRef     (target centre x, channel centre y) [m]
     * \param x, v     end-effector position [m] and velocity [m/s]
     * \param Fh       estimated human force on the handle, robot frame (s * filtered F_int) [N]
     * \param fIntRaw  raw interaction force, RobotM2::getInteractionForce() [N]
     * \param dq       joint velocity, RobotM2::getVelocity() [m/s]
     * \param dt       time since the previous evaluation [s] (0 on the first one)
     */
    BlendOutput compute(double alpha, const Vec2 &xRef, const Vec2 &x, const Vec2 &v, const Vec2 &Fh,
                        const Vec2 &fIntRaw, const Vec2 &dq, double dt);
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

   private:
    PDController pd_;
    BlendParams p_;
};

}  // namespace fsc

#endif  // FITTS_SHARED_CONTROL_H