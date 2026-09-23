/**
 * \file FittsSharedControl.cpp
 * \brief Implementation of the Block 2 shared-control law (see FittsSharedControl.h).
 */
#include "FittsSharedControl.h"

#include <cmath>

namespace fsc {

namespace {

const double kPi = 3.14159265358979323846;

//! Clamp v to [-lim, lim]; lim <= 0 disables the clamp. Returns true if v was clamped.
bool clampAbs(double &v, double lim) {
    if (lim <= 0.) return false;
    if (v > lim) {
        v = lim;
        return true;
    }
    if (v < -lim) {
        v = -lim;
        return true;
    }
    return false;
}

//! Gain of the exact discretisation of a first-order low-pass filter (cut-off hz) over dt.
double lowPassGain(double hz, double dt) {
    const double tau = 1. / (2. * kPi * hz);
    return 1. - std::exp(-dt / tau);
}

}  // namespace

// ------------------------------------------------------------------------------------------------ PD
Vec2 PDController::compute(const Vec2 &x_ref, const Vec2 &v_ref, const Vec2 &x, const Vec2 &v, double dt,
                           bool *saturatedX, bool *saturatedY) {
    if (!x.allFinite() || !v.allFinite() || !x_ref.allFinite() || !v_ref.allFinite()) {
        if (saturatedX) *saturatedX = true;
        if (saturatedY) *saturatedY = true;
        return Vec2::Zero();  // never command a force computed from invalid data
    }
    // Velocity used by the D term: raw, or first-order low-pass filtered
    if (!initialised_ || g_.vel_filter_hz <= 0.) {
        v_filt_ = v;
        initialised_ = true;
    } else if (dt > 0.) {
        v_filt_ += lowPassGain(g_.vel_filter_hz, dt) * (v - v_filt_);
    }
    Vec2 F = g_.kp.cwiseProduct(x_ref - x) + g_.kd.cwiseProduct(v_ref - v_filt_);
    const bool sx = clampAbs(F(0), g_.f_max);
    const bool sy = clampAbs(F(1), g_.f_max);
    if (saturatedX) *saturatedX = sx;
    if (saturatedY) *saturatedY = sy;
    return F;
}

// ------------------------------------------------------------------------------------------ low-pass
Vec2 LowPass2::update(const Vec2 &u, double dt) {
    if (!u.allFinite()) {  // e.g. force sensors not zeroed yet: pass it on, restart on the next valid sample
        y_ = u;
        init_ = false;
        return y_;
    }
    if (!init_ || hz_ <= 0.) {
        y_ = u;
        init_ = true;
    } else if (dt > 0.) {
        y_ += lowPassGain(hz_, dt) * (u - y_);
    }
    return y_;
}

// ------------------------------------------------------------------------------------------- helpers
double channelForce(const BlendParams &p, double yCentre, double y, double dy) {
    if (!p.useYChannel) return 0.;
    double f = p.channelK * (yCentre - y) - p.channelD * dy;
    if (p.channelSaturate) clampAbs(f, p.channelFMax);
    return f;
}

double humanChannelGain(const BlendParams &p, double fIntRawX, double dqX) {
    // Activation test of RobotM2::setEndEffForceWithCompensation(F, true), x axis:
    //     |F_int| > threshold_f && |dq| > threshold_v   ->   tau_f += -gamma*blend*F_int   (= -g_p*s*F_h)
    // A non-finite reading fails the test, as it does in RobotM2.
    const bool active = std::fabs(fIntRawX) > p.platformForceThreshold && std::fabs(dqX) > p.platformVelThreshold;
    return active ? 1. - p.platformAssistGain * p.humanForceSign : 1.;
}

// ----------------------------------------------------------------------------------------------- law
BlendOutput SharedControlLaw::compute(double alpha, const Vec2 &xRef, const Vec2 &x, const Vec2 &v, const Vec2 &Fh,
                                      const Vec2 &fIntRaw, const Vec2 &dq, double dt) {
    BlendOutput o;
    if (!std::isfinite(alpha) || alpha < 0. || alpha > 1. || !xRef.allFinite() || !x.allFinite() ||
        !v.allFinite() || !std::isfinite(Fh(0))) {
        o.valid = false;  // F_cmd stays zero
        return o;
    }

    // u_r: PD of the robot-alone calibration, set-point = target centre, v_ref = 0
    bool pdSatX = false;
    o.F_pd = pd_.compute(xRef, Vec2::Zero(), x, v, dt, &pdSatX);
    o.pdSaturated = pdSatX;

    // Remove alpha of the Block 1 human channel (human force + RobotM2 force-proportional assist)
    o.k_h = humanChannelGain(p_, fIntRaw(0), dq(0));
    double cancel = -alpha * o.k_h * Fh(0);
    o.cancelSaturated = clampAbs(cancel, p_.cancelFMax);
    o.F_cancel = cancel;

    // Robot term: the PD of M2FittsMachine, plus its stiction compensation while the handle is stuck away from the
    // target centre (same law as fitts::PDController), both scaled by alpha
    double robot = alpha * o.F_pd(0);
    if (p_.robotStictionComp > 0. && std::fabs(v(0)) < p_.stictionRestSpeed &&
        std::fabs(xRef(0) - x(0)) > p_.stictionDeadband && o.F_pd(0) != 0.) {
        o.F_stiction = alpha * p_.robotStictionComp * (o.F_pd(0) > 0. ? 1. : -1.);
        robot += o.F_stiction;
    }

    // Blend on x; Block 1 channel on y
    double fx = robot + cancel;
    o.cmdSaturated = clampAbs(fx, p_.cmdFMax);
    o.F_cmd = Vec2(fx, channelForce(p_, xRef(1), x(1), v(1)));
    o.valid = true;
    return o;
}

}  // namespace fsc