/**
 * \file M2FittsUILink.h
 * \brief Protocol layer between the M2 Fitts state machine (CORC, authority)
 *        and the Unity display client, over libFLNL.
 */

#ifndef M2FITTSUILINK_H
#define M2FITTSUILINK_H

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "FLNLHelper.h"

//! Wire-protocol version.
#define M2FITTS_PROTOCOL_VERSION 1

/**
 * \brief Thin protocol layer over an existing FLNLHelper.
 *
 * The FLNLHelper/server is owned by M2FittsHumanMachine as UIserver.
 * This class does NOT create its own FLNL server.
 */
class M2FittsUILink {
   public:
    M2FittsUILink() = default;
    ~M2FittsUILink() = default;

    //---------------------------------------------------------------- set-up

    /**
     * \brief Attach an already-created FLNLHelper.
     *
     * The FLNLHelper/server is owned by M2FittsHumanMachine.
     */
    bool init(std::shared_ptr<FLNLHelper> helper,
              int divider,
              bool required);

    //! Register a value to be streamed.
    //! Registration order defines the wire order.
    template <typename T>
    int registerState(const T &v) {
        return (helper_ == nullptr) ? 0 : helper_->registerState(v);
    }

    //! Session descriptor replayed on every connection.
    void setSession(const std::vector<double> &params) {
        session_ = params;
    }

    //! Reference to the machine clock.
    void setClock(const double *t) {
        clock_ = t;
    }

    //---------------------------------------------------------------- per-cycle update

    /**
     * \brief Call once per control cycle.
     *
     * Decodes at most one inbound command and sends the decimated
     * state frame.
     */
    void update();

    //---------------------------------------------------------------- outbound

    //! Fire-and-forget event.
    void send(const std::string &cmd,
              const std::vector<double> &params = {});

    //! Event that also defines what should be on screen.
    void sendContext(const std::string &cmd,
                     const std::vector<double> &params = {});

    //---------------------------------------------------------------- inbound

    bool connected() const {
        return helper_ != nullptr && helper_->isConnected();
    }

    bool required() const {
        return required_;
    }

    //! True after compatible HELO.
    bool handshakeDone() const {
        return handshake_;
    }

    //! Consume pending GO request.
    bool consumeGo() {
        bool g = go_;
        go_ = false;
        return g;
    }

    //! Consume pending ABORT request.
    bool consumeAbort() {
        bool a = abort_;
        abort_ = false;
        return a;
    }

    //! Last measured round trip, in ms.
    double roundTripMs() const {
        return roundTripMs_;
    }

    //! Latest UI marker.
    const double &markCode() const {
        return markCode_;
    }

    const double &markTime() const {
        return markTime_;
    }

    //! Machine time at which marker was received.
    const double &markServerTime() const {
        return markServerTime_;
    }

    void close();

   private:
    void processIncoming();
    void onConnected();

    // IMPORTANT:
    // This is NOT creating the FLNLHelper anymore.
    // It points to M2FittsHumanMachine::UIserver.
    std::shared_ptr<FLNLHelper> helper_ = nullptr;

    std::vector<double> session_;
    const double *clock_ = nullptr;

    bool required_ = true;
    bool handshake_ = false;
    bool wasConnected_ = false;

    int divider_ = 1;
    int streamCount_ = 0;

    double lastReconnectAttempt_ = -1e9;

    bool go_ = false;
    bool abort_ = false;

    std::string contextCmd_;
    std::vector<double> contextParams_;

    double roundTripMs_ =
        std::numeric_limits<double>::quiet_NaN();

    double markCode_ = 0.;
    double markTime_ = 0.;
    double markServerTime_ = 0.;
};

#endif  // M2FITTSUILINK_H