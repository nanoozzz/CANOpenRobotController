#include "M2FittsUILink.h"

#include <cmath>
#include <iostream>

#include "spdlog/spdlog.h"

bool M2FittsUILink::init(
    std::shared_ptr<FLNLHelper> helper,
    int divider,
    bool required)
{
    divider_ = (divider < 1) ? 1 : divider;
    required_ = required;

    streamCount_ = 0;
    handshake_ = false;
    wasConnected_ = false;

    go_ = false;
    abort_ = false;

    lastReconnectAttempt_ = -1e9;

    helper_ = helper;

    if (helper_ == nullptr) {
        spdlog::error(
            "M2FittsRobotHuman/UI: received null FLNLHelper.");
        return false;
    }

    spdlog::info(
        "M2FittsRobotHuman/UI: using existing FLNL server/helper. "
        "State stream decimated by {} ({}).",
        divider_,
        required_ ? "client required" : "client optional");

    return true;
}


// ================================================================
// UPDATE
// ================================================================

void M2FittsUILink::update()
{
    if (helper_ == nullptr)
        return;

    const bool isConnected =
        helper_->isConnected();

    // ------------------------------------------------------------
    // Connection transition
    // ------------------------------------------------------------

    if (isConnected && !wasConnected_) {

        spdlog::info(
            "M2FittsRobotHuman/UI: client connected.");

        handshake_ = false;

        onConnected();
    }
    else if (!isConnected && wasConnected_) {

        spdlog::warn(
            "M2FittsRobotHuman/UI: client disconnected.");

        handshake_ = false;
    }

    wasConnected_ = isConnected;


    // ------------------------------------------------------------
    // No client
    // ------------------------------------------------------------

    if (!isConnected) {

        // IMPORTANT:
        // The existing UIserver owns the socket.
        //
        // We do NOT call helper_->reconnect() here because
        // M2FittsRobotHumanMachine/UIserver is the owner of the
        // FLNL server.
        //
        // The original UIserver handles the server lifecycle.

        return;
    }


    // ------------------------------------------------------------
    // Process one incoming command
    // ------------------------------------------------------------

    processIncoming();


    // ------------------------------------------------------------
    // Stream state
    // ------------------------------------------------------------

    ++streamCount_;

    if (streamCount_ >= divider_) {

        streamCount_ = 0;

        helper_->sendState();
    }
}


// ================================================================
// CONNECTION
// ================================================================

void M2FittsUILink::onConnected()
{
    spdlog::info(
        "M2FittsRobotHuman/UI: client connection detected.");

    // Replay session information.
    if (!session_.empty()) {

        helper_->sendCmd(
            "SESS",
            session_);
    }

    // Replay current screen/context.
    if (!contextCmd_.empty()) {

        helper_->sendCmd(
            contextCmd_,
            contextParams_);
    }
}


// ================================================================
// INCOMING COMMANDS
// ================================================================

void M2FittsUILink::processIncoming()
{
    if (helper_ == nullptr)
        return;

    if (!helper_->isCmd())
        return;


    std::string cmd;
    std::vector<double> p;

    helper_->getCmd(cmd, p);
    helper_->clearCmd();


    const double now =
        (clock_ != nullptr) ? *clock_ : 0.0;


    spdlog::debug(
        "M2FittsRobotHuman/UI: received command '{}' with {} parameters.",
        cmd,
        p.size());


    // ============================================================
    // HELO
    // ============================================================

    if (cmd == "HELO") {

        const double clientVersion =
            p.empty() ? 0.0 : p[0];

        if (std::lround(clientVersion)
            != M2FITTS_PROTOCOL_VERSION) {

            spdlog::critical(
                "M2FittsRobotHuman/UI: protocol mismatch. "
                "Client={}, Server={}.",
                clientVersion,
                M2FITTS_PROTOCOL_VERSION);

            helper_->sendCmd("VERR");

            handshake_ = false;

            return;
        }


        handshake_ = true;


        // Send session again after handshake.
        if (!session_.empty()) {

            helper_->sendCmd(
                "SESS",
                session_);
        }


        // Send current UI context again after handshake.
        if (!contextCmd_.empty()) {

            helper_->sendCmd(
                contextCmd_,
                contextParams_);
        }


        spdlog::info(
            "M2FittsRobotHuman/UI: handshake completed "
            "(protocol v{}).",
            M2FITTS_PROTOCOL_VERSION);

        return;
    }


    // ============================================================
    // PING
    // ============================================================

    if (cmd == "PING") {

        // Unity sends:
        //
        // PING [client timestamp]
        //
        // We return:
        //
        // PONG [client timestamp, server timestamp]

        std::vector<double> pongParams;

        if (!p.empty()) {
            pongParams.push_back(p[0]);
        }

        pongParams.push_back(now);

        helper_->sendCmd(
            "PONG",
            pongParams);

        return;
    }


    // ============================================================
    // GO
    // ============================================================

    if (cmd == "GTNS") {

        go_ = true;

        spdlog::debug(
            "M2FittsRobotHuman/UI: GO received.");

        return;
    }


    // ============================================================
    // ABORT
    // ============================================================

    if (cmd == "ABRT") {

        abort_ = true;

        spdlog::debug(
            "M2FittsRobotHuman/UI: ABRT received.");

        return;
    }


    // ============================================================
    // MARK
    // ============================================================

    if (cmd == "MARK") {

        if (!p.empty())
            markCode_ = p[0];

        if (p.size() > 1)
            markTime_ = p[1];

        markServerTime_ = now;


        spdlog::debug(
            "M2FittsRobotHuman/UI: MARK received. "
            "code={}, clientTime={}, serverTime={}.",
            markCode_,
            markTime_,
            markServerTime_);

        return;
    }

    // RTTR
        if (cmd == "RTTR") {  //client reports the round trip it measured (display only)
            roundTripMs_ = p.empty() ? std::numeric_limits<double>::quiet_NaN() : p[0];
            return;
    }


    // ============================================================
    // Unknown command
    // ============================================================

    spdlog::warn(
        "M2FittsRobotHuman/UI: unknown command '{}'.",
        cmd);
}


// ================================================================
// SEND
// ================================================================

void M2FittsUILink::send(
    const std::string &cmd,
    const std::vector<double> &params)
{
    if (helper_ == nullptr)
        return;

    if (!helper_->isConnected())
        return;

    helper_->sendCmd(
        cmd,
        params);
}


// ================================================================
// SEND CONTEXT
// ================================================================

void M2FittsUILink::sendContext(
    const std::string &cmd,
    const std::vector<double> &params)
{
    // Remember the latest context so it can be replayed
    // if Unity reconnects.

    contextCmd_ = cmd;
    contextParams_ = params;


    // Send immediately if connected.

    if (helper_ == nullptr)
        return;

    if (!helper_->isConnected())
        return;

    helper_->sendCmd(
        cmd,
        params);
}


// ================================================================
// CLOSE
// ================================================================

void M2FittsUILink::close()
{
    handshake_ = false;
    wasConnected_ = false;

    helper_.reset();

    session_.clear();
    contextCmd_.clear();
    contextParams_.clear();

    go_ = false;
    abort_ = false;

    spdlog::info(
        "M2FittsRobotHuman/UI: link closed.");
}