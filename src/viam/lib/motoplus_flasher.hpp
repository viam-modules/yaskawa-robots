#pragma once

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

namespace robot {

/// Result of a single flasher operation.
struct FlashOutcome {
    bool ok = false;         ///< controller reported "... Success"
    bool servos_on = false;  ///< rc=0x2010: existing app still active; servo power must be OFF
    std::string detail;      ///< controller's response message(s), joined with " | "
};

/// Client for the Yaskawa MotoPlus DevTools "online download" protocol used by the Windows
/// OnlineDownload.exe: a plain-text line protocol over TCP (default port 12000). Each call is a
/// self-contained connect / send / read-response / close. Reverse-engineered + capture-verified;
/// see yaskawa-controller/tools/mp_online_download.py and
/// motoplus-sdk/HSES_MOTOPLUS_DEVTOOLS_PROTOCOL.md.
///
/// Preconditions on the controller (else the flash is rejected): servo power OFF + HOLD released,
/// the MotoPlus Temporary File initialized (one-time, maintenance mode), S4C1084=1, firmware
/// >= YAS4.12. This class does not manage those; a failed DELETE/DOWNLOAD with rc=0x2010 surfaces
/// as FlashOutcome::servos_on so the caller can tell the operator to drop servo power.
///
/// Uses a private Boost.Asio io_context internally (the calls are synchronous, blocking admin
/// operations), so it does not share or drive the controller's io_context.
class MotoPlusFlasher {
   public:
    static constexpr uint16_t k_default_port = 12000;

    explicit MotoPlusFlasher(std::string host, uint16_t port = k_default_port);

    /// Remove an installed app without rebooting (`DELETENRB <name>`). A failure with rc=0x3902
    /// ("nothing to remove") is benign on a fresh controller — the caller can proceed to download.
    FlashOutcome delete_app(const std::string& name, std::chrono::seconds timeout = std::chrono::seconds(30));

    /// Two-phase transfer: send `DOWNLOAD[NRB] <name>,<size>`, wait for `DOWNLOAD Start`, then
    /// stream the raw bytes and wait for `DOWNLOAD Success`/`Failed`. `reboot` selects the verb:
    /// `DOWNLOAD` reboots the controller after install, `DOWNLOADNRB` does not.
    FlashOutcome download_app(const std::string& name,
                              const std::vector<char>& data,
                              bool reboot = true,
                              std::chrono::seconds timeout = std::chrono::seconds(30));

   private:
    std::string host_;
    uint16_t port_;
};

}  // namespace robot
