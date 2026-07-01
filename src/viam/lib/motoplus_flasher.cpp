#include "motoplus_flasher.hpp"

#include <sys/socket.h>
#include <sys/time.h>

#include <array>
#include <cstddef>
#include <ctime>
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include <boost/asio.hpp>

#include "logger.hpp"

namespace asio = boost::asio;
using boost::asio::ip::tcp;

namespace robot {
namespace {

std::string join(const std::vector<std::string>& msgs) {
    std::string out;
    for (const auto& m : msgs) {
        if (!out.empty()) {
            out += " | ";
        }
        out += m;
    }
    return out;
}

bool contains(const std::string& s, const char* sub) {
    return s.find(sub) != std::string::npos;
}

bool is_terminal(const std::string& m) {
    return contains(m, "Success") || contains(m, "Failed");
}

// Connect with a timeout via a transient io_context; on success, set blocking recv/send timeouts
// on the socket so the subsequent blocking reads/writes can't hang forever (Linux target).
bool connect_with_timeout(
    asio::io_context& io, tcp::socket& sock, const std::string& host, uint16_t port, std::chrono::seconds timeout, std::string& err) {
    boost::system::error_code ec;
    tcp::resolver resolver(io);
    const auto endpoints = resolver.resolve(host, std::to_string(port), ec);
    if (ec) {
        err = "resolve failed: " + ec.message();
        return false;
    }

    bool connected = false;
    boost::system::error_code connect_ec = asio::error::would_block;
    asio::async_connect(sock, endpoints, [&](const boost::system::error_code& e, const tcp::endpoint&) {
        connect_ec = e;
        connected = !e;
    });
    io.restart();
    io.run_for(timeout);
    if (!connected) {
        sock.close(ec);
        err = "connect failed: " + connect_ec.message();
        return false;
    }

    timeval tv{};
    tv.tv_sec = static_cast<time_t>(timeout.count());
    (void)::setsockopt(sock.native_handle(), SOL_SOCKET, SO_RCVTIMEO, &tv, static_cast<socklen_t>(sizeof(tv)));
    (void)::setsockopt(sock.native_handle(), SOL_SOCKET, SO_SNDTIMEO, &tv, static_cast<socklen_t>(sizeof(tv)));
    return true;
}

// Read NUL-terminated ASCII messages until one satisfies `done`, the peer closes, or a timeout.
// Appends each decoded message (without its NUL) to `msgs`.
void read_messages(tcp::socket& sock, const std::function<bool(const std::string&)>& done, std::vector<std::string>& msgs) {
    std::string buf;
    std::array<char, 1024> tmp{};
    for (;;) {
        boost::system::error_code ec;
        const std::size_t n = sock.read_some(asio::buffer(tmp), ec);
        if (n > 0) {
            buf.append(tmp.data(), n);
            bool hit = false;
            std::size_t nul;
            while ((nul = buf.find('\0')) != std::string::npos) {
                msgs.emplace_back(buf.substr(0, nul));
                buf.erase(0, nul + 1);
                if (done(msgs.back())) {
                    hit = true;
                }
            }
            if (hit) {
                return;
            }
        }
        if (ec) {
            if (!buf.empty()) {
                msgs.push_back(buf);
            }
            return;
        }
    }
}

FlashOutcome classify(std::vector<std::string> msgs) {
    FlashOutcome out;
    out.detail = join(msgs);
    for (const auto& m : msgs) {
        if (contains(m, "Success")) {
            out.ok = true;
        }
        if (contains(m, "0x2010")) {
            out.servos_on = true;
        }
    }
    return out;
}

}  // namespace

MotoPlusFlasher::MotoPlusFlasher(std::string host, uint16_t port) : host_(std::move(host)), port_(port) {}

FlashOutcome MotoPlusFlasher::delete_app(const std::string& name, std::chrono::seconds timeout) {
    asio::io_context io;
    tcp::socket sock(io);
    std::string err;
    if (!connect_with_timeout(io, sock, host_, port_, timeout, err)) {
        return FlashOutcome{false, false, std::move(err)};
    }

    boost::system::error_code ec;
    const std::string header = "DELETENRB " + name;
    asio::write(sock, asio::buffer(header), ec);
    if (ec) {
        return FlashOutcome{false, false, "send failed: " + ec.message()};
    }

    std::vector<std::string> msgs;
    read_messages(sock, is_terminal, msgs);
    LOGGING(debug) << "DELETENRB " << name << " -> " << join(msgs);
    return classify(std::move(msgs));
}

FlashOutcome MotoPlusFlasher::download_app(const std::string& name,
                                           const std::vector<char>& data,
                                           bool reboot,
                                           std::chrono::seconds timeout) {
    asio::io_context io;
    tcp::socket sock(io);
    std::string err;
    if (!connect_with_timeout(io, sock, host_, port_, timeout, err)) {
        return FlashOutcome{false, false, std::move(err)};
    }

    boost::system::error_code ec;
    const std::string verb = reboot ? "DOWNLOAD" : "DOWNLOADNRB";
    const std::string header = verb + " " + name + "," + std::to_string(data.size());
    asio::write(sock, asio::buffer(header), ec);
    if (ec) {
        return FlashOutcome{false, false, "send header failed: " + ec.message()};
    }

    // Phase 1: wait for "DOWNLOAD Start" before streaming. Gluing the bytes onto the header makes
    // the controller do a fresh read of <size> bytes it never receives, and it times out.
    std::vector<std::string> msgs;
    read_messages(sock, [](const std::string& m) { return contains(m, "Start") || contains(m, "Failed"); }, msgs);
    bool started = false;
    for (const auto& m : msgs) {
        if (contains(m, "Start")) {
            started = true;
        }
    }
    if (!started) {
        FlashOutcome out = classify(std::move(msgs));
        if (out.detail.empty()) {
            out.detail = "no 'DOWNLOAD Start' from controller";
        }
        return out;
    }

    // Phase 2: stream the raw .out bytes, then wait for Success/Failed.
    asio::write(sock, asio::buffer(data), ec);
    if (ec) {
        return FlashOutcome{false, false, "send payload failed: " + ec.message()};
    }
    read_messages(sock, is_terminal, msgs);
    LOGGING(debug) << header << " -> " << join(msgs);
    return classify(std::move(msgs));
}

}  // namespace robot
