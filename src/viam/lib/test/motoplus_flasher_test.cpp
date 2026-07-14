#define BOOST_TEST_MODULE motoplus_flasher_test
#include <array>
#include <cstdint>
#include <functional>
#include <string>
#include <thread>
#include <vector>

#include <boost/asio.hpp>
#include <boost/test/unit_test.hpp>

#include <viam/lib/motoplus_flasher.hpp>

namespace asio = boost::asio;
using asio::ip::tcp;

namespace {

// A one-shot loopback TCP server that speaks the MotoPlus 12000 line protocol. Binds an ephemeral
// port, accepts a single connection, and runs `handler(socket)` on it in a background thread.
class FakeController {
   public:
    explicit FakeController(std::function<void(tcp::socket&)> handler)
        : acceptor_(io_, tcp::endpoint(tcp::v4(), 0)), port_(acceptor_.local_endpoint().port()) {
        thread_ = std::thread([this, handler = std::move(handler)] {
            boost::system::error_code ec;
            tcp::socket sock(io_);
            acceptor_.accept(sock, ec);
            if (!ec) {
                handler(sock);
            }
        });
    }
    ~FakeController() {
        join();
    }
    void join() {
        if (thread_.joinable()) {
            thread_.join();
        }
    }
    uint16_t port() const {
        return port_;
    }

   private:
    asio::io_context io_;
    tcp::acceptor acceptor_;
    uint16_t port_;
    std::thread thread_;
};

// Send an ASCII message with the protocol's trailing NUL.
void send_msg(tcp::socket& s, const std::string& msg) {
    std::string framed = msg;
    framed.push_back('\0');
    boost::system::error_code ec;
    asio::write(s, asio::buffer(framed), ec);
}

// Read exactly `n` bytes (blocking).
std::string read_n(tcp::socket& s, std::size_t n) {
    std::string out;
    std::array<char, 4096> tmp{};
    while (out.size() < n) {
        boost::system::error_code ec;
        const std::size_t got = s.read_some(asio::buffer(tmp), ec);
        out.append(tmp.data(), got);
        if (ec) {
            break;
        }
    }
    return out;
}

std::vector<char> make_payload(std::size_t n) {
    std::vector<char> v(n);
    for (std::size_t i = 0; i < n; ++i) {
        v[i] = static_cast<char>(i & 0xFF);
    }
    return v;
}

}  // namespace

BOOST_AUTO_TEST_CASE(delete_success) {
    FakeController srv([](tcp::socket& s) {
        (void)read_n(s, std::string("DELETENRB viammoto.out").size());
        send_msg(s, "DELETE Success");
    });
    robot::MotoPlusFlasher flasher("127.0.0.1", srv.port());
    const auto out = flasher.delete_app("viammoto.out");
    srv.join();
    BOOST_TEST(out.ok);
    BOOST_TEST(!out.servos_on);
}

BOOST_AUTO_TEST_CASE(delete_fresh_controller_rc3902_benign) {
    FakeController srv([](tcp::socket& s) {
        (void)read_n(s, std::string("DELETENRB viammoto.out").size());
        send_msg(s, "DELETE Failed (uninstall error) rc=0x3902");
    });
    robot::MotoPlusFlasher flasher("127.0.0.1", srv.port());
    const auto out = flasher.delete_app("viammoto.out");
    srv.join();
    BOOST_TEST(!out.ok);
    BOOST_TEST(!out.servos_on);  // 0x3902 is "nothing to remove", not servos-on
}

BOOST_AUTO_TEST_CASE(download_two_phase_and_byte_identical) {
    const auto payload = make_payload(500);
    const std::string expected_header = "DOWNLOAD viammoto.out," + std::to_string(payload.size());

    std::string got_header;
    std::string got_payload;

    FakeController srv([&](tcp::socket& s) {
        // Read up to the header length. If the flasher (wrongly) glued the payload onto the header,
        // more than header bytes arrive before we send "Start" — captured here and asserted below.
        got_header = read_n(s, expected_header.size());
        send_msg(s, "DOWNLOAD Start");
        got_payload = read_n(s, payload.size());
        send_msg(s, "DOWNLOAD Success");
    });

    robot::MotoPlusFlasher flasher("127.0.0.1", srv.port());
    const auto out = flasher.download_app("viammoto.out", payload, /*reboot=*/true);
    srv.join();

    BOOST_TEST(out.ok);
    BOOST_TEST(!out.servos_on);
    // byte-identical command header, and NOTHING extra arrived before "Start" (two-phase respected)
    BOOST_TEST(got_header == expected_header);
    BOOST_TEST(got_payload.size() == payload.size());
    BOOST_TEST(std::equal(payload.begin(), payload.end(), got_payload.begin()));
}

BOOST_AUTO_TEST_CASE(download_reboot_verb_toggle) {
    std::string got_header;
    FakeController srv([&](tcp::socket& s) {
        got_header = read_n(s, std::string("DOWNLOADNRB viammoto.out,3").size());
        send_msg(s, "DOWNLOAD Start");
        (void)read_n(s, 3);
        send_msg(s, "DOWNLOAD Success");
    });
    robot::MotoPlusFlasher flasher("127.0.0.1", srv.port());
    const auto out = flasher.download_app("viammoto.out", std::vector<char>{1, 2, 3}, /*reboot=*/false);
    srv.join();
    BOOST_TEST(out.ok);
    BOOST_TEST(got_header == "DOWNLOADNRB viammoto.out,3");
}

BOOST_AUTO_TEST_CASE(download_servos_on_rc2010) {
    const auto payload = make_payload(64);
    FakeController srv([&](tcp::socket& s) {
        (void)read_n(s, ("DOWNLOAD viammoto.out," + std::to_string(payload.size())).size());
        send_msg(s, "DOWNLOAD Start");
        (void)read_n(s, payload.size());
        send_msg(s, "DOWNLOAD Failed (install error) rc=0x2010");
    });
    robot::MotoPlusFlasher flasher("127.0.0.1", srv.port());
    const auto out = flasher.download_app("viammoto.out", payload, /*reboot=*/true);
    srv.join();
    BOOST_TEST(!out.ok);
    BOOST_TEST(out.servos_on);  // rc=0x2010 -> caller tells operator to drop servo power
}
