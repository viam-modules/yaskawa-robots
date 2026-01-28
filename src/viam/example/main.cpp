#include <algorithm>
#include <boost/asio.hpp>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <iterator>
#include <list>
#include <memory>
#include <mutex>
#include <ostream>
#include <thread>
#include <utility>
#include <vector>

#include "logger.hpp"
#include "robot_socket.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Core/util/Constants.h>
#include <third_party/trajectories/Trajectory.h>
#include <Eigen/Dense>

using namespace robot;
namespace asio = boost::asio;
namespace {

std::vector<CartesianPosition> generateCirclePosition(double r, double lb, double hb, int steps, const CartesianPosition& seed) {
    auto inc = (hb - lb) / (double)steps;
    std::vector<CartesianPosition> out;
    for (int i = 0; i < steps; i++) {
        const double x = r * cos(lb + (inc * (double)i));
        const double y = r * sin(lb + (inc * (double)i));
        CartesianPosition pos(seed);
        pos.x = x;
        pos.y = y;
        out.push_back(std::move(pos));
    }
    return out;
}

void example(asio::io_context& io_context) {
    const viam::sdk::ResourceConfig cfg(
        "type",
        "name",
        "rdk",
        {{"host", "10.1.11.177"}, {"speed_rad_per_sec", 1.1}, {"acceleration_rad_per_sec2", 1.1}, {"group_index", 0}},
        "rdk:component:arm",
        viam::sdk::Model("viam", "yaskawa-robots", "gp12"));

    auto robot = std::make_shared<YaskawaController>(io_context, cfg);
    try {
        std::cout << "Connecting to robot..." << '\n';
        robot->connect().get();
        std::thread([=] {
            try {
                while (1) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    // std::cout << "RUN" << '\n';
                    StatusMessage pos_vel_msg(robot->get_robot_position_velocity_torque().get());
                    std::cout << " Position [" << pos_vel_msg.position[0] << " " << pos_vel_msg.position[1] << " "
                              << pos_vel_msg.position[2] << " " << pos_vel_msg.position[3] << " " << pos_vel_msg.position[4] << " "
                              << pos_vel_msg.position[5] << "] Velocity [ " << pos_vel_msg.velocity[0] << " " << pos_vel_msg.velocity[1]
                              << " " << pos_vel_msg.velocity[2] << " " << pos_vel_msg.velocity[3] << " " << pos_vel_msg.velocity[4] << " "
                              << pos_vel_msg.velocity[5] << "]" << '\n';

                    // RobotStatusMessage robotStatus(robot->get_robot_status().get());
                    // std::cout << " Robot status : Powered Drive" <<
                    // robotStatus.drives_powered << "  InMotion " <<
                    // robotStatus.in_motion
                    //           << " EStopped " << robotStatus.e_stopped << " Mode " <<
                    //           robotStatus.mode << '\n';
                    robot->send_heartbeat().get();
                }
            } catch (const std::exception& e) {
                std::cerr << "Monitoring thread error: " << e.what() << '\n';
            } catch (...) {
                std::cerr << "Monitoring thread encountered unknown error" << '\n';
            }
        }).detach();
        std::cout << "Reset errors " << robot->reset_errors().get() << '\n';
        for (int i = 0; i < 1; i++) {
            auto currentCartPositon = CartesianPosition(robot->getCartPosition().get());
            currentCartPositon.z -= 100;
            StatusMessage pos_vel_msg(robot->get_robot_position_velocity_torque().get());
            auto currentAnglePos = AnglePosition(pos_vel_msg.position);
            // currentAnglePos.toRad()
            auto currentAnglePosEigen = Eigen::VectorXd::Map(pos_vel_msg.position.data(), (long)pos_vel_msg.position.size()).eval();
            auto circleWayPointCart = generateCirclePosition(1009.647, -M_PI / 2, M_PI, 20, currentCartPositon);
            auto circleWayPointAngle = std::list<Eigen::VectorXd>();
            std::transform(
                circleWayPointCart.begin(), circleWayPointCart.end(), std::back_inserter(circleWayPointAngle), [&](CartesianPosition& pos) {
                    auto angle = AnglePosition(robot->cartPosToAngle(pos).get());
                    angle.toRad();
                    return Eigen::VectorXd::Map(angle.pos.data(), (long)angle.pos.size()).eval();
                });

            auto it = std::min_element(circleWayPointAngle.begin(), circleWayPointAngle.end(), [&](const auto& a, const auto& b) -> bool {
                return (a - currentAnglePosEigen).template lpNorm<Eigen::Infinity>() <
                       (b - currentAnglePosEigen).template lpNorm<Eigen::Infinity>();
            });
            auto finalPoints = std::list<Eigen::VectorXd>();
            finalPoints.push_back(currentAnglePosEigen);
            std::copy(it, circleWayPointAngle.end(), std::back_inserter(finalPoints));
            std::copy(circleWayPointAngle.rbegin(), circleWayPointAngle.rend(), std::back_inserter(finalPoints));
            std::copy(circleWayPointAngle.begin(), it, std::back_inserter(finalPoints));
            finalPoints.push_back(currentAnglePosEigen);
            std::cout << "ccurrent pos " << currentAnglePos.toString() << " idx " << std::distance(circleWayPointAngle.begin(), it) << '\n';

            for (auto& p : finalPoints) {
                std::cout << std::format(
                                 " P ({:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f}) - P "
                                 "({:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f}) ",
                                 p[0],
                                 p[1],
                                 p[2],
                                 p[3],
                                 p[4],
                                 p[5],
                                 p[0] * (180.0 / M_PI),
                                 p[1] * (180.0 / M_PI),
                                 p[2] * (180.0 / M_PI),
                                 p[3] * (180.0 / M_PI),
                                 p[4] * (180.0 / M_PI),
                                 p[5] * (180.0 / M_PI))
                          << '\n';
            }

            std::cout << "Turning servo power ON..." << '\n';
            auto servo_response = robot->turn_servo_power_on().get();
            std::cout << "Motion trajectory mode..." << '\n';

            robot->setMotionMode(1).get();
            auto ret = robot->move(finalPoints, "");
            std::cout << "will wait" << '\n';
            ret->wait();
        }

        robot->disconnect();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << '\n';
        robot->disconnect();
    }
}
}  // namespace

/// Custom logger that writes to both file and console
class FileAndConsoleLogger : public viam::yaskawa::ILogger {
   public:
    explicit FileAndConsoleLogger(const std::string& filename, viam::yaskawa::LogLevel min_level = viam::yaskawa::LogLevel::INFO)
        : min_level_(min_level), show_timestamps_(true) {
        file_.open(filename, std::ios::out | std::ios::app);
        if (!file_.is_open()) {
            throw std::runtime_error("Failed to open log file: " + filename);
        }
    }

    ~FileAndConsoleLogger() override {
        if (file_.is_open()) {
            file_.close();
        }
    }

    // Delete copy operations
    FileAndConsoleLogger(const FileAndConsoleLogger&) = delete;
    FileAndConsoleLogger& operator=(const FileAndConsoleLogger&) = delete;

    void set_min_level(viam::yaskawa::LogLevel level) override {
        const std::lock_guard<std::mutex> lock(mutex_);
        min_level_ = level;
    }

    viam::yaskawa::LogLevel get_min_level() const noexcept override {
        const std::lock_guard<std::mutex> lock(mutex_);
        return min_level_;
    }

   protected:
    void write_log(viam::yaskawa::LogLevel level, std::string_view message) override {
        if (level < get_min_level()) {
            return;
        }

        const std::string formatted = format_message(level, message);

        const std::lock_guard<std::mutex> lock(mutex_);

        // Write to console
        std::cout << formatted << '\n';

        // Write to file
        if (file_.is_open()) {
            file_ << formatted << '\n';
            file_.flush();  // Ensure it's written immediately
        }
    }

   private:
    std::string format_message(viam::yaskawa::LogLevel level, std::string_view message) const {
        std::ostringstream oss;

        if (show_timestamps_) {
            oss << "[" << get_timestamp() << "] ";
        }

        oss << "[" << viam::yaskawa::to_string(level) << "] " << message;

        return oss.str();
    }

    static std::string get_timestamp() {
        using namespace std::chrono;

        auto now = system_clock::now();
        auto now_time_t = system_clock::to_time_t(now);
        auto now_ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;

        std::ostringstream oss;

        std::tm tm_buf;
#if defined(_WIN32)
        localtime_s(&tm_buf, &now_time_t);
#else
        auto* result_ptr = localtime_r(&now_time_t, &tm_buf);
        if (result_ptr == NULL) {
            perror("Error converting time with localtime_r");  // Print error message based on errno
            return "Error converting time with localtime_r";   // Indicate error
        }
#endif

        oss << std::put_time(&tm_buf, "%Y-%m-%d %H:%M:%S") << '.' << std::setfill('0') << std::setw(3) << now_ms.count();

        return oss.str();
    }

    mutable std::mutex mutex_;
    viam::yaskawa::LogLevel min_level_;
    bool show_timestamps_;
    std::ofstream file_;
};

int main() {
    try {
        // Set up custom logger that writes to both file and console
        auto custom_logger = std::make_shared<FileAndConsoleLogger>("robot.log", viam::yaskawa::LogLevel::DEBUG);
        viam::yaskawa::set_global_logger(custom_logger);

        std::cout << "Custom file and console logger initialized. Logs will be "
                     "written to robot.log"
                  << '\n';

        // Create io_context with concurrency hint for handling async operations
        constexpr int k_io_context_concurrency_hint = 2;
        asio::io_context io_context(k_io_context_concurrency_hint);

        // Run the io_context in a separate thread to handle async operations
        std::thread io_thread([&io_context]() {
            std::cout << "io context running \n";
            const asio::executor_work_guard<asio::io_context::executor_type> work_guard(io_context.get_executor());
            io_context.run();
            std::cout << "IO thread shutting down \n";
        });

        example(io_context);
        std::cout << "stopping io thread" << '\n';
        io_context.stop();
        io_thread.join();

        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << '\n';
        return 1;
    }
}
