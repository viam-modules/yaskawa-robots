#include <boost/asio.hpp>
#include <chrono>
#include <fstream>
#include <iostream>
#include <list>
#include <memory>
#include <mutex>
#include <ostream>
#include <thread>
#include <utility>

#include "logger.hpp"
#include "robot_socket.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <third_party/trajectories/Trajectory.h>

using namespace robot;
namespace asio = boost::asio;

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
        std::lock_guard<std::mutex> lock(mutex_);
        min_level_ = level;
    }

    viam::yaskawa::LogLevel get_min_level() const noexcept override {
        std::lock_guard<std::mutex> lock(mutex_);
        return min_level_;
    }

   protected:
    void write_log(viam::yaskawa::LogLevel level, std::string_view message) override {
        if (level < get_min_level()) {
            return;
        }

        std::string formatted = format_message(level, message);

        std::lock_guard<std::mutex> lock(mutex_);

        // Write to console
        std::cout << formatted << std::endl;

        // Write to file
        if (file_.is_open()) {
            file_ << formatted << std::endl;
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

    std::string get_timestamp() const {
        using namespace std::chrono;

        auto now = system_clock::now();
        auto now_time_t = system_clock::to_time_t(now);
        auto now_ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;

        std::ostringstream oss;

        std::tm tm_buf;
#if defined(_WIN32)
        localtime_s(&tm_buf, &now_time_t);
#else
        localtime_r(&now_time_t, &tm_buf);
#endif

        oss << std::put_time(&tm_buf, "%Y-%m-%d %H:%M:%S") << '.' << std::setfill('0') << std::setw(3) << now_ms.count();

        return oss.str();
    }

    mutable std::mutex mutex_;
    viam::yaskawa::LogLevel min_level_;
    bool show_timestamps_;
    std::ofstream file_;
};
void move_request(std::shared_ptr<YaskawaController> robot) {
    std::list<Eigen::VectorXd> wpt;
    wpt.emplace_back(Eigen::Vector<double, 6>({0, 0, 0, 0, 0, 0}));
    wpt.emplace_back(Eigen::Vector<double, 6>({1, 0, 0, 1, 1.1, 1}));
    wpt.emplace_back(Eigen::Vector<double, 6>({0, 0, 0, 0, 0, 0}));
    auto ret = robot->move(std::move(wpt), "");
    std::cout << "will wait" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));

    ret->wait();
    std::cout << "Done" << std::endl;
}

void example(asio::io_context& io_context) {
    auto robot = std::make_shared<YaskawaController>(io_context, 0.17, 0.4, "10.1.11.177");

    try {
        std::cout << "Connecting to robot..." << std::endl;
        robot->connect().get();
        std::thread([=] {
            try {
                while (1) {
                    std::cout << "RUN" << std::endl;
                    StatusMessage pos_vel_msg(robot->get_robot_position_velocity_torque().get());
                    std::cout << " Position [" << pos_vel_msg.position[0] << " " << pos_vel_msg.position[1] << " "
                              << pos_vel_msg.position[2] << " " << pos_vel_msg.position[3] << " " << pos_vel_msg.position[4] << " "
                              << pos_vel_msg.position[5] << "] Velocity [ " << pos_vel_msg.velocity[0] << " " << pos_vel_msg.velocity[1]
                              << " " << pos_vel_msg.velocity[2] << " " << pos_vel_msg.velocity[3] << " " << pos_vel_msg.velocity[4] << " "
                              << pos_vel_msg.velocity[5] << "] PositionCorrected [" << pos_vel_msg.position[0] << " "
                              << pos_vel_msg.position[1] << " " << pos_vel_msg.position[2] << " " << pos_vel_msg.position[3] << " "
                              << pos_vel_msg.position[4] << " " << pos_vel_msg.position[5] << "]" << std::endl;

                    RobotStatusMessage robotStatus(robot->get_robot_status().get());
                    std::cout << " Robot status : Powered Drive" << robotStatus.drives_powered << " InMotion " << robotStatus.in_motion
                              << " EStopped " << robotStatus.e_stopped << " Mode " << robotStatus.mode << std::endl;
                    robot->send_heartbeat().get();

                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
            } catch (const std::exception& e) {
                std::cerr << "Monitoring thread error: " << e.what() << std::endl;
            } catch (...) {
                std::cerr << "Monitoring thread encountered unknown error" << std::endl;
            }
        }).detach();
        std::cout << "Reset errors " << robot->reset_errors().get() << std::endl;
        std::cout << "Turning servo power ON..." << std::endl;

        // Example: Query cartesian position and convert to joint angles
        for (int i = 0; i < 10; ++i) {
            auto getCarPos = robot->getCartPosition().get();
            std::cout << "Cartesian position: " << CartesianPosition(getCarPos).toString() << std::endl;

            auto pos = CartesianPosition(getCarPos);
            auto cartPosToAngle = robot->cartPosToAngle(pos).get();
            auto ang = AnglePosition(cartPosToAngle);
            std::cout << "Joint angles: " << ang.toString() << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        //        auto pp = StatusMessage(robot->get_robot_position_velocity_torque().get());

        // auto servo_response = robot->turn_servo_power_on().get();
        // std::cout << "Motion trajectory mode..." << std::endl;
        // robot->setMotionMode(1).get();
        // move_request(robot);
        // std::cout << "Sending test trajectory " << servo_response << std::endl;

        robot->disconnect();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        robot->disconnect();
    }
}

int main() {
    try {
        // Set up custom logger that writes to both file and console
        auto custom_logger = std::make_shared<FileAndConsoleLogger>("robot.log", viam::yaskawa::LogLevel::DEBUG);
        viam::yaskawa::set_global_logger(custom_logger);

        std::cout << "Custom file and console logger initialized. Logs will be written to robot.log" << std::endl;

        // Create io_context with concurrency hint for handling async operations
        constexpr int k_io_context_concurrency_hint = 2;
        asio::io_context io_context(k_io_context_concurrency_hint);

        // Run the io_context in a separate thread to handle async operations
        std::thread io_thread([&io_context]() {
            std::cout << "io context running \n";
            asio::executor_work_guard<asio::io_context::executor_type> work_guard(io_context.get_executor());
            io_context.run();
            std::cout << "IO thread shutting down \n";
        });

        example(io_context);
        std::cout << "stopping io thread" << std::endl;
        io_context.stop();
        io_thread.join();

        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }
}
