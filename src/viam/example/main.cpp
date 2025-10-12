#include <boost/asio.hpp>
#include <chrono>
#include <iostream>
#include <list>
#include <memory>
#include <ostream>
#include <thread>
#include <utility>

#include "robot_socket.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <third_party/trajectories/Trajectory.h>

using namespace robot;
namespace asio = boost::asio;
void move_request(std::shared_ptr<YaskawaController> robot) {
    std::list<Eigen::VectorXd> wpt;
    for (int i = 0; i < 10; i++) {
        wpt.emplace_back(Eigen::Vector<double, 6>({i * 0.3, i * 0.3, i * 0.3, i * 0.3, i * 0.3, i * 0.3}));
    }
    auto ret = robot->move(std::move(wpt), "");
    std::cout << "will wait" << std::endl;
    ret->wait();
    std::cout << "Done" << std::endl;
}

void example(asio::io_context& io_context) {
    auto robot = std::make_shared<YaskawaController>(io_context, 1.0, 2.0, "10.1.11.177");
    try {
        std::cout << "Connecting to robot..." << std::endl;
        robot->connect().get();
        std::cout << "Turning servo power ON..." << std::endl;
        auto servo_response = robot->turn_servo_power_on().get();
        // move_request(robot);
        std::cout << "Sending test trajectory " << servo_response << std::endl;
        auto test_traj = robot->send_test_trajectory().get();

        for (int i = 0; i < 100; i++) {
            StatusMessage pos_vel_msg(robot->get_robot_position_velocity_torque().get());
            std::cout << " Position [" << pos_vel_msg.position[0] << " " << pos_vel_msg.position[1] << " " << pos_vel_msg.position[2] << " "
                      << pos_vel_msg.position[3] << " " << pos_vel_msg.position[4] << " " << pos_vel_msg.position[5] << "] Velocity [ "
                      << pos_vel_msg.velocity[0] << " " << pos_vel_msg.velocity[1] << " " << pos_vel_msg.velocity[2] << " "
                      << pos_vel_msg.velocity[3] << " " << pos_vel_msg.velocity[4] << " " << pos_vel_msg.velocity[5]
                      << "] PositionCorrected [" << pos_vel_msg.position[0] << " " << pos_vel_msg.position[1] << " "
                      << pos_vel_msg.position[2] << " " << pos_vel_msg.position[3] << " " << pos_vel_msg.position[4] << " "
                      << pos_vel_msg.position[5] << "]" << std::endl;
            RobotStatusMessage robotStatus(robot->get_robot_status().get());
            std::cout << " Robot status : Powered Drive" << robotStatus.drives_powered << " InMotion " << robotStatus.in_motion
                      << " EStopped " << robotStatus.e_stopped << " Mode " << robotStatus.mode << std::endl;
            robot->send_heartbeat().get();

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        robot->disconnect();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        robot->disconnect();
    }
}

int main() {
    try {
        asio::io_context io_context(2);

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
