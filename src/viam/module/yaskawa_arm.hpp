#pragma once

#include <boost/asio/io_context.hpp>
#include <list>
#include <memory>
#include <shared_mutex>

#include <Eigen/Core>
#include <thread>
#include <viam/sdk/components/arm.hpp>
#include <viam/sdk/config/resource.hpp>
#include <viam/sdk/registry/registry.hpp>
#include <viam/sdk/resource/reconfigurable.hpp>
#include "../lib/robot_socket.hpp"

using namespace viam::sdk;
using namespace robot;

class YaskawaArm final : public Arm, public Reconfigurable {
   public:
    /// @brief Returns the common ModelFamily for all implementations
    static const ModelFamily& model_family();

    /// @brief Returns a Model in the correct family for the given model name.
    static Model model(std::string model_name);

    /// @brief Returns a registration for each model of ARM supported by this class.
    static std::vector<std::shared_ptr<ModelRegistration>> create_model_registrations(boost::asio::io_context& io_context);

    explicit YaskawaArm(Model model, const Dependencies& deps, const ResourceConfig& cfg, boost::asio::io_context& io_context);
    ~YaskawaArm() override;

    void reconfigure(const Dependencies& deps, const ResourceConfig& cfg) override;

    /// @brief Get the joint positions of the arm (in degrees)
    /// @param extra Any additional arguments to the method.
    /// @return a vector of joint positions of the arm in degrees
    std::vector<double> get_joint_positions(const ProtoStruct& extra) override;

    /// @brief Move to the the specified joint positions (in degrees)
    /// @param positions The joint positions in degrees to move to
    /// @param extra Any additional arguments to the method.
    void move_to_joint_positions(const std::vector<double>& positions, const ProtoStruct& extra) override;

    /// @brief Move through the specified joint positions (in degrees)
    /// @param positions The joint positions to move through
    /// @param options Optional parameters that should be obeyed during the motion
    /// @param extra Any additional arguments to the method.
    void move_through_joint_positions(const std::vector<std::vector<double>>& positions,
                                      const MoveOptions& options,
                                      const viam::sdk::ProtoStruct& extra) override;

    /// @brief Get the cartesian pose of the end effector
    /// @param extra Any additional arguments to the method.
    /// @return Pose of the end effector with respect to the arm base.
    pose get_end_position(const ProtoStruct& extra) override;

    /// @brief Reports if the arm is in motion.
    bool is_moving() override;

    /// @brief Get the kinematics data associated with the arm.
    /// @param extra Any additional arguments to the method.
    /// @return A variant of kinematics data, with bytes field containing the raw bytes of the file
    /// and the object's type indicating the file format.
    KinematicsData get_kinematics(const ProtoStruct& extra) override;

    /// @brief Stops the Arm.
    /// @param extra Extra arguments to pass to the resource's `stop` method.
    void stop(const ProtoStruct& extra) override;

    /// @brief This is being used as a proxy to move_to_joint_positions except with support for
    /// multiple waypoints
    /// @param command Will contain a std::vector<std::vector<double>> called positions that will
    /// contain joint waypoints
    ProtoStruct do_command(const ProtoStruct& command) override;

    // --------------- UNIMPLEMENTED FUNCTIONS ---------------
    void move_to_position(const pose&, const ProtoStruct&) override {
        throw std::runtime_error("unimplemented");
    }

    // the arm server within RDK will reconstruct the geometries from the kinematics and joint positions if left unimplemented
    std::vector<GeometryConfig> get_geometries(const ProtoStruct&) override {
        throw std::runtime_error("unimplemented");
    }

   private:
    void configure_(const Dependencies& deps, const ResourceConfig& cfg);

    template <template <typename> typename lock_type>
    void check_configured_(const lock_type<std::shared_mutex>&);

    void move_(std::shared_lock<std::shared_mutex> config_rlock, std::list<Eigen::VectorXd> waypoints, const std::string& unix_time_ms);

    template <template <typename> typename lock_type>
    void stop_(const lock_type<std::shared_mutex>&);

    const Model model_;

    std::shared_mutex config_mutex_;
    std::shared_ptr<YaskawaController> robot_;
    std::optional<double> threshold_;
    boost::asio::io_context& io_context_;
    std::filesystem::path resource_root_;
};
