#define BOOST_TEST_MODULE KinematicModelTest
#include <boost/test/unit_test.hpp>

#include <filesystem>
#include <optional>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

#include <Eigen/Dense>
#include <viam/lib/robot_socket.hpp>
#include <viam/sdk/common/instance.hpp>
#include <viam/sdk/common/kinematic_model_file.hpp>
#include <viam/sdk/common/kinematics.hpp>

#include "../utils.hpp"

namespace {

using viam::sdk::KinematicJoint;
using viam::sdk::KinematicModel;
using viam::sdk::RevoluteJoint;

// These are built field by field since the build treats a designated initializer that leaves a
// member out as an error.
viam::sdk::KinematicLink link(std::string id, std::string parent) {
    viam::sdk::KinematicLink result;
    result.id = std::move(id);
    result.parent = std::move(parent);
    return result;
}

KinematicJoint revolute(std::string id, std::string parent, double min_deg, double max_deg) {
    RevoluteJoint spec;
    spec.min_deg = min_deg;
    spec.max_deg = max_deg;

    KinematicJoint result;
    result.id = std::move(id);
    result.parent = std::move(parent);
    result.axis = {0, 0, 1};
    result.spec = spec;
    return result;
}

viam::sdk::KinematicProperties operating_at(double freq_hz) {
    viam::sdk::KinematicProperties result;
    result.operating_freq_hz = freq_hz;
    return result;
}

// A two joint model, trimmed to what the limits touch. Keeping it small and local means the
// assertions below are about the limits and not about whichever shipped model we picked.
KinematicModel two_joint_model() {
    KinematicModel model;
    model.name = "test_arm";
    model.links = {link("base_link", "world"), link("link_1", "joint_1"), link("link_2", "joint_2")};
    model.joints = {revolute("joint_1", "base_link", -170.0, 170.0), revolute("joint_2", "link_1", -105.0, 155.0)};
    return model;
}

Eigen::VectorXd vec(std::vector<double> values) {
    return Eigen::VectorXd::Map(values.data(), static_cast<Eigen::Index>(values.size())).eval();
}

const RevoluteJoint& revolute_spec(const KinematicJoint& joint) {
    BOOST_REQUIRE(std::holds_alternative<RevoluteJoint>(joint.spec));
    return std::get<RevoluteJoint>(joint.spec);
}

// kinematic_model_for_config warns through the SDK logger, which throws without an instance.
struct sdk_instance_fixture {
    viam::sdk::Instance instance;
};

}  // namespace

BOOST_TEST_GLOBAL_FIXTURE(sdk_instance_fixture);

BOOST_AUTO_TEST_SUITE(kinematic_model_tests)

BOOST_AUTO_TEST_CASE(test_limits_are_narrowed_in_degrees_per_joint) {
    // Distinct per joint, so transposing the two joints or broadcasting one value fails here.
    const auto velocity = vec({degrees_to_radians(110.0), degrees_to_radians(120.0)});
    const auto acceleration = vec({degrees_to_radians(210.0), degrees_to_radians(220.0)});

    auto model = two_joint_model();
    narrow_to_configured_limits(model, velocity, acceleration);

    BOOST_REQUIRE_EQUAL(model.joints.size(), 2U);
    BOOST_CHECK_CLOSE(revolute_spec(model.joints[0]).max_velocity_deg_per_sec.value(), 110.0, 1e-9);
    BOOST_CHECK_CLOSE(revolute_spec(model.joints[0]).max_acceleration_deg_per_sec2.value(), 210.0, 1e-9);
    BOOST_CHECK_CLOSE(revolute_spec(model.joints[1]).max_velocity_deg_per_sec.value(), 120.0, 1e-9);
    BOOST_CHECK_CLOSE(revolute_spec(model.joints[1]).max_acceleration_deg_per_sec2.value(), 220.0, 1e-9);
}

BOOST_AUTO_TEST_CASE(test_narrowing_leaves_the_rest_of_the_model_alone) {
    const auto limits = vec({1.0, 1.0});
    auto model = two_joint_model();
    narrow_to_configured_limits(model, limits, limits);

    // Position bounds, ids, parents, and axes have to survive, since the motion service still
    // needs them and we only meant to set two fields. Clearing those two has to give the model
    // we started with.
    for (auto& joint : model.joints) {
        auto& spec = std::get<RevoluteJoint>(joint.spec);
        spec.max_velocity_deg_per_sec.reset();
        spec.max_acceleration_deg_per_sec2.reset();
    }
    BOOST_CHECK(model == two_joint_model());
}

BOOST_AUTO_TEST_CASE(test_joint_count_mismatch_throws_and_leaves_the_model_alone) {
    // A config whose DOF disagrees with the model is not describing this arm, so we would rather
    // narrow nothing than the joints we happen to have values for.
    for (const auto& limits : {vec({1.0}), vec({1.0, 1.0, 1.0}), Eigen::VectorXd{}}) {
        auto model = two_joint_model();
        BOOST_CHECK_THROW(narrow_to_configured_limits(model, limits, limits), std::invalid_argument);
        BOOST_CHECK(model == two_joint_model());
    }
}

BOOST_AUTO_TEST_CASE(test_velocity_and_acceleration_lengths_must_agree) {
    auto model = two_joint_model();
    BOOST_CHECK_THROW(narrow_to_configured_limits(model, vec({1.0, 1.0}), vec({1.0})), std::invalid_argument);
}

BOOST_AUTO_TEST_CASE(test_zero_limits_are_set_not_dropped) {
    // Only an absent limit means unbounded. A zero limit is a real one saying the joint does not
    // move, so it has to come through as zero.
    const auto zeros = vec({0.0, 0.0});
    auto model = two_joint_model();
    narrow_to_configured_limits(model, zeros, zeros);

    for (const auto& joint : model.joints) {
        BOOST_REQUIRE(revolute_spec(joint).max_velocity_deg_per_sec.has_value());
        BOOST_REQUIRE(revolute_spec(joint).max_acceleration_deg_per_sec2.has_value());
        BOOST_CHECK_EQUAL(*revolute_spec(joint).max_velocity_deg_per_sec, 0.0);
        BOOST_CHECK_EQUAL(*revolute_spec(joint).max_acceleration_deg_per_sec2, 0.0);
    }
}

BOOST_AUTO_TEST_CASE(test_negative_limits_are_refused) {
    // Config validation already rejects these, but a negative limit must not reach the model if
    // that ever stops being true.
    auto model = two_joint_model();
    BOOST_CHECK_THROW(narrow_to_configured_limits(model, vec({1.0, -1.0}), vec({1.0, 1.0})), std::exception);
    BOOST_CHECK_THROW(narrow_to_configured_limits(model, vec({1.0, 1.0}), vec({-1.0, 1.0})), std::exception);
    BOOST_CHECK(model == two_joint_model());
}

BOOST_AUTO_TEST_CASE(test_non_revolute_and_mimic_joints_are_refused) {
    const auto limits = vec({1.0, 1.0});

    // A prismatic joint holds mm/s, so the degrees we would write are the wrong unit for it.
    auto prismatic = two_joint_model();
    prismatic.joints[1].spec = viam::sdk::PrismaticJoint{};
    BOOST_CHECK_THROW(narrow_to_configured_limits(prismatic, limits, limits), std::invalid_argument);

    // A mimic joint takes its limits from the joint it follows.
    auto mimic = two_joint_model();
    mimic.joints[1].mimic = viam::sdk::MimicJoint{"joint_1", 1.0, 0.0};
    BOOST_CHECK_THROW(narrow_to_configured_limits(mimic, limits, limits), std::exception);
}

BOOST_AUTO_TEST_CASE(test_a_speed_in_the_file_can_be_lowered_but_not_raised) {
    // A limit already in the file is the model's own claim about the hardware, so a config may
    // ask for less than it and never for more. No shipped model carries one today.
    auto with_speeds = two_joint_model();
    for (auto& joint : with_speeds.joints) {
        auto& spec = std::get<RevoluteJoint>(joint.spec);
        spec.max_velocity_deg_per_sec = 90.0;
        spec.max_acceleration_deg_per_sec2 = 45.0;
    }

    auto lowered = with_speeds;
    narrow_to_configured_limits(
        lowered, vec({degrees_to_radians(60.0), degrees_to_radians(60.0)}), vec({degrees_to_radians(30.0), degrees_to_radians(30.0)}));
    BOOST_CHECK_CLOSE(revolute_spec(lowered.joints[0]).max_velocity_deg_per_sec.value(), 60.0, 1e-9);
    BOOST_CHECK_CLOSE(revolute_spec(lowered.joints[0]).max_acceleration_deg_per_sec2.value(), 30.0, 1e-9);

    auto raised = with_speeds;
    BOOST_CHECK_THROW(
        narrow_to_configured_limits(
            raised, vec({degrees_to_radians(120.0), degrees_to_radians(60.0)}), vec({degrees_to_radians(30.0), degrees_to_radians(30.0)})),
        std::exception);
    BOOST_CHECK(raised == with_speeds);
}

BOOST_AUTO_TEST_CASE(test_served_model_carries_limits_and_operating_frequency) {
    const auto velocity = vec({degrees_to_radians(110.0), degrees_to_radians(120.0)});
    const auto acceleration = vec({degrees_to_radians(210.0), degrees_to_radians(220.0)});

    const auto served = kinematic_model_for_config(two_joint_model(), velocity, acceleration, 25.0);

    BOOST_CHECK_CLOSE(revolute_spec(served.joints[1]).max_velocity_deg_per_sec.value(), 120.0, 1e-9);
    BOOST_CHECK_CLOSE(revolute_spec(served.joints[1]).max_acceleration_deg_per_sec2.value(), 220.0, 1e-9);

    BOOST_REQUIRE(served.properties.has_value());
    BOOST_CHECK(served.properties->operating_freq_hz == std::optional<double>{25.0});
    BOOST_CHECK(!served.properties->max_linear_tcp_speed_mm_per_sec.has_value());
}

BOOST_AUTO_TEST_CASE(test_served_model_tolerates_a_dof_mismatch) {
    // RDK builds the machine's frame system from this model, so a config whose DOF disagrees with
    // the model has to get the file's model back without limits and not an error.
    const auto limits = vec({1.0, 1.0, 1.0});

    KinematicModel served;
    BOOST_REQUIRE_NO_THROW(served = kinematic_model_for_config(two_joint_model(), limits, limits, 3.0));

    auto expected = two_joint_model();
    expected.properties = operating_at(3.0);
    BOOST_CHECK(served == expected);
}

BOOST_AUTO_TEST_CASE(test_every_shipped_model_loads_and_narrows_at_the_default_dof) {
    // A default config gives us k_default_dof limits, so every shipped model has to have that
    // many joints or it is served without limits. This is the test that fails when a new model
    // file arrives with a different joint count, or in a format the loader does not read.
    const auto limits = Eigen::VectorXd::Constant(k_default_dof, degrees_to_radians(90.0));

    const std::filesystem::path kinematics_dir{VIAM_YASKAWA_TEST_KINEMATICS_DIR};
    std::vector<std::filesystem::path> shipped;
    for (const auto& entry : std::filesystem::directory_iterator{kinematics_dir}) {
        if (entry.path().extension() == ".json") {
            shipped.push_back(entry.path());
        }
    }
    BOOST_REQUIRE_MESSAGE(!shipped.empty(), "no shipped kinematics files found in " + kinematics_dir.string());

    for (const auto& path : shipped) {
        BOOST_TEST_CONTEXT(path.filename().string()) {
            KinematicModel model;
            BOOST_REQUIRE_NO_THROW(model = viam::sdk::read_kinematic_model_file(path.string()));
            BOOST_REQUIRE_NO_THROW(narrow_to_configured_limits(model, limits, limits));

            BOOST_REQUIRE_EQUAL(model.joints.size(), static_cast<std::size_t>(k_default_dof));
            for (const auto& joint : model.joints) {
                // every shipped joint is position bounded, and those bounds have to survive.
                BOOST_CHECK(revolute_spec(joint).min_deg.has_value());
                BOOST_CHECK(revolute_spec(joint).max_deg.has_value());
                BOOST_CHECK_CLOSE(revolute_spec(joint).max_velocity_deg_per_sec.value(), 90.0, 1e-9);
                BOOST_CHECK_CLOSE(revolute_spec(joint).max_acceleration_deg_per_sec2.value(), 90.0, 1e-9);
            }
        }
    }
}

BOOST_AUTO_TEST_SUITE_END()
