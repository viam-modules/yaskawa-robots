#define BOOST_TEST_MODULE SvaJointLimitsTest
#include <boost/test/unit_test.hpp>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <json/json.h>

#include <Eigen/Dense>
#include <viam/lib/robot_socket.hpp>

#include "../utils.hpp"

namespace {

// A two-joint SVA document, trimmed to the fields the patch touches. Keeping it small and local
// means the assertions below are about the patch and not about whichever shipped model we picked.
constexpr char k_sva_two_joints[] = R"({
  "name": "test_arm",
  "kinematic_param_type": "SVA",
  "links": [
    {"id": "base_link", "parent": "world"},
    {"id": "link_1", "parent": "joint_1"},
    {"id": "link_2", "parent": "joint_2"}
  ],
  "joints": [
    {"id": "joint_1", "type": "revolute", "parent": "base_link",
     "axis": {"x": 0, "y": 0, "z": 1}, "min": -170.0, "max": 170.0},
    {"id": "joint_2", "type": "revolute", "parent": "link_1",
     "axis": {"x": 0, "y": 0, "z": 1}, "min": -105.0, "max": 155.0}
  ]
})";

Json::Value parse(const std::string& text) {
    Json::Value root;
    std::istringstream in{text};
    const Json::CharReaderBuilder reader_builder;
    std::string errs;
    BOOST_REQUIRE_MESSAGE(Json::parseFromStream(reader_builder, in, &root, &errs), errs);
    return root;
}

Eigen::VectorXd vec(std::vector<double> values) {
    return Eigen::VectorXd::Map(values.data(), static_cast<Eigen::Index>(values.size())).eval();
}

}  // namespace

BOOST_AUTO_TEST_SUITE(sva_joint_limits_tests)

BOOST_AUTO_TEST_CASE(test_limits_are_written_in_degrees_per_joint) {
    // Distinct per joint, so transposing the two joints or broadcasting one value fails here.
    const auto velocity = vec({degrees_to_radians(110.0), degrees_to_radians(120.0)});
    const auto acceleration = vec({degrees_to_radians(210.0), degrees_to_radians(220.0)});

    const Json::Value patched = parse(sva_with_joint_limits(k_sva_two_joints, velocity, acceleration));

    BOOST_REQUIRE_EQUAL(patched["joints"].size(), 2U);
    BOOST_CHECK_CLOSE(patched["joints"][0]["max_velocity"].asDouble(), 110.0, 1e-9);
    BOOST_CHECK_CLOSE(patched["joints"][0]["max_acceleration"].asDouble(), 210.0, 1e-9);
    BOOST_CHECK_CLOSE(patched["joints"][1]["max_velocity"].asDouble(), 120.0, 1e-9);
    BOOST_CHECK_CLOSE(patched["joints"][1]["max_acceleration"].asDouble(), 220.0, 1e-9);
}

BOOST_AUTO_TEST_CASE(test_patch_leaves_the_rest_of_the_document_alone) {
    const auto limits = vec({1.0, 1.0});
    const Json::Value patched = parse(sva_with_joint_limits(k_sva_two_joints, limits, limits));
    const Json::Value original = parse(k_sva_two_joints);

    BOOST_CHECK_EQUAL(patched["name"].asString(), "test_arm");
    BOOST_CHECK_EQUAL(patched["kinematic_param_type"].asString(), "SVA");
    BOOST_CHECK(patched["links"] == original["links"]);

    // Position bounds, ids, parents, and axes have to survive, since the motion service still
    // needs them and we only meant to add two fields.
    for (Json::ArrayIndex i = 0; i < patched["joints"].size(); ++i) {
        for (const char* field : {"id", "type", "parent", "axis", "min", "max"}) {
            BOOST_CHECK(patched["joints"][i][field] == original["joints"][i][field]);
        }
    }
}

BOOST_AUTO_TEST_CASE(test_joint_count_mismatch_throws) {
    // Two joints in the document, six limits configured. Patching the first two would publish a
    // document that silently disagrees with the config, so this has to fail.
    const auto six = Eigen::VectorXd::Constant(k_default_dof, 1.0);
    BOOST_CHECK_THROW(sva_with_joint_limits(k_sva_two_joints, six, six), std::invalid_argument);

    const auto one = vec({1.0});
    BOOST_CHECK_THROW(sva_with_joint_limits(k_sva_two_joints, one, one), std::invalid_argument);
}

BOOST_AUTO_TEST_CASE(test_velocity_and_acceleration_lengths_must_agree) {
    BOOST_CHECK_THROW(sva_with_joint_limits(k_sva_two_joints, vec({1.0, 1.0}), vec({1.0})), std::invalid_argument);
}

BOOST_AUTO_TEST_CASE(test_non_sva_and_malformed_documents_throw) {
    const auto limits = vec({1.0, 1.0});

    BOOST_CHECK_THROW(sva_with_joint_limits("{ not json", limits, limits), std::invalid_argument);

    constexpr char urdf_ish[] = R"({"kinematic_param_type": "URDF", "joints": []})";
    BOOST_CHECK_THROW(sva_with_joint_limits(urdf_ish, limits, limits), std::invalid_argument);

    constexpr char no_joints[] = R"({"kinematic_param_type": "SVA", "links": []})";
    BOOST_CHECK_THROW(sva_with_joint_limits(no_joints, limits, limits), std::invalid_argument);
}

BOOST_AUTO_TEST_CASE(test_zero_limits_are_written_not_dropped) {
    // Zero is a real limit of zero on the wire, not a way of saying unbounded, so it has to be
    // written. Yaskawa's own config validation rejects a zero element, but the schema permits one
    // and dropping it would describe an axis that cannot move as unbounded.
    const Json::Value patched = parse(sva_with_joint_limits(k_sva_two_joints, vec({0.0, 1.0}), vec({1.0, 0.0})));

    BOOST_REQUIRE(patched["joints"][0].isMember("max_velocity"));
    BOOST_CHECK_EQUAL(patched["joints"][0]["max_velocity"].asDouble(), 0.0);
    BOOST_REQUIRE(patched["joints"][1].isMember("max_acceleration"));
    BOOST_CHECK_EQUAL(patched["joints"][1]["max_acceleration"].asDouble(), 0.0);
}

BOOST_AUTO_TEST_CASE(test_absent_param_type_is_treated_as_sva) {
    // RDK reads a missing `kinematic_param_type` as SVA (referenceframe/model_json.go), so we have
    // to as well, or a shipped file that omits it loses its limits.
    constexpr char no_param_type[] = R"({
      "joints": [
        {"id": "joint_1", "type": "revolute", "min": -170.0, "max": 170.0}
      ]
    })";

    const auto limits = vec({degrees_to_radians(90.0)});
    const Json::Value patched = parse(sva_with_joint_limits(no_param_type, limits, limits));
    BOOST_CHECK_CLOSE(patched["joints"][0]["max_velocity"].asDouble(), 90.0, 1e-9);
}

BOOST_AUTO_TEST_CASE(test_non_revolute_and_mimic_joints_are_refused) {
    const auto limits = vec({degrees_to_radians(90.0)});

    // RDK does not convert `max_velocity` for a prismatic joint, so a radian value would be read as
    // millimetres per second. We have no mm/s to offer from a `speed_rad_per_sec` config.
    constexpr char prismatic[] = R"({
      "kinematic_param_type": "SVA",
      "joints": [{"id": "rail", "type": "prismatic", "min": 0.0, "max": 500.0}]
    })";
    BOOST_CHECK_THROW(sva_with_joint_limits(prismatic, limits, limits), std::invalid_argument);

    // A mimic joint carrying its own limits makes RDK reject the entire model with
    // ErrMimicWithLimits, so publishing one would be worse than publishing no limits at all.
    constexpr char mimic[] = R"({
      "kinematic_param_type": "SVA",
      "joints": [{"id": "follower", "type": "revolute", "mimic": {"source": "joint_1"}}]
    })";
    BOOST_CHECK_THROW(sva_with_joint_limits(mimic, limits, limits), std::invalid_argument);

    // A joint with no `type` is not something RDK can build a frame from either.
    constexpr char untyped[] = R"({
      "kinematic_param_type": "SVA",
      "joints": [{"id": "mystery", "min": -1.0, "max": 1.0}]
    })";
    BOOST_CHECK_THROW(sva_with_joint_limits(untyped, limits, limits), std::invalid_argument);
}

BOOST_AUTO_TEST_CASE(test_every_shipped_model_patches_at_the_default_dof) {
    // A default config gives us k_default_dof limits, so every shipped document has to have that
    // many joints or get_kinematics throws for that model. This is the test that fails when a new
    // model file arrives with a different joint count.
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
        std::ifstream in{path};
        BOOST_REQUIRE_MESSAGE(in, "unable to open " + path.string());
        std::ostringstream buffer;
        buffer << in.rdbuf();
        const std::string text = buffer.str();

        BOOST_TEST_CONTEXT(path.filename().string()) {
            std::string patched_text;
            BOOST_REQUIRE_NO_THROW(patched_text = sva_with_joint_limits(text, limits, limits));

            const Json::Value patched = parse(patched_text);
            BOOST_REQUIRE_EQUAL(patched["joints"].size(), static_cast<Json::ArrayIndex>(k_default_dof));
            for (Json::ArrayIndex i = 0; i < patched["joints"].size(); ++i) {
                BOOST_CHECK_CLOSE(patched["joints"][i]["max_velocity"].asDouble(), 90.0, 1e-9);
                BOOST_CHECK_CLOSE(patched["joints"][i]["max_acceleration"].asDouble(), 90.0, 1e-9);
            }
        }
    }
}

BOOST_AUTO_TEST_SUITE_END()
