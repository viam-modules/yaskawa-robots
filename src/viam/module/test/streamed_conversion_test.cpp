#define BOOST_TEST_MODULE StreamedConversionTest
#include <boost/test/unit_test.hpp>

#include <chrono>
#include <stdexcept>
#include <vector>

#include <viam/lib/robot_socket.hpp>
#include <viam/sdk/components/arm.hpp>

#include "../utils.hpp"

using viam::sdk::Arm;
using namespace std::chrono_literals;

namespace {

constexpr std::size_t k_dof = 6;

// A well-formed 6-DOF point, all joints at the same value, so tests can perturb one thing at a
// time.
Arm::trajectory_point make_point(std::chrono::microseconds time,
                                 double position_deg,
                                 double velocity_deg_per_sec,
                                 boost::optional<double> acceleration_deg_per_sec2 = boost::none) {
    Arm::trajectory_point point;
    point.time = time;
    point.positions = std::vector<double>(k_dof, position_deg);

    Arm::trajectory_point::kinematic_constraints constraints;
    constraints.velocities_degs_per_sec = std::vector<double>(k_dof, velocity_deg_per_sec);
    if (acceleration_deg_per_sec2) {
        constraints.accelerations_degs_per_sec2 = std::vector<double>(k_dof, *acceleration_deg_per_sec2);
    }
    point.constraints = std::move(constraints);
    return point;
}

}  // namespace

BOOST_AUTO_TEST_SUITE(streamed_point_conversion)

BOOST_AUTO_TEST_CASE(positions_and_velocities_convert_to_radians) {
    const auto converted = convert_streamed_point(make_point(0us, 180.0, 90.0), k_dof);

    for (std::size_t i = 0; i < k_dof; ++i) {
        BOOST_CHECK_CLOSE(converted.positions[i], M_PI, 1e-9);
        BOOST_CHECK_CLOSE(converted.velocities[i], M_PI / 2.0, 1e-9);
    }
}

BOOST_AUTO_TEST_CASE(accelerations_convert_when_present) {
    const auto converted = convert_streamed_point(make_point(0us, 0.0, 0.0, 360.0), k_dof);

    for (std::size_t i = 0; i < k_dof; ++i) {
        BOOST_CHECK_CLOSE(converted.accelerations[i], 2.0 * M_PI, 1e-9);
    }
}

// Accelerations are optional on the wire and the controller ignores them anyway, so a point
// without them converts cleanly rather than being rejected.
BOOST_AUTO_TEST_CASE(accelerations_default_to_zero_when_absent) {
    const auto converted = convert_streamed_point(make_point(0us, 10.0, 5.0), k_dof);

    for (std::size_t i = 0; i < k_dof; ++i) {
        BOOST_CHECK_EQUAL(converted.accelerations[i], 0.0);
    }
}

// Joints past the arm's DOF are left at zero rather than carrying stale data: the wire struct is
// always NUMBER_OF_DOF wide regardless of how many joints the arm has.
BOOST_AUTO_TEST_CASE(unused_joints_stay_zero) {
    const auto converted = convert_streamed_point(make_point(0us, 45.0, 45.0, 45.0), k_dof);

    for (std::size_t i = k_dof; i < static_cast<std::size_t>(NUMBER_OF_DOF); ++i) {
        BOOST_CHECK_EQUAL(converted.positions[i], 0.0);
        BOOST_CHECK_EQUAL(converted.velocities[i], 0.0);
        BOOST_CHECK_EQUAL(converted.accelerations[i], 0.0);
    }
}

BOOST_AUTO_TEST_SUITE_END()

BOOST_AUTO_TEST_SUITE(streamed_time_conversion)

BOOST_AUTO_TEST_CASE(zero_time_converts_to_zero_duration) {
    const auto converted = convert_streamed_point(make_point(0us, 0.0, 0.0), k_dof);

    BOOST_CHECK_EQUAL(converted.time_from_start.sec, 0U);
    BOOST_CHECK_EQUAL(converted.time_from_start.nanos, 0U);
}

BOOST_AUTO_TEST_CASE(sub_second_time_lands_entirely_in_nanos) {
    const auto converted = convert_streamed_point(make_point(500'000us, 0.0, 0.0), k_dof);

    BOOST_CHECK_EQUAL(converted.time_from_start.sec, 0U);
    BOOST_CHECK_EQUAL(converted.time_from_start.nanos, 500'000'000U);
}

BOOST_AUTO_TEST_CASE(time_splits_into_seconds_and_nanos) {
    const auto converted = convert_streamed_point(make_point(2'250'000us, 0.0, 0.0), k_dof);

    BOOST_CHECK_EQUAL(converted.time_from_start.sec, 2U);
    BOOST_CHECK_EQUAL(converted.time_from_start.nanos, 250'000'000U);
}

BOOST_AUTO_TEST_CASE(whole_second_leaves_no_remainder) {
    const auto converted = convert_streamed_point(make_point(3'000'000us, 0.0, 0.0), k_dof);

    BOOST_CHECK_EQUAL(converted.time_from_start.sec, 3U);
    BOOST_CHECK_EQUAL(converted.time_from_start.nanos, 0U);
}

// `duration_t` is unsigned, so a negative offset has no representation. The SDK stub requires the
// first point to be at zero and the rest to increase, but a bad time must fail loudly rather than
// wrap around into an enormous one.
BOOST_AUTO_TEST_CASE(negative_time_is_rejected) {
    BOOST_CHECK_THROW(convert_streamed_point(make_point(-1us, 0.0, 0.0), k_dof), std::exception);
}

BOOST_AUTO_TEST_SUITE_END()

BOOST_AUTO_TEST_SUITE(streamed_point_validation)

// The controller drives a velocity-parameterized queue, so a position-only point is rejected
// rather than having velocities synthesized for it.
BOOST_AUTO_TEST_CASE(missing_constraints_is_rejected) {
    Arm::trajectory_point point;
    point.time = 0us;
    point.positions = std::vector<double>(k_dof, 0.0);

    BOOST_CHECK_THROW(convert_streamed_point(point, k_dof), std::invalid_argument);
}

BOOST_AUTO_TEST_CASE(wrong_position_count_is_rejected) {
    auto point = make_point(0us, 0.0, 0.0);
    point.positions.pop_back();

    BOOST_CHECK_THROW(convert_streamed_point(point, k_dof), std::invalid_argument);
}

BOOST_AUTO_TEST_CASE(wrong_velocity_count_is_rejected) {
    auto point = make_point(0us, 0.0, 0.0);
    point.constraints->velocities_degs_per_sec.push_back(0.0);

    BOOST_CHECK_THROW(convert_streamed_point(point, k_dof), std::invalid_argument);
}

BOOST_AUTO_TEST_CASE(wrong_acceleration_count_is_rejected) {
    auto point = make_point(0us, 0.0, 0.0, 1.0);
    point.constraints->accelerations_degs_per_sec2->pop_back();

    BOOST_CHECK_THROW(convert_streamed_point(point, k_dof), std::invalid_argument);
}

BOOST_AUTO_TEST_SUITE_END()

BOOST_AUTO_TEST_SUITE(streamed_batch_conversion)

BOOST_AUTO_TEST_CASE(batch_preserves_order) {
    const std::vector<Arm::trajectory_point> batch{
        make_point(0us, 0.0, 0.0),
        make_point(1'000'000us, 90.0, 10.0),
        make_point(2'000'000us, 180.0, 20.0),
    };

    const auto converted = convert_streamed_batch(batch, k_dof);

    BOOST_REQUIRE_EQUAL(converted.size(), 3U);
    BOOST_CHECK_EQUAL(converted[0].time_from_start.sec, 0U);
    BOOST_CHECK_EQUAL(converted[1].time_from_start.sec, 1U);
    BOOST_CHECK_EQUAL(converted[2].time_from_start.sec, 2U);
    BOOST_CHECK_CLOSE(converted[1].positions[0], M_PI / 2.0, 1e-9);
    BOOST_CHECK_CLOSE(converted[2].positions[0], M_PI, 1e-9);
}

BOOST_AUTO_TEST_CASE(empty_batch_converts_to_nothing) {
    BOOST_CHECK_EQUAL(convert_streamed_batch({}, k_dof).size(), 0U);
}

BOOST_AUTO_TEST_CASE(dof_beyond_protocol_capacity_is_rejected) {
    BOOST_CHECK_THROW(convert_streamed_batch({}, static_cast<std::size_t>(NUMBER_OF_DOF) + 1), std::invalid_argument);
}

BOOST_AUTO_TEST_CASE(zero_dof_is_rejected) {
    BOOST_CHECK_THROW(convert_streamed_batch({}, 0), std::invalid_argument);
}

// One bad point fails the whole batch: partially feeding a trajectory to the arm would leave it
// executing a motion the caller never described.
BOOST_AUTO_TEST_CASE(one_bad_point_fails_the_batch) {
    std::vector<Arm::trajectory_point> batch{make_point(0us, 0.0, 0.0), make_point(1'000'000us, 90.0, 10.0)};
    batch[1].constraints = boost::none;

    BOOST_CHECK_THROW(convert_streamed_batch(batch, k_dof), std::invalid_argument);
}

BOOST_AUTO_TEST_SUITE_END()
