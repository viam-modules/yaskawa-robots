#define BOOST_TEST_MODULE StreamedConversionTest
#include <boost/test/unit_test.hpp>

#include <chrono>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <vector>

#include <viam/lib/robot_socket.hpp>
#include <viam/sdk/components/arm.hpp>

#include "../utils.hpp"

using viam::sdk::Arm;
using namespace std::chrono_literals;

namespace {

constexpr std::size_t k_dof = 6;

// the same velocity and acceleration limit on every joint, in degrees.
streamed_limits make_limits(double velocity_deg_per_sec, double acceleration_deg_per_sec2, std::size_t dof = k_dof) {
    const auto size = static_cast<Eigen::Index>(dof);
    return {Eigen::VectorXd::Constant(size, velocity_deg_per_sec), Eigen::VectorXd::Constant(size, acceleration_deg_per_sec2)};
}

// well above every value the conversion tests use, so they only exercise conversion.
const streamed_limits k_loose_limits = make_limits(1000.0, 1000.0);

viam::sdk::ResourceConfig make_config(viam::sdk::ProtoStruct attributes) {
    return {"arm", "test-arm", "", std::move(attributes), "rdk:component:arm", viam::sdk::Model("test", "test", "test")};
}

// a good 6 dof point with every joint at the same value, so a test can change one thing at a
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
    const auto converted = convert_streamed_point(make_point(0us, 180.0, 90.0), k_loose_limits);

    for (std::size_t i = 0; i < k_dof; ++i) {
        BOOST_CHECK_CLOSE(converted.positions[i], std::numbers::pi, 1e-9);
        BOOST_CHECK_CLOSE(converted.velocities[i], std::numbers::pi / 2.0, 1e-9);
    }
}

BOOST_AUTO_TEST_CASE(accelerations_convert_when_present) {
    const auto converted = convert_streamed_point(make_point(0us, 0.0, 0.0, 360.0), k_loose_limits);

    for (std::size_t i = 0; i < k_dof; ++i) {
        BOOST_CHECK_CLOSE(converted.accelerations[i], 2.0 * std::numbers::pi, 1e-9);
    }
}

// accelerations are optional on the wire and the controller ignores them anyway, so a point
// without them should convert fine instead of being rejected.
BOOST_AUTO_TEST_CASE(accelerations_default_to_zero_when_absent) {
    const auto converted = convert_streamed_point(make_point(0us, 10.0, 5.0), k_loose_limits);

    for (std::size_t i = 0; i < k_dof; ++i) {
        BOOST_CHECK_EQUAL(converted.accelerations[i], 0.0);
    }
}

// the wire struct is always NUMBER_OF_DOF wide no matter how many joints the arm has, so we leave
// the joints past the end at zero instead of letting old data sit there.
BOOST_AUTO_TEST_CASE(unused_joints_stay_zero) {
    const auto converted = convert_streamed_point(make_point(0us, 45.0, 45.0, 45.0), k_loose_limits);

    for (std::size_t i = k_dof; i < static_cast<std::size_t>(NUMBER_OF_DOF); ++i) {
        BOOST_CHECK_EQUAL(converted.positions[i], 0.0);
        BOOST_CHECK_EQUAL(converted.velocities[i], 0.0);
        BOOST_CHECK_EQUAL(converted.accelerations[i], 0.0);
    }
}

BOOST_AUTO_TEST_SUITE_END()

BOOST_AUTO_TEST_SUITE(streamed_time_conversion)

BOOST_AUTO_TEST_CASE(zero_time_converts_to_zero_duration) {
    const auto converted = convert_streamed_point(make_point(0us, 0.0, 0.0), k_loose_limits);

    BOOST_CHECK_EQUAL(converted.time_from_start.sec, 0U);
    BOOST_CHECK_EQUAL(converted.time_from_start.nanos, 0U);
}

BOOST_AUTO_TEST_CASE(sub_second_time_lands_entirely_in_nanos) {
    const auto converted = convert_streamed_point(make_point(500'000us, 0.0, 0.0), k_loose_limits);

    BOOST_CHECK_EQUAL(converted.time_from_start.sec, 0U);
    BOOST_CHECK_EQUAL(converted.time_from_start.nanos, 500'000'000U);
}

BOOST_AUTO_TEST_CASE(time_splits_into_seconds_and_nanos) {
    const auto converted = convert_streamed_point(make_point(2'250'000us, 0.0, 0.0), k_loose_limits);

    BOOST_CHECK_EQUAL(converted.time_from_start.sec, 2U);
    BOOST_CHECK_EQUAL(converted.time_from_start.nanos, 250'000'000U);
}

BOOST_AUTO_TEST_CASE(whole_second_leaves_no_remainder) {
    const auto converted = convert_streamed_point(make_point(3'000'000us, 0.0, 0.0), k_loose_limits);

    BOOST_CHECK_EQUAL(converted.time_from_start.sec, 3U);
    BOOST_CHECK_EQUAL(converted.time_from_start.nanos, 0U);
}

// duration_t is unsigned so there is no way to hold a negative offset. the sdk stub already
// requires the first point to be at zero and the rest to increase, but if a bad time does get here
// we want it to fail rather than wrap around into a huge one.
BOOST_AUTO_TEST_CASE(negative_time_is_rejected) {
    BOOST_CHECK_THROW(convert_streamed_point(make_point(-1us, 0.0, 0.0), k_loose_limits), std::exception);
}

BOOST_AUTO_TEST_SUITE_END()

BOOST_AUTO_TEST_SUITE(streamed_point_validation)

// the controller's queue is parameterized by velocity, so we reject a point that only has
// positions instead of making up velocities for it.
BOOST_AUTO_TEST_CASE(missing_constraints_is_rejected) {
    Arm::trajectory_point point;
    point.time = 0us;
    point.positions = std::vector<double>(k_dof, 0.0);

    BOOST_CHECK_THROW(convert_streamed_point(point, k_loose_limits), std::invalid_argument);
}

BOOST_AUTO_TEST_CASE(wrong_position_count_is_rejected) {
    auto point = make_point(0us, 0.0, 0.0);
    point.positions.pop_back();

    BOOST_CHECK_THROW(convert_streamed_point(point, k_loose_limits), std::invalid_argument);
}

BOOST_AUTO_TEST_CASE(wrong_velocity_count_is_rejected) {
    auto point = make_point(0us, 0.0, 0.0);
    point.constraints->velocities_degs_per_sec.push_back(0.0);

    BOOST_CHECK_THROW(convert_streamed_point(point, k_loose_limits), std::invalid_argument);
}

BOOST_AUTO_TEST_CASE(wrong_acceleration_count_is_rejected) {
    auto point = make_point(0us, 0.0, 0.0, 1.0);
    point.constraints->accelerations_degs_per_sec2->pop_back();

    BOOST_CHECK_THROW(convert_streamed_point(point, k_loose_limits), std::invalid_argument);
}

BOOST_AUTO_TEST_SUITE_END()

BOOST_AUTO_TEST_SUITE(streamed_limit_checks)

constexpr double k_velocity_limit = 100.0;
constexpr double k_acceleration_limit = 200.0;

const streamed_limits k_limits = make_limits(k_velocity_limit, k_acceleration_limit);

BOOST_AUTO_TEST_CASE(velocities_within_the_limit_are_accepted) {
    for (const double velocity : {0.0, 50.0, -50.0, 99.999, -99.999}) {
        BOOST_TEST_CONTEXT("velocity " << velocity) {
            BOOST_CHECK_NO_THROW(convert_streamed_point(make_point(0us, 0.0, velocity), k_limits));
        }
    }
}

// the limit is inclusive, so a client planning right up to it is not rejected.
BOOST_AUTO_TEST_CASE(velocities_exactly_at_the_limit_are_accepted) {
    BOOST_CHECK_NO_THROW(convert_streamed_point(make_point(0us, 0.0, k_velocity_limit), k_limits));
    BOOST_CHECK_NO_THROW(convert_streamed_point(make_point(0us, 0.0, -k_velocity_limit), k_limits));
}

// negative values are checked by magnitude, so moving the other way does not get around the limit.
BOOST_AUTO_TEST_CASE(velocities_over_the_limit_are_rejected) {
    for (const double velocity : {100.001, -100.001, 150.0, -150.0, 1e6}) {
        BOOST_TEST_CONTEXT("velocity " << velocity) {
            BOOST_CHECK_THROW(convert_streamed_point(make_point(0us, 0.0, velocity), k_limits), std::invalid_argument);
        }
    }
}

BOOST_AUTO_TEST_CASE(accelerations_within_or_at_the_limit_are_accepted) {
    for (const double acceleration : {0.0, 150.0, -150.0, k_acceleration_limit, -k_acceleration_limit}) {
        BOOST_TEST_CONTEXT("acceleration " << acceleration) {
            BOOST_CHECK_NO_THROW(convert_streamed_point(make_point(0us, 0.0, 0.0, acceleration), k_limits));
        }
    }
}

BOOST_AUTO_TEST_CASE(accelerations_over_the_limit_are_rejected) {
    for (const double acceleration : {200.001, -200.001, 1e6}) {
        BOOST_TEST_CONTEXT("acceleration " << acceleration) {
            BOOST_CHECK_THROW(convert_streamed_point(make_point(0us, 0.0, 0.0, acceleration), k_limits), std::invalid_argument);
        }
    }
}

// accelerations are optional, so a point without them only has its velocities checked, even
// against an acceleration limit of zero.
BOOST_AUTO_TEST_CASE(absent_accelerations_are_not_checked) {
    BOOST_CHECK_NO_THROW(convert_streamed_point(make_point(0us, 0.0, 50.0), make_limits(k_velocity_limit, 0.0)));
}

// a zero limit is a real one meaning the joint does not move, so only zero itself gets through.
BOOST_AUTO_TEST_CASE(a_zero_limit_accepts_only_zero) {
    const auto zero_limits = make_limits(0.0, 0.0);

    BOOST_CHECK_NO_THROW(convert_streamed_point(make_point(0us, 0.0, 0.0, 0.0), zero_limits));
    BOOST_CHECK_THROW(convert_streamed_point(make_point(0us, 0.0, 0.001), zero_limits), std::invalid_argument);
    BOOST_CHECK_THROW(convert_streamed_point(make_point(0us, 0.0, 0.0, 0.001), zero_limits), std::invalid_argument);
}

// either kind of violation is enough on its own to reject the point.
BOOST_AUTO_TEST_CASE(velocity_and_acceleration_are_checked_independently) {
    BOOST_CHECK_THROW(convert_streamed_point(make_point(0us, 0.0, 150.0, 50.0), k_limits), std::invalid_argument);
    BOOST_CHECK_THROW(convert_streamed_point(make_point(0us, 0.0, 50.0, 250.0), k_limits), std::invalid_argument);
}

BOOST_AUTO_TEST_CASE(one_joint_over_the_limit_rejects_the_point) {
    auto point = make_point(0us, 0.0, 50.0, 50.0);
    point.constraints->velocities_degs_per_sec[3] = 100.5;
    BOOST_CHECK_THROW(convert_streamed_point(point, k_limits), std::invalid_argument);

    point = make_point(0us, 0.0, 50.0, 50.0);
    (*point.constraints->accelerations_degs_per_sec2)[5] = -200.5;
    BOOST_CHECK_THROW(convert_streamed_point(point, k_limits), std::invalid_argument);
}

// a value that is fine for one joint can be over the limit for another, so each joint has to be
// checked against its own limit rather than against any shared one.
BOOST_AUTO_TEST_CASE(each_joint_is_checked_against_its_own_limit) {
    streamed_limits limits{Eigen::VectorXd(k_dof), Eigen::VectorXd(k_dof)};
    limits.velocity_degs_per_sec << 10.0, 20.0, 30.0, 40.0, 50.0, 60.0;
    limits.acceleration_degs_per_sec2 << 15.0, 25.0, 35.0, 45.0, 55.0, 65.0;

    auto at_limits = make_point(0us, 0.0, 0.0, 0.0);
    for (std::size_t i = 0; i < k_dof; ++i) {
        at_limits.constraints->velocities_degs_per_sec[i] = limits.velocity_degs_per_sec[static_cast<Eigen::Index>(i)];
        (*at_limits.constraints->accelerations_degs_per_sec2)[i] = limits.acceleration_degs_per_sec2[static_cast<Eigen::Index>(i)];
    }
    BOOST_REQUIRE_NO_THROW(convert_streamed_point(at_limits, limits));

    for (std::size_t i = 0; i < k_dof; ++i) {
        BOOST_TEST_CONTEXT("joint " << i) {
            auto over_velocity = at_limits;
            over_velocity.constraints->velocities_degs_per_sec[i] += 0.01;
            BOOST_CHECK_THROW(convert_streamed_point(over_velocity, limits), std::invalid_argument);

            auto over_acceleration = at_limits;
            (*over_acceleration.constraints->accelerations_degs_per_sec2)[i] += 0.01;
            BOOST_CHECK_THROW(convert_streamed_point(over_acceleration, limits), std::invalid_argument);
        }
    }
}

// a NaN compares false against everything, so a careless "over the limit" check would let it
// through. these are not numbers the arm can run, so they must be rejected.
BOOST_AUTO_TEST_CASE(non_finite_values_are_rejected) {
    for (const double bad : {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()}) {
        BOOST_TEST_CONTEXT("value " << bad) {
            BOOST_CHECK_THROW(convert_streamed_point(make_point(0us, 0.0, bad), k_limits), std::invalid_argument);
            BOOST_CHECK_THROW(convert_streamed_point(make_point(0us, 0.0, 0.0, bad), k_limits), std::invalid_argument);
        }
    }
}

BOOST_AUTO_TEST_SUITE_END()

BOOST_AUTO_TEST_SUITE(streamed_limit_construction)

BOOST_AUTO_TEST_CASE(limits_convert_from_radians_to_degrees) {
    const auto limits = make_streamed_limits(Eigen::VectorXd::Constant(k_dof, std::numbers::pi),
                                             Eigen::VectorXd::Constant(k_dof, 2.0 * std::numbers::pi),
                                             1.0);

    for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(k_dof); ++i) {
        BOOST_CHECK_CLOSE(limits.velocity_degs_per_sec[i], 180.0, 1e-9);
        BOOST_CHECK_CLOSE(limits.acceleration_degs_per_sec2[i], 360.0, 1e-9);
    }
}

// the tolerance lets a point land a little over the configured limit, and no further.
BOOST_AUTO_TEST_CASE(tolerance_ratio_widens_the_limits) {
    const auto limits = make_streamed_limits(Eigen::VectorXd::Constant(k_dof, std::numbers::pi / 2.0),
                                             Eigen::VectorXd::Constant(k_dof, std::numbers::pi),
                                             1.01);

    BOOST_CHECK_CLOSE(limits.velocity_degs_per_sec[0], 90.9, 1e-9);
    BOOST_CHECK_CLOSE(limits.acceleration_degs_per_sec2[0], 181.8, 1e-9);

    BOOST_CHECK_NO_THROW(convert_streamed_point(make_point(0us, 0.0, 90.85, 181.75), limits));
    BOOST_CHECK_THROW(convert_streamed_point(make_point(0us, 0.0, 90.95), limits), std::invalid_argument);
    BOOST_CHECK_THROW(convert_streamed_point(make_point(0us, 0.0, 0.0, 181.85), limits), std::invalid_argument);
}

// with a ratio of 1 the configured limit is the bound, so just over it is rejected.
BOOST_AUTO_TEST_CASE(a_tolerance_ratio_of_one_adds_no_slack) {
    const auto limits = make_streamed_limits(Eigen::VectorXd::Constant(k_dof, std::numbers::pi / 2.0),
                                             Eigen::VectorXd::Constant(k_dof, std::numbers::pi),
                                             1.0);

    BOOST_CHECK_NO_THROW(convert_streamed_point(make_point(0us, 0.0, 89.99, 179.99), limits));
    BOOST_CHECK_THROW(convert_streamed_point(make_point(0us, 0.0, 90.01), limits), std::invalid_argument);
    BOOST_CHECK_THROW(convert_streamed_point(make_point(0us, 0.0, 0.0, 180.01), limits), std::invalid_argument);
}

BOOST_AUTO_TEST_SUITE_END()

BOOST_AUTO_TEST_SUITE(stream_limit_tolerance_ratio_parsing)

BOOST_AUTO_TEST_CASE(absent_ratio_is_nullopt) {
    BOOST_CHECK(!parse_stream_limit_tolerance_ratio(make_config({})).has_value());
}

BOOST_AUTO_TEST_CASE(ratios_of_one_or_more_are_accepted) {
    BOOST_CHECK_EQUAL(parse_stream_limit_tolerance_ratio(make_config({{"stream_limit_tolerance_ratio", 1.0}})).value(), 1.0);
    BOOST_CHECK_EQUAL(parse_stream_limit_tolerance_ratio(make_config({{"stream_limit_tolerance_ratio", 1.05}})).value(), 1.05);
}

BOOST_AUTO_TEST_CASE(ratios_below_one_are_rejected) {
    for (const double ratio : {0.99, 0.0, -1.0, std::numeric_limits<double>::quiet_NaN()}) {
        BOOST_TEST_CONTEXT("ratio " << ratio) {
            BOOST_CHECK_THROW(parse_stream_limit_tolerance_ratio(make_config({{"stream_limit_tolerance_ratio", ratio}})),
                              std::invalid_argument);
        }
    }
}

BOOST_AUTO_TEST_CASE(non_numeric_ratio_is_rejected) {
    BOOST_CHECK_THROW(parse_stream_limit_tolerance_ratio(make_config({{"stream_limit_tolerance_ratio", std::string{"1.05"}}})),
                      std::invalid_argument);
}

BOOST_AUTO_TEST_SUITE_END()

BOOST_AUTO_TEST_SUITE(streamed_batch_conversion)

BOOST_AUTO_TEST_CASE(batch_preserves_order) {
    const std::vector<Arm::trajectory_point> batch{
        make_point(0us, 0.0, 0.0),
        make_point(1'000'000us, 90.0, 10.0),
        make_point(2'000'000us, 180.0, 20.0),
    };

    const auto converted = convert_streamed_batch(batch, k_loose_limits);

    BOOST_REQUIRE_EQUAL(converted.size(), 3U);
    BOOST_CHECK_EQUAL(converted[0].time_from_start.sec, 0U);
    BOOST_CHECK_EQUAL(converted[1].time_from_start.sec, 1U);
    BOOST_CHECK_EQUAL(converted[2].time_from_start.sec, 2U);
    BOOST_CHECK_CLOSE(converted[1].positions[0], std::numbers::pi / 2.0, 1e-9);
    BOOST_CHECK_CLOSE(converted[2].positions[0], std::numbers::pi, 1e-9);
}

BOOST_AUTO_TEST_CASE(empty_batch_converts_to_nothing) {
    BOOST_CHECK_EQUAL(convert_streamed_batch({}, k_loose_limits).size(), 0U);
}

BOOST_AUTO_TEST_CASE(dof_beyond_protocol_capacity_is_rejected) {
    const auto too_many_joints = make_limits(1000.0, 1000.0, static_cast<std::size_t>(NUMBER_OF_DOF) + 1);
    BOOST_CHECK_THROW(convert_streamed_batch({}, too_many_joints), std::invalid_argument);
}

BOOST_AUTO_TEST_CASE(zero_dof_is_rejected) {
    BOOST_CHECK_THROW(convert_streamed_batch({}, make_limits(1000.0, 1000.0, 0)), std::invalid_argument);
}

// one bad point fails the whole batch. feeding the arm part of a trajectory would leave it running
// a motion the caller never described.
BOOST_AUTO_TEST_CASE(one_bad_point_fails_the_batch) {
    std::vector<Arm::trajectory_point> batch{make_point(0us, 0.0, 0.0), make_point(1'000'000us, 90.0, 10.0)};
    batch[1].constraints = boost::none;

    BOOST_CHECK_THROW(convert_streamed_batch(batch, k_loose_limits), std::invalid_argument);
}

BOOST_AUTO_TEST_CASE(batch_within_the_limits_is_accepted) {
    const std::vector<Arm::trajectory_point> batch{
        make_point(0us, 0.0, 0.0, 0.0), make_point(1'000'000us, 10.0, 100.0, 200.0), make_point(2'000'000us, 20.0, -100.0, -200.0)};

    BOOST_CHECK_EQUAL(convert_streamed_batch(batch, make_limits(100.0, 200.0)).size(), 3U);
}

// a point over the limit anywhere in the batch, not just the first, fails all of it.
BOOST_AUTO_TEST_CASE(one_point_over_the_limit_fails_the_batch) {
    const std::vector<Arm::trajectory_point> batch{
        make_point(0us, 0.0, 0.0), make_point(1'000'000us, 10.0, 50.0), make_point(2'000'000us, 20.0, 100.5)};

    BOOST_CHECK_THROW(convert_streamed_batch(batch, make_limits(100.0, 200.0)), std::invalid_argument);
}

BOOST_AUTO_TEST_CASE(mismatched_limit_lengths_are_rejected) {
    const streamed_limits limits{Eigen::VectorXd::Constant(k_dof, 1000.0), Eigen::VectorXd::Constant(k_dof - 1, 1000.0)};

    BOOST_CHECK_THROW(convert_streamed_batch({make_point(0us, 0.0, 0.0)}, limits), std::invalid_argument);
}

BOOST_AUTO_TEST_SUITE_END()

// the controllers we care about report a 4ms interpolation period, so 250 Hz is the fastest stream
// they can run.
BOOST_AUTO_TEST_SUITE(streamed_spacing)

constexpr auto k_period = std::chrono::microseconds{4000};

BOOST_AUTO_TEST_CASE(spacing_at_the_interpolation_period_is_accepted) {
    const std::vector<Arm::trajectory_point> batch{make_point(0us, 0.0, 0.0), make_point(4000us, 1.0, 0.0), make_point(8000us, 2.0, 0.0)};

    std::optional<std::chrono::microseconds> previous;
    BOOST_CHECK_NO_THROW(check_streamed_spacing(batch, k_period, previous));
    BOOST_REQUIRE(previous.has_value());
    BOOST_CHECK(*previous == 8000us);
}

BOOST_AUTO_TEST_CASE(spacing_below_the_interpolation_period_is_rejected) {
    const std::vector<Arm::trajectory_point> batch{make_point(0us, 0.0, 0.0), make_point(3999us, 1.0, 0.0)};

    std::optional<std::chrono::microseconds> previous;
    BOOST_CHECK_THROW(check_streamed_spacing(batch, k_period, previous), std::invalid_argument);
}

// the gap between the last point of one batch and the first of the next is a real gap in the
// trajectory, so splitting points across batches should not get them past the check.
BOOST_AUTO_TEST_CASE(spacing_is_checked_across_batch_boundaries) {
    std::optional<std::chrono::microseconds> previous;
    const std::vector<Arm::trajectory_point> first{make_point(0us, 0.0, 0.0), make_point(4000us, 1.0, 0.0)};
    BOOST_REQUIRE_NO_THROW(check_streamed_spacing(first, k_period, previous));

    const std::vector<Arm::trajectory_point> second{make_point(5000us, 2.0, 0.0)};
    BOOST_CHECK_THROW(check_streamed_spacing(second, k_period, previous), std::invalid_argument);
}

// the first point has nothing before it, so any start time is fine.
BOOST_AUTO_TEST_CASE(the_first_point_is_not_checked) {
    const std::vector<Arm::trajectory_point> batch{make_point(1us, 0.0, 0.0)};

    std::optional<std::chrono::microseconds> previous;
    BOOST_CHECK_NO_THROW(check_streamed_spacing(batch, k_period, previous));
}

BOOST_AUTO_TEST_CASE(an_empty_batch_leaves_the_previous_time_alone) {
    std::optional<std::chrono::microseconds> previous{4000us};

    BOOST_CHECK_NO_THROW(check_streamed_spacing({}, k_period, previous));
    BOOST_REQUIRE(previous.has_value());
    BOOST_CHECK(*previous == 4000us);
}

BOOST_AUTO_TEST_SUITE_END()
