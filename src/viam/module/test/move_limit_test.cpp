#define BOOST_TEST_MODULE MoveLimitTest
#include <boost/test/unit_test.hpp>

#include <vector>

#include <boost/variant.hpp>

#include <Eigen/Dense>
#include <viam/lib/robot_socket.hpp>

#include "../utils.hpp"

constexpr int k_dof = 6;

BOOST_AUTO_TEST_SUITE(move_limit_tests)

BOOST_AUTO_TEST_CASE(test_move_limit_both_vector) {
    Eigen::VectorXd vel_limits = Eigen::VectorXd::Zero(k_dof);
    Eigen::VectorXd acc_limits = Eigen::VectorXd::Zero(k_dof);

    const boost::variant<double, std::vector<double>> vel = std::vector<double>{10.0, 20.0, 30.0, 40.0, 50.0, 60.0};
    const boost::variant<double, std::vector<double>> acc = std::vector<double>{100.0, 200.0, 300.0, 400.0, 500.0, 600.0};

    BOOST_CHECK_NO_THROW(apply_move_limit(vel_limits, vel));
    BOOST_CHECK_NO_THROW(apply_move_limit(acc_limits, acc));

    const std::array<double, k_dof> expected_vel{10.0, 20.0, 30.0, 40.0, 50.0, 60.0};
    const std::array<double, k_dof> expected_acc{100.0, 200.0, 300.0, 400.0, 500.0, 600.0};
    for (size_t i = 0; i < k_dof; ++i) {
        BOOST_CHECK_CLOSE(vel_limits[static_cast<Eigen::Index>(i)], degrees_to_radians(expected_vel[i]), 1e-9);
        BOOST_CHECK_CLOSE(acc_limits[static_cast<Eigen::Index>(i)], degrees_to_radians(expected_acc[i]), 1e-9);
    }
}

BOOST_AUTO_TEST_CASE(test_move_limit_scalar_fills_all_joints) {
    Eigen::VectorXd vel_limits = Eigen::VectorXd::Zero(k_dof);
    Eigen::VectorXd acc_limits = Eigen::VectorXd::Zero(k_dof);

    const boost::variant<double, std::vector<double>> vel = std::vector<double>{15.0, 25.0, 35.0, 45.0, 55.0, 65.0};
    const boost::variant<double, std::vector<double>> acc = 180.0;

    BOOST_CHECK_NO_THROW(apply_move_limit(vel_limits, vel));
    BOOST_CHECK_NO_THROW(apply_move_limit(acc_limits, acc));

    const std::array<double, k_dof> expected_vel{15.0, 25.0, 35.0, 45.0, 55.0, 65.0};
    for (size_t i = 0; i < k_dof; ++i) {
        BOOST_CHECK_CLOSE(vel_limits[static_cast<Eigen::Index>(i)], degrees_to_radians(expected_vel[i]), 1e-9);
    }

    const double expected_acc = degrees_to_radians(180.0);
    for (size_t i = 0; i < k_dof; ++i) {
        BOOST_CHECK_CLOSE(acc_limits[static_cast<Eigen::Index>(i)], expected_acc, 1e-9);
    }
}

BOOST_AUTO_TEST_CASE(test_move_limit_scalar_zero_throws) {
    Eigen::VectorXd limits = Eigen::VectorXd::Constant(k_dof, 42.0);

    const boost::variant<double, std::vector<double>> zero = 0.0;
    const boost::variant<double, std::vector<double>> negative = -5.0;

    BOOST_CHECK_THROW(apply_move_limit(limits, zero), std::invalid_argument);
    for (int i = 0; i < k_dof; ++i) {
        BOOST_CHECK_EQUAL(limits[i], 42.0);
    }

    BOOST_CHECK_THROW(apply_move_limit(limits, negative), std::invalid_argument);
    for (int i = 0; i < k_dof; ++i) {
        BOOST_CHECK_EQUAL(limits[i], 42.0);
    }
}

BOOST_AUTO_TEST_CASE(test_move_limit_vector_too_few_elements) {
    Eigen::VectorXd limits = Eigen::VectorXd::Constant(k_dof, 42.0);

    const boost::variant<double, std::vector<double>> value = std::vector<double>{10.0, 20.0, 30.0};

    BOOST_CHECK_THROW(apply_move_limit(limits, value), std::invalid_argument);

    for (int i = 0; i < k_dof; ++i) {
        BOOST_CHECK_EQUAL(limits[i], 42.0);
    }
}

BOOST_AUTO_TEST_CASE(test_move_limit_vector_too_many_elements) {
    Eigen::VectorXd limits = Eigen::VectorXd::Constant(k_dof, 42.0);

    const boost::variant<double, std::vector<double>> value = std::vector<double>{1, 2, 3, 4, 5, 6, 7, 8};

    BOOST_CHECK_THROW(apply_move_limit(limits, value), std::invalid_argument);

    for (int i = 0; i < k_dof; ++i) {
        BOOST_CHECK_EQUAL(limits[i], 42.0);
    }
}

BOOST_AUTO_TEST_CASE(test_move_limit_vector_negative_element) {
    Eigen::VectorXd limits = Eigen::VectorXd::Constant(k_dof, 42.0);

    const boost::variant<double, std::vector<double>> value = std::vector<double>{10.0, 20.0, -5.0, 40.0, 50.0, 60.0};

    BOOST_CHECK_THROW(apply_move_limit(limits, value), std::invalid_argument);

    for (int i = 0; i < k_dof; ++i) {
        BOOST_CHECK_EQUAL(limits[i], 42.0);
    }
}

BOOST_AUTO_TEST_SUITE_END()
