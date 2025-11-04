#include "utils.hpp"
#include <Eigen/Dense>

/// Transform a vector using the inverse of a rotation matrix
/// @param vector The vector to transform
/// @param rotation_matrix The rotation matrix (applied in transpose/inverse)
/// @return The transformed vector
Eigen::Vector3d transform_vector(const Eigen::Vector3d& vector, const Eigen::Matrix3d& rotation_matrix) {
    return rotation_matrix.transpose() * vector;
}
