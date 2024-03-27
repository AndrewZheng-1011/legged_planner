#ifndef LEGGED_MATH_UTILS
#define LEGGED_MATH_UTILS

#include <ocs2_core/misc/LoadData.h>

namespace math_utils {

/**
 * @brief Differentiate input vector with central difference method
 *
 * @param data
 * @param dt The time difference between data
 * @return Differentiated vector
 */
std::vector<double> centralDiff(std::vector<double> data, double dt);

/**
 * @brief Differentiate data using difference method (f(x2) - f(x1))/dt
 *
 * @param upper_data Upper data (f(x2))
 * @param lower_data Lower data (f(x1))
 * @param double dt Discrete time
 * @return double
 */
double numericalDiff(double upper_data, double lower_data, double dt);

/**
 * Base to origin rotation matrix | Utilized from ocs2 library
 * @tparam Derived
 * @param [in] eulerAngles
 * @return
 */
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 3> rotationMatrixBaseToOrigin(
    const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAnglesXYZ) {
  // Generated code for:
  //  // inputs are the intrinsic rotation angles in RADIANTS
  //  SCALAR_T sinAlpha = sin(eulerAngles(0));
  //  SCALAR_T cosAlpha = cos(eulerAngles(0));
  //  SCALAR_T sinBeta = sin(eulerAngles(1));
  //  SCALAR_T cosBeta = cos(eulerAngles(1));
  //  SCALAR_T sinGamma = sin(eulerAngles(2));
  //  SCALAR_T cosGamma = cos(eulerAngles(2));
  //
  //  Eigen::Matrix<SCALAR_T, 3, 3> Rx, Ry, Rz;
  //  Rx << SCALAR_T(1), SCALAR_T(0), SCALAR_T(0), SCALAR_T(0), cosAlpha,
  //  -sinAlpha, SCALAR_T(0), sinAlpha, cosAlpha; Ry << cosBeta, SCALAR_T(0),
  //  sinBeta, SCALAR_T(0), SCALAR_T(1), SCALAR_T(0), -sinBeta, SCALAR_T(0),
  //  cosBeta; Rz << cosGamma, -sinGamma, SCALAR_T(0), sinGamma, cosGamma,
  //  SCALAR_T(0), SCALAR_T(0), SCALAR_T(0), SCALAR_T(1);
  //
  //  return Rx * Ry * Rz;
  Eigen::Matrix<SCALAR_T, 3, 3> o_R_b;

  // auxiliary variables
  std::array<SCALAR_T, 8> v{};

  v[0] = cos(eulerAnglesXYZ[1]);
  v[1] = cos(eulerAnglesXYZ[2]);
  o_R_b(0) = v[0] * v[1];
  v[2] = sin(eulerAnglesXYZ[0]);
  v[3] = -v[2];
  o_R_b(6) = sin(eulerAnglesXYZ[1]);
  v[4] = -o_R_b(6);
  v[5] = v[3] * v[4];
  v[6] = cos(eulerAnglesXYZ[0]);
  v[7] = sin(eulerAnglesXYZ[2]);
  o_R_b(1) = v[5] * v[1] + v[6] * v[7];
  v[4] = v[6] * v[4];
  o_R_b(2) = v[4] * v[1] + v[2] * v[7];
  v[7] = -v[7];
  o_R_b(3) = v[0] * v[7];
  o_R_b(4) = v[5] * v[7] + v[6] * v[1];
  o_R_b(5) = v[4] * v[7] + v[2] * v[1];
  o_R_b(7) = v[3] * v[0];
  o_R_b(8) = v[6] * v[0];
  return o_R_b;
}

}  // namespace math_utils
#endif  // LEGGED_BODY_MATH_UTILS