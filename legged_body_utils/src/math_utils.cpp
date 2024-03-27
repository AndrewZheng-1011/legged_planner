#include "legged_body_utils/math_utils.h"

namespace math_utils {

std::vector<double> centralDiff(std::vector<double> data, double dt) {
  std::vector<double> data_diff;
  for (int i = 0; i < data.size(); i++) {
    // Compute lower and upper indices, with forward/backward difference at the
    // ends
    int lower_index = std::max(i - 1, 0);
    int upper_index = std::min(i + 1, (int)data.size() - 1);

    double estimate = (data[upper_index] - data[lower_index]) /
                      (dt * (upper_index - lower_index));
    data_diff.push_back(estimate);
  }

  return data_diff;
}

double numericalDiff(double upper_data, double lower_data, double dt) {
  return (upper_data - lower_data) / dt;
}
}  // namespace math_utils