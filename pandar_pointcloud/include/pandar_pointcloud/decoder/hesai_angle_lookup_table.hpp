/// @file angle_lookup_table.hpp
/// @brief Pre-computed sine/cosine lookup table for fast spherical-to-Cartesian conversion.

#ifndef HESAI_ANGLE_LOOKUP_TABLE_HPP
#define HESAI_ANGLE_LOOKUP_TABLE_HPP

#include <cmath>
#include <vector>

namespace hesai
{

/// @brief Pre-computed trigonometric lookup table indexed by fine-resolution angle units.
/// @details Angles are represented as integers in "fine angle" units where a full circle
///          equals 36000 * 256 = 9,216,000 steps. This avoids runtime sin/cos calls
///          during point cloud decoding.
class AngleLookupTable
{
public:
  static constexpr int CIRCLE = 36000 * 256;                    ///< Full circle in fine angle units.
  static constexpr float HALF_CIRCLE_FLOAT = 18000.0f;          ///< Half circle in hundredths of degrees.
  static constexpr int FINE_RESOLUTION_INT = 256;               ///< Sub-steps per hundredth of a degree.
  static constexpr float FINE_RESOLUTION_FLOAT = 256.0f;        ///< Sub-steps per hundredth (float).
  static constexpr int ALL_FINE_RESOLUTION_INT = 25600;         ///< Fine steps per degree (100 * 256).
  static constexpr float ALL_FINE_RESOLUTION_FLOAT = 25600.0f;  ///< Fine steps per degree (float).

  /// @brief Construct the table by pre-computing sin/cos for every fine angle.
  AngleLookupTable()
  {
    sin_.resize(CIRCLE);
    cos_.resize(CIRCLE);
    for (int i = 0; i < CIRCLE; ++i)
    {
      sin_[i] = std::sin(i * 2.0 * M_PI / CIRCLE);
      cos_[i] = std::cos(i * 2.0 * M_PI / CIRCLE);
    }
  }

  /// @brief Look up the sine value for a fine angle.
  /// @param fine_angle Angle in fine units (must be in [0, CIRCLE)).
  /// @return Sine of the angle.
  [[nodiscard]] float sin(int fine_angle) const
  {
    return sin_[fine_angle];
  }

  /// @brief Look up the cosine value for a fine angle.
  /// @param fine_angle Angle in fine units (must be in [0, CIRCLE)).
  /// @return Cosine of the angle.
  [[nodiscard]] float cos(int fine_angle) const
  {
    return cos_[fine_angle];
  }

  /// @brief Wrap a fine angle into the valid range [0, CIRCLE).
  /// @param angle Fine angle value to normalize (modified in-place).
  static void circleRevise(int& angle)
  {
    while (angle < 0)
      angle += CIRCLE;
    while (angle >= CIRCLE)
      angle -= CIRCLE;
  }

  /// @brief Convert degrees to radians.
  /// @param degree Angle in degrees.
  /// @return Angle in radians.
  static constexpr double degreeToRadian(double degree)
  {
    return degree * M_PI / 180.0;
  }

  /// @brief Convert degrees to fine angle units.
  /// @param degrees Angle in degrees.
  /// @return Angle in fine units.
  static int degreeToFine(float degrees)
  {
    return static_cast<int>(degrees * ALL_FINE_RESOLUTION_FLOAT + 0.0625f);
  }

private:
  std::vector<float> sin_;  ///< Pre-computed sine values indexed by fine angle.
  std::vector<float> cos_;  ///< Pre-computed cosine values indexed by fine angle.
};

}  // namespace hesai

#endif  // HESAI_ANGLE_LOOKUP_TABLE_HPP
