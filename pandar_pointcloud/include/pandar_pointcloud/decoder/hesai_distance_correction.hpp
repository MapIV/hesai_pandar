/// @file distance_correction.hpp
/// @brief Ray-sphere geometric distance correction for Hesai LiDAR points.

#ifndef HESAI_DISTANCE_CORRECTION_HPP
#define HESAI_DISTANCE_CORRECTION_HPP

#include "pandar_pointcloud/decoder/hesai_angle_lookup_table.hpp"
#include <cmath>

namespace hesai
{

/// @brief Optical center offset of the LiDAR transmitter relative to the geometric origin.
/// @details Each Hesai model has a unique optical center. The distance correction algorithm
///          uses this offset to correct the measured range via ray-sphere intersection.
struct OpticalCenter
{
  float x = 0.0f;  ///< X offset in metres.
  float y = 0.0f;  ///< Y offset in metres.
  float z = 0.0f;  ///< Z offset in metres.
};

/// @brief Apply ray-sphere intersection distance correction (GeometricCenter type).
/// @details Corrects for the offset between the optical center and the geometric origin.
///          Modifies azimuth, elevation, and distance in-place.
/// @param oc        Optical center offset for the specific LiDAR model.
/// @param lut       Sine/cosine lookup table.
/// @param azimuth   Horizontal fine angle (modified in-place).
/// @param elevation Vertical fine angle (modified in-place).
/// @param distance  Range in metres (modified in-place).
/// @note Points closer than 0.09 m are skipped (too close for reliable correction).
inline void applyDistanceCorrection(const OpticalCenter& oc, const AngleLookupTable& lut, int& azimuth, int& elevation,
                                    float& distance)
{
  if (distance <= 0.09f)
    return;
  AngleLookupTable::circleRevise(azimuth);
  AngleLookupTable::circleRevise(elevation);
  float tx = lut.cos(elevation) * lut.sin(azimuth);
  float ty = lut.cos(elevation) * lut.cos(azimuth);
  float tz = lut.sin(elevation);
  float d = distance;
  float B = 2.0f * (tx * oc.x + ty * oc.y + tz * oc.z);
  float C = oc.x * oc.x + oc.y * oc.y + oc.z * oc.z - d * d;
  float d_optical = std::sqrt(B * B / 4.0f - C) - B / 2.0f;
  float x = d_optical * tx + oc.x;
  float y = d_optical * ty + oc.y;
  float z = d_optical * tz + oc.z;
  azimuth = static_cast<int>(std::atan2(x, y) * AngleLookupTable::HALF_CIRCLE_FLOAT *
                             AngleLookupTable::FINE_RESOLUTION_FLOAT / M_PI);
  elevation = static_cast<int>(std::asin(z / d) * AngleLookupTable::HALF_CIRCLE_FLOAT *
                               AngleLookupTable::FINE_RESOLUTION_FLOAT / M_PI);
  distance = d;
  AngleLookupTable::circleRevise(azimuth);
  AngleLookupTable::circleRevise(elevation);
}

}  // namespace hesai

#endif  // HESAI_DISTANCE_CORRECTION_HPP
