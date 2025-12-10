#ifndef SIMPLE_IK_H
#define SIMPLE_IK_H

#include <Arduino.h>

class SimpleIK {
public:
  // Config
  float thigh_len;
  float calf_len;

  SimpleIK(float thigh, float calf)
    : thigh_len(thigh), calf_len(calf) {}

  // Solve 2D Inverse Kinematics (Planar)
  // y: Horizontal distance from hip pivot (Positive = Forward/Away)
  // z: Vertical distance from hip pivot (Positive = Down)
  // knee_bends_positive: If true, knee bends in Positive servo direction. Else Negative.
  //                      For "Spot-like" (knee backward), this depends on servo orientation.
  //                      Usually: Backward Bend = Negative Angle relative to thigh.
  // Returns true if reachable.
  bool solve(float y, float z, bool knee_bends_positive, float &out_hip_angle, float &out_knee_angle) {

    // 1. Distance from hip to target (hypotenuse)
    float dist_sq = y * y + z * z;
    float dist = sqrt(dist_sq);

    // Check reachability
    if (dist > (thigh_len + calf_len) || dist < abs(thigh_len - calf_len)) {
      return false;  // Target out of reach
    }

    // 2. Law of Cosines for Internal Knee Angle (beta)
    // c^2 = a^2 + b^2 - 2ab*cos(C)
    float cos_beta = (thigh_len * thigh_len + calf_len * calf_len - dist_sq) / (2 * thigh_len * calf_len);
    cos_beta = constrain(cos_beta, -1.0f, 1.0f);
    float beta = acos(cos_beta);

    // Knee Servo Angle: 0 = Straight leg (PI - beta)
    float knee_angle_rad = PI - beta;

    // Apply bend direction constraint
    if (!knee_bends_positive) {
      knee_angle_rad = -knee_angle_rad;
    }

    // 3. Solve Hip Angle
    // Calculate the base angle 'phi' of the target vector relative to Vertical (Z)
    float phi = atan2(y, z);

    // Calculate internal angle 'alpha' between Thigh and Target Vector
    float cos_alpha = (thigh_len * thigh_len + dist_sq - calf_len * calf_len) / (2 * thigh_len * dist);
    cos_alpha = constrain(cos_alpha, -1.0f, 1.0f);
    float alpha = acos(cos_alpha);

    float hip_angle_rad;
    if (!knee_bends_positive) {
      // Knee Backward: Hip angle is shallower than vector angle
      hip_angle_rad = phi - alpha;
    } else {
      // Knee Forward: Hip angle is steeper than vector angle
      hip_angle_rad = phi + alpha;
    }

    // Convert to Degrees and apply Hardware Inversion
    // Invert Hip Angle to match physical servo orientation (Standard: 0 = Horizontal, Solver: 0 = Vertical)
    out_hip_angle = -hip_angle_rad * RAD_TO_DEG;
    out_knee_angle = knee_angle_rad * RAD_TO_DEG;

    return true;
  }
};

#endif
