// Copyright 2026 Juan Francisco Rascon Crespo

#include <gtest/gtest.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

#include <cmath>

namespace
{
  constexpr double kTolerance{1e-9};

  tf2::Transform make_transform(const double x, const double y, const double yaw)
  {
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    return tf2::Transform{q, tf2::Vector3{x, y, 0.0}};
  }

  tf2::Transform make_global_fr_sim_world_reference_fr_transform(const double delta_x,
                                                                 const double delta_y,
                                                                 const double delta_yaw)
  {
    return make_transform(delta_x, delta_y, delta_yaw).inverse();
  }

  double yaw_from_transform(const tf2::Transform& transform)
  {
    double roll;
    double pitch;
    double yaw;
    tf2::Matrix3x3{transform.getRotation()}.getRPY(roll, pitch, yaw);
    return yaw;
  }

  void expect_transform_near(const tf2::Transform& actual,
                             const double expected_x,
                             const double expected_y,
                             const double expected_yaw)
  {
    EXPECT_NEAR(actual.getOrigin().x(), expected_x, kTolerance);
    EXPECT_NEAR(actual.getOrigin().y(), expected_y, kTolerance);
    EXPECT_NEAR(actual.getOrigin().z(), 0.0, kTolerance);
    EXPECT_NEAR(yaw_from_transform(actual), expected_yaw, kTolerance);
  }
}  // namespace

TEST(TransformMath, DeltaOffsetsAreInvertedBeforeUse)
{
  const tf2::Transform T2_global_fr_sim_world_reference_fr{
    make_global_fr_sim_world_reference_fr_transform(5.0, 2.0, 0.0)};

  expect_transform_near(T2_global_fr_sim_world_reference_fr, -5.0, -2.0, 0.0);
}

TEST(TransformMath, RotatedDeltaOffsetsAreInvertedAsRigidTransforms)
{
  constexpr double half_pi{1.5707963267948966};

  const tf2::Transform T2_global_fr_sim_world_reference_fr{
    make_global_fr_sim_world_reference_fr_transform(5.0, 2.0, half_pi)};

  expect_transform_near(T2_global_fr_sim_world_reference_fr, -2.0, 5.0, -half_pi);
}

TEST(TransformMath, SimulatorPoseIsConvertedIntoGlobalFrame)
{
  const tf2::Transform T2_global_fr_sim_world_reference_fr{
    make_global_fr_sim_world_reference_fr_transform(5.0, 2.0, 0.0)};
  const tf2::Transform T2_sim_world_reference_fr_robot_fr{make_transform(8.0, 6.0, 0.0)};

  const tf2::Transform T2_global_fr_robot_fr{T2_global_fr_sim_world_reference_fr * T2_sim_world_reference_fr_robot_fr};

  expect_transform_near(T2_global_fr_robot_fr, 3.0, 4.0, 0.0);
}

TEST(TransformMath, GlobalOdomTransformPreservesTheLocalizationChain)
{
  const tf2::Transform T2_global_fr_robot_fr{make_transform(10.0, 3.0, 0.4)};
  const tf2::Transform T2_odom_fr_robot_fr{make_transform(2.0, 0.5, 0.1)};
  const tf2::Transform T2_robot_fr_odom_fr{T2_odom_fr_robot_fr.inverse()};

  const tf2::Transform T2_global_fr_odom_fr{T2_global_fr_robot_fr * T2_robot_fr_odom_fr};

  const tf2::Transform T2_global_fr_robot_fr_reconstructed{T2_global_fr_odom_fr * T2_odom_fr_robot_fr};
  expect_transform_near(T2_global_fr_robot_fr_reconstructed, 10.0, 3.0, 0.4);
}

TEST(TransformMath, InitialPoseCorrectionMovesPublishedPoseToDesiredPose)
{
  const tf2::Transform T2_global_fr_sim_world_reference_fr_old{make_transform(0.0, 0.0, 0.0)};
  const tf2::Transform T2_sim_world_reference_fr_robot_fr{make_transform(8.0, 6.0, 0.0)};
  const tf2::Transform T2_global_fr_robot_fr_current{T2_global_fr_sim_world_reference_fr_old *
                                                     T2_sim_world_reference_fr_robot_fr};
  const tf2::Transform T2_global_fr_desired_robot_fr{make_transform(3.0, 4.0, 0.0)};

  const tf2::Transform T2_frame_correction{T2_global_fr_desired_robot_fr * T2_global_fr_robot_fr_current.inverse()};
  const tf2::Transform T2_global_fr_sim_world_reference_fr_new{T2_frame_correction *
                                                               T2_global_fr_sim_world_reference_fr_old};
  const tf2::Transform T2_global_fr_robot_fr_after_correction{T2_global_fr_sim_world_reference_fr_new *
                                                              T2_sim_world_reference_fr_robot_fr};

  expect_transform_near(T2_global_fr_robot_fr_after_correction, 3.0, 4.0, 0.0);
}
