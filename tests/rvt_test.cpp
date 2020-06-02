/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, PickNik Consulting
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik Consulting nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   Test for Rviz Visual tools
*/

// C++
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <ros/ros.h>

// For visualizing things in rviz
#include <rviz_visual_tools/rviz_visual_tools.h>

class RVTTest
{
public:
  bool initialize()
  {
    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("base", "/rviz_visual_tools"));

    // Allow time to publish messages
    ROS_INFO_STREAM_NAMED("test", "Waiting 4 seconds to start test...");
    return true;
  }

  bool testIsometry3d(const std::string& id, const Eigen::Isometry3d& expect, const Eigen::Isometry3d& actual)
  {
    static const double EPSILON = 0.000001;
    EXPECT_GT(EPSILON, fabs(expect.translation().x() - actual.translation().x()))
        << id << " Translation x - expect: " << expect.translation().x() << " actual: " << actual.translation().x();
    EXPECT_GT(EPSILON, fabs(expect.translation().y() - actual.translation().y()))
        << id << " Translation y - expect: " << expect.translation().y() << " actual: " << actual.translation().y();
    EXPECT_GT(EPSILON, fabs(expect.translation().z() - actual.translation().z()))
        << id << " Translation z - expect: " << expect.translation().z() << " actual: " << actual.translation().z();

    Eigen::Quaterniond q1(expect.rotation());
    Eigen::Quaterniond q2(actual.rotation());
    EXPECT_GT(EPSILON, fabs(q1.x() - q2.x())) << id << " Quaternion x - expect: " << q1.x() << " actual: " << q2.x();
    EXPECT_GT(EPSILON, fabs(q1.y() - q2.y())) << id << " Quaternion y - expect: " << q1.y() << " actual: " << q2.y();
    EXPECT_GT(EPSILON, fabs(q1.z() - q2.z())) << id << " Quaternion z - expect: " << q1.z() << " actual: " << q2.z();
    EXPECT_GT(EPSILON, fabs(q1.w() - q2.w())) << id << " Quaternion w - expect: " << q1.w() << " actual: " << q2.w();

    return true;
  }

  bool testVector(const std::string& id, const std::vector<double>& expect, const std::vector<double>& actual)
  {
    EXPECT_EQ(expect.size(), actual.size()) << id << " Unequal vector sizes";

    static const double EPSILON = 0.000001;
    for (std::size_t i = 0; i < expect.size(); ++i)
    {
      EXPECT_GT(EPSILON, fabs(expect[i] - actual[i])) << "Section " << id << ", Element " << i
                                                      << ", Expect: " << expect[i] << ", Actual: " << actual[i];
    }

    return true;
  }

  // A shared node handle
  // ros::NodeHandle nh_;

  // For visualizing things in rviz
  rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

};  // class

/* Create instance of test class ---------------------------------------------------------- */
RVTTest BASE;

/* Run tests ------------------------------------------------------------------------------ */
TEST(RVTTest, initialize)
{
  ASSERT_TRUE(BASE.initialize());
}

// Test rpy conversion
TEST(RVTTest, test_rpy_conversions)
{
  // Identity conversions with RPY
  Eigen::Isometry3d expected_affine = Eigen::Isometry3d::Identity();
  std::vector<double> xyzrpy;
  BASE.visual_tools_->convertToXYZRPY(expected_affine, xyzrpy);
  std::vector<double> expected_vector;
  expected_vector.push_back(0);
  expected_vector.push_back(0);
  expected_vector.push_back(0);
  expected_vector.push_back(0);
  expected_vector.push_back(0);
  expected_vector.push_back(0);
  EXPECT_TRUE(BASE.testVector("Identity: ", expected_vector, xyzrpy));

  // Identity conversion back to Eigen
  Eigen::Isometry3d expected_affine2 = BASE.visual_tools_->convertFromXYZRPY(xyzrpy, rviz_visual_tools::XYZ);
  EXPECT_TRUE(BASE.testIsometry3d("Identity convert back", expected_affine, expected_affine2));

  // -------------------------------------------------------------------
  // Translation conversions to RPY
  expected_affine.translation().x() = 1;
  expected_affine.translation().y() = 2;
  expected_affine.translation().z() = 3;
  BASE.visual_tools_->convertToXYZRPY(expected_affine, xyzrpy);
  expected_vector[0] = 1;
  expected_vector[1] = 2;
  expected_vector[2] = 3;
  EXPECT_TRUE(BASE.testVector("123: ", expected_vector, xyzrpy));

  // Translation convertion back to Eigen
  expected_affine2 = BASE.visual_tools_->convertFromXYZRPY(xyzrpy, rviz_visual_tools::XYZ);
  EXPECT_TRUE(BASE.testIsometry3d("123 convert back", expected_affine, expected_affine2));

  // Translation convertion back to Eigen via long function
  expected_affine2 = BASE.visual_tools_->convertFromXYZRPY(xyzrpy[0], xyzrpy[1], xyzrpy[2], xyzrpy[3], xyzrpy[4],
                                                           xyzrpy[5], rviz_visual_tools::XYZ);
  EXPECT_TRUE(BASE.testIsometry3d("123 convert back long", expected_affine, expected_affine2));

  // Translation convertion back to Eigen via NEW long function
  expected_affine2 = BASE.visual_tools_->convertFromXYZRPY(xyzrpy[0], xyzrpy[1], xyzrpy[2], xyzrpy[3], xyzrpy[4],
                                                           xyzrpy[5], rviz_visual_tools::XYZ);
  EXPECT_TRUE(BASE.testIsometry3d("123 convert back new long", expected_affine, expected_affine2));

  // -------------------------------------------------------------------
  // Rotation conversions to RPY
  expected_affine = expected_affine * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()) *
                    Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitZ()) *
                    Eigen::AngleAxisd(-0.5 * M_PI, Eigen::Vector3d::UnitX());
  BASE.visual_tools_->convertToXYZRPY(expected_affine, xyzrpy);

  // Rotation convertion back to Eigen
  expected_affine2 = BASE.visual_tools_->convertFromXYZRPY(xyzrpy, rviz_visual_tools::XYZ);
  EXPECT_TRUE(BASE.testIsometry3d("123 convert back", expected_affine, expected_affine2));

  // Rotation convertion back to Eigen via long function
  expected_affine2 = BASE.visual_tools_->convertFromXYZRPY(xyzrpy[0], xyzrpy[1], xyzrpy[2], xyzrpy[3], xyzrpy[4],
                                                           xyzrpy[5], rviz_visual_tools::XYZ);
  EXPECT_TRUE(BASE.testIsometry3d("123 convert back long", expected_affine, expected_affine2));

  // Rotation convertion back to Eigen via NEW long function
  expected_affine2 = BASE.visual_tools_->convertFromXYZRPY(xyzrpy[0], xyzrpy[1], xyzrpy[2], xyzrpy[3], xyzrpy[4],
                                                           xyzrpy[5], rviz_visual_tools::XYZ);
  EXPECT_TRUE(BASE.testIsometry3d("123 convert back new long", expected_affine, expected_affine2));
}

TEST(RVTTest, default_arguments)
{
  // Check for correct number of parameters and correct size of path
  std::vector<geometry_msgs::Point> path1;
  EXPECT_FALSE(BASE.visual_tools_->publishPath(path1, rviz_visual_tools::RED, rviz_visual_tools::MEDIUM));
  path1.resize(1);
  EXPECT_FALSE(BASE.visual_tools_->publishPath(path1, rviz_visual_tools::RED, rviz_visual_tools::MEDIUM));
  path1.resize(2);
  EXPECT_TRUE(BASE.visual_tools_->publishPath(path1, rviz_visual_tools::RED, rviz_visual_tools::MEDIUM));

  EigenSTL::vector_Isometry3d path2;
  EXPECT_FALSE(BASE.visual_tools_->publishPath(path2, rviz_visual_tools::GREEN, rviz_visual_tools::MEDIUM));
  path2.resize(1);
  EXPECT_FALSE(BASE.visual_tools_->publishPath(path2, rviz_visual_tools::RED, rviz_visual_tools::MEDIUM));
  path2.resize(2);
  EXPECT_TRUE(BASE.visual_tools_->publishPath(path2, rviz_visual_tools::RED, rviz_visual_tools::MEDIUM));

  EigenSTL::vector_Vector3d path3;
  EXPECT_FALSE(BASE.visual_tools_->publishPath(path3, rviz_visual_tools::BLUE, rviz_visual_tools::MEDIUM));
  path3.resize(1);
  EXPECT_FALSE(BASE.visual_tools_->publishPath(path3, rviz_visual_tools::RED, rviz_visual_tools::MEDIUM));
  path3.resize(2);
  EXPECT_TRUE(BASE.visual_tools_->publishPath(path3, rviz_visual_tools::RED, rviz_visual_tools::MEDIUM));
}

TEST(RVTTest, get_vector_between_points)
{
  using namespace Eigen;
  const auto x_actual = BASE.visual_tools_->getVectorBetweenPoints(Vector3d::UnitX(), Vector3d::Zero());
  const auto x_expected =
      Isometry3d(Isometry3d::Identity()).translate(Vector3d::UnitX()).rotate(AngleAxisd(M_PI, Vector3d::UnitY()));
  EXPECT_TRUE(BASE.testIsometry3d("get_vector_between_points X", x_expected, x_actual));

  const auto x2_actual = BASE.visual_tools_->getVectorBetweenPoints(2 * Vector3d::UnitX(), Vector3d::Zero());
  const auto x2_expected =
      Isometry3d(Isometry3d::Identity()).translate(2 * Vector3d::UnitX()).rotate(AngleAxisd(M_PI, Vector3d::UnitY()));
  EXPECT_TRUE(BASE.testIsometry3d("get_vector_between_points 2X", x2_expected, x2_actual));

  const auto random_actual = BASE.visual_tools_->getVectorBetweenPoints({ 2.0, 3.0, 4.0 }, { 5.0, 6.0, 7.0 });
  const auto random_expected = []() {
    Eigen::Matrix4d d;
    d << 0.57735, -0.211325, -0.788675, 2, 0.57735, 0.788675, 0.211325, 3, 0.57735, -0.57735, 0.57735, 4, 0, 0, 0, 1;
    return Eigen::Isometry3d(d);
  }();
  EXPECT_TRUE(BASE.testIsometry3d("get_vector_between_points random", random_expected, random_actual));
}

/* Main  ------------------------------------------------------------------------------------- */
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "rviz_visual_tools_tests");
  return RUN_ALL_TESTS();
}

/*
reminders:
EXPECT_FALSE(robot_state.hasFixedLinks());
EXPECT_EQ(robot_state.getFixedLinksCount(), 0);
EXPECT_TRUE(robot_state.getPrimaryFixedLink() == NULL);
EXPECT_GT(robot_state.getFixedLinksMode(), 0);
EXPECT_LT( fabs(vars[0] - 0), EPSILON) << "Virtual joint in wrong position " << vars[0];
*/
