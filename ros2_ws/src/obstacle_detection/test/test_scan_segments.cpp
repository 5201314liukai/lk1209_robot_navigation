#include <algorithm>
#include <cmath>
#include <limits>

#include "gtest/gtest.h"
#include "obstacle_detection/scan_processor.hpp"

namespace
{
sensor_msgs::msg::LaserScan wall(float x, size_t beams)
{
  sensor_msgs::msg::LaserScan scan;
  scan.angle_increment = 0.01F;
  scan.angle_min = -0.5F * (beams - 1) * scan.angle_increment;
  scan.range_min = 0.12F;
  scan.range_max = 8.0F;
  for (size_t i = 0; i < beams; ++i) {
    scan.ranges.push_back(x / std::cos(scan.angle_min + i * scan.angle_increment));
  }
  return scan;
}

TEST(ScanSegments, FitsStraightWallInSensorFrame)
{
  auto segments = obstacle_detection::ScanProcessor().features(wall(2.0F, 15)).segments;
  ASSERT_EQ(segments.size(), 1U);
  EXPECT_NEAR(segments[0].first_point.x, 2.0, 1e-5);
  EXPECT_NEAR(segments[0].last_point.x, 2.0, 1e-5);
  EXPECT_NEAR(segments[0].length(), 4.0 * std::tan(0.07), 1e-4);
}

TEST(ScanSegments, InvalidBeamAlwaysSplitsEvenWithinMergeDistance)
{
  for (float invalid : {std::numeric_limits<float>::infinity(),
      std::numeric_limits<float>::quiet_NaN(), 0.08F, 9.0F}) {
    auto scan = wall(0.5F, 15);
    scan.ranges[7] = invalid;
    auto result = obstacle_detection::ScanProcessor().features(scan);
    const auto & seg = result.segments;
    ASSERT_EQ(seg.size(), 2U);
    EXPECT_EQ(result.points.size(), 14U);
    for (const auto & point : result.points) {
      EXPECT_GT(std::abs(point.y), 0.003);
    }
    EXPECT_LT(std::max(seg[0].first_point.y, seg[0].last_point.y), 0.0);
    EXPECT_GT(std::min(seg[1].first_point.y, seg[1].last_point.y), 0.0);
  }
}

TEST(ScanSegments, EmptySceneAndCloseSmallSurface)
{
  auto scan = wall(0.25F, 7);
  auto small = obstacle_detection::ScanProcessor().features(scan).segments;
  ASSERT_EQ(small.size(), 1U);
  EXPECT_NEAR(small[0].length(), 0.5 * std::tan(0.03), 1e-4);
  std::fill(scan.ranges.begin(), scan.ranges.end(), std::numeric_limits<float>::infinity());
  auto empty = obstacle_detection::ScanProcessor().features(scan);
  EXPECT_TRUE(empty.segments.empty());
  EXPECT_TRUE(empty.points.empty());
}

TEST(ScanSegments, AdjacentBeamsAtDifferentDepthsAreSeparate)
{
  auto scan = wall(0.4F, 16);
  auto far_wall = wall(2.0F, 16);
  std::copy(far_wall.ranges.begin() + 8, far_wall.ranges.end(), scan.ranges.begin() + 8);
  auto seg = obstacle_detection::ScanProcessor().features(scan).segments;
  ASSERT_EQ(seg.size(), 2U);
  EXPECT_NEAR(seg[0].first_point.x, 0.4, 1e-5);
  EXPECT_NEAR(seg[1].first_point.x, 2.0, 1e-5);
}

TEST(ScanSegments, CloudUsesOnlyMeasuredSupportingPoints)
{
  auto scan = wall(0.5F, 15);
  scan.ranges[7] = 0.56F;  // One off-surface beam in the same spatial cluster.
  auto result = obstacle_detection::ScanProcessor().features(scan);
  ASSERT_EQ(result.segments.size(), 1U);
  ASSERT_EQ(result.points.size(), 14U);
  size_t j = 0;
  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    if (i == 7) {continue;}
    const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    EXPECT_NEAR(result.points[j].x, scan.ranges[i] * std::cos(angle), 1e-8);
    EXPECT_NEAR(result.points[j].y, scan.ranges[i] * std::sin(angle), 1e-8);
    ++j;
  }
}

TEST(ScanSegments, ShortIsolatedGroupCreatesNoCloudPoints)
{
  auto scan = wall(0.20F, 3);
  auto result = obstacle_detection::ScanProcessor().features(scan);
  EXPECT_TRUE(result.segments.empty());
  EXPECT_TRUE(result.points.empty());
}
}  // namespace
