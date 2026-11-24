#include <gtest/gtest.h>
#include "obstacle_detection/cloud_filter.hpp"

using namespace obstacle_detection::cloud;

TEST(CloudFilter, SeparatesFloorFromThreeCentimeterStepAtGrazingIncidence)
{
  // This cell samples z/x = -5/128. At 4.61 m its nominal ray places a
  // real floor return 19 mm above the ground, outside the 12 mm floor band.
  const double pitch = -2. * std::acos(-1.) / 180.;
  const Point nominal{std::cos(pitch), 0., std::sin(pitch)};
  const Point sampled = ogre2_floor_ray(nominal);
  EXPECT_NEAR(sampled.z/sampled.x, -5./128., 1e-12);
  EXPECT_NEAR(std::hypot(sampled.x, sampled.y, sampled.z), 1., 1e-12);
  const GroundPlane floor{0., 0., 0.};
  const double floor_range = -.18 / sampled.z;
  EXPECT_FALSE(floor.contains({floor_range*nominal.x, 0., .18+floor_range*nominal.z}));
  for (double height : {0., 0.03}) {
    const double measured_range = (.18-height) / -sampled.z;
    EXPECT_EQ(floor.contains({measured_range*sampled.x, measured_range*sampled.y,
                             .18+measured_range*sampled.z}), height == 0.);
  }
  EXPECT_DOUBLE_EQ(ogre2_floor_ray({1.,0.,0.}).z, 0.);
}

TEST(CloudFilter, MasksOnlyMeasuredTopSurfaceInsideChassis)
{
  EXPECT_TRUE(on_chassis_top({-0.1428, -0.1098, 0.1489}));
  EXPECT_TRUE(on_chassis_top({0.1425, 0.1100, 0.1510}));
  EXPECT_TRUE(on_chassis_top({0.15, -0.11, 0.15}));
  EXPECT_FALSE(on_chassis_top({0.15001, 0.0, 0.15}));
  EXPECT_FALSE(on_chassis_top({0.0, -0.11001, 0.15}));
  EXPECT_FALSE(on_chassis_top({0.14, 0.10, 0.15201}));
  EXPECT_FALSE(on_chassis_top({0.14, 0.10, 0.14799}));
  EXPECT_FALSE(on_chassis_top({0.14, 0.10, 0.20}));
}

TEST(CloudFilter, SignedMissesFollowTheBeamDirection)
{
  const auto inf = std::numeric_limits<double>::infinity();
  const auto nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(inf, range_of({-inf, nan, nan}, {-1, 0, 0}));
  EXPECT_EQ(-inf, range_of({inf, nan, nan}, {-1, 0, 0}));
  EXPECT_EQ(inf, range_of({nan, -inf, nan}, {0, -1, 0}));
  EXPECT_TRUE(std::isnan(range_of({nan, nan, nan}, {1, 0, 0})));
  EXPECT_TRUE(std::isnan(range_of({inf, 0, 0}, {1, 0, 0})));
  EXPECT_DOUBLE_EQ(5.0, range_of({3, 0, 4}, {1, 0, 0}));
}

TEST(CloudFilter, FitsDistributedFloorAndPreservesLowObstacles)
{
  std::vector<Point> candidates;
  for (int i = -6; i <= 6; ++i) {
    for (int j = -6; j <= 6; ++j) {
      candidates.push_back({0.1*i, 0.1*j, 0.001*i});
    }
  }
  candidates.push_back({0.3, 0.5, 0.03});
  const auto plane = fit_ground(candidates);
  ASSERT_TRUE(plane);
  EXPECT_TRUE(plane->contains({0.2, 0.2, 0.002}));
  EXPECT_FALSE(plane->contains({0.3, 0.5, 0.03}));
}

TEST(CloudFilter, NarrowClusterCannotEstablishTheFloor)
{
  std::vector<Point> patch;
  for (int i = 1; i <= 10; ++i) {
    for (int j = 1; j <= 10; ++j) patch.push_back({0.2*i, 0.2*j, 0.01});
  }
  EXPECT_FALSE(fit_ground(patch));
  EXPECT_FALSE(fit_ground({}));
}

TEST(CloudFilter, VoxelKeysDistinguishNegativeCoordinatesAndHeight)
{
  EXPECT_EQ(voxel_key({0.011, 0.001, 0.03}), voxel_key({0.014, 0.018, 0.039}));
  EXPECT_NE(voxel_key({-0.001, 0, 0}), voxel_key({0.001, 0, 0}));
  EXPECT_NE(voxel_key({0, 0, 0.001}), voxel_key({0, 0, 0.025}));
}
