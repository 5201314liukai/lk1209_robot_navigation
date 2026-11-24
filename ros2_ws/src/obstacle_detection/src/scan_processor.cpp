#include "obstacle_detection/scan_processor.hpp"

#include <algorithm>
#include <cmath>

#include "obstacle_detection/obstacle_extractor.hpp"
#include "obstacle_detection/obstacle_grouper.hpp"

namespace obstacle_detection
{
namespace
{
// Each run contains adjacent reported beams. A missing return never connects
// two surfaces, even when their endpoints are closer than the merge threshold.
std::vector<PointSet> continuousRuns(const sensor_msgs::msg::LaserScan & scan)
{
  std::vector<PointSet> runs;
  bool start_run = true;
  if (!std::isfinite(scan.angle_min) || !std::isfinite(scan.angle_increment)) {
    return runs;
  }
  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    const double r = scan.ranges[i];
    if (!(std::isfinite(r) && r >= scan.range_min && r <= scan.range_max)) {
      start_run = true;
      continue;
    }
    if (start_run) {
      runs.emplace_back();
      start_run = false;
    }
    const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    runs.back().addPoint(Point(r * std::cos(angle), r * std::sin(angle)));
  }
  return runs;
}
}  // namespace

PointSet ScanProcessor::process(const sensor_msgs::msg::LaserScan & scan)
{
  PointSet combined;
  for (const auto & run : continuousRuns(scan)) {
    combined.points.insert(combined.points.end(), run.points.begin(), run.points.end());
  }
  return combined;
}

ScanFeatures ScanProcessor::features(const sensor_msgs::msg::LaserScan & scan)
{
  ObstacleGrouper grouper;
  ObstacleExtractor extractor;
  ScanFeatures result;
  for (const auto & run : continuousRuns(scan)) {
    for (const auto & group : grouper.groupPoints(run.points)) {
      const auto fitted = extractor.extract(group);
      result.segments.insert(result.segments.end(), fitted.begin(), fitted.end());
      for (const auto & point : group.points) {
        // Half of one 5 cm navigation cell; distanceTo clamps to the finite segment.
        // Mark original hits once, even where two fitted surfaces meet.
        if (std::any_of(fitted.begin(), fitted.end(), [&point](const Segment & segment) {
            return segment.distanceTo(point) <= 0.025;
          }))
        {
          result.points.push_back(point);
        }
      }
    }
  }
  return result;
}
}  // namespace obstacle_detection
