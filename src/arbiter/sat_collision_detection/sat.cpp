#include "agribot_obstacle_detection/arbiter/sat_collision_detection/sat.h"

bool SAT::intersect(const agribot_msgs::PolygonStamped& firstPolygon,
                    const agribot_msgs::PolygonStamped& secondPolygon)
{
  // Get projection axes for both polygons
  std::vector<Eigen::Vector2f> firstPolyNormalAxes = getNormalAxes(firstPolygon);
  std::vector<Eigen::Vector2f> secondPolyNormalAxes = getNormalAxes(secondPolygon);

  // Join vectors with normal axes into one
  std::vector<Eigen::Vector2f> normalAxes;
  normalAxes.reserve(firstPolyNormalAxes.size() + secondPolyNormalAxes.size());
  normalAxes.insert(normalAxes.end(), firstPolyNormalAxes.begin(), firstPolyNormalAxes.end());
  normalAxes.insert(normalAxes.end(), secondPolyNormalAxes.begin(), secondPolyNormalAxes.end());

  // Project this polygon on all axes
  std::vector<Projection> firstPolyProjections = getProjections(firstPolygon, normalAxes);

  // Project other polygon on all axes
  std::vector<Projection> secondPolyProjections = getProjections(secondPolygon, normalAxes);

  // Check if all projections overlap
  for (std::size_t i = 0; i < firstPolyProjections.size(); i++)
  {
    if (!firstPolyProjections[i].overlap(secondPolyProjections[i]))
    {
      // If any of the projections is not overlapping, polygons are not intersecting
      return false;
    }
  }

  // If all projections overlap, polygons are intersecting
  return true;
}

Projection SAT::getProjection(const agribot_msgs::PolygonStamped& polygon, const Eigen::Vector2f& axis)
{
  // TODO(olek) - maybe it can be optimized with some lambda / filter / something else
  std::vector<float> pointProjections;
  for (const auto& p : polygon.polygon.points)
  {
    Eigen::Vector2f xyPoint = toEigenVector2f(p);
    float projection = axis.dot(xyPoint);
    pointProjections.push_back(projection);
  }

  float minPP = *min_element(pointProjections.begin(), pointProjections.end());
  float maxPP = *max_element(pointProjections.begin(), pointProjections.end());

  return Projection(minPP, maxPP);
}

std::vector<Projection> SAT::getProjections(const agribot_msgs::PolygonStamped& polygon,
                                            const std::vector<Eigen::Vector2f>& axes)
{
  std::vector<Projection> projections;

  for (const auto& axis : axes)
  {
    projections.push_back(getProjection(polygon, axis));
  }

  return projections;
}

std::vector<Eigen::Vector2f> SAT::getNormalAxes(const agribot_msgs::PolygonStamped& polygon)
{
  std::vector<Eigen::Vector2f> axes;

  /* General case - axes from consecutive points */
  for (std::size_t i = 0; i < polygon.polygon.points.size() - 1; i++)
  {
    // Get edge between next and current point
    Eigen::Vector2f currentPointVector = toEigenVector2f(polygon.polygon.points[i]);
    Eigen::Vector2f nextPointVector = toEigenVector2f(polygon.polygon.points[i + 1]);

    Eigen::Vector2f edge = nextPointVector - currentPointVector;

    // Get axis vector normal to the edge
    Eigen::Vector2f perpendicularAxis = Eigen::Vector2f(edge[1], -edge[0]);

    // Push back normalized vector
    axes.push_back(perpendicularAxis.normalized());
  }

  /* Special case - edge between first and last point */
  // Get edge between last and first point
  Eigen::Vector2f backPointVector = toEigenVector2f(polygon.polygon.points.back());
  Eigen::Vector2f frontPointVector = toEigenVector2f(polygon.polygon.points.front());
  Eigen::Vector2f edge = backPointVector - frontPointVector;

  // Get axis vector normal to the edge
  Eigen::Vector2f perpendicularAxis = Eigen::Vector2f(edge[1], -edge[0]);

  // Push back normalized vector
  axes.push_back(perpendicularAxis.normalized());

  return axes;
}

Eigen::Vector2f SAT::toEigenVector2f(const geometry_msgs::Point& point)
{
  return Eigen::Vector2f(point.x, point.y);
}

geometry_msgs::Point SAT::toRosPoint(const Eigen::Vector2f& pointVector)
{
  geometry_msgs::Point point;
  point.x = pointVector[0];
  point.y = pointVector[1];

  return point;
}
