#include "agribot_obstacle_detection/arbiter/sat_collision_detection/Polygon.h"

Polygon::Polygon()
{
}

Polygon::~Polygon()
{
}

void Polygon::addPoint(const Eigen::Vector2f& point)
{
    points.push_back(point);
}

void Polygon::addPoint(float x, float y)
{
    points.push_back(Eigen::Vector2f(x, y));
}

std::vector<Eigen::Vector2f> Polygon::getNormalAxes() const
{
    std::vector<Eigen::Vector2f> axes;

    /* General case - axes from consecutive points */
    for (std::size_t i = 0; i < points.size() - 1; i++)
    {
        // Get edge between next and current point
        Eigen::Vector2f edge =  points[i+1] - points[i];

        // Get axis vector normal to the edge
        Eigen::Vector2f perpendicularAxis = Eigen::Vector2f(edge[1], -edge[0]);

        // Push back normalized vector
        axes.push_back(perpendicularAxis.normalized());
    }

    /* Special case - edge between first and last point */
    // Get edge between last and first point
    Eigen::Vector2f edge = points.back() - points.front();

    // Get axis vector normal to the edge
    Eigen::Vector2f perpendicularAxis = Eigen::Vector2f(edge[1], -edge[0]);

    // Push back normalized vector
    axes.push_back(perpendicularAxis.normalized());

    return axes;
}

std::vector<Projection> Polygon::getProjections(const std::vector<Eigen::Vector2f>& axes) const
{
    std::vector<Projection> projections;

    for (const auto& axis : axes)
    {
        projections.push_back(getProjection(axis));
    }

    return projections;
}

Projection Polygon::getProjection(const Eigen::Vector2f& axis) const
{
    // TODO(olek) - maybe it can be optimized with some lamba / filter / something else
    std::vector<float> pointProjections;
    for (const auto& p : points)
    {
        float projection = axis.dot(p);
        pointProjections.push_back(projection);
    }

    float minPP = *min_element(pointProjections.begin(), pointProjections.end());
    float maxPP = *max_element(pointProjections.begin(), pointProjections.end());

    return Projection(minPP, maxPP);
}

bool Polygon::intersect(const Polygon& other) const
{
    // Get projection axes
    std::vector<Eigen::Vector2f> thisPolyNormalAxes = this->getNormalAxes();
    std::vector<Eigen::Vector2f> otherPolyNormalAxes = other.getNormalAxes();

    std::vector<Eigen::Vector2f> normalAxes;
    normalAxes.reserve(thisPolyNormalAxes.size() + otherPolyNormalAxes.size());
    normalAxes.insert(normalAxes.end(), thisPolyNormalAxes.begin(), thisPolyNormalAxes.end());
    normalAxes.insert(normalAxes.end(), otherPolyNormalAxes.begin(), otherPolyNormalAxes.end());

    // Project this polygon on all axes
    std::vector<Projection> thisPolyProjections = this->getProjections(normalAxes);

    // Project other polygon on all axes
    std::vector<Projection> otherPolyProjections = other.getProjections(normalAxes);

    // Check if all projections overlap
    for (std::size_t i = 0; i < thisPolyProjections.size(); i++)
    {
        if (!thisPolyProjections[i].overlap(otherPolyProjections[i]))
        {
            // If any of the projections is not overlapping, polygons are not intersecting
            return false;
        }
    }

    // If all projections overlap, polygons are intersecting
    return true;
}
