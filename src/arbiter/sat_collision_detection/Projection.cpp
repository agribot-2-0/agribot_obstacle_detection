#include "agribot_obstacle_detection/arbiter/sat_collision_detection/Projection.h"

Projection::Projection(float minValue, float maxValue)
{
    min = minValue;
    max = maxValue;
}

Projection::~Projection()
{
}

bool Projection::overlap(const Projection& other) const
{
    if ((this->min >= other.min) && (this->min <= other.max))
    {
        return true;
    }
    else if ((this->max >= other.min) && (this->max <= other.max))
    {
        return true;
    }
    else if ((other.min >= this->min) && (other.min <= this->max))
    {
        return true;
    }
    else if ((other.max >= this->min) && (other.max <= this->max))
    {
        return true;
    }
    else
    {
        // No overlap condition met
        return false;
    }
}
