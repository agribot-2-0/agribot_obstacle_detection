#pragma once

class Projection
{
 private:
    float min;
    float max;

 public:
    Projection(float minValue, float maxValue);
    ~Projection();
    bool overlap(const Projection& other) const;
};
