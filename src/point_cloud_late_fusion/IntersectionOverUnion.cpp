/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#include "agribot_obstacle_detection/point_cloud_late_fusion/IntersectionOverUnion.h"

double intersectionOverUnion(const agribot_msgs::Object3D& object_1, const agribot_msgs::Object3D& object_2)
{
  return intersectionOverUnion(object_1.bbox, object_2.bbox);
}

double intersectionOverUnion(const agribot_msgs::BoundingBox3D& box_1, const agribot_msgs::BoundingBox3D& box_2)
{
  auto box_1_volume = box_1.size.x * box_1.size.y * box_1.size.z;
  auto box_2_volume = box_2.size.x * box_2.size.y * box_2.size.z;

  auto x_overlap =
      edgeOverlapLength(box_1.center.pose.position.x, box_1.size.x, box_2.center.pose.position.x, box_2.size.x);
  auto y_overlap =
      edgeOverlapLength(box_1.center.pose.position.y, box_1.size.y, box_2.center.pose.position.y, box_2.size.y);
  auto z_overlap =
      edgeOverlapLength(box_1.center.pose.position.z, box_1.size.z, box_2.center.pose.position.z, box_2.size.z);

  auto box_intersection_volume = x_overlap * y_overlap * z_overlap;
  auto box_union_volume = box_1_volume + box_2_volume - box_intersection_volume;

  if (box_union_volume == 0)
  {
    return 0;
  }

  return (box_intersection_volume / box_union_volume);
}

double edgeOverlapLength(double center_1, double size_1, double center_2, double size_2)
{
  auto b1_min = center_1 - size_1 / 2;
  auto b1_max = center_1 + size_1 / 2;
  auto b2_min = center_2 - size_2 / 2;
  auto b2_max = center_2 + size_2 / 2;

  if ((b1_min >= b2_min) && (b1_min <= b2_max))
  {
    auto a = b2_max - b1_min;
    auto b = b1_max - b1_min;
    return (a > b ? b : a);
  }
  else if ((b1_max >= b2_min) && (b1_max <= b2_max))
  {
    auto a = b1_max - b2_min;
    auto b = b1_max - b1_min;
    return (a > b ? b : a);
  }
  else if ((b2_min >= b1_min) && (b2_min <= b1_max))
  {
    auto a = b1_max - b2_min;
    auto b = b2_max - b2_min;
    return (a > b ? b : a);
  }
  else if ((b2_max >= b1_min) && (b2_max <= b1_max))
  {
    auto a = b2_max - b1_min;
    auto b = b2_max - b2_min;
    return (a > b ? b : a);
  }
  else
  {
    // No overlap condition met
    return 0;
  }
}
