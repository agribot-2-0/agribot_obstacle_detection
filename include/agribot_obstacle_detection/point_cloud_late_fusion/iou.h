/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#pragma once

#include <agribot_msgs/Object3D.h>

double iou(const agribot_msgs::Object3D& object_1, const agribot_msgs::Object3D& object_2);
double iou(const agribot_msgs::BoundingBox3D& box_1, const agribot_msgs::BoundingBox3D& box_2);
double edgeOverlapLength(double center_1, double size_1, double center_2, double size_2);
