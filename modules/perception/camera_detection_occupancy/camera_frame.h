/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#pragma once

#include <memory>
#include <vector>

#include "modules/perception/common/base/hdmap_struct.h"
#include "modules/perception/common/base/lane_struct.h"
#include "modules/perception/common/base/object_pool_types.h"
#include "modules/perception/common/base/traffic_light.h"
#include "modules/perception/common/camera/common/data_provider.h"

namespace apollo {
namespace perception {
namespace camera {

enum class OccType {
    CAR = 0,
    TRUCK = 1,
    TRAILER = 2,
    BUS = 3,
    CONSTRUCTION_VEHICLE = 4,
    BICYCLE = 5,
    MOTORCYCLE = 6,
    PEDESTRIAN = 7,
    TRAFFIC_CONE = 8,
    BARRIER = 9,
    DRIVEABLE_SURFACE = 10,
    OTHER_FLAT = 11,
    SIDEWALK = 12,
    TERRAIN = 13,
    MANMADE = 14,
    VEGETATION = 15,
    UNKNOWN = 16
};

const std::map<int, OccType> kIndex2OccName = {
    {0, OccType::CAR},          {1, OccType::TRUCK},   {2, OccType::TRAILER},
    {3, OccType::BUS},          {4, OccType::CONSTRUCTION_VEHICLE}, {5, OccType::BICYCLE},
    {6, OccType::MOTORCYCLE},   {7, OccType::PEDESTRIAN}, {8, OccType::TRAFFIC_CONE},
    {9, OccType::BARRIER},   {10, OccType::DRIVEABLE_SURFACE}, {11, OccType::OTHER_FLAT},
    {12, OccType::SIDEWALK},   {13, OccType::TERRAIN}, {14, OccType::MANMADE},
    {15, OccType::VEGETATION},   {16, OccType::UNKNOWN}, 
};

struct OccData {
  // OccData();
  void Reset();
  int occ_id = -1;
  OccType occ_type = OccType::UNKNOWN;
  float occ_confidence = 1.0f;
  float occ_timestamp = 0.0f;
  float occ_size = 0.0f;
  float occ_x = 0.0f;
  float occ_y = 0.0f;
  float occ_z = 0.0f;
  float occ_theta = 0.0f;
};
using OccDataPtr = std::shared_ptr<OccData>;

struct CameraFrame {
  // frame sequence id
  std::uint64_t frame_id;
  // timestamp
  double timestamp;
  std::vector<std::shared_ptr<DataProvider>> data_provider;
  std::vector<base::ObjectPtr> detected_objects;
  std::vector<OccDataPtr> occ_status;
  // lidar2img matrix
  std::vector<float> k_lidar2img;
  // occ pseudo pointcloud
  std::shared_ptr<base::AttributePointCloud<base::PointF>> cloud;

};

}  // namespace camera
}  // namespace perception
}  // namespace apollo
