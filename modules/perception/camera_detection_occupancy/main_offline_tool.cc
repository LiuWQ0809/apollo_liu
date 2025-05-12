/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include <memory>
#include <string>
#include <vector>
#include <unistd.h>


#include "gflags/gflags.h"
#include "cyber/cyber.h"
#include "cyber/common/file.h"
#include "opencv2/opencv.hpp"
#include "modules/perception/common/algorithm/io/io_util.h"
#include "modules/perception/common/algorithm/sensor_manager/sensor_manager.h"
#include "modules/perception/common/camera/common/data_provider.h"
// #include "modules/perception/common/interface/base_obstacle_detector.h"
#include "modules/perception/camera_detection_occupancy/camera_detection_occupancy_component.h"

// #include "modules/perception/tools/common/ground_truth.h"
// #include "modules/perception/tools/common/util.h"
// #include "modules/perception/tools/common/visualizer.h"

DEFINE_int32(height, 900, "image height");
DEFINE_int32(width, 1600, "image width");
DEFINE_int32(gpu_id, 0, "gpu id");
DEFINE_string(dest_dir, "./data/output", "output dir");
DEFINE_string(dist_type, "", "dist pred type: H-on-h, H-from-h");
DEFINE_string(kitti_dir, "", "pre-detected obstacles (skip Detect)");
DEFINE_string(root_dir,
              "modules/perception/tools/offline_camera_detection/",
              "image root dir");
DEFINE_string(image_ext, ".png", "extension of image name");
DEFINE_string(config_path, "perception/camera_detection_occupancy/data",
              "config path");
DEFINE_string(config_file, "occ_det_nus.pb.txt", "config file");
DEFINE_string(camera_name, "CAM_FRONT", "camera name");
DEFINE_string(detector_name, "BEVFORMERObstacleDetector", "detector name");
// sensor_manager
DEFINE_string(off_obs_sensor_intrinsic_path,
              "/apollo/modules/perception/data/params/",
              "The intrinsics/extrinsics dir.");

namespace apollo {
namespace perception {
namespace camera {

std::shared_ptr<CameraFrame> frame_ptr_;

bool OfflineLoadCameraExtrinsicNus(
    const std::string &camera_extrinsic_file_path,
    const std::string &lidar_extrinsic_file_path,
    Eigen::Affine3d *camera2lidar_rt) {
  Eigen::Affine3d local2lidar_rt = Eigen::Affine3d::Identity();
  YAML::Node config = YAML::LoadFile(lidar_extrinsic_file_path);
  if (config["transform"]) {
    if (config["transform"]["translation"]) {
      local2lidar_rt.translation()(0) =
          config["transform"]["translation"]["x"].as<double>();
      local2lidar_rt.translation()(1) =
          config["transform"]["translation"]["y"].as<double>();
      local2lidar_rt.translation()(2) =
          config["transform"]["translation"]["z"].as<double>();
      if (config["transform"]["rotation"]) {
        double qx = config["transform"]["rotation"]["x"].as<double>();
        double qy = config["transform"]["rotation"]["y"].as<double>();
        double qz = config["transform"]["rotation"]["z"].as<double>();
        double qw = config["transform"]["rotation"]["w"].as<double>();
        local2lidar_rt.linear() =
            Eigen::Quaterniond(qw, qx, qy, qz).toRotationMatrix();
      }
    }
  }

  Eigen::Affine3d local2cam_rt = Eigen::Affine3d::Identity();
  config = YAML::LoadFile(camera_extrinsic_file_path);
  if (config["transform"]) {
    if (config["transform"]["translation"]) {
      local2cam_rt.translation()(0) =
          config["transform"]["translation"]["x"].as<double>();
      local2cam_rt.translation()(1) =
          config["transform"]["translation"]["y"].as<double>();
      local2cam_rt.translation()(2) =
          config["transform"]["translation"]["z"].as<double>();
      if (config["transform"]["rotation"]) {
        double qx = config["transform"]["rotation"]["x"].as<double>();
        double qy = config["transform"]["rotation"]["y"].as<double>();
        double qz = config["transform"]["rotation"]["z"].as<double>();
        double qw = config["transform"]["rotation"]["w"].as<double>();
        local2cam_rt.linear() =
            Eigen::Quaterniond(qw, qx, qy, qz).toRotationMatrix();
      }
    }
  }
  *camera2lidar_rt = (local2cam_rt.inverse() * local2lidar_rt).inverse();
  return false;
}

bool InitDataProvider() {
  DataProvider::InitOptions init_options;
  init_options.sensor_name = FLAGS_camera_name;
  init_options.image_height = FLAGS_height;
  init_options.image_width = FLAGS_width;
  init_options.do_undistortion = false; // TODO
  init_options.device_id = FLAGS_gpu_id;

  std::vector<std::string> bev_sensor_name;
  bev_sensor_name.push_back("CAM_FRONT");
  bev_sensor_name.push_back("CAM_FRONT_RIGHT");
  bev_sensor_name.push_back("CAM_FRONT_LEFT");
  bev_sensor_name.push_back("CAM_BACK");
  bev_sensor_name.push_back("CAM_BACK_LEFT");
  bev_sensor_name.push_back("CAM_BACK_RIGHT");

  frame_ptr_ = std::make_shared<CameraFrame>();
  frame_ptr_->data_provider.resize(bev_sensor_name.size());
  frame_ptr_->k_lidar2img.clear();
  FLAGS_obs_sensor_intrinsic_path = FLAGS_off_obs_sensor_intrinsic_path;
  for (int i = 0; i < bev_sensor_name.size(); ++i) {
    init_options.sensor_name = bev_sensor_name[i];
    // get extrinsic
    // std::string camera_extrinsic_file_path =
    //     algorithm::SensorManager::Instance()->ExtrinsicPath(
    //         init_options.sensor_name);
    std::string camera_extrinsic_file_path = FLAGS_off_obs_sensor_intrinsic_path +
                                             "/" + init_options.sensor_name +
                                             "_extrinsics.yaml";
    Eigen::Affine3d camera2lidar_rt = Eigen::Affine3d::Identity();

    std::string lidar_extrinsic_file_path =
      FLAGS_off_obs_sensor_intrinsic_path + "/LIDAR_TOP_novatel_extrinsics.yaml";
    OfflineLoadCameraExtrinsicNus(
      camera_extrinsic_file_path,
      lidar_extrinsic_file_path,
      &camera2lidar_rt);
    Eigen::Matrix4d lidar2camera_rt =
        camera2lidar_rt.inverse().matrix();

    // get intrinsic
    algorithm::SensorManager *sensor_manager =
        algorithm::SensorManager::Instance();
    if (!sensor_manager->IsSensorExist(init_options.sensor_name)) {
      AERROR << "Sensor '" << init_options.sensor_name << "' not exists!";
      return false;
    }

    Eigen::Matrix3f camera_intrinsic =
        std::dynamic_pointer_cast<base::BrownCameraDistortionModel>(
            sensor_manager->GetDistortCameraModel(init_options.sensor_name))
            ->get_intrinsic_params();
    Eigen::Matrix4f viewpad = Eigen::Matrix4f::Identity();
    viewpad(0, 0) = camera_intrinsic(0, 0);
    viewpad(0, 2) = camera_intrinsic(0, 2);
    viewpad(1, 1) = camera_intrinsic(1, 1);
    viewpad(1, 2) = camera_intrinsic(1, 2);

    // lidar2img_rt = (viewpad @ lidar2cam_rt.T)
    Eigen::Matrix4d lidar2img_rt =
        viewpad.cast<double>() * lidar2camera_rt;

    for (size_t i = 0; i < lidar2img_rt.rows(); i++) {
      for (size_t j = 0; j < lidar2img_rt.cols(); j++) {
        frame_ptr_->k_lidar2img.push_back(lidar2img_rt(i, j));
      }
    }

    frame_ptr_->data_provider[i] = std::make_shared<DataProvider>();
    frame_ptr_->data_provider[i]->Init(init_options);
  }

  // bool res = camera_frame->data_provider->Init(init_options);
  return true;
}

bool FillImage() {
  // Read image from file_name
  std::string file_name1 = "/apollo_workspace/data/camera_data/test1/0.png";
  std::string file_name2 = "/apollo_workspace/data/camera_data/test1/1.png";
  std::string file_name3 = "/apollo_workspace/data/camera_data/test1/2.png";
  std::string file_name4 = "/apollo_workspace/data/camera_data/test1/3.png";
  std::string file_name5 = "/apollo_workspace/data/camera_data/test1/4.png";
  std::string file_name6 = "/apollo_workspace/data/camera_data/test1/5.png";
  cv::Mat image0 = cv::imread(file_name1);
  cv::resize(image0, image0, cv::Size(1600, 900), 0, 0);
  cv::Mat image1 = cv::imread(file_name2);
  cv::resize(image1, image1, cv::Size(1600, 900), 0, 0);
  cv::Mat image2 = cv::imread(file_name3);
  cv::resize(image2, image2, cv::Size(1600, 900), 0, 0);
  cv::Mat image3 = cv::imread(file_name4);
  cv::resize(image3, image3, cv::Size(1600, 900), 0, 0);
  cv::Mat image4 = cv::imread(file_name5);
  cv::resize(image4, image4, cv::Size(1600, 900), 0, 0);
  cv::Mat image5 = cv::imread(file_name6);
  cv::resize(image5, image5, cv::Size(1600, 900), 0, 0);
  if (image0.empty()) {
    AERROR << "Read image failed! " << file_name1;
    return false;
  }

  // Fill image to frame->data_provider
  frame_ptr_->data_provider[0]->FillImageData(image0.rows, image0.cols,
                                                 image0.data, "bgr8");
  frame_ptr_->data_provider[1]->FillImageData(image1.rows, image1.cols,
                                                 image1.data, "bgr8");
  frame_ptr_->data_provider[2]->FillImageData(image2.rows, image2.cols,
                                                 image2.data, "bgr8");
  frame_ptr_->data_provider[3]->FillImageData(image3.rows, image3.cols,
                                                 image3.data, "bgr8");
  frame_ptr_->data_provider[4]->FillImageData(image4.rows, image4.cols,
                                                 image4.data, "bgr8");
  frame_ptr_->data_provider[5]->FillImageData(image5.rows, image5.cols,
                                                 image5.data, "bgr8");
  return true;
}

bool TestDetection() {

  // Init data
  ACHECK(InitDataProvider());

  ObstacleDetectorInitOptions init_options;
  // Init conf file
  init_options.config_path = FLAGS_config_path;
  init_options.config_file = FLAGS_config_file;
  init_options.gpu_id = FLAGS_gpu_id;

  // Init camera params
  std::string camera_name = FLAGS_camera_name;
  base::BaseCameraModelPtr model =
      algorithm::SensorManager::Instance()->GetUndistortCameraModel(
          camera_name);
  ACHECK(model) << "Can't find " << camera_name
                << " in data/conf/sensor_meta.pb.txt";
  auto pinhole = static_cast<base::PinholeCameraModel*>(model.get());
  init_options.intrinsic = pinhole->get_intrinsic_params();
  init_options.image_height = model->get_height();
  init_options.image_width = model->get_width();
  // Init detection pipeline
  std::shared_ptr<BaseObstacleDetector> detector;
  detector.reset(
      BaseObstacleDetectorRegisterer::GetInstanceByName(FLAGS_detector_name));
  detector->Init(init_options);

  ACHECK(FillImage());
  ACHECK(detector->Detect(frame_ptr_.get()));
  PERF_BLOCK_END

  return true;
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo

int main(int argc, char* argv[]) {
  chdir("/apollo"); 
  apollo::cyber::Init(argv[0]);
  FLAGS_alsologtostderr = 1;
  google::ParseCommandLineFlags(&argc, &argv, true);
  // google::InitGoogleLogging(argv[0]);
  google::SetUsageMessage(
      "command line brew\n"
      "Usage: camera_benchmark <args>\n");
  apollo::perception::camera::TestDetection();
  return 0;
}
