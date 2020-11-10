/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>
#include <ctime>
#include <sstream>
#include <memory>

#include <opencv2/core/core.hpp>


#include<System.h>
#include "ImuTypes.h"
#include "Optimizer.h"

/*
 * ILLIXR related headers
 */
#include <eigen3/Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include "plugin.hpp"
#include "switchboard.hpp"
#include "data_format.hpp"
#include "phonebook.hpp"

using namespace std;
using ILLIXR::imu_cam_type;
using ILLIXR::imu_integrator_input;
using ILLIXR::phonebook;
using ILLIXR::switchboard;
using ILLIXR::plugin;
using ILLIXR::pose_type;
using ILLIXR::time_type;
using ILLIXR::writer;

std::string get_path() {
    const char* ORB_SLAM3_ROOT_c_str = std::getenv("ORB_SLAM3_ROOT");
	if (!ORB_SLAM3_ROOT_c_str) {
		std::cerr << "Please define ORB_SLAM3_ROOT" << std::endl;
		abort();
	}
	std::string ORB_SLAM3_ROOT = std::string{ORB_SLAM3_ROOT_c_str};
    return ORB_SLAM3_ROOT;
}

class ORB_SLAM3_ILLIXR : public plugin {
  private:
   const std::shared_ptr<switchboard> sb;
   std::unique_ptr<writer<pose_type>> _m_pose;
   std::unique_ptr<writer<imu_integrator_input>> _m_imu_integrator_input;
   time_type _m_begin;
   const imu_cam_type *imu_cam_buffer;
   double previous_timestamp = 0.0;
   vector<ORB_SLAM3::IMU::Point> vImuMeas;
   std::unique_ptr<ORB_SLAM3::System> SLAM_sys;

   cv::Mat M1l, M2l, M1r, M2r;

  public:
   ORB_SLAM3_ILLIXR(std::string name_, phonebook *pb_)
       : plugin{name_, pb_},
         sb{pb->lookup_impl<switchboard>()},
         _m_pose{sb->publish<pose_type>("slow_pose")},
         _m_imu_integrator_input{
             sb->publish<imu_integrator_input>("imu_integrator_input")} {
      _m_begin = std::chrono::system_clock::now();
      imu_cam_buffer = NULL;
      _m_pose->put(new pose_type{
          .sensor_time = std::chrono::time_point<std::chrono::system_clock>(),
          .position = Eigen::Vector3f{0, 0, 0},
          .orientation = Eigen::Quaternionf{1, 0, 0, 0}});
#ifdef CV_HAS_METRICS
      cv::metrics::setAccount(new std::string{"-1"});
#endif
      std::string root = get_path();
      std::string voc_path = root + "Vocabulary/ORBvoc.txt";
      std::string settings_path = root + "Examples/Stereo-Intertial/EuRoC.yaml";
      SLAM_sys = std::make_unique<ORB_SLAM3::System>(
          voc_path, settings_path, ORB_SLAM3::System::IMU_STEREO, true);

      // Read rectification parameters
      cv::FileStorage fsSettings(settings_path, cv::FileStorage::READ);
      if (!fsSettings.isOpened()) {
         cerr << "ERROR: Wrong path to settings" << endl;
         abort();
      }

      cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
      fsSettings["LEFT.K"] >> K_l;
      fsSettings["RIGHT.K"] >> K_r;

      fsSettings["LEFT.P"] >> P_l;
      fsSettings["RIGHT.P"] >> P_r;

      fsSettings["LEFT.R"] >> R_l;
      fsSettings["RIGHT.R"] >> R_r;

      fsSettings["LEFT.D"] >> D_l;
      fsSettings["RIGHT.D"] >> D_r;

      int rows_l = fsSettings["LEFT.height"];
      int cols_l = fsSettings["LEFT.width"];
      int rows_r = fsSettings["RIGHT.height"];
      int cols_r = fsSettings["RIGHT.width"];

      if (K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() ||
          R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
          rows_l == 0 || rows_r == 0 || cols_l == 0 || cols_r == 0) {
         cerr << "ERROR: Calibration parameters to rectify stereo are missing!"
              << endl;
         abort();
      }

      cv::Mat M1l, M2l, M1r, M2r;
      cv::initUndistortRectifyMap(K_l, D_l, R_l,
                                  P_l.rowRange(0, 3).colRange(0, 3),
                                  cv::Size(cols_l, rows_l), CV_32F, M1l, M2l);
      cv::initUndistortRectifyMap(K_r, D_r, R_r,
                                  P_r.rowRange(0, 3).colRange(0, 3),
                                  cv::Size(cols_r, rows_r), CV_32F, M1r, M2r);
   }
   virtual ~ORB_SLAM3_ILLIXR() override {};

   virtual void start() override {
      plugin::start();
      sb->schedule<imu_cam_type>(
          id, "imu_cam",
          [&](const imu_cam_type *dataum) { this->feed_imu_cam(dataum); });
   }

   std::size_t iteration_no = 0;
   void feed_imu_cam(const imu_cam_type *dataum) {
      // Ensures that slam doesnt start before valid IMU readings come in
      if (dataum == NULL) {
         assert(previous_timestamp == 0);
         return;
      }

      // This ensures that every data point is coming in chronological order If
      // youre failing this assert, make sure that your data folder matches the
      // name in offline_imu_cam/plugin.cc
      assert(dataum->dataset_time > previous_timestamp);
      previous_timestamp = dataum->dataset_time;

      imu_cam_buffer = dataum;

      const Eigen::Vector3f &acc = dataum->linear_a;
      const Eigen::Vector3f &gyro = dataum->angular_v;
      ORB_SLAM3::IMU::Point ORB_p(acc(0), acc(1), acc(2), gyro(0), gyro(1),
                                  gyro(2), previous_timestamp);

      // Feed the IMU measurement. There should always be IMU data in each
      // call to feed_imu_cam
      assert((dataum->img0.has_value() && dataum->img1.has_value()) ||
             (!dataum->img0.has_value() && !dataum->img1.has_value()));
      vImuMeas.push_back(ORB_p);

#ifdef CV_HAS_METRICS
      cv::metrics::setAccount(new std::string{std::to_string(iteration_no)});
      iteration_no++;
      if (iteration_no % 20 == 0) {
         cv::metrics::dump();
      }
#else
#warning \
    "No OpenCV metrics available. Please recompile OpenCV from git clone --branch 3.4.6-instrumented https://github.com/ILLIXR/opencv/. (see install_deps.sh)"
#endif

      std::optional<cv::Mat *> imLeft = imu_cam_buffer->img0;
      std::optional<cv::Mat *> imRight = imu_cam_buffer->img1;
      cv::Mat imLeftRect, imRightRect;

#ifdef COMPILEDWITHC11
      std::chrono::steady_clock::time_point t_Start_Rect =
          std::chrono::steady_clock::now();
#else
      std::chrono::monotonic_clock::time_point t_Start_Rect =
          std::chrono::monotonic_clock::now();
#endif
      cv::remap(**imLeft, imLeftRect, M1l, M2l, cv::INTER_LINEAR);
      cv::remap(**imRight, imRightRect, M1r, M2r, cv::INTER_LINEAR);

#ifdef COMPILEDWITHC11
      std::chrono::steady_clock::time_point t_End_Rect =
          std::chrono::steady_clock::now();
#else
      std::chrono::monotonic_clock::time_point t_End_Rect =
          std::chrono::monotonic_clock::now();
#endif
      double t_rect = std::chrono::duration_cast<std::chrono::duration<double>>(
                          t_End_Rect - t_Start_Rect)
                          .count();

      double tframe = dataum->dataset_time;

      // If there is not cam data this func call, break early
      if (!dataum->img0.has_value() && !dataum->img1.has_value()) {
         return;
      } else {
#ifdef COMPILEDWITHC11
         std::chrono::steady_clock::time_point t1 =
             std::chrono::steady_clock::now();
#else
         std::chrono::monotonic_clock::time_point t1 =
             std::chrono::monotonic_clock::now();
#endif

         // Pass the images to the SLAM system
         cv::Mat Tcw =
             SLAM_sys->TrackStereo(imLeftRect, imRightRect, tframe, vImuMeas);

#ifdef COMPILEDWITHC11
         std::chrono::steady_clock::time_point t2 =
             std::chrono::steady_clock::now();
#else
         std::chrono::monotonic_clock::time_point t2 =
             std::chrono::monotonic_clock::now();
#endif
         vImuMeas.clear();
         cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
         cv::Mat twc = -Rwc * Tcw.rowRange(0, 3).col(3);
         Eigen::Vector3f position{twc.at<float>(0), twc.at<float>(1),
                                  twc.at<float>(2)};
         Eigen::Matrix3f emat3f;
         cv2eigen(Rwc, emat3f);
         Eigen::Quaternionf orientation(emat3f);
         _m_pose->put(new pose_type{.sensor_time = imu_cam_buffer->time,
                                    .position = position,
                                    .orientation = orientation});
      }
   }

};

PLUGIN_MAIN(ORB_SLAM3_ILLIXR);
