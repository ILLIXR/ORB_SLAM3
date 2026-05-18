#pragma once

#include "System.h"

#include "illixr/data_format/imu.hpp"
#include "illixr/data_format/opencv_data_types.hpp"
#include "illixr/data_format/poses/head_pose.hpp"
#include "illixr/phonebook.hpp"
#include "illixr/plugin.hpp"
#include "illixr/relative_clock.hpp"
#include "illixr/switchboard.hpp"

#include <boost/filesystem.hpp>

namespace ILLIXR {

class orb_slam3 : public plugin {
public:
    orb_slam3(const std::string& name_, phonebook* pb_);

    void start() override;

    void feed_imu_cam(const switchboard::ptr<const data_format::imu_type>& datum, std::size_t iteration_no);

    /*
    #if !defined(STEREO_IMU) && !defined(ZED)
        void feed_rgbd(switchboard::ptr<const data_format::rgb_depth_type> datum,
                       std::size_t iteration_no);
    #endif
    */
    ~orb_slam3() override {
        SLAM_->Shutdown();
    }

private:
    const std::shared_ptr<switchboard>                            switchboard_;
    switchboard::writer<data_format::pose::head_pose_type>        pose_;
    switchboard::buffered_reader<data_format::binocular_cam_type> cam_reader_;
    switchboard::ptr<const data_format::binocular_cam_type>       cam_;
    switchboard::ptr<const data_format::binocular_cam_type>       cam_buffer_;
    switchboard::writer<data_format::imu_integrator_input>        imu_integrator_input_;
    bool                                                          is_first_cam_ = true;

    double min_runtime_   = 1000000000;
    double max_runtime_   = -10;
    double total_runtime_ = 0;
    int    num_runtime_   = 0;

    std::unique_ptr<ORB_SLAM3::System> SLAM_;
    ORB_SLAM3::Tracking*               slam_tracker_ = nullptr;
    ORB_SLAM3::Frame                   output_frame_;

    std::vector<ORB_SLAM3::IMU::Point> current_input_;
    std::vector<ORB_SLAM3::IMU::Point> prev_input_;
    boost::filesystem::path            root_path_;
    boost::filesystem::path            vocab_path_;
    boost::filesystem::path            setting_path_;
    bool                               use_zed_ = false;
};

} // namespace ILLIXR
