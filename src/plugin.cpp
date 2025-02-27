#include "plugin.hpp"

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <opencv2/core.hpp>


using namespace ILLIXR;
using namespace ILLIXR::data_format;

orb_slam3::orb_slam3(const std::string& name_, phonebook* pb_)
        : plugin{name_, pb_}
        , switchboard_{phonebook_->lookup_impl<switchboard>()}
        , pose_{switchboard_->get_writer<pose_type>("slow_pose")}
        , imu_integrator_input_{switchboard_->get_writer<imu_integrator_input>("imu_integrator_input")}
        , cam_reader_{switchboard_->get_buffered_reader<binocular_cam_type>("cam")}
        , cam_buffer_{nullptr} {
    assert(getenv("ILLIXR_BINARY_PATH"));
    root_path_  = boost::filesystem::path(getenv("ILLIXR_BINARY_PATH"));
    root_path_ = root_path_  / ".." / "share" / "ORB_SLAM3";
    vocab_path_ = root_path_ / "Vocabulary" / "ORBvoc.txt";
    use_zed_    = ILLIXR::str_to_bool(ILLIXR::getenv_or("USE_ZED", "False"));
    // set initial slow pose
    pose_.put(pose_.allocate(time_point{}, Eigen::Vector3f{0, 0, 0}, Eigen::Quaternionf{1, 0, 0, 0}));

    assert(boost::filesystem::exists(vocab_path_));

    // set setting path and initialize ORB_SLAM3
    if (!use_zed_) {
        setting_path_ = root_path_ / "Examples" / "Stereo-Inertial" / "EuRoC.yaml";
        assert(boost::filesystem::exists(setting_path_));
        SLAM_ = std::make_unique<ORB_SLAM3::System>(vocab_path_.string(), setting_path_.string(), ORB_SLAM3::System::IMU_STEREO,
                                                    false);
    } else {
        setting_path_ = root_path_ / "Examples" / "Stereo-Inertial" / "Zed.yaml";
        assert(boost::filesystem::exists(setting_path_));
        SLAM_ = std::make_unique<ORB_SLAM3::System>(vocab_path_.string(), setting_path_.string(), ORB_SLAM3::System::IMU_STEREO,
                                                    false);
    }
    /*
    setting_path_ = root_path_ / "Examples" / "RGB-D" / "ETH3D.yaml";
    assert(boost::filesystem::exists(setting_path_));
    SLAM_ = std::make_unique<ORB_SLAM3::System>(vocab_path_.string(), setting_path_.string(), ORB_SLAM3::System::RGBD, false);
*/
}

void orb_slam3::start() {
    plugin::start();

    switchboard_->schedule<imu_type>(id_, "imu", [this](const switchboard::ptr<const imu_type>& datum, std::size_t iteration_no) {
        this->feed_imu_cam(datum, iteration_no);
    });
    /*
    switchboard_->schedule<rgb_depth_type>(id, "rgb_depth",
                                           [&](switchboard::ptr<const rgb_depth_type> datum, std::size_t iteration_no) {
                                               this->feed_rgbd(datum, iteration_no);
                                           });
    */
}

void orb_slam3::feed_imu_cam(const switchboard::ptr<const imu_type>& datum, std::size_t iteration_no) {
    (void) iteration_no;
    // Ensures that slam doesnt start before valid IMU readings come in
    if (datum == nullptr) {
        return;
    }

    // Get current IMU data
    cv::Point3f           acc(static_cast<float>(datum->linear_a.x()), static_cast<float>(datum->linear_a.y()), static_cast<float>(datum->linear_a.z()));
    cv::Point3f           gyro(static_cast<float>(datum->angular_v.x()), static_cast<float>(datum->angular_v.y()), static_cast<float>(datum->angular_v.z()));
    ORB_SLAM3::IMU::Point input_imu(acc.x, acc.y, acc.z, gyro.x, gyro.y, gyro.z,
                                    duration_to_double(datum->time.time_since_epoch()));
    current_input_.push_back(input_imu);

    if (cam_buffer_) {
        if (cam_buffer_->time <= datum->time) {
            cam_        = cam_buffer_;
            cam_buffer_ = nullptr;
        } else {
            return;
        }
    } else {
        cam_ = cam_reader_.size() == 0 ? nullptr : cam_reader_.dequeue();
        // If there is no cam data this func call, break early
        if (!cam_) {
            return;
        }
        if (cam_->time > datum->time) {
            cam_buffer_ = cam_;
            return;
        }
    }

    // If there is cam data, load IMU data from the last cam data up until now
    prev_input_.clear();
    if (!is_first_cam_) {
        for (const auto& imu_point : current_input_) {
            prev_input_.push_back(imu_point);
        }
    } else {
        is_first_cam_ = false;
    }
    current_input_.clear();

    cv::Mat cam0{cam_->at(image::LEFT_EYE)};
    cv::Mat cam1{cam_->at(image::RIGHT_EYE)};

    auto start = std::chrono::steady_clock::now();
    SLAM_->TrackStereo(cam0, cam1, duration_to_double(cam_->time.time_since_epoch()), prev_input_);
    auto end = std::chrono::steady_clock::now();

#ifndef NDEBUG
    // print duration to compute pose
    double duration = std::chrono::duration<double>(end - start).count();
    total_runtime_ += duration;
    min_runtime_ = std::min(min_runtime_, duration);
    max_runtime_ = std::max(max_runtime_, duration);
    num_runtime_++;
    printf("timestamp: %f current: %f min: %f max: %f average: %f", duration_to_double(cam_->time.time_since_epoch()), duration,
           min_runtime_, max_runtime_, total_runtime_ / num_runtime_);
#endif

    slam_tracker_ = SLAM_->mpTracker;
    if (slam_tracker_->mState != ORB_SLAM3::Tracking::eTrackingState::OK &&
        slam_tracker_->mState != ORB_SLAM3::Tracking::eTrackingState::OK_KLT) {
        return;
    }
    output_frame_ = slam_tracker_->mCurrentFrame;

    // get translation matrix
    Eigen::Vector3f trans = output_frame_.GetImuPose().translation();
    Eigen::Vector3d posd  = Eigen::Vector3d{double(trans.x()), double(trans.y()), double(trans.z())};

    // get rotation matrix
    Eigen::Quaternionf quat = output_frame_.GetImuPose().unit_quaternion();
    Eigen::Quaterniond rotd = Eigen::Quaterniond{double(quat.w()), double(quat.x()), double(quat.y()), double(quat.z())};

    // get velocity vector
    Eigen::Vector3f velf = output_frame_.GetVelocity();
    Eigen::Vector3d vel  = Eigen::Vector3d{double(velf.x()), double(velf.y()), double(velf.z())};

    // get bias
    ORB_SLAM3::IMU::Bias imu_bias = output_frame_.mImuBias;
    Eigen::Vector3d      gyro_bias(double(imu_bias.bwx), double(imu_bias.bwy), double(imu_bias.bwz));
    Eigen::Vector3d      acc_bias(double(imu_bias.bax), double(imu_bias.bay), double(imu_bias.baz));
    Eigen::Vector3d      zeroVector(0, 0, 0);

    // IMU has not been initialized yet
    if (gyro_bias == zeroVector && acc_bias == zeroVector) {
        return;
    }

    assert(isfinite(posd[0]));
    assert(isfinite(posd[1]));
    assert(isfinite(posd[2]));
    assert(isfinite(rotd.w()));
    assert(isfinite(rotd.x()));
    assert(isfinite(rotd.y()));
    assert(isfinite(rotd.z()));

    pose_.put(pose_.allocate(cam_->time, trans, quat));

    imu_integrator_input_.put(
            imu_integrator_input_.allocate(cam_->time, ILLIXR::duration{0L},
                                           imu_params{.gyro_noise            = SLAM_->settings_->noiseGyro(),
                                                   .acc_noise             = SLAM_->settings_->noiseAcc(),
                                                   .gyro_walk             = SLAM_->settings_->gyroWalk(),
                                                   .acc_walk              = SLAM_->settings_->accWalk(),
                                                   .n_gravity             = Eigen::Matrix<double, 3, 1>(0.0, 0.0, -9.81),
                                                   .imu_integration_sigma = 1.0,
                                                   .nominal_rate          = SLAM_->settings_->imuFrequency()},
                                           acc_bias, gyro_bias, posd, vel, rotd));

    // clear imu vector if there are images
    prev_input_.clear();
    cam_buffer_ = nullptr;
}

/*
#if !defined(STEREO_IMU) && !defined(ZED)
void orb_slam3::feed_rgbd(switchboard::ptr<const rgb_depth_type> datum, std::size_t iteration_no){
    // Ensures that slam doesnt start before valid IMU readings come in
    if (datum == NULL) {
        return;
    }


    // get and clone the images
    cv::Mat img{datum->at(image::RGB)};
    cv::Mat depth{datum->at(image::DEPTH)};
    cv::Mat input_cam = img.clone();
    cv::Mat input_depth = depth.clone();

    auto start = std::chrono::steady_clock::now();
    SLAM_->TrackRGBD(input_cam,input_depth,duration_to_double(datum->time.time_since_epoch())).inverse();
    auto end = std::chrono::steady_clock::now();

#ifndef NDEBUG
    // print duration to compute pose
    double duration = std::chrono::duration<double>(end-start).count();
    total_runtime_ += duration;
    min_runtime_ = std::min(min_runtime_, duration);
    max_runtime_ = std::max(max_runtime_, duration);
    num_runtime_++;
    printf("timestamp: %f current: %f min: %f max: %f average: %f", duration_to_double(datum->time.time_since_epoch()),
duration, min_runtime_, max_runtime_, total_runtime_ / num_runtime_); #endif

    slam_tracker_ = SLAM_->mpTracker;
    if (slam_tracker_->mState != ORB_SLAM3::Tracking::eTrackingState::OK) {
        return;
    }
    output_frame_ = slam_tracker_->mCurrentFrame;

    // get translation matrix
    Eigen::Vector3f trans = output_frame_.GetPoseInverse().translation();
    Eigen::Vector3d posd = Eigen::Vector3d{double(trans.x()), double(trans.y()), double(trans.z())};

    //get rotation matrix
    Eigen::Quaternionf quat = output_frame_.GetPoseInverse().unit_quaternion();
    Eigen::Quaterniond rotd = Eigen::Quaterniond{double(quat.w()),double(quat.x()),double(quat.y()),double(quat.z())};

    // get velocity vector
    Eigen::Vector3f velf = output_frame_.GetVelocity();
    Eigen::Vector3d vel = Eigen::Vector3d{double(velf.x()), double(velf.y()), double(velf.z())};

    assert(isfinite(posd[0]));
    assert(isfinite(posd[1]));
    assert(isfinite(posd[2]));
    assert(isfinite(rotd.w()));
    assert(isfinite(rotd.x()));
    assert(isfinite(rotd.y()));
    assert(isfinite(rotd.z()));

    pose_.put(pose_.allocate(
        datum->time,
        trans,
        quat
    ));

    // clear imu vector if there are images
    prev_input_.clear();

}
#endif
*/
PLUGIN_MAIN(orb_slam3)
