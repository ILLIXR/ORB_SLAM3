#include <functional>
#include <fstream>
#ifdef USING_OPENCV4
#include <opencv2/core.hpp>
#else
#include <opencv/cv.hpp>
#endif
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <chrono>
#include <vector>

#include <math.h>
#include <eigen3/Eigen/Dense>
#include <boost/filesystem.hpp>

#include <System.h>
#include "ImuTypes.h"
#include "Optimizer.h"
#include "Tracking.h"

#include "illixr/plugin.hpp"
#include "illixr/phonebook.hpp"
#include "illixr/switchboard.hpp"
#include "illixr/data_format.hpp"
#include "illixr/relative_clock.hpp"

#define STEREO_IMU

using namespace ILLIXR;

class orb_slam3 : public plugin {
public:
    orb_slam3(std::string name_, phonebook* pb_)
            : plugin{name_, pb_}
            , sb{pb->lookup_impl<switchboard>()}
            , _m_pose{sb->get_writer<pose_type>("slow_pose")}
            , root_path{getenv("ORB_SLAM_ROOT")}
            , vocab_path{root_path / "Vocabulary" / "ORBvoc.txt"}
            , _m_imu_integrator_input{sb->get_writer<imu_integrator_input>("imu_integrator_input")}
            , _m_cam{sb->get_buffered_reader<cam_type>("cam")}
            , cam_buffer{nullptr}
    {

        // set initial slow pose
        _m_pose.put(_m_pose.allocate(
                time_point{},
                Eigen::Vector3f{0, 0, 0},
                Eigen::Quaternionf{1, 0, 0, 0}
        ));

#ifdef CV_HAS_METRICS
        cv::metrics::setAccount(new std::string{"-1"});
#endif

        // set setting path and initialize ORB_SLAM3
#ifdef STEREO_IMU
        setting_path = root_path / "Examples" / "Stereo-Inertial" / "EuRoC.yaml";
        SLAM = std::make_unique<ORB_SLAM3::System>(vocab_path.string(), setting_path.string(), ORB_SLAM3::System::IMU_STEREO, false);


#else
        setting_path = root_path / "Examples" / "RGB-D" / "ETH3D.yaml";
        SLAM = std::make_unique<ORB_SLAM3::System>(vocab_path.string(), setting_path.string(), ORB_SLAM3::System::RGBD, false);
#endif
    }

    virtual void start() override {
        plugin::start();
#ifdef STEREO_IMU
        sb->schedule<imu_type>(id, "imu", [&](switchboard::ptr<const imu_type> datum, std::size_t iteration_no) {
            this->feed_imu_cam(datum, iteration_no);
        });
#else
        sb->schedule<rgb_depth_type>(id, "rgb_depth", [&](switchboard::ptr<const rgb_depth_type> datum, std::size_t iteration_no) {
			this->feed_rgbd(datum, iteration_no);
		});
#endif
    }

    void feed_imu_cam(switchboard::ptr<const imu_type> datum, std::size_t iteration_no){
        // Ensures that slam doesnt start before valid IMU readings come in
        if (datum == NULL) {
            return;
        }

        // Get current IMU data
        cv::Point3f acc(datum->linear_a.x(), datum->linear_a.y(), datum->linear_a.z());
        cv::Point3f gyro(datum->angular_v.x(), datum->angular_v.y(), datum->angular_v.z());
        ORB_SLAM3::IMU::Point input_imu(acc.x, acc.y, acc.z, gyro.x, gyro.y, gyro.z, duration2double(datum->time.time_since_epoch()));
        current_input.push_back(input_imu);

        if (cam_buffer) {
            if (cam_buffer->time <= datum->time) {
                cam = cam_buffer;
                cam_buffer = nullptr;
            } else {
                return;
            }
        } else {
            cam = _m_cam.size() == 0 ? nullptr : _m_cam.dequeue();
            // If there is not cam data this func call, break early
            if (!cam) {
                return;
            }
            if (cam->time > datum->time) {
                cam_buffer = cam;
                return;
            }
        }

        // If there is cam data, load IMU data from the last cam data up until now
        prev_input.clear();
        for (int i = 0; i < current_input.size(); i++){
            ORB_SLAM3::IMU::Point imu_point = current_input[i];
            prev_input.push_back(imu_point);
        }
        current_input.clear();

#ifdef CV_HAS_METRICS
        cv::metrics::setAccount(new std::string{std::to_string(iteration_no)});
		if (iteration_no % 20 == 0) {
		    cv::metrics::dump();
		}
#else
#warning "No OpenCV metrics available. Please recompile OpenCV from git clone --branch 3.4.6-instrumented https://github.com/ILLIXR/opencv/. (see install_deps.sh)"
#endif

        cv::Mat cam0{cam->img0};
        cv::Mat cam1{cam->img1};

        auto start = std::chrono::steady_clock::now();
        Sophus::SE3f mat_pose = SLAM->TrackStereo(cam0, cam1, duration2double(cam->time.time_since_epoch()), prev_input).inverse();
        auto end = std::chrono::steady_clock::now();

#ifndef NDEBUG
        // print duration to compute pose
        double duration = std::chrono::duration<double>(end-start).count();
        total_runtime += duration;
        min_runtime = std::min(min_runtime, duration);
        max_runtime = std::max(max_runtime, duration);
        num_runtime++;
        printf("timestamp: %f current: %f min: %f max: %f average: %f", duration2double(cam->time.time_since_epoch()), duration,
            min_runtime, max_runtime, total_runtime / num_runtime);
#endif

        slam_tracker = SLAM->mpTracker;
        if (slam_tracker->mState != ORB_SLAM3::Tracking::eTrackingState::OK && slam_tracker->mState != ORB_SLAM3::Tracking::eTrackingState::OK_KLT) {
            return;
        }
        output_frame = slam_tracker->mCurrentFrame;

        // get translation matrix
        Eigen::Vector3f trans = mat_pose.translation();
        Eigen::Vector3d posd = Eigen::Vector3d{double(trans.x()), double(trans.y()), double(trans.z())};

        //get rotation matrix
        Eigen::Quaternionf quat = mat_pose.unit_quaternion();
        Eigen::Quaterniond rotd = Eigen::Quaterniond{double(quat.w()),double(quat.x()),double(quat.y()),double(quat.z())};

        // get velocity vector
        Eigen::Vector3f velf = output_frame.GetVelocity();
        Eigen::Vector3d vel = Eigen::Vector3d{double(velf.x()), double(velf.y()), double(velf.z())};
        std::cout << "VEL: " << velf.x() << " " << velf.y() << " " << velf.z() << std::endl;

        // get bias
        ORB_SLAM3::IMU::Bias imu_bias = output_frame.mImuBias;
        Eigen::Vector3d gyro_bias(double(imu_bias.bwx), double(imu_bias.bwy), double(imu_bias.bwz));
        Eigen::Vector3d acc_bias(double(imu_bias.bax), double(imu_bias.bay), double(imu_bias.baz));
        Eigen::Vector3d zeroVector(0,0,0);

        // break early if there is no bias
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

        _m_pose.put(_m_pose.allocate(
                cam->time,
                trans,
                quat
        ));

        _m_imu_integrator_input.put(_m_imu_integrator_input.allocate(
                cam->time,
                ILLIXR::duration{0L},
                imu_params{
                        SLAM->settings_->noiseGyro(),
                        SLAM->settings_->noiseAcc(),
                        SLAM->settings_->gyroWalk(),
                        SLAM->settings_->accWalk(),
                        .n_gravity = Eigen::Matrix<double,3,1>(0.0, 0.0, -9.81),
                        .imu_integration_sigma = 1.0,
                        SLAM->settings_->imuFrequency()
                },
                acc_bias,
                gyro_bias,
                posd,
                vel,
                rotd
        ));
        // clear imu vector if there are images
        prev_input.clear();

    }

#ifndef STEREO_IMU
    void feed_rgbd(switchboard::ptr<const rgb_depth_type> datum, std::size_t iteration_no){
        // Ensures that slam doesnt start before valid IMU readings come in
		if (datum == NULL) {
			return;
		}

#ifdef CV_HAS_METRICS
		cv::metrics::setAccount(new std::string{std::to_string(iteration_no)});
		if (iteration_no % 20 == 0) {
		    cv::metrics::dump();
		}
#else
#warning "No OpenCV metrics available. Please recompile OpenCV from git clone --branch 3.4.6-instrumented https://github.com/ILLIXR/opencv/. (see install_deps.sh)"
#endif

        // get and clone the images
        cv::Mat img{datum->rgb};
		cv::Mat depth{datum->depth};
        cv::Mat input_cam = img.clone();
        cv::Mat input_depth = depth.clone();

        auto start = std::chrono::steady_clock::now();
        Sophus::SE3f mat_pose = SLAM->TrackRGBD(input_cam,input_depth,duration2double(datum->time.time_since_epoch())).inverse();
        auto end = std::chrono::steady_clock::now();

#ifndef NDEBUG
        // print duration to compute pose
        double duration = std::chrono::duration<double>(end-start).count();
        total_runtime += duration;
        min_runtime = std::min(min_runtime, duration);
        max_runtime = std::max(max_runtime, duration);
        num_runtime++;
        printf("timestamp: %f current: %f min: %f max: %f average: %f", duration2double(datum->time.time_since_epoch()), duration,
            min_runtime, max_runtime, total_runtime / num_runtime);
#endif

        slam_tracker = SLAM->mpTracker;
        if (slam_tracker->mState != ORB_SLAM3::Tracking::eTrackingState::OK) {
            return;
        }
        output_frame = slam_tracker->mCurrentFrame;

        // get translation matrix
        Eigen::Vector3f trans = mat_pose.translation();
        Eigen::Vector3d posd = Eigen::Vector3d{double(trans.x()), double(trans.y()), double(trans.z())};

        //get rotation matrix
        Eigen::Quaternionf quat = mat_pose.unit_quaternion();
        Eigen::Quaterniond rotd = Eigen::Quaterniond{double(quat.w()),double(quat.x()),double(quat.y()),double(quat.z())};

        // get velocity vector
        Eigen::Vector3f velf = output_frame.GetVelocity();
        Eigen::Vector3d vel = Eigen::Vector3d{double(velf.x()), double(velf.y()), double(velf.z())};

        assert(isfinite(posf[0]));
        assert(isfinite(posf[1]));
        assert(isfinite(posf[2]));
        assert(isfinite(rotf.w()));
        assert(isfinite(rotf.x()));
        assert(isfinite(rotf.y()));
        assert(isfinite(rotf.z()));

        _m_pose.put(_m_pose.allocate(
            datum->time,
            trans,
            quat
        ));

        // clear imu vector if there are images
        prev_input.clear();

    }
#endif

    virtual ~orb_slam3() override {
        SLAM->Shutdown();
    }

private:
    const std::shared_ptr<switchboard> sb;
    switchboard::writer<pose_type> _m_pose;
    std::shared_ptr<RelativeClock> _m_rtc;
    switchboard::buffered_reader<cam_type> _m_cam;
    switchboard::ptr<const cam_type> cam;
    switchboard::ptr<const cam_type> cam_buffer;
    switchboard::writer<imu_integrator_input> _m_imu_integrator_input;
    int cam_count;

    double min_runtime = 1000000000;
    double max_runtime = -10;
    double total_runtime = 0;
    int num_runtime = 0;

    std::unique_ptr<ORB_SLAM3::System> SLAM;
    ORB_SLAM3::Tracking * slam_tracker;
    ORB_SLAM3::Frame output_frame;

    std::vector<ORB_SLAM3::IMU::Point> current_input;
    std::vector<ORB_SLAM3::IMU::Point> prev_input;
    boost::filesystem::path root_path;
    boost::filesystem::path vocab_path;
    boost::filesystem::path setting_path;
};

PLUGIN_MAIN(orb_slam3);
