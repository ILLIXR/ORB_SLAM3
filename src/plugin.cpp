
#include <functional>
#include <fstream>
#include <opencv/cv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <math.h>
#include <eigen3/Eigen/Dense>
#include <boost/filesystem.hpp>

#include<System.h>
#include "ImuTypes.h"
#include "Optimizer.h"
#include "Tracking.h"

#include "../common/plugin.hpp"
#include "../common/phonebook.hpp"
#include "../common/switchboard.hpp"
#include "../common/data_format.hpp"

using namespace ILLIXR;

//#define ZED

class orb_slam3 : public plugin {
public:
    orb_slam3(std::string name_, phonebook* pb_)
        : plugin{name_, pb_}
        , sb{pb->lookup_impl<switchboard>()}
        , _m_pose{sb->get_writer<pose_type>("slow_pose")}
        , _m_imu_integrator_input{sb->get_writer<imu_integrator_input>("imu_integrator_input")}
        , _m_begin{std::chrono::system_clock::now()}
        , root_path{getenv("ORB_SLAM_ROOT")}
        , imu_cam_buffer{nullptr}
    {
        _m_pose.put(_m_pose.allocate(
			std::chrono::time_point<std::chrono::system_clock>{},
			Eigen::Vector3f{0, 0, 0},
			Eigen::Quaternionf{1, 0, 0, 0}
		));

        boost::filesystem::path vocab_path = root_path / "Vocabulary" / "ORBvoc.txt"; 
        boost::filesystem::path setting_path = root_path / "src" / "config" / "euroc.yaml"; 

        // set up ORB_SLAM
        SLAM = std::make_unique<ORB_SLAM3::System>(vocab_path.string(), setting_path.string(), ORB_SLAM3::System::IMU_STEREO, false);
        
    #ifdef CV_HAS_METRICS
        cv::metrics::setAccount(new std::string{"-1"});
    #endif
    }

    virtual void start() override {
        plugin::start();
        sb->schedule<imu_cam_type>(id, "imu_cam", [&](switchboard::ptr<const imu_cam_type> datum, std::size_t iteration_no) {
			this->feed_imu_cam(datum, iteration_no);
		});
    }

    void feed_imu_cam(switchboard::ptr<const imu_cam_type> datum, std::size_t iteration_no){
        // Ensures that slam doesnt start before valid IMU readings come in
		if (datum == NULL) {
			assert(previous_timestamp == 0);
			return;
		}

        // This ensures that every data point is coming in chronological order If youre failing this assert, 
		// make sure that your data folder matches the name in offline_imu_cam/plugin.cc
		double timestamp_in_seconds = double(datum->dataset_time)/1e9;
        assert(datum->dataset_time > previous_timestamp);

        // Feed the IMU measurement. There should always be IMU data in each call to feed_imu_cam
        assert((datum->img0.has_value() && datum->img1.has_value()) || (!datum->img0.has_value() && !datum->img1.has_value()));
        cv::Point3f acc(datum->linear_a.x(), datum->linear_a.y(), datum->linear_a.z());
        cv::Point3f gyro(datum->angular_v.x(), datum->angular_v.y(), datum->angular_v.z());
        ORB_SLAM3::IMU::Point input_imu(acc.x, acc.y, acc.z, gyro.x, gyro.y, gyro.z, timestamp_in_seconds);
        input_imu_data.push_back(input_imu);

        // If there is not cam data this func call, break early
		if (!datum->img0.has_value() && !datum->img1.has_value()) {
			return;
		}else if (imu_cam_buffer == NULL) {
			imu_cam_buffer = datum;
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

        // get the images
        cv::Mat img0{imu_cam_buffer->img0.value()};
		cv::Mat img1{imu_cam_buffer->img1.value()};

        // Pass the images and imu data to the SLAM system
        slam_tracker = SLAM->returnTracker(img0,img1,timestamp_in_seconds,input_imu_data);  //SLAM->returnTracker(img0, img1, timestamp_in_seconds, input_imu_data);
        output_frame = slam_tracker->mCurrentFrame;

        Eigen::Vector3f posf = output_frame.GetImuPose().translation();
        Eigen::Vector3f pos = Eigen::Vector3f{posf.x(), posf.y(), posf.z()};
        Eigen::Vector3d posd = Eigen::Vector3d{double(posf.x()), double(posf.y()), double(posf.z())};

        Eigen::Quaternionf rotf(output_frame.GetImuPose().rotationMatrix());
        Eigen::Quaternionf rot = Eigen::Quaternionf{rotf.w(),rotf.x(),rotf.y(),rotf.z()};
        Eigen::Quaterniond rotd = Eigen::Quaterniond{double(rotf.w()),double(rotf.x()),double(rotf.y()),double(rotf.z())};
        
        Eigen::Vector3f velf = output_frame.GetVelocity();
        Eigen::Vector3d vel = Eigen::Vector3d{double(velf.x()), double(velf.y()), double(velf.z())};

        ORB_SLAM3::IMU::Bias imu_bias = output_frame.mImuBias;
        Eigen::Vector3d gyro_bias(double(imu_bias.bwx), double(imu_bias.bwy), double(imu_bias.bwz));
        Eigen::Vector3d acc_bias(double(imu_bias.bax), double(imu_bias.bay), double(imu_bias.baz));

        assert(isfinite(posf.x()));
        assert(isfinite(posf.y()));
        assert(isfinite(posf.z()));
        assert(isfinite(rotf.w()));
        assert(isfinite(rotf.x()));
        assert(isfinite(rotf.y()));
        assert(isfinite(rotf.z()));
        
        _m_pose.put(_m_pose.allocate(
            imu_cam_buffer->time,
            pos,
            rot
        ));
        
        _m_imu_integrator_input.put(_m_imu_integrator_input.allocate(
            timestamp_in_seconds,
            -0.05, // not sure
            imu_params{
                SLAM->settings_->noiseGyro(),
                SLAM->settings_->noiseAcc(),
                SLAM->settings_->gyroWalk(),
                SLAM->settings_->accWalk(),
                .n_gravity = Eigen::Matrix<double,3,1>(0.0, 0.0, -9.81),
                .imu_integration_sigma = 1.0,
                .nominal_rate = 200.0,
            },
            acc_bias,
            gyro_bias,
            posd,
            vel,
            rotd
        ));
        
        // clear imu vector if there are images
        input_imu_data.clear();

        imu_cam_buffer = datum;
    }

    virtual ~orb_slam3() override {}
private:
    const std::shared_ptr<switchboard> sb;
    switchboard::writer<pose_type> _m_pose;
    switchboard::writer<imu_integrator_input> _m_imu_integrator_input;
    time_type _m_begin;

    std::unique_ptr<ORB_SLAM3::System> SLAM;
    ORB_SLAM3::Tracking * slam_tracker;
    ORB_SLAM3::Frame output_frame;
    

    vector<ORB_SLAM3::IMU::Point> input_imu_data;
    boost::filesystem::path root_path;

    switchboard::ptr<const imu_cam_type> imu_cam_buffer;
    double previous_timestamp = 0.0;
};

PLUGIN_MAIN(orb_slam3);
