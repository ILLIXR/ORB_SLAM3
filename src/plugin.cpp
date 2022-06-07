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
#include "../common/relative_clock.hpp"

using namespace ILLIXR;

class orb_slam3 : public plugin {
public:
    orb_slam3(std::string name_, phonebook* pb_)
        : plugin{name_, pb_}
        , sb{pb->lookup_impl<switchboard>()}
        , _m_pose{sb->get_writer<pose_type>("slow_pose")}
        , _m_imu_integrator_input{sb->get_writer<imu_integrator_input>("imu_integrator_input")}
        , root_path{getenv("ORB_SLAM_ROOT")}
    {

        _m_pose.put(_m_pose.allocate(
			time_point{},
			Eigen::Vector3f{0, 0, 0},
			Eigen::Quaternionf{1, 0, 0, 0}
		));

        // TODO: set vocab and setting paths
        boost::filesystem::path vocab_path = root_path / "Vocabulary" / "ORBvoc.txt"; 
        boost::filesystem::path setting_path = root_path / "Examples" / "Stereo-Inertial" / "EuRoC.yaml";

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
			return;
		}

        // Get current IMU data
        cv::Point3f acc(datum->linear_a.x(), datum->linear_a.y(), datum->linear_a.z());
        cv::Point3f gyro(datum->angular_v.x(), datum->angular_v.y(), datum->angular_v.z());
        ORB_SLAM3::IMU::Point input_imu(acc.x, acc.y, acc.z, gyro.x, gyro.y, gyro.z, duration2double(datum->time.time_since_epoch()));
        
        // If there is cam data, load IMU data from the last cam data up until now
        if (datum->img0.has_value() || datum->img1.has_value()) {
            prev_input.clear();
            for (int i = 0; i < current_input.size(); i++){
                ORB_SLAM3::IMU::Point imu_point = current_input[i];
                prev_input.push_back(imu_point);
            }
            current_input.clear();
            current_input.push_back(input_imu);

            assert((datum->img0.has_value() && datum->img1.has_value()) || (!datum->img0.has_value() && !datum->img1.has_value()));

        // If there is not cam data this func call, break early
		} else {
            current_input.push_back(input_imu);
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
        cv::Mat img0{datum->img0.value()};
		cv::Mat img1{datum->img1.value()};

        cv::Mat im_left = img0.clone();
        cv::Mat im_right = img1.clone();
        
        // Pass the images and imu data to the SLAM system
        slam_tracker = SLAM->returnTracker(im_left,im_right,duration2double(datum->time.time_since_epoch()),prev_input);
        output_frame = slam_tracker->mCurrentFrame;

        // get translation matrix
        Eigen::Vector3f posf = output_frame.GetImuPosition();
        Eigen::Vector3f pos = Eigen::Vector3f{posf.x(), posf.y(), posf.z()};
        Eigen::Vector3d posd = Eigen::Vector3d{double(posf.x()), double(posf.y()), double(posf.z())};

        //get rotation matrix
        Eigen::Quaternionf rotf(output_frame.GetImuRotation());
        Eigen::Quaternionf rot = Eigen::Quaternionf{rotf.w(),rotf.x(),rotf.y(),rotf.z()};
        Eigen::Quaterniond rotd = Eigen::Quaterniond{double(rotf.w()),double(rotf.x()),double(rotf.y()),double(rotf.z())};

        // get velocity vector
        Eigen::Vector3f velf = output_frame.GetVelocity();
        Eigen::Vector3d vel = Eigen::Vector3d{double(velf.x()), double(velf.y()), double(velf.z())};

        // get bias
        ORB_SLAM3::IMU::Bias imu_bias = output_frame.mPredBias;
        Eigen::Vector3d gyro_bias(double(imu_bias.bwx), double(imu_bias.bwy), double(imu_bias.bwz));
        Eigen::Vector3d acc_bias(double(imu_bias.bax), double(imu_bias.bay), double(imu_bias.baz));
        Eigen::Vector3d zeroVector(0,0,0);

        // break early if there is no bias
        if (gyro_bias == zeroVector && acc_bias == zeroVector) {
            return;
        }

        assert(isfinite(posf[0]));
        assert(isfinite(posf[1]));
        assert(isfinite(posf[2]));
        assert(isfinite(rotf.w()));
        assert(isfinite(rotf.x()));
        assert(isfinite(rotf.y()));
        assert(isfinite(rotf.z()));
        
        _m_pose.put(_m_pose.allocate(
            datum->time,
            pos,
            rot
        ));
    
        _m_imu_integrator_input.put(_m_imu_integrator_input.allocate(
            datum->time,
            duration{0L},
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

    virtual ~orb_slam3() override {
        SLAM->Shutdown();
    }
    
private:
    const std::shared_ptr<switchboard> sb;
    switchboard::writer<pose_type> _m_pose;
    switchboard::writer<imu_integrator_input> _m_imu_integrator_input;

    std::unique_ptr<ORB_SLAM3::System> SLAM;
    ORB_SLAM3::Tracking * slam_tracker;
    ORB_SLAM3::Frame output_frame;
    
    vector<ORB_SLAM3::IMU::Point> current_input;
    vector<ORB_SLAM3::IMU::Point> prev_input;
    boost::filesystem::path root_path;
};

PLUGIN_MAIN(orb_slam3);
