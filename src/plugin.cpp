#include <functional>

#include <opencv/cv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <math.h>
#include <eigen3/Eigen/Dense>

#include<System.h>
#include "ImuTypes.h"
#include "Optimizer.h"

#include "common/plugin.hpp"
#include "common/phonebook.hpp"
#include "common/switchboard.hpp"
#include "common/data_format.hpp"

using namespace ILLIXR;

class orb_slam3 : public plugin {
public:
    orb_slam3(std::string name_, phonebook* pb_)
        : plugin{name_, pb_}
        , sb{pb->lookup_impl<switchboard>()}
        , _m_pose{sb->get_writer<pose_type>("slow_pose")}
        , _m_imu_integrator_input{sb->get_writer<imu_integrator_input>("imu_integrator_input")}
        , _m_begin{std::chrono::system_clock::now()}
        , imu_cam_buffer{nullptr}
    {
        _m_pose.put(_m_pose.allocate(
			std::chrono::time_point<std::chrono::system_clock>{},
			Eigen::Vector3f{0, 0, 0},
			Eigen::Quaternionf{1, 0, 0, 0}
		));

        // set up ORB_SLAM
        std::string volcab_path; // TODO: add volcabulary path (txt)
        std::string setting_path; // TODO: add setting path (yaml)
        SLAM = std::make_unique<ORB_SLAM3::System>(volcab_path, setting_path, ORB_SLAM3::System::IMU_STEREO, false);
        
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
        // TODO:how old imu data is (buffered back by one) (NEED TO VERIFY THIS)!
		double timestamp_in_seconds = (double(datum->dataset_time) / NANO_SEC);
		assert(timestamp_in_seconds > previous_timestamp);
		previous_timestamp = timestamp_in_seconds;
        
        // TODO: how old cam data is (NEED TO VERIFY THIS)!
        double buffer_timestamp_seconds = double(imu_cam_buffer->dataset_time) / NANO_SEC;

        // Feed the IMU measurement. There should always be IMU data in each call to feed_imu_cam
        assert((datum->img0.has_value() && datum->img1.has_value()) || (!datum->img0.has_value() && !datum->img1.has_value()));
        
        input_imu_data.clear();
        if(timestamp_in_seconds <= buffer_timestamp_seconds) {
            const Eigen::Vector3f &acc = datum->linear_a;
            const Eigen::Vector3f &gyro = datum->angular_v;
            ORB_SLAM3::IMU::Point input_imu(acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2], timestamp_in_seconds);
            input_imu_data.push_back(input_imu);

            double timestamp_in_seconds = (double(datum->dataset_time) / NANO_SEC);
            assert(timestamp_in_seconds > previous_timestamp);
            previous_timestamp = timestamp_in_seconds;
        }

        // If there is not cam data this func call, break early
		if (!datum->img0.has_value() && !datum->img1.has_value()) {
			return;
		} else if (imu_cam_buffer == NULL) {
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
    
        // get the images, this might need to be modified further
        cv::Mat img0{imu_cam_buffer->img0.value()};
		cv::Mat img1{imu_cam_buffer->img1.value()};

        // Pass the images and imu data to the SLAM system
        Eigen::Vector3f slam_output = SLAM->TrackStereo(img0,img1,timestamp_in_seconds,input_imu_data);

        // TODO: convert the output to ILLIXR-readable output 
    }
private:
    const std::shared_ptr<switchboard> sb;
    switchboard::writer<pose_type> _m_pose;
    switchboard::writer<imu_integrator_input> _m_imu_integrator_input;
    time_type _m_begin;
    
    double previous_timestamp = 0.0;
    switchboard::ptr<const imu_cam_type> imu_cam_buffer;

    std::unique_ptr<ORB_SLAM3::System> SLAM;

    vector<ORB_SLAM3::IMU::Point> input_imu_data;
};

PLUGIN_MAIN(orb_slam3);