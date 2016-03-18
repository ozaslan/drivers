// *************************************** //
// Author : Salih Tolga Ozaslan            //
// Year   : June 2014                      //
//                                         //

#include "bluefox_camera.hh"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <sensor_msgs/CameraInfo.h>
#include <calib_params.hh>
//#include "yaml-cpp/yaml.h"

// ### TODO List
// * Option for undistortion. Publish from a dedicated topic
// * White balance
// * Send parameters through topic / update if any changes
// * Listen to a topic for parameter changes
// * Write device printer standalone applications (test apps)
// * Camera selection using serial number
// * Unplug/plug detection and restart camera
// * Publish in different color spaces (first try the driver, if not let opencv do it)
// * AOI in launch file
// * Try to get exposure, gain etc. params when set to auto mode
// * Do binning if image size reduction is required.
// * Add flash control (for dam)
// * Pixel format still cannot be modified (leave it!)
// * Test mode
// * Save images to file (with folder names and image naming format | save color, distorted, undistorted)
// * Allow modification of HDR Knee values 
// * Do better naming for topics
// * If camera not found, keep runnig the node, but publish empty image
// * Allow methods for auto while balance calibration (save to user data files if you think this makes sense)
// * ...

using namespace std;
using namespace cv;

int		_trigger_mode;
int		_cam_idx;
bool	_debug_mode;
bool	_hdr_mode;
bool	_color_mode;
double	_fps;
double	_gain_dB;
double	_timeout;
double	_clock_rate;
double	_exposure_us;
double  _frame_delay;
string  _camera_name;
string  _cam_serial;
int		_width, _height;
bool    _print_devices;
bool    _print_stats;
bool    _flip_image;
sensor_msgs::CameraInfo _camInfo;
int		_aoi_x		, _aoi_y, 
		_aoi_width	, _aoi_height;
string _calib_file;

int process_inputs(const ros::NodeHandle &n)                                                                                                                           
{
	n.param("trigger_mode" , _trigger_mode , -1);
	n.param("camera_idx"   , _cam_idx	   , 0);
	n.param("debug_mode"   , _debug_mode   , false);
    n.param("hdr_mode"	   , _hdr_mode	   , true);
	n.param("color_mode"   , _color_mode   , true);
    n.param("fps"		   , _fps		   , 60.0);
	n.param("gain_dB"	   , _gain_dB	   , 0.10);
    n.param("timeout"	   , _timeout	   , 2000.0);
    n.param("clock_rate"   , _clock_rate   , 40.0);
    n.param("exposure_us"  , _exposure_us  , 30000.0);
    n.param("frame_delay"  , _frame_delay  , 0.0);
	n.param("camera_name"  , _camera_name  , string("left"));
	n.param("width"		   , _width		   , 640);
	n.param("height"	   , _height	   , 480);
	n.param("print_devices", _print_devices, false);
	n.param("print_stats"  , _print_stats  , false);
	n.param("flip_image"   , _flip_image   , false);
	n.param("camera_serial", _cam_serial   , string(""));
	n.param("aoi_x"		   , _aoi_x        , -1);
	n.param("aoi_y"		   , _aoi_y        , -1);
	n.param("aoi_width"	   , _aoi_width    , -1);
	n.param("aoi_height"   , _aoi_height   , -1);
	n.param("calib_file"   , _calib_file   , string(""));

	// Load the camera calibration file and populate the camera info message.	
	CameraCalibParams params;
	params.load(_calib_file);

	_camInfo.width  = _width;
	_camInfo.height = _height;

	_camInfo.distortion_model = params.dist_model;
	_camInfo.D = params.dist_coeffs;
	for(int r = 0 ; r < 3 ; r++){
		for(int c = 0 ; c < 3 ; c++){
		_camInfo.K[r * 3 + c] = params.camera_matrix.at<double>(r, c);
		_camInfo.R[r * 3 + c] = 0;
		}
	}
	_camInfo.R[0] = _camInfo.R[4] = _camInfo.R[8] = 1;
	
	for(int r = 0 ; r < 3 ; r++){
		for(int c = 0 ; c < 4 ; c++){
		_camInfo.P[r * 4 + c] = params.projection_matrix.at<double>(r, c);
		}
	}

	_camInfo.binning_x = 0;
	_camInfo.binning_y = 0;
	_camInfo.roi.x_offset	= 0;
	_camInfo.roi.y_offset	= 0;
	_camInfo.roi.height		= _height;
	_camInfo.roi.width		= _width;
	_camInfo.roi.do_rectify	= false;

	// ------------------------------------------------------------------- //
    ROS_INFO(" ---------------- BLUEFOX NODE ------------------");
	ROS_INFO("[trigger_mode] ----- : [%d]", _trigger_mode);
    ROS_INFO("[camera_idx] ------- : [%d]", _cam_idx);
    ROS_INFO("[camera_serial] ---- : [%s]", _cam_serial.c_str());
    ROS_INFO("[debug_mode] ------- : [%s]", _debug_mode ? "TRUE" : "FALSE");
    ROS_INFO("[hdr_mode] --------- : [%s]", _hdr_mode   ? "TRUE" : "FALSE");
    ROS_INFO("[color_mode] ------- : [%s]", _color_mode ? "TRUE" : "FALSE");
    ROS_INFO("[fps] -------------- : [%.3lf]"  , _fps);
    ROS_INFO("[gain_dB] ---------- : [%.3lf]"  , _gain_dB);
    ROS_INFO("[timeout] ---------- : [%.3lf]"  , _timeout);
    ROS_INFO("[clock_rate] ------- : [%.3lf]"  , _clock_rate);
    ROS_INFO("[exposure_us] ------ : [%.3lf]"  , _exposure_us);
    ROS_INFO("[frame_delay] ------ : [%.3lf]"  , _frame_delay);
	ROS_INFO("[camera_name] ------ : [%s]"     , _camera_name.c_str());
	ROS_INFO("[width x height] --- : [%d x %d]", _width, _height);
	ROS_INFO("[print_devices] ---- : [%s]"	 , _print_devices ? "TRUE" : "FALSE");
	ROS_INFO("[print_stats] ------ : [%s]"	 , _print_stats ? "TRUE" : "FALSE");
	ROS_INFO("[flip_image] ------- : [%s]"	 , _flip_image ? "TRUE" : "FALSE");
	ROS_INFO("AOI-[x, y, w, h] --- : [%d, %d, %d, %d]"	, _aoi_x, _aoi_y, _aoi_width, _aoi_height);
	ROS_INFO("Camera calibration parameters : ");
	params.print();
    ROS_INFO(" ------------------------------------------------");
	return 0;
}

int main(int argc, char* argv[]){
	ros::init(argc, argv, "bluefox_node");
	ros::NodeHandle nh("~");
	
	BlueFoxCamera bfcam;

	process_inputs(nh);
	
	image_transport::ImageTransport it(nh);
	string topic_name = "/" + _camera_name + "/image_raw";
	image_transport::Publisher image_publ = it.advertise(topic_name.c_str(), 1);
	ros::Publisher camInfo_publ = nh.advertise<sensor_msgs::CameraInfo>("/" + _camera_name + "/camera_info", 1);

	if(_print_devices)
		bfcam.print_available_devices(false, false);

	cv::Mat frame;
	if(_cam_serial == ""){
		ROS_INFO("Camera serial # not provided. Starting camera using the index value.");
		bfcam.open(_cam_idx);
	} else {
		ROS_INFO("Starting the camera with the serial # : %s", _cam_serial.c_str());
		bfcam.open(_cam_serial);
	}

	bfcam.set_trigger_source(_trigger_mode);
	bfcam.set_hdr_mode(_hdr_mode);
	bfcam.set_gain_dB(_gain_dB);
	bfcam.set_image_request_timeout(_timeout);
	bfcam.set_pixel_clock_rate(_clock_rate);
	bfcam.set_exposure_us(_exposure_us);
	bfcam.set_frame_delay_us(_frame_delay);
	bfcam.set_aoi(_aoi_x, _aoi_y, _aoi_width, _aoi_height);

	int seq = 0;

	bfcam.set_binning_mode(0,0,0);
	ros::Rate rate(_fps);

	while (nh.ok()) {
		bfcam.grab_frame(frame, _print_stats);
	
		cv_bridge::CvImage cv_image;
		cv_image.header.seq = seq++;
		cv_image.header.frame_id = _camera_name;
		cv_image.header.stamp    = ros::Time::now();

		if(frame.rows != _height || frame.cols != _width)
			resize(frame, frame, Size(_width, _height));

		if(_flip_image)
			flip(frame, frame, -1);

		if(_color_mode == false) {
			if(frame.channels() == 3)
				cvtColor(frame, cv_image.image, CV_BGR2GRAY);
			else if(frame.channels() == 4)
				cvtColor(frame, cv_image.image, CV_BGRA2GRAY);
			else if(frame.channels() == 1)
				cv_image.image = frame;
			cv_image.encoding = "mono8";
		} else if(_color_mode == true) {
			if(frame.channels() == 3)
				cv_image.image = frame;
			else if(frame.channels() == 4)
				cvtColor(frame, cv_image.image, CV_BGRA2RGB);
			else if(frame.channels() == 1)
				cvtColor(frame, cv_image.image, CV_GRAY2RGB);
			cv_image.encoding = "rgb8";
		}

		image_publ.publish(cv_image.toImageMsg());
		_camInfo.header = cv_image.header;
		camInfo_publ.publish(_camInfo);
	    
		rate.sleep();
	}

	bfcam.close();
	return 0;
}
