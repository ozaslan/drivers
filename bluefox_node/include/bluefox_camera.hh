// *************************************** //
// Author : Salih Tolga Ozaslan            //
// Year   : June 2014                      //
//                                         //
#ifndef __BLUEFOX_CAMERA_HH__
#define __BLUEFOX_CAMERA_HH__

#include <string>
#include <opencv2/opencv.hpp>
#include <mvIMPACT_CPP/mvIMPACT_acquire.h> 

using namespace cv;
using namespace std;
using namespace mvIMPACT::acquire;

// Comparison function fed to algorithm::sort(...) in set_pixel_clock_rate(...) function.
bool pair_sort(pair<TCameraPixelClock, double> p1, pair<TCameraPixelClock, double> p2){
	return p1.second < p2.second;
}

class BlueFoxCamera{
private:
	// For details of these classes, resort to the online/chm manual.
	// (requires registration.)
	DeviceManager _devMgr; 
	Device				*_pDev;
	SettingsBlueFOX		*_settings; // ImageDestination - ImageProcessing
	FunctionInterface	*_fi;
	Statistics			*_stat;
	Request				*_request;
	ImageRequestControl *_irc;
	HDRControl			*_hdr;

	int    _device_idx;			// Store the index of the device as it appears in the device list {see print_available_devices(...)}
	string _device_serial;		// Store the serial of the device. This is going to be set to a non-empty string if the 
								// open(string ...) function is used.
	int    _timeout_ms;			// The duration for which the image request will be waited. If the image does not arrive
								// in that interval, an error is generated and no image returned. {see grab_frame(...)}
	bool   _device_initialized;	// After opening the camera, an initialization step is required. This flag
								// keeps track whether this process is fulfilled. {see _initialize_device(...)}

	// After the device is opened, this function is called before any image acquisition.
	// This function first checks if the device is already in use. Then makes an dummy image request
	// to check if the acquisition is successful. If required, then manual image acquisition is started.
	// (Some devices start sending images when it is plugged, whereas devices including BlueFOX waits
	// for a start-up signal)
	int _initialize_device(); 

	// The following two print functions are helper routines for print_available_devices(...)
	int _print_device_details(Device *pDev, int idx, bool print_detailed = false, bool print_invisible = false);
	int _print_component_list(Component &c, int depth = 0, bool print_invisible = false);

	// This functions is a helper function for grab_frame(...). It allocates memory
	// and copies the image from driver buffer to user given buffer.
	// NOTE : This function has been tested with BlueFOX MLC200wC and MLC202wC.
	// Both use 'ibpfRGBx888Packed' pixel format.
	int _copy_image(cv::Mat &frame);

	// This is a helper function for _copy_image(...) which gives the bits
	// used to store one pixel in the driver buffer.
	int _get_bits_per_pixel();
public:
	// --------------------------------------------------------------------- //
	// These function return '0' if the operation is successful.
	// Otherwise a negative value is returned.
	// --------------------------------------------------------------------- //
	// This function queries the available devices and opens the 'idx'th camera
	// in the device list. This order may not be the same for different runs.
	// Once a device is selected,  MV drvier is going to remove this camera from
	// the available devices list. This fact should be taken into account when 
	// using multiple cameras. e.g. the second device will not have the index of
	// "1" but "0" in the second process.
	int open(int idx = 0);
	// This function is similar to open(int ...) function except the device is
	// selected by its serial number. In case the device cannot be found, device
	// pointer is set to NULL.
	int open(string serial);
	// This function releases the device, and it becomes available in
	// the list of driver's device query. It also releases the pointer
	// in order to prevent memory corruption or leakage. These pointers
	// include those for device itself, statistics, function interface.
	// These classes cannot be instantiated without a device. Thus 
	// use of pointers is a must in this case if we want them to be
	// global in the class's scope.
	int close();
	// This function assumes that the user has already initialized/selected
	// a device and got no error messages. Using the FunctionInterface class
	// grab_frame(...) queries for a frame and waits a certain amount of time
	// before actually reading in the image data into the user memory. 
	// Before that happens, it makes several checks to see whether the 
	// request is actually ready. Otherwise, informs the user through both
	// prompting and a negative return value. Once ready, it copies
	// the image into the 'frame' and sends a new request for the next
	// frame. It also prints statistical information such as frames per
	// second, number of missed frames etc. 
	int grab_frame(cv::Mat &frame, bool print_stats = false);
	// The private class 'Statistics' keeps track of the performance of the
	// frame grabbing process. This function is an interface to retreive
	// frames per second, number of errors encountered during fetching image
	// and time passed while processing the latest image in the drive stack.
	int get_stats(double &fps, int error_cnt, double &frame_time);
	// Printer function for the statistical values explained for the
	// 'get_stats(...)' function.
	int print_stats();
	// ------------------------------------------------------------------ //
	// AIO : Area of interest once set, commands the driver to crop the 
	// rectangle defined by the four parameters. It should be noted that
	// setting AIO is different than scaling the image. The latter keeps
	// the whole image, without cropping any pieces, while reducing the 
	// image size. The former, however, deletes some parts of the image,
	// and corresponds to using the subregion of the CCD of the camera.
	// A smaller AOI is going to increase the frame rate upto 50/60 frames
	// per second. If any of the cases (x < 0), (y < 0), (w < 0) or 
	// (h < 0) holds, AOI is reset to the full extend of the CCD.
	inline int set_aoi(int x, int y, int w, int h){
		int w_max = _settings->cameraSetting.aoiWidth.getMaxValue();
		int w_min = _settings->cameraSetting.aoiWidth.getMinValue();
		int h_max = _settings->cameraSetting.aoiHeight.getMaxValue();
		int h_min = _settings->cameraSetting.aoiHeight.getMinValue();
		int x_max = _settings->cameraSetting.aoiStartX.getMaxValue() - w_min;
		int x_min = _settings->cameraSetting.aoiStartX.getMinValue();
		int y_max = _settings->cameraSetting.aoiStartY.getMaxValue() - h_min;
		int y_min = _settings->cameraSetting.aoiStartY.getMinValue();

		//cout << "w_max : " << w_max << endl;
		//cout << "w_min : " << w_min << endl;
		//cout << "h_max : " << h_max << endl;
		//cout << "h_min : " << h_min << endl;
		//cout << "x_min : " << x_min << endl;
		//cout << "x_max : " << x_max << endl;
		//cout << "y_min : " << y_min << endl;
		//cout << "y_max : " << y_max << endl;

		if(x < 0 || y < 0 || w < 0 || h < 0){
			_settings->cameraSetting.aoiStartX.write(x_min);
			_settings->cameraSetting.aoiStartY.write(y_min);
			_settings->cameraSetting.aoiHeight.write(h_max);
			_settings->cameraSetting.aoiWidth.write(w_max);
			return 0;
		} else if(x > x_max || y > y_max || w > w_max || h > h_max || w < w_min || h < h_min){
			cout << "*** ERROR : Arguments are out of allowable bounds." << endl;
			cout << "*** given(x, y, w, h) = [" << x     << ", " << y     << ", " << w     << ", " << h     << "]" << endl;
			cout << "***   min(x, y, w, h) = [" << x_min << ", " << y_min << ", " << w_min << ", " << h_min << "]" << endl;
			cout << "***   max(x, y, w, h) = [" << x_max << ", " << y_max << ", " << w_max << ", " << h_max << "]" << endl;
			return -1;
		} else {
			_settings->cameraSetting.aoiStartX.write(x);
			_settings->cameraSetting.aoiStartY.write(y);
			_settings->cameraSetting.aoiHeight.write(h);
			_settings->cameraSetting.aoiWidth.write(w);
			return 0;
		}
	}

	// ###TO BE TESTED : In applications where external trigger is used
	// 'delay' defines the duration the camera waits for before it starts 
	// capturing the image. This function can be used to fine tune timing 
	// and obtain better synchronization between multiple cameras / IMUs / 
	// laser / scanners / flashers etc.
	inline int set_frame_delay_us(int delay){
		_settings->cameraSetting.frameDelay_us.write(delay);
		return 0;
	}

	// 'timeout' defines the duration after which the image request timeouts.
	// When it is set to '0' the timeout never elapses and blocks the code
	// until an image is sent. Otherwise, if image request timeouts, then the
	// drive will still return a request with an undetermined image data.
	// Thus, the image request should always be checked for validity.
	inline int set_image_request_timeout(int timeout){
		_timeout_ms = timeout;
		return 0;
	}
	// This function acts as an interface to modify the pixel clock rate 
	// of the camera. Different cameras are very likely to have different
	// sets of allowable clock rates. Thus, it may not be possible to
	// apply the 'rate' value directly. For allowable values either follow 
	// the code or resort to the online/chm manual, 'TCameraPixelClock' enumeration.
	// Smaller values reduce the frame rate of the camera and larger this 
	// value is, closer the camera to its highest frame rate. Thus, this function 
	// serves as an indirect way to modify the frame rate. However, smaller 
	// 'rate' values will result in greater noise/signal ratio and the image 
	// will be much noisier.
	inline int set_pixel_clock_rate(double rate){
		int num_valid_clks = _settings->cameraSetting.pixelClock_KHz.dictSize();
		vector<pair<TCameraPixelClock, double> > valid_clks;
		valid_clks.reserve(num_valid_clks);
		for(int i = 0 ; i < num_valid_clks ; i++){
			switch(_settings->cameraSetting.pixelClock_KHz.getTranslationDictValue(i)){
				case cpc57600KHz:
					valid_clks.push_back(make_pair(cpc57600KHz, 57.6));
					break;
				case cpc50000KHz:
					valid_clks.push_back(make_pair(cpc50000KHz, 50.0));
					break;
				case cpc40000KHz:
					valid_clks.push_back(make_pair(cpc40000KHz, 40.0));
					break;
				case cpc37600KHz:
					valid_clks.push_back(make_pair(cpc37600KHz, 37.6));
					break;
				case cpc32000KHz:
					valid_clks.push_back(make_pair(cpc32000KHz, 32.0));
					break;
				case cpc27000KHz:
					valid_clks.push_back(make_pair(cpc27000KHz, 27.0));
					break;
				case cpc24540KHz:
					valid_clks.push_back(make_pair(cpc24540KHz, 24.54));
					break;
				case cpc24000KHz:
					valid_clks.push_back(make_pair(cpc24000KHz, 24.0));
					break;
				case cpc20000KHz:
					valid_clks.push_back(make_pair(cpc20000KHz, 20.0));
					break;
				case cpc13500KHz:
					valid_clks.push_back(make_pair(cpc13500KHz, 13.5));
					break;
				case cpc12000KHz:
					valid_clks.push_back(make_pair(cpc12000KHz, 12.0));
					break;
				case cpc10000KHz:
					valid_clks.push_back(make_pair(cpc10000KHz, 10.0));
					break;
				case cpc8000KHz:
					valid_clks.push_back(make_pair(cpc8000KHz, 8.0));
					break;
				case cpc6000KHz:
					valid_clks.push_back(make_pair(cpc6000KHz, 6.0));
					break;
				default:
					break;
			}
		}	
		std::sort(valid_clks.begin(), valid_clks.end(), pair_sort);
		//for(int i = 0 ; i < valid_clks.size() ; i++){
		//	cout << "valid_clks[" << i << "] : " << valid_clks[i].second << endl;
		//}
		
		if( rate <= valid_clks[0].second){
			_settings->cameraSetting.pixelClock_KHz.write(valid_clks[0].first);
			//cout << "setting clock rate to : " << valid_clks[0].second << endl;
			return 0;
		} else if ( rate > valid_clks.back().second){
			_settings->cameraSetting.pixelClock_KHz.write(valid_clks.back().first);
			//cout << "setting clock rate to : " << valid_clks.back().second << endl;
			return 0;
		} else {
			for(int i = 0 ; i < (int)valid_clks.size() - 2 ; i++){
				if(rate > valid_clks[i].second && rate <= valid_clks[i+1].second){
					_settings->cameraSetting.pixelClock_KHz.write(valid_clks[i+1].first);
					//cout << "setting clock rate to : " << valid_clks[i+1].second << endl;
					return 0;
				}
			} 
		}

		return -1;
	}
	
	// This function sets the exposure time in microseconds. A negative value
	// sets the camera to auto exposure mode. Lower exposure values
	// will help in obtaining non-over-bright images when the camera is used
	// outdoors. Especially fish eye lenses should be used with lower exposure values
	// since they capture more light rays per unit area which results 
	// in brighter images than otherwise would be obtained with a normal lens.
	// Higher exposure values will slow the camera down. When small values are
	// are given, the camera will try to compensate for the dim image by
	// increasing the gain which in turn causes the noise amplification.
	// Thus the coder is recommended to set the gain control to manual mode.
	// NOTE : Changing the exposure time too frequently will reduce the
	// effective fps greatly. (Experienced reductions are 25fps -> 8fps / 60fps -> 20fps)
	inline int set_exposure_us(double expo_us){
		if(expo_us <= 0)
			_settings->cameraSetting.autoExposeControl.write(aecOn);
		else {
			_settings->cameraSetting.autoExposeControl.write(aecOff);
			_settings->cameraSetting.expose_us.write(expo_us);
		}
		return 0;
	}

	// This function is used to set the gain (in dB) applied to each
	// pixel value after the period of integration. Higher gains will
	// cause amplification of noise. A non-positive value will set the 
	// gain control to auto mode.
	inline int set_gain_dB(double gain){

		if(gain <= 0)
			_settings->cameraSetting.autoGainControl.write(agcOn);
		else {
			_settings->cameraSetting.autoGainControl.write(agcOff);
			double max_gain = _settings->cameraSetting.gain_dB.getMaxValue();
			if(max_gain < gain){
				cout << "*** Given gain (" << gain << ") is greater than the allowable value." << endl; 
				cout << "*** Setting the parameter to its maximum (" << max_gain << ")" << endl;
				gain = max_gain;
			}
			_settings->cameraSetting.gain_dB.write(gain);
		}
		return 0;
	}

	// This function is used to set the trigger source to either 
	// 'internal = -1', 'external0 = 0' or 'external1 = 1'. 
	// Before setting the mode, device is first checked for compatibility. 
	// Then in the case of internal trigger, the mode is set to 'continuous'. 
	// In this mode there is no requirement from the user to start the image 
	// capture process. Even if the user does not fetch the image from the 
	// driver, images are going to be captured to the drive buffer. This process 
	// will NOT load the CPU unless the user asks for image transfer. 
	// In the case of 'externalX' mode, image is captured when
	// the signal is high (and also an image request is sent) on the
	// 'inputX' pin. NOTE : Tested BlueFOX devices does not wait for
	// external trigger when input channel is set to 'ctsDigIn1' and
	// behaves as 'ctmContinuous'.
	inline int set_trigger_source(int source = -1){
		bool success = false;
		// check compatibility
		TCameraTriggerSource pin = ctsDigIn1;
		TCameraTriggerMode mode  = ctmContinuous;
		if(source == -1)
			mode = ctmContinuous;
		else if (source == 0 || source == 1){
			mode = ctmOnHighLevel;
			if (source == 0)
				pin = ctsDigIn0;
			else if(source == 1)
				pin = ctsDigIn1;
		}

		int num1 = _settings->cameraSetting.triggerMode.dictSize();
		int num2 = _settings->cameraSetting.triggerSource.dictSize();
	
		for(int i = 0 ; i < num1 && !success ; i++){
			//cout << _settings->cameraSetting.triggerMode.getTranslationDictString(i) << endl;
			if(_settings->cameraSetting.triggerMode.getTranslationDictValue(i) == mode){
				//cout << "Setting mode to : " << _settings->cameraSetting.triggerMode.getTranslationDictString(i) << endl;
				_settings->cameraSetting.triggerMode.write(mode);				
				if(source >= 0){
					for(int j = 0 ; j < num2 && !success ; j++){
						//cout << _settings->cameraSetting.triggerSource.getTranslationDictString() << endl;
						if(_settings->cameraSetting.triggerSource.getTranslationDictValue(j) == pin){
							//cout << "Setting input to : " << _settings->cameraSetting.triggerSource.getTranslationDictString() << endl;
							_settings->cameraSetting.triggerSource.write(pin);
							success = true;
						}
					}
				} else
					success = true;
			}
		}
		
		if(success == false){
			cout << "*** Failed to set up trigger mode. Your input should be" << endl;
			cout << "*** referring to one of the allowable settings listed below." << endl;
			for(int i = 0 ; i < num1 ; i++)
				if(_settings->cameraSetting.triggerMode.getTranslationDictValue(i) == ctmContinuous ||
				   _settings->cameraSetting.triggerMode.getTranslationDictValue(i) == ctmOnHighLevel)
					cout << "*** > Mode - " << _settings->cameraSetting.triggerMode.getTranslationDictString(i) << endl;
			for(int i = 0 ; i < num2 ; i++)
				if(_settings->cameraSetting.triggerSource.getTranslationDictValue(i) == ctsDigIn0 ||
				   _settings->cameraSetting.triggerSource.getTranslationDictValue(i) == ctsDigIn1)
					cout << "*** > Input - " << _settings->cameraSetting.triggerSource.getTranslationDictString(i) << endl;
		}
		return success;
	}

	// This function can be used to get a list (yet only printed) of available 
	// devices. The options provide alternatives in the details of the list.
	// When both inputs are 'false', a minimal list is printed which includes
	// name, serial, product, family and some more basic properties.
	// 'print_detailed' should be set in order to get list of all properties
	// of the available devices. When 'print_visible' is set to 'true',
	// properties which are in- active (e.g. when auto gain is set, user given
	// gain value does not affect the behaviour which we call in-active) are not
	// printed. Otherwise each parameter with flags describing the properties
	// of the property are printed as well.
	int print_available_devices(bool print_detailed = false, bool print_invisible = false);

	// This function first checks if the device supports HDR option. Then sets the
	// HDRKnee point to the zeroth default. In order HDR to be effectively working, 
	// exposure should either set to auto mode or a reasonably high exposure time 
	// should be given. This value depends on the environment lighting conditions. 
	// Upon setting HDR on, exposure can be modified using set_exposure_us(...) 
	// if the image is pale and/or dim.
	int set_hdr_mode(bool on = true){
		HDRControl &hdr = _settings->cameraSetting.getHDRControl();
		if(hdr.isAvailable()){
			set_exposure_us(-1);
			hdr.HDREnable.write(on ? bTrue : bFalse);
			hdr.HDRMode.write(cHDRmFixed0);
			return 0;
		} else {
			cout << "*** HDR is not available on this device." << endl;
			return -1;
		}
	}

	// This function sets the binnig mode of the camera. Depending on the
	// devide used, allowable modes differ. In case the current device does
	// not support the expected binnig mode, this function returns a negative
	// value and keeps the previously set binnig mode. This function accepts
	// three parameters which encode the number of pixels binned in horizontal
	// and vertical directions and methods used when calculating the resultant
	// RBG/Gray value. For example 'h = 3' will reduce the image width to 1/4
	// and 'v = 1' will halve the image height. The last parameters defines
	// the method used in calculating the resultant pixel value which is one
	// of 'averaging' or 'summing'. Lastly a negative 'h' or 'v' value corresponds
	// to dropping the intermediate pixels. See the Bluefox C++ API for more
	// details. 
	int set_binning_mode(int h, int v, int method){
		PropertyICameraBinningMode binning;
		/*
		if(h == 0 && v == 0)
			binning = cbmOff;
		else if(h == 1 && v == 1);
		*/
		int num_valid_binnings = _settings->cameraSetting.binningMode.dictSize();
		for(int i = 0 ; i < num_valid_binnings ; i++)
			cout << "binning[" << i << "] = " << _settings->cameraSetting.binningMode.getTranslationDictString(i) << endl;
				
		 _settings->cameraSetting.binningMode.write(cbmBinningH) ;
		return 0;
	}

	// Constructor sets the private variables to their default values.
	BlueFoxCamera();

	// See set_aoi(...)	function.
	inline int get_aoi(int &x, int &y, int &w, int &h){
		x = _settings->cameraSetting.aoiStartX.read();
		y = _settings->cameraSetting.aoiStartY.read();
		w = _settings->cameraSetting.aoiWidth.read();
		h = _settings->cameraSetting.aoiHeight.read();
		return 0;
	}

	// This function returns '-1' for internal trigger (ctmContinuous)
	// '0' for {'ctmOnHighLevel' & 'ctsDigIn0'} and '1' for {'ctmOnHighLevel' &
	// 'ctsDigIn1'}.
	inline int get_trigger_source(){
		TCameraTriggerSource source = _settings->cameraSetting.triggerSource.read();
		TCameraTriggerMode   mode   = _settings->cameraSetting.triggerMode.read();
		cout << _settings->cameraSetting.triggerMode.readS() << endl;
		if(mode == ctmContinuous)
			return -1;
		else if(mode == ctmOnHighLevel){
			if(source == ctsDigIn0)
				return 0;
			else if (source == ctsDigIn1)
				return 1;
		}
		return -2;
	}

	// See set_frame_delay_us(...) function.
	inline int get_frame_delay_us(){
		return _settings->cameraSetting.frameDelay_us.read();
	}

	// See get_image_request_timeout(...) function.
	inline int get_image_request_timeout(){
		return _settings->cameraSetting.imageRequestTimeout_ms.read();
	}
	
	// This function returns 'true' if the auto exposure mode is set.
	inline bool get_exposure_mode(){
		return _settings->cameraSetting.autoExposeControl.read() == aecOn;
	}

	// If the exposure mode is set to manual, this returns the user set value.
	// Otherwise it will return a negative value.
	inline double get_exposure_us(){
		if(_settings->cameraSetting.autoExposeControl.read() == aecOff)
			return _settings->cameraSetting.expose_us.read();
		else
			return -1;
	}

	// This function returns 'true' if the auto gain mode is set.
	inline bool get_gain_mode(){
		return _settings->cameraSetting.autoGainControl.read() == agcOn;
	}

	// This function returns the pixel clock rate in KHz.
	inline double get_pixel_clock_rate(){
		switch(_settings->cameraSetting.pixelClock_KHz.read()){
			case cpc57600KHz:
				return 57.6;
			case cpc50000KHz:
				return 50.0;
			case cpc40000KHz:
				return 40.0;
			case cpc37600KHz:
				return 37.6;
			case cpc32000KHz:
				return 32.0;
			case cpc27000KHz:
				return 27.0;
			case cpc24540KHz:
				return 24.54;
			case cpc24000KHz:
				return 24.0;
			case cpc20000KHz:
				return 20.0;
			case cpc13500KHz:
				return 13.5;
			case cpc12000KHz:
				return 12.0;
			case cpc10000KHz:
				return 10.0;
			case cpc8000KHz:
				return 8.0;
			case cpc6000KHz:
				return 6.0;
			default:
				return 0.0;
		}
	}
	
	// If the gain mode is set to manual, this return the user set value.
	// Otherwise it will return a negative value.
	inline double get_gain_dB(){
		if(_settings->cameraSetting.autoGainControl.read() == agcOff)
			return _settings->cameraSetting.gain_dB.read();
		else
			return -1;
	}

	// See function set_hdr_mode(...)
	inline bool get_hdr_mode(){
		HDRControl &hdr = _settings->cameraSetting.getHDRControl();
		if(!hdr.isAvailable())
			return false;
		else
			return hdr.HDREnable.read() == bTrue;
	}
};


#endif

