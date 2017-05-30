// *************************************** //
// Author : Salih Tolga Ozaslan            //
// Year   : June 2014                      //
//                                         //

#include "bluefox_camera.hh"
#include <iostream>
#include <mvIMPACT_CPP/mvIMPACT_acquire.h>

using namespace std;

// ### Coloring needs more work, but not urgent.
#define COUT		   cout << "\033[1;32m"
#define ENDL		   "\033[0m" << endl
//#define COLOR(x)	   "\033[1;32m" << x << "\033[0m"
//#define COUT_COLOR(x)  cout << "\033[1;34m" << x << "\033[0m"

int BlueFoxCamera::print_available_devices(bool print_detailed, bool print_invisible){
  int num_devices = _devMgr.deviceCount();

  COUT << "============================================================================" << ENDL;
  cout << num_devices << " devices are found : " << endl;
  cout << "----------------------------------------" << endl;
  for(int i = 0 ; i < num_devices ; i++)
    _print_device_details(_devMgr[i], i, print_detailed, print_invisible);
  COUT << "============================================================================" << ENDL;
  return 0;
}

int BlueFoxCamera::_print_device_details(Device *pDev, int idx, bool print_detailed, bool print_invisible){
  if(pDev == NULL)
    return -1;

  COUT << "Device[" << idx << "] : " << ENDL;
  cout << "-- Is in Use : " << (pDev->isInUse() ? "TRUE" : "FALSE")
    << " | Is Open : " << (pDev->isOpen() ? "TRUE" : "FALSE") << endl; 
  cout << "-- ID : " << pDev->deviceID.readS() 
    << " | Product : " << pDev->product.readS() 
    << " | Family : " << pDev->family.readS() 
    << " | Class : " << pDev->deviceClass.readS() << endl; 
  cout << "-- Version : " << pDev->deviceVersion.readS()
    << " | Serial : " << pDev->serial.readS()
    << " | Interface Layout : " << pDev->interfaceLayout.readS()
    << " | State : " << pDev->state.readS() << endl;
  cout << "-- Firmware Version : " << pDev->firmwareVersion.readS()
    << " | Granted Access : " << (pDev->grantedAccess.isValid() ? pDev->grantedAccess.readS() : "Property not supported") << endl;
  cout << "-- Acquisition Start/Stop Behaviour : " << pDev->acquisitionStartStopBehaviour.readS() << endl;
  cout << "-- Capabilities : " << pDev->capabilities.readS() << endl;
  cout << "-- Custom Data Directory : \"" << pDev->customDataDirectory.readS() << "\"" 
    << " | Default Request Count : " << pDev->defaultRequestCount.readS() << endl;
  cout << "-- Desired Access : " << (pDev->desiredAccess.isValid() ? pDev->desiredAccess.readS() : "Property not supported")
    << " | Result Queue Count : " << pDev->resultQueueCount.readS() << endl;

  if(print_detailed){
    bool is_open = pDev->isOpen();
    DeviceComponentLocator locator(pDev, dltSetting, "Base");
    Component  c = ComponentIterator(locator.searchbase_id()).firstChild();
    _print_component_list(c, 0, print_invisible);
    locator = DeviceComponentLocator(pDev, dltRequest);
    c = ComponentIterator(locator.searchbase_id()).firstChild();
    _print_component_list(c, 0, print_invisible);
    locator = DeviceComponentLocator(pDev, dltSystemSettings);
    c = ComponentIterator(locator.searchbase_id()).firstChild();
    _print_component_list(c, 0, print_invisible);
    locator = DeviceComponentLocator(pDev, dltInfo);
    c = ComponentIterator(locator.searchbase_id()).firstChild();
    _print_component_list(c, 0, print_invisible);
    if(is_open == false)
      pDev->close();
  }

  return 0;
}

int BlueFoxCamera::_print_component_list(Component &c, int depth, bool print_invisible){
  string tab;
  for(int i = 0 ; i < depth ; i++)
    tab += "--";
  tab += " ";

  while(true){
    if(!c.isValid())
      break;
    else if(c.isList()){
      cout << tab << c.name() << " : " << endl;
      Component _c = ((ComponentList)c).firstChild();
      _print_component_list(_c ,depth + 1, print_invisible);
    } else if(c.isMeth()){
      Method m = (Method)c;
      cout << tab << m.name() << " : " << endl;
    } else if(c.isProp()){
      Property p = (Property)c;
      if(p.isVisible() || print_invisible)
        cout << tab << p.name() << " : " << p.readSArray() << " (" << p.flagsAsString() << ")" <<  endl;
    }
    c = c.nextSibling();
  }
  return 0;
}

//$$$

// Constructor sets all of the internal variables to their default values.
BlueFoxCamera::BlueFoxCamera(){
  _pDev        = NULL;
  _settings    = NULL;
  _fi          = NULL;
  _stat        = NULL;
  _request     = NULL;
  _irc         = NULL;
  _timeout_ms  = 0;
  _device_initialized = false;
}

int BlueFoxCamera::open(int idx) {
  // First,  close the open camera
  this->close();

  // Check if there are available devices
  int num_devices = _devMgr.deviceCount(); 

  if(idx < 0){
    for(int i = 0 ; i < num_devices ; i++)
      if(this->open(i) == 0)
        return 0;
    return -1;
  }

  // If possible both assign the new device and open it.
  if( num_devices == 0 ) { 
    cout << "*** No device found! Unable to continue!" << endl; 
    return -1; 
  } else if (idx >=  num_devices || idx < 0 ) {
    cout << "*** Given index (" << idx << ") is not valid. Valid range is [0, " << num_devices - 1 << "]" << endl;
    return -1;
  } else {
    _pDev = _devMgr[idx];
    if(_pDev->isInUse() == true){
      cout << "*** Device with the id = " << idx << " is in use by another process." << endl;
      cout << "*** Unable to continue." << endl;
      _pDev = NULL;
      return -1;
    } else {
      try {
        _pDev->open();
        _device_idx = idx;
        _device_serial = "";
        if(_initialize_device() < 0)
          return -1;
        else
          return 0;
      } catch(mvIMPACT::acquire::ImpactAcquireException &e) {
        cout << "*** An error has occured while opening the device with index = " << idx << endl;
        cout << "*** Error code  : " << e.getErrorCode() << endl;
        cout << "*** Description : " << e.getErrorCodeAsString() << endl;
        return -1;
      }
    }
  }
}

int BlueFoxCamera::open(string serial) {
  // First,  close the open camera
  this->close();

  _pDev = _devMgr.getDeviceBySerial(serial.c_str());
  if(_pDev == NULL){
    cout << "*** Cannot find the device with the serial number : " << serial << endl;
    return -1;
  } else if(_pDev->isInUse() == true){
    cout << "*** Device with the serial = " << serial << " is in use by another process." << endl;
    cout << "*** Unable to continue." << endl;
    _pDev = NULL;
    return -1;
  } else {
    try {
      _pDev->open();
      _device_serial = serial;
      _device_idx    = -1;
      if(_initialize_device() < 0)
        return -1;
      else
        return 0;
    } catch(mvIMPACT::acquire::ImpactAcquireException &e) {
      cout << "*** An error has occured while opening the device with serial = " << serial << endl;
      cout << "*** Error code  : " << e.getErrorCode() << endl;
      cout << "*** Description : " << e.getErrorCodeAsString() << endl;
      return -1;
    }
  }
}

int BlueFoxCamera::close(){
  if(_pDev != NULL)
    _pDev->close();
  _pDev = NULL;

  if(_fi != NULL)
    delete _fi;
  _fi = NULL;
  if(_stat != NULL)
    delete _stat;
  _stat = NULL;
  if(_irc != NULL)
    delete _irc;
  _irc = NULL;
  if(_settings != NULL)
    delete _settings;
  _settings = NULL;

  _device_initialized = false;

  return 0;
}

int BlueFoxCamera::_get_bits_per_pixel(){
  switch( _request->imagePixelFormat.read()){
    case ibpfRGBx888Planar:
    case ibpfRGB888Packed:
      return 24;
    case ibpfRaw:
    case ibpfMono8:
    default:
      return 8;
    case ibpfMono10:
      return 10;
    case ibpfMono12:
      return 12;
    case ibpfMono14:
      return 14;
    case ibpfYUV422Packed:
    case ibpfYUV422Planar:
    case ibpfMono16:
      return 16;
    case ibpfRGB101010Packed:
      return 30;
    case ibpfMono32:
    case ibpfRGBx888Packed:
      return 32;
    case ibpfRGB121212Packed:
      return 36;
    case ibpfRGB141414Packed:
      return 42;
    case ibpfRGB161616Packed:
      return 48;
  }
}

int BlueFoxCamera::_initialize_device(){
  if(_pDev == NULL || !_pDev->isOpen()){
    cout << "*** Device not ready for data acquisition. Try opening the device with a valid index." << endl;
    cout << "*** Devive pointer : " << hex << _pDev << endl;
    if(_pDev != NULL)
      cout << "*** device.isOpen() : " << _pDev->isOpen() << endl;
    return -1;
  }

  if(_fi != NULL)
    delete _fi;
  _fi = new  FunctionInterface(_pDev);

  if(_irc != NULL)
    delete _irc;
  _irc = new ImageRequestControl(_pDev);

  if(_stat != NULL)
    delete _stat;
  _stat = new Statistics(_pDev);

  if(_settings != NULL)
    delete _settings;
  _settings = new SettingsBlueFOX(_pDev);

  TDMR_ERROR result = DMR_NO_ERROR;
  while((result = static_cast<TDMR_ERROR>(_fi->imageRequestSingle())) == DMR_NO_ERROR);
  if(result != DEV_NO_FREE_REQUEST_AVAILABLE){
    cout << "*** Image request resulted in unexpected error : " << result << endl;
    cout << "*** Error description : " << ImpactAcquireException::getErrorCodeAsString(result) << endl;
    return -1;
  }

  // Start the acquisition manually if necessary
  if(_pDev->acquisitionStartStopBehaviour.read() == assbUser){
    if(( result = static_cast<TDMR_ERROR>(_fi->acquisitionStart() ) ) != DMR_NO_ERROR){
      cout << "*** Attempt to start image acquisition resulted in the error : " << result << endl;
      cout << "*** Error description : " << ImpactAcquireException::getErrorCodeAsString(result) << endl;
      return -1;
    }
  }

  _device_initialized = true;

  return 0;
}

int BlueFoxCamera::_copy_image(cv::Mat &frame){
  int pixel_pitch = _request->imagePixelPitch.read();
  int width, height;

  width  = _request->imageWidth.read();
  height = _request->imageHeight.read();

  //cout << "width  : " << width  << endl;
  //cout << "height : " << height << endl;
  //cout << "pixel_pitch : " << pixel_pitch << endl;
  //namedWindow("win");

  frame = cv::Mat(height, width, CV_8UC(pixel_pitch));

  memcpy(frame.data, _request->imageData.read(), _request->imageSize.read());

  //imshow("win", frame);
  //waitKey(1);

  return 0;
}

int BlueFoxCamera::print_stats(){
  if(_device_idx != -1)
    COUT << "Device[" <<  _device_idx << "] statistics :" << ENDL;
  else
    COUT << "Device[" <<  _device_serial << "] statistics :" << ENDL;
  cout << "--- FPS : " << _stat->framesPerSecond.read();
  cout << " | Capture time : " << _stat->captureTime_s.read() << endl;
  cout << "--- Image proc. time : " << _stat->imageProcTime_s.read();
  cout << " | Retransmit count : " << _stat->retransmitCount.read() << endl;
  cout << "--- Timedout count : " << _stat->timedOutRequestsCount.read();
  cout << " | Missing data avg. : " << _stat->missingDataAverage_pc.read() << "%" << endl;
  return 0;
}

#include <ros/ros.h>

int BlueFoxCamera::grab_frame(cv::Mat &frame, bool printstats){
  static int requestNr;
  if(_device_initialized == false)
    if(_initialize_device() < 0){
      cout << "*** Couldn't initialize the device." << endl;
      return -1;
    }

  requestNr = _fi->imageRequestWaitFor(0);

  //cout << "Req count : " << _fi->requestCount() << endl;

  if(_fi->isRequestNrValid(requestNr)){
    _request = _fi->getRequest(requestNr);
  } else {
    _request = NULL;
  }

  if(_request == NULL){
    /*
       cout << "*** Image request failed with error : " << requestNr  << endl;
       cout << "*** Error description : " << ImpactAcquireException::getErrorCodeAsString(requestNr) << endl;
       cout << "*** This might be due to (1) small timeout duration (2) device is unplugged" << endl;
       cout << "*** (3) exposure time is too long" << endl;
     */
    return -1;
  }

  if(_request->isOK()){

    int bufferSize, bufferAlignment;
    _fi->getCurrentCaptureBufferLayout(*_irc, bufferSize, bufferAlignment);

    _copy_image(frame);

    _request->unlock();
    _fi->imageRequestSingle();

  } else {
    if(_timeout_ms > 0){
      cout << "*** Image request failed with error : " << _request->requestResult.readS() << endl;
      cout << "*** This problem is usually due to the USB port." << endl;
      cout << "*** Try using a USB2 port instead of USB3." << endl;
      cout << "*** If running in the external trigger mode, slow trigger signals may cause this." << endl;
    }
    return -1;
  }


  if(printstats)
    print_stats();
  return 0;
}







