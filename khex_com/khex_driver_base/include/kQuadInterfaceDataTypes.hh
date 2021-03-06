/* Copyright KMel Robotics 2014. Must read KMEL_LICENSE.pdf for terms and conditions before use. */
/* LOG
  10/10/2012:
  - initial release
  
  11/1/2012:
  - added forgoten mutex locking and msg queue length limiting in the push functions for the data container
 
 
  12/21/2012
  - added more descriptions in the data structures
  - general clean-up
*/

#ifndef KQUAD_INTERFACE_DATA_TYPES_HH
#define KQUAD_INTERFACE_DATA_TYPES_HH

#include <string>
#include <stdint.h>
#include <list>
#include <iostream>
#include <fstream>
#include <sstream>

#define DEFAULT_MSG_QUEUE_LENGTH 10
#define MAX_MSG_QUEUE_LENGTH 1000

#define PRINT_INFO( msg )    { std::ostringstream msgStream; msgStream <<"["<<__FUNCTION__<<" : "<<__LINE__<<"] : "<< msg << std::flush; std::cout<<msgStream.str(); }
#define PRINT_INFO_RAW( msg )    { std::ostringstream msgStream; msgStream <<"["<<__FUNCTION__<<" : "<<__LINE__<<"] : "<< msg << std::flush; std::cout<<msgStream.str(); }
#define PRINT_WARNING( msg ) { std::ostringstream msgStream; msgStream <<"["<<__FUNCTION__<<" : "<<__LINE__<<"] : "<< msg << std::flush; std::cout<<msgStream.str(); }
#define PRINT_ERROR( msg )   { std::ostringstream msgStream; msgStream <<"["<<__FUNCTION__<<" : "<<__LINE__<<"] : "<< msg << std::flush; std::cout<<msgStream.str(); }

using namespace std;

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Data container
//_______________________________________________________
template <class T>
class kDataContainer
{
  public: 
    kDataContainer() { queueLen = DEFAULT_MSG_QUEUE_LENGTH; pthread_mutex_init( &(this->mutex) , NULL ); };
   ~kDataContainer() { pthread_mutex_destroy(&(this->mutex)); };
    int PushData(uint8_t * raw, uint32_t len, uint8_t type = 0);
    int GetData(list<T> & data, uint32_t sizer = 0);
    int SetQueueLength(uint32_t len);
    int PrintData(list<T> & data);
  
  protected:
    void LockMutex();
    void UnlockMutex();
    list<T> data;
    uint32_t queueLen;
    pthread_mutex_t mutex;
};

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Lock / unlock the mutex
//_______________________________________________________
template <class T>
void kDataContainer<T>::LockMutex()   { pthread_mutex_lock( &this->mutex );   }
template <class T>
void kDataContainer<T>::UnlockMutex() { pthread_mutex_unlock( &this->mutex ); }

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Push data into the container
//_______________________________________________________
template <class T>
int kDataContainer<T>::PushData(uint8_t * raw, uint32_t len, uint8_t type)
{
  T dat;
  if (dat.Parse(raw,len,type) == 0)
  {
    this->LockMutex();
    this->data.push_back(dat);
    while (this->data.size() > this->queueLen)
      this->data.pop_front();
    this->UnlockMutex();
  }
  return 0;
}

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Return data to the user. sizer is size requested
//_______________________________________________________
template <class T>
int kDataContainer<T>::GetData(list<T> & data, uint32_t sizer)
{
  int ret = 0;
  data.clear();
  this->LockMutex();
 
  uint32_t sizea = this->data.size(); //available size
  if (sizea < 1)
  {
    ret = -1;
  }
  else if (sizer == 0)
  {
    data = this->data;
    this->data.clear();
    ret = sizea;
  }
  else if (sizer > 0)
  {
    uint32_t n = sizer < sizea ? sizer : sizea;
    ret = n;
    while(n--) {
      data.push_back(this->data.front());
      this->data.pop_front();
    } 
  } 
  
  this->UnlockMutex();
  return ret;
}


//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Set message queue length
//_______________________________________________________
template <class T>
int kDataContainer<T>::SetQueueLength(uint32_t len)
{
  if ((len < 1) || (len > MAX_MSG_QUEUE_LENGTH))
    return -1;
  
  this->queueLen = len;
  return 0;
}

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Print data
//_______________________________________________________
template <class T>
int kDataContainer<T>::PrintData(list<T> & data)
{
  int size = data.size();
  typename list<T>::iterator it = data.begin();
  while(size--)
  {
    T & dat = *it++;
    dat.Print();
  }
  return 0; 
}

struct ImuRawData
{
  uint32_t id;      //id of the sender
  double   tpc;     //unix time
  uint32_t tpci;    //unix time (0.1 msec), offset
  uint32_t tuc;     //micro time (0.1 msec)
  
  int32_t wxr1;     //raw x1 rate
  int32_t wyr1;     //raw y1 rate
  int32_t wzr1;     //raw z1 rate
  
  int32_t wxtr1;    //raw x1 rate temperature
  int32_t wytr1;    //raw y1 rate temperature
  int32_t wztr1;    //raw z1 rate temperature
  
  int32_t wxr2;     //raw x2 rate
  int32_t wyr2;     //raw y2 rate
  int32_t wzr2;     //raw z2 rate
  
  int32_t axr;      //raw x acceleration 
  int32_t ayr;      //raw y acceleration
  int32_t azr;      //raw z acceleration
  int32_t atr;      //raw accelerometer temperature
  
  int Print();
  int Parse(uint8_t * raw, int len, uint8_t type = 0);
};

struct ImuFiltData
{
  uint32_t id;        //id of the sender
  double   tpc;       //unix time
  uint32_t tuc;       //micro time (0.1 msec)
  uint8_t  cntr;

  float    roll;      //radians
  float    pitch;     //radians
  float    yaw;       //radians

  float    wroll;     //roll rate,  radians/sec
  float    wpitch;    //pitch rate, radians/sec
  float    wyaw;      //yaw rate,   radians/sec

  float    ax;        //x-acceleration, Gs
  float    ay;        //y-acceleration, Gs
  float    az;        //z-acceleration, Gs
  
  int Print();
  int Parse(uint8_t * raw, int len, uint8_t type = 0);
};

struct BatteryData
{
  uint32_t id;        //id of the sender
  double   tpc;       //unix time
  uint32_t tpci;      //unix time (0.1 msec), offset
  uint32_t tuc;       //micro time (0.1 msec)

  float voltage1;     //battery1 voltage (Volts)
  float voltage2;     //battery2 voltage (Volts)
  float voltage3;     //power supply voltage (Volts)
  float current1;     //battery1 current (Amps) - may not be available 
  float current2;     //battery2 current (Amps) - may not be available 
  float current3;     //power supply current (Amps) - may not be available 
  
  int Print();
  int Parse(uint8_t * raw, int len, uint8_t type = 0);
};

struct RcData
{
  //uint32_t id;        //id of the sender
  double   tpc;       //unix time
  //uint32_t tpci;      //unix time (0.1 msec), offset
  uint32_t tuc;       //micro time (0.1 msec)
  uint16_t data[12];   //up to 12 RC channels - 0 if not available
  uint16_t rcTimeouts;
  
  int Print();
  int Parse(uint8_t * raw, int len, uint8_t type = 0);
};

struct GpsNmeaData
{
  uint32_t id;        //id of the sender
  double   tpc;       //unix time
  uint32_t tpci;      //unix time (0.1 msec), offset
  uint32_t tuc;       //micro time (0.1 msec)
  uint8_t  gid;       //gps id (in case there are several)
  uint16_t size;      //size of the string
  uint8_t  data[128]; //raw gps string
  
  int Print();
  int Parse(uint8_t * raw, int len, uint8_t type = 0);
};

struct GpsUbloxData
{
  uint32_t id;       //id of the sender
  double   tpc;      //unix time
  uint32_t tpci;     //unix time (0.1 msec), offset
  uint32_t tuc;      //micro time (0.1 msec)
  uint8_t  gid;      //gps id (in case there are several)
  
  uint8_t  ubclass;  //ublox message class
  uint8_t  ubid;     //ublox message id
  uint16_t ublen;    //ublox message length
  uint8_t  ubdata[512];  //ublox message data
  
  int Print();
  int Parse(uint8_t * raw, int len, uint8_t type = 0);
};

struct QuadStatusData
{
  uint32_t id;        //id of the sender
  uint32_t tuc;       //micro time (0.1 msec)
  double   tpc;       //unix time : pc timestamp
  uint32_t tpci;      //unix time (0.1 msec), offset
  float    voltage;   //battery voltage (Volts)
  float    current;   //total current (Amperes)
  uint32_t state;     //state machine state
  uint32_t autoCntr;  //number of autonomous commands received (uart or zigbee)
  uint32_t rcCntr;    //number of RC packets received from Spektrum
  uint32_t nerr;      //number of checksum errors
  uint32_t lastError; //reserved
  uint32_t sigstren;  //signal strength of last zigbee packet
  
  int Print();
  int Parse(uint8_t * raw, int len, uint8_t type = 0);
};

struct ServoData
{
  uint32_t id;     //id of the sender
  double   tpc;    //unix time
  uint32_t tpci;   //unix time (0.1 msec), offset
  uint32_t tuc;    //micro time (0.1 msec)
  uint32_t sid;    //id of the servo
  uint32_t cntr;   //number of packets sent
  float    angle;  //angle of the servo
  
  int Print();
  int Parse(uint8_t * raw, int len, uint8_t type = 0);
};

struct PressureMagData
{
  double   tpc;      //unix time
  uint32_t tuc;      //micro time (0.1 msec)
  
  float pressure;       //pascals
  float temperature;    //deg celcius
  uint32_t press_time;
  float baro_zpos;
  
  float mx,my,mz;       //magnetic field strength
  uint32_t mag_time;
  
  int Print();
  int Parse(uint8_t * raw, int len, uint8_t type = 0);
};

struct ZigBeeRxStatus
{
  uint32_t id;       //id of the sender
  double   tpc;      //unix time
  uint32_t tpci;     //unix time (0.1 msec), offset
  uint32_t tuc;
  
  uint32_t err;      //number of checksum errors
  
  uint8_t  kid;      //id of the kbee
  uint8_t  len;      //length of previous received packet
  uint8_t  lqi;      //link quality indicator (LQI)
  uint8_t  ed;       //received signal strength (ED)
  
  uint8_t  status;   //status byte from the zigbee transceiver
  uint8_t  chan;     //current channel
  
  int Print();
  int Parse(uint8_t * raw, int len, uint8_t type = 0);
};

struct MotorStatusData
{
  uint32_t id;       //id of the sender
  double   tpc;      //unix time
  uint32_t tpci;     //unix time (0.1 msec), offset
  uint32_t tuc;
  
  int16_t crpm[4];   //commanded rpm
  int16_t arpm[4];   //actual rpm
  uint8_t mstat[4];  //motor status
  
  int Print();
  int Parse(uint8_t * raw, int len, uint8_t type = 0);
};

struct GpsData
{
  uint32_t id;      //id of the sender
  double   tpc;     //unix time
  uint32_t tpci;    //unix time (0.1 msec), offset
  uint32_t tuc;

  float    x;       
  float    y;       
  float    z;       

  float    xvel;
  float    yvel;
  float    zvel;

  uint8_t  gpsFix;
  uint32_t hAcc;
  uint8_t  numSV;
  uint32_t sAcc;

  uint32_t fixTime;

  int32_t  lon0;
  int32_t  lat0;
  uint8_t  set0;

  uint8_t  diffSoln;

  double   lat;
  double   lon;

  uint32_t nreceived1;

  float    voltage;

  int16_t  posCmds_x;
  int16_t  posCmds_y;
  int16_t  posCmds_z;

  int16_t  filter_accel_zpos;
  float    filter_baro_zpos;

  int16_t  filter_accel_zvel;
  float    filter_baro_zvel;

  float    azw;

  int16_t  ux_int;
  int16_t  uy_int;

  uint8_t  vehicleId;

  int16_t  mx;
  int16_t  my;
  int16_t  mz;

  int Print();
  int Parse(uint8_t * raw, int len, uint8_t type = 0);
};


#endif