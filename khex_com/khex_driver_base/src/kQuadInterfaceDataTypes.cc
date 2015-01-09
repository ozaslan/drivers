/* Copyright KMel Robotics 2014. Must read KMEL_LICENSE.pdf for terms and conditions before use. */
#include "kQuadInterfaceDataTypes.hh"
#include "Timer.hh"
#include <list>
#include <vector>
#include <stdint.h>
#include <pthread.h>
#include <string.h>

using namespace std;

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Raw IMU
//_______________________________________________________
int ImuRawData::Parse(uint8_t * raw, int len, uint8_t type)
{ 
  int32_t * idata = (int32_t*)raw;
  
  tpc   = Timer::GetUnixTime();
  wxr1  = *idata++; wyr1  = *idata++; wzr1  = *idata++;
  wxr2  = *idata++; wyr2  = *idata++; wzr2  = *idata++;
  axr   = *idata++; ayr   = *idata++; azr   = *idata++;
  wxtr1 = *idata++; wytr1 = *idata++; wztr1 = *idata++;
  atr   = *idata++; idata++; //skip counter
  tuc   = *((uint32_t*)idata++);
  
  //Print();
  return 0;
}

int ImuRawData::Print()
{
  PRINT_INFO("IMU RAW: ("<<tpc<<" "<<tuc<<") wxyz=("<<wxr1<<" "<<wyr1<<" "<<wzr1<<") "
                    <<"wxyz2=("<<wxr2<<" "<<wyr2<<" "<<wzr2<<") "
                    <<"axyz=("<<axr<<" "<<ayr<<" "<<azr<<") "
                    <<"temps=("<<wxtr1<<" "<<wytr1<<" "<<wztr1<<" "<<atr<<")\n");
  
  return 0;
}

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Filtered IMU
//_______________________________________________________
int ImuFiltData::Parse(uint8_t * raw, int len, uint8_t type)
{
  if (type == 1)
  {
    float * fdata = (float *)raw;
    
    tpc    = Timer::GetUnixTime();        tuc    = *((uint32_t*)fdata++);
    roll   = *fdata++; pitch  = *fdata++; yaw    = *fdata++;
    wroll  = *fdata++; wpitch = *fdata++; wyaw   = *fdata++;      
    ax     = *fdata++; ay     = *fdata++; az     = *fdata++;
  }
  else if (type == 34)
  {
    tpc = Timer::GetUnixTime();
    tuc = *(uint32_t*)raw; raw += 4;
    
    id = *raw; ++raw;
    cntr = *raw; ++raw;
    roll = *(int16_t*)raw / 5000.; raw += 2;
    pitch = *(int16_t*)raw / 5000.; raw += 2;
    yaw = *(int16_t*)raw / 5000.; raw += 2;
    wroll = *(int16_t*)raw / 500.; raw += 2;
    wpitch = *(int16_t*)raw / 500.; raw += 2;
    wyaw = *(int16_t*)raw / 500.; raw += 2;
    ax = *(int16_t*)raw / 5000.; raw += 2;
    ay = *(int16_t*)raw / 5000.; raw += 2;
    az = *(int16_t*)raw / 5000.;
  }
  
  //Print();
  return 0;
}


int ImuFiltData::Print()
{
  PRINT_INFO("IMU [id="<<id<<"]: ("<<tpc<<" "<<tuc<<") rpy=("<<roll<<" "<<pitch<<" "<<yaw<<"), wrpy("
                     <<wroll<<" "<<wpitch<<" "<<wyaw<<"), acc("
                     <<ax<<" "<<ay<<" "<<az<<")\n");
  return 0;
}



//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Battery 
//_______________________________________________________
int BatteryData::Parse(uint8_t * raw, int len, uint8_t type)
{
  return 0;
}
int BatteryData::Print()
{
  return 0;
}

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// RC data
//_______________________________________________________
int RcData::Parse(uint8_t * raw, int len, uint8_t type)
{
  if (type == 5)
  {
    /*uint16_t * data2 = (uint16_t *)raw;
    uint32_t * t    = (uint32_t*)(&data2[8]);
    uint8_t  * id2  = (uint8_t*)(&data2[10]);
  
    for (int ii=0; ii<8; ii++)
      data[ii] = data2[ii];

    tpc     = Timer::GetUnixTime();
    tuc     = *t;
    id      = *id2;*/
  }
  else if (type == 42)
  {
    tpc = Timer::GetUnixTime();
    tuc = *(uint32_t*)raw; raw += 4;
    
    for (int ii = 0; ii < 12; ++ii)
    {
      data[ii] = *(uint16_t*)raw;
      raw += 2;
    }
    
    rcTimeouts = *(uint16_t*)raw;
  }
  
  //Print();
  return 0;
}
int RcData::Print()
{
  PRINT_INFO("RC ("<<tuc<<"): Timeouts: "<<rcTimeouts<<" Raw Data:" << 
                       data[0] <<" "<< data[1] <<" "<< data[2] <<  
                 " "<< data[3] <<" "<< data[4] <<" "<< data[5]<<
                 " "<< data[6] <<" "<< data[7] <<" "<< data[8]<<
                 " "<< data[9] <<" "<< data[10] <<" "<< data[11] <<"\n");
  return 0;
}

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// GPS NMEA
//_______________________________________________________
int GpsNmeaData::Parse(uint8_t * raw, int len, uint8_t type)
{
  id      = 0;
  tpc     = Timer::GetUnixTime();
  tuc     = *((uint32_t*)(raw+len-4));
  size    = len-4;
  memcpy(data,raw,len-4);
  
  /*
  printf("length = %d\n",len);
  
  for (int ii=0; ii<size; ii++)
    printf("%c",raw[ii]);
  printf("\n");
  */
  return 0;
}
int GpsNmeaData::Print()
{  
  PRINT_INFO("GPS: ("<<(int)id<<","<<tuc<<","<<tpc<<"): ");
  for (int ii=0; ii<size; ii++)
    printf("%c",data[ii]);
  printf("\n");
  return 0;
}

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// GPS Ublox
//_______________________________________________________
int GpsUbloxData::Parse(uint8_t * raw, int len, uint8_t type)
{
  tpc     = Timer::GetUnixTime();
  tuc     = *((uint32_t*)(raw+len-4));
  ubclass = raw[2];
  ubid    = raw[3];
  ublen   = raw[4]; ublen |= ((uint16_t)raw[5]) << 8;
  
  if (ublen > 512)
    return -1;
  
  memcpy(ubdata,raw+6,ublen);
  
  //printf("got ublox packet class %d, id %d, length %d, time %d\n",ubclass,ubid,ublen,tuc);
  
  return 0;
}
int GpsUbloxData::Print()
{
  return 0;
}


//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Quad Status
//_______________________________________________________
int QuadStatusData::Parse(uint8_t * raw, int len, uint8_t type)
{
  if (type == 5)
  {
    tpc = Timer::GetUnixTime();
    tuc = *((uint32_t*)raw); raw+=4;
    
    uint16_t * data2 = (uint16_t *)raw;
    voltage = *data2++ / 1000.0f;
    current = *data2++ / 1000.0f;
    
    raw+=4;
    id        = *raw++;
    state     = *raw++;
    autoCntr  = *raw++;
    rcCntr    = *raw++;
    nerr      = *raw++;
    lastError = *raw++;
    sigstren  = *raw++;
  }
  else if (type == 41)
  {
    tpc = Timer::GetUnixTime();
    tuc = *((uint32_t*)raw); raw += 4;
    
    voltage = *(uint16_t*)raw / 1000.; raw += 2;
    id = *raw; raw++;
    state = *raw; raw++;
    autoCntr = *(uint16_t*)raw; raw += 2;
    rcCntr = *(uint16_t*)raw;
  }
  
  //Print();
  
  return 0;
}
int QuadStatusData::Print()
{
  PRINT_INFO("Quad Status: t:" <<tuc<<", id: "<<id<<", state: "<<state<<", voltage: "<<voltage<<
           ", autoCntr: "<<autoCntr<<", rcCntr "<<rcCntr<<"\n");
  return 0;
}

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Servo Data
//_______________________________________________________
int ServoData::Parse(uint8_t * raw, int len, uint8_t type)
{
  return 0;
}

int ServoData::Print()
{
  PRINT_INFO("Servo Data: " << cntr << " "<<tuc<<" "<<angle<<"\n");
  return 0;
}


//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Pressure Mag Data
//_______________________________________________________
int PressureMagData::Parse(uint8_t * raw, int len, uint8_t type)
{  
  if (type == 33)
  {
    /*tpc = Timer::GetUnixTime();
    id = *raw++;
    
    uint32_t * pdata = (uint32_t*)raw;
    
    tucp0 = *pdata++;
    tucp1 = *pdata++;
    tucm  = *pdata++;
    
    pressure[0] = *pdata++;
    pressure[1] = *pdata++;
    
    int16_t * pdata2 = (int16_t*)pdata;
    
    temperature[0] = *pdata2++;
    temperature[1] = *pdata2++;
    
    mx = *pdata2++;
    my = *pdata2++;
    mz = *pdata2++;*/
  }
  else if (type == 35)
  {
    /*tpc = Timer::GetUnixTime();
    id = *raw++;
    
    uint32_t * pdata = (uint32_t*)raw;
    
    tucp0 = *pdata++;
    tucp1 = tucp0;
    tucm  = tucp0;
    
    int16_t * pdata2 = (int16_t*)pdata;
    
    pressure[0] = *pdata2++ + 100000;
    pressure[1] = *pdata2++ + 100000;
    
    temperature[0] = *pdata2++ / 100.0; //scaling
    temperature[1] = *pdata2++ / 100.0;
    
    mx = *pdata2++;
    my = *pdata2++;
    mz = *pdata2++;*/
  }
  else if (type == 40)
  {
    tpc = Timer::GetUnixTime();
    tuc = *((uint32_t*)raw); raw += 4;
    
    pressure = *(int16_t*)raw + 100000; raw += 2;
    temperature = *(int16_t*)raw / 100.; raw += 2;
    press_time = *(uint32_t*)raw; raw += 4;
    baro_zpos = *(float*)raw; raw += 4;
    
    mx = *(int16_t*)raw; raw += 2;
    my = *(int16_t*)raw; raw += 2;
    mz = *(int16_t*)raw; raw += 2;
    mag_time = *(int32_t*)raw;
  }

  //Print();
  return 0;
}

int PressureMagData::Print()
{
  PRINT_INFO("Pressure : "<<press_time<<" "<<pressure<<" "<<" "<<temperature<<
             " "<<baro_zpos<<" || Mag Data: " <<mag_time<<" "<<mx<<" "<<my<<" "<<mz<<" "<<"\n");
  return 0;
}


//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Zigbee Rx status
//_______________________________________________________
int ZigBeeRxStatus::Parse(uint8_t * raw, int len, uint8_t type)
{
  //printf("got zigbee rx status!!\r\n");
 
  tpc = Timer::GetUnixTime();
  uint32_t * pdata = (uint32_t*)raw;
  tuc = *pdata++;
  err = *pdata++;
  
  raw = (uint8_t*)pdata;
  
  kid = *raw++;
  len = *raw++;
  lqi = *raw++;
  ed  = *raw++;
  status = *raw++;
  chan = *raw++;
 
  //Print();
  return 0;
}

int ZigBeeRxStatus::Print()
{
  PRINT_INFO("ZigbeeRxStatus : tuc: "<<tuc<<", ed: "<<(int)ed<<", chan: "<<(int)chan<<", err: "<<(int)err<<", kid: "<<(int)kid<<"\n");
  return 0;
}


//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Motor status
//_______________________________________________________
int MotorStatusData::Parse(uint8_t * raw, int len, uint8_t type)
{
  tpc = Timer::GetUnixTime();
  uint32_t * pdata = (uint32_t*)raw;
  tuc = *pdata++;
  
  uint8_t * pdata2 = (uint8_t*)pdata;
  id = *pdata2++;
  
  uint16_t * pdata3 = (uint16_t*)pdata2;
  
  memcpy(crpm,pdata3,4*sizeof(uint16_t)); pdata3 += 4;
  memcpy(arpm,pdata3,4*sizeof(uint16_t)); pdata3 += 4;
  
  pdata2 = (uint8_t*)pdata3;
  memcpy(mstat,pdata2,4);
 
  //Print();
  return 0;
}

int MotorStatusData::Print()
{
  PRINT_INFO("MotorStatus : tuc: "<<tuc<<", commanded: ("<<(int)crpm[0]<<" "<<(int)crpm[1]<<" "<<(int)crpm[2]<<" "<<(int)crpm[3]<<" )"<<
                                         ", actual: ("<<(int)arpm[0]<<" "<<(int)arpm[1]<<" "<<(int)arpm[2]<<" "<<(int)arpm[3]<<" )"<<
                                         ", state: ("<<(int)mstat[0]<<" "<<(int)mstat[1]<<" "<<(int)mstat[2]<<" "<<(int)mstat[3]<<" )\n");
  return 0;
}


//NOTE to be tested
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// GPS from nanohex
//_______________________________________________________
int GpsData::Parse(uint8_t * raw, int len, uint8_t type)
{  
  tpc = Timer::GetUnixTime();
  tuc = *(uint32_t*)raw; raw += 4;
  
  x = *(float*)raw; raw += 4;
  y = *(float*)raw; raw += 4;
  z = *(float*)raw; raw += 4;
  
  xvel = *(float*)raw; raw += 4;
  yvel = *(float*)raw; raw += 4;
  zvel = *(float*)raw; raw += 4;
  
  gpsFix = *raw; ++raw;
  hAcc = *(uint32_t*)raw; raw += 4;
  numSV = *raw; ++raw;
  sAcc = *(uint32_t*)raw; raw += 4;
  
  fixTime = *(uint32_t*)raw; raw += 4;
  
  lon0 = *(int32_t*)raw; raw += 4;
  lat0 = *(uint32_t*)raw; raw += 4;
  set0 = *raw; ++raw;
  
  diffSoln = *raw; ++raw;
  
  lat = *(int32_t*)raw / 10000000.; raw += 4;
  lon = *(int32_t*)raw / 10000000.; raw += 4;
  
  nreceived1 = *(uint32_t*)raw; raw += 4;
  
  voltage = *(uint16_t*)raw / 1000.; raw += 2;
  
  posCmds_x = *(int16_t*)raw; raw += 2;
  posCmds_y = *(int16_t*)raw; raw += 2;
  posCmds_z = *(int16_t*)raw; raw += 2;
  
  filter_accel_zpos = *(int16_t*)raw; raw += 2;
  filter_baro_zvel = *(int16_t*)raw / 100.; raw += 2;
  
  azw = *(int16_t*)raw / 5000.; raw += 2;
  
  ux_int = *(int16_t*)raw; raw += 2;
  uy_int = *(int16_t*)raw; raw += 2;
  
  vehicleId = *raw; ++raw;
  mx = *(int16_t*)raw; raw += 2;
  my = *(int16_t*)raw; raw += 2;
  mz = *(int16_t*)raw;
  
  //Print();
  return 0;
}

int GpsData::Print()
{
  
  PRINT_INFO("GpsData: tuc: "<<tuc<<" gpsFix: "<<gpsFix<<" numSV: "<<numSV<<" lat: "<<lat<<" lon: "<<lon<<
             " baro zpos: "<<filter_baro_zpos<<"\n");
  
  return 0;
}
