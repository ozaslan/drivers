/* Copyright KMel Robotics 2014. Must read KMEL_LICENSE.pdf for terms and conditions before use. */
#include "kQuadInterface.hh"
#include <mex.h>
#include <math.h>
#include <string>
#include "Timer.hh"
#include <iostream>
#include <fstream>

#define CHECK_CONNECTION if (connected != 1) { mexErrMsgTxt("kQuadInterfaceAPI : not connected"); }

using namespace std;

kQuadInterface kqi;
int connected = 0;

uint8_t temp[256];


void mexExit(void)
{
  printf("Exiting kQuadInterfaceAPI.\n");
  kqi.StopSendThread();
  kqi.Disconnect();
}


void CreateEmptyOutput(mxArray * plhs[])
{
  plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
  plhs[1] = mxCreateDoubleMatrix(0,0,mxREAL);
  plhs[2] = mxCreateDoubleMatrix(0,0,mxREAL);
}

void CreateUInt32Scalar(mxArray ** p,uint32_t t)
{
  int ndims = 2;
  int dims[] = {1,1};
  *p = mxCreateNumericArray(ndims,dims,mxUINT32_CLASS,mxREAL);
  memcpy(mxGetData(*p),&t,sizeof(uint32_t));
}

mxArray * CreateUInt32Matrix(int m, int n)
{
  int ndims = 2;
  int dims[] = {m,n};
  return mxCreateNumericArray(ndims,dims,mxUINT32_CLASS,mxREAL);
}

int CreateOutputArrayUint8(mxArray ** out, uint8_t * data, int len)
{
  if (len > 0)
  {
    int ndims  = 2;
    int dims[] = {1,len};
    *out       = mxCreateNumericArray(ndims,dims,mxUINT8_CLASS,mxREAL);
    
    uint8_t * pdata = (uint8_t*)mxGetData(*out);
    memcpy(pdata,temp,len);
    return 0;
  }
  else
  {
    return -1;
    //*out = mxCreateDoubleMatrix(0,0,mxREAL);
  }
}



void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{

  // Get input arguments
  if (nrhs == 0)
    mexErrMsgTxt("Need input argument");


  const int BUFLEN = 256;
  char buf[BUFLEN];

  if (mxGetString(prhs[0], buf, BUFLEN) != 0)
    mexErrMsgTxt("could not read string.");
  
  if (strcasecmp(buf, "connect") == 0)
  {
    if (nrhs < 3)
      mexErrMsgTxt("connect option needs device and baud rate");

    if (mxGetString(prhs[1], buf, BUFLEN) != 0)
      mexErrMsgTxt("could not read string (device)");

    int baud = (int)mxGetPr(prhs[2])[0];


    if (kqi.IsConnected())
    {
      plhs[0] = mxCreateDoubleScalar(1);
      return;
    }

    if (kqi.Connect(buf,baud))
      mexErrMsgTxt("could not connect");
    else if (kqi.StartSendThread())
      mexErrMsgTxt("could not start thread");
    else
    {
      connected = 1;
      mexAtExit(mexExit);
      plhs[0] = mxCreateDoubleScalar(1);
      return;
    }
  }


  else if (strcasecmp(buf, "SendQuadCmd1") == 0)
  {
    CHECK_CONNECTION;
    
    if (nrhs != 5) mexErrMsgTxt("need 4 arguments: id, channel, quadType, [thrust, roll, pitch yaw]");
    if ( (mxGetClassID(prhs[1]) != mxDOUBLE_CLASS) || (mxGetClassID(prhs[2]) != mxDOUBLE_CLASS) ||
         (mxGetClassID(prhs[3]) != mxDOUBLE_CLASS) || (mxGetClassID(prhs[4]) != mxDOUBLE_CLASS) )
      mexErrMsgTxt("All input variables must be double class");
    
    double * id   = mxGetPr(prhs[1]);
    double * chan = mxGetPr(prhs[2]);
    double * type = mxGetPr(prhs[3]);
    double * trpy = mxGetPr(prhs[4]);
    
    //check dimensions
    if ((mxGetNumberOfElements(prhs[1]) != 1) || (mxGetNumberOfElements(prhs[2]) != 1) ||
        (mxGetNumberOfElements(prhs[3]) != 1) || (mxGetNumberOfElements(prhs[4]) != 4))
      mexErrMsgTxt("check input vector dimensions");
    
    int ret = kqi.SendQuadCmd1(id[0], type[0], chan[0], trpy[0], trpy[1], trpy[2], trpy[3]);

    if (ret < 0)
      mexErrMsgTxt("error in SendQuadCmd1");
    
    plhs[0] = mxCreateDoubleScalar(ret);
    return;
  }
  
  
  else if (strcasecmp(buf, "SendQuadCmd2") == 0)
  {
    CHECK_CONNECTION;
    
    if (nrhs != 5) mexErrMsgTxt("need 4 arguments: id, channel, quadType, [rpm0 rpm1 rpm2 rpm3]");
    if ( (mxGetClassID(prhs[1]) != mxDOUBLE_CLASS) || (mxGetClassID(prhs[2]) != mxDOUBLE_CLASS) ||
         (mxGetClassID(prhs[3]) != mxDOUBLE_CLASS) || (mxGetClassID(prhs[4]) != mxDOUBLE_CLASS) )
      mexErrMsgTxt("All input variables must be double class");
    
    double * id   = mxGetPr(prhs[1]);
    double * chan = mxGetPr(prhs[2]);
    double * type = mxGetPr(prhs[3]);
    double * rpm  = mxGetPr(prhs[4]);
    
    //check dimensions
    if ((mxGetNumberOfElements(prhs[1]) != 1) || (mxGetNumberOfElements(prhs[2]) != 1) ||
        (mxGetNumberOfElements(prhs[3]) != 1) || (mxGetNumberOfElements(prhs[4]) != 4))
      mexErrMsgTxt("check input vector dimensions");
    
    int ret = kqi.SendQuadCmd2(id[0], type[0], chan[0], rpm[0], rpm[1], rpm[2], rpm[3]);
    
    if (ret < 0)
      mexErrMsgTxt("error in SendQuadCmd2");

    plhs[0] = mxCreateDoubleScalar(ret);
    return;
  }
  
  else if (strcasecmp(buf, "SendQuadCmd3") == 0)
  {
    CHECK_CONNECTION;
    
    if (nrhs != 8) mexErrMsgTxt("need 6 arguments: id, channel, quadType, trpy, angular rates, kp, kd");
    if ( (mxGetClassID(prhs[1]) != mxDOUBLE_CLASS) || (mxGetClassID(prhs[2]) != mxDOUBLE_CLASS) ||
         (mxGetClassID(prhs[3]) != mxDOUBLE_CLASS) || (mxGetClassID(prhs[4]) != mxDOUBLE_CLASS) ||
         (mxGetClassID(prhs[5]) != mxDOUBLE_CLASS) || (mxGetClassID(prhs[6]) != mxDOUBLE_CLASS) ||
         (mxGetClassID(prhs[7]) != mxDOUBLE_CLASS) )
      mexErrMsgTxt("All input variables must be double class");
    
    double * id   = mxGetPr(prhs[1]);
    double * chan = mxGetPr(prhs[2]);
    double * type = mxGetPr(prhs[3]);
    double * trpy = mxGetPr(prhs[4]);
    double * drpy = mxGetPr(prhs[5]);
    double * kp   = mxGetPr(prhs[6]);
    double * kd   = mxGetPr(prhs[7]);
    
    //check dimensions
    if ((mxGetNumberOfElements(prhs[1]) != 1) || (mxGetNumberOfElements(prhs[2]) != 1) ||
        (mxGetNumberOfElements(prhs[3]) != 1) ||
        (mxGetNumberOfElements(prhs[4]) != 4) || (mxGetNumberOfElements(prhs[5]) != 3) ||
        (mxGetNumberOfElements(prhs[6]) != 3) || (mxGetNumberOfElements(prhs[7]) != 3))
      mexErrMsgTxt("check input vector dimensions");
    
    
    int ret = kqi.SendQuadCmd3(id[0], type[0],  chan[0],  trpy[0], trpy[1], trpy[2], trpy[3],
                              drpy[0], drpy[1],  drpy[2], kp[0],     kp[1],    kp[2],
                              kd[0],     kd[1],    kd[2]);

    if (ret < 0)
      mexErrMsgTxt("error in SendQuadCmd3");
                              
    plhs[0] = mxCreateDoubleScalar(ret);
    return;
  }
  
  else if (strcasecmp(buf, "SendQuadCmd4") == 0)
  {
    CHECK_CONNECTION;
    
    if (nrhs != 9) mexErrMsgTxt("need 6 arguments: id, channel, quadType, trpy, angular rates, kp, kd, moments");
    if ( (mxGetClassID(prhs[1]) != mxDOUBLE_CLASS) || (mxGetClassID(prhs[2]) != mxDOUBLE_CLASS) ||
         (mxGetClassID(prhs[3]) != mxDOUBLE_CLASS) || (mxGetClassID(prhs[4]) != mxDOUBLE_CLASS) ||
         (mxGetClassID(prhs[5]) != mxDOUBLE_CLASS) || (mxGetClassID(prhs[6]) != mxDOUBLE_CLASS) ||
         (mxGetClassID(prhs[7]) != mxDOUBLE_CLASS) || (mxGetClassID(prhs[8]) != mxDOUBLE_CLASS) )
      mexErrMsgTxt("All input variables must be double class");
    
    double * id   = mxGetPr(prhs[1]);
    double * chan = mxGetPr(prhs[2]);
    double * type = mxGetPr(prhs[3]);
    double * trpy = mxGetPr(prhs[4]);
    double * drpy = mxGetPr(prhs[5]);
    double * kp   = mxGetPr(prhs[6]);
    double * kd   = mxGetPr(prhs[7]);
    double * ms   = mxGetPr(prhs[8]);
    
    //check dimensions
    if ((mxGetNumberOfElements(prhs[1]) != 1) || (mxGetNumberOfElements(prhs[2]) != 1) ||
        (mxGetNumberOfElements(prhs[3]) != 1) ||
        (mxGetNumberOfElements(prhs[4]) != 4) || (mxGetNumberOfElements(prhs[5]) != 3) ||
        (mxGetNumberOfElements(prhs[6]) != 3) || (mxGetNumberOfElements(prhs[7]) != 3) ||
        (mxGetNumberOfElements(prhs[8]) != 3))
      mexErrMsgTxt("check input vector dimensions");
    
    
    int ret = kqi.SendQuadCmd4(id[0], type[0],  chan[0],  trpy[0], trpy[1], trpy[2], trpy[3],
                              drpy[0], drpy[1],  drpy[2], kp[0],     kp[1],    kp[2],
                              kd[0],     kd[1],    kd[2], ms[0], ms[1], ms[2]);

    
    if (ret < 0)
      mexErrMsgTxt("error in SendQuadCmd4");
    
    plhs[0] = mxCreateDoubleScalar(ret);
    return;
  }
  
  else if (strcasecmp(buf, "SendKbeeChannelCmd") == 0)
  {
    CHECK_CONNECTION;
    
    if (nrhs != 3) mexErrMsgTxt("need 2 arguments: kbee id and channel");
    if ( (mxGetClassID(prhs[1]) != mxDOUBLE_CLASS) || (mxGetClassID(prhs[2]) != mxDOUBLE_CLASS) )
      mexErrMsgTxt("All input variables must be double class");
    
    double * id = mxGetPr(prhs[1]);
    double * channel = mxGetPr(prhs[2]);
    
    //check dimensions
    if ((mxGetNumberOfElements(prhs[1]) != 1) || (mxGetNumberOfElements(prhs[2]) != 1))
      mexErrMsgTxt("check input vector dimensions");
    
    int ret = kqi.SendKbeeChannelCmd(id[0],channel[0]);
    
    if (ret < 0)
      mexErrMsgTxt("error in SendKbeeChannelCmd");
    
    plhs[0] = mxCreateDoubleScalar(ret);
    return;
  }
  
  else if (strcasecmp(buf, "SendKbeeModeCmd") == 0)
  {
    CHECK_CONNECTION;
    
    if (nrhs != 3) mexErrMsgTxt("need 2 arguments: kbee id and mode");
    if ( (mxGetClassID(prhs[1]) != mxDOUBLE_CLASS) || (mxGetClassID(prhs[2]) != mxDOUBLE_CLASS) )
      mexErrMsgTxt("All input variables must be double class");
    
    double * id = mxGetPr(prhs[1]);
    double * mode = mxGetPr(prhs[2]);
    
    //check dimensions
    if ((mxGetNumberOfElements(prhs[1]) != 1) || (mxGetNumberOfElements(prhs[2]) != 1))
      mexErrMsgTxt("check input vector dimensions");
    
    int ret = kqi.SendKbeeModeCmd(id[0],mode[0]);
    
    if (ret < 0)
      mexErrMsgTxt("error in SendKbeeModeCmd");
    
    plhs[0] = mxCreateDoubleScalar(ret);
    return;
  }
  
  
  else if (strcasecmp(buf, "PackQuadCmd1") == 0)
  {
    if (nrhs != 5) mexErrMsgTxt("need 4 arguments: id, channel, quadType, [thrust, roll, pitch yaw]");
    if ( (mxGetClassID(prhs[1]) != mxDOUBLE_CLASS) || (mxGetClassID(prhs[2]) != mxDOUBLE_CLASS) ||
         (mxGetClassID(prhs[3]) != mxDOUBLE_CLASS) || (mxGetClassID(prhs[4]) != mxDOUBLE_CLASS) )
      mexErrMsgTxt("All input variables must be double class");
    
    double * id   = mxGetPr(prhs[1]);
    double * chan = mxGetPr(prhs[2]);
    double * type = mxGetPr(prhs[3]);
    double * trpy = mxGetPr(prhs[4]);
    
    //check dimensions
    if ((mxGetNumberOfElements(prhs[1]) != 1) || (mxGetNumberOfElements(prhs[2]) != 1) ||
        (mxGetNumberOfElements(prhs[3]) != 1) || (mxGetNumberOfElements(prhs[4]) != 4))
      mexErrMsgTxt("check input vector dimensions");
    
    int len = kqi.PackQuadCmd1(id[0], type[0], chan[0], trpy[0], trpy[1], trpy[2], trpy[3], temp, 256);
    
    if (CreateOutputArrayUint8(&(plhs[0]), temp, len)) mexErrMsgTxt("error in PackQuadCmd1");
    return;
  }
  
  else if (strcasecmp(buf, "PackQuadCmd2") == 0)
  {
    if (nrhs != 5) mexErrMsgTxt("need 4 arguments: id, channel, quadType, [rpm0 rpm1 rpm2 rpm3]");
    if ( (mxGetClassID(prhs[1]) != mxDOUBLE_CLASS) || (mxGetClassID(prhs[2]) != mxDOUBLE_CLASS) ||
         (mxGetClassID(prhs[3]) != mxDOUBLE_CLASS) || (mxGetClassID(prhs[4]) != mxDOUBLE_CLASS) )
      mexErrMsgTxt("All input variables must be double class");
    
    double * id   = mxGetPr(prhs[1]);
    double * chan = mxGetPr(prhs[2]);
    double * type = mxGetPr(prhs[3]);
    double * rpm  = mxGetPr(prhs[4]);
    
    //check dimensions
    if ((mxGetNumberOfElements(prhs[1]) != 1) || (mxGetNumberOfElements(prhs[2]) != 1) ||
        (mxGetNumberOfElements(prhs[3]) != 1) || (mxGetNumberOfElements(prhs[4]) != 4))
      mexErrMsgTxt("check input vector dimensions");
    
    int len = kqi.PackQuadCmd2(id[0], type[0], chan[0], rpm[0], rpm[1], rpm[2], rpm[3], temp, 256);
    
    if (CreateOutputArrayUint8(&(plhs[0]), temp, len)) mexErrMsgTxt("error in PackQuadCmd2");
    return;
  }
  
  else if (strcasecmp(buf, "PackQuadCmd3") == 0)
  {
    if (nrhs != 8) mexErrMsgTxt("need 6 arguments: id, channel, quadType, trpy, angular rates, kp, kd");
    if ( (mxGetClassID(prhs[1]) != mxDOUBLE_CLASS) || (mxGetClassID(prhs[2]) != mxDOUBLE_CLASS) ||
         (mxGetClassID(prhs[3]) != mxDOUBLE_CLASS) || (mxGetClassID(prhs[4]) != mxDOUBLE_CLASS) ||
         (mxGetClassID(prhs[5]) != mxDOUBLE_CLASS) || (mxGetClassID(prhs[6]) != mxDOUBLE_CLASS) ||
         (mxGetClassID(prhs[7]) != mxDOUBLE_CLASS) )
      mexErrMsgTxt("All input variables must be double class");
    
    double * id   = mxGetPr(prhs[1]);
    double * chan = mxGetPr(prhs[2]);
    double * type = mxGetPr(prhs[3]);
    double * trpy = mxGetPr(prhs[4]);
    double * drpy = mxGetPr(prhs[5]);
    double * kp   = mxGetPr(prhs[6]);
    double * kd   = mxGetPr(prhs[7]);
    
    //check dimensions
    if ((mxGetNumberOfElements(prhs[1]) != 1) || (mxGetNumberOfElements(prhs[2]) != 1) ||
        (mxGetNumberOfElements(prhs[3]) != 1) ||
        (mxGetNumberOfElements(prhs[4]) != 4) || (mxGetNumberOfElements(prhs[5]) != 3) ||
        (mxGetNumberOfElements(prhs[6]) != 3) || (mxGetNumberOfElements(prhs[7]) != 3))
      mexErrMsgTxt("check input vector dimensions");
    
    
    int len = kqi.PackQuadCmd3(id[0], type[0],  chan[0],  trpy[0], trpy[1], trpy[2], trpy[3],
                              drpy[0], drpy[1],  drpy[2], kp[0],     kp[1],    kp[2],
                              kd[0],     kd[1],    kd[2], temp, 256);
    
    if (CreateOutputArrayUint8(&(plhs[0]), temp, len)) mexErrMsgTxt("error in PackQuadCmd3");
    return;
  }
  
  else if (strcasecmp(buf, "PackQuadCmd4") == 0)
  {
    if (nrhs != 9) mexErrMsgTxt("need 6 arguments: id, channel, quadType, trpy, angular rates, kp, kd, moments");
    if ( (mxGetClassID(prhs[1]) != mxDOUBLE_CLASS) || (mxGetClassID(prhs[2]) != mxDOUBLE_CLASS) ||
         (mxGetClassID(prhs[3]) != mxDOUBLE_CLASS) || (mxGetClassID(prhs[4]) != mxDOUBLE_CLASS) ||
         (mxGetClassID(prhs[5]) != mxDOUBLE_CLASS) || (mxGetClassID(prhs[6]) != mxDOUBLE_CLASS) ||
         (mxGetClassID(prhs[7]) != mxDOUBLE_CLASS) || (mxGetClassID(prhs[8]) != mxDOUBLE_CLASS) )
      mexErrMsgTxt("All input variables must be double class");
    
    double * id   = mxGetPr(prhs[1]);
    double * chan = mxGetPr(prhs[2]);
    double * type = mxGetPr(prhs[3]);
    double * trpy = mxGetPr(prhs[4]);
    double * drpy = mxGetPr(prhs[5]);
    double * kp   = mxGetPr(prhs[6]);
    double * kd   = mxGetPr(prhs[7]);
    double * ms   = mxGetPr(prhs[8]);
    
    //check dimensions
    if ((mxGetNumberOfElements(prhs[1]) != 1) || (mxGetNumberOfElements(prhs[2]) != 1) ||
        (mxGetNumberOfElements(prhs[3]) != 1) ||
        (mxGetNumberOfElements(prhs[4]) != 4) || (mxGetNumberOfElements(prhs[5]) != 3) ||
        (mxGetNumberOfElements(prhs[6]) != 3) || (mxGetNumberOfElements(prhs[7]) != 3) ||
        (mxGetNumberOfElements(prhs[8]) != 3))
      mexErrMsgTxt("check input vector dimensions");
    
    
    int len = kqi.PackQuadCmd4(id[0], type[0],  chan[0],  trpy[0], trpy[1], trpy[2], trpy[3],
                              drpy[0], drpy[1],  drpy[2], kp[0],     kp[1],    kp[2],
                              kd[0],     kd[1],    kd[2], ms[0], ms[1], ms[2], temp, 256);
    
    if (CreateOutputArrayUint8(&(plhs[0]), temp, len)) mexErrMsgTxt("error in PackQuadCmd4");
    return;
  }
  
  else if (strcasecmp(buf, "PackKbeeChannelCmd") == 0)
  {
    if (nrhs != 3) mexErrMsgTxt("need 2 arguments: kbee id and channel");
    if ( (mxGetClassID(prhs[1]) != mxDOUBLE_CLASS) || (mxGetClassID(prhs[2]) != mxDOUBLE_CLASS) )
      mexErrMsgTxt("All input variables must be double class");
    
    double * id = mxGetPr(prhs[1]);
    double * channel = mxGetPr(prhs[2]);
    
    //check dimensions
    if ((mxGetNumberOfElements(prhs[1]) != 1) || (mxGetNumberOfElements(prhs[2]) != 1))
      mexErrMsgTxt("check input vector dimensions");
    
    int len = kqi.PackKbeeChannelCmd(id[0],channel[0],temp,256);

    if (CreateOutputArrayUint8(&(plhs[0]), temp, len)) mexErrMsgTxt("error in PackKbeeChannelCmd");
    return;
  }
  
  else if (strcasecmp(buf, "PackKbeeModeCmd") == 0)
  {
    if (nrhs != 3) mexErrMsgTxt("need 2 arguments: kbee id and mode");
    if ( (mxGetClassID(prhs[1]) != mxDOUBLE_CLASS) || (mxGetClassID(prhs[2]) != mxDOUBLE_CLASS) )
      mexErrMsgTxt("All input variables must be double class");

    double * id = mxGetPr(prhs[1]);
    double * mode = mxGetPr(prhs[2]);
    
    //check dimensions
    if ((mxGetNumberOfElements(prhs[1]) != 1) || (mxGetNumberOfElements(prhs[2]) != 1))
      mexErrMsgTxt("check input vector dimensions");
    
    int len = kqi.PackKbeeModeCmd(id[0],mode[0],temp,256);
    
    if (CreateOutputArrayUint8(&(plhs[0]), temp, len)) mexErrMsgTxt("error in PackKbeeModeCmd");
    return;
  }
  
  else
    mexErrMsgTxt("kQuadInterfaceAPI : unknown command");
}

