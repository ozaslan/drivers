/* Copyright KMel Robotics 2014. Must read KMEL_LICENSE.pdf for terms and conditions before use. */
#include "SerialDevice.hh"
#include "kBotPacket2.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

using namespace std;

int main(int argc, char * argv[])
{
  char * dev = (char*)"/dev/ttyUSB0";
  int baud   = 115200;
  uint8_t id   = 0;
  uint8_t type = 0;
  char * payload = NULL;
  uint8_t payloadSize = 0;

  if (argc < 5)
  {
    printf("need at least 4 arguments: dev, baud, id, type\n");
    return -1;
  }

  dev  = argv[1];
  baud = atoi(argv[2]);
  id   = atoi(argv[3]);
  type = atoi(argv[4]);

  if (argc == 6)
  {
    payload = argv[5];
    payloadSize = strlen(payload);
  }
  

  const int bufSize = 128;
  uint8_t buf[bufSize];

  int ret = kBotPacket2WrapData(id,type,(uint8_t*)payload,payloadSize,buf,bufSize);
  if (ret < 1)
  {
    printf("could not wrap data\n");
    return -1;
  }

  SerialDevice sd;

  if (sd.Connect(dev,baud))
  {
    printf("could not connect to device\n");
    return -1;
  }
  
  for (int ii=0; ii<ret; ii++)
  {
    sd.WriteChars((char*)(buf+ii),1);
    usleep(10000);
  }

/*
  if (sd.WriteChars((char*)buf,ret) != ret)
  {
    printf("could not write chars\n");
    return -1;
  }
  */

  usleep(0.1);
  sd.FlushInputBuffer();
  sd.Disconnect();


  printf("send cmd with payload size %d\n",payloadSize);

  return 0;
}
