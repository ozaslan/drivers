/* 
 * testSendQuadCmd1
 * 
 * Demonstrates sending control command using kQuadInterface.
 * SendQuadCmd1 sends desired thrust, roll, pitch, yaw.
 * 
 * Copyright KMel Robotics 2014. 
 * Must read KMEL_LICENSE.pdf for terms and conditions before use. 
*/

#include "kQuadInterface.hh"

void printUsage()
{
  std::cout << "Usage: ./testSendQuadCmd1 [PATH/TO/USB/DEVICE]" << std::endl;
}

int main(int argc, char* argv[])
{
  if (argc < 2)
  {
    printUsage();
    return -1;
  }
  
  char* dev = argv[1];
  int baud   = 921600;
  
  // kQuadInterface is the interface used to send/receive messages
  kQuadInterface kqi;
  
  // Connect to the device
  if (kqi.Connect(dev,baud))
  {
    printf("could not connect to the device\n");
    return -1;
  }
  
  // Initialize the send thread
  if (kqi.StartSendThread())
  {
    printf("could not start send thread\n");
    return -1;
  }
  
  // Initialize the receiving thread; won't be used in this example
  if (kqi.StartRecvThread())
  {
    printf("could not start receive thread\n");
    return -1;
  }
  
  uint8_t quadId   = 7; //must match the vehicle id
  uint8_t quadType = 0; //0 for standard and Nano+, 1 for Nano only
  uint8_t channel  = 1; //zigbee channel, does not matter for uart connection
  
  float thrust = 1; //(grams) 1 for idle
  float roll   = 0; //radians
  float pitch  = 0; //radians
  float yaw    = 0; //radians
  
  list<QuadStatusData> qdata;
  
  while(1)
  {
    kqi.SendQuadCmd1(quadId, quadType, channel, thrust, roll, pitch, yaw);
    // int nmsg  = kqi.GetQuadStatusData(qdata);
    
    auto p = qdata.end(); p--;
    
    std::cout << p->state << std::endl;
    
    usleep(20000);
    
    //printf("."); fflush(stdout);
  }
  
 
  return 0;
}
