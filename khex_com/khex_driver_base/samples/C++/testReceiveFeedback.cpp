/* 
 * testReceiveFeedback
 * 
 * Demonstrates receiving messages using kQuadInterface.
 * 
 * Copyright KMel Robotics 2014. 
 * Must read KMEL_LICENSE.pdf for terms and conditions before use. 
*/

#include <iostream>
#include "kQuadInterface.hh"
#include "Timer.hh"

void printUsage()
{
  std::cout << "Usage: ./testReceiveMsgs [PATH/TO/USB/DEVICE] [OPTIONS]" << std::endl;
  std::cout << "Options: " << std::endl;
  std::cout << "-v : activates verbose mode" << std::endl;
}

void parseCmdArgs(int argc, char* argv[], bool *verbose)
{
  int ch;
  for (int n = 1; n < argc; n++)
  {
    switch ((int)argv[n][0])
    {
      case '-':
        ch = (int)argv[n][1];
        switch (ch)
        {
          case 'v':
            *verbose = true;
            break;
          default:
            std::cout << "Invalid option." << std::endl;
        }
    }
  }
}

int main(int argc, char* argv[]) 
{
  if (argc < 2)
  {
    printUsage();
    return -1;
  }
  
  bool verbose = false;
  parseCmdArgs(argc, argv, &verbose);
  
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
  
  // Initialize the send thread; won't be used in this example
  if (kqi.StartSendThread())
  {
    printf("could not start send thread\n");
    return -1;
  }
  
  // Initialize the receiving thread
  if (kqi.StartRecvThread())
  {
    printf("could not start receive thread\n");
    return -1;
  }
  
  // Set the maximum queue length if desired.
  // kqi will buffer up to this many messages in a list
  kqi.SetMaxQueueLength(10);
  
  // Containers for receiving data from vehicle
  // See kQuadInterfaceDataTypes.hh for struct definitions
  list<ImuFiltData>     ifdata;
  list<RcData>          rcdata;
  list<QuadStatusData>  qdata;
  list<PressureMagData> pmdata;
  
  uint32_t nimu = 0, nrc = 0, nstat = 0, npress = 0;
  
  Timer t0; t0.Tic();
  
  int nmsg = 0;
  
  while(1)
  {    
    // Loops at approx. 100 Hz
    // Should receive about 100 messages of each type
    usleep(1000);
    double dt = t0.Toc();
    
    nmsg  = kqi.GetImuFiltData(ifdata); 
    if (verbose) kqi.PrintImuFiltData(ifdata);
    if (nmsg > 0) nimu += nmsg;
    
    nmsg  = kqi.GetRcData(rcdata);
    if (verbose) kqi.PrintRcData(rcdata);
    if (nmsg > 0) nrc += nmsg;
    
    nmsg  = kqi.GetQuadStatusData(qdata);
    if (verbose) kqi.PrintQuadStatusData(qdata);
    if (nmsg > 0) nstat += nmsg;
    
    nmsg  = kqi.GetPressMagData(pmdata);
    if (verbose) kqi.PrintPressMagData(pmdata);
    if (nmsg > 0) npress += nmsg;
    
    if (dt > 1)
    {
      printf("----------------------------------------\n");
      printf("received : imu(%d), rc(%d), status(%d), press(%d)\n", nimu, nrc, nstat, npress);
      
      nimu = nrc = nstat = npress = 0;
            
      t0.Tic(); 
    }
  }
  
  return 0;
}