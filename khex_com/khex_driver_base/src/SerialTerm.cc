/* Copyright KMel Robotics 2014. Must read KMEL_LICENSE.pdf for terms and conditions before use. */
#include "SerialDevice.hh"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

using namespace std;

struct termios oldterm,newterm;

int MakeTermNonblock()
{
  if (fcntl(0, F_SETFL, O_NONBLOCK))
  {
    printf("could not set non-blocking input\n");
    return -1;
  }
  
  //get current port settings
  if( tcgetattr( 0, &oldterm ) < 0 )
  {
    printf("Unable to get serial port attribute\n");
    return -1;
  }
  
  newterm = oldterm;

  //cfmakeraw initializes the port to standard configuration. Use this!
  cfmakeraw( &newterm );
  
  //set new attributes 
  if( tcsetattr( 0, TCSAFLUSH, &newterm ) < 0 )
  {
    printf("Unable to set serial port attributes\n");
    return -1;
  }
  
  return 0;
}

int RestoreTermSettings()
{
  if( tcsetattr( 0, TCSAFLUSH, &oldterm ) < 0 )
  {
    printf("Unable to set serial port attributes\n");
    return -1;
  }
  
  return 0;
}

int main(int argc, char * argv[])
{
  char * dev = (char*)"/dev/ttyUSB0";
  int baud   = 115200;

  if (argc < 3)
  {
    printf("need at least 4 arguments: dev, baud\n");
    return -1;
  }

  dev  = argv[1];
  baud = atoi(argv[2]);

  SerialDevice sd;

  if (sd.Connect(dev,baud))
  {
    printf("could not connect to device\n");
    return -1;
  }
  
  MakeTermNonblock();

  char c;
  int nchars;

  while(1)
  {
    if (sd.ReadChars(&c,1,1000) == 1)
    {
      printf("%c",c);
      fflush(stdout);
      
      //if (c == '\r')
      //  putchar('\n');
    }
    
    nchars = read(0,&c,1);
    if (nchars==1)
    {
      if (c == 'q')
      {
        break;
      }
      
      sd.WriteChars(&c,1);
      //printf("%c",c);
      fflush(stdout);
    }
  }

  sd.Disconnect();
  RestoreTermSettings();

  return 0;
}
