% Copyright KMel Robotics 2014. Must read KMEL_LICENSE.pdf for terms and conditions before use.

function testSendQuadCmd1()

addpath ../../api
%open the interface
KQUAD.dev    = '/dev/ttyUSB0';
KQUAD.driver = @kQuadInterfaceAPI;
KQUAD.baud   = 921600;
KQUAD.id     = 7;
KQUAD.chan   = 1;
KQUAD.type   = 0; %0 for standard, 1 for nano

KQUAD.driver('connect',KQUAD.dev,KQUAD.baud);


while(1)
  thrust = 1; %grams. 1 for idle
  roll   = 0; %radians
  pitch  = 0; %radians
  yaw    = 0; %radians
  
  trpy = [thrust roll pitch yaw];
  
  KQUAD.driver('SendQuadCmd1',KQUAD.id, KQUAD.chan, KQUAD.type, trpy);

  pause(0.02);
  
  ch = getch();
    if ~isempty(ch)
        if ch == 'q'
            clear all
            break;
        end
    end
end
