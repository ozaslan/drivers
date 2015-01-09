% Copyright KMel Robotics 2014. Must read KMEL_LICENSE.pdf for terms and conditions before use.

clear all;
close all;

addpath ../../api
dev  = '/dev/ttyUSB0';
baud = 921600;
SerialDeviceAPI('connect',dev,baud);

tic

while(1)
    packet = ReceivePacket();
    if ~isempty(packet)
        id   = packet(3);
        type = packet(5);
        
        %fprintf('got packet %d %d\n',id,type);
        
        if (id == 0 && type == 42)
            %rc raw data
            
            % uint32_t tuc;  //microcontroller time, 10000 cnts per seconds
            % uint16_t rawData[12]; //raw rc data
            % uint16_t rcTimeouts;  //increments at 500Hz, max of 250
            
            rc.tuc         = double(typecast(packet(6:9),'uint32'));
            rc.rc_raw_data = double(typecast(packet(10:33),'uint16'));
            rc.rc_timeouts = double(typecast(packet(34:35),'uint16'));
            
            rc
            
            
        end
        
        if (id == 0 && type == 34)
            st.cnt =    double(typecast(packet(6:9),'uint32'));
            
            st.id   = double(typecast(packet(10),'uint8'));
            st.cnt  = double(typecast(packet(11),'uint8'));
            st.rpy  = double(typecast(packet(12:17),'int16'))/5000;
            st.wrpy = double(typecast(packet(18:23),'int16'))/500;
            st.acc  = double(typecast(packet(24:29),'int16'))/5000;

            st
            
            
        end
        
        if (id == 0 && type == 41)
            
            %   uint32_t tuc;
            %   uint16_t voltage;  //voltage*100
            %   uint8_t  id;
            %   uint8_t  state;
            %   uint16_t  autoCntr;
            %   uint16_t  rcCntr;
            
            p = 6;
            s = 4; data.tuc   = double(typecast(packet(p:p+s-1),'uint32')); p = p + s;
            
            s = 2; data.voltage = double(typecast(packet(p:p+s-1),'uint16'))*0.001; p = p + s;
            
            s = 1; data.id    = double(typecast(packet(p:p+s-1),'uint8')); p = p + s;
            s = 1; data.state = double(typecast(packet(p:p+s-1),'uint8')); p = p + s;
            
            s = 2; data.autoCntr = double(typecast(packet(p:p+s-1),'uint16')); p = p + s;
            s = 2; data.rcCntr   = double(typecast(packet(p:p+s-1),'uint16')); p = p + s;
            
            data   
            
        end
        
        
        if(id == 0 && type == 40)
            
            % uint32_t tuc;
            %
            % int16_t press; //pressure in pascals (100,000 Pa subtracted)
            % int16_t temp; //degrees celsius * 100
            % uint32_t press_time;
            % float baro_zpos;
            %
            % int16_t mxyz[3];
            % uint32_t mag_time;
            
            
            p = 6;
            
            s = 4; pdata.tuc        = double(typecast(packet(p:p+s-1),'uint32')); p = p + s;
            s = 2; pdata.press      = double(typecast(packet(p:p+s-1),'int16'))+100000; p = p + s;
            s = 2; pdata.temp       = double(typecast(packet(p:p+s-1),'int16'))/100; p = p + s;
            s = 4; pdata.press_time = double(typecast(packet(p:p+s-1),'uint32')); p = p + s;
            s = 4; pdata.baro_zpos  = double(typecast(packet(p:p+s-1),'single')); p = p + s;
            
            s = 6; pdata.mxyz       = double(typecast(packet(p:p+s-1),'int16')); p = p + s;
            s = 4; pdata.mag_time   = double(typecast(packet(p:p+s-1),'int32')); p = p + s;
            
            
            pdata

            
        end
        
    end
    ch = getch();
    if ~isempty(ch)
        if ch == 'q'
            clear all
            break;
        end
    end
end