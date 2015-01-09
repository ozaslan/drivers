// Possible improvements are noted down as comments starting with #'s

#include <ros/ros.h>
#include <iostream>
#include <string>
#include <list>
#include <stdlib.h>

#include <Eigen/Dense>

#include <khex_com/KHexCmd.h>
#include <khex_com/KHexPMT.h>
#include <khex_com/KHexRC.h>
#include <khex_com/KHexStatus.h>
#include <sensor_msgs/Imu.h>
#include <kQuadInterface.hh>

using namespace std;
using namespace khex_com;
using Eigen::MatrixXd;

int robot_id = 0;
int refresh_rate;
bool debug_mode;
int baud_rate = 921600;
string dev_path;
string frame_set;
ros::Publisher imu_publ;
ros::Publisher status_publ;
ros::Publisher pmt_publ;
ros::Publisher rc_publ;
ros::Subscriber cmd_subs;

// kQuadInterface is the interface used to send/receive messages
kQuadInterface kqi;
// Containers for receiving data from the vehicle
// See kQuadInterfaceDataTypes.hh for struct definitions
list<ImuFiltData>     ifdata;
list<RcData>          rcdata;
list<QuadStatusData>  qdata;
list<PressureMagData> pmdata;

// This callback listens to PD commands topic. Low level communication
// is carried through kQuadInterface functions.
void cmd_callback(const KHexCmd &msg);
// This function is called before the ros loop starts.
// Parameter server is querried in this function.
void process_inputs(const ros::NodeHandle &n);
// This function setups the topics to be listened and
// those through which messages will be sent.
void setup_messaging_interface(ros::NodeHandle &n);
// Main loop.
void loop(const ros::NodeHandle &n);
// Upon retreiving data through kQuadInterface, ros topics
// are populated and pushed into the messaging queue.
void publish_data();

int main(int argc, char* argv[]){
	
	ros::init(argc, argv, "khex_com_node");
  	ros::NodeHandle n("~");

	process_inputs(n);

	setup_messaging_interface(n);
	
	// Connect to the device.
	if (kqi.Connect(dev_path, baud_rate)){
		ROS_ERROR("--- KHEX COM ---");
		ROS_ERROR("Could not connect to the device [%s] with baud rate [%d]", dev_path.c_str(), baud_rate);
		return -1;
	}
  
	// Initialize the send thread
	if (kqi.StartSendThread()){
		ROS_ERROR("--- KHEX COM ---");
		ROS_ERROR("Could not start the send thread\n");
	    return -1;
	}
  
	// Initialize the receiving thread
	if (kqi.StartRecvThread()){
		ROS_ERROR("--- KHEX COM ---");
		ROS_ERROR("Could not start the receive thread\n");
		return -1;
	}
  
	// Set the maximum queue length if desired.
	// kqi will buffer up to this many messages in a list
	kqi.SetMaxQueueLength(10);
  
	loop(n);	
	
	return 1;
}

void process_inputs(const ros::NodeHandle &n)
{
	n.param("dev_path", dev_path, string("/dev/ttyUSB0"));
	n.param("refresh_rate", refresh_rate, 100);
	n.param("debug_mode", debug_mode, false);

	// NOTE : Default forward direction of the KHex is not
	// aligned with the Inspection KHex configuration.
	// The other option for frame_set is "default". When "inspection_khex"
	// is selected, raw data is post-processed to align with the 
	// inspection KHex coordinate frame.
	n.param("frame_set", frame_set, string("inspection_khex"));
	n.param("baud_rate", baud_rate, 921600);

	ROS_INFO(" --------------- KHEX COM ---------------");
	ROS_INFO("[refresh_rate] -- : [%d]", refresh_rate);
	ROS_INFO("[debug_mode] ---- : [%s]", debug_mode ? "TRUE" : "FALSE");
	ROS_INFO("[baud_rate] ----- : [%d]", baud_rate);
	ROS_INFO("[dev_path] ------ : [%s]", dev_path.c_str());
	ROS_INFO("[frame_set] ----- : [%s]", frame_set.c_str());
	ROS_INFO(" ----------------------------------------");
}

void setup_messaging_interface(ros::NodeHandle &n)
{
	imu_publ     = n.advertise<sensor_msgs::Imu>("imu", 10);
	status_publ  = n.advertise<KHexStatus>("status", 10);
	pmt_publ     = n.advertise<KHexPMT>("pmt", 10);
	rc_publ      = n.advertise<KHexRC>("rc", 10);
	cmd_subs     = n.subscribe("cmd", 10, cmd_callback, ros::TransportHints().tcpNoDelay()); 

	ROS_INFO("----------- KHEX COM -------------------------------------------");
	ROS_INFO("Publishing filtered imu data to <imu>");
	ROS_INFO("Publishing robot status to <status>");
	ROS_INFO("Publishing pressure/magnetometer/temperature to <pmt>");
	ROS_INFO("Publishing RC data to <rc>");
	ROS_INFO("Listening to <cmd> for commands");
	ROS_INFO("----------------------------------------------------------------");
}

void loop(const ros::NodeHandle &n)
{
	// # of messages retreived in one query.
	int nif, nrc, nqs, npmt;
	ros::Rate r(refresh_rate);

	while (n.ok())
	{      
		ros::spinOnce();
	 
		nif  = kqi.GetImuFiltData(ifdata); 
    
		nrc  = kqi.GetRcData(rcdata);
    
		nqs  = kqi.GetQuadStatusData(qdata);
    
		npmt = kqi.GetPressMagData(pmdata);

		if (debug_mode){
			if(nif != 0 || nrc != 0 || nqs != 0 || npmt != 0)
				ROS_INFO("------ KHEX COM ------");
			if(nif > 0)
				kqi.PrintImuFiltData(ifdata);
			if(nrc > 0)
				kqi.PrintRcData(rcdata);
			if(nqs > 0)
				kqi.PrintPressMagData(pmdata);
			if(npmt > 0)
				kqi.PrintQuadStatusData(qdata);
			if(nif != 0 || nrc != 0 || nqs != 0 || npmt != 0)
				ROS_INFO("----------------------");
		}

		publish_data();

		r.sleep();
	}
}

void publish_data(){
	static sensor_msgs::Imu	khex_if;
	static KHexRC			khex_rc;
	static KHexPMT          khex_pmt;
	static KHexStatus       khex_sta;
	static double g = 9.81;
	static MatrixXd R(3, 3);

	if(ifdata.size() !=0){
		ImuFiltData ifd = ifdata.back();
		robot_id =  ifd.id;
		khex_if.header.seq++;
		khex_if.header.stamp = ros::Time::now();
		khex_if.header.frame_id = "body";

		// Convert from euler to dcm
		double psi = ifd.yaw;   // yaw   - around z
		double phi = ifd.roll;  // roll  - around interm. x
		double the = ifd.pitch; // pitch - around interm. y
		
		double cpsi = cos(psi);
		double spsi = sin(psi);
		double cphi = cos(phi);
		double sphi = sin(phi);
		double cthe = cos(the);
		double sthe = sin(the);

		// Due to Mellinger RAM2010 paper.
		// Z-X-Y Euler angles are used to encode the orientation.
		R(0, 0) =  cpsi * cthe - sphi * spsi * sthe;
		R(0, 1) = -cphi * spsi;
		R(0, 2) =  cpsi * sthe + sphi * spsi * cthe;
		R(1, 0) =  spsi * cthe + sphi * cpsi * sthe;
		R(1, 1) =  cphi * cpsi;
		R(1, 2) =  spsi * sthe - sphi * cpsi * cthe;
		R(2, 0) = -cphi * sthe;
		R(2, 1) =  sphi;
		R(2, 2) =  cphi * cthe;

		double tr = R(0, 0) + R(1, 1) + R(2, 2);
		double qw, qx, qy, qz, S;
		// ### Should start using 'utilities'
		if (tr > 0) { 
			S = sqrt(tr + 1.0) * 2; // S=4*qw 
			qw = 0.25 * S;
			qx = (R(2, 1) - R(1, 2)) / S;
			qy = (R(0, 2) - R(2, 0)) / S; 
			qz = (R(1, 0) - R(0, 1)) / S; 
		} else if ((R(0, 0) > R(1, 1))&(R(0, 0) > R(2, 2))) { 
			S = sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2)) * 2; // S=4*qx 
			qw = (R(2, 1) - R(1, 2)) / S;
			qx = 0.25 * S;
			qy = (R(0, 1) + R(1, 0)) / S; 
			qz = (R(0, 2) + R(2, 0)) / S; 
		} else if (R(1, 1) > R(2, 2)) { 
			S = sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2)) * 2; // S=4*qy
			qw = (R(0, 2) - R(2, 0)) / S;
			qx = (R(0, 1) + R(1, 0)) / S; 
			qy = 0.25 * S;
			qz = (R(1, 2) + R(2, 1)) / S; 
		} else { 
			S = sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1)) * 2; // S=4*qz
			qw = (R(1, 0) - R(0, 1)) / S;
			qx = (R(0, 2) + R(2, 0)) / S;
			qy = (R(1, 2) + R(2, 1)) / S;
			qz = 0.25 * S;
		}

		// In the quaternion space, permuting the basis vectors
		// corresponds to swapping the normal vector coordinates 
		// and flipping their directions.
		if(frame_set == "inspection_khex"){
			std::swap(qx, qy);
			qy *= -1;
			std::swap(ifd.wroll, ifd.wpitch);
			ifd.wroll *= -1;
			std::swap(ifd.ax, ifd.ay);
			ifd.ax *= -1;
		}

		khex_if.orientation.w = qw;
		khex_if.orientation.x = qx;
		khex_if.orientation.y = qy;
		khex_if.orientation.z = qz;

		khex_if.angular_velocity.x = ifd.wroll;
		khex_if.angular_velocity.y = ifd.wpitch;
		khex_if.angular_velocity.z = ifd.wyaw;
		
		khex_if.linear_acceleration.x = ifd.ax * g;
		khex_if.linear_acceleration.y = ifd.ay * g;
		khex_if.linear_acceleration.z = ifd.az * g;

		imu_publ.publish(khex_if);
	}

	if(rcdata.size() !=0){
		RcData rc = rcdata.back();
		khex_rc.header.seq++;
		khex_rc.header.stamp = ros::Time::now();
		khex_rc.header.frame_id = "body";
	
		khex_rc.right_fb = rc.data[2];
		khex_rc.right_rl = rc.data[1];
		khex_rc.left_fb  = rc.data[0];
		khex_rc.left_rl  = rc.data[3];
		
		// First two values correspond to switches on the left
		// of the RC.
		for(int i = 0 ; i < 6 ; i++)
			khex_rc.misc[i] = rc.data[i+4];

		rc_publ.publish(khex_rc);
	}

	if(pmdata.size() !=0){
		PressureMagData pmt = pmdata.back();
		khex_pmt.header.seq++;
		khex_pmt.header.stamp = ros::Time::now();
		khex_pmt.header.frame_id = "body";
	
		khex_pmt.pressure    = pmt.pressure;
		khex_pmt.temperature = pmt.temperature;
		khex_pmt.baro_zpos   = pmt.baro_zpos;

		if(frame_set == "default"){
			khex_pmt.mag[0] = pmt.mx;
			khex_pmt.mag[1] = pmt.my;
			khex_pmt.mag[2] = pmt.mz;
		} else { // if(frame_set == "inspection_khex")
			// ### Not sure if the conversion is correct.
			// Current robot does not have a magnetometer.
			khex_pmt.mag[0] = -pmt.my;
			khex_pmt.mag[1] =  pmt.mx;
			khex_pmt.mag[2] =  pmt.mz;
		}

		pmt_publ.publish(khex_pmt);
	}

	if(qdata.size() !=0){
		QuadStatusData s = qdata.back();
		khex_sta.header.seq++;
		khex_sta.header.stamp = ros::Time::now();
		khex_sta.header.frame_id = "body";
	
		khex_sta.voltage  = s.voltage;
		khex_sta.current  = s.current;
		// ### Cpu load part needs some effort. Left to the
		// future versions.
		khex_sta.cpu_load = 0;

		status_publ.publish(khex_sta);
	}
}

void cmd_callback(const KHexCmd &msg){
	static unsigned char quadType = 0; //0 for standard and Nano+, 1 for Nano only
	static unsigned char channel  = 1; //zigbee channel, does not matter for uart connection

	// ### Have to handle SendQuadCmd3 and SendQuadCmd4 alternatives as well.
	if(debug_mode){
		ROS_INFO("KHEX COM : Got KHexCmd Message");
		ROS_INFO("[id, T, R, P, Y] = [%d, %.3f, %.3f, %.3f, %.3f]", robot_id, msg.thrust, msg.roll, msg.pitch, msg.yaw);
	}

	kqi.SendQuadCmd1(robot_id, quadType, channel, msg.thrust, msg.roll, msg.pitch, msg.yaw);
	// ### Currently I cannot change the onboard coefficients. Thus kp, kd parameters in the message does not affect.
}
