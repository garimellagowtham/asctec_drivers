/*=====================================================================

	MAVCONN Micro Air Vehicle Flying Robotics Toolkit

	(c) 2009, 2010 MAVCONN PROJECT  <http://MAVCONN.ethz.ch>

	This file is part of the MAVCONN project

	MAVCONN is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	MAVCONN is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with MAVCONN. If not, see <http://www.gnu.org/licenses/>.

	======================================================================*/

/**
 * @file
 *   @brief The serial interface process
 *
 *   This process connects any external MAVLink UART device to ROS
 *
 *   @author Lorenz Meier, <mavteam@student.ethz.ch>
 *
 */

#include "ros/ros.h"
#include "ros/time.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
//#include "sensor_msgs/MagneticField.h"
//#include "sensor_msgs/Temperature.h"
//#include "sensor_msgs/FluidPressure.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "sensor_msgs/Imu.h"
#include "asctec_msgs/LLStatus.h"
#include "mav_srvs/SetMotorsOnOff.h"
#include "asctec_ground_station/ARM_out.h"
#include "asctec_ground_station/RC_in.h"
// Standard includes
#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <cmath>
#include <string.h>
#include <inttypes.h>
#include <fstream>
// Serial includes
#include <stdio.h>   /* Standard input/output definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */

// Latency Benchmarking
#include <sys/time.h>
#include <time.h>

//#define TF_EULER_DEFAULT_ZYX
#define PI 3.143
//payload size is defined in mavlink_ros_serial.cpp and we just use it here
using std::string;
using namespace std;

ros::Publisher cmd_rpyt;
ros::Publisher batteryvoltage_pub;
ros::Publisher rpy_pub;

ros::Subscriber imudata_sub;
ros::Subscriber lowlevelstat_sub;

ros::Subscriber rcoverridereq_sub;
ros::Subscriber armsetreq_sub;
ros::Subscriber commandreq_sub;

ros::ServiceClient arming_client;


//tf broadcaster:
uint8_t heartbeatcount = 0;
geometry_msgs::Vector3 rpy_vec;
//uint8_t prevmsgid = 0;


//Calibrated transformation from vrpn to imu. Use this for ur pid algorithm.
tf::StampedTransform caltransform;
void armsetreqCallback(const asctec_ground_station::ARM_out &armmsg)
{
	//only 3 motors are possible right now:
	/*
	mavlink_message_t mavmsg;
	mavlink_arm_ctrl_pwm_t msg;
	msg.pwm1 = armmsg.pwm[0];
	msg.pwm2 = armmsg.pwm[1];
	msg.pwm3 = armmsg.pwm[2];
	*/
	//for now not publishing on anything
}


void rcoverridereqCallback(const asctec_ground_station::RC_in &datatype)
{ 
	geometry_msgs::Quaternion pubmsg;
	pubmsg.x = datatype.rcvalues[0];
	pubmsg.y = datatype.rcvalues[1];
	pubmsg.z = datatype.rcvalues[2];
	pubmsg.w = datatype.rcvalues[3];
	cmd_rpyt.publish(pubmsg);//publishing rpyt command
}


void commandreqCallback(const std_msgs::String &datatype)
{
	//construct command
	string data = datatype.data;
	istringstream iss(data);
	string substr;
	iss>>substr;
	ROS_INFO("\n%s",substr.c_str());
	mav_srvs::SetMotorsOnOff srvmsg;
	if(substr == "ARM")
	{
		srvmsg.request.on = true;
		cout<<"Arming_Status: "<<arming_client.call(srvmsg)<<endl;
	} 
	else  if(substr == "DISARM")
	{
		srvmsg.request.on = false;
		cout<<"Disarming_Status: "<<arming_client.call(srvmsg)<<endl;
	} 
	else
	{
		ROS_INFO("Invalid Arguments");
		return;
	}
}

void timerCallback(const ros::TimerEvent&)
{
	if(heartbeatcount == 0 )
	{
		ROS_INFO("System INACTIVE");
	}
	//else if(heartbeatcount == 255)
	//{
		//heartbeatcount = 0;
	//}
	//ROS_INFO("Heartbeat count: %d",heartbeatcount);
}

void llstatusCallback(const asctec_msgs::LLStatusPtr& ll_status_msg)
{
	heartbeatcount++;
	if((heartbeatcount%5) == 0)
	{
			//publish batttery voltage
			std_msgs::Float32 voltagemsg;
			voltagemsg.data = ((float)ll_status_msg->battery_voltage_1)/1000.0f;//in volts
			batteryvoltage_pub.publish(voltagemsg);
			ROS_INFO("Sys Status: cpu_load: %d flying: %d motors_on: %d flightmode: %d up_time: %d serial_status: %d\n",ll_status_msg->cpu_load, ll_status_msg->flying, ll_status_msg->motors_on, ll_status_msg->flightMode, ll_status_msg->up_time, ll_status_msg->status);
	}
	if(heartbeatcount == 255)
		heartbeatcount = 0;
}
void imudataCallback(const sensor_msgs::Imu imu_msg)
{
	static tf::TransformBroadcaster broadcaster;
	static tf::TransformListener listener;
	//Now publish the attitude message to /tf for visualization
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(0,0,0));
	//create a vector for rates to display:

	tf::Quaternion qatt(imu_msg.orientation.x,imu_msg.orientation.y,imu_msg.orientation.z,imu_msg.orientation.w);
	//qatt2.setEulerZYX(0,0,PI);
	transform.setRotation(qatt);
	//tf::Transform transform2;
	//transform2.setIdentity();
	broadcaster.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"APM2_BASE","APM2_IMU"));
	//get rpy
	tf::Matrix3x3 rotmat = transform.getBasis();
	rotmat.getEulerYPR(rpy_vec.z,rpy_vec.y,rpy_vec.x);
	rpy_vec.x = (180/PI)*rpy_vec.x;
	rpy_vec.y = (180/PI)*rpy_vec.y;
	rpy_vec.z = (180/PI)*rpy_vec.z;
	rpy_pub.publish(rpy_vec);	
	//printf("yaw: %f pitch: %f roll: %f \n",attitudemsg.yaw*(180/PI),attitudemsg.pitch*(180/PI),attitudemsg.roll*(180/PI));

	//create IMU_BASE wrt to optitrak:
	tf::Transform I_O;//imu_base with respect to optitrak
	I_O.setIdentity();
	tf::Quaternion qi_o;
	qi_o.setEulerZYX(-1.0,0,PI);//may be same if it uses North East and Down
	I_O.setRotation(qi_o);
	//create distance for imu_base by transforming distance from optitrak:
	//get the transform from optitrak to uav from ros_vrpn_client:
	tf::StampedTransform UV_O;
	//Lookup vrpn transformation
	try{
		listener.lookupTransform("optitrak","APM2_VRPN",ros::Time(0),UV_O);
	}
	catch (tf::TransformException ex) {
		return;
		//ROS_ERROR("\n%s",ex.what());
	}
	I_O.setOrigin(UV_O.getOrigin());
	broadcaster.sendTransform(tf::StampedTransform(I_O,ros::Time::now(),"optitrak","APM2_BASE"));
	//tf::Vector3 imu_baseorigin = I_O*UV_O.getOrigin();
	//transform APM2_VRPN to APM2_IMUDerived
	tf::Transform UID_UI;
	UID_UI.setIdentity();
	tf::Quaternion quid_ui;
	quid_ui.setEulerZYX(PI/2,0,PI);//yaw,pitch,roll
	//I_O.setRotation(qi_o);
	UID_UI.setRotation(quid_ui);
	broadcaster.sendTransform(tf::StampedTransform(UID_UI,ros::Time::now(),"APM2_IMU","APM2_IMUD"));
	
}

//int pubcount = 1;
int main(int argc, char **argv)
{
	ros::init(argc, argv, "gcs_receiver");
	// SETUP ROS
	ros::NodeHandle gcsr_nh;

	cmd_rpyt = gcsr_nh.advertise<geometry_msgs::Quaternion>("/mav/cmd_rpyt",5);
	batteryvoltage_pub = gcsr_nh.advertise<std_msgs::Float32>("/gcs/batteryvoltage",1);
	rpy_pub = gcsr_nh.advertise<geometry_msgs::Vector3>("imurpy",10);

	arming_client = gcsr_nh.serviceClient<mav_srvs::SetMotorsOnOff>("/mav/setMotorsOnOff",1);

	imudata_sub = gcsr_nh.subscribe("/mav/imu",2,imudataCallback);
	lowlevelstat_sub = gcsr_nh.subscribe("/asctec/LL_STATUS",2,llstatusCallback);

	rcoverridereq_sub = gcsr_nh.subscribe("/gcs/rcoverride",2,rcoverridereqCallback); 
	armsetreq_sub = gcsr_nh.subscribe("/gcs/armsetreq",2,armsetreqCallback); 
	commandreq_sub = gcsr_nh.subscribe("/gcs/commandreq",2,commandreqCallback); 

  ros::Timer timer = gcsr_nh.createTimer(ros::Duration(2), timerCallback);
	//set transformation to identity to begin with:
	caltransform.setIdentity();
	ros::spin();
	return 0;
}
