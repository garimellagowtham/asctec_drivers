#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <cmath>
#include <string.h>
#include <inttypes.h>
#include <fstream>
#include <ncurses.h>
//#include "common_definitions.h"

// Latency Benchmarking
#include <sys/time.h>
#include <time.h>

//message types
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "asctec_ground_station/RC_in.h"
#include "asctec_ground_station/PIDGains.h"
#include "SetptCtrl.h"

#include <pthread.h>

#define THROTCHAN 3
#define RC_SIZE 4

using namespace std;

//ros::Subscriber mavlink_sub;
ros::Publisher rcoverride_pub;
//ros::Publisher datareq_pub; not implemented yet
ros::Publisher commandreq_pub;
ros::Publisher modereq_pub;
ros::Publisher usergains_pub;

ros::Subscriber currframe_sub;
ros::Subscriber lowlevel_sub;
//RC_Trim min and max values used for control and landing:
SetptCtrl *setptctrlinst;
pthread_mutex_t setptctrl_mutex = PTHREAD_MUTEX_INITIALIZER;


/*
void timerCallback(const ros::TimerEvent&)
{
	printw("Spinning");
	ros::spinOnce();
	refresh();
}
*/

float RC_TRIM[RC_SIZE] {0,0,0,0.5};
float RC_MIN[RC_SIZE] {-1,-1,-1,0};
float RC_MAX[RC_SIZE] {1,1,1,1};
float Fextz = -4.0; //-mg roughly start with smaller value and increase slowly
float rptmin[4] {-PI/3.4615, -PI/3.4615, -2, 2};
float rptmax[4] {PI/3.4615, PI/3.4615, 2, 10};
//float rptbounds[3] {PI/18,PI/18,10};
float rptbounds[4] {PI/9,PI/9,2,10};
asctec_ground_station::RC_in rcmsg;
int row,col;
//int refreshcount = 0;
bool setptctrl_active = false;

void statuszoneclear()
{
	move(row-5,0);
	clrtobot();
}


float scale(float val, int id)
{
	float res = 0;
	//cutoff for the input value so that we dont ever command uav to rotate more than bounds
	if(val < -rptbounds[id])
	{
		val =  -rptbounds[id];
	}
	else if(val > rptbounds[id])
	{
		val = rptbounds[id];
	}
	res = RC_MIN[id] + ((val - rptmin[id])/(rptmax[id] - rptmin[id]))*(RC_MAX[id] - RC_MIN[id]);//will need a better way to scale based on custom trims
	return res;
}

void rcpub(Vector3 rpy, float throttle)
{
			//scale
			rcmsg.rcvalues[0] = (double)scale(rpy[0],0);//roll
			rcmsg.rcvalues[1] = (double)scale(rpy[1],1);//pitch
			rcmsg.rcvalues[2] = (double)RC_TRIM[2];//yaw is default trim
			rcmsg.rcvalues[3] = (double)scale(throttle,3);//throttle
			//publish message
			//ROS_INFO("Publishing rc_override");
			//ROS_INFO("Publishing done");
			//print commanded data:
			mvprintw(0,0,"RC_1: %f\t RC_2: %f\t RC_3: %f\t RC_4: %f\t",rcmsg.rcvalues[0], rcmsg.rcvalues[1], rcmsg.rcvalues[2], rcmsg.rcvalues[3]);
			mvprintw(1,0,"r: %f\t p: %f\t y: %f\t t: %f\t\n",rpy[0]*(180/PI), rpy[1]*(180/PI), rpy[2]*(180/PI), throttle);
			rcoverride_pub.publish(rcmsg);
}
void *rcCallback(void *dummy)
{
	while(1)
	{
		ros::spinOnce();
		pthread_mutex_lock(&setptctrl_mutex);
		if(setptctrl_active == true)
		{
			if(setptctrlinst->freshdata)
			{
				setptctrlinst->freshdata = false;
				rcpub(setptctrlinst->rpycommand,setptctrlinst->throtcommand);
				//print rcvals etc on screen
				setptctrlinst->freshdata = false;
				mvprintw(2,0,"position: %f\t%f\t%f\n",setptctrlinst->prev_position[0],setptctrlinst->prev_position[1],setptctrlinst->prev_position[2]);
				if(setptctrlinst->criticalvoltage)
				{
					std_msgs::String cmdparse;
					cmdparse.data = "DISARM";
					statuszoneclear();
					printw("Critical Voltage Reached Disarming");
					commandreq_pub.publish(cmdparse);
					pthread_exit(NULL);
				}
				refresh();
			}
		}
		pthread_mutex_unlock(&setptctrl_mutex);
		usleep(15000);//10ms = 100Hz
	}
	return NULL; 
}

int main(int argc, char **argv)
{
	pthread_t rcpub_thread;
	ros::init(argc, argv, "keyboardctrl");
	// SETUP ROS
	ros::NodeHandle gcs_nh("/gcs");

	initscr();//initialize screen ncurses
	//settings for ncurses:
	cbreak();
	//timeout(100);//waits for 100 ms
	timeout(-1);
	getmaxyx(stdscr,row,col);

	//setptctrlinst:
	setptctrlinst = new SetptCtrl(&gcs_nh);
	currframe_sub = gcs_nh.subscribe("pose",1,&SetptCtrl::Set,setptctrlinst);
	//set goal position:
	setptctrlinst->setgoal(1.0f,1.2f,0.5f);
	//setptctrlinst->setgoal(0.0f,0.0f,0.5f);
	//setptctrlinst->kdr = 1.5;
	pthread_create(&rcpub_thread,NULL,rcCallback,NULL);//create rcthread for publishing data
	//wait for all nodes to begin
	usleep(500000);
	//publishers
	rcoverride_pub = gcs_nh.advertise<asctec_ground_station::RC_in>("rcoverride", 2);
	//datareq_pub = gcs_nh.advertise<std_msgs::String>("datareq",2);
	modereq_pub = gcs_nh.advertise<std_msgs::String>("modereq",2);
	commandreq_pub = gcs_nh.advertise<std_msgs::String>("commandreq",2);
	usergains_pub = gcs_nh.advertise<asctec_ground_station::PIDGains>("usergains",2);
	//set size of rcmsg:
	rcmsg.rcvalues.resize(RC_SIZE,0);
 // ros::Timer timer = gcs_nh.createTimer(ros::Duration(1.0/30.0), timerCallback);
 //request parameter list so that u have update params:
	//buffer for reading vals:
	char cbuffer;
	string sbuffer;

	//variables for control:
	//char rcid[10];
	//char parambuf[40];
	char mode = '+';
	char mode2 = 'u';
	//messages for parsing and sending
	std_msgs::String cmdparse;
	std_msgs::String modeparse;
	std_msgs::Empty emptymsg;
	asctec_ground_station::PIDGains gainsmsg;
	tf::Vector3 rpycommand;
	float throtcommand;
	refresh();
	usleep(1000000);
	statuszoneclear();
	printw("Available modes:\n Setpoint Control: s \t Test motors: t \tArm: a \t Disarm: d \t Land: l \t Quit: q ");
	while(1)
	{
		move(row/2,col/2);//go to center of screen
		cbuffer = getch();
		statuszoneclear();
		switch(cbuffer)
		{
			case 's':
				printw("Setpoint Control: mode user gains(u)");
				timeout(-1);
				setptctrlinst->kpr = 2.7;
				setptctrlinst->kdr = 2.0;
				setptctrlinst->kpt = 0.8;
				setptctrlinst->kdt = 0.7;
				setptctrlinst->cbatt = 5.0;
				//dataparseval.data = "ATTITUDE START";
			//datareq_pub.publish(dataparseval);
				//register the callback for ros topic:
				//ROS_INFO("Hai");
				//dataparseval.data = "EXTENDED START";
				//datareq_pub.publish(dataparseval);//starts extended value
				move((row+20)/2,0);
				clrtoeol();
				//make setptctrl thread active:
				pthread_mutex_lock(&setptctrl_mutex);
					setptctrl_active = true;
				pthread_mutex_unlock(&setptctrl_mutex);
				while(1)
				{
					move(row/2,col/2);//go to center of screen
					cbuffer = getch();
					if(cbuffer == 'q')
						goto QUIT;
					else if(cbuffer == 'l')
						goto LAND;
					else if(cbuffer == '+'|| cbuffer == '-')
						mode = cbuffer;
					else if(cbuffer == 'o' || cbuffer == 'u' || cbuffer == 'i')
					{
						mode2 = cbuffer;
						statuszoneclear();
						if(mode2 == 'u')
							printw("Controls: mode +/- to increase decrease values \n r-> kpr (both roll and pitch tied) \t R-> kdr \t t->kpt \t T -> kdt \t b-> Fextz (Hover coeff) \t \n");
					}
					pthread_mutex_lock(&setptctrl_mutex);
					//action base on mode
					if(mode2 == 'u')
					{
						if(cbuffer == 'r')
						{
							if(mode == '+')
								setptctrlinst->kpr *= 1.1;
							else 
								setptctrlinst->kpr /= 1.1;
						}
						else if(cbuffer == 't')
						{
							if(mode == '+')
								setptctrlinst->kpt *= 1.1;
							else 
								setptctrlinst->kpt /= 1.1;
						}
						else if(cbuffer == 'T')
						{
							if(mode == '+')
								setptctrlinst->kdt *= 1.1;
							else 
								setptctrlinst->kdt /= 1.1;
						}
						else if(cbuffer == 'R')
						{
							if(mode == '+')
								setptctrlinst->kdr *= 1.1;
							else 
								setptctrlinst->kdr /= 1.1;
						}
						else if(cbuffer == 'b')
						{
							if(mode == '+')
							{
								setptctrlinst->cbatt *= 1.05;
								//Fextz *= 1.05;
								//setptctrlinst->Fext.setValue(0,0,Fextz);
							}
							else 
							{
								setptctrlinst->cbatt /= 1.05;
								//Fextz /= 1.05;
								//setptctrlinst->Fext.setValue(0,0,Fextz);
							}
						}
							move((row+20)/2,0);
							clrtoeol();
							printw("kpr: %f\t kdr: %f\t kpt: %f\t kdt: %f\t Fextz: %f\t mode: %c",setptctrlinst->kpr,setptctrlinst->kdr,setptctrlinst->kpt,setptctrlinst->kdt,setptctrlinst->cbatt, mode);
							//publish gains:
							gainsmsg.kp = setptctrlinst->kpr;
							gainsmsg.kd = setptctrlinst->kdr;
							gainsmsg.ki = 0;//pd controller
							gainsmsg.id = string("Roll pd");
							usergains_pub.publish(gainsmsg);
							gainsmsg.kp = setptctrlinst->kpt;
							gainsmsg.kd = setptctrlinst->kdt;
							gainsmsg.ki = 0;//pd controller
							gainsmsg.id = string("Throttle pd");
							usergains_pub.publish(gainsmsg);
					}
					pthread_mutex_unlock(&setptctrl_mutex);
					refresh();
				}
				timeout(-1);
				break;
			case 't':
				statuszoneclear();
				printw("Testing motors: roll r pitch p throttle t mode +/-");
				rpycommand.setValue(0.0,0.0,0.0);
				throtcommand = 3.0;
				timeout(50);//20Hz
				//dataparseval.data = "ATTITUDE START";
				//datareq_pub.publish(dataparseval);
				//dataparseval.data = "EXTENDED START";
				//datareq_pub.publish(dataparseval);//starts extended value
				//pthread_mutex_lock(&setptctrl_mutex);
					//setptctrl_active = true;
				//pthread_mutex_unlock(&setptctrl_mutex);
				//move((row+20)/2,0);
				//clrtoeol();
				while(1)
				{
					ros::spinOnce();
					if(setptctrlinst->freshdata)
					{
						if(setptctrlinst->criticalvoltage)
						{
							std_msgs::String cmdparse;
							cmdparse.data = "DISARM";
							statuszoneclear();
							printw("Critical Voltage Reached Disarming");
							//publish multiple times
							commandreq_pub.publish(cmdparse);
							pthread_exit(NULL);
						}
					}
					rcpub(rpycommand,throtcommand);
					//print rcvals etc on screen
					//setptctrlinst->freshdata = false;
					mvprintw(2,0,"position: %f\t%f\t%f\n",setptctrlinst->prev_position[0],setptctrlinst->prev_position[1],setptctrlinst->prev_position[2]);
					move(row/2,col/2);//go to center of screen
					cbuffer = getch();
					if(cbuffer == 'q')
						goto QUIT;
					else if(cbuffer == 'l')
						goto LAND;
					else if(cbuffer == 'r')
					{
						if(mode == '+')
							rpycommand[0] += 2*(PI/180);
						else 
							rpycommand[0] -= 2*(PI/180);
					}
					else if(cbuffer == 'p')
					{
						if(mode == '+')
							rpycommand[1] += 2*(PI/180);
						else 
							rpycommand[1] -= 2*(PI/180);
					}
					else if(cbuffer == 't')
					{
						if(mode == '+')
							throtcommand *= 1.1;
						else 
							throtcommand /= 1.1;
					}
					else if(cbuffer == '+'|| cbuffer == '-')
						mode = cbuffer;
					//mvprintw((row+20)/2,0,"r: %f\t p: %f\t t: %f\t kdt: %f\t Fextz: %f\t mode: %c",setptctrlinst.kpr,setptctrlinst.kdr,setptctrlinst.kpt,setptctrlinst.kdt,-Fextz, mode);
					refresh();
				}
				break;
			case 'a':
				//ARM
				statuszoneclear();
				printw("Setting rc values to minimum\n");
				for(int i = 0;i<RC_SIZE;i++)
					rcmsg.rcvalues[i] = RC_MIN[i];
				rcoverride_pub.publish(rcmsg);
				usleep(10000);
				//rcmsg.rcvalues[THROTCHAN] = LAND_THROT;
				printw("Arming\n");
				cmdparse.data = "ARM";
				commandreq_pub.publish(cmdparse);
				break;
			case 'd':
				//DISARM
DISARM:
				pthread_mutex_lock(&setptctrl_mutex);
					setptctrl_active = false;
				pthread_mutex_unlock(&setptctrl_mutex);
				statuszoneclear();
				printw("Disarming\n");
				cmdparse.data = "DISARM";
				commandreq_pub.publish(cmdparse);
				break;
			case 'l':
LAND:
				//we land.
				//First set mode to stabilize:
				pthread_mutex_lock(&setptctrl_mutex);
					setptctrl_active = false;
				pthread_mutex_unlock(&setptctrl_mutex);
				statuszoneclear();
				printw("Setting Stabilize mode\n");
				modeparse.data = "STABILIZE";
				modereq_pub.publish(modeparse);
				usleep(10000);
				//override RC_VALUES:
				//asctec_ground_station::RC_in rc_inp;
				//rc_inp.rcvalues = {rc_trim,rc_trim, rc_trim, rc_trim};
				printw("Rcoverride");
				for(int i=0;i < THROTCHAN;i++)
				{
					rcmsg.rcvalues[i] = RC_TRIM[i];
				}
				while(rcmsg.rcvalues[THROTCHAN] > RC_MIN[THROTCHAN])
				{
					rcoverride_pub.publish(rcmsg);	
					rcmsg.rcvalues[THROTCHAN] -= 10;
					//mvprintw("%u",rcmsg.rcvalues[THROTCHAN]);
					mvprintw(1,0,"RC_1: %lu\t RC_2: %lu\t RC_3 %lu\t RC_4 %lu\t",rcmsg.rcvalues[0], rcmsg.rcvalues[1], rcmsg.rcvalues[2], rcmsg.rcvalues[3]);
					refresh();
					usleep(50000);
				}
				timeout(-1);
				goto DISARM;
				break;
			case 'q':
				goto QUIT;
				break;
		}
		refresh();
		//usleep(100000);
	}

QUIT:
	//disarm and quit
	pthread_mutex_lock(&setptctrl_mutex);
	setptctrl_active = false;
	pthread_mutex_unlock(&setptctrl_mutex);
	statuszoneclear();
	printw("Disarming\n");
	cmdparse.data = "DISARM";
	for(int i =0;i<10;i++)
		commandreq_pub.publish(cmdparse);//attempting to disarm 5 times if the prev one did not succeed
	printw("Quitting\n");
	//dataparseval.data = "ATTITUDE STOP";
	//datareq_pub.publish(dataparseval);
	//dataparseval.data = "EXTENDED STOP";
	//datareq_pub.publish(dataparseval);
	refresh();
	usleep(5000000);
	endwin();

	return 0;
	
}
