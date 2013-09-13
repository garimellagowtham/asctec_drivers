#ifndef SETPTCTRL_H
#define SETPTCTRL_H
#include "ros/ros.h"
#include "ros/time.h"
#include "ncurses.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/transform_datatypes.h"
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>
//#include <DspFilters/Butterworth.h>
#include "fstream"
#include "iostream"
#include <ncurses.h>

#define PI 3.14159
/*
 *
*/
using namespace tf;
using namespace std;
class SetptCtrl
{
	public:
		float kpr,kpt,kdr,kdt;
		float battvoltage;
		float mbatt,cbatt;
		double tprev;
		Vector3 Fext;
		Vector3 rpycommand;
		Vector3 prev_position;
		double throtcommand;
		bool freshdata;
		bool criticalvoltage;
	protected:
		Vector3 goal_position;
		ros::NodeHandle *n;
		//ros::Publisher posn_pub;
		ros::Publisher marker_pubv;
		ros::Publisher velocity_pub;
		ros::Publisher vrpnrpy_pub;
		visualization_msgs::Marker velmarker;
		//ros::Publisher marker_pubdx;
		ofstream ofile;
		//int countline;
		//double basetime;
		Transform goal_frame;
		//float *tbuf;
		float posfiltbuf[3][4];
		float posbuf[3][4];
		//float posbuf[3][3];
		int inds[4];
		float doteval(float*,float*,int);
		void mapindex(int);
		geometry_msgs::Point p1;
		geometry_msgs::Point p2;
		geometry_msgs::Vector3 vel;
		geometry_msgs::Vector3 rpymsg;
		//Dsp::SimpleFilter<Dsp::Butterworth::Design::LowPass <3>,3> butter;
	public:
		SetptCtrl();
		SetptCtrl(ros::NodeHandle *n1);
		void Set(const geometry_msgs::TransformStamped::ConstPtr &currframe);
		void voltageupdate( const std_msgs::Float32 &currvoltage);
		void setgoal(const Vector3 &v1);
		void setgoal(float xg, float yg, float zg);
		void applybutter(const Vector3 dx, Vector3 &dxfilt);
		~SetptCtrl();
	
};
#endif
