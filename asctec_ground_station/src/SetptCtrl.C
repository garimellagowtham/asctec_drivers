#include "SetptCtrl.h" 

//static float butternumcoeff[4] = {0.0053905   ,0.0161714   ,0.0161714   ,0.0053905};
static float butternumcoeff[4] = {0.018935   ,0.056805   ,0.056805   ,0.018935};
//static float butterdenomcoeff[3] = {-2.20590   ,1.69762  ,-0.44734};
static float butterdenomcoeff[3] = {-1.73832   ,1.16129  ,-0.27149};

SetptCtrl::SetptCtrl(ros::NodeHandle *n1)
{
	n = n1;
	kpr = 1.0;
	kpt = 0.5;
	kdr = 1.0;
	kdt = 0.5;
	//countline = 0;
	freshdata = false;
	goal_frame.setIdentity();
	goal_position.setValue(0.0f,0.0f,0.0f);
	rpycommand.setValue(0.0f,0.0f,0.0f);
	Fext.setValue(0.0f,0.0f,-0.4*9.81);
	mbatt = 0.5;
	cbatt = 5;
	battvoltage = 10.5;//default
	criticalvoltage = false;
	//tbuf = new float[4]{0};
	//posbuf = new float[3][3] {0};
	for(int i =0;i<3;i++)
	{
		for(int j=0;j<4;j++)
		{
			posbuf[i][j] = 0;
			posfiltbuf[i][j] = 0;
		}
	}
/*	for(int i = 0;i<3;i++)
	{
		posbuf[i] = new float[4]{0};
		posfiltbuf[i] = new float[4] {0};//although only 3 are reqd
	}
	*/
	
	//publisher for velocity and posn info:
	//vel_pub = n1->advertise<geometry_msgs::Vector3>("vel",100);
	marker_pubv = n->advertise<visualization_msgs::Marker>("velocityo", 1);
	//velocity_pub = n->advertise<geometry_msgs::Vector3>("velfilt", 10);
	vrpnrpy_pub = n->advertise<geometry_msgs::Vector3>("vrpnrpy",10);
	ctrlcmd_pub = n->advertise<asctec_ground_station::ctrl_command>("rpyctrl",10);
	//set velocity marker configuration
	velmarker.header.frame_id = "/optitrak";
	velmarker.header.stamp = ros::Time::now();
	velmarker.ns = "gcs";
	velmarker.id = 0;
	velmarker.type = visualization_msgs::Marker::ARROW;
	velmarker.action = visualization_msgs::Marker::ADD;
	velmarker.scale.x = 0.05;
	velmarker.scale.y = 0.1;
	velmarker.scale.z = 0.05;
	//set color:
	velmarker.color.r = 1.0f;
	velmarker.color.g = 1.0f;
	velmarker.color.b = 0.0f;
	velmarker.color.a = 1.0f;
	//lifetime
	velmarker.lifetime = ros::Duration();

	velmarker.points.resize(2);//creating capacity for two points
//	marker_pubdx = n.advertise<visualization_msgs::Marker>("desiredvecwithoutFext", 1);
	//setup butterworth filter:
	//butter.setup(
	//posn_pub = n1->advertise<geometry_msgs::Vector3>("posn",100);
	//open file
	ofile.open("/home/gowtham/Documents/posn.dat", ios_base::out| ios_base::trunc);
}

SetptCtrl::SetptCtrl()
{
	kpr = 1.0;
	kpt = 0.5;
	kdr = 1.0;
	kdt = 0.5;
	//countline = 0;
	freshdata = false;
	goal_frame.setIdentity();
	goal_position.setValue(0.0f,0.0f,0.0f);
	rpycommand.setValue(0.0f,0.0f,0.0f);
	Fext.setValue(0.0f,0.0f,-0.4*9.81);
	mbatt = 0.5;
	cbatt = 5;
	battvoltage = 10.5;//default
	criticalvoltage = false;
	//tbuf = new float[4]{0};
	//posbuf = new float[3][3] {0};
	for(int i =0;i<3;i++)
	{
		for(int j=0;j<4;j++)
		{
			posbuf[i][j] = 0;
			posfiltbuf[i][j] = 0;
		}
	}
}

SetptCtrl::~SetptCtrl()
{
	ofile.close();
}

static Vector3 getrpy(Transform t1)
{
	Matrix3x3 rotmat = t1.getBasis();
	Vector3 result(0.0f,0.0f,0.0f);
	rotmat.getEulerYPR(result[2],result[1],result[0]);
	return result;
}
static void gettransformtogoal(Transform &t2, Vector3 desiredvec, float curryaw)
{
	Vector3 yawvec(cos(curryaw),sin(curryaw),0);
	Matrix3x3 basis;
	basis[2] = desiredvec;
	basis[1] = tfCross(basis[2],yawvec).normalize();//desiredvec is normalized and is towards z axis
	basis[0] = tfCross(basis[1],basis[2]);
	t2.setBasis(basis.transpose());
	return;
}

void SetptCtrl::mapindex(int framecount)
{
	for(int i = 0;i<4;i++)
	{
		inds[i] = (framecount+4-i)%4;
	}
	return;
}

float SetptCtrl::doteval(float *coeff, float *x,int size)
{
	//size is the number of terms or order+1;
	float result = 0;
	for(int i = 4-size;i<4;i++) //max size is 4
	{
		result += coeff[i-(4-size)]*x[inds[i]];
	}
	return result;
}

void SetptCtrl::applybutter(const Vector3 dx, Vector3 &dxfilt)
{
	static int framecount = 0;
	SetptCtrl::mapindex(framecount);
	//cout<<"inds"<<inds[0]<<inds[1]<<inds[2]<<inds[3]<<endl;
	//debug statements:
	//plot
	//updatebuf
	//tbuf[inds[0]]= time;//latest time
	for(int i = 0;i<3;i++)
	{
		posbuf[i][inds[0]] = dx[i];
		/*
		cout<<i<<endl;
		cout<<"posbuf:"<<endl;
		for(int j =0;j<4;j++)
		{
			cout<<posbuf[i][j]<<"\t";
		}
		cout<<endl;
		cout<<"posfiltbuf:"<<endl;
		for(int j =0;j<4;j++)
		{
			cout<<posfiltbuf[i][j]<<"\t";
		}
		cout<<endl;
		*/
		dxfilt[i] = SetptCtrl::doteval(butternumcoeff,posbuf[i],4)-SetptCtrl::doteval(butterdenomcoeff,posfiltbuf[i],3);
		posfiltbuf[i][inds[0]] = dxfilt[i];
	}
	if(framecount < 3)
	{
		framecount++;
	}
	else
	{
		framecount = 0;
		//exit(0);
	}
}

void SetptCtrl::voltageupdate(const std_msgs::Float32 &currvoltage)
{
	battvoltage = currvoltage.data;
	//mvprintw(3,0,"Battery Voltage: %f",battvoltage);
	//update Fext based on battvoltage;
	Fext[2] = -(cbatt + mbatt*(10.5 -battvoltage));//approximately around 10.5 V 
	if(battvoltage < 9.5)
		criticalvoltage = true;
}

void SetptCtrl::Set(const geometry_msgs::TransformStamped::ConstPtr &currframe)
{
	freshdata = true;
	//static tf::TransformListener listener;
	//StampedTransform UV_O;
	//try{
		//listener.lookupTransform("optitrak","APM2_VRPN",ros::Time(0),UV_O);
	//}
	//catch (tf::TransformException ex) {
		//return;
		//ROS_ERROR("\n%s",ex.what());
	//}
	//geometry_msgs::TransformStamped framecopy = *currframe;
	//tf::Stamped<geometry_msgs::TransformStamped> convcurrframe(framecopy);
	//tf::Stamped<geometry_msgs::TransformStamped> convcurrframe();
	//ROS_INFO("Hai");
	tf::StampedTransform UV_O;
	transformStampedMsgToTF(*currframe,UV_O);//converts to the right format 
	//getrpy:
	Vector3 rpy = getrpy((Transform)UV_O);
	//publish rpy for checking consistency:
	vector3TFToMsg(rpy,rpymsg);
	rpymsg.x *= 180/PI;
	rpymsg.y *= 180/PI;
	rpymsg.z *= 180/PI;
	vrpnrpy_pub.publish(rpymsg);
	//compute the desired roll pitch yaw to go to goal position:
	Vector3 dx = goal_position - UV_O.getOrigin();//wrt optitrak
	Vector3 dxfilt = dx;
	//apply butterworth filter 3rd order:
	//SetptCtrl::applybutter(dx, dxfilt);
	double tcurr = currframe->header.stamp.toSec();
	Vector3 ddx = (prev_position - dxfilt)/(tcurr - tprev);
	prev_position = dxfilt;
	tprev = tcurr;
	//desired vec:
	Vector3 desiredvec = -Fext + Vector3(kpr*dxfilt[0],kpr*dxfilt[1],kpt*dxfilt[2]) -Vector3(kdr*ddx[0],kdr*ddx[1],kdt*ddx[2]); 
	//set throttle:
	throtcommand = tfDot(desiredvec,Vector3(0,0,1.0));
	Transform t2;//create a transform for finding the action
	gettransformtogoal(t2,desiredvec/desiredvec.length(),rpy[2]);//using current yaw
	rpycommand = getrpy(t2);
	ctrl_cmd.roll = rpycommand[0];
	ctrl_cmd.pitch = rpycommand[1];
	ctrl_cmd.rateyaw = rpycommand[2];
	ctrl_cmd.thrust = throtcommand;
	ctrlcmd_pub.publish(ctrl_cmd);
	//Publish rpyt commands:
	//mvprintw(7,0,"current rpy: %f\t%f\t%f",rpy[0],rpy[1],rpy[2]);
	//convert rpy to the imu frame for commanding purposes:
	//double temp = rpycommand[0];
	//rpycommand[0] = rpycommand[1];
	//rpycommand[1] = temp;//exchange roll and pitch to get the values wrt imu frame;
	//cout<<throtcommand<<"\t"<<rpycommand[0]*(180/PI)<<"\t"<<rpycommand[1]*(180/PI)<<"\t"<<rpycommand[2]*(180/PI)<<"\t"<<endl;
	//Broadcasting
	static tf::TransformBroadcaster broadcaster;
	broadcaster.sendTransform(tf::StampedTransform(goal_frame,ros::Time::now(),"optitrak","goal_posn"));
	//publish desired frame whose z axis is along the desired vector
	t2.setOrigin(UV_O.getOrigin());
	broadcaster.sendTransform(tf::StampedTransform(t2,ros::Time::now(),"optitrak","desiredframe"));
	// publish markers for visualizing velocities:
	pointTFToMsg(UV_O.getOrigin(),p1);
	p2.x = UV_O.getOrigin().x() + ddx[0];
	p2.y = UV_O.getOrigin().y() + ddx[1];
	p2.z = UV_O.getOrigin().z() + ddx[2];
	velmarker.points[0] = p1;
	velmarker.points[1] = p2; 
	marker_pubv.publish(velmarker);
	//publish the velocity for visualization:
	//vector3TFToMsg(dxfilt,vel);
	//velocity_pub.publish(vel);
	/*vector3TFToMsg(ddx,vel);
	velocity_pub.publish(vel);
	*/
	//cout<<setprecision(13);
	ofile<<setprecision(14);
	ofile<<tcurr<<"\t"<<dx[0]<<"\t"<<dx[1]<<"\t"<<dx[2]<<"\t"<<dxfilt[0]<<"\t"<<dxfilt[1]<<"\t"<<dxfilt[2]<<"\t"<<ddx[0]<<"\t"<<ddx[1]<<"\t"<<ddx[2]<<"\t"<<desiredvec[0]<<"\t"<<desiredvec[1]<<"\t"<<desiredvec[2]<<endl;
	//ofile<<currframe->header.stamp.toSec()<<"\t"<<rpy[0]<<"\t"<<rpy[1]<<"\t"<<rpy[2]<<endl;//roll
		ofile.flush();
	//cout<<tcurr<<"\t"<<dxfilt[0]<<"\t"<<dxfilt[1]<<"\t"<<dxfilt[2]<<endl;//roll
		//usleep(5000);
}
void SetptCtrl::setgoal(const Vector3 &v1)
{
	goal_position = v1;//copy constructor
	goal_frame.setOrigin(goal_position);
}
void SetptCtrl::setgoal(float xg, float yg, float zg)
{
	goal_position.setValue(xg,yg,zg);
	goal_frame.setOrigin(goal_position);
}


