//LOCALIZATIONS
//s.shimizu
//t.hagiwara
//
//azimuth estimation + odometryでのlocalization
//tf名は
//		header	/map 
//		child	/matching_base_link
//
														

#include <stdio.h>
#include <iostream>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>
#include <ceres_msgs/AMU_data.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose2D.h>

#include <localization/localization.h>

using namespace std;	

//const size_t N = 3;
const size_t N = 4;
bool is_started = true;
bool DGauss_flag = false;
bool DGauss_yaw_flag = false;
bool init_flag=false;
Localization lcl[N];
//double initialpose[3] = { 95.278519 , -18.005138 , ( 173.7 * M_PI/ 180.0) };//ikucha
geometry_msgs::Pose2D init_pose;
nav_msgs::Odometry gps_odom_msgs_;
nav_msgs::Odometry gps_msgs;
nav_msgs::Odometry DGauss_pose;
nav_msgs::Odometry DGauss_yaw;
nav_msgs::Odometry lcl_ini;

template <class T>
void getParam(ros::NodeHandle &n, string param, T &val, bool* flag)
{
    string str;
    if(!n.hasParam(param)){
        std::cout << param << " don't exist." << std::endl;
        *flag=true;
    }

    if(!n.getParam(param, str)){
        std::cout << "NG" << std::endl;
        *flag=true;
    }
    std::stringstream ss(str);
    T rsl;
    ss >> rsl;
    val = rsl;
    std::cout << param << " = " << str << std::endl;
}

void setInitPose(ros::NodeHandle &n, geometry_msgs::Pose2D& init_pose)
{
    bool error_flag=false;
    getParam(n, "init_x", init_pose.x, &error_flag);
    getParam(n, "init_y", init_pose.y, &error_flag);
    getParam(n, "init_yaw", init_pose.theta, &error_flag);
	init_flag=true;
}

void amu_callback(ceres_msgs::AMU_data::ConstPtr msg){
	is_started = true;
	fflush(stdout);
	lcl[0].pitch = M_PI * msg->pitch / 180.0;
	lcl[1].pitch = M_PI * msg->pitch / 180.0;
	lcl[2].pitch = M_PI * msg->pitch / 180.0;
	lcl[3].pitch = M_PI * msg->pitch / 180.0;
	lcl[1].w = - M_PI * ( msg->dyaw - 0.275030001 ) / 180.0;
	//lcl[1].w = - M_PI * ( msg->dyaw - 0.265126523 ) / 180.0;
	//lcl[1].w += 0.0041196016 - 0.00025 + 0.000171428; //correction
	lcl[1].altering();
}

void odom_callback(nav_msgs::Odometry::ConstPtr msg){
	is_started = true;
	printf("\r□ ■ □");
	fflush(stdout);
	lcl[0].v = msg->twist.twist.linear.x;
	lcl[1].v = msg->twist.twist.linear.x;
	lcl[2].v = msg->twist.twist.linear.x;
	lcl[3].v = msg->twist.twist.linear.x;

	lcl[0].w = msg->twist.twist.angular.z;
	//lcl[1].w += 0.00222222;
	lcl[0].altering();
//	lcl[1].showState();

//	cout<<msg->pose.pose.position.x<<", "<<msg->pose.pose.position.y<<endl;
}

void azm_callback(sensor_msgs::Imu::ConstPtr msg){	
	if (init_flag){
		is_started = true;
		printf("\r□ □ ■");
		fflush(stdout);
		lcl[2].yaw = (init_pose.theta + msg->orientation.z)*M_PI / 180.0;
		lcl[2].altering();
		lcl[3].yaw = (init_pose.theta + msg->orientation.z)*M_PI / 180.0;
		lcl[3].altering2();
//		lcl[2].showState();
	}
}
/*
void gps_callback(const nav_msgs::OdometryConstPtr msg)
{
	gps_msgs = *msg;
	lcl[2].x = gps_msgs.pose.pose.position.x;
	lcl[2].y = gps_msgs.pose.pose.position.y;
}
*/

void DGauss_callback(const nav_msgs::Odometry msg){
	DGauss_pose.pose.pose.position.x = msg.pose.pose.position.x;
	DGauss_pose.pose.pose.position.y = msg.pose.pose.position.y;
	DGauss_flag = true;
}

void DGauss_yaw_callback(const nav_msgs::Odometry msg){
	DGauss_yaw.pose.pose.orientation.z = msg.pose.pose.orientation.z;
	DGauss_yaw_flag = true;
}

int main (int argc, char** argv)
{
	ros::init(argc, argv, "localization_test");
  	ros::NodeHandle n;
	ros::Rate roop(100);
	
    setInitPose(n, init_pose);
	
	for(size_t i=0;i<N;i++){
		lcl[i].x =   init_pose.x;
		lcl[i].y =   init_pose.y;
		lcl[i].yaw = init_pose.theta*M_PI/180.0;
	}
	//previous version//
	//for(size_t i=0;i<N;i++){
	//	lcl[i].x =   initialpose[0];
	//	lcl[i].y =   initialpose[1];
	//	lcl[i].yaw = initialpose[2];
	//}
	
	for(size_t i=0;i<N;i++)
		lcl[i].start();
	
	lcl_ini.pose.pose.position.x = init_pose.x;
	lcl_ini.pose.pose.position.y = init_pose.y;
	lcl_ini.pose.pose.orientation.z = init_pose.theta*M_PI/180.0;//初期方位[rad]が入っている
	//lcl_ini.pose.pose.position.x = initialpose[0];
	//lcl_ini.pose.pose.position.y = initialpose[1];
	//lcl_ini.pose.pose.orientation.z = initialpose[2];//初期方位[rad]が入っている

	vector<ros::Publisher> pub;
	vector<sensor_msgs::PointCloud> pc;

	pub.resize(N);
	pc.resize(N);

	pub[0] = n.advertise<sensor_msgs::PointCloud>("/odm_lcl", 100);
	pub[1] = n.advertise<sensor_msgs::PointCloud>("/amu_lcl", 100);
	pub[2] = n.advertise<sensor_msgs::PointCloud>("/azm_lcl", 100);

	ros::Subscriber odm_sub = n.subscribe("/tinypower/odom", 100, odom_callback);
	ros::Subscriber amu_sub = n.subscribe("/AMU_data", 100, amu_callback);
	ros::Subscriber azm_sub = n.subscribe("/perfect_velodyne/filtered_pose", 100, azm_callback);
	ros::Subscriber DGauss_sub = n.subscribe("/gauss_sphere/pose", 100, DGauss_callback);
	ros::Subscriber DGauss_yaw_sub = n.subscribe("/gauss_sphere/yaw", 100, DGauss_yaw_callback);

//	ros::Subscriber gps_sub = n.subscribe("gps/xy", 100, gps_callback);
//	ros::Subscriber odm_sub = n.subscribe("/tinypower/state", 100, odom_callback);
	
	ros::Publisher pub_lcl = n.advertise<nav_msgs::Odometry>("/lcl", 10);
	ros::Publisher pub_lcl2 = n.advertise<nav_msgs::Odometry>("/lcl2", 10);
	ros::Publisher pub_lcl_ini = n.advertise<nav_msgs::Odometry>("/lcl/initial_pose", 10);
	
	
	tf::TransformBroadcaster br;
	tf::Transform transform;
	nav_msgs::Odometry lcl_;
	lcl_.header.frame_id = "map";
	lcl_.child_frame_id = "matching_base_link";
	nav_msgs::Odometry lcl2_;
	lcl2_.header.frame_id = "map";
	lcl2_.child_frame_id = "matching_base_link";

	unsigned int cnt = 0;

//	FILE *fp_1;
	FILE *fp_2;
	fp_2 = fopen("./position.csv","w");

	fprintf(fp_2, "W.O._x,W.O_y,G.O._x,G.O._y,Pro._x,Pro._y\n");


	while(ros::ok()){
		if(is_started){
			if(cnt % 50 == 0){
				fprintf(fp_2, "%lf,%lf,%lf,%lf,%lf,%lf\n", lcl[0].x, lcl[0].y, lcl[1].x, lcl[1].y, lcl[2].x, lcl[2].y);
				cnt = 0;
			}
			if(DGauss_flag){
				lcl[1].x = DGauss_pose.pose.pose.position.x;
				lcl[1].y = DGauss_pose.pose.pose.position.y;
				//lcl[1].yaw += DGauss_pose.pose.pose.orientation.z;
				
				//cout<<"DGauss_pose.pose.pose.position.x : "<<DGauss_pose.pose.pose.position.x<<endl;
				//cout<<"DGauss_pose.pose.pose.position.y : "<<DGauss_pose.pose.pose.position.y<<endl;
				cout<<"yaw : "<<lcl[1].yaw<<endl;
				
				DGauss_flag = false;
			}
			if(DGauss_yaw_flag){
				//lcl[1].x = DGauss_pose.pose.pose.position.x;
				//lcl[1].y = DGauss_pose.pose.pose.position.y;
				lcl[1].yaw += DGauss_yaw.pose.pose.orientation.z;
				
				//cout<<"DGauss_pose.pose.pose.position.x : "<<DGauss_pose.pose.pose.position.x<<endl;
				//cout<<"DGauss_pose.pose.pose.position.y : "<<DGauss_pose.pose.pose.position.y<<endl;
				cout<<"yaw : "<<lcl[1].yaw<<endl;
				
				DGauss_yaw_flag = false;
			}
			lcl_.header.stamp = ros::Time::now();
			lcl_.pose.pose.position.x = lcl[1].x;
			lcl_.pose.pose.position.y = lcl[1].y;
			lcl_.pose.pose.position.z = 0.0;
			lcl_.pose.pose.orientation.z = lcl[1].yaw;
			//lcl_.pose.pose.position.x = lcl[2].x;
			//lcl_.pose.pose.position.y = lcl[2].y;
			//lcl_.pose.pose.position.z = 0.0;
			//lcl_.pose.pose.orientation.z = lcl[2].yaw;
					
			pub_lcl.publish(lcl_);
			pub_lcl_ini.publish(lcl_ini);
			
			lcl2_.header.stamp = ros::Time::now();
			lcl2_.pose.pose.position.x = lcl[3].x;
			lcl2_.pose.pose.position.y = lcl[3].y;
			lcl2_.pose.pose.position.z = 0.0;
			lcl2_.pose.pose.orientation.z = lcl[3].yaw;
			pub_lcl2.publish(lcl2_);
			
			transform.setOrigin( tf::Vector3(lcl[1].x, lcl[1].y, 0.0) );
			tf::Quaternion q;
			q.setRPY(0, 0, lcl[1].yaw);
			//transform.setOrigin( tf::Vector3(lcl[2].x, lcl[2].y, 0.0) );
			//tf::Quaternion q;
			//q.setRPY(0, 0, lcl[2].yaw);
			
			transform.setRotation(q);
			br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "matching_base_link"));
		
			cnt ++;
		}
		roop.sleep();
		ros::spinOnce();
	}
//	fclose(fp_1);
	fclose(fp_2);
	return 0;

}

