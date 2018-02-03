///////////////////sdd
//L OCALIZATIONS
//
//s.shimizu
//t.hagiwara
//
//azimuth estimation + odometryでのlocalization
//tf名は
//		header	/map 
//		child	/matching_base_link
//
														


//lcl[0]:odom
//lcl[1]:odom+amu
//lcl[2]:odom+amu+ndt


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
#include <geometry_msgs/Pose.h>

#include "std_msgs/Int32.h"

#include <complement/localization.h>

using namespace std;	

bool error_flag=false;
bool prepare_flag=true;
bool is_started=false;
bool ndt_flag=false;

bool alter_flag = false;
bool start_flag = true;

bool yaw_first_flag = true;
bool w_first_flag = true;

tfScalar yaw_first;
double w_first;

const size_t N = 4;
Localization lcl[N];



geometry_msgs::Pose2D init_pose; // [deg] This expresses a position and orientation on a 2D manifold.

nav_msgs::Odometry lcl_ini;
nav_msgs::Odometry lcl_;
nav_msgs::Odometry lcl_1;
nav_msgs::Odometry lcl_2;
nav_msgs::Odometry lcl_3;
// nav_msgs::Odometry lcl_vis;
geometry_msgs::Pose lcl_vis;


sensor_msgs::Imu imu_msg;
std_msgs::Int32 number;

double yaw_before = 0;
double d_yaw = 0;
int num = 0;


template <class T>
void getParam(ros::NodeHandle &n, string param, T &val, bool* flag)
{
    // string str;
    double prm;
    if(!n.hasParam(param)){
        std::cout << param << " don't exist." << std::endl;
        *flag=true;
    }

    // if(!n.getParam(param, str)){
    if(!n.getParam(param, prm)){
        std::cout << "NG" << std::endl;
        *flag=true;
    }
    // std::stringstream ss(str);
    // T rsl;
    // ss >> rsl;
    // val = rsl;
    val = prm;
    std::cout << param << " = " << val << std::endl;
}

void setInitPose(ros::NodeHandle &n, geometry_msgs::Pose2D& init_pose)
{
 
    bool error_flag=false;
    getParam(n, "/init_x", init_pose.x, &error_flag);
    getParam(n, "/init_y", init_pose.y, &error_flag);
    getParam(n, "/init_yaw", init_pose.theta, &error_flag);
}


void imu_callback(const sensor_msgs::Imu &imu){//pitch だから前後の縦揺れ
	is_started = true;
	fflush(stdout);


    tf::Quaternion orientation;
    tf::quaternionMsgToTF(imu.orientation, orientation);
    tfScalar yaw, pitch, roll;
    tf::Matrix3x3(orientation).getEulerYPR(yaw, pitch, roll);


    if(yaw_first_flag){
        yaw_first = yaw;

        yaw_first_flag = false;
 
    }
	
    yaw = yaw - yaw_first;
    d_yaw = yaw - yaw_before;
    yaw_before = yaw;

    if(d_yaw < -0.0010 || 0.0010 < d_yaw){ //小さいほうがndtが吹っ飛ぶ前に補正できる//1[deg] = 0.017[rad]
    num = 10;
    
    }

    else num = 0;
    if(alter_flag){
 
 //       cout<<"gyro_lcl"<<endl;
        lcl[0].yaw = yaw + init_pose.theta*M_PI/180.0;
        lcl[0].pitch = pitch;
        lcl[0].roll = roll;
        cout << "change" << endl;
        lcl[0].altering3();
    }

    //cout<<"d_yaw補正"<<lcl[1].d_yaw<<endl;
    prepare_flag=false;
}

void odom_callback(nav_msgs::Odometry::ConstPtr msg){
	is_started = true;
	fflush(stdout);
	
    if(w_first_flag){
        w_first = msg->twist.twist.angular.z;
        w_first_flag = false;
    }	
 
    if(alter_flag){ 
        lcl[0].v = msg->twist.twist.linear.x;
        lcl[0].w = msg->twist.twist.angular.z - w_first;
    }

    prepare_flag=false;

}

void ekf_callback(const nav_msgs::Odometry msg){
	is_started = true;

	fflush(stdout);
	
    prepare_flag=false;

    lcl[0].x = msg.pose.pose.position.x;
	lcl[0].y = msg.pose.pose.position.y;
	lcl[0].yaw = msg.pose.pose.orientation.z;


}

/*
void ekf_callback(const nav_msgs::Odometry msg){
	is_started = true;

	fflush(stdout);
	
    prepare_flag=false;



    if((13 < msg.pose.pose.position.x && msg.pose.pose.position.x < 27)&&(4 < msg.pose.pose.position.y && msg.pose.pose.position.y < 16)){
        
        if(start_flag){ 
            
            cout<<"---gyro_start----"<<endl;
            lcl[0].v = 0;
            lcl[0].w = 0;
            lcl[0].pitch = 0;
            lcl[0].dt = 0;
            start_flag = false;
        }     

        alter_flag = true;
    
    }
    
    else{

        lcl[0].x = msg.pose.pose.position.x;
	    lcl[0].y = msg.pose.pose.position.y;
	    lcl[0].yaw = msg.pose.pose.orientation.z;

        alter_flag = false;
        // alter_flag = true;
        start_flag = true; 
    }

}
*/



int main (int argc, char** argv)
{
	ros::init(argc, argv, "sq_local");
  	ros::NodeHandle n;
	ros::Rate roop(100);
	
    setInitPose(n, init_pose);


//Localization lcl[N] 格納;

	for(size_t i=0;i<N;i++){
		lcl[i].x =   init_pose.x;
		lcl[i].y =   init_pose.y;
		//lcl[i].yaw = init_pose.theta*M_PI/180.0;
		lcl[i].yaw = init_pose.theta*M_PI/180.0;
	}
	
	for(size_t i=0;i<N;i++)
		lcl[i].start();
	
	lcl_ini.pose.pose.position.x = init_pose.x;
	lcl_ini.pose.pose.position.y = init_pose.y;
	lcl_ini.pose.pose.orientation.z = init_pose.theta*M_PI/180.0;//初期方位[rad]が入っている

    
	ros::Subscriber odm_sub = n.subscribe("/odom", 100, odom_callback);
	ros::Subscriber amu_sub = n.subscribe("/imu/data", 100, imu_callback);
	// ros::Subscriber ndt_sub = n.subscribe("/sq_ndt_data", 100, ndt_callback);
    
    ros::Subscriber ekf_sub = n.subscribe("/ekf_NDT", 100, ekf_callback);

    // ros::Subscriber num_pub    = n.subscribe("/waypoint/now", 1, wpCallback);
	



	ros::Publisher lcl_pub = n.advertise<nav_msgs::Odometry>("/lcl_sq_ekf", 10);


	ros::Publisher lcl_hantei_pub = n.advertise<std_msgs::Int32>("/hantei", 10);//0:直線 1:カーブ
	ros::Publisher lcl_vis_pub = n.advertise<geometry_msgs::Pose>("/lcl_sq_vis", 10);

	tf::TransformBroadcaster br;
	tf::Transform transform; //位置と姿勢を持つ座標系を表すクラス
	
    tf::Quaternion q;
	
    lcl_.header.frame_id = "map";
	lcl_.child_frame_id = "matching_base_link";
	// lcl_.child_frame_id = "centerlaser_";
    

	unsigned int cnt = 0;


//	FILE *fp_1;
	FILE *fp_2;
	fp_2 = fopen("./position.csv","w");

	//fprintf(fp_2, "W.O._x,W.O_y,G.O._x,G.O._y,ndt._x,ndt._y\n");
	fprintf(fp_2, "G.O._x,G.O._y,G.O._yaw,ndt._x,ndt._y,ndt._yaw,do\n");
	//fprintf(fp_2, "W.O._x,W.O_y,w_yaw,G.O._x,G.O._y,G_yaw\n");


    while(ros::ok()){
        
        if(prepare_flag){
        //    cout<<"prepare"<<endl; 
			transform.setOrigin( tf::Vector3(init_pose.x, init_pose.y, 0.0) );
			tf::Quaternion q;
			q.setRPY(0, 0, init_pose.theta*M_PI/180.0);
			
			transform.setRotation(q);
			br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "matching_base_link"));
			// br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "centerlaser_"));
		
        }

		if(is_started){


    
            lcl_.header.stamp = ros::Time::now(); //timestampのメッセージを送ろうとしている 例）1370000000
		    lcl_.pose.pose.position.x = lcl[0].x;
		    lcl_.pose.pose.position.y = lcl[0].y;
		    lcl_.pose.pose.position.z = 0.0;
		    lcl_.pose.pose.orientation.z = lcl[0].yaw;
		   		
		    lcl_pub.publish(lcl_);


            // cout<<"z:"<<lcl_3.pose.pose.position.z<<endl;
	
            transform.setOrigin( tf::Vector3(lcl[0].x, lcl[0].y, 0.0) );
		    q.setRPY(0, 0, lcl[0].yaw);
		   
           
            transform.setRotation(q);
		   // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "centerlaser_"));
		    br.sendTransform(tf::StampedTransform(transform, lcl_.header.stamp , "map", "matching_base_link"));

            


//////////////visualize
            lcl_vis = lcl_.pose.pose;

			lcl_vis_pub.publish(lcl_vis);


            number.data = num;

            lcl_hantei_pub.publish(number);//0:直線 1:カーブ
            
		}
		
        cnt++;
        
        roop.sleep();
		ros::spinOnce();
	}

	fclose(fp_2);
    
    return 0;

}


