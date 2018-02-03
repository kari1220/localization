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

bool yaw_first_flag = true;
bool w_first_flag = true;

tfScalar yaw_first;
double w_first;

const size_t N = 5;
Localization lcl[N];

geometry_msgs::Pose2D init_pose; // [deg] This expresses a position and orientation on a 2D manifold.

nav_msgs::Odometry lcl_ini;
nav_msgs::Odometry lcl_;
nav_msgs::Odometry lcl_1;
nav_msgs::Odometry lcl_2;
nav_msgs::Odometry lcl_3;
nav_msgs::Odometry lcl_4;
// nav_msgs::Odometry lcl_vis;
geometry_msgs::Pose lcl_vis;


sensor_msgs::Imu imu_msg;
std_msgs::Int32 number;

double yaw_before = 0;
int num = 0;
int score = 0;


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


//void imu_callback(const sensor_msgs::Imu::ConstPtr& msg){
// void imu_callback(const sensor_msgs::Imu &imu){//pitch だから前後の縦揺れ
// void amu_callback(const ceres_msgs::AMU_data &msg){
void amu_callback(ceres_msgs::AMU_data::ConstPtr msg){
    is_started = true;
	fflush(stdout);

	
	lcl[0].pitch = M_PI * msg->pitch / 180.0;
	lcl[1].pitch = M_PI * msg->pitch / 180.0;
	lcl[2].pitch = M_PI * msg->pitch / 180.0;

	lcl[1].w = - M_PI * (msg->dyaw - 0.273163) / 180.0;
	// lcl[2].w = - M_PI * (msg->dyaw - 0.273163) / 180.0;


	lcl[1].altering();
	// lcl[2].altering();
    // lcl[1].altering3();


    cout <<"dyaw"<<lcl[1].w<<endl;
    if(lcl[1].w < -0.05 || 0.05 < lcl[1].w){ //小さいほうがndtが吹っ飛ぶ前に補正できる//1[deg] = 0.017[rad]
    // if(lcl[1].w < -0.10 || 0.10 < lcl[1].w){ //小さいほうがndtが吹っ飛ぶ前に補正できる//1[deg] = 0.017[rad]
    // num = 10;
    num = 0;

    }

    // if(lcl[1].w < -0.20 || 0.20 < lcl[1].w){ //小さいほうがndtが吹っ飛ぶ前に補正できる//1[deg] = 0.017[rad]
    // num = 10;
    //
    // }
    //
    else num = 0;
    //cout<<"d_yaw補正"<<lcl[1].d_yaw<<endl;
    prepare_flag=false;
}

void odom_callback(nav_msgs::Odometry::ConstPtr msg){
	is_started = true;
	fflush(stdout);
	
    lcl[0].v = msg->twist.twist.linear.x;
	lcl[1].v = msg->twist.twist.linear.x;
	lcl[2].v = msg->twist.twist.linear.x;
	

    //lcl_.header.stamp = msg->header.stamp;

    //lcl[2].v = msg->twist.twist.linear.x;

    // if(w_first_flag){
    //     w_first = msg->twist.twist.angular.z;
    //
    //     w_first_flag = false;
    // }
	
    // lcl[0].w = msg->twist.twist.angular.z - w_first;
    lcl[0].w = msg->twist.twist.angular.z;


    //cout<<"odomd_yaw補正"<<lcl[0].w<<endl;

    
    lcl[0].altering();
	// lcl[0].altering4(); //pitchが変だから

    prepare_flag=false;



}


void ndt_callback(const nav_msgs::Odometry msg){
	//ndt_odom = msg;
    

	is_started = true;

	fflush(stdout);
    
    lcl[2].x = msg.pose.pose.position.x;
	lcl[2].y = msg.pose.pose.position.y;
	lcl[2].yaw = msg.pose.pose.orientation.z;
	
    //lcl[2].yaw_ = msg.pose.pose.orientation.z-yaw_before;
	//yaw_before = msg.pose.pose.orientation.z;
	
    ndt_flag = true;


	lcl[2].alter_dyaw(); //pitchが変だから


    //cout<<"補正"<<lcl[2].w<<endl;

    prepare_flag=false;

}


void ekf_callback(const nav_msgs::Odometry msg){
	is_started = true;

	fflush(stdout);
    
    lcl[3].x = msg.pose.pose.position.x;
	lcl[3].y = msg.pose.pose.position.y;
	lcl[3].yaw = msg.pose.pose.orientation.z;
	
	
    prepare_flag=false;

}


void ndt_i_callback(const nav_msgs::Odometry msg){
	
    is_started = true;

	fflush(stdout);
    
    lcl[4].x = msg.pose.pose.position.x;
	lcl[4].y = msg.pose.pose.position.y;
	lcl[4].yaw = msg.pose.pose.orientation.z;
	
	
    prepare_flag=false;

}




void score_callback(std_msgs::Int32 msg){

	score = msg.data;
    // cout <<"fitness score:"<<score<<endl;


}





int main (int argc, char** argv)
{
	ros::init(argc, argv, "velo_local");
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

    
	// ros::Subscriber odm_sub = n.subscribe("/odom", 100, odom_callback);
	// ros::Subscriber amu_sub = n.subscribe("/imu/data", 100, imu_callback);
	// ros::Subscriber ndt_sub = n.subscribe("/sq_ndt_data", 100, ndt_callback);
    // ros::Subscriber ekf_sub = n.subscribe("/ekf_NDT", 100, ekf_callback);


	ros::Subscriber odom_sub = n.subscribe("/tinypower/odom", 100, odom_callback);
	ros::Subscriber amu_sub = n.subscribe("/AMU_data", 100, amu_callback);
	ros::Subscriber ndt_sub = n.subscribe("/sq_ndt_data", 100, ndt_callback);
	ros::Subscriber ndt_i_sub = n.subscribe("/ndt_data_i", 100, ndt_i_callback);
    ros::Subscriber ekf_sub = n.subscribe("/ekf_NDT", 100, ekf_callback);

    
    ros::Subscriber score_sub = n.subscribe("/fitness_score", 100, score_callback);


	ros::Publisher lcl_pub = n.advertise<nav_msgs::Odometry>("/lcl_odo", 10);
	ros::Publisher lcl_1_pub = n.advertise<nav_msgs::Odometry>("/lcl_imu", 10);
	ros::Publisher lcl_2_pub = n.advertise<nav_msgs::Odometry>("/lcl_ndt", 10);
	ros::Publisher lcl_3_pub = n.advertise<nav_msgs::Odometry>("/lcl_ekf", 10);
	ros::Publisher lcl_4_pub = n.advertise<nav_msgs::Odometry>("/lcl_ndt_i", 10);


	ros::Publisher lcl_hantei_pub = n.advertise<std_msgs::Int32>("/hantei", 10);//0:直線 1:カーブ


	ros::Publisher lcl_vis_pub = n.advertise<geometry_msgs::Pose>("/lcl_vis", 10);

	tf::TransformBroadcaster br;
	tf::Transform transform; //位置と姿勢を持つ座標系を表すクラス
	
    lcl_.header.frame_id = "map";
	// lcl_.child_frame_id = "base_link";
	// lcl_.child_frame_id = "matching_base_link3";
	lcl_.child_frame_id = "matching_base_link";
	// lcl_.child_frame_id = "centerlaser_";
    
    lcl_1.header.frame_id = "map";
	// lcl_1.child_frame_id = "matching_base_link3";
	lcl_1.child_frame_id = "matching_base_link";
	// lcl_1.child_frame_id = "base_link";
	// lcl_1.child_frame_id = "centerlaser_";

    lcl_2.header.frame_id = "map";
	// lcl_2.child_frame_id = "matching_base_link3";
	lcl_2.child_frame_id = "matching_base_link";
	// lcl_2.child_frame_id = "base_link";
	// lcl_2.child_frame_id = "centerlaser_";


    lcl_3.header.frame_id = "map";
	// lcl_3.child_frame_id = "matching_base_link3";
	lcl_3.child_frame_id = "matching_base_link";
	// lcl_3.child_frame_id = "base_link";
	// lcl_3.child_frame_id = "centerlaser_";


    lcl_4.header.frame_id = "map";
	// lcl_3.child_frame_id = "matching_base_link3";
	lcl_4.child_frame_id = "matching_base_link";


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
			// br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "matching_base_link"));
			// br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));
			// br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "centerlaser_"));
		
        }

		if(is_started){



            lcl_.header.stamp = ros::Time::now(); //timestampのメッセージを送ろうとしている 例）1370000000
			lcl_.pose.pose.position.x = lcl[0].x;
			lcl_.pose.pose.position.y = lcl[0].y;
			lcl_.pose.pose.position.z = 0.0;
			lcl_.pose.pose.orientation.z = lcl[0].yaw;
					
			lcl_pub.publish(lcl_);


            lcl_1.header.stamp = ros::Time::now(); //timestampのメッセージを送ろうとしている 例）1370000000
			lcl_1.pose.pose.position.x = lcl[1].x;
			lcl_1.pose.pose.position.y = lcl[1].y;
			lcl_1.pose.pose.position.z = 0.0;
			lcl_1.pose.pose.orientation.z = lcl[1].yaw;
					
			lcl_1_pub.publish(lcl_1);

			
            lcl_2.header.stamp = ros::Time::now(); //timestampのメッセージを送ろうとしている 例）1370000000
			lcl_2.pose.pose.position.x = lcl[2].x;
			lcl_2.pose.pose.position.y = lcl[2].y;
			lcl_2.pose.pose.position.z = 0.0;
			lcl_2.pose.pose.orientation.z = lcl[2].yaw;
					
			lcl_2_pub.publish(lcl_2);
            
            
            
            
            lcl_3.header.stamp = ros::Time::now(); //timestampのメッセージを送ろうとしている 例）1370000000
			lcl_3.pose.pose.position.x = lcl[3].x;
			lcl_3.pose.pose.position.y = lcl[3].y;
			//lcl_3.pose.pose.position.z = lcl[1].z;
			lcl_3.pose.pose.position.z = 0.0;	
            lcl_3.pose.pose.orientation.z = lcl[3].yaw;
					
			lcl_3_pub.publish(lcl_3);
	        
            lcl_vis = lcl_3.pose.pose;
    

            // cout<<"z:"<<lcl_3.pose.pose.position.z<<endl;


			lcl_vis_pub.publish(lcl_vis);


            lcl_4.header.stamp = ros::Time::now(); //timestampのメッセージを送ろうとしている 例）1370000000
			lcl_4.pose.pose.position.x = lcl[4].x;
			lcl_4.pose.pose.position.y = lcl[4].y;
			lcl_4.pose.pose.position.z = 0.0;
			lcl_4.pose.pose.orientation.z = lcl[4].yaw;
					
			lcl_4_pub.publish(lcl_4);




            number.data = num;
            // number.data = score;

            lcl_hantei_pub.publish(number);//0:直線 1:カーブ

            // transform.setOrigin( tf::Vector3(lcl[2].x, lcl[2].y, 0.0) );
			// tf::Quaternion q;
			// q.setRPY(0, 0, lcl[2].yaw);
            
            transform.setOrigin( tf::Vector3(lcl[3].x, lcl[3].y, 0.0) );
			tf::Quaternion q;
			q.setRPY(0, 0, lcl[3].yaw);
		

            
//////////////  ndt による位置推定
		/*
            transform.setOrigin( tf::Vector3(lcl[2].x, lcl[2].y, 0.0) );
			tf::Quaternion q;
			q.setRPY(0, 0, lcl[2].yaw);
		*/	
            
            transform.setRotation(q);
			// br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "centerlaser_"));
			// br.sendTransform(tf::StampedTransform(transform, lcl_.header.stamp , "map", "matching_base_link"));
			br.sendTransform(tf::StampedTransform(transform, lcl_.header.stamp , "map", "matching_base_link"));
			// br.sendTransform(tf::StampedTransform(transform, lcl_.header.stamp , "map", "base_link"));
		
		}
		
        cnt++;
        
        roop.sleep();
		ros::spinOnce();
	}

	fclose(fp_2);
    
    return 0;

}


