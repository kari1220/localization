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

#include <complement/localization.h>

using namespace std;	

bool error_flag=false;
bool prepare_flag=true;
bool is_started=false;
bool ndt_flag=false;


const size_t N = 3;
Localization lcl[N];

geometry_msgs::Pose2D init_pose; // [deg] This expresses a position and orientation on a 2D manifold.

nav_msgs::Odometry lcl_ini;
nav_msgs::Odometry lcl_;
nav_msgs::Odometry lcl_1;
nav_msgs::Odometry lcl_2;
nav_msgs::Odometry lcl_vis;




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


void amu_callback(ceres_msgs::AMU_data::ConstPtr msg){//pitch だから前後の縦揺れ
	is_started = true;
	fflush(stdout);
	
    lcl[0].pitch = M_PI * msg->pitch / 180.0;
	lcl[1].pitch = M_PI * msg->pitch / 180.0;
	lcl[2].pitch = M_PI * msg->pitch / 180.0;
	lcl[1].w = - M_PI * ( msg->dyaw - 0.275030001 ) / 180.0;
	lcl[2].w = - M_PI * ( msg->dyaw - 0.275030001 ) / 180.0;
	//lcl[1].w += 0.0041196016 - 0.00025 + 0.000171428; //correction
	lcl[1].altering();
	lcl[2].altering();


    prepare_flag=false;
}

void odom_callback(nav_msgs::Odometry::ConstPtr msg){
	is_started = true;
	fflush(stdout);
	
    lcl[0].v = msg->twist.twist.linear.x;
	lcl[1].v = msg->twist.twist.linear.x;
	lcl[2].v = msg->twist.twist.linear.x;

	lcl[0].w = msg->twist.twist.angular.z;
	lcl[0].altering();



    prepare_flag=false;
}


void ndt_callback(const nav_msgs::Odometry msg){
	//ndt_odom = msg;
    


    cout<<"補正"<<endl;
    
    lcl[2].x = msg.pose.pose.position.x;
	lcl[2].y = msg.pose.pose.position.y;
	lcl[2].yaw = msg.pose.pose.orientation.z;
	ndt_flag = true;
}

int main (int argc, char** argv)
{
	ros::init(argc, argv, "lcl_manage");
  	ros::NodeHandle n;
	ros::Rate roop(100);
	
    setInitPose(n, init_pose);


//Localization lcl[N] 格納;

	for(size_t i=0;i<N;i++){
		lcl[i].x =   init_pose.x;
		lcl[i].y =   init_pose.y;
		lcl[i].yaw = init_pose.theta*M_PI/180.0;
	}
	
	for(size_t i=0;i<N;i++)
		lcl[i].start();
	
	lcl_ini.pose.pose.position.x = init_pose.x;
	lcl_ini.pose.pose.position.y = init_pose.y;
	lcl_ini.pose.pose.orientation.z = init_pose.theta*M_PI/180.0;//初期方位[rad]が入っている

    
	ros::Subscriber odm_sub = n.subscribe("/tinypower/odom", 100, odom_callback);
	ros::Subscriber amu_sub = n.subscribe("/AMU_data", 100, amu_callback);
	ros::Subscriber ndt_sub = n.subscribe("/ndt_data", 100, ndt_callback);

	
	ros::Publisher lcl_pub = n.advertise<nav_msgs::Odometry>("/lcl_odo", 10);
	ros::Publisher lcl_1_pub = n.advertise<nav_msgs::Odometry>("/lcl_amu", 10);
	ros::Publisher lcl_2_pub = n.advertise<nav_msgs::Odometry>("/lcl_ndt", 10);


	ros::Publisher lcl_vis_pub = n.advertise<nav_msgs::Odometry>("/lcl_vis", 10);

	tf::TransformBroadcaster br;
	tf::Transform transform; //位置と姿勢を持つ座標系を表すクラス
	
    lcl_.header.frame_id = "map";
	lcl_.child_frame_id = "matching_base_link";
    
    lcl_1.header.frame_id = "map";
	lcl_1.child_frame_id = "matching_base_link";

    lcl_2.header.frame_id = "map";
	lcl_2.child_frame_id = "matching_base_link";
    

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
		
        }

		if(is_started){

		if(cnt % 50 == 0){ //cnt++よりカウント

            
			//fprintf(fp_2, "%lf,%lf,%lf,%lf,%lf,%lf\n", lcl[0].x, lcl[0].y, lcl[0].yaw, lcl[1].x, lcl[1].y, lcl[1].yaw);
			fprintf(fp_2, "%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",  lcl[1].x, lcl[2].x, lcl[1].y, lcl[2].y, lcl[1].yaw, lcl[2].yaw, ndt_flag);
			//fprintf(fp_2, "%lf,%lf,%lf,%lf,%lf,%lf\n", lcl[0].x, lcl[0].y, lcl[1].x, lcl[1].y, lcl[2].x, lcl[2].y);
			cnt = 0;
		
            ndt_flag=false;
        }
            
     //       cout<<"start"<<endl; 




/////////////////確認
/*
            cout<<"lcl[0].dt"<<lcl[0].dt<<endl;
            cout<<"lcl[0].pitch"<<lcl[0].pitch<<endl;
            cout<<"lcl[0].yaw"<<lcl[0].yaw<<endl;
            cout<<"lcl[0].w"<<lcl[0].w<<endl;
            cout<<"lcl[0].v"<<lcl[0].v<<endl;
*/



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
	        
            lcl_vis = lcl_2;
            
            
            //transform.setOrigin( tf::Vector3(lcl[0].x, lcl[0].y, 0.0) );
			//tf::Quaternion q;
			//q.setRPY(0, 0, lcl[0].yaw);
			
			transform.setOrigin( tf::Vector3(lcl[2].x, lcl[2].y, 0.0) );
			tf::Quaternion q;
			q.setRPY(0, 0, lcl[2].yaw);
			
            
            transform.setRotation(q);
			br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "matching_base_link"));
		
		}
		
        cnt++;
        
        roop.sleep();
		ros::spinOnce();
	}

	fclose(fp_2);
    
    return 0;

}

