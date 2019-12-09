#include <header.h>
#include <ros/package.h>
#include <imageTransporter.hpp>
#include <chrono>
#include <kobuki_msgs/WheelDropEvent.h>
#include <kobuki_msgs/BumperEvent.h>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

#include <stdio.h>
#include <cmath>

#include <tf/transform_datatypes.h>

using namespace std;

double linear = 0;
double angular = 0;
double posX;
double posY;
double yaw;
double desireyaw = 0;
double pi = 3.1416;
bool bumperLeft = 0, bumperCenter = 0, bumperRight = 0;
int bumperCount = 0;

geometry_msgs::Twist follow_cmd;
int world_state = 0;
int prevState = -1;
double prevAngular = 0;
std::chrono::time_point<std::chrono::system_clock> liftedstart;
uint64_t liftedtime = 0;
bool is_playing;

bool checkIfGetDesireYaw(){
	float threshold = 0.05;
	if ((abs(desireyaw - yaw) < threshold) || (abs(abs(desireyaw-yaw)-2*pi) < threshold)){
		return true;
	}else{
		return false;
	}
}

void setSpeed(float lin, float ang){
	linear = lin;
	angular = ang;
}

double turn(double currentyaw, double angle){
	double outputyaw;
	outputyaw = currentyaw + angle;
	//cout << "check 1!!!!" << "\n\n\n";
	if (outputyaw > pi){
		outputyaw -= 2*pi;
	//cout << "check 2!!!!" << "\n\n\n";
	}
	else if (outputyaw < -pi){
		outputyaw += 2*pi;
	}
	//cout << "check 3!!!!" << "\n\n\n";
	//cout << "   currentyaw" << currentyaw;
	//cout << "   angle" << angle;
	//cout << "   outputyaw" << outputyaw << "\n\n\n	";
	return outputyaw;
}

void followerCB(const geometry_msgs::Twist msg){
    follow_cmd = msg;
}

void wheeldropCB(const kobuki_msgs::WheelDropEvent msg){
    if(msg.wheel == 0 && msg.wheel == 1){
        world_state = 3;
        is_playing = false;
        liftedtime = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-liftedstart).count();
        if(liftedtime >= 5){
            world_state = 4;
	    is_playing = false;
        }
    }
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
	posX = msg->pose.pose.position.x;
	posY = msg->pose.pose.position.y;
	yaw = tf::getYaw(msg->pose.pose.orientation);

	ROS_INFO("Position: (%f, %f) Orientation: %f rad or %f degrees.", posX, posY, yaw, yaw*180/pi);
}

void bumperCallback(const kobuki_msgs::BumperEvent msg){
    //Fill with code
	std::cout << "check11111111111111111111111111111111111111111111111111111111111" <<std::endl;
	if(msg.bumper == 0)
		bumperLeft = !bumperLeft;
		if (bumperLeft == 1){
			setSpeed(0,0);
			prevState = world_state;
			desireyaw = turn(yaw, -90.0/180.0*pi);
			world_state = 1;
		}
	else if(msg.bumper == 1)
		bumperCenter = !bumperCenter;
		if (bumperCenter == 1){
			setSpeed(0,0);
			prevState = world_state;
			desireyaw = turn(yaw, -90.0/180.0*pi);
			world_state = 1;
		}
	else if(msg.bumper == 2)
		bumperRight = !bumperRight;
		if (bumperRight == 1){
			setSpeed(0,0);
			prevState = world_state;
			desireyaw = turn(yaw, -90.0/180.0*pi);
			world_state = 1;
		}

	/*
    if(msg.bumper == 1){
        bumperCenter = !bumperCenter;
        if (bumperCenter == 1){
            linear = 0;
            angular = 0;
            if(prevState == 0){
                world_state = 1;
                is_playing = false;
            }
            if(prevState == 1){
                bumperCount += 1;
                if(bumperCount >= 10){
                    world_state = 2;
                    is_playing = false;
                }
            }
        }
    }
	*/
}

//-------------------------------------------------------------

int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh;
	sound_play::SoundClient sc;
	string path_to_sounds = ros::package::getPath("mie443_contest3") + "/sounds/";
	teleController eStop;

	//publishers
	ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop",1);

	//subscribers
	ros::Subscriber follower = nh.subscribe("follower_velocity_smoother/smooth_cmd_vel", 10, &followerCB);
	ros::Subscriber bumper_sub = nh.subscribe("/mobile_base/events/bumper", 10, &bumperCallback);
	ros::Subscriber wheel = nh.subscribe("mobile_base/events/wheel_drop", 10, &wheeldropCB);
	ros::Subscriber odom = nh.subscribe("odom", 1, odomCallback);
	imageTransporter rgbTransport("camera/image/", sensor_msgs::image_encodings::BGR8); //--for Webcam
	//imageTransporter rgbTransport("camera/rgb/image_raw", sensor_msgs::image_encodings::BGR8); //--for turtlebot Camera
	imageTransporter depthTransport("camera/depth_registered/image_raw", sensor_msgs::image_encodings::TYPE_32FC1);

	std::chrono::time_point<std::chrono::system_clock> start;
    	uint64_t song_time = 0; 

	geometry_msgs::Twist vel;
	
	sc.repeat("My Name is Mars");
	//sc.playWave(path_to_sounds + "dejavu.wav");
	ros::Duration(0.5).sleep();
	bool is_playing = false;
        bool liftedstart_flag = false;

	while(ros::ok()){
		ros::spinOnce();
		std::cout<<"World_state: "<<world_state <<std::endl;
		//.....**E-STOP DO NOT TOUCH**.......
		//eStop.block();
		//...................................
		if(world_state == 0){
			//std::cout<<"CURRENT : "<<world_state <<std::endl;
			//fill with your code
			//vel_pub.publish(vel);
			if(song_time >= 10 || !is_playing){
				sc.stopAll();
				song_time = 0;
				is_playing = true;
				sc.startWave(path_to_sounds
			                     +"dejavu.wav");
				start = std::chrono::system_clock::now(); /* start timer */

			}
			vel_pub.publish(follow_cmd);
			//sc.stopWave(path_to_sounds+"dejavu.wav");
			prevState = 0;
			world_state = 0;
			
		}else if(world_state == 1){
			angular = 1.0;
			linear = 0;
			if (checkIfGetDesireYaw()){
				turn(yaw, 180.0/180.0*pi);
				angular = -1.0;
				linear = 0;
				world_state = 11;
			}
		}
		else if(world_state == 11){
			angular = -1.0;
			linear = 0;
			if (checkIfGetDesireYaw()){
				angular = 0;
				linear = 0;
				world_state = prevState;
			}
		}
/*
		else if(world_state == 2){
			if(song_time >= 10 || !is_playing){
				sc.stopAll();
				song_time = 0;
				is_playing = true;
				sc.startWave(path_to_sounds
			                     +"dejavu.wav");
				start = std::chrono::system_clock::now();

			}
		}
		else if(world_state == 3){
			if(!liftedstart_flag){
				liftedstart = std::chrono::system_clock::now();
                                liftedstart_flag = true;
			}
			if(song_time >= 10 || !is_playing){
				sc.stopAll();
				song_time = 0;
				is_playing = true;
				sc.startWave(path_to_sounds
			                     +"dejavu.wav");
				start = std::chrono::system_clock::now(); 

			}
			prevState = 3;
			world_state = 0;
		}
		else if(world_state = 4){
			if(song_time >= 10 || !is_playing){
				sc.stopAll();
				song_time = 0;
				is_playing = true;
				sc.startWave(path_to_sounds
			                     +"dejavu.wav");
				start = std::chrono::system_clock::now();

			}
		}
		*/
		if(is_playing){
		song_time = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();}
                //ros::Duration(0.5).sleep();
		vel.angular.z = angular;
		vel.linear.x = linear;
  		vel_pub.publish(vel);
	}

	return 0;
}
