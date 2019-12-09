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
#include <visualization_msgs/Marker.h>

#include <stdio.h>
#include <cmath>

#include <tf/transform_datatypes.h>

using namespace std;

geometry_msgs::Twist follow_cmd;

int world_state = 0;
int prevState = -1;
bool is_playing = false;
bool is_bumpering = false;
bool is_following = false;
bool is_lifting = false;
int state1_helper = 0; //how many cycle it has rotated
int num_bumper_pushed = 0; //how many times the bumper has been pushed;.
int num_lifted = 0; //how many times the robot has been lifted;
int lifted_helper = 0; //0 means lifted, 1 means lifted

double angular = 0.2;
double linear = 0.0;

double posX;
double posY;
double yaw;
double markerX;
double markerY;
double markerZ;
double desireyaw = 0;
double oldPx = 0;
double oldPy = 0;
uint64_t song_time = 0;
std::chrono::time_point<std::chrono::system_clock>  song_start;
uint64_t bumper_time = 0;
std::chrono::time_point<std::chrono::system_clock>  bumper_start;
uint64_t follower_time = 0;
std::chrono::time_point<std::chrono::system_clock>  follower_start;
uint64_t lifted_time = 0;
std::chrono::time_point<std::chrono::system_clock>  lifted_start;
double pi = 3.141592653;
bool bumperLeft = 0, bumperCenter = 0, bumperRight = 0;

// define sound track and sound track length
string path_to_sounds = ros::package::getPath("mie443_contest3") + "/sounds/";
string sound_track_emotion_1 = "scream.wav";
double length_emotion_1 = 2.0;
string sound_track_emotion_2 = "suprise.wav";
double length_emotion_2 = 3.0;
string sound_track_emotion_3 = "disgust.wav";
double length_emotion_3 = 2.0;
string sound_track_emotion_4 = "excitex.wav";
double length_emotion_4 = 3.0;

// define constant
float circle_radius = 0.5; // move backward distance
float bumper_tolerance = 5; // multiple bumper pushing time threshold
float lifted_tolerance = 5; // multiple lifting time threshold

void setSpeed(float lin, float ang){
	linear = lin;
	angular = ang;
}

bool checkIfGetDesireYaw(){
	// check if it has reached the desire yaw
	float threshold = 0.05;
	if ((abs(desireyaw - yaw) < threshold) || (abs(abs(desireyaw-yaw)-2*pi) < threshold)){
		return true;
	}else{
		return false;
	}
}

double turn(double currentyaw, double angle){
	// return yaw within the valid range
	double outputyaw;
	outputyaw = currentyaw + angle;
	if (outputyaw > pi){
		outputyaw -= 2*pi;
	}
	else if (outputyaw < -pi){
		outputyaw += 2*pi;
	}
	return outputyaw;
}

bool checkIfGetDesirePos(float temp = circle_radius){
	// check if robot has travel away from original position
	if (sqrt(pow(posX-oldPx, 2) + pow(posY-oldPy, 2)) > temp){
		return true;
	}else{
		return false;
	}
}



void bumperCBHelper(){
	setSpeed(0,0);
	oldPx = posX;
	oldPy = posY;
	if (num_bumper_pushed == 0){
		// the first push
		setSpeed(0,0);
		prevState = world_state;
		is_bumpering = true;
		bumper_start = std::chrono::system_clock::now();
		num_bumper_pushed++;
		world_state = 201;
		std::cout << "check11111111111111111" << endl;
	}else if (num_bumper_pushed >= 2){
		// range push
		setSpeed(0,0);
		num_bumper_pushed++;
		turn(yaw,90.0/180.0*pi);
    	// bumper_time = 0;
    	// num_bumper_pushed = 0;
		world_state = 301;
		std::cout << "check22222222222222222222" << endl;
	}else{
		// intermidiate push
		num_bumper_pushed++;
		world_state = 201;
	}
}


void playSong(string sound_track, double length, sound_play::SoundClient& sc){
	// play song non-stop
	if (!is_playing){
		sc.stopAll();
		song_time = 0;
		is_playing = true;
		sc.startWave(path_to_sounds+sound_track);
		song_start = std::chrono::system_clock::now();
	}
}

void stopSong(sound_play::SoundClient& sc){
	// stop the song playing
	sc.stopAll();
	song_time = 0;
	is_playing = false;
}

// define CallBack function below
void followerCB(const geometry_msgs::Twist msg){
    follow_cmd = msg;
}

void bumperCB(const kobuki_msgs::BumperEvent msg){
    //Fill with code
    if(bumper_time >= bumper_tolerance){
    	// reset bumper_time
    	bumper_time = 0;
    	num_bumper_pushed = 0;
    }

    if(msg.bumper == 1){
		bumperCenter = !bumperCenter;
		if (bumperCenter == 1){
			bumperCBHelper();
		}
	}
	// else if(msg.bumper == 1)
	// 	bumperCenter = !bumperCenter;
	// 	if (bumperCenter == 1){
	// 		bumperCBHelper();
	// 	}
	// else if(msg.bumper == 2)
	// 	bumperRight = !bumperRight;
	// 	if (bumperRight == 1){
	// 		bumperCBHelper();
	// 	}
}

void odomCB(const nav_msgs::Odometry::ConstPtr& msg){
	posX = msg->pose.pose.position.x;
	posY = msg->pose.pose.position.y;
	yaw = tf::getYaw(msg->pose.pose.orientation);

	//ROS_INFO("Position: (%f, %f) Orientation: %f rad or %f degrees.", posX, posY, yaw, yaw*180/pi);
}

void wheeldropCB(const kobuki_msgs::WheelDropEvent msg){
    if(lifted_time >= lifted_tolerance){
    	// reset bumper_time
    	lifted_time = 0;
    	num_lifted = 0;
    }

    if(msg.state == 0){
    	//check the robot has been lifted.
    	if (num_lifted == 0){
		is_lifting = true;
    		lifted_start = std::chrono::system_clock::now();
    		num_lifted++;
    	} else if (num_lifted >= 2){
    		// range lift
    		num_lifted++;
    		prevState = world_state;
    		world_state = 401;
    	}else{
    		num_lifted++;
		}
	}

}

void markerCB(const visualization_msgs::Marker& msg){
	markerX = msg.pose.position.x;
	markerY= msg.pose.position.y;
	markerZ = msg.pose.position.z;

	if (markerZ >= 1000){
		// check if it lost track
		if (follower_time>=1){
			//check if it lost track for 1 second.
			prevState = world_state;
			turn(yaw, 180/180*pi);
			world_state = 101;
		}else if (!is_following){
			//start timer
			is_following = true;
			follower_start = std::chrono::system_clock::now();
	}else{
		is_following = false;
	}
	}

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
	ros::Subscriber bumper = nh.subscribe("mobile_base/events/bumper", 10, &bumperCB);
	ros::Subscriber odom = nh.subscribe("odom", 1, odomCB);
	ros::Subscriber wheel = nh.subscribe("mobile_base/events/wheel_drop", 10, &wheeldropCB);
	ros::Subscriber marker = nh.subscribe("/turtlebot_follower/marker", 10, &markerCB);

	imageTransporter rgbTransport("camera/image/", sensor_msgs::image_encodings::BGR8); //--for Webcam
	//imageTransporter rgbTransport("camera/rgb/image_raw", sensor_msgs::image_encodings::BGR8); //--for turtlebot Camera
	imageTransporter depthTransport("camera/depth_registered/image_raw", sensor_msgs::image_encodings::TYPE_32FC1);

	geometry_msgs::Twist vel;
	int temp = 0;

	// sc.playWave(path_to_sounds + "sound.wav");
	ros::Duration(0.5).sleep();

	while(ros::ok()){
		ros::spinOnce();
	oldPx = posX;
	oldPy = posY;
	setSpeed(-0.1,0);
	playSong(sound_track_emotion_2, length_emotion_2,sc);
	if (checkIfGetDesirePos(0.15)){
		setSpeed(0,0);
		stopSong(sc);
		world_state = prevState;
	}
			vel.angular.z = angular;
			vel.linear.x = linear;
	  		vel_pub.publish(vel);}
	return 0;

	while(ros::ok()){
		ros::spinOnce();
		if (temp >= 50){
		std::cout<<"World_state: "<<world_state <<std::endl;
		temp = 0;
		}
		temp++;

		//.....**E-STOP DO NOT TOUCH**.......
		//eStop.block();
		//...................................


		prevState = 0;
		switch(world_state){
			case 0: // human following
			vel_pub.publish(follow_cmd);
			break;

			case 101: // emotion1_1: rotate half cycle
				//TODO: when to enter this state (remember to store prevState + set desire yaw)
				setSpeed(0, 3);
				playSong(sound_track_emotion_1, length_emotion_1,sc);
				if (checkIfGetDesireYaw()){
					// check if it has rotate 180
					turn(yaw, 180/180*pi);
					world_state = 102;
				}
				break;

			case 102: // emotion1_2: rotate another half cycle
				setSpeed(0, 3);
				playSong(sound_track_emotion_1, length_emotion_1,sc);
				if (checkIfGetDesireYaw()){
					// check if it has rotate 180
					if (state1_helper >= 2){
						//check if it has rotate 2 cycles
						setSpeed(0, 0);
						stopSong(sc);
						world_state = prevState;
					}else{
						state1_helper++;
						turn(yaw, 180/180*pi);
						world_state = 101;
					}
				}
				break;

			case 201: // emotion2_1
				setSpeed(-0.1,0);
				playSong(sound_track_emotion_2, length_emotion_2,sc);
				if (checkIfGetDesirePos(0.15)){
					setSpeed(0,0);
					stopSong(sc);
					world_state = prevState;
				}else{
					setSpeed(-0.5,0);
				}
				break;

			case 301: // emotion3_1
				setSpeed(0,-3);
				playSong(sound_track_emotion_3, length_emotion_3,sc);
				if (checkIfGetDesireYaw()){
					// check if it has rotate 90
					setSpeed(0,0);
					turn(yaw, 180/180*pi);
					world_state = 302;
				}
				break;

			case 302: // emotion3_2
				setSpeed(0,3);
				playSong(sound_track_emotion_3, length_emotion_3,sc);
				if (checkIfGetDesireYaw()){
					// check if it has rotate 180
					setSpeed(0,0);
					stopSong(sc);
					world_state = prevState;
				}
				break;

			case 401: // emotion4_1: move backward
				setSpeed(-1,0);
				playSong(sound_track_emotion_4, length_emotion_4,sc);
				if (checkIfGetDesirePos(0.2)){
					oldPx = posX;
					oldPy = posY;
					world_state = 402;
				}
				break;

			case 402: // emotion4_2:  move forward
				setSpeed(1,0);
				playSong(sound_track_emotion_4, length_emotion_4,sc);
				if (checkIfGetDesirePos(0.2)){
					oldPx = posX;
					oldPy = posY;
					world_state = 403;
				}
				break;

			case 403: // emotion4_3: move backward
				setSpeed(-1,0);
				playSong(sound_track_emotion_4, length_emotion_4,sc);
				if (checkIfGetDesirePos(0.2)){
					oldPx = posX;
					oldPy = posY;
					world_state = 404;
				}
				break;

			case 404: // emotion4_3: move forward
				setSpeed(1,0);
				playSong(sound_track_emotion_4, length_emotion_4,sc);
				if (checkIfGetDesirePos(0.2)){
					stopSong(sc);
					world_state = prevState;
				}
				break;

			default:
				break;
		}

		// timers
		if(is_playing){
			song_time = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-song_start).count();
		}
		if(is_bumpering){
			bumper_time = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-bumper_start).count();
		}
		if(is_lifting){
			lifted_time = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-lifted_start).count();
		}
		if(is_following){
			follower_time = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-follower_start).count();
		}

		// send speed command to robot
		if (world_state != 0){
			vel.angular.z = angular;
			vel.linear.x = linear;
	  		vel_pub.publish(vel);
  		}
        ros::Duration(0.01).sleep();

	}

	return 0;
}
