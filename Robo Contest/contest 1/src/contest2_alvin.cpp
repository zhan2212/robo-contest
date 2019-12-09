#include <iostream>
#include <fstream>
#include <math.h>
#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>
#include <algorithm>

using namespace std;

int main(int argc, char** argv) {
	// Setup ROS.
	ros::init(argc, argv, "contest2");
	ros::NodeHandle n;
	// Robot pose object + subscriber.
	RobotPose robotPose(0,0,0);
	ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, &RobotPose::poseCallback, &robotPose);
	// Initialize box coordinates and templates
	Boxes boxes;
	if(!boxes.load_coords() || !boxes.load_templates()) {
		std::cout << "ERROR: could not load coords or templates" << std::endl;
		return -1;
	}
	for(int i = 0; i < boxes.coords.size(); ++i) {
		std::cout << "Box coordinates: " << std::endl;
		std::cout << i << " x: " << boxes.coords[i][0] << " y: " << boxes.coords[i][1] << " z: "
				<< boxes.coords[i][2] << std::endl;
	}
	// Initialize image object and subscriber.
	ImagePipeline imagePipeline(n);
	std::cout<<"Image Pipeline Initialized"<<std::endl;
	sleep(1);
	ros::spinOnce();
	auto robot_goals_boxes = boxes.coords;
	std::vector<vector<vector<float> > > robot_goals;
	std::vector<float> view_distance{0.6, 0.6, 0.6};
	std::vector<float> angles = {0.0, M_PI/3, -M_PI/3+M_PI*2.0};
	std::cout<<"Loading Goals ... "<<std::endl;
	// Process Goal
	for (auto& it : robot_goals_boxes) {
		std::vector<std::vector<float> > poses;
		for (size_t i = 0; i < view_distance.size(); ++i) {
			auto theta_coord = it.at(2) + angles[i];
			poses.push_back({static_cast<float>(it[0] + view_distance[i] * cos(theta_coord)),
					 static_cast<float>(it[1] + view_distance[i] * sin(theta_coord)),
					 static_cast<float>(fmodf(theta_coord + M_PI + M_PI, 2.0*M_PI)-M_PI)});
			//it[0] += view_distance * cos(theta_coord);
			//it[1] += view_distance * sin(theta_coord);
			//it[2] = theta_coord + M_PI;
			//it[2] = fmodf(it[2]+M_PI,2.0*M_PI)-M_PI;
		}
		robot_goals.push_back(poses);
	}
	std::vector<float> orgin_pose{robotPose.x, robotPose.y, robotPose.phi};
	std::vector<vector<float> > orgin_pose_vec;
	orgin_pose_vec.push_back(orgin_pose);
	robot_goals.push_back(orgin_pose_vec);
	//robot_goals.push_back({robotPose.x, robotPose.y, robotPose.phi});
	std::cout<<"Goals Loaded"<<std::endl;
	size_t current_goal = 0;
	std::vector<int32_t> detection_result;
	std::cout<<"X: "<<robot_goals.at(current_goal).at(0).at(0)<<
			"Y: "<<robot_goals.at(current_goal).at(0).at(1)<<
			"Theta: "<<robot_goals.at(current_goal).at(0).at(2)<<std::endl;
	size_t num_failed(0);
	// Execute strategy.
	while(ros::ok()) {
		ros::spinOnce();
		/***YOUR CODE HERE***/
		// Use: boxes.coords
		// Use: robotPose.x, robotPose.y, robotPose.phi
		size_t ind_goal = num_failed % robot_goals.at(current_goal).size();
		if (Navigation::moveToGoal(robot_goals.at(current_goal).at(ind_goal).at(0),
				robot_goals.at(current_goal).at(ind_goal).at(1),
				robot_goals.at(current_goal).at(ind_goal).at(2))) {
			if(current_goal < robot_goals.size() - 1) {
				++current_goal;
				std::cout<<"New Goal "<<current_goal<<std::endl;
				std::cout<<"X: "<<robot_goals.at(current_goal).at(ind_goal).at(0)<<
						"Y: "<<robot_goals.at(current_goal).at(ind_goal).at(1)<<
						"Theta: "<<robot_goals.at(current_goal).at(ind_goal).at(2)<<std::endl;
				std::vector<size_t> result{0, 0, 0, 0};
				std::cout<<"Goal Reached"<<std::endl;
				for (size_t i = 0; i < 10; ++i) {
					std::cout<<"Process Image"<<std::endl;
					auto ind = imagePipeline.getTemplateID(boxes);
					if (ind >= 0) {
						result[ind]++;
					}
					ros::spinOnce();
				}
				std::cout<<"Determine Image"<<std::endl;
				size_t max_ind = 0;
				for (size_t i = 1; i < result.size(); ++i) {
					if (result[i] > result[max_ind]) {
						max_ind = i;
					}
				}

				detection_result.push_back(max_ind);
			} else {
				break;
			}
		}

		//std::cout<<"Image Detected: "<<
		//		imagePipeline.getTemplateID(boxes)<<std::endl;
		ros::Duration(0.01).sleep();
	}
	std::ofstream output_file;
	output_file.open("Detection_Result.txt");
	for(size_t i = 0; i < detection_result.size(); ++i) {
		output_file <<"Tag: "<<detection_result[i]<<", Location: "<<i<<"\n";
	}
	output_file.close();
	return 0;
}
