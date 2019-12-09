#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>
#include <iostream>
#include <fstream>

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
    //robot initial pose initialization
    float initposex = 0;
    float initposey = 0;
    float initposephi = 0;

    std::vector<int> results;


    float boxx;
    float boxy;
    float boxphi;
    int i = 0;

    // Execute strategy.
    while(ros::ok()) {
        ros::spinOnce();
        if (!imagePipeline.isValid){
           continue;
        }
        /***YOUR CODE HERE***/
        // Use: boxes.coords
        // Use: robotPose.x, robotPose.y, robotPose.phi
        //1st: localization, save the initial pose
        if (i == 0){
            initposex = robotPose.x;
            initposey = robotPose.y;
            initposephi = robotPose.phi;
        }
	//loop through obstacles
	//2nd: move to goal
	std::cout<< "initposex: "<< initposex<< " || " << "initposey: "<< initposey<< " || " << "initposephi: "<< initposephi<< "\n"<<std::endl;
	
	
        boxx = boxes.coords[i][0]+0.5*cos(boxes.coords[i][2]);
        boxy = boxes.coords[i][1]+0.5*sin(boxes.coords[i][2]);
        if(boxes.coords[i][2]>0){
            boxphi = boxes.coords[i][2]-M_PI;
        }
        else{
            boxphi = boxes.coords[i][2]+M_PI;
        }
	std::cout<< "boxx: "<< boxx<< " || " << "boxy: "<< boxy<< " || " << "boxphi: "<< boxphi<< "\n"<<std::endl;
	
        if (Navigation::moveToGoal(boxx, boxy, boxphi) and i <= 4){
            ++i;
            std::cout<<i<<"\n"<<std::endl;
            //3rd: CV 
	    std::vector<int> id_score{0, 0, 0, 0};
            for (int j = 0; j < 10; j++){
                int id = imagePipeline.getTemplateID(boxes);
                std::cout << "id: " << id << std::endl; 
                if (id >= 0){
                    id_score[id]++;
                }
                ros::spinOnce();
            }
            int best_id = -1;
            int max_count = 0;
            for (int j = 0; j < 4; j++){
                if (id_score[j] > max_count){
                    best_id = j;
                    max_count = id_score[j];
                }
            }
            std::cout << "best id: " << best_id << std::endl; 
            results.push_back(best_id);
            Navigation::moveToGoal(initposex, initposey, initposephi);
        }
        //4th: back to the initial pose
        //if (i == 5){
          //  Navigation::moveToGoal(initposex,initposey,initposephi);
        //}
        ros::Duration(0.01).sleep();
	if (i >= 5){
            break;
        }
    }
    std::ofstream f;
    f.open("Results.txt");
    for (int i = 0; i < results.size(); i++){
        f << "Tag: " << results[i] << "    Location: " << i << std::endl;
    }
    f.close();
    return 0;
}
