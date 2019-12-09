#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>

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
    // Initialize image objectand subscriber.
    ImagePipeline imagePipeline(n);
    //robot initial pose initialization
    float initposex = 0;
    float initposey = 0;
    float initposephi = 0;

    float boxx;
    float boxy;
    float boxphi;

    std::vector<int> result;

    // Execute strategy.
    while(ros::ok()) {
        ros::spinOnce();
        /***YOUR CODE HERE***/
        // Use: boxes.coords
        // Use: robotPose.x, robotPose.y, robotPose.phi
        //1st: localization, save the initial pose
        initposex = robotPose.x;
        initposey = robotPose.y;
        initposephi = robotPose.phi;
	//loop through obstacles
        int i = 0;
	//2nd: move to goal
        boxx = boxes.coords[i][0]+0.5*cos(boxes.coords[i][2]);
        boxy = boxes.coords[i][0]+0.5*sin(boxes.coords[i][2]);
        if(boxes.coords[i][2]>0){
            boxphi = boxes.coords[i][2]-M_PI;
        }
        else{
            boxphi = boxes.coords[i][2]+M_PI;
        }
        if(Navigation::moveToGoal(boxx, boxy, boxphi)){
            i++;
            //3rd: CV 
       	    std::vector<int> id_score{0, 0, 0, 0};
            for (int j = 0; j < 10; j++){
	        int id = imagePipeline.getTemplateID(boxes);
                if (id >= 0){
                    id_score[id]++;
                }
                ros::spinOnce();
            }
            int max_id = *std::max_element(id_score.begin(), id_score.end());
            std::cout << "id: " << max_id << std::endl; 
            result.push_back(max_id);
        }
        //4th: back to the initial pose
        Navigation::moveToGoal(initposex,initposey,initposephi);

        ros::Duration(0.01).sleep();
	break;
	     //imagePipeline.getTemplateID(boxes);
    }
 
    return 0;
}
