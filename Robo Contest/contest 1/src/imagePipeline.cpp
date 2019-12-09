#include <imagePipeline.h>
#include <opencv2/xfeatures2d.hpp>

#define IMAGE_TYPE sensor_msgs::image_encodings::BGR8
#define IMAGE_TOPIC "camera/rgb/image_raw" // kinect:"camera/rgb/image_raw" webcam:"camera/image"

using namespace cv;
using namespace cv::xfeatures2d;

ImagePipeline::ImagePipeline(ros::NodeHandle& n) {
    image_transport::ImageTransport it(n);
    sub = it.subscribe(IMAGE_TOPIC, 1, &ImagePipeline::imageCallback, this);
    isValid = false;
}

void ImagePipeline::imageCallback(const sensor_msgs::ImageConstPtr& msg) {  
    try {
        if(isValid) {
            img.release();
        }
        img = (cv_bridge::toCvShare(msg, IMAGE_TYPE)->image).clone();
        isValid = true;
    } catch (cv_bridge::Exception& e) {
        std::cout << "ERROR: Could not convert from " << msg->encoding.c_str()
                  << " to " << IMAGE_TYPE.c_str() << "!" << std::endl;
        isValid = false;
    }    
}

int ImagePipeline::getTemplateID(Boxes& boxes) {
    int template_id = -1;
    if(!isValid) {
        std::cout << "ERROR: INVALID IMAGE!" << std::endl;
    } else if(img.empty() || img.rows <= 0 || img.cols <= 0) {
        std::cout << "ERROR: VALID IMAGE, BUT STILL A PROBLEM EXISTS!" << std::endl;
        std::cout << "img.empty():" << img.empty() << std::endl;
        std::cout << "img.rows:" << img.rows << std::endl;
        std::cout << "img.cols:" << img.cols << std::endl;
    } else {
        /***YOUR CODE HERE***/
        // Use: boxes.templates
                //cvtColor(img, img, CV_RGB2GRAY);

		std::vector<Mat> template_imgs;
		for (int i = 0; i < 3; i++){
			template_imgs.push_back(boxes.templates[i]);
		}
		std::vector<float> avg_dists;
                std::vector<int> n_center;
		for (int i = 0; i < 3; i++){
			int minHessian = 400;
			Ptr<SURF> detector = SURF::create(minHessian);
			std::vector<KeyPoint> keypoints_object, keypoints_scene;
			Mat descriptors_object, descriptors_scene;
			detector->detectAndCompute(img, Mat(), keypoints_object, descriptors_object);
			detector->detectAndCompute(template_imgs[i], Mat(), keypoints_scene, descriptors_scene);
			FlannBasedMatcher matcher;
			std::vector<DMatch> matches;
			matcher.match(descriptors_object, descriptors_scene, matches);
			double max_dist = 0; double min_dist = 100;
			for (int j = 0; j < descriptors_object.rows; j++){
				double dist = matches[j].distance;
				if (dist < min_dist) min_dist = dist;
				if (dist > max_dist) max_dist = dist;
			}
			std::vector<DMatch> good_matches;
                        std::vector<float> dists;
			for (int j = 0; j < descriptors_object.rows; j++){
				if (matches[j].distance < 2*min_dist){
					good_matches.push_back(matches[j]);
                                        dists.push_back(matches[j].distance);
				}
			}
                        Mat img_matches;
                        drawMatches(img, keypoints_object, template_imgs[i], keypoints_scene, good_matches, img_matches, Scalar::all(-1), Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
                        float sum = 0;
                        for (int j = 0; j < dists.size(); j++){
                            sum += dists[j];
                        }
                        avg_dists.push_back(sum/dists.size());

                        int count = 0;
                        for (int j = 0; j < good_matches.size(); j++){
                            Point2f kpt = keypoints_object[good_matches[j].queryIdx].pt;
                            if ((kpt.y > img.rows/2.0) && (kpt.x > img.cols/3.0) && (kpt.x < img.cols*2.0/3.0)){
                                 count++;
                                 //circle(img_matches, kpt, 5,Scalar(0,0,0));
   

                            }
                        }
                        n_center.push_back(count);
                        //std::cout<<"count:"<<count<<std::endl;
                        //circle(img_matches, Point(img.cols/3.0, img.rows/2.0), 5,Scalar(0,0,0));
                        //circle(img_matches, Point(img.cols*2.0/3.0, img.rows/2.0), 5,Scalar(0,0,0));
   
   
                        
                        cv::imshow("view", img_matches);
                        cv::waitKey(10);
		}
                float dist_best = 9999999.0;
                for (int i = 0; i < 3; i++){
                    if (avg_dists[i] < dist_best){
                        template_id = i;
                        dist_best = avg_dists[i];
                    }
                }
                //std::cout<<"temp id: "<<template_id<<std::endl;
                if (n_center[template_id] == 0){
                    template_id = 3;
                }
                if (template_id == 3){
                   template_id = 0;
                }
                else{
                   template_id++;
                } 

    }  
    return template_id;
}
