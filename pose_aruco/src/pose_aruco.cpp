#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <math.h>
#include <iostream>
#include <vector>
#include <array>
#include <limits>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <tuple>


typedef cv_bridge::CvImageConstPtr cv_bridge_image;
cv_bridge_image raw_image;

void raw_image_cb (const sensor_msgs::ImageConstPtr& ros_image){

  raw_image = cv_bridge::toCvShare(ros_image, sensor_msgs::image_encodings::RGB8);

}

void show_camera_params(cv::Mat& cameraMatrix, cv::Mat& distCoeffs){
    ROS_INFO("Intrinsic parameters of the camera");
    for(int i{0};i<cameraMatrix.rows;++i ){
        for(int c{0};c<cameraMatrix.cols;++c){
            std::cout<<cameraMatrix.at<double>(i,c)<<" ";
        }
        std::cout<<std::endl;
    }

    ROS_INFO("Distortion Coefficients: ");
    for(int i{0}; i < distCoeffs.cols;++i){
        std::cout<<distCoeffs.at<double>(0,i)<<" ";
    }
    std::cout<<std::endl;
}

void read_camera_params(sensor_msgs::CameraInfoConstPtr& camera_params, int& image_width, 
    int& image_height, cv::Mat& cameraMatrix, cv::Mat& distCoeffs){

    int r = 0;
    for(int i{0}; i<camera_params->K.size(); ++i){
        cameraMatrix.at<double>(r,i%3) =  camera_params->K[i];
        if(((i+1)%3) == 0) ++r;
    }

    for(int i{0};i<camera_params->D.size();++i){
        distCoeffs.at<double>(0,i) = camera_params->D[i];
    }

    image_width = camera_params->width;
    image_height = camera_params->height;
    ROS_INFO("Received the camera parameters");
    ROS_INFO("Image Width: %d",image_width);
    ROS_INFO("Image Height: %d",image_height);
    show_camera_params(cameraMatrix,distCoeffs);
}

double getArucoDist(cv::Vec3d& aruco_translation){
    
    tf2::Vector3 camera_frame(0.0,0.0,0.0); //need to be changed to camera_frame pose w.r.t base_link
    tf2::Vector3 arucoPose(aruco_translation[0],aruco_translation[1],aruco_translation[2]);
    double camToArucoDist = camera_frame.distance(arucoPose);
    
    return camToArucoDist;
}

Eigen::MatrixXd getRotTransAsMat(tf::Matrix3x3& rot_matrix, tf2::Vector3& translation){

    Eigen::MatrixXd transform(4,4);

    std::cout<<"Rot matrix to homogeneous: \n";
    for(int i{0};i<3;++i){
        auto col = rot_matrix.getColumn(i);
        std::cout<<"column: "<<i<<" x: "<< col.getX()<<" y: "<< col.getY()<<" z: "<< col.getZ()<<std::endl;
        transform.col(i)<<col.getX(),col.getY(),col.getZ(),0.0;
    }

    transform.col(3)<<translation.getX(),translation.getY(),translation.getZ(),1.0;

    return transform;

}

Eigen::MatrixXd getFrameTransform(std::string sourceFrame, std::string targetFrame){

    tf::StampedTransform transform;
    tf::TransformListener listener;

    auto tf_time = ros::Time(0);//at current instant
    listener.waitForTransform(sourceFrame, targetFrame, tf_time, ros::Duration(2.0));
    listener.lookupTransform(sourceFrame, targetFrame, tf_time, transform);


    tf::Quaternion rot_q = transform.getRotation();//quaternion from ros transforms
    tf::Matrix3x3 rot_matrix(rot_q);

    tf2::Vector3 translation(transform.getOrigin()[0],transform.getOrigin()[1],transform.getOrigin()[2]);

    auto m_transform = getRotTransAsMat(rot_matrix,translation);
    std::cout<<"Tranformation Matrix: "<<m_transform<<std::endl;
    return m_transform;

}

std::tuple<tf2::Quaternion,tf2::Vector3> getGTPose(int& aruco_id, cv::Vec3d& translation, cv::Vec3d& rotation){

    //pose of aruco marker w.r.t world
    auto t_aruco_world = getFrameTransform("world","aruco_"+std::to_string(aruco_id));

    //pose of base_link w.r.t camera_link
    auto t_base_link_camera = getFrameTransform("camera_link","base_link");

    
    //aruco_marker pose w.r.t. camera_link
    tf::Quaternion marker_orientation;
    marker_orientation.setRPY(rotation[0],rotation[1],rotation[2]);
    marker_orientation = marker_orientation.normalize();
    tf::Matrix3x3 rot_matrix_marker(marker_orientation);

    tf2::Vector3 marker_translation(translation[0],translation[1],translation[2]);

    auto t_aruco_camera = getRotTransAsMat(rot_matrix_marker, marker_translation);

    //camera_link w.r.t aruco_marker
    auto t_camera_aruco = t_aruco_camera.inverse();


    //base_link pose w.r.t world
    auto t_base_link_world = t_aruco_world * t_camera_aruco * t_base_link_camera;


    //extract quaternion and translation
    tf2::Matrix3x3 r_matrix(t_base_link_world(0,0),t_base_link_world(0,1),t_base_link_world(0,2),
                            t_base_link_world(1,0),t_base_link_world(1,1),t_base_link_world(1,2),
                            t_base_link_world(2,0),t_base_link_world(2,1),t_base_link_world(2,2));

    tf2::Quaternion quat_base_world;
    r_matrix.getRotation(quat_base_world);

    tf2::Vector3 trans_base_world(t_base_link_world(0,3),t_base_link_world(1,3),t_base_link_world(2,3));

    return std::make_tuple(quat_base_world,trans_base_world);

}

geometry_msgs::PoseWithCovarianceStamped gtMsg(uint seq, tf2::Vector3& translation, tf2::Quaternion& rotation){

    geometry_msgs::PoseWithCovarianceStamped pose;
    pose.header.seq = seq;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "world";

    pose.pose.pose.position.x = translation.getX();
    pose.pose.pose.position.y = translation.getY();
    pose.pose.pose.position.z = translation.getZ();

    geometry_msgs::Quaternion quat;
    quat = tf2::toMsg(rotation);
    pose.pose.pose.orientation = quat;

    return pose;

}

int main(int argc, char** argv){
    
    ros::init(argc,argv,"ground_truth_aruco");
    ros::NodeHandle nh;
    ros::Subscriber image_sub = nh.subscribe("/camera/color/image_raw", 30, raw_image_cb);
    ros::Publisher gt_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/ground_truth/pose",30);
    
    // record the camera parameters once from the topic /camera/color/camera_info;
    ROS_INFO("Waiting to receive the camera parameters....");
    auto camera_params = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/camera/color/camera_info",nh);
    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat distCoeffs = cv::Mat::zeros(1,5,CV_64F);
    int imageWidth,imageHeight;
    read_camera_params(camera_params, imageWidth, imageHeight, cameraMatrix, distCoeffs);
    
    //detect and find the aruco close to camera continuously
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

    std::cout<<"Defined the aruco dictionary"<<std::endl;
    std::cout<<"Checking for the live feed: "<<std::endl;
    ros::spinOnce();
    ros::Rate loop_rate(30);
    uint seq{0};
    while (ros::ok()) {
        
        ROS_INFO("Accessing the image from shared pointer");
        cv::Mat image = raw_image->image;
        ROS_INFO("Image available");

        //detect markers
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(image, dictionary, corners, ids);

        if (ids.size() > 0) {
            cv::aruco::drawDetectedMarkers(image, corners, ids);
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(corners, 0.15, cameraMatrix, distCoeffs, rvecs, tvecs);
            
            //get aruco close to the camera
            double dist{std::numeric_limits<double>::max()};
            uint arucoIndex{tvecs.size()};
            // draw axis for each marker
            for(uint i=0; i<ids.size(); i++){ 
                cv::aruco::drawAxis(image, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
                auto camToArucoDist = getArucoDist(tvecs[i]);
                if(camToArucoDist<dist){
                    dist = camToArucoDist;
                    arucoIndex = i;
                }
            }

            auto [quaternion,translation] = getGTPose(ids[arucoIndex], tvecs[arucoIndex], rvecs[arucoIndex]);

            auto pose_msg = gtMsg(++seq, translation, quaternion);

            gt_pub.publish(pose_msg);

            /*ROS_INFO("Aruco Marker Detected Close to Camera: %d",ids[arucoIndex]);
            ROS_INFO("Pose of the Aruco Marker");
            std::cout<<"Rotation: "<<rvecs[arucoIndex]<<std::endl;
            std::cout<<"Translation: "<<tvecs[arucoIndex]<<std::endl;*/
        }

        cv::imshow("Raw Image", raw_image->image);
        cv::imshow("Marker Image", image);
        cv::waitKey(1);
        
        loop_rate.sleep();
        ros::spinOnce();


    }
    return 0;
}