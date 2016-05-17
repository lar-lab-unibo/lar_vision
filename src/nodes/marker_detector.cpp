#include "ros/ros.h"
#include <dynamic_reconfigure/server.h>

#include "iostream"
#include "aruco/aruco.h"

#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "geometry_msgs/Pose.h"

#include <tf/transform_broadcaster.h>
#include <dynamic_reconfigure/server.h>

#include "std_msgs/Int32MultiArray.h"

#include "lar_tool_utils/UDPNode.h"


static const char* DEFAULT_VIDEO_NODE_PARAMETERS_FILE = "/opt/visionSystemLegacy/data/kinect_parameters.txt";

using namespace cv;

/**
    Parameters
 */

//std::string camera_topic_name = "/usb_cam/image_raw";///camera/rgb/image_mono";
//std::string camera_info_file_name = "/home/daniele/catkin_ws/src/lar_visionsystem/data/lifeCamera640x480.yml";
std::string camera_topic_name = "/camera/rgb/image_raw";
std::string camera_info_file_name = "/home/daniele/Desktop/workspace_bonmet_ros/src/lar_vision/data/calibrations/ideal.yml";


ros::NodeHandle* nh = NULL;
tf::TransformBroadcaster* br;

/**
    Aruco Detectors
 */
aruco::CameraParameters camera_parameters;
aruco::MarkerDetector marker_detector;
double marker_size = 0.1f;
double offx = 0.0;
double offy = 0.0;
double offz = 0.0;
int marker_id =  800;
std::string camera_frame = "comau_t_6_camera";

vector<aruco::Marker> markers_list;
vector<aruco::Marker> filtered_markers_list;
std::map<int, tf::Transform> filtered_markers_tf;
std::map<int, tf::Transform> approach_markers_tf;
int current_approach_target = -1;

Mat getTransformMatrix(Mat* rotation, Mat* translation, int cv_type) {

        Mat zeros = Mat::zeros(1, 3, CV_32FC1);

        if (rotation == NULL) {
                rotation = new Mat(Mat::eye(3, 3, CV_32FC1));
        }

        if (translation == NULL) {
                translation = new Mat(Mat::zeros(3, 1, CV_32FC1));
        }


        Mat rot(*rotation);
        vconcat(rot, zeros, rot);

        Mat one = (Mat_<float>(1, 1) << 1);
        Mat trans(*translation);
        if (trans.size().width > trans.size().height) {
                trans = trans.t();
        }
        if (trans.size().height == 3) {
                vconcat(trans, one, trans);
        }

        Mat t;
        hconcat(rot, trans, t);
        return t;
}

Mat getRodriguezMatrix(Mat* v) {
        Mat mat;
        if (v != NULL) {
                Rodrigues(*v, mat);
        }
        return mat;
}

Mat getRodriguezVector(Mat* m) {
        Mat v;
        if (m != NULL) {
                Rodrigues(*m, v);
        }
        return v;
}

tf::Transform matToTF(cv::Mat& mat){
        tf::Vector3 origin;
        tf::Matrix3x3 tf3d;
        origin.setValue(
                static_cast<float>(mat.at<float>(0,3))/1000.0f,
                static_cast<float>(mat.at<float>(1,3))/1000.0f,
                static_cast<float>(mat.at<float>(2,3))/1000.0f
                );

        tf3d.setValue(
                static_cast<float>(mat.at<float>(0,0)), static_cast<float>(mat.at<float>(0,1)), static_cast<float>(mat.at<float>(0,2)),
                static_cast<float>(mat.at<float>(1,0)), static_cast<float>(mat.at<float>(1,1)), static_cast<float>(mat.at<float>(1,2)),
                static_cast<float>(mat.at<float>(2,0)), static_cast<float>(mat.at<float>(2,1)), static_cast<float>(mat.at<float>(2,2))
                );

        for(int i = 0; i < 3; i++) {
                for(int j = 0; j < 3; j++) {
                        tf3d[i][j] = mat.at<float>(i,j);

                }
                std::cout<<std::endl;

        }


        tf::Quaternion tfqt;
        tf3d.getRotation(tfqt);
        tfqt = tfqt.normalized();

        tf::Transform transform;
        transform.setOrigin(origin);
        transform.setRotation(tfqt);
        return transform;
}

Mat getTMarker(aruco::Marker& marker) {
        Mat rot = getRodriguezMatrix(&(marker.Rvec));
        Mat trans = marker.Tvec.clone()*1000;
        Mat T = getTransformMatrix(&rot, &trans, CV_32FC1);
        return T;
}


void lowPassFilter(tf::Transform& newTF, tf::Transform& oldTF,tf::Transform& targetTF, float alpha){
        for(int i = 0; i < 3; i++)
                targetTF.getOrigin()[i] = (1.0f-alpha)*oldTF.getOrigin()[i] + alpha*newTF.getOrigin()[i];

        tf::Quaternion newQ = newTF.getRotation();
        tf::Quaternion oldQ = oldTF.getRotation();
        tf::Quaternion targetQ = targetTF.getRotation();

        for(int i = 0; i < 4; i++)
                targetQ[i] = (1.0f-alpha)*oldQ[i] + alpha*newQ[i];

        targetTF.setRotation(targetQ);
}

/**
   Image Callback
 */
cv::Mat current_image;
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
        cv::Mat source = cv_bridge::toCvShare(msg, "bgr8")->image;
        cv::blur( source, source, cv::Size(3,3));
        marker_detector.detect(source, markers_list, camera_parameters, marker_size, false);

        float alpha = 0.99f;
        //float alpha = 0.11f;
        ROS_INFO("Found: %d",(int)markers_list.size());
        for (unsigned int i = 0; i < markers_list.size(); i++) {
                cv::Mat T = getTMarker(markers_list[i]);
                std::cout <<"\n"<<markers_list[i].id<<":  "<<  T.at<float>(0,2)<<","<< T.at<float>(1,2)<<","<< T.at<float>(2,2)<<"\n";
                tf::Transform tf = matToTF(T);

                std::map<int, tf::Transform>::iterator iter = filtered_markers_tf.find(markers_list[i].id);
                if(iter != filtered_markers_tf.end()) {
                        tf::Transform ftf = iter->second;

                        lowPassFilter(tf,ftf,ftf,alpha);

                        filtered_markers_tf[markers_list[i].id] = ftf;
                }else{
                        filtered_markers_tf[markers_list[i].id] = tf;
                }

        }

        for (unsigned int i = 0; i < markers_list.size(); i++) {
                if(current_approach_target==markers_list[i].id) {
                        markers_list[i].draw(source, cv::Scalar(255, 255, 0), 6,true);
                }else{
                        markers_list[i].draw(source, cv::Scalar(0, 255, 0), 2,true);
                }
                aruco::CvDrawingUtils::draw3dAxis(source, markers_list[i], camera_parameters);
        }

        //cv::flip(source,source,-1);
        cv::imshow("view", source);
        current_image = source;
        cv::waitKey(1000/30);
}



/*--------------------------------------------------------------------
 * main()
 * Main function to set up ROS node.
 *------------------------------------------------------------------*/

int main(int argc, char **argv)
{
        /** Camera Parameters */
        std::cout << "Reading from: "<<camera_info_file_name<<std::endl;
        camera_parameters.readFromXMLFile(camera_info_file_name);
        ros::init(argc, argv, "marker_detector_target");
        nh = new ros::NodeHandle("~");

        /* PARAMS */
        nh->param<double>("marker_size", marker_size, 0.1f);
        nh->param<double>("offx",offx, 0.0f);
        nh->param<double>("offy",offy, 0.0f);
        nh->param<double>("offz",offz, 0.0f);
        nh->param<int>("marker_id", marker_id, 800);
        nh->param<std::string>("camera_frame",camera_frame,"comau_t_6_camera");
        nh->param<std::string>("camera_topic",camera_topic_name,"/camera/rgb");

        /* OFFSET */
        Eigen::Matrix4d T_MARKER_OFFSET  =Eigen::Matrix4d::Identity();
        T_MARKER_OFFSET(0,3) = offx;
        T_MARKER_OFFSET(1,3) = offy;
        T_MARKER_OFFSET(2,3) = offz;
        geometry_msgs::Pose marker_pose;

        ROS_INFO("Marker detector single target run!");


        image_transport::ImageTransport it(*nh);
        image_transport::Subscriber sub = it.subscribe(camera_topic_name.c_str(), 1, imageCallback);
        image_transport::Publisher image_publisher = it.advertise("lar_visionsystem/marker_detector_feedback", 1);
        std::string marker_full_name = "lar_marker_"+boost::lexical_cast<std::string>(marker_id);
        ros::Publisher cartesian_publisher = nh->advertise<geometry_msgs::Pose>("lar_visionsystem/"+marker_full_name, 1);

        br = new tf::TransformBroadcaster();

        cv::namedWindow("view");
        cv::startWindowThread();


        while(nh->ok()) {

                ROS_INFO("Searching for Marker: %d with size %f m!",marker_id,marker_size);

                std::map<int, tf::Transform>::iterator iter;

                std::stringstream ss;
                std::stringstream ss2;
                ROS_INFO("Size: %d",(int)filtered_markers_tf.size());
                for (iter = filtered_markers_tf.begin(); iter != filtered_markers_tf.end(); iter++) {

                        ss.str("");
                        ss << "lar_marker_"<<iter->first;
                        //std::cout << ss.str()<<std::endl;

                        if(iter->first==marker_id) {
                          ROS_INFO("FOUND!");
                          tf::Transform t_marker = iter->second;
                          tf::Transform t_marker_offset;
                          Eigen::Matrix4d T_MARKER;
                          lar_tools::eigen_4x4_d_to_tf(T_MARKER,t_marker,true);
                          T_MARKER = T_MARKER * T_MARKER_OFFSET;
                          lar_tools::eigen_4x4_d_to_tf(T_MARKER,t_marker_offset);
                          br->sendTransform(tf::StampedTransform(t_marker_offset, ros::Time::now(),  camera_frame,marker_full_name));
                          lar_tools::geometrypose_to_tf(marker_pose,  t_marker_offset,true);
                          cartesian_publisher.publish(marker_pose);
                        }

                }



                sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", current_image).toImageMsg();
                image_publisher.publish(msg);

                ros::spinOnce();
                //std::system("clear");

        }
        cv::destroyWindow("view");

        return 0;
}
