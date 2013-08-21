#ifndef ARPLUS_NODE_H
#define ARPLUS_NODE_H

#include <string>
#include <ros/ros.h>

#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv/cv.h>
#include <algorithm>
#include <ARToolKitPlus/TrackerMultiMarker.h>


class ARPlus_Node {

public:
    ARPlus_Node(ros::NodeHandle& n) : n_(n), it_(n){

        ros::NodeHandle n_param("~");

        if (!n_param.getParam("threshold", threshold_))
            threshold_ = 100;
        ROS_INFO ("\tThreshold: %d", threshold_);
        if (!n_param.getParam("marker_width", markerWidth_))
            markerWidth_ = 127.0;
        ROS_INFO ("\tMarker Width: %.1f", markerWidth_);

        sub_ = n_.subscribe ("camera_info", 1,
                             &ARPlus_Node::camInfoCallback, this);
        getCamInfo_ = false;
        rvizMarkerPub_ = n_.advertise<visualization_msgs::Marker>(
                             "visualization_marker", 0);
        posePub_ = n_.advertise<geometry_msgs::PoseStamped>("pose", 0);
    }

protected:

    //Multi Marker
    void init_tracker(ARToolKitPlus::Camera* camera) {
        tracker_.reset(new ARToolKitPlus::TrackerMultiMarker(camera->xsize,
                                                             camera->ysize,
                                                             16));

        std::string multiFile;
        ros::NodeHandle n_param ("~");
        n_param.getParam("multiPatternFile", multiFile);

        tracker_->setPixelFormat(ARToolKitPlus::PIXEL_FORMAT_BGRA);
        if (! tracker_->init(NULL, multiFile.c_str(), 1.0f, 1000.0f)) {
            ROS_ERROR("Error while reading multipattern file %s", multiFile.c_str());
            ros::shutdown();
            return;
        }

        tracker_->setThreshold(threshold_);
        //parameters that might need to change
        tracker_->setUndistortionMode(ARToolKitPlus::UNDIST_LUT);
        tracker_->setMarkerMode(ARToolKitPlus::MARKER_ID_BCH);
        tracker_->setImageProcessingMode(ARToolKitPlus::IMAGE_FULL_RES);
        tracker_->setPoseEstimator(ARToolKitPlus::POSE_ESTIMATOR_RPP);
//        tracker_->setPoseEstimator(ARToolKitPlus::POSE_ESTIMATOR_ORIGINAL);
//        tracker_->setPoseEstimator(ARToolKitPlus::POSE_ESTIMATOR_ORIGINAL_CONT);
        tracker_->setBorderWidth(0.125f);
        tracker_->setCamera(camera);
        tracker_->activateAutoThreshold(true);
        tracker_->setNumAutoThresholdRetries(20);
        tracker_->setUseDetectLite(false);

        tracker_->setHullMode(ARToolKitPlus::HULL_FULL);
//        tracker_->setHullMode(ARToolKitPlus::HULL_OFF);

    }


    void camInfoCallback (const sensor_msgs::CameraInfoConstPtr & cam_info) {

        ROS_INFO("Camera info received");
        if (!getCamInfo_)
        {
            cam_info_ = (*cam_info);
            ARToolKitPlus::Camera* camera = new ARToolKitPlus::Camera;

            camera->xsize = cam_info_.width;
            camera->ysize = cam_info_.height;

            camera->mat[0][0] = cam_info_.P[0];
            camera->mat[1][0] = cam_info_.P[4];
            camera->mat[2][0] = cam_info_.P[8];
            camera->mat[0][1] = cam_info_.P[1];
            camera->mat[1][1] = cam_info_.P[5];
            camera->mat[2][1] = cam_info_.P[9];
            camera->mat[0][2] = cam_info_.P[2];
            camera->mat[1][2] = cam_info_.P[6];
            camera->mat[2][2] = cam_info_.P[10];
            camera->mat[0][3] = cam_info_.P[3];
            camera->mat[1][3] = cam_info_.P[7];
            camera->mat[2][3] = cam_info_.P[11];

            camera->kc[0] = cam_info_.D[0];
            camera->kc[1] = cam_info_.D[1];
            camera->kc[2] = cam_info_.D[2];
            camera->kc[3] = cam_info_.D[3];
            camera->kc[4] = cam_info_.D[4];

            //VIOLATING THE PRIVATE, CHANGE Camera.h!!
            //not sure this is necessary
            camera->fc[0] = camera->mat[0][0];
            camera->fc[1] = camera->mat[1][1];
            camera->cc[0] = camera->mat[0][2];
            camera->cc[1] = camera->mat[1][2];
            camera->undist_iterations = 1;

            init_tracker(camera);

            ROS_INFO ("Subscribing to image topic");
            cam_sub_ = it_.subscribe ("image", 1,
                                      &ARPlus_Node::getTransformationCallback, this);
            getCamInfo_ = true;
            tracker_->getCamera()->printSettings();

            //stop getting camera info messages
            sub_.shutdown();
        }
    }

    void getTransformationCallback (const sensor_msgs::ImageConstPtr & image_msg) {

        cv_bridge::CvImagePtr capture;
        try {
            capture = cv_bridge::toCvCopy (image_msg,
                                           sensor_msgs::image_encodings::BGRA8);
        }
        catch (cv_bridge::Exception& e)  {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
        uchar* camerabuffer = (uchar *) ((IplImage) capture->image).imageData;


        int numDetected = tracker_->calc(camerabuffer);
        ROS_INFO("Threshod: %d, Number of markers found: %d",
                 tracker_->getThreshold(),
                 numDetected);

        if (numDetected == 0)
            return;


        const ARToolKitPlus::ARMultiMarkerInfoT* config = tracker_->getMultiMarkerConfig();


        ARFloat trans_matrix[3][4];
        memset(trans_matrix, 0, sizeof(trans_matrix));
        memcpy(trans_matrix, config->trans, sizeof(trans_matrix));
//        tracker_->getARMatrix(trans_matrix);

        //conversion from ArPoseToolkit (float) to tf (double)
        tf::Matrix3x3 rotation;
        rotation.setValue(trans_matrix[0][0], trans_matrix[0][1], trans_matrix[0][2],
                          trans_matrix[1][0], trans_matrix[1][1], trans_matrix[1][2],
                          trans_matrix[2][0], trans_matrix[2][1], trans_matrix[2][2]);
        tf::Vector3 translation(trans_matrix[0][3],
                                trans_matrix[1][3],
                                trans_matrix[2][3]);
        translation *= 0.001;

        tf::Transform T(rotation, translation);


        //dwelling with ROS structures
        visualization_msgs::Marker rvizMarker;
        tf::Vector3 markerOrigin (0, 0, 0.25 * markerWidth_ * 0.001);
        tf::Transform m (tf::Quaternion::getIdentity (), markerOrigin);
        tf::Transform markerPose = T * m; // marker pose in the camera frame
        markerPose.setRotation(markerPose.getRotation().normalized());

        tf::poseTFToMsg(markerPose, rvizMarker.pose);
        rvizMarker.header.frame_id = image_msg->header.frame_id;
        rvizMarker.header.stamp = image_msg->header.stamp;
        rvizMarker.id = 0;

        rvizMarker.scale.x = 1.0 * markerWidth_ * 0.001;
        rvizMarker.scale.y = 1.0 * markerWidth_ * 0.001;
        rvizMarker.scale.z = 0.5 * markerWidth_ * 0.001;
        rvizMarker.ns = "basic_shapes";
        rvizMarker.type = visualization_msgs::Marker::CUBE;
        rvizMarker.action = visualization_msgs::Marker::ADD;
        rvizMarker.color.r = 0.0f;
        rvizMarker.color.g = 1.0f;
        rvizMarker.color.b = 0.0f;
        rvizMarker.color.a = 1.0;

        rvizMarker.lifetime = ros::Duration(1.0);
        rvizMarkerPub_.publish(rvizMarker);

        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = image_msg->header.frame_id;
        pose.header.stamp = image_msg->header.stamp;
        pose.pose = rvizMarker.pose;
        posePub_.publish(pose);
        ROS_INFO_STREAM("Pose: "<<pose);

    }

    ros::NodeHandle n_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber cam_sub_;
    ros::Subscriber sub_;
    sensor_msgs::CameraInfo cam_info_;
    ros::Publisher rvizMarkerPub_;
    ros::Publisher posePub_;

    bool getCamInfo_;
    int threshold_;
    double markerWidth_;
    boost::shared_ptr<ARToolKitPlus::TrackerMultiMarker> tracker_;



};


#endif
