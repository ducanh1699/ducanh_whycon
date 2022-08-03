#ifndef ROBOT_POSE_PUBLISHER_H
#define ROBOT_POSE_PUBLISHER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>

namespace whycon {
  class RobotPosePublisher {
    public:
      RobotPosePublisher(ros::NodeHandle& n);
      ros::Publisher pub_target;
      ros::Subscriber pose_sub, mavposeSub_, ;
      boost::shared_ptr<tf::TransformBroadcaster> broadcaster;

      double axis_length_tolerance;
      std::string world_frame, target_frame, axis_file;
      void on_poses(const geometry_msgs::PoseArrayConstPtr& pose_array);
      void mavposeCallback(const geometry_msgs::PoseStamped &msg);

    private:
        int num_pose;
        bool received_home_pose = false;
        geometry_msgs::Pose home_pose_;
        Eigen::Vector3d mavPos_, uav_Marker_pose, uav_Marker_pose1, world_Marker_pose, world_Marker_pose1, world_Marker_pose2, world_Marker_pose3, uav_Marker_pose1, uav_Marker_pose2,uav_Marker_pose3;
        Eigen::Vector4d mavAtt_;
        Eigen::Matrix3f uav2cam_matrix_, neu2uav_matrix_;
        Eigen::Quaternionf quat;


  };
}

#endif // ROBOT_POSE_PUBLISHER_H