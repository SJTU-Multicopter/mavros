/**
 * @brief ViconPoseEstimate plugin
 * @file Vicon_pose_estimate.cpp

 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2014 M.H.Kabir.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <mavros/setpoint_mixin.h>
#include <pluginlib/class_list_macros.h>
#include <eigen_conversions/eigen_msg.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "nav_msgs/Odometry.h"
#include <cmath>

namespace mavplugin {
/**
 * @brief Vicon pose estimate plugin
 *
 * Send pose estimation from various Vicon estimators
 * to FCU position and attitude estimators.
 *
 */
class ViconPoseEstimatePlugin : public MavRosPlugin,
    private TF2ListenerMixin<ViconPoseEstimatePlugin> {
public:
    ViconPoseEstimatePlugin() :
        sp_nh("~vicon_pose"),
		uas(nullptr),
		tf_rate(10.0),
		init(true)
	{ };

	void initialize(UAS &uas_)
	{
		bool tf_listen;

		uas = &uas_;

		// tf params
		sp_nh.param("tf/listen", tf_listen, false);
		sp_nh.param<std::string>("tf/frame_id", tf_frame_id, "map");
        sp_nh.param<std::string>("tf/child_frame_id", tf_child_frame_id, "vicon");
		sp_nh.param("tf/rate_limit", tf_rate, 50.0);

        vicon_sub = sp_nh.subscribe("/mocap/pose", 2, &ViconPoseEstimatePlugin::vicon_cb, this);

	}

	const message_map get_rx_handlers() {
		return { /* Rx disabled */ };
	}

private:
	friend class TF2ListenerMixin;
	ros::NodeHandle sp_nh;
	UAS *uas;

    ros::Subscriber vicon_sub;

	std::string tf_frame_id;
	std::string tf_child_frame_id;
	double tf_rate;
	ros::Time last_transform_stamp;

	geometry_msgs::PoseStamped  pos_old;//chg
	geometry_msgs::PoseStamped  pos_rec;//chg
	bool init;

	/* -*- low-level send -*- */

    void vicon_position_estimate(uint64_t usec,
			float x, float y, float z,
			float roll, float pitch, float yaw) {
		mavlink_message_t msg;
		mavlink_msg_vision_position_estimate_pack_chan(UAS_PACK_CHAN(uas), &msg,
				usec,
				x,
				y,
				z,
				roll,
				pitch,
				yaw);
		UAS_FCU(uas)->send_message(&msg);
	//	ROS_INFO("x=%f,y=%f,z=%f",x,y,z);
	}

    void vicon_cb(const geometry_msgs::PoseStamped &pos) {

        // geometry_msgs::PoseStamped pos = msg;

        // pos.header.stamp = ros::Time::now(); //use ros time

        vicon_position_estimate(pos.header.stamp.toNSec() / 1000,
                pos.pose.position.x, -pos.pose.position.y, -pos.pose.position.z,
                pos.pose.orientation.x, -pos.pose.orientation.z, pos.pose.orientation.y);

//        vicon_position_estimate(pos.header.stamp.toNSec() / 1000,
//                -pos.pose.position.y, pos.pose.position.x, -pos.pose.position.z,
//                0.0, 0.0, 0.0);
	}
};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::ViconPoseEstimatePlugin, mavplugin::MavRosPlugin)
