/**
 * @brief Lidar plugin
 * @file vision_pose_estimate.cpp
 * @author M.H.Kabir <mhkabir98@gmail.com>
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
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

#include <unordered_map>
#include <mavros/utils.h>
#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <eigen_conversions/eigen_msg.h>

#include <sensor_msgs/Range.h>

namespace mavplugin {
/**
 * @brief Vision pose estimate plugin
 *
 * Send pose estimation from various vision estimators
 * to FCU position and attitude estimators.
 *
 */
class LidarPlugin : public MavRosPlugin{
public:
	LidarPlugin() :
		sp_nh("~lidar"),
		uas(nullptr)
	{ 
		ROS_INFO("Mavros Lidar started!!!!!!");
	};

	void initialize(UAS &uas_)
	{
		uas = &uas_;
		lidar_sub = sp_nh.subscribe("/range", 10, &LidarPlugin::lidar_cb, this);
	}

	const message_map get_rx_handlers() {
		return { /* Rx disabled */ };
	}

private:
	ros::NodeHandle sp_nh;
	UAS *uas;
	ros::Subscriber lidar_sub;

	/* -*- low-level send -*- */

	void distance_sensor(uint32_t time_boot_ms,
			uint32_t min_distance,
			uint32_t max_distance,
			uint32_t current_distance,
			uint8_t type, uint8_t id,
			uint8_t orientation, uint8_t covariance) {
		mavlink_message_t msg;
		mavlink_msg_distance_sensor_pack_chan(UAS_PACK_CHAN(uas), &msg,
				time_boot_ms,
				min_distance,
				max_distance,
				current_distance,
				type,
				id,
				orientation,
				covariance);
		UAS_FCU(uas)->send_message(&msg);
		//ROS_INFO("Lidar Received!");
	}

	/* -*- mid-level helpers -*- */

	void lidar_cb(const sensor_msgs::Range::ConstPtr &msg) {
		uint8_t type = MAV_DISTANCE_SENSOR_LASER;
		uint8_t covariance = 0.1;
		uint8_t sensor_id = 3;
		int orientation = MAV_SENSOR_ROTATION_ROLL_180;
		
		float height = msg->range; // CHG
		if(height < 0.31) height = 0.11;	

		distance_sensor(
			//msg->header.stamp = ros::Time::now().toNSec() / 1000000,
			ros::Time::now().toNSec() / 1000000,
			5, //msg->min_range / 1E-2,
			msg->max_range / 1E-2,
			height / 1E-2,
			type,
			sensor_id,
			orientation,
			covariance);
	}

};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::LidarPlugin, mavplugin::MavRosPlugin)
