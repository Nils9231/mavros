/**
 * @brief UUV1Pose plugin
 * @file mocap_pose_estimate.cpp
 * @author Tony Baltovski <tony.baltovski@gmail.com>
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2014,2015,2016 Vladimir Ermakov, Tony Baltovski.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <eigen_conversions/eigen_msg.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>


namespace mavros {
namespace extra_plugins{
/**
 * @brief uuv_com plugin
 *
 * Sends motion capture data to FCU.
 */
class UUV5PosePlugin : public plugin::PluginBase
{
public:
        UUV5PosePlugin() : PluginBase(),
                mp_nh("~UUV5Pose")
        { }

        void initialize(UAS &uas_)
        {
                PluginBase::initialize(uas_);

                uuv5_pose_sub = mp_nh.subscribe("pose", 1, &UUV5PosePlugin::uuv5_pose_cb, this);

        }

        Subscriptions get_subscriptions()
        {
                return { /* Rx disabled */ };
        }

private:
        ros::NodeHandle mp_nh;

        ros::Subscriber uuv5_pose_sub;

        /* -*- low-level send -*- */
        void uuv5_pose_send
                (uint64_t usec,
                        Eigen::Quaterniond &q,
                        Eigen::Vector3d &v)
        {
                mavlink::common::msg::UUV_FIVE_POS pos;

                pos.time_usec = usec;
                ftf::quaternion_to_mavlink(q, pos.q);
                pos.x = v.x();
                pos.y = v.y();
                pos.z = v.z();

                UAS_FCU(m_uas)->send_message_ignore_drop(pos);
        }

        /* -*- mid-level helpers -*- */
        void uuv5_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &pose)
        {
                Eigen::Quaterniond q_enu;

                tf::quaternionMsgToEigen(pose->pose.orientation, q_enu);
                auto q = ftf::transform_orientation_enu_ned(
                                        ftf::transform_orientation_baselink_aircraft(q_enu));

                auto position = ftf::transform_frame_enu_ned(
                                Eigen::Vector3d(
                                        pose->pose.position.x,
                                        pose->pose.position.y,
                                        pose->pose.position.z));

                uuv5_pose_send(pose->header.stamp.toNSec() / 1000,
                                q,
                                position);
        }

};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::UUV5PosePlugin, mavros::plugin::PluginBase)
