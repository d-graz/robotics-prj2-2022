#include <ros/ros.h>

// to read from "/odom" topic
#include <nav_msgs/Odometry.h>

// to broadcast tf transform
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

class OdomDynamicBroadcaster {
    
    ros::NodeHandle n;
    ros::Subscriber odometry_listener;
    tf2_ros::TransformBroadcaster tf_broadcaster;

public:
    OdomDynamicBroadcaster() {
        odometry_listener = n.subscribe("odom", 1000, &OdomDynamicBroadcaster::broadcastCallback, this);
    }

    void broadcastCallback(const nav_msgs::Odometry::ConstPtr& msg) {

        geometry_msgs::TransformStamped position_tf;
        
        // tf header
        position_tf.header.stamp = msg->header.stamp;
        position_tf.header.frame_id = "odom";
        position_tf.header.frame_id = msg->header.frame_id;

        // tf child frame
        position_tf.child_frame_id = "base_footprint";

        position_tf.transform.translation.x = msg->pose.pose.position.x;
        position_tf.transform.translation.y = msg->pose.pose.position.y;
        position_tf.transform.translation.z = msg->pose.pose.position.z;

        position_tf.transform.rotation.w = msg->pose.pose.orientation.w;
        position_tf.transform.rotation.x = msg->pose.pose.orientation.x;
        position_tf.transform.rotation.y = msg->pose.pose.orientation.y;
        position_tf.transform.rotation.z = msg->pose.pose.orientation.z;

        tf_broadcaster.sendTransform(position_tf);
    }

    void main_loop() {
        ros::spin();
    }
};

// Starts node (broadcast tf from "/odometry" topic)
int main(int argc, char* argv[]) {
    ros::init(argc, argv, "odom_dynamic_broadcaster");
    OdomDynamicBroadcaster odomDynamicBroadcaster;
    ROS_INFO("odom_dynamic_broadcaster: ready");
    odomDynamicBroadcaster.main_loop();

    return 0;
}