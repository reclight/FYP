#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <myworkcell_core/LocalizePart.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float32.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <geometry_msgs/Vector3.h>

class Localizer
{
	public:
	ros::ServiceServer server_;
	tf::TransformListener listener_;
	ros::Subscriber ar_sub_;
	geometry_msgs::PoseArray last_msg_;
	ros::Subscriber object_height_sub;
	ros::Publisher pose_pub_;
	ros::Subscriber dimension_sub;
	
	geometry_msgs::Vector3 dimension_msg;

	Localizer(ros::NodeHandle& nh)
	{		
		dimension_sub = nh.subscribe<geometry_msgs::Vector3>("/point_cloud_node/Dimension", 1, &Localizer::dimensionCallback, this);
		ar_sub_ = nh.subscribe<geometry_msgs::PoseArray>("/point_cloud_node/Poses", 1, &Localizer::visionCallback, this);
		server_ = nh.advertiseService("localize_part", &Localizer::localizePart, this);
		pose_pub_ = nh.advertise<geometry_msgs::Pose>("/robotrelative", 1);
		
	}

	void dimensionCallback(const geometry_msgs::Vector3 msg)
	{
		dimension_msg = msg;
	}
	
	void visionCallback(const geometry_msgs::PoseArray msg)
	{
		last_msg_ = msg;
	}
	

	bool localizePart(myworkcell_core::LocalizePart::Request& req, myworkcell_core::LocalizePart::Response& res)
	{
		geometry_msgs::PoseArray objectpose = last_msg_;
		
		if(objectpose.poses.empty())
		{
			return false;
		}
		
		res.pose = objectpose.poses[0];

		std::cout<<res.pose;
		
		//transform relative to pose
		tf::TransformBroadcaster br;
		tf::Transform transform;
		ros::Rate rate(10.0);
		
		transform.setOrigin( tf::Vector3(res.pose.position.x, res.pose.position.y, res.pose.position.z));
		transform.setRotation ( tf::Quaternion(res.pose.orientation.x , res.pose.orientation.y, res.pose.orientation.z, res.pose.orientation.w));
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_link", "hand"));
		ros::Duration(0.5).sleep();

		//transform camera to target
		tf::Transform cam_to_target, req_to_target;
		tf::poseMsgToTF(objectpose.poses[0], cam_to_target);
		ROS_INFO_STREAM(req.base_frame);

		tf::StampedTransform hand_to_cam, cam_to_hand;
		listener_.lookupTransform("hand", "camera_link", ros::Time(0), hand_to_cam);

		ros::Duration(1).sleep();
				
		tf::Transform hand_to_target;
		hand_to_target = hand_to_cam * cam_to_target;


		tf::poseTFToMsg(hand_to_target, res.pose);
		res.pose.position.x = res.pose.position.x + 0.130; 
		
		//rotate the gripper to be in x-y direction
		tf::Quaternion q_orig, q_rot, q_new, q_r, q_1, q_1_neg, q_2;
		double r=0, p=-1.5707963267, y=0;  // Rotate the previous pose by 180* about X

		q_rot = tf::createQuaternionFromRPY(r, p, y);
		
		quaternionMsgToTF(res.pose.orientation , q_orig);
		q_new = q_rot * q_orig;
		q_new.normalize();

		r=1.5707963267, p=0, y=0;
		q_rot = tf::createQuaternionFromRPY(r, p, y);
		q_new = q_rot * q_new;
		q_new.normalize();

		quaternionTFToMsg(q_new, res.pose.orientation);

		//end of rotation
		tf::poseMsgToTF(res.pose, hand_to_target);
		listener_.lookupTransform("camera_link", "hand", ros::Time(0), cam_to_hand);
		
		cam_to_target = cam_to_hand * hand_to_target;

		/*geometry_msgs::Pose robot_rel;
		tf::poseTFToMsg(cam_to_target, robot_rel);
		pose_pub_.publish(robot_rel);

		double roll, pitch, yaw;
		quaternionMsgToTF(robot_rel.orientation , q_1);
		q_rot = tf::createQuaternionFromRPY(y, n, 2*r);
		tf::Matrix3x3(q_1).getRPY(roll, pitch, yaw);
		if(yaw<0)
		{
			yaw = yaw + 2 * r;
		}
		q_1 = tf::createQuaternionFromRPY(roll, pitch, yaw);
		quaternionTFToMsg(q_1, robot_rel.orientation);
		tf::poseMsgToTF(robot_rel, cam_to_target);*/

		tf::StampedTransform req_to_cam;
		listener_.lookupTransform(req.base_frame, "camera_link", ros::Time(0), req_to_cam);

		req_to_target = req_to_cam * cam_to_target;
		tf::poseTFToMsg(req_to_target, res.pose);
		
		

		return true;
		

		
	}
};


int main(int argc, char* argv[])
{
	ros::init(argc, argv, "vision_node");
	ros::NodeHandle nh;
	
	Localizer localizer(nh);
	ROS_INFO("Vision node starting");

	ros::spin();
}
