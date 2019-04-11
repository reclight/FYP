#include <ros/ros.h>
#include <myworkcell_core/LocalizePart.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <string>
#include <geometry_msgs/Vector3.h>

class ScanNPlan
{
	public:
	ScanNPlan(ros::NodeHandle& nh, std::string base)
	{	
		vision_client_ = nh.serviceClient<myworkcell_core::LocalizePart>("localize_part");
		gripper_pub = nh.advertise<std_msgs::String>("/CModelCommand", 100);
		dimension_sub = nh.subscribe<geometry_msgs::Vector3>("/point_cloud_node/Dimension", 1, &ScanNPlan::dimensionCallback, this);
		pose_status_sub = nh.subscribe<std_msgs::Bool>("/poseStatus", 1, &ScanNPlan::poseStatusCallback, this);
		base_frame = base;
		move_robot_ = nh.advertiseService("move_robot", &ScanNPlan::start, this);
		
	}

	void dimensionCallback(const geometry_msgs::Vector3 msg)
	{
		dimension_msg = msg;
	}

	void poseStatusCallback(const std_msgs::Bool msg)
	{
		posestatus = msg;
	}

	bool start(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
	{
		ROS_INFO("Attempting to localize part");
		myworkcell_core::LocalizePart srv;
		srv.request.base_frame= base_frame;
		
		int grippersize = 220.4415 - dimension_msg.y * 2582.25+10;


		ROS_INFO_STREAM("Requesting pose in base frame: " << base_frame);

		if (!vision_client_.call(srv))
		{
			ROS_ERROR("Could not localize part");
			res.success=false;
			res.message="Could not localize part";
			return true;
		}

		ROS_INFO_STREAM("part localized: " << srv.response);
		geometry_msgs::Pose move_target = srv.response.pose;
		geometry_msgs::Pose move_top = move_target;

		float height=move_target.position.z;

		move_top.position.z=0.9;

		moveit::planning_interface::MoveGroupInterface move_group("manipulator");

		move_group.setPoseReferenceFrame(base_frame);
		move_group.setPoseTarget(move_top);
		move_group.move();

		move_group.setPoseReferenceFrame(base_frame);
		move_group.setPoseTarget(move_target);
		move_group.move();

		std_msgs::String c;
		ROS_INFO_STREAM("part localized: " << dimension_msg);
		
		c.data = std::to_string(grippersize);
		std_msgs::String o;
		o.data ='o';

		//open gripper
		ROS_INFO("clossing gripper at ");
		gripper_pub.publish(c);
		ros::Duration(1).sleep();

		//increase z first
		move_target.position.z=0.9;
		move_target.orientation.x=1;
		move_target.orientation.y=0;
		move_target.orientation.z=0;
		move_target.orientation.w=0;
		move_group.setPoseTarget(move_target);
		move_group.move();

		//send to destination place
		destination_target.position.x=0.15;
		destination_target.position.y=0.6;
		destination_target.position.z=0.9;

		destination_target.orientation.x=1;
		destination_target.orientation.y=0;
		destination_target.orientation.z=0;
		destination_target.orientation.w=0;

		//destination_target.orientation.x=0.718;
		//destination_target.orientation.y=-0.696;
		//destination_target.orientation.z=0;
		//destination_target.orientation.w=0;

		move_group.setPoseTarget(destination_target);
		move_group.move();

		//std_msgs::Float64 height;
		//height.data = dimension_msg.x;

		destination_target.position.x=0.15;
		destination_target.position.y=0.6;
		destination_target.position.z=height;

		move_group.setPoseTarget(destination_target);
		move_group.move();
		//}
			
		gripper_pub.publish(o);

		ros::Duration(1).sleep();
		destination_target.position.x=0.15;
		destination_target.position.y=0.6;
		destination_target.position.z=0.9;

		move_group.setPoseTarget(destination_target);
		move_group.move();
		
		//move back to original position
		destination_target.position.x=-0.2;
		destination_target.position.y=0.6;
		destination_target.position.z=0.9;

		destination_target.orientation.x=1;
		destination_target.orientation.y=0;
		destination_target.orientation.z=0;
		destination_target.orientation.w=0;

		move_group.setPoseTarget(destination_target);
		move_group.move();

		res.success=true;
		return true;
	}

	private:
		ros::ServiceClient vision_client_;
		ros::Publisher gripper_pub;
		ros::Subscriber gripper_sub;
		ros::Subscriber gripper_size_sub;
		ros::Subscriber pose_status_sub;
		ros::Subscriber dimension_sub;
		ros::Subscriber object_height_sub;
		ros::ServiceServer move_robot_;

		geometry_msgs::Pose destination_target;
		std_msgs::Int64 gripperposition;
		std_msgs::Bool posestatus;
		geometry_msgs::Vector3 dimension_msg;
		
		std::string base_frame;
};

int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "myworkcell_node");

	ros::AsyncSpinner async_spinner(1);
	async_spinner.start();

	ros::NodeHandle nh;
	ros::NodeHandle private_node_handle ("~");
	std::string base_frame;

	private_node_handle.param<std::string>("base_frame", base_frame, "world");

	ScanNPlan app(nh, base_frame);
	
	ROS_INFO("ScanNPlan node has been initialized");
	ros::spin();

}
