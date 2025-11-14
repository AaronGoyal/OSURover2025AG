// pc_filter.cpp
//
// Osian Leahy
//
// This node removes the extreme points from the point cloud data given for HW7, leaving just the table and stuff on it

// Include the basic ROS 2 stuff.
#include <rclcpp/rclcpp.hpp>

//Include the ROS2 msg for point cloud:
#include <sensor_msgs/msg/point_cloud2.hpp>

//Include the pointcloud stuff
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/crop_box.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <memory>

//Placeholders for callback binding:
using std::placeholders::_1;

// The idiom in C++ is the same as in Python; we create a class that
// inherits from the ROS 2 Node class.
class CountObjects : public rclcpp::Node {
public:
	
	//Simplify the syntax by abstracting this namespace
	using PointCloud2 = sensor_msgs::msg::PointCloud2;

	//(I Think)This needs to be public because it's used to construct the class
	CountObjects() : Node("pc_filter"),
		tf_buffer_(std::make_unique<tf2_ros::Buffer>(this->get_clock())),
		tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)),
		target_frame_("base_link") {
		
		//Subscribe to the rosbag stream
		subscriber_ = this->create_subscription<PointCloud2>("raw_point_cloud", 5, std::bind(&CountObjects::sub_callback, this, _1));
		
		//Republish to:
		publisher_ = this->create_publisher<PointCloud2>("truncated_point_cloud", 10);

		RCLCPP_INFO(this->get_logger(), "pc_filter started");

	}

private:

	//Class variable prototypes, 2 pass compiler blah blah
	rclcpp::Publisher<PointCloud2>::SharedPtr publisher_;
	rclcpp::Subscription<PointCloud2>::SharedPtr subscriber_;
	std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
	std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
	std::string target_frame_;

	//Where the magic is supposed to happen...
	void sub_callback(const PointCloud2::SharedPtr msg) {
		//Log that we've recieved a msg
		RCLCPP_INFO(this->get_logger(),"Point Cloud Recieved");

		//First, pull in the point cloud, translate from ROS PC2 to PCL:
		//We get RGB data with this camera, as specified in the PC2 message fields
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>); //Create the point cloud object
		pcl::fromROSMsg(*msg, *pcl_cloud); //pass it and the msg by reference for conversion
		
		//Remove the unneeded data using cropbox.
		//Bounding points
		float x_min = -2.0;
		float x_max = 2.0;
		float y_min = -2.0;
		float y_max = 0.4;
		float z_min = -1;
		float z_max = 1;

		//Create the cropbox filter:
		pcl::CropBox<pcl::PointXYZRGB> box_filter;
		box_filter.setInputCloud(pcl_cloud);
		box_filter.setMin(Eigen::Vector4f(x_min,y_min,z_min,1.0f));
		box_filter.setMax(Eigen::Vector4f(x_max,y_max,z_max,1.0f));

		//Create output object:
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud_trunc(new pcl::PointCloud<pcl::PointXYZRGB>);
		//filter it ig?
		box_filter.filter(*pcl_cloud_trunc);

		//Convert back into a ros message and output:
		PointCloud2 msg_out;
		pcl::toROSMsg(*pcl_cloud_trunc, msg_out);
		msg_out.header = msg->header;

		//Transform the truncated cloud into the target frame
		sensor_msgs::msg::PointCloud2 cloud_in_target;
		try {
			geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_->lookupTransform(
				target_frame_, msg_out.header.frame_id, msg_out.header.stamp);
			tf2::doTransform(msg_out, cloud_in_target, transform_stamped);
		} catch (const tf2::TransformException &ex) {
			RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s",
				msg_out.header.frame_id.c_str(), target_frame_.c_str(), ex.what());
			return;
		}

		//Publish and log the result:
		this->publisher_->publish(cloud_in_target);
		RCLCPP_INFO(this->get_logger(), "point cloud truncated.");

	}
};


// This is the entry point for the executable. 
int main(int argc, char **argv) {
	// Initialize ROS.
	rclcpp::init(argc, argv);

	// Create a node instance and store a shared pointer to it.
	auto node = std::make_shared<CountObjects>();

	// Give control to ROS via the shared pointer.
	rclcpp::spin(node);

	// Once the event handler is done, shut things down nicely.
	rclcpp::shutdown();

	// Main always returns 0 on success.
	return 0;
}