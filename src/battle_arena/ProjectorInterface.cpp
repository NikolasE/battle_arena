#include <battle_arena/ProjectorInterface.h>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/buffer_client.h>

#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

using namespace std;
using namespace cv;


BattleProjectorInterface::BattleProjectorInterface() :
	ProjectorCalibrator(),
	background_color_(125,0,0)
{
	ROS_INFO("Subscribing");
	string ns = "/arena_manager/";
	sub_object_states_ = nh_private_.subscribe(ns+"object_states", 10, &BattleProjectorInterface::object_states_cb, this);

	img_ = Mat(projector_resolution_, CV_8UC3);
	img_.setTo(background_color_);

	sensor_msgs::CameraInfo projector_intrinsics;

	ProjectorCalibrator::readCalibration(data_folder_path_+"/projector_calib.bag", projector_intrinsics, projector2camera_);

	pinhole_.fromCameraInfo(projector_intrinsics);

	tf2_ros::BufferClient tf2_client("/tf2_buffer_server", 200.0);
	if (!tf2_client.waitForServer(ros::Duration(0.5)))
	{
		ROS_ERROR("No tf2 server at '/tf2_buffer_server'");
		return;
	}

	arena_frame_ = nh_private_.param<std::string>("/arena_frame", "arena");
	projector_frame_ = projector_intrinsics.header.frame_id;

	try {
		arena2projector_ = tf2_client.lookupTransform(projector_frame_, arena_frame_,
													  ros::Time::now(), ros::Duration(0.5));
	} catch (...) {
		ROS_ERROR("TF2 exception %s to %s", arena_frame_.c_str(), projector_frame_.c_str());
	}

	pub_objects_projector_ = nh_private_.advertise<pcl_cloud>("objects_projector_frame", 1);
	redraw_timer_ = nh_private_.createTimer(ros::Duration(0.1), &BattleProjectorInterface::redraw_trigger, this);
}



void BattleProjectorInterface::object_states_cb(const battle_arena_msgs::ArenaObjectStateListConstPtr& object_list)
{
	ROS_INFO_ONCE("Received first object states");
	for (const auto& o: object_list->states)
	{
		object_states[o.object_id] = o;
	}
}


void BattleProjectorInterface::redraw_trigger(const ros::TimerEvent& e)
{
	ROS_INFO("Drawing, last call took %.1f ms", e.profile.last_duration.toSec()*100.0);
	draw_visualization_simple();
}

/**
 * @brief BattleProjectorInterface::draw_visualization_simple draws a simple visualization for testing
 */
void BattleProjectorInterface::draw_visualization_simple()
{
	img_.setTo(background_color_);

	pcl_cloud arena_positions;
	ROS_INFO("Size: %zu", object_states.size());

	if (object_states.size() == 0)
	{
		send_image_to_projector(img_);
		return;
	}

	for (const auto& o: object_states)
	{
		ROS_INFO("%f", o.second.pose.x_pos);
		pcl::PointXYZ p(o.second.pose.x_pos/1000.0, o.second.pose.x_pos/1000.0, 0);
		arena_positions.push_back(p);
	}

	/// transform points into projector frame
	pcl_cloud projector_positions;
	pcl::transformPointCloud(arena_positions, projector_positions, tf2::transformToEigen(arena2projector_));
	projector_positions.header.frame_id = projector_frame_;
	pub_objects_projector_.publish(projector_positions);

	/// projecting points into projector
	vector<cv::Point3f> points_3d;
	for (const auto&p : projector_positions)
	{
		points_3d.push_back(Point3f(p.x, p.y, p.z));
		ROS_INFO("3d: %f %f %f", p.x, p.y, p.z);
	}

	/// no further transformation (we could fill these with the arena2projector_ transformation)
	Mat rvec(1,3,CV_64FC1); rvec.setTo(0);
	Mat tvec(1,3,CV_64FC1); tvec.setTo(0);

	vector<Point2f> projected;
	cv::projectPoints(points_3d, rvec, tvec, pinhole_.fullIntrinsicMatrix(), pinhole_.distortionCoeffs(), projected);

	cout << pinhole_.fullIntrinsicMatrix() << endl;

	ROS_INFO("%i %i", img_.cols, img_.rows);
	/// simple visualization
	for (const auto&p : projected)
	{
		ROS_INFO("%f %f ", p.x, p.y);
		cv::circle(img_, p, 100, CV_RGB(0, 0, 255), -1);
	}



	send_image_to_projector(img_);
}
