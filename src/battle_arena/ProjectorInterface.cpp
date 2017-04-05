#include <battle_arena/ProjectorInterface.h>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/buffer_client.h>

#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

using namespace std;
using namespace cv;


BattleProjectorInterface::BattleProjectorInterface() :
	ProjectorCalibrator(),
	use_projector_publisher_(true),  /// param?
	background_color_(125,0,0)
{
	ROS_INFO("Subscribing");
	string ns = "/arena_manager/";
	sub_object_states_ = nh_private_.subscribe(ns+"object_states", 10, &BattleProjectorInterface::object_states_cb, this);

	img_ = Mat(projector_resolution_, CV_8UC3);
	img_.setTo(background_color_);

	sensor_msgs::CameraInfo projector_intrinsics;

	ProjectorCalibrator::readCalibration(data_folder_path_+"/projector_calib.bag", projector_intrinsics, projector2camera_);

	/// TODO: move to function
	ros::Publisher pub_tf_static = nh_private_.advertise<tf2_msgs::TFMessage>("/tf_static", 1);
	pub_proj_intrinsics = nh_private_.advertise<sensor_msgs::CameraInfo>("/projector_intrinsics", 1, true);

	ros::Duration(0.5).sleep();

	tf2_msgs::TFMessage tfm;
	tfm.transforms.push_back(projector2camera_);
	pub_tf_static.publish(tfm);

	pub_proj_intrinsics.publish(projector_intrinsics);

	pinhole_.fromCameraInfo(projector_intrinsics);

	tf2_ros::BufferClient tf2_client("/tf2_buffer_server", 200.0);
	if (!tf2_client.waitForServer(ros::Duration(0.5)))
	{
		ROS_ERROR("No tf2 server at '/tf2_buffer_server'");
		return;
	}

	arena_frame_ = nh_private_.param<std::string>("/arena_frame", "marker");
	projector_frame_ = projector_intrinsics.header.frame_id;

	try {
		arena2projector_ = tf2_client.lookupTransform(projector_frame_, arena_frame_,
													  ros::Time::now(), ros::Duration(0.5));
	} catch (...) {
		ROS_ERROR("TF2 exception %s to %s", arena_frame_.c_str(), projector_frame_.c_str());
		return;
	}


	depth_frame_ = "ensenso_base";
	// depth_frame_ = "rgbd_cam_rgb_optical_frame";
	try {
		arena2depth_ = tf2_client.lookupTransform(depth_frame_, arena_frame_,
													  ros::Time::now(), ros::Duration(0.5));
	} catch (...) {
		ROS_ERROR("TF2 exception %s to %s", arena_frame_.c_str(), depth_frame_.c_str());
		return;
	}

	pub_objects_projector_ = nh_private_.advertise<pcl_cloud>("objects_projector_frame", 1);
	redraw_timer_ = nh_private_.createTimer(ros::Duration(0.05), &BattleProjectorInterface::redraw_trigger, this);
}



void BattleProjectorInterface::object_states_cb(const battle_arena_msgs::ArenaObjectStateListConstPtr& object_list)
{
	ROS_INFO_ONCE("Received first object states");
	object_states.clear();
	for (const auto& o: object_list->states)
	{
		object_states[o.object_id] = o;
	}
}


void BattleProjectorInterface::redraw_trigger(const ros::TimerEvent& e)
{
//	ROS_INFO("Drawing, last call took %.1f ms", e.profile.last_duration.toSec()*100.0);
	draw_visualization_simple();
}

/**
 * @brief BattleProjectorInterface::draw_visualization_simple draws a simple visualization for testing
 */
void BattleProjectorInterface::draw_visualization_simple()
{	
	img_.setTo(background_color_);

	pcl_cloud arena_positions;

	if (object_states.size() == 0)
	{
		send_image_to_projector(img_);
		return;
	}

	for (const auto& o: object_states)
	{
		pcl::PointXYZ p(o.second.pose.x_pos, o.second.pose.y_pos, 0);
		arena_positions.push_back(p);
	}

	/// transform points into depth camera frame
	pcl_cloud depth_positions;
	pcl::transformPointCloud(arena_positions, depth_positions, tf2::transformToEigen(arena2depth_));
	depth_positions.header.frame_id = depth_frame_;
	pub_objects_projector_.publish(depth_positions);

	/// projecting points into projector
	vector<cv::Point3f> points_3d;
	ROS_INFO("in projector frame:");
	for (const auto&p : depth_positions)
	{
		points_3d.push_back(Point3f(p.x, p.y, p.z));
		ROS_INFO("3d: %f %f %f", p.x, p.y, p.z);
	}

//	/// no further transformation (we could fill these with the arena2projector_ transformation)
	Mat rvec(1,3,CV_64FC1); rvec.setTo(0);
	Mat tvec(1,3,CV_64FC1); tvec.setTo(0);

	geometry_msgs::TransformStamped depth2projector = ProjectorCalibrator::invert(projector2camera_);
	geometryTransform2openCV(depth2projector.transform, rvec, tvec);


	vector<Point2f> projected;
//	cout << rvec << tvec << endl;
	cv::projectPoints(points_3d, rvec, tvec, pinhole_.fullIntrinsicMatrix(), pinhole_.distortionCoeffs(), projected);

//	cout << "Projector intrinsics " <<  pinhole_.fullIntrinsicMatrix() << endl;

	/// simple visualization
//	for (const auto&p : projected)
//	{
//		ROS_INFO("%f %f ", p.x, p.y);
//		cv::circle(img_, p, 100, CV_RGB(0, 0, 255), -1);
//	}


//	for (size_t i=0; i<object_states.size(); ++i)
	int i=-1;
	for (const auto& s: object_states)
	{
		i+=1;
		const Point& p = Point(projected[i].x, projected[i].y);
		battle_arena_msgs::ArenaObjectState state = s.second;
		ROS_INFO("Visualizaing type %i", state.type);


		if (state.type == state.ROCKET)
		{
			cv::circle(img_, p, 20, CV_RGB(255, 0, 0), -1);
		}

		if (state.type == state.SENTRY)
		{
			cv::circle(img_, p, 100, CV_RGB(100, 100, 0), -1);

		}

		if (state.type == state.PLAYER)
		{
			ROS_INFO("player: %f %f", state.player_hp, state.player_shield);

			float hp = state.player_hp;
			float shield = state.player_shield;

			if (hp <=0)
			{
				cv::circle(img_, p, 70, CV_RGB(255, 0, 0), -1);
			}else
			{
				if (shield > 0)
				{
//					cv::ellipse(img_, p, Size(120,120), 0, 0, 360, CV_RGB(0, 255, 255), 2);
					cv::ellipse(img_, p, Size(100,100), 0, 0, shield/40.0*360, CV_RGB(255, 255, 255), 10);
				}
				cv::ellipse(img_, p, Size(90,90),0,0, hp/100.0*360, CV_RGB(0, 255, 0), 10);
			}
		}

	}

	if (use_projector_publisher_)
	{
		send_image_to_projector(img_);
	}else
	{
		image_screen_.show_image(img_);
	}
}
