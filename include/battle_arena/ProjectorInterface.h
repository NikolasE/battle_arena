#ifndef PROJECTOR_INTERFACE_H
#define PROJECTOR_INTERFACE_H

#include <projector_calibration/projector_calibration.h>
#include <battle_arena_msgs/ArenaObjectStateList.h>
#include <projector_calibration/ImageScreen.h>

class BattleProjectorInterface : public ProjectorCalibrator
{
public:
	BattleProjectorInterface ();

private:
	bool use_projector_publisher_; /// true if images are send to projector via topic

	ImageScreen image_screen_;

	cv::Mat img_; /// projector image
	std::string arena_frame_; /// tf frame of arena
	std::string depth_frame_; /// tf frame of arena


	std::string projector_frame_; /// tf frame of projector
	geometry_msgs::TransformStamped arena2projector_, projector2camera_;

	geometry_msgs::TransformStamped arena2depth_; /// hack

	image_geometry::PinholeCameraModel pinhole_;


	void redraw_trigger(const ros::TimerEvent& e);
	void draw_visualization_simple();

	std::map<int, battle_arena_msgs::ArenaObjectState> object_states;

	void object_states_cb(const battle_arena_msgs::ArenaObjectStateListConstPtr& object_list);
	ros::Subscriber sub_object_states_;

	ros::Timer redraw_timer_;
	cv::Scalar background_color_;

	ros::Publisher pub_objects_projector_;
	ros::Publisher pub_proj_intrinsics;
};

#endif

