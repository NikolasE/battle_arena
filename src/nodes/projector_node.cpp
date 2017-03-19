
#include <iostream>
#include <battle_arena/ProjectorInterface.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "projector_generator");

	BattleProjectorInterface bpi;

	ros::spin();
	return 0;
}
