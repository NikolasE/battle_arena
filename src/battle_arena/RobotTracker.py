#! /usr/bin/python

import rospy

from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
from tf import transformations
# from math import pi


class RobotTracker:
    def __init__(self):
        self.marker_id_2_player = dict({1: 1, 0: 2})
        self.sub_markers = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.marker_callback)
        self._player_poses = dict()
        self.alpha = 0.95
        rospy.sleep(0.5)

    def get_player_pose(self, player_id):
        assert player_id in self.marker_id_2_player.values()
        if player_id not in self._player_poses:
            rospy.logwarn("No pose for player %i receieved yet", player_id)
            return None
        return self._player_poses[player_id]

    def marker_callback(self, data):
        assert isinstance(data, AlvarMarkers)
        for marker in data.markers:
            assert isinstance(marker, AlvarMarker)
            p = marker.pose.pose.position
            o = marker.pose.pose.orientation
            yaw = transformations.euler_from_quaternion([o.x, o.y, o.z, o.w])[2]
            if marker.id in self.marker_id_2_player:
                player = self.marker_id_2_player[marker.id]
                # print "Found new positing for player", player
                # tracking, filtering could be done here
                # todo: project into ground plane

                if player not in self._player_poses:
                    self._player_poses[player] = [p.x, p.y, yaw]
                else:
                    x,y,phi = self._player_poses[player]
                    x = x + self.alpha*(p.x-x)
                    y = y + self.alpha*(p.y-y)
                    self._player_poses[player] = [x, y, yaw]

if __name__ == "__main__":
    rospy.init_node("tracker_test")
    rt = RobotTracker()
    rospy.spin()
