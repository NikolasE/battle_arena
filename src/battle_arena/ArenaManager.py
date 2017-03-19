#! /usr/bin/python

import rospy

from battle_arena.ArenaObject import PlayerRobot, Rocket, ShieldToken, Sentry
import cv2
import numpy as np
from itertools import combinations
from math import pi
from copy import deepcopy
from battle_arena_msgs.msg import ArenaObjectState, ArenaObjectStateList
from std_msgs.msg import Float32


class ArenaManager:

    def __init__(self):
        self._pub_object_states = rospy.Publisher("~object_states", ArenaObjectStateList, queue_size=10)

        self.px_per_m = 6
        self.arena_width = 150
        self.arena_height = 100
        self.width_px = self.arena_width*self.px_per_m
        self.height_px = self.arena_height*self.px_per_m
        self.arena_image = None  # np.zeros((self.height_px, self.width_px, 3), np.uint8)
        self.arena_background = np.zeros((self.height_px, self.width_px, 3), np.uint8)
        self.arena_objects = list()
        self.setup()

        self._pub_battle_time = rospy.Publisher("~battle_time", Float32, queue_size=1)
        self.publish_object_states()

    def setup(self):
        p1 = PlayerRobot(player_id=1, team_id=1, name="red player")
        p1.position = [20, 20]
        p1.velocity = 10
        p1.yaw = 45 / 180.0 * pi
        self.arena_objects.append(p1)

        # b1 = Rocket(shooter_id=-1, team_id=-1)
        # b1.position[0] = 100
        # b1.velocity = 10
        # b1.yaw = 90 / 180.0 * pi
        # self.arena_objects.append(b1)

        # s1 = ShieldToken()
        # s1.position = np.array([500, 500])
        # self.arena_objects.append(s1)

        sen1 = Sentry(42)
        sen1.set_position(0, 0)
        self.arena_objects.append(sen1)

        sen2 = Sentry(p1.team_id+2)
        sen2.set_position(500, 300)
        sen2.reload_time = 0.2
        self.arena_objects.append(sen2)

        self.arena_background[:] = (60, 0, 0)

    def publish_object_states(self):
        msg = ArenaObjectStateList()
        msg.states = [o.get_state_msg() for o in self.arena_objects]
        self._pub_object_states.publish(msg)

    def visualize(self):
        self.arena_image = deepcopy(self.arena_background)
        for o in self.arena_objects:
            o.visualize(self.arena_image)

    def cleanup_objects(self):
        self.arena_objects = filter(lambda o: not o.to_be_deleted, self.arena_objects)

    def iterate(self, dt):
        for o in self.arena_objects:
            o.move(dt)

        for a, b in combinations(self.arena_objects, 2):
            created = a.interact(b)
            if created:
                self.arena_objects.append(created)
            created = b.interact(a)
            if created:
                self.arena_objects.append(created)

if __name__ == "__main__":
    rospy.init_node("arena_manager")
    am = ArenaManager()

    # cv2.namedWindow("arena")

    game_duration = 10
    step = 0.1

    # for i in range(1000):
    while True:
        am.iterate(step)
        game_duration -+ step
        if game_duration < 0:
            break

        am.publish_object_states()
        am.visualize()
        cv2.imshow("arena", am.arena_image)
        am.cleanup_objects()
        cv2.waitKey(10)
        if rospy.is_shutdown():
            break

        # rospy.sleep(0.2)

