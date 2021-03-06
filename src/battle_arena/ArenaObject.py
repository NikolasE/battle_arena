#! /usr/bin/python

import rospy
from math import sin, cos, pi, atan2
import numpy as np
import cv2
from battle_arena_msgs.msg import ArenaObjectState

class ArenaObject(object):
    next_object_id = 100
    r = 0.1
    px_per_m = -1

    def __init__(self, object_id=-1, name="no_name"):
        self.parent_frame = "arena"
        self.position = np.array([0.0, 0.0])
        self.velocity = 0.0
        self.yaw = 0.0
        self.type = -1
        self.to_be_deleted = False
        self.x_dimension = -1
        self.y_dimension = -1
        self.visualization_state = 0  # something to run animations
        self.enabled = True
        self.red = (0, 0, 255)
        self.green = (0, 255, 0)
        self.blue = (255, 0, 0)

        if object_id < 0:
            self.id = ArenaObject.next_object_id
            ArenaObject.next_object_id += 1
        else:
            self.id = object_id
        self.name = name
        self.team_id = -1

    def _get_state_msg_base(self):
        msg = ArenaObjectState()
        msg.object_id = self.id
        msg.team_id = self.team_id
        msg.pose.x_pos = self.position[0]
        msg.pose.y_pos = self.position[1]
        msg.pose.orientation = self.yaw
        msg.x_dimension = self.x_dimension
        msg.y_dimension = self.y_dimension
        msg.type = self.type
        msg.name = self.name
        return msg

    def get_state_msg(self):
        return self._get_state_msg_base()

    def get_angle_towards(self, other):
        assert isinstance(other, ArenaObject)
        d = other.position - self.position
        return atan2(float(d[1]), float(d[0]))

    def set_pose(self, x, y, yaw):
        assert abs(yaw) < 10
        self.set_position(x, y)
        self.yaw = yaw

    def set_position(self, x, y):
        self.position[0] = float(x)
        self.position[1] = float(y)

    def circle(self, img, radius_m, color, thickness=5):
        if 0 < thickness < 1:
            thickness = int(thickness * self.px_per_m)
        cv2.circle(img, self.get_pixel_pos(), int(radius_m * self.px_per_m), color, thickness)

    def ellipse(self, img, radius_m, color, ratio=1, thickness=5):
        if 0 < thickness < 1:
            thickness = int(thickness*self.px_per_m)
        cv2.ellipse(img, self.get_pixel_pos(), (int(radius_m * self.px_per_m),
                                                int(radius_m*self.px_per_m)), 0, 0, int(360*ratio), color, thickness)

    def get_pixel_pos(self):
        return int(self.position[0]*self.px_per_m), int(self.position[1]*self.px_per_m)

    def move(self, dt):
        self.position[0] += dt * self.velocity * cos(self.yaw)
        self.position[1] += dt * self.velocity * sin(self.yaw)

    def visualize(self, img):
        if not self.enabled:
            return
        self.circle(img, self.r, (255, 255, 255), -1)

    def interact(self, other):
        assert isinstance(other, ArenaObject)
        pass

    def distance_to(self, other):
        assert isinstance(other, ArenaObject)
        return np.linalg.norm(self.position-other.position)


class Sentry(ArenaObject):
    reload_time = 6
    max_fire_distance = 0.5

    def __init__(self, team_id):
        ArenaObject.__init__(self)
        self.team_id = team_id
        self.remaining_reload_time = -1  # can fire at start of round
        self.type = ArenaObjectState.SENTRY

    def move(self, dt):
        self.remaining_reload_time -= dt

    def visualize(self, img):
        self.circle(img, 0.05, (255, 0, 255), thickness=-1)
        self.ellipse(img, Sentry.max_fire_distance, (255, 0, 255), thickness=2)

    def get_state_msg(self):
        msg = self._get_state_msg_base()
        msg.sentry_vision_radius = self.max_fire_distance
        return msg

    def interact(self, other):
        if not isinstance(other, PlayerRobot):
            return

        if other.team_id == self.team_id:
            return

        if self.remaining_reload_time > 0:
            return

        if not other.is_alive():
            return

        distance = self.distance_to(other)
        if distance > self.max_fire_distance:
            return

        # rospy.loginfo("Firing on player %i", other.player_id)
        # print "shooting towards", other.get_pixel_pos()
        self.remaining_reload_time = Sentry.reload_time

        r = Rocket(shooter_id=self.id, team_id=self.team_id)
        r.velocity = 0.1
        r.position = np.copy(self.position)
        r.yaw = self.get_angle_towards(other)

        return r


class Rocket(ArenaObject):
    max_lifetime = 50

    def __init__(self, shooter_id, team_id):
        ArenaObject.__init__(self)
        self.damage = 30
        self.velocity = 0
        self.shooter_id = shooter_id
        self.team_id = team_id
        self.trigger_distance = 0.15
        self.type = ArenaObjectState.ROCKET
        self.lifetime = Rocket.max_lifetime

    def move(self, dt):
        super(Rocket, self).move(dt)
        self.lifetime -= dt
        if self.lifetime < 0:
            self.to_be_deleted = True

    def visualize(self, img):
        if not self.enabled or self.to_be_deleted:
            return
        self.circle(img, self.r, (0, 255, 255), -1)
        self.circle(img, self.trigger_distance, (0, 0, 255), 2)

        # cv2.circle(img, self.get_int_pos(), self.r/2*self.px_per_m, (0, 255, 255), -1)
        # cv2.circle(img, self.get_int_pos(), self.trigger_distance*self.px_per_m, (0, 0, 255), 2)

    def interact(self, other):
        if not self.enabled or self.to_be_deleted:
            return

        if not isinstance(other, PlayerRobot):
            return

        if not other.is_alive():
            return

        if other.player_id == self.shooter_id:
            rospy.loginfo("Rocket does not kill its shooter")
            return

        distance = self.distance_to(other)
        if distance > self.trigger_distance:
            return

        rospy.loginfo("Player %i was hit by a rocket", other.player_id)
        other.apply_damage(self.damage)
        self.enabled = False  # or after animation
        self.to_be_deleted = True


class PlayerRobot(ArenaObject):
    max_shield = 40
    max_hp = 100

    def __init__(self, player_id, team_id, name):
        ArenaObject.__init__(self, name=name)
        assert isinstance(team_id, int)
        self.player_id = player_id
        self.team_id = team_id

        self.hp = PlayerRobot.max_hp
        self.shield = PlayerRobot.max_shield
        self.score = 0
        self.max_velocity = 1
        self.player_color = (255, 0, 0) if player_id == 1 else (0, 255, 0)
        self.type = ArenaObjectState.PLAYER

    def move(self, dt):
        # updated by markers (and maybe extrapolation)
        pass

    def no_move(self, dt):
        pass

    def get_state_msg(self):
        msg = self._get_state_msg_base()
        msg.player_hp = self.hp
        msg.player_shield = self.shield
        return msg

    def visualize(self, img):
        # player
        self.circle(img, self.r, self.player_color, -1)

        if not self.is_alive():
            self.circle(img, 0.05, self.red, -1)
            self.move = self.no_move
            return

        # shield
        if self.shield > 0:
            self.ellipse(img, self.r + 0.05, (0, 255, 255), self.shield * 1.0 / PlayerRobot.max_shield, thickness=0.01)
        # hp
        self.ellipse(img, self.r + 0.03, (0, 255, 0), self.hp * 1.0 / PlayerRobot.max_hp, thickness=2)

    def apply_damage(self, damage):
        assert damage > 0
        if self.shield > damage:
            self.shield -= damage
            rospy.loginfo("player %i: shield protected hp, now at %i+%i", self.player_id, self.shield, self.hp)
            return

        damage -= self.shield
        self.shield = 0

        self.hp -= damage
        rospy.loginfo("player %i got %i damage, no shield, remaining hp: %i", self.player_id, damage, self.hp)

        if not self.is_alive():
            rospy.logwarn("Player %i died", self.player_id)

    def is_alive(self):
        return self.hp > 0


class ShieldToken(ArenaObject):
    recharge_time = 20

    def __init__(self):
        ArenaObject.__init__(self)
        self.strength = 50
        self.activation_distance = 0.3
        self.remaining_recharge_time = -1
        self.type = ArenaObjectState.SHIELD

    def move(self, dt):
        self.remaining_recharge_time = max(0, self.remaining_recharge_time - dt)

    def get_state_msg(self):
        msg = self._get_state_msg_base()
        msg.shield_upgrade = self.strength
        msg.shield_radius = self.activation_distance
        msg.shield_remaining_recharge_time = self.remaining_recharge_time
        return msg

    def visualize(self, img):
        # cv2.circle(img, self.get_int_pos(), self.r*self.px_per_m, (0, 255, 0), -1)
        self.circle(img, self.r, self.green, -1)

        # overpaint with growing partial circle during recharge
        if self.remaining_recharge_time > 0:
            self.circle(self.r, (0, 100, 0), -1)
            # cv2.circle(img, self.get_int_pos(), self.r*self.px_per_m, (0, 100, 0), -1)
            self.ellipse(img, self.r, (0, 255, 0), 1 - self.remaining_recharge_time * 1.0 / ShieldToken.recharge_time,
                         thickness=-1)

    def interact(self, other):
        if not isinstance(other, PlayerRobot):
            return

        if self.distance_to(other) > self.activation_distance or self.remaining_recharge_time > 0:
            return

        if other.shield == other.max_shield or not other.is_alive():
            return

        rospy.loginfo("Shield token for %i shield activated by player %i", self.strength, other.id)
        other.shield += self.strength

        self.remaining_recharge_time = ShieldToken.recharge_time


if __name__ == "__main__":

    p1 = Rocket(1, 2)
    p1.velocity = 0.1
    p1.yaw = 45/180.0*pi

    l = [p1]

    for i in range(5):
        p1.move(1)
        print p1.position
