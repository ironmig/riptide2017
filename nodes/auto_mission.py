#!/usr/bin/env python

from __future__ import division
import warnings

import numpy as np
from tf import transformations
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Point, Vector3, Twist
from std_msgs.msg import String
from std_srvs.srv import Trigger
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import rospy
import actionlib


UP = np.array([0.0, 0.0, 1.0], np.float64)
UNITS = {'m': 1, 'ft': 0.3048, 'yard': 0.9144, 'rad': 1, 'deg': 0.0174533}

def rosmsg_to_numpy(rosmsg, keys=None):
    '''Convert an arbitrary ROS msg to a numpy array
    With no additional arguments, it will by default handle:
        Point2D, Point3D, Vector3D, and quaternions

    Ex:
        quat = Quaternion(1.0, 0.0, 0.0, 0.0)
        quat is not a vector, you have quat.x, quat.y,... and you can't do math on that

        But wait, there's hope!
            rosmsg_to_numpy(quat) -> array([1.0, 0.0, 0.0, 0.0])

        Yielding a vector, which you can do math on!

        Further, imagine a bounding box message, BB, with properties BB.x, BB.h, BB.y, and BB.w

            rosmsg_to_numpy(BB, ['x', 'h', 'y', 'w']) -> array([BB.x, BB.h, BB.y, BB.w])
            or...
            rosmsg_to_numpy(some_Pose2D, ['x', 'y', 'yaw']) = array([x, y, yaw])

    Note:
        - This function is designed to handle the most common use cases (vectors, points and quaternions)
            without requiring any additional arguments.
    '''
    if keys is None:
        keys = ['x', 'y', 'z', 'w']
        output_array = []
        for key in keys:
            # This is not necessarily the fastest way to do this
            if hasattr(rosmsg, key):
                output_array.append(getattr(rosmsg, key))
            else:
                break

        assert len(output_array) is not 0, "Input type {} has none of these attributes {}.".format(type(rosmsg).__name__, keys)

        return np.array(output_array).astype(np.float32)

    else:
        output_array = np.zeros(len(keys), np.float32)
        for n, key in enumerate(keys):
            output_array[n] = getattr(rosmsg, key)

        return output_array

def normalized(x):
    x = np.array(x)
    if max(map(abs, x)) == 0:
        warnings.warn('Normalizing zero-length vector to random unit vector')
        x = np.random.standard_normal(x.shape)
    x = x / max(map(abs, x))
    x = x / np.linalg.norm(x)
    return x


def get_perpendicular(a, b=None):
    a = np.array(a)
    if max(map(abs, a)) == 0:
        if b is not None:
            return get_perpendicular(b)
        return normalized(np.random.standard_normal(a.shape))

    if b is None:
        b = np.random.standard_normal(a.shape)
    b = np.array(b)
    x = np.cross(a, b)
    if max(map(abs, x)) == 0:
        return get_perpendicular(a)
    return normalized(x)


def triad((a1, a2), (b1, b2)):
    # returns quaternion that rotates b1 to a1 and b2 near a2
    # can get orientation by passing in (global, local)
    aa = get_perpendicular(a1, a2)
    A = np.array([normalized(a1), aa, normalized(np.cross(a1, aa))])
    bb = get_perpendicular(b1, b2)
    B = np.array([normalized(b1), bb, normalized(np.cross(b1, bb))])
    rot = A.T.dot(B)
    return transformations.quaternion_from_matrix(
        [(a, b, c, 0) for a, b, c in rot] +
        [(0, 0, 0, 1)])

def look_at(forward, upish=UP):
    # assumes standard forward-left-up body coordinate system
    return triad((forward, upish), ([1, 0, 0], UP))


def look_at_without_pitching(forwardish, up=UP):
    # assumes standard forward-left-up body coordinate system
    return triad((up, forwardish), (UP, [1, 0, 0]))


def look_at_camera(forward, upish=UP):
    # assumes camera right-down-forward coordinate system
    return triad((forward, upish), (UP, [0, -1, 0]))

class PoseEditor(object):
    UNITS = {'m': 1, 'ft': 0.3048, 'yard': 0.9144, 'rad': 1, 'deg': 0.0174533}
    def __init__(self, pose):
        print type(pose)
        if isinstance(pose, PoseStamped):
            self._from_pose(pose.pose)
        elif isinstance(pose, Pose):
            self._from_pose(pose)
        elif pose == None:
            self.position = np.array([0,0,0])
            self.orientaiton = np.array([0,0,0,1])
        else:
            self.position, self.orientation = pose

    def __repr__(self):
        return "p: {}, q: {}".format(self.position, self.orientation)

    @property
    def _rot(self):
        return transformations.quaternion_matrix(self.orientation)[:3, :3]

    @property
    def distance(self, position):
        return np.linalg.norm(self.position - position)

    def _from_pose(self, pose):
        self.position = rosmsg_to_numpy(pose.position)
        self.orientation = rosmsg_to_numpy(pose.orientation)

    def set_position(self, position):
        return PoseEditor([np.array(position), np.array(self.orientation)])

    def rel_position(self, rel_pos):
        position = self.position + self._rot.dot(np.array(rel_pos))
        return self.set_position(position)

    def forward(self, dist, unit='m'):
        return self.rel_position([dist * PoseEditor.UNITS[unit], 0, 0])

    def backward(self, dist, unit='m'):
        return self.rel_position([-dist * PoseEditor.UNITS[unit], 0, 0])

    def left(self, dist, unit='m'):
        return self.rel_position([0, dist * PoseEditor.UNITS[unit], 0])

    def right(self, dist, unit='m'):
        return self.rel_position([0, -dist  * PoseEditor.UNITS[unit], 0])

    def stop(self):
        return self.forward(0)

    # Orientation
    def set_orientation(self, orientation):
        orientation = np.array(orientation)
        if orientation.shape == (4, 4):
            # We're getting a homogeneous rotation matrix - not a quaternion
            orientation = transformations.quaternion_from_matrix(orientation)

        return PoseEditor([self.position, orientation])

    def yaw_left(self, angle, unit='rad'):
        return self.set_orientation(transformations.quaternion_multiply(
            transformations.quaternion_about_axis(angle * PoseEditor.UNITS[unit], UP),
            self.orientation
        ))

    def yaw_right(self, angle, unit='rad'):
        return self.yaw_left(-angle, unit)

    def look_at_rel(self, rel_point):
        return self.set_orientation(look_at_without_pitching(rel_point))  # Using no pitch here since we are 2D

    def look_at(self, point):
        return self.look_at_rel(point - self.position)

    @property
    def pose(self):
        p = Point(*self.position)
        q = Quaternion(*self.orientation)
        return Pose(position= p, orientation= q)
    
    @property
    def pose_numpy(self):
        return np.array([self.position, self.orientation])

"""

If fails- try drive forward auto (velocity or effort?)

If in center
1) Hope that target is visible, otherwise abort (or drive forward a bit)

2) Drive to pose 

"""


class AutoProgram(object):
    def __init__(self):
        self.latest_vision_pose = None
        self.vision_pose_fresh = False
        self.status = "idle"
        self.stauts_timer = rospy.Timer(rospy.Duration(0.5), self.status_ping)
        self.vision_sub = rospy.Subscriber("/lifter_pose", PoseStamped, self.vision_cb)
        self.status_pub = rospy.Publisher("/auto_status", String, queue_size=10)
        self.left_server = rospy.Service("/auto_left", Trigger, self.left_cb)
        self.right_server = rospy.Service("/auto_right", Trigger, self.right_cb)
        self.move_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)

    def vision_cb(self, msg):
        self.latest_vision_pose = msg
        self.vision_pose_fresh = True

    def status_ping(self, event):
        self.status_pub.publish(self.status)

    def wait_for_vision(self, timeout):
        start = rospy.get_rostime()
        while rospy.get_rostime() - start < timeout:
            if self.vision_pose_fresh:
                return True
            rospy.sleep(rospy.Duration(0.1))
        return False

    def right_cb(self, msg):
        print "Right"
        return {"success" : True, "message" : ""}

    def make_goal(self, pose, frame="map"):
        goal = MoveBaseGoal()
        goal.header.frame_id = frame
        goal.header.stamp = rospy.get_rostime()
        goal.pose = pose

    def run_left(self, timer):
        self.status = "running"
        print "Running Left"
        
        #Move forward, yaw_left
        pose_one = PoseEditor().forward(3, units="m").yaw_right(45, "deg").pose
        goal_one = self.make_goal(pose_one, frame="base_link")
        self.move_client.send_goal(goal_one)
        if not self.move_client.wait_for_result(timeout=rospy.Duration(10)):
            self.status = "Error: goal one timed out"
            return
        print "Goal one ", self.move_client.get_result()
        self.vision_pose_fresh = False
        
        #Wait to see target now that facing
        if not self.wait_for_vision(3):
            self.status = "Error: cant see target"
            return
        
        #Move to infront of vision target
        pose_two = PoseEditor(self.latest_vision_pose).yaw_left(180, "deg").backward(2, "m").pose
        goal_two = self.make_goal(pose_two, frame="base_link")
        self.move_client.send_goal(goal_two)
        if not self.move_client.wait_for_result(timeout=rospy.Duration(10)):
            self.status = "Error: goal two timed out"
            return
        print "Goal two ", self.move_client.get_result()
        self.vision_pose_fresh = False
        
        #Wait to see target now that facing
        if not self.wait_for_vision(3):
            print "No new vision seen, using old"
            
        
        pose_three = PoseEditor(self.latest_vision_pose).yaw_left(180, "deg").pose
        goal_three= self.make_goal(pose_three, frame="base_link")
        self.move_client.send_goal(goal_three)
        if not self.move_client.wait_for_result(timeout=rospy.Duration(5)):
            self.status = "Error: goal three timed out"
            return
        print "Goal three ", self.move_client.get_result()
        self.status = "done"
        
    def left_cb(self, msg):
        self.mission_timer = rospy.Timer(rospy.Duration(2.5), self.run_left, True)
        return {"success" : True, "message" : ""}

if __name__ == '__main__':
    print "Running vision"
    rospy.init_node("auto_program")
    program = AutoProgram()
    rospy.spin()

