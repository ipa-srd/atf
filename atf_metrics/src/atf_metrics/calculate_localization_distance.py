#!/usr/bin/env python
import rospy
from roslib.message import get_message_class
from nav_msgs.msg import Odometry
import rosgraph
import tf
import math
import PyKDL
import numpy as np
import yaml

class CalculateLocalizationDistanceParamHandler:
    def __init__(self):
        """
        Class for returning the corresponding metric class with the given parameter.
        """
        pass

    def parse_parameter(self, testblock_name, params):
        """
        Method that returns the metric method with the given parameter.
        :param params: Parameter
        """
        metrics = []
        if type(params) is not list:
            rospy.logerr("metric config not a list")
            return False

        for metric in params:
            try:
                groundtruth_topic = metric["groundtruth_topic"]
                root_frame = metric["root_frame"]
                measured_frame = metric["measured_frame"]
                metrics.append(
                    CalculateLocalizationDistance(root_frame, measured_frame, None,
                                                  groundtruth_topic))
            except (TypeError, KeyError):
                rospy.logwarn("No groundtruth topic given in testblock '%s' try to use tf evaluation", testblock_name)
                groundtruth_topic = None
            try:
                groundtruth_frame = metric["ground_truth_frame"]
                root_frame = metric["root_frame"]
                measured_frame = metric["measured_frame"]

                metrics.append(
                    CalculateLocalizationDistance(root_frame, measured_frame, groundtruth_frame,
                                                  None))
            except (TypeError, KeyError):
                rospy.logwarn("No groundtruth frame given in testblock '%s'", testblock_name)
                groundtruth_frame = None
        print metrics
        return metrics



class CalculateLocalizationDistance:
    def __init__(self, root_frame, measured_frame, groundtruth_frame, groundtruth_topic):
        """
        Class for calculating the distance between the root_frame-measured_frame  pose and if available the
        groundtruth_topic pose or the root_frame-groundtruth_frame pose
        """
        self.active = False
        self.root_frame = root_frame
        self.measured_frame = measured_frame
        self.groundtruth_topic = groundtruth_topic
        self.groundtruth_frame = groundtruth_frame
        self.tf_sampling_freq = 2.
        self.finished = False
        self.delta_trans = []
        self.delta_rot =  []
        self.max_pos_error = rospy.get_param("max_pos_error", 2.5)
        self.max_ang_error = rospy.get_param("max_ang_error", 2.0)
        self.fails = 0
        self.fail_time = rospy.get_time()
        self.fail_timeout = 5 #sec to wait until new fail is counted
        self.count = 0
        self.master = rosgraph.Master(rospy.get_name())
        self.listener = tf.TransformListener()
        self.latest_gt_time = rospy.get_time()
        if groundtruth_topic is not None:
            self.sub = rospy.Subscriber(groundtruth_topic, rospy.AnyMsg, self.groundtruth_callback)
        elif groundtruth_frame is not None:
            rospy.Timer(rospy.Duration.from_sec(1 / self.tf_sampling_freq), self.record_tf)
            rospy.loginfo("started Timer")
    def start(self, timestamp):
        self.active = True

    def stop(self, timestamp):
        self.active = False
        self.finished = True

    def pause(self, timestamp):
        self.active = False
        self.first_value = True

    def purge(self, timestamp):
        pass

    def groundtruth_callback(self, data):
        if (rospy.get_time() - self.latest_gt_time) < (1. / self.tf_sampling_freq):
            return
        self.latest_gt_time = rospy.get_time()
        topic_types = self.master.getTopicTypes()
        msg_name = [ty for tp, ty in topic_types if tp == self.sub.name][0]
        msg_class = get_message_class(msg_name)
        msg = msg_class().deserialize(data._buff)
        if (self.active):
            try:
                trans_gt = []
                rot_loc = PyKDL.Rotation.Quaternion(0.0,0.0,0.0,1)
                rot_gt = PyKDL.Rotation.Quaternion(0.0,0.0,0.0,1)
                self.listener.waitForTransform(self.root_frame,
                                               self.measured_frame,
                                               msg.header.stamp, #rospy.Time(0),
                                               rospy.Duration.from_sec(1 /(2* self.tf_sampling_freq)))
                (trans_loc, rot) = self.listener.lookupTransform(self.root_frame, self.measured_frame, msg.header.stamp)
                print msg._type
                if msg._type == 'geometry_msgs/PoseStamped':
                    trans_gt.append(msg.pose.position.x)
                    trans_gt.append(msg.pose.position.y)
                    rot_gt = PyKDL.Rotation.Quaternion(msg.pose.orientation.x, msg.pose.orientation.y,
                                                       msg.pose.orientation.z, msg.pose.orientation.w)
                    rot_loc = PyKDL.Rotation.Quaternion(rot[0], rot[1], rot[2], rot[3])
                elif msg._type == 'nav_msgs/Odometry':
                    trans_gt.append(msg.pose.pose.position.x)
                    trans_gt.append(msg.pose.pose.position.y)
                    rot_gt = PyKDL.Rotation.Quaternion(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                                                       msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
                    rot_loc = PyKDL.Rotation.Quaternion(rot[0], rot[1], rot[2], rot[3])
                delta_trans = math.sqrt((trans_gt[0] - trans_loc[0])**2 + (trans_gt[1] - trans_loc[1])**2)
                self.delta_trans.append(delta_trans)
                delta_rot = rot_gt * rot_loc.Inverse()
                delta_yaw = abs(delta_rot.GetRPY()[2] * 180/math.pi)
                self.delta_rot.append(delta_yaw)
                rospy.loginfo("delta_yaw" + str(delta_yaw))
                self.count = self.count + 1
                if delta_trans > self.max_pos_error: #or delta_rot > self.max_ang_error:
                    if rospy.get_time() > self.fail_time + self.fail_timeout:
                        self.fails += 1
                        self.fail_time = rospy.get_time()
            except (tf.Exception, tf.LookupException, tf.ConnectivityException) as e:
                rospy.logwarn(e)

    def calcDelta(self, trans_loc, trans_gt, rot_loc, rot_gt):
        pass

    def record_tf(self, event):
        rospy.loginfo("callback")
        if (self.active):
            try:
                self.listener.waitForTransform(self.groundtruth_frame,
                                               self.measured_frame,
                                               rospy.Time(0),
                                               rospy.Duration.from_sec(1 /(2* self.tf_sampling_freq)))
                (trans, rot) = self.listener.lookupTransform(self.groundtruth_frame, self.measured_frame, rospy.Time(0))
                delta_trans = math.sqrt(trans[0]**2 + trans[1]**2)
                delta_quat = PyKDL.Rotation.Quaternion(rot[0], rot[1], rot[2], rot[3])
                delta_yaw = abs(delta_quat.GetRPY()[2] * 180/math.pi)
                rospy.loginfo(delta_trans)
                self.delta_trans.append(delta_trans)
                self.delta_rot.append(delta_yaw)
                rospy.loginfo("delta_yaw" + str(delta_yaw))
                self.count = self.count + 1
            except (tf.Exception, tf.LookupException, tf.ConnectivityException) as e:
                rospy.logwarn(e)

    def get_result(self):
        groundtruth_result = None
        data = {}
        details = {"root_frame": self.root_frame, "measured_frame": self.measured_frame, "topic": self.groundtruth_topic }
        if self.finished:
            #print "Distances: " + str(self.delta_trans) + " Dist Count: " + str(self.count)
            data["trans"] = self.delta_trans
            data["rot"] = self.delta_rot
            data["max_trans"] = max(data["trans"])
            data["max_rot"] = max(data["rot"])
            data["avg_trans"] = np.mean(data["trans"])
            data["avg_rot"] = np.mean(data["rot"])
            data["loc_fails"] = self.fails
            return "localization_distance", data, None, None, None, details
        else:
            return False
