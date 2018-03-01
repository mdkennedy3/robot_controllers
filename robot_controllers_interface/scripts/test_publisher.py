#!/usr/bin/env python


# Autho: Dinesh Thakur

import sys
import rospy

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


if __name__ == "__main__":

  if len(sys.argv) < 2:
    print("usage: test_publisher.py <arm/base>")
    exit(-1)

  rospy.init_node("test_publisher")

  name = sys.argv[1]

  if not(name == 'arm' or name == 'base'):
    print("usage: test_publisher.py <arm/base>")
    exit(-1)

  jt = JointTrajectory()
  jt.header.stamp = rospy.Time.now()

  topic_name = '/base_controller_torque/command'

  if name == 'base':
    topic_name = '/base_controller_torque/command'

    jt.joint_names = ['l_wheel_joint', 'r_wheel_joint']
    tp = JointTrajectoryPoint()
    tp.effort = 0.1

    for i in range(len(jt.joint_names)):
      jt.points.append(tp)

  if name == 'arm':
    topic_name = '/arm_controller/torque_control_arm/command'

    #jt.joint_names = [ 'shoulder_pan_joint', 'shoulder_lift_joint', 'upperarm_roll_joint', 'elbow_flex_joint',
    #'forearm_roll_joint', 'wrist_flex_joint', 'wrist_roll_joint']

    #jt.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_flex_joint', 'wrist_flex_joint']
    jt.joint_names = ['forearm_roll_joint', 'wrist_flex_joint', 'wrist_roll_joint']
    #jt.joint_names = ['wrist_roll_joint']


    tp = JointTrajectoryPoint()
    #tp.effort = 0.1

    for i in range(len(jt.joint_names)):
      tp.effort.append(1.0)

    jt.points.append(tp)

  pub = rospy.Publisher(topic_name, JointTrajectory, queue_size=10)

  print jt

  r = rospy.Rate(50) # 10hz
  while not rospy.is_shutdown():
    pub.publish(jt)
    r.sleep()



