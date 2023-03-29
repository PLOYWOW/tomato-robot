#!/usr/bin/env python


import roslib
import rospy
from trac_ik_python.trac_ik import IK
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

print("ok")


    
class IKCalculator():
    def __init__(self):
        self.i = 0
        self.currentJoints = []
        self.previousJoints = []
        self.endPose = [0 for i in range(7)]

    def poseCallback(self,msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        w = msg.pose.orientation.w
        self.endPose = [x,y,z,qx,qy,qz,w]
        return 1

    def handleIK(self,msg):
        print("msgere")
        x = self.endPose[0]
        y = self.endPose[1]
        z = self.endPose[2]
        qx = self.endPose[3]
        qy = self.endPose[4]
        qz = self.endPose[5]
        w = self.endPose[6]

        jointList = JointState()
        jointList.header = Header()
        jointList.header.stamp = rospy.Time.now()
        jointList.name = ['joint_s', 'joint_l', 'joint_e', 'joint_u', 'joint_r', 'joint_b', 'joint_t', 'twintool_joint1']

        print("iking")
        ik_solver = IK("base_link",
                    "tool1_center_point",
                    solve_type="Manipulation2")

        print(ik_solver.joint_names)

        seed_state = [0.0] * ik_solver.number_of_joints

        updateJoints = ik_solver.get_ik(seed_state,
                            x, y, z,  # X, Y, Z
                            qx, qy, qz,w)  # QX, QY, QZ, QW

        
        if updateJoints != None:
            self.currentJoints = list(updateJoints)
            print(self.currentJoints)
            jointDiff = [abs(a_i - b_i) for a_i, b_i in zip(self.currentJoints, self.previousJoints)]
            jointDistance = sum(jointDiff)
            print(jointDistance)

            if jointDistance < 10.0:
                jointList.position = self.currentJoints
                jointList.velocity = []
                jointList.effort = []
                self.joints_publisher.publish(jointList)
                self.previousJoints = self.currentJoints
            else:
                jointList.position = self.previousJoints
                jointList.velocity = []
                jointList.effort = []
                self.joints_publisher.publish(jointList)

            
    def process(self):
        rospy.init_node('ikNode')

        self.joints_publisher = rospy.Publisher('virtual/joint_states', JointState, queue_size=10)
        rospy.Subscriber('grasp_pose',
                        PoseStamped,
                        self.poseCallback)
        rate = rospy.Rate(60) # 10hz
        print("55")
        while not rospy.is_shutdown():
            self.handleIK(self.endPose)
            rate.sleep()
        
if __name__ == '__main__':
    try:
        ikHandler = IKCalculator()
        ikHandler.process()

    except rospy.ROSInterruptException:
        pass
