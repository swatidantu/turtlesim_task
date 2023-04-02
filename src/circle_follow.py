#!/usr/bin/env python
import rospy
from turtlesim.msg import Pose
from math import pow,atan2,sqrt
import turtlesim.srv
import random
from geometry_msgs.msg import Twist

class TurtleBot_PT:

    def __init__(self):
        rospy.init_node('turtlebot_follow', anonymous=True)
        # Publisher for velocity
        self.velocity_publisher_pt = rospy.Publisher('/turtle_PT/cmd_vel',Twist, queue_size=10)

        #Subscriber to Pose
        self.pose_pt_subscriber = rospy.Subscriber('/turtle_PT/pose',Pose, self.pose_CB)
        self.pose_rt_subscriber = rospy.Subscriber('/rt_real_pose',Pose, self.pose_CB_rt)

        self.pose = Pose()
        self.pose_rt = Pose()
        self.rate = rospy.Rate(20)

        # Linear Velocity Parameters
        self.derivative_lin = 0
        self.integrator_lin = 0
        self.kp_lin = 1.5
        self.kd_lin = 0
        self.ki_lin = 0


        # Angular Velocity Parameters
        self.derivative_ang = 0
        self.integrator_ang = 0
        self.kp_ang = 6
        self.kd_ang = 0
        self.ki_ang = 0


    def pose_CB_rt(self, data):

        self.pose_rt = data
        self.pose_rt.x = round(self.pose_rt.x, 4)
        self.pose_rt.y = round(self.pose_rt.y, 4)

    def pose_CB(self, data):

        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

    def euclidean_distance(self, goal_pose):
        return sqrt(pow((goal_pose.x - self.pose.x), 2) +pow((goal_pose.y - self.pose.y), 2))

    def linear_velocity(self, goal_pose):
        return self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_velocity(self, goal_pose):
        return (self.steering_angle(goal_pose) - self.pose.theta)


    #Spawn function
    def Turtle_spawn(self,location_x,location_y): 
        self.spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
        self.spawner(location_x, location_y, 0, 'turtle_PT') 

    def Turtle_kill(self,name): 
        self.killer=rospy.ServiceProxy('kill',turtlesim.srv.Kill)
        self.killer(name)

    def PID_update_linear(self,error):

        P = self.kp_lin * error
        D = self.kd_lin * (error - self.derivative_lin)
        self.derivative_lin = error
        self.integrator_lin = self.integrator_lin + error
        I = self.ki_lin * self.integrator_lin
        PID = P + I + D 
        return PID


    def PID_update_angular(self,error):

        P = self.kp_ang * error
        D = self.kd_ang* (error - self.derivative_ang)
        self.derivative_ang = error
        self.integrator_ang = self.integrator_ang + error
        I = self.ki_ang * self.integrator_ang
        PID = P + I + D 
        return PID



    def move2goal(self):

        x = round(random.randrange(2,8),2)
        y = round(random.randrange(2,8),2)
        self.Turtle_spawn(x,y)

        goal_pose = self.pose_rt
        vel_msg_pt = Twist()

        while(True):
            goal_pose = self.pose_rt

            vel_msg_pt.linear.x = self.PID_update_linear(self.linear_velocity(goal_pose))
            vel_msg_pt.linear.y = 0
            vel_msg_pt.linear.z = 0

            # Angular velocity in the z-axis.
            vel_msg_pt.angular.x = 0
            vel_msg_pt.angular.y = 0
            vel_msg_pt.angular.z = self.PID_update_angular(self.angular_velocity(goal_pose))

            # Publishing our vel_msg
            self.velocity_publisher_pt.publish(vel_msg_pt)

            # Publish at the desired rate.
            self.rate.sleep()

            # If we press control + C, the node will stop.
        rospy.spin()



if __name__ == '__main__':
    try:


        x = TurtleBot_PT()
        x.move2goal()
    except rospy.ROSInterruptException:
        pass



