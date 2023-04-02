#!/usr/bin/env python
import rospy
from turtlesim.msg import Pose
from math import pow,atan2,sqrt
import turtlesim.srv
import random
from geometry_msgs.msg import Twist

class TurtleBot_PID:

    def __init__(self):
        rospy.init_node('turtlebot_Random_RT', anonymous=True)
        # Publisher for velocity
        self.velocity_publisher = rospy.Publisher('/turtle_random_RT/cmd_vel',Twist, queue_size=10)

        # Publisher for Pose
        self.rt_real_pose = rospy.Publisher('/rt_real_pose',Pose, queue_size=10)

        #Subscriber to Pose
        self.pose_subscriber = rospy.Subscriber('/turtle_random_RT/pose',Pose, self.pose_CB)

        self.pose = Pose()
        self.rate = rospy.Rate(10)

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
        self.spawner(location_x, location_y, 0, 'turtle_random_RT') 

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

        self.Turtle_kill('turtle1')

        x = round(random.randrange(2,8),2)
        y = round(random.randrange(2,8),2)
        self.Turtle_spawn(x,y)
        
        goal_pose = Pose()

        # Get the input from the user.
        goal_pose.x = round(random.randrange(2,4),2)
        goal_pose.y = round(random.randrange(2,4),2)

        vel_msg = Twist()
        now = rospy.get_time()

        
        while(True):
            goal_pose.x = round(random.randrange(2,4),2)
            goal_pose.y = round(random.randrange(2,4),2)

            # Linear velocity in the x-axis.
            vel_msg.linear.x = self.PID_update_linear(self.linear_velocity(goal_pose))
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.PID_update_angular(self.angular_velocity(goal_pose))

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)

            # Publish at the desired rate.
            self.rate.sleep()

            after = rospy.get_time()
            if (after - now > 5):
                self.rt_real_pose.publish(self.pose)
                now = rospy.get_time()

            if(self.linear_velocity(goal_pose) < 0.1):

                # Stopping our robot after the movement is over.
                vel_msg.linear.x = 0
                vel_msg.angular.z = 0
                self.velocity_publisher.publish(vel_msg)


        # If we press control + C, the node will stop.
        rospy.spin()

if __name__ == '__main__':
    try:
        x = TurtleBot_PID()
        x.move2goal()
    except rospy.ROSInterruptException:
        pass