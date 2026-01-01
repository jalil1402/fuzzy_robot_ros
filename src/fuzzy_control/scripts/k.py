#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import SpawnModel
from tf.transformations import euler_from_quaternion

class KinematicPathFollowing:
    def __init__(self):
        rospy.init_node('kinematic_path_node')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        # --- YOUR COEFFICIENTS ---
        self.gamma = 0.7
        self.a = 0.4
        self.b = 0.8
        
        # Robot State
        self.x, self.y, self.theta = 0.0, 0.0, 0.0
        
        # Path Settings
        self.v_path = 0.25      # Speed we want to maintain on the path (m/s)
        self.lookahead = 0.3    # How far ahead on the curve to look (meters)
        
        # Draw the curve in Gazebo
        self.draw_path_markers()

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        (_, _, self.theta) = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

    def get_path_info(self, x_query):
        """ Calculates the y, slope, and curvature at a specific x on the path """
        # y = sin(0.5x) + 0.5x
        y_d = np.sin(0.5 * x_query) + 0.5 * x_query
        
        # First derivative (slope) dy/dx
        dy_dx = 0.5 * np.cos(0.5 * x_query) + 0.5
        theta_d = np.arctan2(dy_dx, 1.0)
        
        # Second derivative d2y/dx2
        d2y_dx2 = -0.25 * np.sin(0.5 * x_query)
        
        # Curvature (k) = |y''| / (1 + y'^2)^(3/2)
        curvature = d2y_dx2 / (1 + dy_dx**2)**(1.5)
        
        # Desired angular velocity = speed * curvature
        omega_d = self.v_path * curvature
        
        return y_d, theta_d, omega_d

    def draw_path_markers(self):
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        spawn_srv = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        rospy.loginfo("Visualizing Path...")
        
        # Draw dots for 10 meters
        for i in range(50):
            px = i * 0.2
            py = np.sin(0.5 * px) + 0.5 * px
            marker_xml = f"""
            <sdf version='1.6'>
              <model name='p_{i}'>
                <static>1</static>
                <link name='link'>
                  <visual name='v'>
                    <geometry><sphere><radius>0.04</radius></sphere></geometry>
                    <material><ambient>0 0 1 1</ambient><diffuse>0 0 1 1</diffuse></material>
                  </visual>
                </link>
              </model>
            </sdf>"""
            p = Pose()
            p.position.x, p.position.y, p.position.z = px, py, 0.01
            spawn_srv(f"p_{i}", marker_xml, "", p, "world")

    def run(self):
        rate = rospy.Rate(50)
        rospy.loginfo("Time-Independent Path Following Started.")

        while not rospy.is_shutdown():
            # 1. Project a target point ahead of the robot's current X position
            x_d = -self.x - self.lookahead
            y_d, theta_d, omega_d = self.get_path_info(x_d)
            v_d = self.v_path

            # 2. Calculate errors based on the projected target
            e1 = self.x - x_d
            e2 = self.y - y_d
            e3 = self.theta - theta_d
            e3 = np.arctan2(np.sin(e3), np.cos(e3))

            # 3. Apply your Kinematic Law (Equation 7)
            cos_theta = np.cos(self.theta)
            
            # v_c = (v_d*cos(theta_d) - gamma*e1) / cos(theta)
            if abs(cos_theta) < 0.1:
                v_c = 0.1 # Safety crawl if sideways
            else:
                v_c = (v_d * np.cos(theta_d) - self.gamma * e1) / cos_theta

            # omega_c = omega_d - a*e2 - b*e3
            omega_c = omega_d - self.a * e2 - self.b * e3

            # 4. Limit and Publish
            cmd = Twist()
            # If the robot moves away from the Blue path, use: -v_c
            cmd.linear.x = np.clip(v_c, -0.5, 0.5)
            cmd.angular.z = np.clip(omega_c, -1.2, 1.2)
            self.pub.publish(cmd)

            if int(rospy.get_time()*10) % 20 == 0:
                rospy.loginfo(f"Pos: ({self.x:.2f}, {self.y:.2f}) | Lat Error: {e2:.2f}")

            rate.sleep()

if __name__ == '__main__':
    try:
        KinematicPathFollowing().run()
    except rospy.ROSInterruptException:
        pass