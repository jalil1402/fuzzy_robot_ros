#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import SpawnModel
from tf.transformations import euler_from_quaternion

class PointToPointController:
    def __init__(self):
        rospy.init_node('kinematic_point_node')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        # Controller Constants (gamma, a, b from your Equations)
        self.gamma = 0.7
        self.a = -0.4
        self.b = -0.8
        
        # Robot State
        self.x, self.y, self.theta = 0.0, 0.0, 0.0
        
        # --- WAYPOINTS ---
        # Generate points along the path: y = sin(0.5x) + 0.5x
        self.waypoints = []
        for i in range(1, 11):
            wx = -float(i)
            wy = np.sin(0.5 * wx) + 0.5 * wx
            self.waypoints.append((wx, wy))
            
        self.current_wp_idx = 0
        self.arrival_threshold = 0.1 # meters
        
        # Draw the points correctly
        self.draw_waypoints()

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        (_, _, self.theta) = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

    def draw_waypoints(self):
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        spawn_srv = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        
        rospy.loginfo("Spawning Waypoint Markers...")
        for i, (wx, wy) in enumerate(self.waypoints):
            # FIXED: Use a unique placeholder "MODEL_NAME" instead of just "m"
            marker_sdf = f"""
            <sdf version='1.6'>
              <model name='wp_{i}'>
                <static>1</static>
                <link name='link'>
                  <visual name='visual'>
                    <geometry><sphere><radius>0.1</radius></sphere></geometry>
                    <material>
                      <ambient>1 0 0 1</ambient>
                      <diffuse>1 0 0 1</diffuse>
                    </material>
                  </visual>
                </link>
              </model>
            </sdf>"""
            
            p = Pose()
            p.position.x, p.position.y, p.position.z = wx, wy, 0.05
            # Service call: (model_name, model_xml, robot_namespace, initial_pose, reference_frame)
            spawn_srv(f"wp_{i}", marker_sdf, "", p, "world")

    def run(self):
        rate = rospy.Rate(50)
        rospy.loginfo("Point-to-Point Control Started. Heading for first target.")

        while not rospy.is_shutdown():
            if self.current_wp_idx >= len(self.waypoints):
                rospy.loginfo("--- ALL WAYPOINTS REACHED! ---")
                self.pub.publish(Twist()) # Stop robot
                break

            # 1. Get current target point
            tx, ty = self.waypoints[self.current_wp_idx]
            
            # 2. Calculate errors to the target point
            dx = tx - self.x
            dy = ty - self.y
            dist = np.sqrt(dx**2 + dy**2)
            
            # Heading needed to reach the point
            thetar = np.arctan2(dy, dx)
            
            # Arrived check
            if dist < self.arrival_threshold:
                rospy.loginfo(f"DONE: Point {self.current_wp_idx} at ({tx}, {ty})")
                self.current_wp_idx += 1
                continue

            # 3. Kinematic Control (Equation 7)
            # Targets are static: v_d = 0, omega_d = 0
            e1 = self.x - tx
            e2 = self.y - ty
            e3 = self.theta - thetar
            e3 = np.arctan2(np.sin(e3), np.cos(e3)) # Normalize orientation error

            cos_theta = np.cos(self.theta)
            # Prevent division by zero and handle Gazebo direction
            if abs(cos_theta) < 0.1:
                v_c = 0.1 # Move slow to turn
            else:
                # Based on Eq 7: v_c = (-gamma * e1) / cos(theta)
                v_c = (self.gamma * e1) / cos_theta

            # omega_c = -a*e2 - b*e3
            omega_c = self.a * e2 + self.b * e3

            # 4. Limit and Publish
            cmd = Twist()
            # Direction check: If robot moves away, add a minus sign to v_c
            cmd.linear.x = np.clip(v_c, -0.4, 0.4) 
            cmd.angular.z = np.clip(omega_c, -1.2, 1.2)
            self.pub.publish(cmd)

            if int(rospy.get_time()) % 2 == 0:
                rospy.loginfo(f"Target: {self.current_wp_idx} | Dist: {dist:.2f} | V: {v_c:.2f}")

            rate.sleep()

if __name__ == '__main__':
    try:
        PointToPointController().run()
    except rospy.ROSInterruptException:
        pass