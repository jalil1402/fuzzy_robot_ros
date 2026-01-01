#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import SpawnModel
from tf.transformations import euler_from_quaternion

class MamdaniZigZagController:
    def __init__(self):
        rospy.init_node('mamdani_zigzag_control')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        self.x, self.y, self.theta = 0.0, 0.0, 0.0
        self.start_time = None

        # --- ZIG-ZAG PARAMETERS ---
        # Waypoints: (x, y) starting from origin and going negative X
        self.waypoints = [
            (0.0, 0.0),
            (-1.0, 0.6),
            (-2.0, -0.6),
            (-3.0, 0.6),
            (-4.0, 0.0)
        ]
        self.speed = 0.2  # Speed of the virtual target (m/s)
        
        # Draw path in Gazebo before moving
        self.draw_path_in_gazebo()

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        rot_q = msg.pose.pose.orientation
        (_, _, self.theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    def draw_path_in_gazebo(self):
        """ Spawns yellow dots in Gazebo to show the zig-zag path """
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        spawn_srv = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        
        marker_sdf = """
        <sdf version='1.6'>
          <model name='marker'>
            <static>1</static>
            <link name='link'>
              <visual name='visual'>
                <geometry><cylinder><radius>0.04</radius><length>0.01</length></cylinder></geometry>
                <material><ambient>1 1 0 1</ambient><diffuse>1 1 0 1</diffuse></material>
              </visual>
            </link>
          </model>
        </sdf>"""

        rospy.loginfo("Visualizing Zig-Zag path in Gazebo...")
        # Create markers along the segments
        for i in range(len(self.waypoints) - 1):
            p1 = np.array(self.waypoints[i])
            p2 = np.array(self.waypoints[i+1])
            for j in range(10): # 10 dots per segment
                alpha = j / 10.0
                pos = p1 + alpha * (p2 - p1)
                p = Pose()
                p.position.x, p.position.y, p.position.z = pos[0], pos[1], 0.01
                spawn_srv(f"zz_dot_{i}_{j}", marker_sdf.replace("marker", f"zz_{i}_{j}"), "", p, "world")

    # --- MAMDANI FUZZY LOGIC ---
    def fuzzy_membership(self, x, a, b, c):
        if x <= a or x >= c: return 0.0
        if a < x <= b: return (x - a) / (b - a)
        if b < x < c: return (c - x) / (c - b)
        return 0.0

    def get_fuzzy_vel(self, dist_err, angle_err):
        # Linear sets
        near = self.fuzzy_membership(dist_err, -0.1, 0.0, 0.4)
        med  = self.fuzzy_membership(dist_err, 0.1, 0.5, 1.2)
        far  = self.fuzzy_membership(dist_err, 0.8, 2.0, 5.0)
        v = ((near * 0.0) + (med * 0.25) + (far * 0.45)) / (near + med + far + 1e-6)

        # Angular sets
        nb = self.fuzzy_membership(angle_err, -3.14, -1.5, -0.5)
        ns = self.fuzzy_membership(angle_err, -1.0, -0.3, 0.0)
        ze = self.fuzzy_membership(angle_err, -0.2, 0.0, 0.2)
        ps = self.fuzzy_membership(angle_err, 0.0, 0.3, 1.0)
        pb = self.fuzzy_membership(angle_err, 0.5, 1.5, 3.14)
        w = ((nb * -1.3) + (ns * -0.4) + (ze * 0.0) + (ps * 0.4) + (pb * 1.3)) / (nb + ns + ze + ps + pb + 1e-6)

        return v, w

    def run(self):
        rate = rospy.Rate(50)
        current_waypoint_idx = 0
        
        while not rospy.is_shutdown():
            if self.start_time is None:
                self.start_time = rospy.get_time()
                continue
            
            # 1. FIND CURRENT TARGET ON ZIG-ZAG
            # We target the next waypoint in the list
            tx, ty = self.waypoints[current_waypoint_idx + 1]
            
            dx = tx - self.x
            dy = ty - self.y
            dist_to_wp = np.sqrt(dx**2 + dy**2)

            # If we are close to the current target waypoint, switch to the next one
            if dist_to_wp < 0.2:
                if current_waypoint_idx < len(self.waypoints) - 2:
                    current_waypoint_idx += 1
                    rospy.loginfo(f"Heading to Waypoint {current_waypoint_idx + 1}")
                else:
                    # Final Waypoint reached
                    self.pub.publish(Twist())
                    rospy.loginfo("Zig-Zag Complete. Stopping.")
                    break

            # 2. CALCULATE ERRORS
            target_angle = np.arctan2(dy, dx)
            angle_error = target_angle - self.theta
            angle_error = np.arctan2(np.sin(angle_error), np.cos(angle_error))

            # 3. MAMDANI INFERENCE
            v_cmd, w_cmd = self.get_fuzzy_vel(dist_to_wp, angle_error)

            # 4. PUBLISH
            cmd = Twist()
            cmd.linear.x = np.clip(v_cmd, -0.4, 0.4)
            cmd.angular.z = np.clip(w_cmd, -1.2, 1.2)
            self.pub.publish(cmd)
            
            rate.sleep()

if __name__ == '__main__':
    try:
        MamdaniZigZagController().run()
    except rospy.ROSInterruptException:
        pass