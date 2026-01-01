#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import SpawnModel
from tf.transformations import euler_from_quaternion

class MamdaniCircleController:
    def __init__(self):
        rospy.init_node('mamdani_circle_control')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        self.x, self.y, self.theta = 0.0, 0.0, 0.0
        self.start_time = None

        # --- CIRCLE PARAMETERS ---
        # Radius = 1.4m (approx 7x robot size 0.4m)
        self.R = 1.4 
        self.v_target = 0.25 # Speed m/s
        self.omega_path = self.v_target / self.R
        self.duration = (2 * np.pi) / self.omega_path # Time for 1 full circle

        # Draw the circle markers in Gazebo
        self.draw_circle_in_gazebo()

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        rot_q = msg.pose.pose.orientation
        (_, _, self.theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    def draw_circle_in_gazebo(self):
        """ Spawns small markers to visualize the path """
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        spawn_srv = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        
        # Simple SDF for a small flat marker
        marker_sdf = """
        <sdf version='1.6'>
          <model name='marker'>
            <static>1</static>
            <link name='link'>
              <visual name='visual'>
                <geometry><cylinder><radius>0.05</radius><length>0.01</length></cylinder></geometry>
                <material><ambient>1 1 0 1</ambient><diffuse>1 1 0 1</diffuse></material>
              </visual>
            </link>
          </model>
        </sdf>"""

        rospy.loginfo("Drawing circle markers in Gazebo...")
        for i in range(24):
            angle = (i / 24.0) * 2 * np.pi
            # Same equation as trajectory
            mx = -self.R * np.sin(angle)
            my = self.R - self.R * np.cos(angle)
            
            pose = Twist() # Using Twist as a dummy container for coords
            model_xml = marker_sdf.replace("marker", f"dot_{i}")
            
            from geometry_msgs.msg import Pose
            p = Pose()
            p.position.x, p.position.y, p.position.z = mx, my, 0.01
            spawn_srv(f"dot_{i}", model_xml, "", p, "world")

    # --- FUZZY LOGIC (Mamdani) ---
    def fuzzy_membership(self, x, a, b, c):
        if x <= a or x >= c: return 0.0
        if a < x <= b: return (x - a) / (b - a)
        if b < x < c: return (c - x) / (c - b)
        return 0.0

    def get_velocities(self, dist_err, angle_err):
        # Linear Logic (Near, Med, Far)
        n = self.fuzzy_membership(dist_err, -0.1, 0.0, 0.3)
        m = self.fuzzy_membership(dist_err, 0.1, 0.4, 1.0)
        f = self.fuzzy_membership(dist_err, 0.6, 1.5, 5.0)
        v = ((n * 0.0) + (m * 0.5) + (f * 1.0)) / (n + m + f + 1e-6)

        # Angular Logic (NB, NS, ZE, PS, PB)
        nb = self.fuzzy_membership(angle_err, -3.14, -1.5, -0.5)
        ns = self.fuzzy_membership(angle_err, -1.0, -0.3, 0.0)
        ze = self.fuzzy_membership(angle_err, -0.2, 0.0, 0.2)
        ps = self.fuzzy_membership(angle_err, 0.0, 0.3, 1.0)
        pb = self.fuzzy_membership(angle_err, 0.5, 1.5, 3.14)
        w = ((nb * -1.7) + (ns * -0.9) + (ze * 0.0) + (ps * 0.9) + (pb * 1.7)) / (nb + ns + ze + ps + pb + 1e-6)

        return v, w

    def run(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.start_time is None:
                self.start_time = rospy.get_time()
                continue
            
            t = rospy.get_time() - self.start_time

            # 1. GENERATE REFERENCE
            if t < self.duration:
                # Target moves along the circle
                xr = -self.R * np.sin(self.omega_path * t)
                yr = self.R - self.R * np.cos(self.omega_path * t)
                # Tangent angle for negative X start:
                thetar = np.pi + (self.omega_path * t) 
            else:
                # Stop at origin
                xr, yr, thetar = 0.0, 0.0, 0.0

            # 2. CALCULATE ERRORS
            dx, dy = xr - self.x, yr - self.y
            dist_error = np.sqrt(dx**2 + dy**2)
            
            target_angle = np.arctan2(dy, dx)
            # If very close to target, use the path orientation instead
            if dist_error < 0.1: target_angle = thetar

            angle_error = target_angle - self.theta
            angle_error = np.arctan2(np.sin(angle_error), np.cos(angle_error))

            # 3. MAMDANI INFERENCE
            v_cmd, w_cmd = self.get_velocities(dist_error, angle_error)

            # 4. STOP CONDITION
            if t > self.duration and dist_error < 0.05:
                v_cmd, w_cmd = 0.0, 0.0
                rospy.loginfo("Circle Complete. Robot Stopped.")
                self.pub.publish(Twist())
                break

            # 5. PUBLISH
            cmd = Twist()
            cmd.linear.x = np.clip(v_cmd, -0.4, 0.4)
            cmd.angular.z = np.clip(w_cmd, -1.2, 1.2)
            self.pub.publish(cmd)
            rate.sleep()

if __name__ == '__main__':
    try:
        MamdaniCircleController().run()
    except rospy.ROSInterruptException:
        pass