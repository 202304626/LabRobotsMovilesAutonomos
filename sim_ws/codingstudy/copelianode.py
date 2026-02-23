from rclpy.qos import (
QoSDurabilityPolicy ,
QoSHistoryPolicy ,
QoSProfile ,
QoSReliabilityPolicy
)
from amr_simulation.coppeliasim import CoppeliaSim
import message_filters

qos_profile = QoSProfile(
history=QoSHistoryPolicy. KEEP_LAST , # KEEP_LAST, KEEP_ALL
depth=10, # Solo se tiene en cuenta si history=QoSHistoryPolicy.KEEP_LAST
reliability=QoSReliabilityPolicy.BEST_EFFORT , # BEST_EFFORT, RELIABLE
durability= QoSDurabilityPolicy. VOLATILE , # TRANSIENT_LOCAL, VOLATILE
)

from amr_msgs.msg import PoseStamped
from geometry_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan

class CoppeliaSimNode(LifecycleNode):
    def __init__(self):
        super().__init__("coppeliasim")
        self.declare_parameter("dt",0.05)
        self.declare_parameter("enable_localization", False)
        self.declare_parameter("goal",(float("inf"),float("inf")))

    def on_configure(self, state):
        try:
            dt = self.get_parameter("dt").get_parameter_value().double_value
            enable_location = self.get_parameter("enable_location").get_parameter_value
            self._coppeliasim = CoppeliaSim(dt, start, pose_tolerance)

            self._odometry_publisher = self.create_publisher(msg_type = Odometry, topic="odometry", qos_profile=10)
            self._laserScan_publisher = self.create_publisher(msg_type = LaserScan, topic="scan", qos_profile= qos_profile)
            if not enable_location:
                self.cmd_vel_suscriber = self.create_suscription(TwistStamped, "/cmd_vel", self._next_step_callback,10)
            else:
                self._sucribrers = []
                self._sucribrers.append(message_filters.Suscriber(self, msg_type=TwistStamped, topic = "/cmd_vel", qos_profile=10))
                self._sucribrers.append(message_filters.Suscriber(self, msg_type = PoseStamped, topic="pose", qos_profile=10))

                ts = message_filters.ApproximateTimeSynchronizer(fs=self._sucribrers, queque_size = 10, slop=0.05 ) # tiempo en el q consideramos q estan sincronizados 

                ts.registerCallback(self._next_step_callback)

            
        except Exception:
            self.get_logger().error(f"{traceback.format_exc()}")
            return TransitionCallbackReturn.ERROR
        
        return super().on_configure(state)
    

    def on_activate(self, state):
        self.get_logger().info(f"Transitioning from '{state.label}' to 'active' state.")

        try:
            # Initial method calls
            self._next_step_callback(cmd_vel_msg=TwistStamped())

        except Exception:
            self.get_logger().error(f"{traceback.format_exc()}")
            return TransitionCallbackReturn.ERROR

        return super().on_activate(state)
    def _next_step_callback(self, cmd_vel_msg: TwistStamped, pose_msg: PoseStamped = PoseStamped()):
        """Subscriber callback. Executes a simulation step and publishes the new measurements.

        Args:
            cmd_vel_msg: Message containing linear (v) and angular (w) speed commands.
            pose_msg: Message containing the estimated robot pose.

        """
        # Check estimated pose
        self._check_estimated_pose(pose_msg)

        # TODO: 2.13. Parse the velocities from the TwistStamped message (i.e., read v and w).
        v = cmd_vel_msg.twist.linear.x 
        w = cmd_vel_msg.twist.angular.z
        self.robot.move(v,w)
        self._coppeliasim.next_step()
        z_scan, z_v, z_w = self._robot.sense()  
        self._publish_odometry(z_v,z_w)
        self._publish_scan(z_scan)

    def _check_estimated_pose(self, pose_msg):
        self._localized = pose_msg.localized
        if self._localized:
            x_h = pose_msg.pose.position.x 



    def move(self,v,w):
        b = self._track /2
        left_wheel_vel = (v - b*w)/self._r
        right_wheel_vel = (v+ b*w)/self._r 

    def sense_enconders()
        
