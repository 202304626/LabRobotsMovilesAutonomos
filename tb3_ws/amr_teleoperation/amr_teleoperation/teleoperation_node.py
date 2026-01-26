import math
import rclpy
from rclpy.node import Node
import numpy as np
from amr_msgs.msg import KeyPressed
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
 
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

MAX_ANGULAR_VEL = 2.81
MAX_LINEAR_VEL = 0.22
OBSTACLE_THRESHOLD = 0.15

qos_lidar_profile = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
)
 
qos_cmdvel_profile = QoSProfile(
    history=QoSHistoryPolicy.UNKNOWN,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
)

class NodeMapper(Node):
    """
    Nodo mínimo que publica mensajes cada 500ms.

    Hereda de la clase base Node, que proporciona toda
    la funcionalidad de ROS (logging, timers, publishers, etc.)
    """

    def __init__(self) -> None:
        # Llamar al constructor de la clase padre
        # "minimal_publisher" es el nombre del nodo
        super().__init__("node_mapper")
        self.map = {
            "w": (1.0, 0.0),
            "s": (-1.0, 0.0),
            "a": (0.0, 1.0),
            "d": (0.0, -1.0),
            "space": (0.0, 0.0),
        }

        self.vel_lin = 0.0
        self.vel_ang = 0.0

        # Crear un publicador
        # msg_type: Tipo de mensaje que publicará
        # topic: Nombre del topic
        # qos_profile: Calidad de servicio (10 = mantener últimos 10 mensajes)
        self._publisher = self.create_publisher(msg_type=Twist, topic="cmd_vel", qos_profile=10)
        self._subscriber = self.create_subscription(
            msg_type=KeyPressed,
            topic="listen_key",
            callback=self.key_callback,  # Función que procesa mensajes
            qos_profile=10,
        )

        self.teleoperationLiDARSubs = self.create_subscription(  # LiDAR msg_type and topic and qos_profile
                msg_type=LaserScan,
                topic="scan",
                callback=self.AnalyzeLiDAR,
                qos_profile=qos_lidar_profile,
            )
        
    def AnalyzeLiDAR(self, msg):
        # --- 1. Calculate Indices ---
        # Front index (Angle 0.0)
        index_front = int((0.0 - msg.angle_min) / msg.angle_increment)
        
        # Back index (Angle PI, or 180 degrees)
        index_back = int((math.pi - msg.angle_min) / msg.angle_increment)

        # Protección: Si el cálculo se sale del array, usamos el último elemento disponible
        if index_back >= len(msg.ranges):
            index_back = len(msg.ranges) - 1

        # --- 2. Get Distances ---
        dist_front = msg.ranges[index_front]
        dist_back = msg.ranges[index_back]

        # --- 3. Verify validity of the data ---
        # We check if the readings are valid numbers (not infinite) and within range
        valid_front = math.isfinite(dist_front) and msg.range_min <= dist_front <= msg.range_max
        valid_back = math.isfinite(dist_back) and msg.range_min <= dist_back <= msg.range_max

        # Assign values for visualization or external logic
        self.distance_front = dist_front if valid_front else float("inf")
        self.distance_back = dist_back if valid_back else float("inf")

        # --- 4. STOP LOGIC ---
        # Condition: (Obstacle in front) OR (Obstacle behind)
        danger_front = valid_front and (dist_front < OBSTACLE_THRESHOLD)
        danger_back = valid_back and (dist_back < OBSTACLE_THRESHOLD)

        if danger_front or danger_back:
            if danger_front:
                print(f"DANGER IN FRONT! Distance: {dist_front:.2f}m")
            if danger_back:
                print(f"DANGER BEHIND! Distance: {dist_back:.2f}m")

            # Stop the robot
            stop_msg = Twist()
            stop_msg.linear.x = 0.0
            stop_msg.angular.z = 0.0
            self._publisher.publish(stop_msg)



    def key_callback(self, msg: KeyPressed) -> None:
        """
        Esta función se ejecuta automáticamente cada vez que
        llega un mensaje al topic "hello".

        Args:
            msg: El mensaje recibido del tipo String
        """
        self.process_key(msg.key)

    def process_key(self, key):
        if key in self.map:
            if key == "space":
                self.vel_lin = 0
                self.vel_ang = 0

                self.get_logger().info(f"Stopping...")

            
            else:
                lin, ang = self.map.get(key)

                self.vel_lin += lin * 0.05
                self.vel_ang += ang * 0.3
            
                if abs(self.vel_ang) >= MAX_ANGULAR_VEL:
                    self.get_logger().info(f"Max angular vel reached")

                elif abs(self.vel_lin) >= MAX_LINEAR_VEL:
                    self.get_logger().info(f"Max linear vel reached")

                else:
                    t = Twist()
                    t.linear.x = self.vel_lin
                    t.angular.z = self.vel_ang
                    self.get_logger().info(f"Publishing key: {key}")
                    self._publisher.publish(t)

        else:
            self.get_logger().info(f"Not a valid key: {key}")


def main(args=None) -> None:
    """
    Función principal que se ejecuta al lanzar el nodo.
    """
    # Inicializar ROS 2
    # SIEMPRE debe ser la primera llamada
    rclpy.init(args=args)

    # Crear instancia del nodo
    mapping_node = NodeMapper()

    # spin() mantiene el nodo vivo y procesando callbacks
    # Se bloquea aquí hasta que se interrumpa (Ctrl+C)
    rclpy.spin(mapping_node)

    # Limpieza al salir
    # destroy_node() es opcional: el garbage collector lo haría
    # pero es buena práctica hacerlo explícitamente
    mapping_node.destroy_node()

    # Apagar ROS 2
    rclpy.shutdown()


# Punto de entrada cuando se ejecuta el script directamente
if __name__ == "__main__":
    main()
