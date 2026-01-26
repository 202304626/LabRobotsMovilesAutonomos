import rclpy
from rclpy.node import Node
from more_interfaces.msg import KeyTouched
from geometry_msgs.msg import Twist


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
            "space": (0.0, 0.0)
        }

        self.vel_lin = 0.0
        self.vel_ang = 0.0

        # Crear un publicador
        # msg_type: Tipo de mensaje que publicará
        # topic: Nombre del topic
        # qos_profile: Calidad de servicio (10 = mantener últimos 10 mensajes)
        self._publisher = self.create_publisher(msg_type=Twist, topic="cmd_vel", qos_profile=10)
        self._subscriber = self.create_subscription(
            msg_type=KeyTouched,
            topic="listen_key",
            callback=self.key_callback,  # Función que procesa mensajes
            qos_profile=10,
        )

    def key_callback(self, msg: KeyTouched) -> None:
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
            lin, ang = self.map.get(key)
            self.vel_lin += lin*0.1
            self.vel_ang += ang*0.1

            t = Twist()
            t.linear.x = self.vel_lin
            t.angular.z = self.vel_ang
            self.get_logger().info(f"Publishing {key}...")
            self._publisher.publish(t) 
        else:
            self.get_logger().info(f"Not a valid key, {key}")

    def publish_twist(self, t: Twist):
        self._publisher.publish(t)

    


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
