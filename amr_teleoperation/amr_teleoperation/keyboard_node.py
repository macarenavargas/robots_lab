import rclpy
from rclpy.node import Node
from amr_msgs.msg import KeyboardMessage
from sshkeyboard import listen_keyboard

class MinimalPublisher(Node):
    """
    Nodo publicador simple que envía un mensaje 'Hello, World!' cada 500ms.
    """
    def __init__(self) -> None:
        # Inicializa la clase base Node con el nombre del nodo
        super().__init__("minimal_publisher")
        
        # Configuración del Publisher
        # msg_type: Tipo de mensaje (String)
        # topic: Nombre del tópico ("hello")
        # qos_profile: Tamaño de la cola (10)
        self._publisher = self.create_publisher(
            msg_type=KeyboardMessage, 
            topic="hello", 
            qos_profile=10
        )
        
        # Temporizador para ejecutar el callback cada 0.5 segundos
        self._timer = self.create_timer(
            timer_period_sec=0.5, 
            callback=self._timer_callback
        )

        self._keyboard = listen_keyboard(
            on_press=press,
            on_release=release,
        )

    def _keyboard_callback(self, key) -> None:
        """
        Función que se ejecuta periódicamente para publicar el mensaje.
        """
        msg = KeyboardMessage()
        msg.keyboard_input = key
        
        # Publica el mensaje en el tópico
        self._publisher.publish(msg)
        
        # Muestra el mensaje en la terminal del contenedor
        self.get_logger().info(f"Publishing: {msg.keyboard_input}")

def main(args=None):
    rclpy.init(args=args)
    node = MinimalPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()