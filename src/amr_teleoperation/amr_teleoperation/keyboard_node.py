import rclpy
from rclpy.node import Node
from amr_msgs.msg import Key
from sshkeyboard import listen_keyboard

class MinimalPublisher(Node):
    """
    Nodo publicador simple que envía un mensaje 'Hello, World!' cada 500ms.
    """
    def __init__(self) -> None:
        # Inicializa la clase base Node con el nombre del nodo
        super().__init__("minimal_publisher")
        
        # Configuración del Publisher
        self._publisher = self.create_publisher(
            msg_type=Key, 
            topic="keyboard", 
            qos_profile=10
        )
        
       
        listen_keyboard(on_press = self._listen_keyboard_callback,
                        sequential=True)

    def _listen_keyboard_callback(self, key) -> None:
        """
        Función que se ejecuta periódicamente para publicar el mensaje.
        """
        msg = Key()
        msg.key_input = key
        
        # Publica el mensaje en el tópico
        self._publisher.publish(msg)
        
        # Muestra el mensaje en la terminal del contenedor
        self.get_logger().info(f"Publishing: {msg.key_input}")

def main(args=None):
    rclpy.init(args=args)
    node = MinimalPublisher()
    
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()