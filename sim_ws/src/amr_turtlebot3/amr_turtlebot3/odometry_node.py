import rclpy
from rclpy.node import Node
from rclpy.time import Time
from transforms3d.euler import quat2euler
import math
# Message types
from nav_msgs.msg import Odometry

class OdometryNode(Node):  # Nodes inherit from the base class Node
    def __init__(self) -> None:
        super().__init__("minimal_subscriber")  # Set the node name
        self.prev_time = None
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.prev_theta = 0.0
        # Subscribers (beware self._subscriptionsis reserved)
        self._subscriber= self.create_subscription(
        msg_type= Odometry,
        topic="/odom",
        callback=self._estimate_velocities_callback,
        qos_profile=10,
                )

        self._publisher= self.create_publisher(
			msg_type=Odometry, topic="/odometry", qos_profile=10
		)
        
        
    def _estimate_velocities_callback(self, msg: Odometry) -> None:
        # Log to the terminal with information (info) level
        
        current_time = Time.from_msg(msg.header.stamp).nanoseconds / 1e9

        # Extraer posición actual
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y

        # Extraer orientación (Convertir Cuaternio a Euler/Yaw)
        q = msg.pose.pose.orientation
        quart_list = [q.x, q.y, q.z, q.w]
        (roll, pitch, yaw) = quat2euler(quart_list)
        current_theta = yaw

        # Si es el primer mensaje, solo guardamos y salimos (no podemos derivar)
        if self.prev_time is None:
            self.prev_time = current_time
            self.prev_x = current_x
            self.prev_y = current_y
            self.prev_theta = current_theta
            return 0.0, 0.0

        # Calcular Deltas
        dt = current_time - self.prev_time
        
        if dt == 0.0: # Evitar división por cero
            return 0.0, 0.0

        dx = current_x - self.prev_x
        dy = current_y - self.prev_y
        d_theta = current_theta - self.prev_theta

        # Normalizar delta theta (para manejar el salto de PI a -PI)
        if d_theta > math.pi:
            d_theta -= 2 * math.pi
        elif d_theta < -math.pi:
            d_theta += 2 * math.pi
        
        # Calcular Velocidades
        # Lineal (Distancia euclidiana / dt)
        dist = math.sqrt(dx**2 + dy**2)
        v = dist / dt
        
        # Detectar signo de velocidad lineal (marcha atrás)
        # Si el movimiento va en contra de la orientación actual
        angle_move = math.atan2(dy, dx)
        angle_diff = angle_move - current_theta
        # Normalizar angle_diff
        if angle_diff > math.pi: angle_diff -= 2*math.pi
        if angle_diff < -math.pi: angle_diff += 2*math.pi
        
        if abs(angle_diff) > (math.pi / 2):
            v = -v

        # Angular
        w = d_theta / dt

        # Actualizar previos para la siguiente iteración
        self.prev_time = current_time
        self.prev_x = current_x
        self.prev_y = current_y
        self.prev_theta = current_theta

        msg.twist.twist.linear.x = v
        msg.twist.twist.angular.z = w

        self._publisher.publish(msg)



def main(args=None) -> None:
    rclpy.init(args=args)
    odometry_node= OdometryNode()
    rclpy.spin(odometry_node)
    # Destroy the node explicitly.
    # Optional. Otherwise, it will be done automatically
    # when the garbage collector destroys the node object.
    odometry_node.destroy_node()
    rclpy.shutdown()  # Or rclpy.try_shutdown()


if __name__ == '__main__':
    main()
