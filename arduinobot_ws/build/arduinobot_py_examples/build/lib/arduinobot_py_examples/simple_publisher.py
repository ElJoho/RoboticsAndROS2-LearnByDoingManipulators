# Importa el cliente de ROS 2 para Python
import rclpy
# Importa la clase base Node para crear nodos
from rclpy.node import Node
# Importa el tipo de mensaje String del paquete std_msgs
from std_msgs.msg import String

# Define un nodo publicador simple que hereda de Node
class SimplePublisher(Node):
    def __init__(self):
        # Inicializa el nodo con el nombre "simple_publisher"
        super().__init__('simple_publisher')
        # Crea un publicador en el tópico "chatter" con cola (depth) 10 para mensajes String
        self.pub_ = self.create_publisher(String, 'chatter', 10)
        # Contador de mensajes publicados
        self.counter_ = 0
        # Frecuencia de publicación en Hz
        self.frequency_hz = 1.0
        # Periodo del temporizador en segundos (create_timer espera periodo, no frecuencia)
        self.period_s = 1.0 / self.frequency_hz
        # Mensaje informativo en el log (usa f-string para formatear floats correctamente)
        self.get_logger().info(f'Publishing at {self.frequency_hz:.1f} Hz')
        # Crea un temporizador que dispara cada 'period_s' y llama a 'timer_callback'
        self.timer_ = self.create_timer(self.period_s, self.timer_callback)

    # Callback que se ejecuta cada vez que vence el temporizador
    def timer_callback(self):
        # Crea una instancia del mensaje String
        msg = String()
        # Asigna el contenido del mensaje con el valor actual del contador
        msg.data = f'Hello ROS 2 - counter: {self.counter_}'
        # Publica el mensaje en el tópico "chatter"
        self.pub_.publish(msg)
        # Incrementa el contador para el siguiente ciclo
        self.counter_ += 1

# Punto de entrada del programa
def main(args=None):
    # Inicializa el sistema de ROS 2 (acepta argumentos de línea de comandos si los hay)
    rclpy.init(args=args)
    # Crea una instancia del nodo publicador
    node = SimplePublisher()
    try:
        # Mantiene el nodo en ejecución procesando callbacks
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Permite salir con Ctrl+C sin traza de error
        pass
    finally:
        # Destruye el nodo y cierra ROS 2 de forma ordenada
        node.destroy_node()
        rclpy.shutdown()

# Ejecuta main() solo si este archivo es el programa principal
if __name__ == '__main__':
    main()