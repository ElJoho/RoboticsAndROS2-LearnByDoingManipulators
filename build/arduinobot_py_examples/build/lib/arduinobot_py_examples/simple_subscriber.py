# Importa la librería principal de ROS2 en Python
import rclpy
# Importa la clase base Node para crear nodos en ROS2
from rclpy.node import Node
# Importa el tipo de mensaje String del paquete std_msgs
from std_msgs.msg import String

# Define una clase llamada SimpleSubscriber que hereda de Node
class SimpleSubscriber(Node):
    # Constructor de la clase (se ejecuta al crear el nodo)
    def __init__(self):
        # Llama al constructor de la clase padre (Node) y asigna el nombre del nodo
        super().__init__("simple_subscriber")
        # Crea una suscripción al tópico "chatter"
        # Recibe mensajes tipo String y ejecuta la función msgCallback cuando llega un mensaje
        # El número 10 indica la "cola" de mensajes (cuántos puede almacenar mientras procesa)
        self.sub_ = self.create_subscription(String, "chatter", self.msgCallback, 10)

    # Función que se ejecuta cada vez que llega un mensaje al tópico
    def msgCallback(self, msg):
        # Muestra el contenido del mensaje en la consola usando el logger de ROS2
        self.get_logger().info("I heard: %s" % msg.data)

# Función principal que arranca el nodo
def main():
    # Inicializa el sistema de ROS2
    rclpy.init()
    # Crea una instancia del nodo SimpleSubscriber
    simple_subscriber = SimpleSubscriber()
    # Mantiene el nodo en ejecución escuchando mensajes
    rclpy.spin(simple_subscriber)
    # Cuando el nodo termina, se destruye limpiamente
    simple_subscriber.destroy_node()
    # Cierra la comunicación con ROS2
    rclpy.shutdown()

# Punto de entrada del programa: ejecuta main() si el archivo se corre directamente
if __name__ == '__main__':
    main()
