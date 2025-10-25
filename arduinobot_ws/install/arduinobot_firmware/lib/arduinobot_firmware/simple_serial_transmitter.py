#!/usr/bin/env pyhton3
# Importa la librería principal de ROS2 en Python
import rclpy
# Importa la clase base Node para crear nodos en ROS2
from rclpy.node import Node
# Importa el tipo de mensaje String del paquete std_msgs
from std_msgs.msg import String
import serial
# Define una clase llamada SimpleSerialTransmitter que hereda de Node
class SimpleSerialTransmitter(Node):
    # Constructor de la clase (se ejecuta al crear el nodo)
    def __init__(self):
        # Llama al constructor de la clase padre (Node) y asigna el nombre del nodo
        super().__init__("simple_seria_transmitter")
        # Crea una suscripción al tópico "serial_transmitter"
        # Recibe mensajes tipo String y ejecuta la función msgCallback cuando llega un mensaje
        # El número 10 indica la "cola" de mensajes (cuántos puede almacenar mientras procesa)
        
        self.declare_parameter("port","/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 115200)
        
        
        self.port_ = self.get_parameter("port").value

        self.baud_rate_ = self.get_parameter("baud_rate").value

        self.sub_ = self.create_subscription(String, "serial_transmitter", self.msgCallback, 10)

        self.arduino_ = serial.Serial(port=self.port_, baudrate=self.baud_rate_ , timeout=0.1)

    # Función que se ejecuta cada vez que llega un mensaje al tópico
    def msgCallback(self, msg):
        # Muestra el contenido del mensaje en la consola usando el logger de ROS2
        self.get_logger().info("New message received, publishing on serial port: %s" % self.arduino_.name)
        self.arduino_.write(msg.data.encode("utf-8"))


# Función principal que arranca el nodo
def main():
    # Inicializa el sistema de ROS2
    rclpy.init()
    # Crea una instancia del nodo SimpleSerialTransmitter
    simple_seria_transmitter = SimpleSerialTransmitter()
    # Mantiene el nodo en ejecución escuchando mensajes
    rclpy.spin(simple_seria_transmitter)
    # Cuando el nodo termina, se destruye limpiamente
    simple_seria_transmitter.destroy_node()
    # Cierra la comunicación con ROS2
    rclpy.shutdown()

# Punto de entrada del programa: ejecuta main() si el archivo se corre directamente
if __name__ == '__main__':
    main()
