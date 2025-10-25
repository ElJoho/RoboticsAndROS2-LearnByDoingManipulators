#!/usr/bin/env python3

# Importa el cliente de ROS 2 para Python
import rclpy
# Importa la clase base Node para crear nodos
from rclpy.node import Node
# Importa el tipo de mensaje String del paquete std_msgs
from std_msgs.msg import String

import serial

# Define un nodo publicador simple que hereda de Node
class SimpleSerialReceiver(Node):
    def __init__(self):
        # Inicializa el nodo con el nombre "simple_serial_receiver"
        super().__init__('simple_serial_receiver')
        # Crea un publicador en el tópico "serial_receiver" con cola (depth) 10 para mensajes String
        self.pub_ = self.create_publisher(String, 'serial_receiver', 10)
        
        self.frequency_ = 0.01


        self.declare_parameter("port","/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 115200)
        
        
        self.port_ = self.get_parameter("port").value

        self.baud_rate_ = self.get_parameter("baud_rate").value

        self.arduino_ = serial.Serial(port = self.port_,baudrate= self.baud_rate_,timeout=0.1)
        
        # Crea un temporizador que dispara cada 'period_s' y llama a 'timer_callback'
        self.timer_ = self.create_timer(self.frequency_, self.timer_callback)


    # Callback que se ejecuta cada vez que vence el temporizador
    def timer_callback(self):
        if rclpy.ok() and self.arduino_.is_open():
            data = self.arduino_.readline()
            try:
                data.decode("utf-8")
            except:
                return
            

            # Crea una instancia del mensaje String
            msg = String()
            msg.data= str(data)

            # Publica el mensaje en el tópico "serial_receiver"
            self.pub_.publish(msg)
            

# Punto de entrada del programa
def main(args=None):
    # Inicializa el sistema de ROS 2 (acepta argumentos de línea de comandos si los hay)
    rclpy.init(args=args)
    # Crea una instancia del nodo publicador
    simple_serial_receiver = SimpleSerialReceiver()
    try:
        # Mantiene el nodo en ejecución procesando callbacks
        rclpy.spin(simple_serial_receiver)
    except KeyboardInterrupt:
        # Permite salir con Ctrl+C sin traza de error
        pass
    finally:
        # Destruye el nodo y cierra ROS 2 de forma ordenada
        simple_serial_receiver.destroy_node()
        rclpy.shutdown()

# Ejecuta main() solo si este archivo es el programa principal
if __name__ == '__main__':
    main()