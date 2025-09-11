#!/usr/bin/env python3

import rclpy  # Importa la librería principal de ROS 2 para Python
from rclpy.node import Node  # Importa la clase base para crear nodos en ROS 2
from arduinobot_msgs.srv import EulerToQuaternion, QuaternionToEuler
from tf_transformations import quaternion_from_euler, euler_from_quaternion

class AnglesConverter(Node):  # Define un nodo que expone servicios de conversión de ángulos
    def __init__(self):  # Constructor del nodo
        super().__init__("angles_converter")  # Inicializa el nodo con el nombre "angles_converter"
        
        self.euler_to_quaternion_ = self.create_service(  # Crea el servidor del servicio y lo registra en ROS 2
            EulerToQuaternion,                      # Tipo del servicio
            "euler_to_quaternion",                  # Nombre del servicio en el grafo de ROS 2
            self.euelerToQuaternionCallback         # Función callback que atiende las peticiones del servicio
        )

        self.quaternion_to_euler_ = self.create_service(  # Crea el servidor del servicio y lo registra en ROS 2
            QuaternionToEuler,                      # Tipo del servicio
            "quaternion_to_euler",                  # Nombre del servicio en el grafo de ROS 2
            self.quaternionToEulerCallback          # Función callback que atiende las peticiones del servicio
        )
        self.get_logger().info("Angle Conversion Services are ready")  # Mensaje de log indicando que los servicios están listos

    def euelerToQuaternionCallback(self, req, res):  # Callback del servicio: recibe la petición (req) y escribe la respuesta (res)
        self.get_logger().info("Requested to convert euler angles roll: %f, pitch: %f, yaw: %f, into a quaternion." % (req.roll, req.pitch, req.yaw))
        (res.x, res.y, res.z, res.w) = quaternion_from_euler(req.roll, req.pitch, req.yaw)
        self.get_logger().info("Corresponding quaternion x: %f, y: %f, z: %f, w: %f" % (res.x, res.y, res.z, res.w))  # Registra el cuaternión resultante (valores de res.*)
        return res
    
    def quaternionToEulerCallback(self, req, res):  # Callback del servicio: recibe la petición (req) y escribe la respuesta (res)
        self.get_logger().info("Requested to convert quaternion x: %f, y: %f, z %f, w: %f" % (req.x, req.y, req.z, req.w))
        (res.roll, res.pitch, res.yaw) = euler_from_quaternion((req.x, req.y, req.z, req.w))
        self.get_logger().info("Corresponding euler angles roll: %f, pitch: %f, yaw: %f " % ( res.roll, res.pitch, res.yaw))
        return res  # Devuelve la respuesta del servicio


def main():  # Función principal (debe estar fuera de la clase)
    rclpy.init()  # Inicializa la comunicación con ROS 2
    angles_converter = AnglesConverter()  # Crea una instancia del nodo
    rclpy.spin(angles_converter)  # Mantiene el nodo ejecutándose para atender solicitudes
    angles_converter.destroy_node()  # Destruye el nodo al finalizar
    rclpy.shutdown()  # Cierra la comunicación con ROS 2


if __name__ == "__main__":  # Punto de entrada cuando se ejecuta el script directamente
    main()  # Llama a la función principal
