import rclpy  # Importa la librería principal de ROS 2 para Python
from rclpy.node import Node  # Importa la clase base para crear nodos en ROS 2
from arduinobot_msgs.srv import AddTwoInts  # Importa el tipo de servicio personalizado AddTwoInts


class SimpleServiceServer(Node):  # Define un nodo que actuará como servidor de servicio
    def __init__(self):  # Constructor del nodo
        super().__init__("simple_service_server")  # Inicializa el nodo con el nombre "simple_service_server"
        
        self.service_ = self.create_service(  # Crea el servidor del servicio y lo registra en ROS 2
            AddTwoInts,                      # Tipo del servicio (AddTwoInts.srv)
            "add_two_ints",                  # Nombre del servicio en el grafo ROS
            self.serviceCallback             # Función callback que atiende las peticiones del servicio
        )
        self.get_logger().info("Service add_two_ints Ready")  # Mensaje de log (dejado exactamente igual)

    def serviceCallback(self, req, res):  # Callback del servicio: recibe la petición (req) y la respuesta (res)
        self.get_logger().info("New Message Received a: %d, b: %d" % (req.a, req.b))  # Log con los valores recibidos (sin cambios)
        res.sum = req.a + req.b  # Calcula la suma y la guarda en el campo 'sum' de la respuesta
        self.get_logger().info("Returning sum: %d" %res.sum)  # Log del resultado que se devolverá (sin cambios)
        return res  # Devuelve la respuesta al cliente


def main():  # Función principal (debe estar fuera de la clase)
    rclpy.init()  # Inicializa la comunicación con ROS 2
    simple_service_server = SimpleServiceServer()  # Crea una instancia del servidor de servicio
    rclpy.spin(simple_service_server)  # Mantiene el nodo ejecutándose para atender solicitudes
    simple_service_server.destroy_node()  # Destruye el nodo al finalizar
    rclpy.shutdown()  # Cierra la comunicación con ROS 2


if __name__ == "__main__":  # Punto de entrada cuando se ejecuta el script directamente
    main()  # Llama a la función principal
