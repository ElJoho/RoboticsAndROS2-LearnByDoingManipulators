#include <rclcpp/rclcpp.hpp>  // Cabecera principal de ROS 2 en C++ (nodos, logging, etc.)
#include <memory>              // Punteros inteligentes std::shared_ptr, std::make_shared
#include <functional>          // std::bind y std::placeholders
#include <arduinobot_msgs/srv/add_two_ints.hpp>  // Interfaz del servicio AddTwoInts (req: a,b; res: sum)

using namespace std::placeholders;  // Importa _1, _2, ... para enlazar parámetros en std::bind

class SimpleServiceServer : public rclcpp::Node{  // Declara una clase de nodo que hereda de rclcpp::Node
    public:
        SimpleServiceServer() : Node("simple_service_server"){  // Constructor: nombra el nodo "simple_service_server"
            service_ = create_service<arduinobot_msgs::srv::AddTwoInts>(  // Crea el servidor del servicio con tipo AddTwoInts
                "add_two_ints",                                           // Nombre del servicio en el grafo ROS
                std::bind(                                                // Enlaza el callback miembro con this
                    &SimpleServiceServer::serviceCallback,                // Puntero al método callback
                    this,                                                 // Instancia actual del objeto (this)
                    _1,                                                   // Placeholder para el primer argumento (Request)
                    _2                                                    // Placeholder para el segundo argumento (Response)
                )
            );

            RCLCPP_INFO(                                  // Emite un log informativo
                rclcpp::get_logger("rclcpp"),             // Usa el logger con nombre "rclcpp"
                "Service add_two_ints is Ready"           // Mensaje de que el servicio está listo
            );
        }
    private:
        rclcpp::Service<arduinobot_msgs::srv::AddTwoInts>::SharedPtr service_;  // Manejador del servidor de servicio
        void serviceCallback(                                                    // Callback que atiende cada solicitud del servicio
            const std::shared_ptr<arduinobot_msgs::srv::AddTwoInts::Request> req,   // Puntero a la petición (contiene a y b)
            const std::shared_ptr<arduinobot_msgs::srv::AddTwoInts::Response> res   // Puntero a la respuesta (devolverá sum)
        ){
            RCLCPP_INFO_STREAM(                               // Log informativo con stream (<<)
                rclcpp::get_logger("rclcpp"),                 // Logger "rclcpp"
                "New Request received a: " << req -> a << " b: " << req -> b  // Imprime los valores recibidos
            );
            res -> sum = req -> a + req -> b;                 // Calcula la suma y la asigna al campo sum de la respuesta
            RCLCPP_INFO_STREAM(                               // Otro log con el resultado
                rclcpp::get_logger("rclcpp"),                 // Logger "rclcpp"
                "Returning sum: " << res -> sum               // Imprime la suma calculada
            );
        }
};

int main(int argc, char* argv[]){            // Función principal del programa
    rclcpp::init(argc,argv);                 // Inicializa ROS 2 con argumentos de línea de comandos

    auto node = std::make_shared<SimpleServiceServer>();  // Crea una instancia compartida del nodo servidor
    rclcpp::spin(node);                                    // Mantiene el nodo ejecutándose para atender solicitudes
    rclcpp::shutdown();                                    // Cierra y limpia recursos de ROS 2
    return 0;                                              // Código de salida correcto
}
