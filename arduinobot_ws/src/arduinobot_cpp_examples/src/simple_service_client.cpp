#include <rclcpp/rclcpp.hpp> // Incluye la API principal de ROS 2 para C++ (nodos, logging, clientes/servicios, etc.)
#include <memory>             // Incluye utilidades de memoria (std::shared_ptr, std::make_shared)
#include <arduinobot_msgs/srv/add_two_ints.hpp> // Incluye la definición del servicio AddTwoInts (request/response)
#include <chrono>             // Incluye utilidades de tiempo (durations, literales)
#include <functional>         // Incluye std::bind y placeholders para callbacks
#include <cstdlib>            // Incluye atoi para convertir argumentos de char* a int

using namespace std::chrono_literals; // Habilita literales como 1s (segundos) para std::chrono
using std::placeholders::_1;          // Importa el placeholder _1 para std::bind (primer argumento del callback)

class SimpleServiceClient : public rclcpp::Node{ // Declara una clase de nodo ROS 2 que actuará como cliente de servicio
    public:
        SimpleServiceClient(int a, int b) : Node("simple_service_client") // Constructor: nombra el nodo y recibe los dos enteros a sumar
        {
            client_ = create_client<arduinobot_msgs::srv::AddTwoInts>("add_two_ints"); // Crea el cliente al servicio /add_two_ints de tipo AddTwoInts

            auto request = std::make_shared<arduinobot_msgs::srv::AddTwoInts::Request>(); // Reserva la request del servicio
            request->a = a; // Asigna el primer operando a la request
            request->b = b; // Asigna el segundo operando a la request

            while(!client_->wait_for_service(1s)){ // Espera hasta 1s a que el servicio esté disponible; si no, repite
                RCLCPP_ERROR(
                    rclcpp::get_logger("rclcpp"), // Obtiene el logger global "rclcpp"
                    "Service not available, waiting more time ..." // Mensaje de error/aviso mientras no aparece el servicio
                );
                if(!rclcpp::ok()){ // Si ROS se está cerrando/interrumpido, aborta la espera
                    RCLCPP_ERROR(
                        rclcpp::get_logger("rclcpp"), // Logger
                        "Interrupted while waiting for service" // Informa que se interrumpió la espera
                    );
                    return; // Sale del constructor si no tiene sentido seguir esperando
                }
            }

            auto result = client_->async_send_request( // Envía la request de forma asíncrona y registra un callback para la respuesta
                request, // Mensaje de solicitud con a y b
                std::bind(
                    &SimpleServiceClient::responseCallback, // Método miembro que actuará como callback al completarse la llamada
                    this,_1 // Pasa el puntero this y el primer argumento (future) que entrega rclcpp al callback
                )
            );

        } // Fin del constructor
    private:
        rclcpp::Client<arduinobot_msgs::srv::AddTwoInts>::SharedPtr client_; // Puntero compartido al cliente del servicio AddTwoInts

        void responseCallback(rclcpp::Client<arduinobot_msgs::srv::AddTwoInts>::SharedFuture future){ // Callback que maneja la respuesta del servicio
            if(future.valid()){ // Verifica que el future contenga un resultado válido
                RCLCPP_INFO_STREAM(
                    rclcpp::get_logger("rclcpp"), // Logger
                    "Service Response" << future.get() -> sum // Imprime la suma recibida desde la respuesta del servicio
                );
            }else{ // Si no hay resultado válido
                RCLCPP_ERROR(
                    rclcpp::get_logger("rclcpp"), // Logger
                    "Service failure" // Informa fallo en la llamada al servicio
                );
            }
        }
};

int main(int argc, char* argv[]){ // Punto de entrada del programa
    rclcpp::init(argc,argv); // Inicializa la infraestructura de ROS 2 con los argumentos de línea de comandos

    if(argc != 3){ // Valida que se hayan pasado exactamente dos argumentos (A y B)
        RCLCPP_ERROR(
            rclcpp::get_logger("rclcpp"), // Logger
            "Wrong number of arguments! Usage: simple_service_client A B" // Mensaje de uso correcto
        );
        return 1; // Código de error si faltan argumentos
    }
    auto node = std::make_shared<SimpleServiceClient>(atoi(argv[1]),atoi(argv[2])); // Crea el nodo cliente convirtiendo A y B a enteros
    rclcpp::spin(node); // Mantiene el nodo activo para procesar la respuesta asíncrona del servicio
    rclcpp::shutdown(); // Cierra limpiamente la infraestructura de ROS 2
    return 0; // Fin del programa con éxito
}
