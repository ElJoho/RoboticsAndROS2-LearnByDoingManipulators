#include <rclcpp/rclcpp.hpp>       // Incluye la librería principal de ROS 2 para trabajar con nodos, publishers, subscribers, etc.
#include <std_msgs/msg/string.hpp> // Incluye el tipo de mensaje String definido en std_msgs
#include <chrono>                  // Incluye utilidades para manejar tiempo (aunque en este código no se usa directamente)
#include <memory>                  // Necesario para usar std::make_shared y punteros inteligentes
#include <functional>              // Necesario para usar std::bind y placeholders
using std::placeholders::_1;       // Permite usar _1 como marcador de posición en std::bind para el primer argumento

// Definición de la clase SimpleSubscriber que hereda de rclcpp::Node
class SimpleSubscriber : public rclcpp::Node{
public:
    // Constructor de la clase. Inicializa el nodo con el nombre "simple_subscriber"
    SimpleSubscriber() : Node("simple_subscriber") {
        // Crea una suscripción al tópico "chatter" con cola de tamaño 10.
        // std::bind enlaza el método msgCallback con "this" y el placeholder _1 (el mensaje recibido).
        sub_ = create_subscription<std_msgs::msg::String>(
            "chatter", 
            10, 
            std::bind(&SimpleSubscriber::msgCallback, this, _1)
        );
    }

    // Método callback que se ejecuta cada vez que llega un mensaje al tópico "chatter"
    // Recibe el mensaje como un puntero compartido constante (SharedPtr).
    void msgCallback(const std_msgs::msg::String::SharedPtr msg) const {
        // Muestra en consola el contenido del mensaje recibido.
        RCLCPP_INFO_STREAM(get_logger(), "I heard: " << msg->data);
    }

private:
    // Puntero compartido a la suscripción, necesario para mantener la suscripción activa mientras el nodo viva.
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

// Función principal del programa
int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);                       // Inicializa ROS 2 con los argumentos de línea de comandos
    auto node = std::make_shared<SimpleSubscriber>(); // Crea una instancia compartida del nodo SimpleSubscriber
    rclcpp::spin(node);                             // Mantiene el nodo en ejecución, escuchando mensajes y ejecutando callbacks
    rclcpp::shutdown();                             // Finaliza ROS 2 y libera recursos
    return 0;                                       // Retorna 0 indicando que el programa terminó correctamente
}
