#include <rclcpp/rclcpp.hpp>                // Incluye la librería principal de ROS2 para C++
#include <std_msgs/msg/string.hpp>          // Incluye el tipo de mensaje estándar String
#include <chrono>                           // Incluye utilidades para manejar tiempo y duraciones
using namespace std::chrono_literals;       // Permite escribir duraciones con sufijos (ej: 1s)

// Definición de una clase que hereda de rclcpp::Node (un nodo en ROS2)
class SimplePublisher : public rclcpp::Node{
    public:
        // Constructor del nodo
        SimplePublisher() : Node("simple_publisher"), counter_(0)        
        {
            // Crea un publicador de mensajes tipo String en el tópico "chatter" con cola de tamaño 10
            pub_ = create_publisher<std_msgs::msg::String>("chatter",10);

            // Crea un temporizador que ejecuta la función callback cada 1 segundo
            timer_ = create_wall_timer(1s, std::bind(&SimplePublisher::timerCallback,this));

            // Imprime en consola que el nodo publicará a 1 Hz
            RCLCPP_INFO(get_logger(), "Publishing at 1 Hz");
        }

        // Función que se ejecuta cada vez que vence el temporizador (cada 1 segundo)
        void timerCallback(){
            auto message = std_msgs::msg::String();                          // Crea un mensaje tipo String
            message.data = "Hello ROS 2 - counter: " + std::to_string(counter_++); // Agrega texto con un contador
            pub_->publish(message);                                          // Publica el mensaje en el tópico
        }

    private:
        unsigned int counter_;   // Contador que aumenta cada vez que se publica un mensaje
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;   // Puntero al publicador
        rclcpp::TimerBase::SharedPtr timer_;                       // Puntero al temporizador
};  // Fin de la clase (aquí era donde faltaba el punto y coma)

// Función principal
int main(int argc, char* argv[]){
    rclcpp::init(argc,argv);                        // Inicializa ROS2
    auto node = std::make_shared<SimplePublisher>(); // Crea un objeto compartido del nodo SimplePublisher
    rclcpp::spin(node);                             // Mantiene el nodo en ejecución escuchando y publicando
    rclcpp::shutdown();                             // Apaga ROS2 de manera segura
    return 0;                                       // Fin del programa
}
