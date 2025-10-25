#include <rclcpp/rclcpp.hpp>                // Incluye la librería principal de ROS2 para C++
#include <std_msgs/msg/string.hpp>          // Incluye el tipo de mensaje estándar String
#include <chrono>                           // Incluye utilidades para manejar tiempo y duraciones
#include <libserial/SerialPort.h>
using namespace std::chrono_literals;       // Permite escribir duraciones con sufijos (ej: 1s)

// Definición de una clase que hereda de rclcpp::Node (un nodo en ROS2)
class SimpleSerialReceiver : public rclcpp::Node{
    public:
        // Constructor del nodo
        SimpleSerialReceiver() : Node("simple_serial_receiver")       
        {
            declare_parameter<std::string>("port","/dev/ttyUSB0");

            std::string port_ = get_parameter("port").as_string();

            // Crea un publicador de mensajes tipo String en el tópico "serial_receiver" con cola de tamaño 10
            pub_ = create_publisher<std_msgs::msg::String>("serial_receiver",10);

            // Crea un temporizador que ejecuta la función callback cada 1 segundo
            timer_ = create_wall_timer(0.01s, std::bind(&SimpleSerialReceiver::timerCallback,this));

            arduino_.Open(port_);
            arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);

        }

        ~SimpleSerialReceiver(){
            arduino_.Close();
        }

        // Función que se ejecuta cada vez que vence el temporizador (cada 1 segundo)
        void timerCallback(){
            
            auto message = std_msgs::msg::String();
            if(rclcpp::ok() && arduino_.IsDataAvailable()){
                arduino_.ReadLine(message.data);
            }                          // Crea un mensaje tipo String
            pub_->publish(message);    // Publica el mensaje en el tópico
        }

    private:
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;   // Puntero al publicador
        rclcpp::TimerBase::SharedPtr timer_;                       // Puntero al temporizador
        LibSerial::SerialPort arduino_;
};  // Fin de la clase (aquí era donde faltaba el punto y coma)

// Función principal
int main(int argc, char* argv[]){
    rclcpp::init(argc,argv);                        // Inicializa ROS2
    auto simple_serial_receiver = std::make_shared<SimpleSerialReceiver>(); // Crea un objeto compartido del nodo SimpleSerialReceiver
    rclcpp::spin(simple_serial_receiver);                             // Mantiene el nodo en ejecución escuchando y publicando
    rclcpp::shutdown();                             // Apaga ROS2 de manera segura
    return 0;                                       // Fin del programa
}