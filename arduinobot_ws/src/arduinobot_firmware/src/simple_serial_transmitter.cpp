#include <rclcpp/rclcpp.hpp>       // Incluye la librería principal de ROS 2 para trabajar con nodos, publishers, subscribers, etc.
#include <std_msgs/msg/string.hpp> // Incluye el tipo de mensaje String definido en std_msgs
#include <chrono>                  // Incluye utilidades para manejar tiempo (aunque en este código no se usa directamente)
#include <memory>                  // Necesario para usar std::make_shared y punteros inteligentes
#include <functional>              // Necesario para usar std::bind y placeholders
#include <libserial/SerialPort.h>
using std::placeholders::_1;       // Permite usar _1 como marcador de posición en std::bind para el primer argumento

// Definición de la clase SimpleSerialTransmitter que hereda de rclcpp::Node
class SimpleSerialTransmitter : public rclcpp::Node{
public:
    // Constructor de la clase. Inicializa el nodo con el nombre "simple_serial_transmitter"
    SimpleSerialTransmitter() : Node("simple_serial_transmitter") {
        declare_parameter<std::string>("port","/dev/ttyUSB0");

        std::string port_ = get_parameter("port").as_string();

        // Crea una suscripción al tópico "serial_transmitter" con cola de tamaño 10.
        // std::bind enlaza el método msgCallback con "this" y el placeholder _1 (el mensaje recibido).
        sub_ = create_subscription<std_msgs::msg::String>(
            "serial_transmitter", 
            10, 
            std::bind(&SimpleSerialTransmitter::msgCallback, this, _1)
        );

        arduino_.Open(port_);

        arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);

    }

    ~SimpleSerialTransmitter(){
        arduino_.Close();
    }

    // Método callback que se ejecuta cada vez que llega un mensaje al tópico "serial_transmitter"
    // Recibe el mensaje como un puntero compartido constante (SharedPtr).
    void msgCallback(const std_msgs::msg::String &msg){
        
        
        // Muestra en consola el contenido del mensaje recibido.
        RCLCPP_INFO_STREAM(get_logger(), "New message received, publishing on serial port: " << msg.data);
        arduino_.Write(msg.data);

    }

private:
    // Puntero compartido a la suscripción, necesario para mantener la suscripción activa mientras el nodo viva.
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    LibSerial::SerialPort arduino_;
};

// Función principal del programa
int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);                       // Inicializa ROS 2 con los argumentos de línea de comandos
    auto node = std::make_shared<SimpleSerialTransmitter>(); // Crea una instancia compartida del nodo SimpleSerialTransmitter
    rclcpp::spin(node);                             // Mantiene el nodo en ejecución, escuchando mensajes y ejecutando callbacks
    rclcpp::shutdown();                             // Finaliza ROS 2 y libera recursos
    return 0;                                       // Retorna 0 indicando que el programa terminó correctamente
}
