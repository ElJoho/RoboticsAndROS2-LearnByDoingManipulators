#include <rclcpp/rclcpp.hpp>  // Cabecera principal de ROS 2 en C++ (nodos, logging, etc.)
#include <memory>              // Punteros inteligentes std::shared_ptr, std::make_shared
#include <functional>          // std::bind y std::placeholders
#include <arduinobot_msgs/srv/euler_to_quaternion.hpp>     // Interfaz del servicio EulerToQuaternion (req: roll,pitch,yaw; res: x,y,z,w)
#include <arduinobot_msgs/srv/quaternion_to_euler.hpp>     // Interfaz del servicio QuaternionToEuler (req: x,y,z,w; res: roll,pitch,yaw)
#include <tf2/LinearMath/Quaternion.h>   // tf2::Quaternion
#include <tf2/LinearMath/Matrix3x3.h>    // tf2::Matrix3x3

using namespace std::placeholders;  // Importa _1, _2 para enlazar parámetros en std::bind

// Nodo que ofrece dos servicios para convertir entre Euler <-> Cuaternión
class AnglesConverter : public rclcpp::Node{
    public:
        AnglesConverter() : Node("angles_conversion_service"){  // Constructor: nombra el nodo
            // Servidor: Euler -> Quaternion
            euler_to_quaternion = create_service<arduinobot_msgs::srv::EulerToQuaternion>(
                "euler_to_quaternion",                           // Nombre del servicio en el grafo ROS 2
                std::bind(                                       // Enlaza el callback miembro con this
                    &AnglesConverter::eulerToQuaternionCallback, // Método callback
                    this,                                        // Instancia actual del objeto
                    _1,                                          // Placeholder para Request
                    _2                                           // Placeholder para Response
                )
            );

            // Servidor: Quaternion -> Euler
            quaternion_to_euler = create_service<arduinobot_msgs::srv::QuaternionToEuler>(
                "quaternion_to_euler",                           // Nombre del servicio en el grafo ROS 2
                std::bind(
                    &AnglesConverter::quaternionToEulerCallback, // Método callback
                    this,
                    _1,
                    _2
                )
            );
            
            // Mensaje informativo de arranque
            RCLCPP_INFO(
                rclcpp::get_logger("rclcpp"),
                "Angle conversion services are ready"
            );
        }

    private:
        // Manejadores de servidores de servicio
        rclcpp::Service<arduinobot_msgs::srv::EulerToQuaternion>::SharedPtr euler_to_quaternion;
        rclcpp::Service<arduinobot_msgs::srv::QuaternionToEuler>::SharedPtr quaternion_to_euler;

        // Callback: convierte (roll,pitch,yaw) -> (x,y,z,w)
        void eulerToQuaternionCallback(
            const std::shared_ptr<arduinobot_msgs::srv::EulerToQuaternion::Request> req,  // Petición con ángulos de Euler
            const std::shared_ptr<arduinobot_msgs::srv::EulerToQuaternion::Response> res  // Respuesta con cuaternión
        ){
            RCLCPP_INFO_STREAM(
                rclcpp::get_logger("rclcpp"),
                "Request to convert euler angles roll: " << req->roll
                << ", pitch: " << req->pitch
                << ", yaw: "   << req->yaw
                << " into a quaternion"
            );

            tf2::Quaternion q;
            q.setRPY(req->roll, req->pitch, req->yaw);  // Construye el cuaternión a partir de Euler (RPY)

            res->x = q.getX();
            res->y = q.getY();
            res->z = q.getZ();
            res->w = q.getW();

            RCLCPP_INFO_STREAM(
                rclcpp::get_logger("rclcpp"),
                "Corresponding quaternion x: " << res->x
                << ", y: " << res->y
                << ", z: " << res->z
                << ", w: " << res->w
            );
        }

        // Callback: convierte (x,y,z,w) -> (roll,pitch,yaw)
        void quaternionToEulerCallback(
            const std::shared_ptr<arduinobot_msgs::srv::QuaternionToEuler::Request> req,  // Petición con cuaternión
            const std::shared_ptr<arduinobot_msgs::srv::QuaternionToEuler::Response> res  // Respuesta con ángulos de Euler
        ){
            RCLCPP_INFO_STREAM(
                rclcpp::get_logger("rclcpp"),
                "Request to convert quaternions x: " << req->x
                << ", y: " << req->y
                << " , z: " << req->z
                << " , w: " << req->w
                << " into euler angles"
            );

            tf2::Quaternion q(req->x, req->y, req->z, req->w);  // Construye el cuaternión con los 4 coeficientes
            tf2::Matrix3x3 m(q);                                // Matriz de rotación a partir del cuaternión
            m.getRPY(res->roll, res->pitch, res->yaw);          // Extrae roll, pitch, yaw

            RCLCPP_INFO_STREAM(
                rclcpp::get_logger("rclcpp"),
                "Corresponding Euler Angles roll: " << res->roll
                << ", pitch: " << res->pitch
                << ", yaw: "   << res->yaw
            );
        }
};

int main(int argc, char* argv[]){                 // Función principal
    rclcpp::init(argc, argv);                     // Inicializa ROS 2 con argumentos de línea de comandos
    auto node = std::make_shared<AnglesConverter>();  // Crea la instancia del nodo servidor
    rclcpp::spin(node);                           // Mantiene el nodo ejecutándose para atender solicitudes
    rclcpp::shutdown();                           // Cierra y limpia recursos de ROS 2
    return 0;                                     // Código de salida correcto
}