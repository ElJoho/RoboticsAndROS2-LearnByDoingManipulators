// Incluye la librería principal de ROS 2 en C++
#include <rclcpp/rclcpp.hpp>

// Incluye el mensaje usado para el resultado de la callback de parámetros
#include <rcl_interfaces/msg/set_parameters_result.hpp>

// Librerías estándar de C++
#include <string>     // Para trabajar con cadenas std::string
#include <vector>     // Para manejar vectores de parámetros
#include <memory>     // Para punteros inteligentes (make_shared, SharedPtr)
#include <functional> // Para std::bind y std::placeholders

// Se usa std::placeholders::_1 para enlazar callbacks con un parámetro
using std::placeholders::_1;

// Definición de la clase SimpleParameter que hereda de rclcpp::Node
class SimpleParameter : public rclcpp::Node {
public:
  // Constructor de la clase
  SimpleParameter() : Node("simple_parameter") {
    // Declarar un parámetro entero con valor por defecto 28
    declare_parameter<int>("simple_int_param", 28);

    // Declarar un parámetro tipo string con valor por defecto "Antonio"
    declare_parameter<std::string>("simple_string_param", "Antonio");

    // Registrar el callback que se ejecutará cuando cambien parámetros
    // std::bind enlaza el método "paramChangeCallback" con el objeto actual (this)
    // "_1" representa el argumento que recibirá el callback (vector de parámetros cambiados)
    param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&SimpleParameter::paramChangeCallback, this, _1)
    );
  }

private:
  // Handle para mantener viva la callback de parámetros mientras el nodo exista
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // Función que maneja los cambios de parámetros
  rcl_interfaces::msg::SetParametersResult
  paramChangeCallback(const std::vector<rclcpp::Parameter> &parameters) {
    // Crear el objeto de resultado que se devolverá
    rcl_interfaces::msg::SetParametersResult result;

    // Se asume que el cambio fue exitoso por defecto
    result.successful = true;

    // Iterar sobre todos los parámetros que cambiaron
    for (const auto &param : parameters) {
      // Obtener nombre y tipo del parámetro
      const auto &name = param.get_name();
      const auto type  = param.get_type();

      // Si el parámetro es "simple_int_param" y es de tipo entero
      if (name == "simple_int_param" &&
          type == rclcpp::ParameterType::PARAMETER_INTEGER) {
        // Imprimir en consola el nuevo valor
        RCLCPP_INFO(get_logger(),
                    "Param simple_int_param cambiado! Nuevo valor: %ld",
                    static_cast<long>(param.as_int()));

      // Si el parámetro es "simple_string_param" y es de tipo string
      } else if (name == "simple_string_param" &&
                 type == rclcpp::ParameterType::PARAMETER_STRING) {
        // Imprimir en consola el nuevo valor
        RCLCPP_INFO(get_logger(),
                    "Param simple_string_param cambiado! Nuevo valor: %s",
                    param.as_string().c_str());

      } else {
        // Si se intenta modificar un parámetro desconocido o con tipo inválido,
        // aquí podríamos rechazarlo poniendo: result.successful = false;
      }
    }

    // Retornar el resultado (éxito o fracaso)
    return result;
  }
};

// Función principal (punto de entrada del nodo)
int main(int argc, char *argv[]) {
  // Inicializa ROS 2 con los argumentos de línea de comandos
  rclcpp::init(argc, argv);

  // Crea una instancia del nodo SimpleParameter
  auto node = std::make_shared<SimpleParameter>();

  // Mantiene el nodo en ejecución escuchando callbacks y eventos
  rclcpp::spin(node);

  // Finaliza y libera los recursos de ROS 2
  rclcpp::shutdown();

  // Retorna 0 indicando ejecución exitosa
  return 0;
}
