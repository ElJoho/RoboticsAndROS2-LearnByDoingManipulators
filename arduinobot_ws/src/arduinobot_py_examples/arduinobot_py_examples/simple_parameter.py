# Importa la librería principal de ROS2 para Python
import rclpy
# Importa la clase Node, que es la base para crear nodos en ROS2
from rclpy.node import Node
# Importa la estructura SetParametersResult, usada para indicar si un cambio de parámetro fue exitoso
from rcl_interfaces.msg import SetParametersResult
# Importa la clase Parameter, que representa parámetros dentro de un nodo
from rclpy.parameter import Parameter


# Definición de la clase SimpleParameter, que hereda de Node
class SimpleParameter(Node):
    def __init__(self):
        # Inicializa el nodo con el nombre "simple_paramaeter"
        super().__init__("simple_paramaeter")

        # Declara un parámetro entero llamado "simple_int_param" con valor por defecto 28
        self.declare_parameter("simple_int_param", 28)

        # Declara un parámetro tipo string llamado "simple_string_param" con valor por defecto "Anatonio"
        self.declare_parameter("simple_string_param", "Anatonio")

        # Registra un callback para manejar los cambios en los parámetros
        self.add_on_set_parameters_callback(self.paramChangeCallback)


    # Callback que se ejecuta cada vez que se intenta cambiar un parámetro
    def paramChangeCallback(self, params):
        # Objeto que indica si el cambio fue exitoso o no
        result = SetParametersResult()

        # Recorre todos los parámetros modificados
        for param in params:
            # Si el parámetro es "simple_int_param" y es de tipo entero
            if param.name == "simple_int_param" and param.type == Parameter.Type.INTEGER:
                # Imprime en consola el nuevo valor del parámetro
                self.get_logger().info("Param simple_int_param changed! New value is: %d" % param.value)
                # Marca el cambio como exitoso
                result.successful = True

            # Si el parámetro es "simple_string_param" y es de tipo string
            # (⚠️ Aquí hay un error: debería usarse param.type en lugar de param.type_)
            if param.name == "simple_string_param" and param.type == Parameter.Type.STRING:
                # Imprime en consola el nuevo valor del parámetro
                # (⚠️ Aquí también hay un error: %d es para enteros, debería ser %s para strings)
                self.get_logger().info("Param simple_string_param changed! New value is: %s" % param.value)

        # Devuelve el resultado del intento de cambio
        return result


# Función principal que arranca el nodo
def main():
    # Inicializa el sistema de ROS2
    rclpy.init()

    # Crea una instancia del nodo SimpleParameter
    simple_parameter = SimpleParameter()

    # Mantiene el nodo ejecutándose para escuchar cambios en parámetros
    rclpy.spin(simple_parameter)

    # Destruye el nodo al finalizar
    simple_parameter.destroy_node()

    # Apaga el sistema de ROS2
    rclpy.shutdown()


# Punto de entrada del script
if __name__ == '__main__':
    main()
