from launch import LaunchDescription                           # Importa la clase que describe la lista de acciones/nodos a lanzar.
from launch_ros.actions import Node                            # Importa la acción para definir y lanzar nodos de ROS 2.
from launch_ros.parameter_descriptions import ParameterValue   # Permite definir valores de parámetros (p.ej., robot_description).
from launch.actions import DeclareLaunchArgument               # Acción para declarar argumentos que el usuario puede pasar al launch.
from launch.substitutions import Command, LaunchConfiguration  # Herramientas para construir strings/valores dinámicos en tiempo de lanzamiento.
import os                                                      # Módulo estándar para manipular rutas y el sistema de archivos.
from ament_index_python.packages import get_package_share_directory  # Obtiene la ruta de instalación (share) de un paquete de ROS 2.

def generate_launch_description():                             # Función obligatoria que ROS 2 invoca al ejecutar el launch.
    model_arg = DeclareLaunchArgument(                         # Declara un argumento de launch llamado "model".
        name="model",                                          # Nombre del argumento: se referenciará como LaunchConfiguration("model").
        default_value=os.path.join(                            # Valor por defecto si el usuario no pasa otro: se construye una ruta válida.
            get_package_share_directory("arduinobot_description"),  # Busca el directorio 'share' del paquete 'arduinobot_description'.
            "urdf",                                            # Entra a la carpeta 'urdf' dentro del paquete.
            "arduinobot.urdf.xacro"                            # Archivo Xacro del robot que se convertirá a URDF.
        ),
        description="Absolute path to the robot URDF file"     # Texto descriptivo que aparece en ayuda/errores del launch.
    )

    robot_description = ParameterValue(                        # Define el valor del parámetro 'robot_description' que consumirá el nodo.
        Command(                                               # Ejecuta un comando del sistema en tiempo de lanzamiento y toma su salida.
            [                                                  # La salida de este comando será el contenido URDF generado por xacro.
                "xacro ",                                      # Comando 'xacro' con un espacio final para evitar que se pegue a la ruta.
                LaunchConfiguration("model")                   # Sustitución: toma el valor actual del argumento 'model' (ruta del Xacro).
            ]
        ),
        value_type=str                                         # Indica que el resultado del comando se tratará como cadena (string).
    )

    robot_state_publisher_node = Node(                         # Define el nodo que publicará el URDF en /robot_description y TF.
        package="robot_state_publisher",                       # Paquete donde vive el ejecutable.
        executable="robot_state_publisher",                    # Nombre del ejecutable a lanzar.
        parameters=[{"robot_description": robot_description}]  # Pasa el parámetro 'robot_description' ya resuelto (URDF plano).
    )

    joint_state_publisher_gui = Node(                          # Define el nodo GUI para mover articulaciones con sliders.
        package="joint_state_publisher_gui",                   # Paquete del publicador de estados de junta con interfaz gráfica.
        executable="joint_state_publisher_gui"                 # Ejecutable que abre la ventana con controles deslizantes.
    )

    rviz_node = Node(                                          # Define el nodo que arrancará RViz2 para visualizar el robot.
        package="rviz2",                                       # Paquete de RViz2.
        executable="rviz2",                                    # Ejecutable principal de RViz2.
        name="rviz2",                                          # Nombre del nodo en el grafo de ROS 2 (útil para distinguir instancias).
        output="screen",                                       # Envía la salida de consola (logs) al terminal actual.
        arguments=[                                            # Argumentos de línea de comandos para RViz2.
            "-d",                                              # Bandera para cargar un archivo de configuración al iniciar.
            os.path.join(                                      # Construye la ruta absoluta del archivo .rviz dentro del paquete.
                get_package_share_directory("arduinobot_description"),  # Directorio 'share' del paquete de descripción.
                "rviz",                                        # Carpeta donde se guarda la configuración de RViz.
                "display.rviz"                                 # Archivo de configuración que define vistas, displays, y temas.
            )
        ]
    )

    return LaunchDescription([                                 # Devuelve la descripción del lanzamiento con todas las acciones/nodos.
        model_arg,                                             # 1) El argumento 'model' (para poder sobreescribir la ruta del Xacro).
        robot_state_publisher_node,                                 # 2) Nodo que publica el modelo y los frames.
        joint_state_publisher_gui,                             # 3) Nodo GUI para manipular articulaciones virtualmente.
        rviz_node                                              # 4) Nodo de RViz2 para visualizar el robot y sus transformaciones.
    ])