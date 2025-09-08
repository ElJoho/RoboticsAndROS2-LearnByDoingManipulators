from launch import LaunchDescription                           # Importa la clase que describe la lista de acciones/nodos a lanzar.
from launch_ros.actions import Node                            # Importa la acción para definir y lanzar nodos de ROS 2.
from launch_ros.parameter_descriptions import ParameterValue   # Permite definir valores de parámetros (p.ej., robot_description).
from launch.actions import DeclareLaunchArgument,SetEnvironmentVariable ,IncludeLaunchDescription  # Acción para declarar argumentos, setear variables de entorno e incluir otros launch files.
from launch.launch_description_sources import PythonLaunchDescriptionSource  # Permite cargar un launch file Python como fuente de descripción.
from launch.substitutions import Command, LaunchConfiguration  # Herramientas para construir strings/valores dinámicos en tiempo de lanzamiento.
import os                                                      # Módulo estándar para manipular rutas y el sistema de archivos.
from pathlib import Path                                       # Módulo para trabajar con rutas de forma orientada a objetos.
from ament_index_python.packages import get_package_share_directory  # Obtiene la ruta de instalación (share) de un paquete de ROS 2.

def generate_launch_description():                             # Punto de entrada requerido por ROS 2 para generar la LaunchDescription.

    arduinobot_description_dir = get_package_share_directory("arduinobot_description")  # Localiza el directorio 'share' del paquete con los archivos del robot.

    ros_distro = os.environ["ROS_DISTRO"]                      # Devuelve la distribucion de ros2
    is_ignition = "True" if ros_distro == "humble"  else "False" # Si es humble devuelve verdadero
    physics_engine = "" if ros_distro == "humble" else "--physics-engine gz-physics-bullet-featherstone-plugin"

    gazebo_resource_path = SetEnvironmentVariable(             # Define/ajusta la variable de entorno para que Gazebo encuentre recursos (models, meshes, worlds).
        name="GZ_SIM_RESOURCE_PATH",                           # Nombre correcto de la variable de entorno usada por Gazebo (GZ/Fortress+).
        value=[
            str(Path(arduinobot_description_dir).parent.resolve())  # Agrega esta ruta base (padre de 'share'); útil si tus recursos están más arriba.
        ]
    )

    gazebo = IncludeLaunchDescription(                        # Incluye el launch oficial de ros_gz_sim para levantar el servidor/cliente de Gazebo.
        PythonLaunchDescriptionSource([                       # Especifica la fuente (un archivo .py) del launch a incluir.
            os.path.join(                                     # Une partes de la ruta de forma portable.
                get_package_share_directory("ros_gz_sim"),    # Busca el 'share' del paquete ros_gz_sim.
                "launch"                                      # Carpeta donde están los launch files de ese paquete.
            ),
            "/gz_sim.launch.py"                               # Nombre del launch que inicia Gazebo (unido como string final).
        ]),
        launch_arguments=[
            ("gz_args",[" -v 4 -r  empty.sdf"])               # Pasa argumentos a Gazebo: verbosidad -v 4, modo -r (run), mundo 'empty.sdf'.
        ]
    )

    gz_spawn_entity = Node(                                   # Nodo que crea/spawnea una entidad en Gazebo a partir del tópico /robot_description.
        package="ros_gz_sim",                                 # Paquete que contiene el ejecutable 'create'.
        executable="create",                                  # Ejecutable CLI que realiza el spawn del modelo.
        output="screen",                                      # Muestra salida en pantalla para depuración.
        arguments=["-topic", "robot_description",             # Toma el URDF desde el tópico 'robot_description'.
                    "-name","arduinobot"                      # Asigna el nombre de la entidad en el mundo de Gazebo.
        ]
    )

    gz_ros2_bridge= Node(                                     # Puente ROS <-> Gazebo para tópicos/mensajes específicos.
        package="ros_gz_bridge",                              # Paquete del bridge.
        executable="parameter_bridge",                        # Ejecutable que crea el bridge de parámetros/tópicos.
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",   # Mapea /clock desde Gazebo hacia ROS 2 (gz -> rosgraph_msgs/Clock).
            # Bridge de la imagen de la cámara hacia sensor_msgs/Image.
            "/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            # Bridge de la info intrínseca/extrínseca hacia sensor_msgs/CameraInfo.
            "/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"
        ],
        output="screen",
    )

    model_arg = DeclareLaunchArgument(                        # Declara un argumento de launch llamado "model".
        name="model",                                         # Nombre del argumento: se referenciará como LaunchConfiguration("model").
        default_value=os.path.join(                           # Valor por defecto si el usuario no pasa otro: se construye una ruta válida.
            get_package_share_directory("arduinobot_description"),  # Busca el directorio 'share' del paquete 'arduinobot_description'.
            "urdf",                                           # Entra a la carpeta 'urdf' dentro del paquete.
            "arduinobot.urdf.xacro"                           # Archivo Xacro del robot que se convertirá a URDF.
        ),
        description="Absolute path to the robot URDF file"    # Texto descriptivo que aparece en ayuda/errores del launch.
    )

    robot_description = ParameterValue(                       # Define el valor del parámetro 'robot_description' que consumirá el nodo.
        Command(                                              # Ejecuta un comando del sistema en tiempo de lanzamiento y toma su salida.
            [                                                 # La salida de este comando será el contenido URDF generado por xacro.
                "xacro ",                                     # Comando 'xacro' con un espacio final para evitar que se pegue a la ruta.
                LaunchConfiguration("model"),                 # Sustitución: toma el valor actual del argumento 'model' (ruta del Xacro).
                " is_ignition:=",                              
                is_ignition                                   # Variable que indica si se tiene o no ros2 humble
            ]
        ),
        value_type=str                                        # Indica que el resultado del comando se tratará como cadena (string).
    )

    robot_state_publisher_node = Node(                        # Define el nodo que publicará el URDF en /robot_description y TF.
        package="robot_state_publisher",                      # Paquete donde vive el ejecutable.
        executable="robot_state_publisher",                   # Nombre del ejecutable a lanzar.
        parameters=[{"robot_description": robot_description,  # Pasa el parámetro 'robot_description' ya resuelto (URDF plano).
                "use_sim_time" : True                         # Usa el reloj simulado (sincroniza con /clock de Gazebo).
            }]
    )


    return LaunchDescription([                                # Construye y devuelve la descripción completa del lanzamiento.
        model_arg,                                            # 1) Argumento para escoger el archivo Xacro/URDF.
        gazebo_resource_path,                                 # 2) Variable de entorno para que Gazebo encuentre recursos.
        robot_state_publisher_node,                           # 3) Publicador del URDF y TF.
        gazebo,                                               # 4) Lanzador de Gazebo (servidor/cliente) vía ros_gz_sim.
        gz_spawn_entity,                                      # 5) Nodo que crea la entidad del robot en el mundo.
        gz_ros2_bridge                                        # 6) Bridge de /clock (y otros que quieras añadir) entre Gazebo y ROS.
    ])
