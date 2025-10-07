# Clase 95 - Lanzamiento de la simulación del robot (Arduinobot Bringup)

En esta lección se integraron todas las funcionalidades desarrolladas a lo largo del proyecto del **Arduinobot**, creando un único archivo de lanzamiento que permite iniciar simultáneamente la simulación, el control, el planificador de trayectorias (MoveIt) y la interfaz remota con **Amazon Alexa**. Este proceso se realizó dentro de un nuevo paquete llamado `arduinobot_bringup`.

---

## 🧩 Archivos del paquete

### 1. **simulated_robot_launch.py**

#### Descripción

Este archivo es el corazón del paquete `arduinobot_bringup`. Permite ejecutar todas las funcionalidades del robot en simulación con un solo comando. Su propósito es incluir los distintos archivos de lanzamiento existentes de otros paquetes (Gazebo, controlador, MoveIt y la interfaz remota).

#### Estructura y librerías utilizadas

* **`launch`** y **`launch_ros`**: librerías de ROS2 que permiten definir descripciones de lanzamiento y acciones de ejecución.
* **`IncludeLaunchDescription`**: se utiliza para incluir otros archivos de lanzamiento desde distintos paquetes.
* **`get_package_share_directory`**: localiza la carpeta de recursos compartidos de un paquete específico.
* **`os.path.join`**: crea rutas de archivos dinámicas para acceder a los launch files dentro de sus respectivos paquetes.

#### Pseudocódigo del script

```
INICIO función generate_launch_description
    IMPORTAR librerías necesarias (launch, IncludeLaunchDescription, os, get_package_share_directory)

    CREAR variable gazebo_launch → incluir archivo 'gazebo.launch.py' desde paquete 'arduinobot_description'

    CREAR variable controller_launch → incluir archivo 'controller.launch.py' desde paquete 'arduinobot_controller'
        PASAR argumento {'is_sim': 'True'}

    CREAR variable moveit_launch → incluir archivo 'moveit.launch.py' desde paquete 'arduinobot_moveit'
        PASAR argumento {'is_sim': 'True'}

    CREAR variable remote_interface_launch → incluir archivo 'remote_interface.launch.py' desde paquete 'arduinobot_remote'

    RETORNAR LaunchDescription con lista de los cuatro lanzamientos:
        [gazebo_launch, controller_launch, moveit_launch, remote_interface_launch]
FIN FUNCIÓN
```

#### Funcionamiento

Al ejecutar el archivo `simulated_robot_launch.py`, ROS2 inicia automáticamente:

1. **Gazebo** – para simular el entorno físico del robot.
2. **Controller** – para activar los controladores de las articulaciones y del efector final.
3. **MoveIt** – para planificar y ejecutar trayectorias de movimiento.
4. **Interfaz remota** – que conecta el sistema con **Alexa** a través de la API de Ngrok.

---

### 2. **CMakeLists.txt**

#### Descripción

En este archivo se agregaron las instrucciones necesarias para que el directorio `launch` del paquete `arduinobot_bringup` sea instalado correctamente durante la compilación.

#### Sección relevante añadida

```cmake
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)
```

#### Explicación

* `DIRECTORY launch` indica que se copiará la carpeta `launch` durante la instalación.
* `DESTINATION share/${PROJECT_NAME}` especifica el destino dentro del espacio de instalación.
* `${PROJECT_NAME}` es una variable que contiene el nombre del paquete (`arduinobot_bringup`).

Con esto, el archivo `simulated_robot_launch.py` puede ser ejecutado globalmente desde ROS2 mediante el comando `ros2 launch`.

---

### 3. **package.xml**

#### Descripción

Este archivo define las dependencias del paquete y su metainformación básica.

#### Cambios realizados

Se agregó la dependencia necesaria para los archivos de lanzamiento:

```xml
<exec_depend>ros2launch</exec_depend>
```

#### Explicación

* Permite que el paquete utilice el sistema de lanzamiento de ROS2.
* Es esencial para ejecutar archivos `.launch.py` desde este paquete.

---

## 💻 Comandos ejecutados en los terminales

### **Terminal 1 – Creación y compilación del paquete**

```bash
ros2 pkg create --build-type ament_cmake arduinobot_bringup
colcon build
```

**Objetivo:**

* Crear un nuevo paquete vacío.
* Compilar todo el workspace, incluyendo el nuevo paquete y registrar sus dependencias.

---

### **Terminal 2 – Ejecución de la simulación completa**

```bash
ros2 launch arduinobot_bringup simulated_robot_launch.py
```

**Objetivo:**

* Ejecutar de forma conjunta todas las funcionalidades: Gazebo, controladores, MoveIt y Alexa.
* Ver en acción la coordinación entre los distintos nodos ROS2.

---

### **Terminal 3 – Conexión con Alexa mediante Ngrok**

```bash
ngrok http 5000
```

**Objetivo:**

* Crear un túnel HTTPS público que conecte el servidor local Flask (que corre la API de Alexa) con la nube.
* Proporcionar una URL que se introduce en el **Alexa Developer Console** dentro de la sección **Endpoint**.

---

## ☁️ Integración con Amazon Alexa

En la **Alexa Developer Console**, se configuraron los siguientes *intents* para interactuar con el Arduinobot:

| Intent          | Comandos de activación          | Acción ejecutada                                               |
| --------------- | ------------------------------- | -------------------------------------------------------------- |
| **Invocation**  | “Activate Arduinobot”           | Inicia la sesión de control por voz                            |
| **WakeIntent**  | “Wake up”, “Activate the robot” | Abre el gripper y activa el robot                              |
| **PickIntent**  | “Pick the pen”, “Grab the pen”  | Mueve el brazo hacia la posición de agarre y cierra el gripper |
| **SleepIntent** | “Turn off the robot”, “Rest”    | Envía el robot a la posición de descanso                       |

Cuando Alexa recibe un comando, lo reenvía al servidor Flask a través del túnel Ngrok. Luego, el nodo `alexa_interface.py` interpreta la intención y envía el mensaje correspondiente al **Task Server** del robot. Este a su vez ordena a **MoveIt** ejecutar el movimiento en la simulación de **Gazebo**.

---

## 🤖 Resultado final

Con el lanzamiento `simulated_robot_launch.py` se logra iniciar **toda la arquitectura funcional del Arduinobot** con un solo comando. La simulación en Gazebo reacciona directamente a las órdenes de voz enviadas desde Alexa, demostrando la integración exitosa entre:

* **Simulación física (Gazebo)**
* **Control del robot (ROS2 Control)**
* **Planificación de trayectorias (MoveIt)**
* **Interfaz de voz (Alexa + Flask + Ngrok)**

---

### 🏁 Conclusión

Esta clase marcó el cierre de la fase de simulación del curso. A partir de este punto, el siguiente paso será adaptar todo el sistema para funcionar con el robot físico, manteniendo la misma estructura modular y escalable implementada con ROS2.
