# Clase 89 — Interfaz Remota (Task Server) con MoveIt2

Este documento resume la **lección de arranque del Task Server** y su interacción con la API de MoveIt2 para mover el robot a posiciones predefinidas. Incluye:
- Explicación del *launch* `remote_interface.launch.py` con **pseudocódigo** de su funcionamiento.
- Cambios realizados en **CMakeLists.txt** y **package.xml** del paquete `arduinobot_remote`.
- **Comandos usados en cada terminal** (T1–T5), con propósito y efecto.

> Paquete: `arduinobot_remote`  
> Launch principal: `arduinobot_remote/launch/remote_interface.launch.py`

---

## 1) `remote_interface.launch.py` — Explicación y Pseudocódigo

Este *launch* construye la configuración de MoveIt2 (URDF/SRDF/controladores) y **lanza condicionalmente** el *Task Server* en **Python** o en **C++** según el argumento `use_python`. Además, expone el argumento `is_sim` para distinguir entorno simulado vs. real y fija `use_sim_time` para la simulación.

### Puntos clave
- **Argumentos de lanzamiento**
  - `is_sim` (por defecto: `True`): marca si se usa simulación.
  - `use_python` (por defecto: `False`): si `True` inicia el *Task Server* en Python; si `False`, en C++.
- **MoveItConfigsBuilder**
  - Carga el **URDF/Xacro** desde `arduinobot_description`.
  - Asocia el **SRDF** y `moveit_controllers.yaml` para ejecución de trayectorias.
  - Carga un YAML de API de *planning* (`planning_python_api.yaml`) para la integración.
- **Nodos condicionales**
  - `task_server.py` (Python) con `IfCondition(use_python)`.
  - `task_server_node` (C++) con `UnlessCondition(use_python)`.
  - Ambos reciben `moveit_config.to_dict()` y `{"use_sim_time": True}`.

### Pseudocódigo (alto nivel)
```text
def generate_launch_description():
    declarar argumento is_sim = True
    declarar argumento use_python = False

    is_sim  = LaunchConfiguration("is_sim")
    use_py  = LaunchConfiguration("use_python")

    moveit_config = construir MoveItConfigs para "arduinobot":
        - robot_description: URDF principal (xacro) de arduinobot_description/urdf/arduinobot.urdf.xacro
        - robot_description_semantic: config/arduinobot.srdf
        - trajectory_execution: config/moveit_controllers.yaml
        - moveit_cpp (o API de planificación): config/planning_python_api.yaml

    node_py = Node(
        package="arduinobot_remote",
        executable="task_server.py",
        condition=IfCondition(use_py),
        parameters=[ moveit_config.to_dict(), {"use_sim_time": True} ]
    )

    node_cpp = Node(
        package="arduinobot_remote",
        executable="task_server_node",
        condition=UnlessCondition(use_py),
        parameters=[ moveit_config.to_dict(), {"use_sim_time": True} ]
    )

    return LaunchDescription([ use_python_arg, is_sim_arg, node_cpp, node_py ])
```
**Efecto:** al lanzar `ros2 launch arduinobot_remote remote_interface.launch.py`, se inicializa el *Task Server* (Python o C++). A partir de ahí, podemos enviar **goals** de acción a `/task_server` para ejecutar tareas (p. ej., *home*, *pick*, *rest*).

> **Nota práctica:** para forzar Python, añadir `use_python:=True`; para C++, dejar el valor por defecto `False`.

---

## 2) Cambios en `CMakeLists.txt` (paquete `arduinobot_remote`)

- Se incluyen dependencias de compilación/ejecución: `ament_cmake`, `ament_cmake_python`, `rclcpp`, `rclcpp_action`, `rclcpp_components`, `arduinobot_msgs`, `moveit_ros_planning_interface`.
- **Instalación Python del paquete**: `ament_python_install_package(${PROJECT_NAME})` para exponer `task_server.py` como ejecutable del paquete.
- **Servidor de acciones en C++**:
  - Se crea la librería compartida `task_server` desde `src/task_server.cpp`.
  - Se registran dependencias (`ament_target_dependencies`).
  - Se registra como **componente** con `rclcpp_components_register_node`, creando el ejecutable `task_server_node`.
- **Instalación de artefactos**:
  - `install(TARGETS task_server ...)` para la lib/ejecutable.
  - `INSTALL(PROGRAMS ${PROJECT_NAME}/task_server.py DESTINATION lib/${PROJECT_NAME})` para publicar el ejecutable Python.
  - `install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})` para distribuir el directorio `launch/`.

---

## 3) Cambios en `package.xml`

- **Herramientas de construcción**: `ament_cmake` (buildtool), `ament_cmake_python` (build_depend).
- **Dependencias de ejecución/compilación**:
  - `rclpy`, `arduinobot_msgs`, `rclcpp`, `rclcpp_action`, `rclcpp_components`.
  - `moveit_ros_planning_interface` para integración con MoveIt2 en C++.
  - `ros2launch` para poder usar archivos `launch`.
  - `arduinobot_moveit` como dependencia de configuración MoveIt.
  - (Opcional/condicional) `moveit_py` cuando el *distro* ≥ `iron`.
- **Exporta**: `<build_type>ament_cmake</build_type>`.

---

## 4) Comandos y flujo en las terminales

> Antes de lanzar, asegúrate de estar en `arduinobot_ws/` y de **compilar** el workspace.

### **T1 — Simulación (Ignition Gazebo)**
```bash
colcon build
. install/setup.bash
ros2 launch arduinobot_description gazebo.launch.py
```
- **Propósito:** compilar, cargar el *overlay* y arrancar el **mundo de simulación** con el robot (`robot_state_publisher`, sensores/TF, etc.).
- **Efecto:** aparece el robot en Gazebo listo para ser controlado.

### **T2 — Controladores (ros2_control)**
```bash
. install/setup.bash
ros2 launch arduinobot_controller controller.launch.py
```
- **Propósito:** **configurar y activar** los controladores del brazo y la pinza (`arm_controller`, `gripper_controller`).  
- **Efecto:** los controladores entran en `active` y aceptan trayectorias.

### **T3 — MoveIt2**
```bash
. install/setup.bash
ros2 launch arduinobot_moveit moveit_launch.py
```
- **Propósito:** inicializar **MoveIt2** (planning scene, pipeline, *move_group*, etc.).
- **Efecto:** el sistema de planificación queda operativo (con o sin GUI).

### **T4 — Task Server (Interfaz remota)**
```bash
. install/setup.bash
ros2 launch arduinobot_remote remote_interface.launch.py            # C++ por defecto
# ó para Python:
# ros2 launch arduinobot_remote remote_interface.launch.py use_python:=True
```
- **Propósito:** lanzar el **Task Server** (C++ o Python) que expone la acción `/task_server` y utiliza la configuración de MoveIt.
- **Efecto:** queda escuchando *goals* para ejecutar tareas predefinidas.

### **T5 — Cliente de acciones (pruebas de tareas)**
```bash
ros2 action list
. install/setup.bash
ros2 action send_goal /task_server arduinobot_msgs/action/ArduinobotTask "task_number: 1"
ros2 action send_goal /task_server arduinobot_msgs/action/ArduinobotTask "task_number: 0"
ros2 action send_goal /task_server arduinobot_msgs/action/ArduinobotTask "task_number: 2"
```
- **Propósito:** verificar que `/task_server` está disponible y **enviar metas** para ejecutar tareas numéricas.
- **Efecto esperado:**
  - `task_number: 1` → el robot va a una **posición de pick**.
  - `task_number: 0` → el robot vuelve a **home**.
  - `task_number: 2` → el robot va a **rest**.
- **Resultado:** cada *goal* finaliza con `SUCCEEDED` y `success: 1` si se planifica/ejecuta correctamente.

---

## 5) Consejos de uso y depuración rápida
- Si `/task_server` **no aparece** en `ros2 action list`, revisa que T4 esté activo y sin errores.
- Si los controladores no aceptan trayectorias, confirma que T2 reporta **controladores activos**.
- Para cambiar entre Python/C++, usa `use_python:=True/False` en T4.
- Para simulación en tiempo de simulación, mantener `use_sim_time: True` en los nodos del Task Server.

---

## 6) Estructura mínima del paquete (referencial)
```
arduinobot_remote/
├── launch/
│   └── remote_interface.launch.py
├── src/
│   └── task_server.cpp
├── arduinobot_remote/
│   └── task_server.py
├── CMakeLists.txt
└── package.xml
```

---

## 7) Créditos y contexto
Este README compila los pasos de la clase donde se inicia el **Task Server** y se comprueba su integración con **MoveIt2** para ejecutar tareas simples vía **acciones ROS 2** en simulación.