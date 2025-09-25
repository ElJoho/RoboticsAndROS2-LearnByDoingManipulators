# Clase 87 — `task_server.py` (Action Server con MoveItPy)

Esta clase crea el **servidor de tareas** del Arduinobot como un **Action Server** en ROS 2 que expone una acción `ArduinobotTask` para mover el robot a posiciones predefinidas mediante **MoveItPy**. Además, se estructura el paquete `arduinobot_remote` (tipo `ament_cmake`) para instalar el nodo Python y se añade la nueva interfaz de acción al paquete `arduinobot_msgs`.

---

## Terminales 

**Terminal 1**  
```bash
cd arduinobot_ws/src/
ros2 pkg create --build-type ament_cmake arduinobot_remote
cd ..
colcon build
```
Salida relevante: se generan los archivos base del paquete y la compilación finaliza con éxito (8 paquetes terminados).

---

## Archivos del proyecto (por secciones)

### 1) `arduinobot_remote/arduinobot_remote/__init__.py`
Archivo inicializador del paquete Python. Permite que `ament_python_install_package(${PROJECT_NAME})` trate el directorio como módulo instalable. Suele estar vacío, pero es necesario para empaquetado e importación.

---

### 2) `arduinobot_remote/arduinobot_remote/task_server.py`  — Nodo Action Server
Nodo Python que implementa el **servidor de acciones** para `ArduinobotTask`. Crea dos *planning components* (`arm` y `gripper`) de MoveItPy, interpreta el `task_number` de la meta y planifica/ejecuta la trayectoria correspondiente (brazo y garra).

**Pseudocódigo del script**
```
INICIO
  crear nodo ROS2 "task_server"
  crear ActionServer con tipo de acción ArduinobotTask y nombre "task_server"
  inicializar MoveItPy("moveit_py")
  obtener planning components: arm, gripper

  función goalCallback(goal_handle):
    loggear task_number recibido
    crear RobotState para arm y gripper (desde get_robot_model())

    arm_joint_goal <- []
    gripper_joint_goal <- []

    si task_number == 0:
      arm_joint_goal <- [0.0, 0.0, 0.0]
      gripper_joint_goal <- [-0.7, 0.7]
    sino si task_number == 1:
      arm_joint_goal <- [-1.14, -0.6, -0.07]
      gripper_joint_goal <- [0.0, 0.0]
    sino si task_number == 2:
      arm_joint_goal <- [-1.57, 0.0, -0.9]
      gripper_joint_goal <- [0.0, 0.0]
    en otro caso:
      loggear error y retornar

    set_joint_group_positions(arm, arm_joint_goal) en arm_state
    set_joint_group_positions(gripper, gripper_joint_goal) en gripper_state

    arm.set_start_state_to_current_state()
    gripper.set_start_state_to_current_state()

    arm.set_goal_state(arm_state)
    gripper.set_goal_state(gripper_state)

    arm_plan_result <- arm.plan()
    gripper_plan_result <- gripper.plan()

    si ambos planes son válidos:
      ejecutar trayectorias en MoveItPy
    sino:
      loggear "One or more planners failed"

    marcar objetivo como succeed()
    result.success <- True
    devolver result
FIN
```

**Notas clave**
- Usa `MoveItPy` y `RobotState`; configura metas por grupos de articulaciones `arm` y `gripper`.
- Verifica resultados de planificación antes de ejecutar.
- Devuelve `success=True` al cliente tras ejecución.

---

### 3) `arduinobot_remote/action/ArduinobotTask.action`  — Interfaz de la acción
Interfaz de acción con:
- **Goal:** `int32 task_number`
- **Result:** `bool success`
- **Feedback:** (opcional para esta clase) un `int32 percentage` sugerido en la explicación.

La acción se **declara/instala** en `arduinobot_msgs` para que pueda ser usada por el nodo Python.

---

### 4) `arduinobot_msgs/CMakeLists.txt`
Se registran las interfaces (srv y action), incluyendo **`action/ArduinobotTask.action`** para su generación con `rosidl_generate_interfaces(...)`.  
Puntos clave:
- `find_package(rosidl_default_generators REQUIRED)`
- Listado de `.srv` y `.action` en `rosidl_generate_interfaces(...)`  
Esto habilita la generación de mensajes y tipos de la acción.

---

### 5) `arduinobot_remote/CMakeLists.txt`
Define el paquete `arduinobot_remote` y sus dependencias para nodos Python:
- `find_package(ament_cmake_python REQUIRED)`
- `find_package(rclpy REQUIRED)`
- `find_package(arduinobot_msgs REQUIRED)`
- **Instalación**:
  - `ament_python_install_package(${PROJECT_NAME})`
  - `INSTALL(PROGRAMS ${PROJECT_NAME}/task_server.py DESTINATION lib/${PROJECT_NAME})`

Esto permite instalar el módulo Python y el ejecutable `task_server.py` bajo `lib/<paquete>` para ejecución con `ros2 run` (vía un *entry point* simple basado en ruta instalada).

---

### 6) `arduinobot_remote/package.xml`
Declara dependencias de construcción/ejecución:
- `ament_cmake`, `ament_cmake_python`
- `rclpy`, `arduinobot_msgs`
- Dependencia condicional: `exec_depend condition="$ROS_DISTRO >= iron">moveit_py</exec_depend`  
  (Se usa `MoveItPy` en distros donde está disponible).

---
