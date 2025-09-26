
# Clase 88 — `task_server` (ROS 2 + MoveIt)

## Objetivo de la clase
En esta sesión se creó y documentó un **servidor de acciones** (Task Server) para mover el robot a **posiciones predefinidas** (home, picking y rest) usando **MoveIt**. El servidor se integra en el nuevo paquete **`arduinobot_remote`**, expone la acción `ArduinobotTask` y se registró como **componente rclcpp** para poder lanzarlo como ejecutable.

---

## Resumen del flujo (según la TRANSCRIPTION)
1. Crear el paquete `arduinobot_remote` con tipo de construcción **ament_cmake**.  
2. Generar el archivo fuente **C++** del servidor: `src/task_server.cpp` (derivado del simple action server), cambiar **clase**, **namespace**, **nombre del servidor de acción** y **interfaz** hacia `ArduinobotTask`.  
3. Implementar la lógica con **MoveIt**: instanciar `MoveGroupInterface` para `arm` y `gripper`, fijar **start state**, establecer **targets** según el `task_number`, **planificar** y **mover**.  
4. Declarar dependencias en **CMakeLists.txt** y **package.xml**, registrar el nodo como componente y **instalar** el target.  
5. Compilar el workspace y verificar que el nodo se construye y ejecuta sin errores.

---

## Archivos y cambios

### 1) `task_server.cpp`
**Qué hace:** implementa el **servidor de acciones** `TaskServer` dentro del namespace `arduinobot_remote`. Crea el servidor de la acción `arduinobot_msgs::action::ArduinobotTask`, acepta metas y ejecuta movimientos con MoveIt para **brazo** y **pinza**.

**Puntos clave de la implementación:**
- Crea el servidor con `rclcpp_action::create_server` y callbacks `goal`, `cancel`, `accepted`.
- En `acceptedCallback` inicia `execute()` en un hilo.
- En `execute()`:
  - Inicializa `MoveGroupInterface` para `"arm"` y `"gripper"`.
  - Según `task_number` {0,1,2} asigna `arm_joint_goal_` y `gripper_joint_goal_`.
  - Fija el **estado inicial** con `setStartState(*getCurrentState())` para ambos grupos.
  - Verifica límites con `setJointValueTarget(...)`.
  - Planifica con `plan(...)` y, si ambos planes tienen **SUCCESS**, ejecuta `move()`.
  - Devuelve `result->success = true` con `goal_handle->succeed(result)`.

> Fuente: archivo subido por el usuario `task_server.cpp`. fileciteturn0file2

---

### 2) `CMakeLists.txt`
**Qué hace:** declara y construye la librería/componente del servidor de acciones y registra el nodo como **componente rclcpp** con ejecutable asociado.

**Puntos clave:**
- Busca dependencias: `rclcpp`, `rclcpp_action`, `rclcpp_components`, `arduinobot_msgs`, `moveit_ros_planning_interface`, etc.
- Crea la **librería compartida**:
  ```cmake
  add_library(task_server SHARED src/task_server.cpp)
  ament_target_dependencies(task_server ... moveit_ros_planning_interface)
  rclcpp_components_register_node(task_server
    PLUGIN "arduinobot_remote::TaskServer"
    EXECUTABLE task_server_node)
  ```
- Instala el target en `lib/` y el ejecutable `task_server_node`.

> Fuente: archivo subido por el usuario `CMakeLists.txt`. fileciteturn0file0

---

### 3) `package.xml`
**Qué hace:** declara metadatos del paquete y **dependencias** de compilación/ejecución.

**Puntos clave:**
- `buildtool_depend`: `ament_cmake`.
- `build_depend`: `ament_cmake_python` (si también instala un módulo Python).
- `depend`: `rclpy`, `arduinobot_msgs`, `rclcpp`, `rclcpp_action`, `moveit_ros_planning_interface`.
- **Ojo**: aparece `rclpp_components` (doble p) — debe ser **`rclcpp_components`** para coincidir con CMake y evitar errores de dependencias.
- `exec_depend condition="$ROS_DISTRO >= iron">moveit_py</exec_depend>`: dependencia condicional para entornos Iron o superiores (no afecta el binario C++ pero puede acompañar utilidades Python).

> Fuente: archivo subido por el usuario `package.xml`. fileciteturn0file1

---

## Pseudocódigo — `task_server.py` (equivalente funcional)
> Nota: Aunque en esta clase trabajamos la versión **C++** (`task_server.cpp`), este pseudocódigo refleja la **misma lógica** si se implementara en **Python** usando `rclpy` y `moveit_py`/`MoveGroupCommander`.

```text
iniciar_nodo("task_server")

crear_action_server(
  tipo_accion = ArduinobotTask,
  nombre = "task_server",
  goal_cb = on_goal,
  cancel_cb = on_cancel,
  accepted_cb = on_accepted
)

func on_goal(uuid, goal):
  log("llegó meta con task_number: ", goal.task_number)
  devolver ACEPTAR_Y_EJECUTAR

func on_cancel(goal_handle):
  log("cancel solicitado")
  si arm_move_group existe: arm_move_group.stop()
  si gripper_move_group existe: gripper_move_group.stop()
  devolver ACEPTAR

func on_accepted(goal_handle):
  lanzar_hilo( execute(goal_handle) )

func execute(goal_handle):
  inicializar arm_move_group = MoveGroup("arm")
  inicializar gripper_move_group = MoveGroup("gripper")

  segun goal_handle.goal.task_number:
    caso 0: arm=[0,0,0]; gripper=[-0.7, 0.7]
    caso 1: arm=[-1.14,-0.6,-0.07]; gripper=[0,0]
    caso 2: arm=[-1.57,0,-0.9]; gripper=[0,0]
    otro: log_error("task inválido"); terminar

  arm_move_group.set_start_state(arm_move_group.get_current_state())
  gripper_move_group.set_start_state(gripper_move_group.get_current_state())

  ok_arm = arm_move_group.set_joint_value_target(arm)
  ok_grip = gripper_move_group.set_joint_value_target(gripper)
  si !(ok_arm y ok_grip): log_error("fuera de límites"); terminar

  plan_arm = arm_move_group.plan()
  plan_grip = gripper_move_group.plan()
  si plan_arm.exitoso y plan_grip.exitoso:
    arm_move_group.go(wait=True)
    gripper_move_group.go(wait=True)
  si no: log_error("falló planificación"); terminar

  resultado.success = True
  goal_handle.succeed(resultado)
```

---

## Comandos típicos usados
- Construcción del workspace:
  ```bash
  colcon build --packages-select arduinobot_remote
  . install/setup.bash
  ```
- Listar acciones y ejecutar el nodo (como componente/ejecutable):
  ```bash
  ros2 run arduinobot_remote task_server_node
  ros2 action list
  ros2 action info /task_server
  ```

---

## Errores comunes y cómo evitarlos
- **Nombre de dependencias**: en `package.xml` usar `rclcpp_components` (no `rclpp_components`).  
- **Coincidencia de nombres** entre `add_library(...)`, `rclcpp_components_register_node(...)` e `install(TARGETS ...)`.  
- **Grupos MoveIt**: que existan en la configuración (`arm`, `gripper`).  
- **Estados iniciales**: llamar `setStartState(*getCurrentState())` para **ambos** grupos antes de planificar.
