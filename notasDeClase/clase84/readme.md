# Clase 84 — MoveIt 2 API en C++ (`simple_moveit_interface.cpp`)

> **Objetivo:** Implementar un nodo **C++** mínimo que use la **MoveIt 2 API (MoveGroupInterface)** para planear y ejecutar trayectorias en los *move groups* `arm` y `gripper` del **arduinobot**. Se dejaron listos los cambios en **CMakeLists.txt** y **package.xml** para compilar/instalar el ejecutable.

---

## Comandos usados (Terminal)
```bash
colcon build
```
> Compila el *workspace* con el nuevo ejecutable de C++.

---

## Archivos trabajados

### 1) `src/simple_moveit_interface.cpp`
**Qué hace**
- Crea un nodo ROS 2, instancia dos `MoveGroupInterface` para `arm` y `gripper`.
- Define metas de juntas: brazo `[1.57, 0.0, 0.0]` (≈90° en la base) y gripper `[-0.7, 0.7]` (abrir pinza).
- Verifica que las metas estén dentro de límites articulares.
- Planifica trayectorias para ambos grupos y, si ambas son **SUCCESS**, ejecuta los movimientos.

**Pseudocódigo**
```pseudocode
func move_robot(node):
    arm    ← MoveGroupInterface(node, "arm")
    gripper← MoveGroupInterface(node, "gripper")

    arm_goal     ← [1.57, 0.0, 0.0]
    gripper_goal ← [-0.7, 0.7]

    arm_ok     ← arm.setJointValueTarget(arm_goal)
    gripper_ok ← gripper.setJointValueTarget(gripper_goal)
    if not (arm_ok and gripper_ok):
        log_warn("Target joint position were outside of limits!")
        return

    arm_plan     ← MoveGroupInterface::Plan()
    gripper_plan ← MoveGroupInterface::Plan()

    arm_planned_ok     ← (arm.plan(arm_plan) == SUCCESS)
    gripper_planned_ok ← (gripper.plan(gripper_plan) == SUCCESS)

    if arm_planned_ok and gripper_planned_ok:
        arm.move()
        gripper.move()
    else:
        log_error("One or more planners failed")
        return

func main(argc, argv):
    rclcpp.init(argc, argv)
    node ← rclcpp::Node::make_shared("simple_moveit_interface")
    move_robot(node)
    rclcpp.shutdown()
    return 0
```

**Notas**
- `setJointValueTarget(...)` ya valida límites de las juntas.
- `move()` ejecuta el último plan generado internamente por cada grupo.
- Manejo básico de errores con `RCLCPP_WARN/ERROR`.

---

### 2) `CMakeLists.txt`
**Cambios realizados / verificados**
- **Dependencias**: se añadió `find_package(moveit_ros_planning_interface REQUIRED)` para poder usar `MoveGroupInterface`.
- **Ejecutable**: se registró `add_executable(simple_moveit_interface src/simple_moveit_interface.cpp)`.
- **Vinculación**: `ament_target_dependencies(simple_moveit_interface rclcpp moveit_ros_planning_interface)`.
- **Instalación**: el ejecutable se instala bajo `lib/${PROJECT_NAME}`.

Estos cambios permiten compilar y ejecutar el nodo C++ con MoveIt 2. fileciteturn1file0

---

### 3) `package.xml`
**Cambios realizados / verificados**
- Se declararon dependencias de ejecución/compilación para `moveit_ros_planning_interface`, además de las ya existentes (`rclcpp`, `std_msgs`, `rcl_interfaces`, `arduinobot_msgs`, `rclcpp_action`, `rclcpp_components`).  
- Se mantiene `ament_cmake` como *buildtool* y `build_type` en `ament_cmake`.

Estas entradas garantizan que `rosdep` y `colcon` resuelvan MoveIt al compilar. fileciteturn1file1

---

## Ejecución (sugerida)
1) Compilar y *source*:
```bash
colcon build --symlink-install
. install/setup.bash
```
2) Lanzar tu stack de simulación/MoveIt (controladores, descripción, RViz).  
3) Ejecutar el binario directamente (si no usas un `launch`):
```bash
ros2 run arduinobot_cpp_examples simple_moveit_interface
```

> Si alguna planificación falla, revisa límites articulares, *start state*, colisiones o que los controladores estén cargados.

---

## Lecciones clave
- `MoveGroupInterface` permite controlar grupos independientes (brazo/pinza).
- Validar límites antes de planificar evita fallos de planificador.
- Separar **configuración (CMake/package.xml)** del **código** simplifica escalar a más ejemplos.