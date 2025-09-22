# Clase 83 — `simple_moveit_interface` (ROS 2 + MoveItPy)

> **Objetivo:** Crear y lanzar un nodo Python que use la API V2 de MoveIt (MoveItPy) para planear y ejecutar trayectorias en el **arduinobot** (grupo `arm` y `gripper`) a una posición deseada. Requiere **ROS 2 ≥ Iron** para disponer de `moveit_py`.

---

## Archivos trabajados

### 1) `arduinobot_py_examples/simple_moveit_interface.py`
**Qué hace**  
Implementa un nodo mínimo en Python que:  
- Inicializa ROS 2, construye un objeto `MoveItPy`, obtiene los **planning components** de los grupos `arm` y `gripper`, define **estado inicial** y **estado objetivo** de cada grupo, **planea** y, si los planes son válidos, **ejecuta** las trayectorias en el robot simulado.  
- El brazo se fija a `[1.57, 0.0, 0.0]` rad (gira la base ~90°) y el gripper a `[-0.7, 0.7]` rad (abrir pinza). Si algún plan falla, registra un error en consola.

**Pseudocódigo**
```
func move_robot():
    crear objeto moveit = MoveItPy(node_name="moveit_py")
    arm = moveit.get_planning_component("arm")
    gripper = moveit.get_planning_component("gripper")

    arm_state = RobotState(moveit.get_robot_model())
    gripper_state = RobotState(moveit.get_robot_model())

    arm_state.set_joint_group_positions("arm", [1.57, 0.0, 0.0])
    gripper_state.set_joint_group_positions("gripper", [-0.7, 0.7])

    arm.set_start_state_to_current_state()
    gripper.set_start_state_to_current_state()

    arm.set_goal_state(robot_state=arm_state)
    gripper.set_goal_state(robot_state=gripper_state)

    arm_plan   = arm.plan()
    grip_plan  = gripper.plan()

    if arm_plan y grip_plan:
        moveit.execute(arm_plan.trajectory, controllers=[])
        moveit.execute(grip_plan.trajectory, controllers=[])
    else:
        logger.error("One or more planners failed!")

func main():
    rclpy.init()
    move_robot()
    rclpy.shutdown()
```

---

### 2) `arduinobot_py_examples/launch/simple_moveit_interface.launch.py`
**Qué hace**  
- Construye la **configuración MoveIt** con `MoveItConfigsBuilder` indicando el modelo `arduinobot` (URDF/Xacro, SRDF, controladores).  
- Añade la configuración **MoveItCpp** para la API Python apuntando a `config/planning_python_api.yaml`.  
- Lanza el ejecutable `simple_moveit_interface` del paquete `arduinobot_py_examples` con `use_sim_time=True`.

**Pseudocódigo**
```
func generate_launch_description():
    moveit_config = MoveItConfigsBuilder("arduinobot", package_name="arduinobot_moveit")
        .robot_description(file_path=share("arduinobot_description")/urdf/arduinobot.urdf.xacro)
        .robot_description_semantic(file_path="config/arduinobot.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .moveit_cpp(file_path="config/planning_python_api.yaml")
        .to_moveit_configs()

    node = Node(
        package="arduinobot_py_examples",
        executable="simple_moveit_interface",
        parameters=[moveit_config.to_dict(), {"use_sim_time": True}]
    )

    return LaunchDescription([node])
```

---

### 3) `arduinobot_moveit/config/planning_python_api.yaml`
**Qué hace / parámetros relevantes**
- Selecciona el **pipeline** de planeación: `ompl`.
- Ajusta parámetros de petición: `planner_id: ompl`, `planning_attempts: 1`, `planning_pipeline: ompl`.
- Fija escalamiento de velocidad y aceleración máximas en `1.0` (sin escalado).

```yaml
planning_pipelines:
  pipeline_names: ["ompl"]

plan_request_params:
  planner_id: ompl
  planning_attempts: 1
  planning_pipeline: ompl
  max_velocity_scaling_factor: 1.0
  max_acceleration_scaling_factor: 1.0
```

---

### 4) `arduinobot_py_examples/setup.py`
**Qué hace / cambios claves**
- Publica el ejecutable `simple_moveit_interface` en `console_scripts` para poder llamarlo por nombre desde ROS 2.
- Instala el **directorio de launch** del paquete: incluye cualquier archivo que coincida con `launch/*launch.[pxy][yaml]*` bajo `share/<paquete>/launch`.
- Registra `package.xml` en el índice de ament y define metadatos del paquete Python.

Fragmentos relevantes:
```python
entry_points={
    'console_scripts': [
        # ...
        'simple_moveit_interface = arduinobot_py_examples.simple_moveit_interface:main',
    ],
},
data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'),
        glob(os.path.join('launch','*launch.[pxy][yaml]*'))),
]
```

---

### 5) `arduinobot_py_examples/package.xml`
**Qué hace / dependencias agregadas**
- Declara dependencias de ejecución para: `rclpy`, `std_msgs`, `rcl_interfaces`, `arduinobot_msgs`, **`ros2launch`** (por usar archivos `launch`) y **`moveit_py`** **condicionado** a `ROS_DISTRO >= iron` (la API Python de MoveIt solo existe desde Iron).  
- Exporta el tipo de build `ament_python`.

Fragmento relevante:
```xml
<exec_depend>ros2launch</exec_depend>
<exec_depend condition="$ROS_DISTRO >= iron">moveit_py</exec_depend>
<export>
  <build_type>ament_python</build_type>
</export>
```

---

## Cómo ejecutar

1. Compila desde el *workspace*:
```bash
colcon build --symlink-install
```
2. Sourcing:
```bash
. install/setup.bash
```
3. Lanza el nodo con MoveIt configurado:
```bash
ros2 launch arduinobot_py_examples simple_moveit_interface.launch.py
```

> **Nota:** Si usas una distro anterior a **Iron** (p. ej., **Humble**), `moveit_py` no estará disponible; usa la versión C++ del ejemplo o actualiza tu distro.

---

## Notas finales
- El ejemplo fija el objetivo del **brazo** y del **gripper** mediante estados de robot (`RobotState`) y usa `MoveItPy.execute(...)` con controladores por defecto (lista vacía).  
- Ajusta `planning_python_api.yaml` si necesitas otro *pipeline* (por ejemplo añadir STOMP) o si quieres limitar velocidades y aceleraciones.