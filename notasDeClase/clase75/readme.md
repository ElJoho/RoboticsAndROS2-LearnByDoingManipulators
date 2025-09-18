# Clase 74 Integración de Moveit2

Este documento resume **lo que se hizo en clase** para integrar MoveIt 2 con el robot *Arduinobot*, los **comandos de terminal** utilizados y la **lógica de los archivos** de configuración y *launch*. Está pensado para que puedas **reproducir el flujo** desde cero y entender para qué sirve cada fichero.

---

## Objetivos de la sesión
- Configurar el paquete `arduinobot_moveit` y **exponer** (instalar) sus carpetas `config/` y `launch/`.  
- Crear el *launch* `moveit.launch.py` para iniciar **Move Group** (backend de MoveIt 2) y **RViz2** con la configuración del robot.  
- Conectar MoveIt 2 con los **controladores** del brazo y la garra configurados en ROS 2 Control / Gazebo.

---

## Comandos de terminal usados

### Terminal 1 — Compilar y lanzar simulación vacía en Gazebo
```bash
colcon build
. install/setup.bash
ros2 launch arduinobot_description gazebo.launch.py
```

### Terminal 2 — Iniciar el sistema de control (ros2_control)
```bash
. install/setup.bash
ros2 launch arduinobot_controller controller.launch.py
```

### Terminal 3 — Iniciar MoveIt (Move Group + RViz2)
```bash
. install/setup.bash
ros2 launch arduinobot_moveit moveit.launch.py
```

> Estos pasos siguen el flujo descrito en la transcripción: primero **simulación**, luego **controladores**, y por último **MoveIt** (planificación y ejecución).

---

## Archivos clave y propósito

- `CMakeLists.txt`: instala las carpetas `launch` y `config` dentro de `share/${PROJECT_NAME}` para que ROS 2 las encuentre al ejecutar *launch files*.  fileciteturn0file0

- `package.xml`: declara dependencias de ejecución necesarias para el lanzamiento y visualización: `ros2launch`, `rviz2`, `moveit_configs_utils`, `moveit_ros_move_group` y la descripción del robot `arduinobot_description`.  fileciteturn0file1

- `launch/moveit.launch.py`: inicia el nodo `move_group` (núcleo de MoveIt 2), configura parámetros (URDF/SRDF, cinemática, límites, controladores) y lanza `rviz2` con una sesión preconfigurada.  fileciteturn0file2

- `config/initial_positions.yaml`: posiciones iniciales del sistema *fake* de `ros2_control` para *Arduinobot* durante pruebas/simulación.  fileciteturn0file3

- `config/joint_limits.yaml`: escalados por defecto para velocidad y aceleración y límites por articulación (habilita/ajusta `has_velocity_limits`, `max_velocity`, etc.).  fileciteturn0file4

- `config/kinematics.yaml`: define el *plugin* y parámetros de cinemática inversa (KDL).  fileciteturn0file5

- `config/moveit_controllers.yaml`: integra MoveIt con los controladores de seguimiento de trayectorias (`arm_controller`, `gripper_controller`) y lista las *joints* asociadas.  fileciteturn0file6

- `config/pilz_cartesian_limits.yaml`: límites cartesianos para el planificador Pilz (velocidades y aceleraciones máximas de traslación/rotación).  fileciteturn0file7

> Con estos ficheros, MoveIt 2 conoce el **modelo del robot** (URDF/SRDF), su **cinemática**, sus **límites**, y cómo **enviar trayectorias** a los controladores activos.

---

## Flujo de ejecución (resumen)

1. **Simulación**: `gazebo.launch.py` inicia el mundo vacío y carga `ros2_control` del robot.
2. **Control**: `controller.launch.py` activa `arm_controller` y `gripper_controller` (necesarios para que MoveIt pueda ejecutar).  
3. **Planificación/Ejecución**: `moveit.launch.py` levanta:
   - `move_group` (servicios de planificación y ejecución).
   - `rviz2` con el *Motion Planning* plugin y la configuración (URDF/SRDF/IK/límites).  fileciteturn0file2

En RViz2:
- Cambiar *Fixed Frame* a `world`.
- Cargar el panel de **Motion Planning**.
- Seleccionar planificador **OMPL** y (opcional) *Approx. IK Solution* para acelerar.  
- Definir estados objetivo (brazo/garra) y usar **Plan and Execute** para mover el robot (tal como se describió en clase).

---

## Notas sobre el *build system* (CMake y Package)

- `CMakeLists.txt`:
  - Requiere `ament_cmake` y **instala** `launch/` y `config/` en `share/${PROJECT_NAME}` para que `ros2 launch` pueda encontrarlos.  fileciteturn0file0

- `package.xml`:
  - Dependencias clave:
    - `moveit_configs_utils`: se utiliza el **MoveItConfigsBuilder** para componer parámetros desde archivos (`URDF/SRDF`, YAMLs).  fileciteturn0file1
    - `moveit_ros_move_group`: binario `move_group` (backend MoveIt 2).  fileciteturn0file1
    - `rviz2` y `ros2launch` para la visualización y el sistema de lanzadores.  fileciteturn0file1
    - `arduinobot_description` para resolver la ruta del URDF/Xacro.  fileciteturn0file1

---

## ¿Qué hace exactamente `moveit.launch.py`?

- Declara el argumento `is_sim` (por defecto `True`) para usar **tiempo simulado** (`use_sim_time`) cuando se integre con Gazebo.
- Construye un objeto de configuración con `MoveItConfigsBuilder` indicando:
  - **URDF/Xacro** (`robot_description`) desde `arduinobot_description`.
  - **SRDF** (`robot_description_semantic`).
  - **Controladores** (`trajectory_execution` → `moveit_controllers.yaml`).  fileciteturn0file2
- Lanza:
  - `move_group` con parámetros resultantes.
  - `rviz2` con un `.rviz` predefinido y parámetros de robot/semántica/cinemática/límites.  fileciteturn0file2

> En el archivo adjunto **`moveit.launch.comentado.py`** encontrarás **comentario por línea** explicando la lógica completa.

---

## Reproducción rápida

1. Compila e instala:
   ```bash
   colcon build
   . install/setup.bash
   ```
2. Lanza Gazebo:
   ```bash
   ros2 launch arduinobot_description gazebo.launch.py
   ```
3. Lanza controladores:
   ```bash
   ros2 launch arduinobot_controller controller.launch.py
   ```
4. Lanza MoveIt:
   ```bash
   ros2 launch arduinobot_moveit moveit.launch.py
   ```

¡Listo! Ya puedes planificar y ejecutar movimientos desde RViz2 con **MoveIt 2**.

---