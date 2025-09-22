# Clase 85 — Ejecución de MoveIt2 con 4 terminales (Simulación + Control + MoveIt + Nodo C++)

> En esta clase pusimos en marcha **todo el stack** para mover el _arduinobot_ en simulación: **Gazebo**, **ros2_control**, **MoveIt 2** y finalmente nuestro **nodo C++** (`simple_moveit_interface`) que planifica y ejecuta trayectorias en los grupos `arm` y `gripper`.

---

## Flujo general

1. **Terminal 1**: Levanta **Gazebo** con el robot y publica el `robot_state` (URDF, TFs, reloj, cámara, etc.).  
2. **Terminal 2**: Inicia **controladores** (`joint_state_broadcaster`, `arm_controller`, `gripper_controller`) vía `ros2_control`.  
3. **Terminal 3**: Arranca **MoveIt 2** (`move_group` + RViz) y conecta los controladores para planificar/ejecutar.  
4. **Terminal 4**: Ejecuta el **nodo C++** que define metas articulares, planifica con MoveIt y **mueve** el robot.

> **Importante**: en **cada terminal** se ejecuta primero `source install/setup.bash` para exportar `PATH`, `AMENT_PREFIX_PATH`, `COLCON_CURRENT_PREFIX`, etc., del _workspace_.

---

## Terminal 1 — Simulación en Gazebo

**Comandos ejecutados**
```bash
. install/setup.bash
ros2 launch arduinobot_description gazebo.launch.py
```

**¿Por qué?**  
- `gazebo.launch.py` carga el **mundo vacío** y el **modelo del robot** (URDF) y arranca nodos como `robot_state_publisher` y el _bridge_ ROS ↔ Gazebo.  
- Los logs muestran: creación del mundo `empty.sdf`, carga del hardware simulado vía `gz_ros2_control`, publicación de `/clock`, `/joint_states`, etc.  
- Se inicializa la interfaz de control (`gz_ros2_control`) para las **juntas** `joint_1` a `joint_5` y se anuncia que `joint_5` **imita** a `joint_4` con multiplicador `-1` (mecanismo del gripper).  
- La GUI de Gazebo se abre con los _plugins_ (3D View, World Control, etc.) y queda lista para ver el movimiento.

**Qué confirma la salida**  
- `robot_state_publisher`: segmentos (links) cargados correctamente.  
- `gz_ros2_control`: hardware `RobotSystem` **configurado y activado**.  
- `controller_manager`: posteriormente (al cargar controladores) acepta metas y reporta _goal reached_.

---

## Terminal 2 — Controladores (ros2_control)

**Comandos ejecutados**
```bash
. install/setup.bash
ros2 launch arduinobot_controller controller.launch.py
```

**¿Por qué?**  
- Este `launch` **spawnea** los controladores necesarios para que MoveIt pueda ejecutar trayectorias:  
  - `joint_state_broadcaster` (publica estados de todas las juntas).  
  - `arm_controller` (FollowJointTrajectory para el grupo `arm`).  
  - `gripper_controller` (FollowJointTrajectory para `gripper`).

**Qué confirma la salida**  
- Cada `spawner_*` reporta **Loaded → Configured → Activated**.  
- Cuando MoveIt envía una trayectoria, los controladores responden con **“Received new action goal”** y terminan con **“Goal reached, success!”**.

---

## Terminal 3 — MoveIt 2 (`move_group` + RViz)

**Comandos ejecutados**
```bash
. install/setup.bash
ros2 launch arduinobot_moveit moveit.launch.py
```

**¿Por qué?**  
- Arranca `move_group` (servidor de planificación/ejecución) y **RViz** para visualizar.  
- Carga el **modelo del robot** y los **pipelines de planificación** (`OMPL`, `Pilz`, `CHOMP`, según configuración).  
- **Conecta** con los controladores de `ros2_control` (`arm_controller`, `gripper_controller`) y **gestiona** la ejecución de trayectorias.

**Qué confirma la salida**  
- “You can start planning now!” → listo para recibir peticiones.  
- Al recibir la petición de nuestro nodo C++:  
  - Se selecciona el planificador (`geometric::RRTConnect` para `arm` y `gripper`).  
  - `Trajectory execution is managing controllers` y luego `Starting trajectory execution ...`.  
  - Al final: **`Completed trajectory execution with status SUCCEEDED`** para brazo y gripper.

> Nota: aparecen advertencias sobre **Octomap** y sensores 3D — no afectan a este ejercicio (no usamos mapeo 3D).

---

## Terminal 4 — Nodo C++ (`simple_moveit_interface`)

**Comandos ejecutados**
```bash
. install/setup.bash
ros2 run arduinobot_cpp_examples simple_moveit_interface
```

**¿Por qué?**  
- Este ejecutable (de la **Clase 84**) crea dos `MoveGroupInterface`: `arm` y `gripper`.  
- Establece metas de juntas: `arm` → `[1.57, 0.0, 0.0]` (≈90° base), `gripper` → `[-0.7, 0.7]` (abrir pinza).  
- Valida límites, **planifica** para ambos grupos y si ambos planes son `SUCCESS`, **ejecuta**.

**Qué confirma la salida**  
- `Ready to take commands for planning group ...` → las interfaces se conectaron a `move_group`.  
- `Planning request complete!` con tiempos de planificación.  
- `Plan and Execute request complete!` (dos veces: **arm** y luego **gripper**).  
- En Gazebo y MoveIt se observa el movimiento a la postura objetivo y la apertura de la pinza.

---

## Secuencia correcta (resumen)

1. **T1: Gazebo** → Mundo + robot + ros2_control backend.  
2. **T2: Controladores** → Spawnear y activar `joint_state_broadcaster`, `arm_controller`, `gripper_controller`.  
3. **T3: MoveIt** → `move_group` conecta controladores y habilita planificación/ejecución.  
4. **T4: Nodo C++** → envía metas, planifica y ejecuta.

> Si cambias el orden, es probable que **MoveIt** no encuentre controladores o que la ejecución **falle**.

---

## Troubleshooting rápido

- **No se mueve el robot**: verifica que en T2 todos los controladores estén **Activated**.  
- **Planificación falla**: confirma que las metas están **dentro de límites** y que T3 está corriendo sin errores críticos.  
- **RViz sin TF/URDF**: asegúrate de haber hecho `source install/setup.bash` en **todas** las terminales.

---

## Lo aprendido en la Clase 85
- Orquestar un stack completo de **simulación + control + planificación + aplicación**.  
- Interpretar logs clave de `controller_manager`, `move_group` y del nodo C++.  
- Validar que la ejecución finaliza en **SUCCEEDED** para brazo y gripper.