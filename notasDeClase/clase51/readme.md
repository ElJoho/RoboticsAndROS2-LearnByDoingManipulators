# Clase 51 `ros2_control CLI`

En esta clase verificamos **paso a paso** el arranque del sistema de control del robot en Gazebo y
aprendimos a **inspeccionarlo e interactuar** con la línea de comandos de `ros2_control`. Finalmente,
probamos el *gripper* publicando comandos desde la terminal.

---

## 🎯 Objetivos
- Compilar el *workspace* y lanzar la simulación en Gazebo.
- Iniciar el **controller manager** y los controladores del robot.
- Usar la CLI de `ros2_control` para listar controladores, componentes e interfaces.
- Mover el *gripper* publicando en el *topic* correspondiente.
- (Nota) Entender el cambio de interfaz del *gripper* hacia `JointTrajectoryController` y cómo publicar en ese caso.

---

## 🗂️ Contexto de configuración
El archivo `arduinobot_controllers.yaml` define la configuración del *controller manager* (tasa de actualización) y los
tres controladores: `arm_controller`, `gripper_controller` y `joint_state_broadcaster`. En la versión actual, **tanto el brazo como el gripper**
están configurados como `JointTrajectoryController`, y se usa un `JointStateBroadcaster` para publicar estados articulares.

---

## 🖥️ Terminal 1 – Simulación en Gazebo

```bash
colcon build
. install/setup.bash
ros2 launch arduinobot_description gazebo.launch.py 
```

**¿Qué hace cada comando?**
- `colcon build` – Compila todos los paquetes del *workspace* respetando dependencias.
- `. install/setup.bash` – *Sourcea* el entorno resultante de la compilación (pone los paquetes y recursos en el `AMENT_PREFIX_PATH` y el `ROS_PACKAGE_PATH`).
- `ros2 launch arduinobot_description gazebo.launch.py` – Arranca la simulación del robot en el mundo vacío de Gazebo (spawnea el modelo y los plugins necesarios).

**Resultado esperado:** Se abre Gazebo con el robot en escena y consola sin errores.

---

## 🖥️ Terminal 2 – Sistema de control (controller manager + controladores)

```bash
. install/setup.bash
ros2 launch arduinobot_controller controller.launch.py
```

**¿Qué hace cada comando?**
- `. install/setup.bash` – Habilita en la sesión actual los paquetes y recursos instalados.
- `ros2 launch arduinobot_controller controller.launch.py` – Lanza:
  1) `robot_state_publisher` con el URDF plano generado desde Xacro.
  2) `controller_manager` (`ros2_control_node`) con la configuración YAML.
  3) Los *spawners* de `joint_state_broadcaster`, `arm_controller` y `gripper_controller`.

**Resultado esperado:** En consola se ve que los controladores quedan **configurados y activos** (o listos para activarse).

---

## 🖥️ Terminal 3 – Inspección con la CLI de `ros2_control` y ROS 2

```bash
. install/setup.bash
ros2 control   # (presiona TAB dos veces para ver subcomandos disponibles)
ros2 control list_controllers 
ros2 control list_hardware_components
ros2 control list_hardware_interfaces
ros2 topic list
```

**¿Qué hace cada comando?**
- `ros2 control` *(TAB TAB)* – Muestra todos los subcomandos de la CLI de `ros2_control` (p. ej., `list_controllers`, `set_controller_state`, etc.).
- `ros2 control list_controllers` – Lista controladores en el *controller manager*: nombre, tipo y estado (esperado: `joint_state_broadcaster`, `arm_controller`, `gripper_controller`).
- `ros2 control list_hardware_components` – Muestra componentes de hardware expuestos (en simulación: las *joints* del robot).
- `ros2 control list_hardware_interfaces` – Enumera **interfaces de comando** (p. ej., `position`) y **de estado** disponibles.
- `ros2 topic list` – Lista todos los *topics* activos. Aquí verás, entre otros, los asociados a los controladores (p. ej., del brazo y del gripper).

**Resultado esperado:** Confirmar que el sistema está corriendo y que existen *topics* para comandar y leer estados.

---

## 🖥️ Terminal 4 – Comandos al *gripper* (Forward Command → demo)

> **Demostración de la clase:** Publicar en `/gripper_controller/commands` para **abrir** y **cerrar** el gripper.
> Esto funciona cuando el gripper usa la interfaz **ForwardCommand** (mensaje `Float64MultiArray`).

**Abrir (≈ −1 rad):**
```bash
ros2 topic pub /gripper_controller/commands std_msgs/msg/Float64MultiArray "layout:
  dim: []
  data_offset: 0
data: [-1]"
```

**Cerrar (0 rad):**
```bash
ros2 topic pub /gripper_controller/commands std_msgs/msg/Float64MultiArray "layout:
  dim: []
  data_offset: 0
data: [0]"
```

**Explicación:**
- *Topic* `/gripper_controller/commands` – Entrada del controlador del gripper (modo *Forward Command*).
- Tipo `std_msgs/Float64MultiArray` – **`data`** lleva la posición objetivo (en radianes) para la *joint* del gripper.
- `-1` abre los dedos hasta ~−1 rad; `0` los lleva a la posición neutra (cerrado).  
  *En muchos modelos el `joint_5` replica al `joint_4`, por eso se mueven ambos dedos.*

**Resultado esperado:** El gripper abre con `-1` y cierra con `0` dentro de Gazebo.

---

## 📌 Nota importante – Interfaz actual: `JointTrajectoryController`

En la **configuración actual** del proyecto, el `gripper_controller` ya está definido como
`joint_trajectory_controller/JointTrajectoryController` (igual que el brazo), por lo que **el topic anterior puede no existir** y, en su lugar, se usará un *topic* de trayectoria (por ejemplo, `/gripper_controller/joint_trajectory`).

**Ejemplo mínimo de publicación con trayectoria** (un solo punto a −1 rad en 1 s):
```bash
ros2 topic pub /gripper_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "joint_names: ['joint_4']
points:
- positions: [-1.0]
  time_from_start: {sec: 1, nanosec: 0}"
```

> Si deseas repetir exactamente la demo con `Float64MultiArray`, cambia temporalmente el `type` del `gripper_controller` a `forward_command_controller/ForwardCommandController` en el YAML y vuelve a lanzar el sistema.  
> En la configuración oficial para continuar el curso, se recomienda **mantener `JointTrajectoryController`** para brazo y gripper.

---

## 🧪 Comprobaciones rápidas
- **Controladores activos:** `ros2 control list_controllers`
- **Estados articulares:** `ros2 topic echo /joint_states`
- **Interfaces disponibles:** `ros2 control list_hardware_interfaces`
- **Trajectoria ejecutada:** Observa que el gripper llega a la posición objetivo en el tiempo indicado.

---

## 🚑 Solución de problemas
- *No aparecen controladores:* revisa la terminal 2 (errores de carga YAML/URDF). Vuelve a lanzar.
- *El topic no existe:* confirma qué interfaz usa el gripper (Forward vs Trajectory) y ajusta el comando de publicación.
- *No se mueve en Gazebo:* verifica que los controladores estén `active` y que las *joints* coincidan con las del URDF.

---

## 📝 Resumen
- Lanzamos Gazebo (Terminal 1) y el sistema de control (Terminal 2).
- Inspeccionamos `ros2_control` con su CLI (Terminal 3).
- Movimos el gripper publicando desde la terminal (Terminal 4).
- Dejamos la configuración del gripper alineada con el brazo usando **`JointTrajectoryController`** para trabajos futuros.