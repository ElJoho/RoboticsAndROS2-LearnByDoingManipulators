# Clase 50 `Launch the controller`

En esta clase preparamos y **lanzamos el sistema de control** del robot en **Gazebo** usando **ros2_control**.
El objetivo fue crear un *launch file* que:
1) publique la descripción del robot (URDF),
2) inicie el *controller manager* (ros2_control_node) con nuestro YAML de controladores y
3) **spawnee** los controladores (`joint_state_broadcaster`, `arm_controller`, `gripper_controller`).

---

## 📂 Estructura del proyecto (resumen)
```
arduinobot_ws/
└── src/
    ├── arduinobot_controller/
    │   ├── config/
    │   │   └── arduinobot_controllers.yaml
    │   ├── launch/
    │   │   └── controller.launch.py
    │   ├── include/arduinobot_controller/
    │   ├── src/
    │   ├── CMakeLists.txt
    │   └── package.xml
    ├── arduinobot_description/
    │   ├── launch/
    │   │   ├── display.launch.py
    │   │   └── gazebo.launch.py
    │   └── urdf/
    │       ├── arduinobot.urdf.xacro
    │       ├── arduinobot_ros2_control.xacro
    │       └── arduinobot_gazebo.xacro
    └── arduinobot_cpp_examples/ ...
```
> La captura de pantalla que subiste muestra una estructura equivalente.

---

## 🧠 ¿Qué hace `controller.launch.py`?

Archivo: `arduinobot_controller/launch/controller.launch.py`

1) **Genera `robot_description` desde Xacro**
   - Construye un `ParameterValue` con un `Command("xacro <ruta/arduinobot.urdf.xacro>")` para obtener el URDF plano.
   - Se usa `get_package_share_directory("arduinobot_description")` para localizar el paquete y `os.path.join(...)` para formar la ruta al Xacro.

2) **Nodo `robot_state_publisher`**
   - Lanza el ejecutable `robot_state_publisher` y le pasa `parameters=[{"robot_description": robot_description}]` para publicar TF y `/robot_description`.

3) **Nodo `controller_manager` (ros2_control_node)**
   - Ejecuta `controller_manager/ros2_control_node` con:
     - `{"robot_description": robot_description}`
     - Ruta al YAML `config/arduinobot_controllers.yaml` dentro del paquete `arduinobot_controller`

4) **Spawners de controladores**
   - Tres nodos `controller_manager/spawner` para:
     - `joint_state_broadcaster`
     - `arm_controller`
     - `gripper_controller`
   - Con los argumentos `--controller-manager /controller_manager` para apuntar al *manager* correcto.

Pseudocódigo del `LaunchDescription`:
```python
ld = LaunchDescription([
    robot_state_publisher_node,
    controller_manager,
    joint_state_broadcaster_spawner,
    arm_controller_spawner,
    gripper_controller_spawner
])
```

---

## ⚙️ `arduinobot_controllers.yaml` (configuración de controladores)

- `controller_manager.ros__parameters.update_rate: 10`
- **`arm_controller`**: `joint_trajectory_controller/JointTrajectoryController`
  - Joints: `joint_1`, `joint_2`, `joint_3`
  - `command_interfaces`: `position`
  - `state_interfaces`: `position`
- **`gripper_controller`**: `forward_command_controller/ForwardCommandController`
  - Joint: `joint_4`
  - `interface_name`: `position`
- **`joint_state_broadcaster`**: `joint_state_broadcaster/JointStateBroadcaster`

---

## 🏗️ Instalación de `launch/` y dependencias

### CMake
En `arduinobot_controller/CMakeLists.txt` se instala **`config` y `launch`** para poder usar `ros2 launch`:
```cmake
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)
```

### package.xml
Se declararon dependencias de ejecución para el lanzamiento:
- `ros2launch`
- `robot_state_publisher`
- `xacro`
- `controller_manager`

---

## ▶️ Compilar y lanzar

Desde el *workspace*:
```bash
cd ~/arduinobot_ws
colcon build --symlink-install
source install/setup.bash

ros2 launch arduinobot_controller controller.launch.py
```

> **Tip**: si usas distintas terminales o reinicias, recuerda volver a `source install/setup.bash`.

---

## ✅ Verificación rápida

- Controladores cargados:
  ```bash
  ros2 control list_controllers
  ```
  Deberías ver `joint_state_broadcaster`, `arm_controller`, `gripper_controller` con sus estados (`active`/`inactive`).

- Publicación de `joint_states`:
  ```bash
  ros2 topic echo /joint_states
  ```

- Interfaces disponibles:
  ```bash
  ros2 control list_hardware_interfaces
  ```

- (Opcional) GUI para trayectorias:
  ```bash
  ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller
  ```

---

## 🩺 Errores comunes y soluciones

- **`Package not found` o no encuentra el Xacro**  
  Verifica que `arduinobot_description` esté instalado y la ruta al Xacro sea correcta.  
  Ejecuta `ros2 pkg prefix arduinobot_description` para comprobar el *share*.

- **No aparece `joint_states`**  
  Asegúrate de que `joint_state_broadcaster` esté `active`. Reinicia el spawner si es necesario:
  ```bash
  ros2 run controller_manager spawner joint_state_broadcaster --controller-manager /controller_manager
  ```

- **El *controller manager* no carga el YAML**  
  Revisa que `arduinobot_controllers.yaml` esté instalado (directorio `share/arduinobot_controller/config`) y que la ruta en el launch sea correcta.

- **`inactive` en `arm_controller` o `gripper_controller`**  
  Comprueba compatibilidad de `command_interfaces`/`state_interfaces` y que los *joints* existen en el URDF.

---

## 🔭 Próximos pasos
- Enviar trayectorias al `arm_controller` (por ejemplo, desde un nodo Python que publique en `/arm_controller/joint_trajectory`).
- Añadir *constraints* y *gains* si se requiere mayor precisión.
- Integrar con el *spawner* de Gazebo para autolanzar todo desde `gazebo.launch.py`.

---

## 📎 Referencias de archivos de esta clase
- `arduinobot_controller/launch/controller.launch.py`
- `arduinobot_controller/config/arduinobot_controllers.yaml`
- `arduinobot_controller/CMakeLists.txt`
- `arduinobot_controller/package.xml`

> Con esta configuración, el robot queda listo para recibir comandos de posición y publicar su estado articular dentro del entorno simulado.
