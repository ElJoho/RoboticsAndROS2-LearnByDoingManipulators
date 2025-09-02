# Clase 47 — **`ros2_control` con Gazebo** (Notas de clase)


En esta lección integramos **`ros2_control`** al robot para que, al lanzar **Gazebo**, se inicie el sistema de **control de juntas** y podamos cargar controladores.


## Archivos del repo y rol

- `urdf/arduinobot.urdf.xacro`: URDF principal (solo *links* y *joints*). ➡️ Añadir `<xacro:arg name="is_ignition" default="true"/>` al inicio.

- `urdf/arduinobot_gazebo.xacro`: Selección condicional del **plugin** de `ros2_control` para Gazebo (Ignition y Gazebo Sim).

- `urdf/arduinobot_ros2_control.xacro`: Bloque `<ros2_control>` con el *hardware interface* simulado (IgnitionSystem / GazeboSimSystem).

- `package.xml`: Dependencias (incluye condicionales si aplica).


### Juntas detectadas en tu URDF
Se detectaron (muestra): **virtual_joint, joint_1, joint_2, joint_3, horizontal_arm_to_claw_support, joint_4, joint_5**


---
## 1) Argumento `is_ignition` (Humble vs Iron/Jazzy)

Declaramos un argumento Xacro que actúa como selector de distro:

```xml
<xacro:arg name="is_ignition" default="true"/>
```

- `true` → **Humble** (ecosistema *Ignition*).
- `false` → **Iron/Jazzy o superior** (ecosistema *Gazebo Sim*).



---
## 2) `arduinobot_gazebo.xacro`: plugin de `ros2_control` para Gazebo

Dentro de `<gazebo>`, se carga el plugin correcto según `is_ignition`:

```xml
<gazebo>
  <xacro:if value="$(arg is_ignition)">
    <plugin filename="ign_ros2_control-system" name="ign_ros2_control::IgnitionROS2ControlPlugin">
      <parameters>$(find PACK)/config/arduinobot_controllers.yaml</parameters>
    </plugin>
  </xacro:if>
  <xacro:unless value="$(arg is_ignition)">
    <plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
      <parameters>$(find PACK)/config/arduinobot_controllers.yaml</parameters>
    </plugin>
  </xacro:unless>
</gazebo>
```
> Reemplaza **PACK** por el paquete que realmente contiene tu YAML (p. ej., `arduinobot_description` o `arduinobot_controller`).


En tu archivo parece apuntar a: **arduinobot_controller**.


---
## 3) `arduinobot_ros2_control.xacro`: bloque `<ros2_control>`

Define el **sistema** y el *hardware interface* simulado:

```xml
<ros2_control name="robot_system" type="system">
  <xacro:if value="$(arg is_ignition)">
    <hardware><plugin>ign_ros2_control/IgnitionSystem</plugin></hardware>
  </xacro:if>
  <xacro:unless value="$(arg is_ignition)">
    <hardware><plugin>gz_ros2_control/GazeboSimSystem</plugin></hardware>
  </xacro:unless>
  <!-- Aquí mapeas controladores y juntas -->
</ros2_control>
```

### Interfaces por junta
Para cada junta móvil:
- `command_interface`: p. ej. `position` para enviar consignas.
- `state_interface`: p. ej. `position` para leer retroalimentación.
- Límites (`min`, `max`) en radianes.

**Gripper**: una mordaza puede **imitar** a la otra con `mimic` y `multiplier=-1`.



---
## 4) `package.xml`: dependencias

Ejemplo con dependencias condicionales por distro:

```xml
<exec_depend>ros_gz_sim</exec_depend>
<exec_depend>ros_gz_bridge</exec_depend>
<exec_depend condition="$ROS_DISTRO == humble">ign_ros2_control</exec_depend>
<exec_depend condition="$ROS_DISTRO >= iron">gz_ros2_control</exec_depend>
```


Dependencias encontradas (no exhaustivo):
- urdf
- xacro
- robot_state_publisher
- joint_state_publisher_gui
- ros2launch
- rviz2
- ros_gz_sim
- ros_gz_bridge
- = iron">gz_ros2_control
- ign_ros2_control

Dependencias condicionales detectadas:
- $ROS_DISTRO >= iron → gz_ros2_control
- $ROS_DISTRO == humble → ign_ros2_control


---
## 5) Uso y verificación

```bash
colcon build --packages-select arduinobot_description
source install/setup.bash

# Lanzar simulación
ros2 launch arduinobot_description gazebo.launch.py
```

Verifica control:
```bash
ros2 control list_hardware_interfaces
ros2 control list_controllers
```

---
## 6) Errores típicos y soluciones

- **Undefined substitution argument `is_ignition`**  
  Declara el argumento en el Xacro principal o pásalo desde el `launch` al ejecutar `xacro`.

- **PackageNotFoundError: '..._controller'**  
  El `<parameters>` apunta a `$(find <paquete>)/config/...` y ese paquete no existe/ no está instalado.  
  Opciones: mover el YAML a `arduinobot_description/config` e instalarlo con `install(DIRECTORY config ...)`, o crear el paquete `<paquete>` y añadir `config/`.

- **Controladores no visibles** en `controller_manager`  
  Revisa el YAML en `<parameters>`: al menos `joint_state_broadcaster` y un controlador (p. ej., `position_controllers/JointGroupPositionController`) con la lista correcta de juntas.
