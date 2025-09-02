# Tarea Clasee 44: Simulación de Cámara RGB (Raspberry Pi Camera 3) en Gazebo — *Arduinobot*

Este repositorio documenta cómo se resolvió la tarea de simular una **cámara RGB** montada en el robot y publicar su flujo de video a ROS 2 mediante `ros_gz_bridge`.  
Se modificaron dos archivos principales:

- `urdf/arduinobot.urdf.xacro`
- `launch/gazebo.launch.py`

---

## 1) Objetivo de la tarea

Añadir a **Gazebo** una cámara RGB unida al robot, con **colisiones** e **inercia** para su simulación física, y con un **sensor** configurado con las especificaciones de una **Raspberry Pi Camera 3**:

- Resolución **2304×1296**
- **30 FPS**
- Campo de visión **horizontal 66° (1.15 rad)** y **vertical 41° (0.71 rad)**

> Nota: En Gazebo el **FoV vertical** no se fija explícitamente: se deduce de la relación de aspecto. Con 2304×1296 (16:9) y FoV horizontal 1.15 rad, el FoV vertical resultante ≈ 0.70 rad (≈ 40°), en la práctica **~0.71 rad** (41°) que pide el enunciado.

---

## 2) Cambios en `arduinobot.urdf.xacro`

### 2.1. Añadir colisión al `link` de la cámara

Se añadió un bloque `<collision>` al `link` `rgb_camera` reutilizando la misma malla y `origin` del `<visual>`, para que Gazebo tenga en cuenta el volumen de la cámara en la simulación:

```xml
<link name="rgb_camera">
  <!-- Visual existente -->
  <visual>
    <origin rpy="-1.57 0 -1.57" xyz="-0.1 0.125 0.15"/>
    <geometry>
      <mesh filename="package://arduinobot_description/meshes/pi_camera.STL" scale="0.01 0.01 0.01"/>
    </geometry>
  </visual>

  <!-- NUEVO: colisión -->
  <collision>
    <origin rpy="-1.57 0 -1.57" xyz="-0.1 0.125 0.15"/>
    <geometry>
      <mesh filename="package://arduinobot_description/meshes/pi_camera.STL" scale="0.01 0.01 0.01"/>
    </geometry>
  </collision>
```

### 2.2. Añadir inercia (masa) al `link` de la cámara

Se añadió un bloque `<inertial>` con **masa 0.001 kg**. Los términos de la matriz de inercia se fijan a valores pequeños no nulos, suficientes para la simulación (ajústalos si cuentas con medidas reales):

```xml
  <!-- NUEVO: inercial -->
  <inertial>
    <mass value="0.001"/>
    <!-- Inercia aproximada (diagonal pequeña y no nula) -->
    <inertia ixx="1e-6" ixy="0.0" ixz="0.0"
             iyy="1e-6" iyz="0.0"
             izz="1e-6"/>
  </inertial>
</link>
```

### 2.3. Activar el sensor de cámara en Gazebo

Se añadió un bloque `<gazebo reference="rgb_camera">` con un `<sensor type="camera">` que publica imagen y `camera_info` con los parámetros pedidos (2304×1296, 30 Hz, FoV H 1.15 rad). También se fijó el `gz_frame_id` y el nombre de la cámara a `camera`:

```xml
<!-- Sensor de cámara en Gazebo -->
<gazebo reference="rgb_camera">
  <sensor name="camera" type="camera">
    <always_on>true</always_on>
    <update_rate>30</update_rate>
    <visualize>true</visualize>

    <!-- Tópico que saldrá desde Gazebo -->
    <topic>/image_raw</topic>
    <gz_frame_id>/rgb_camera</gz_frame_id>

    <camera name="camera">
      <horizontal_fov>1.15</horizontal_fov>
      <image>
        <width>2304</width>
        <height>1296</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>
      <distortion>
        <k1>0.0</k1><k2>0.0</k2><k3>0.0</k3>
        <p1>0.0</p1><p2>0.0</p2>
        <center>0.5 0.5</center>
      </distortion>
    </camera>
  </sensor>
</gazebo>
```

---

## 3) Cambios en `launch/gazebo.launch.py`

Se añadió el **bridge** para exponer los tópicos de Gazebo en ROS 2:

```python
gz_ros2_bridge = Node(
    package="ros_gz_bridge",
    executable="parameter_bridge",
    arguments=[
        "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        "/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
        "/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
    ],
    output="screen",
)
```

Además, el `launch` incluye:
- El `IncludeLaunchDescription` de `ros_gz_sim` para levantar Gazebo.
- El **spawn** del modelo usando el URDF generado por `xacro` publicado en `/robot_description`.
- `robot_state_publisher` con `use_sim_time=True`.

---

## 4) Interfaz ROS 2 de la cámara

- **Remapeos**: el enunciado pedía remapear `image_raw → image_raw` y `camera_info → camera_info` (es decir, sin cambios); por eso **no se usa** *namespace* y los nombres de los tópicos quedan **planos** (`/image_raw`, `/camera_info`).
- **Nombre de la cámara**: `camera`
- **Frame**: `gz_frame_id=/rgb_camera`

---

## 5) Ejemplo del instructor (y qué ajustamos)

El instructor mostró:
- Añadir `<collision>` al `link` `rgb_camera` reutilizando la malla de `visual`.
- Un `<gazebo><sensor type="camera">` con `update_rate=30`, `image` en alta resolución y `camera name="camera"`.  
  En su ejemplo el `horizontal_fov` era distinto; aquí lo **ajustamos a 1.15 rad (66°)** para que coincida con la **Raspberry Pi Camera 3** solicitada.

---

## 6) Cómo probar

1. **Lanzar** Gazebo y el robot:
   ```bash
   ros2 launch arduinobot_description gazebo.launch.py
   ```
2. **Verificar** que los tópicos existen:
   ```bash
   ros2 topic list | grep -E "/image_raw|/camera_info"
   ```
3. **Visualizar** la imagen (ejemplos):
   ```bash
   ros2 run rqt_image_view rqt_image_view   # selecciona /image_raw
   ```

---

## 7) Notas sobre `<gazebo><sensor>` vs `<gazebo><plugin>`

- En Gazebo (Ignition/GZ), los **sensores** se describen con `<sensor ...>` y publican datos sin que tengas que cargar manualmente una librería.
- El bloque `<gazebo><plugin>` se usa para otros plugins de modelo/mundo (p. ej., controladores, lógicas personalizadas). Para una **cámara**, el camino recomendado en URDF es **usar `<sensor type="camera">`**.
- Si migras desde Gazebo Classic, encontrarás ejemplos con `<plugin filename="libgazebo_ros_camera.so">`; en GZ/ros_gz el patrón actual es `<sensor>`.

---

## 8) Árbol de archivos (resumen)

```
.
├── launch/
│   └── gazebo.launch.py
└── urdf/
    └── arduinobot.urdf.xacro
```