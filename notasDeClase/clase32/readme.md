# Clase 32 — URDF en ROS 2


## Objetivo

* Crear el paquete `arduinobot_description` (solo archivos XML/Xacro y mallas STL).
* Definir el modelo del robot en **Xacro** (extensión de URDF) y visualizarlo en **RViz2** usando el paquete `urdf_tutorial`.

---

## 1) Pasos rápidos

En `src`:

```bash
ros2 pkg create --build-type ament_cmake arduinobot_description
```

En la **raíz del workspace**:

```bash
colcon build
```

Dentro de `arduinobot_description`:

* Crear una carpeta llamada **`urdf/`**.
* Crear el archivo **`urdf/arduinobot.urdf.xacro`**.
* Crear otra carpeta **`meshes/`** que contendrá los **STL** del robot.
* **Escribir** el archivo `arduinobot.urdf.xacro`.
* **Instalar** las carpetas `urdf` y `meshes` editando `CMakeLists.txt`.
* Ejecutar:

  ```bash
  colcon build
  . install/setup.bash
  ```

Instalar la librería URDF (según las notas):

```bash
sudo apt-get install ros2-humble-urdf-tutorial
```

> **Nota práctica:** en Humble, el nombre habitual del paquete es:
>
> ```bash
> sudo apt-get install ros-humble-urdf-tutorial
> ```

Para usar URDF, recordando que en `model:` va la **ruta absoluta** del `.urdf.xacro`:

```bash
ros2 launch urdf_tutorial display.launch.py model:=/home/joho/Documents/RoboticsAndROS2-LearnByDoingManipulators/arduinobot_ws/src/arduinobot_description/urdf/arduinobot.urdf.xacro
```

---

## 2) ¿Qué es Xacro y por qué usarlo?

* **URDF** (Unified Robot Description Format) describe la **estructura** del robot (links y joints) en **XML**.
* **Xacro** *extiende* URDF: permite **propiedades/variables** (p. ej., `PI`, `effort`, `velocity`) y **macros** reutilizables (p. ej., una macro `default_inertial`).
* En un paquete de *descripción* no añadimos código C++/Python; solo **XML/Xacro** y **mallas**.

---

## 3) Estructura recomendada del paquete

```
arduinobot_ws/
├─ src/
│  └─ arduinobot_description/
│     ├─ CMakeLists.txt
│     ├─ package.xml
│     ├─ urdf/
│     │  └─ arduinobot.urdf.xacro
│     └─ meshes/
│        ├─ basement.STL
│        ├─ base_plate.STL
│        ├─ forward_drive_arm.STL
│        ├─ horizontal_arm.STL
│        ├─ claw_support.STL
│        ├─ left_finger.STL
│        └─ right_finger.STL
```

> Copia todos los **STL** al directorio `meshes/`. Escalarás/posicionarás cada malla desde el Xacro.

---

## 4) Instalar `urdf/` y `meshes/` (CMakeLists)

Agrega esta instrucción :

```cmake
install(
  DIRECTORY urdf meshes
  DESTINATION share/${PROJECT_NAME}
)
```

Después:

```bash
colcon build
. install/setup.bash
```

> Si trabajas con varios workspaces, recuerda “sourcear” primero el underlay y luego tu overlay.

---

## 5) Esqueleto mínimo del Xacro

Encabezado y contenedor principal:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="arduinobot">

  <!-- Propiedades reutilizables -->
  <xacro:property name="PI" value="3.14159265359"/>
  <xacro:property name="effort" value="30.0"/>
  <xacro:property name="velocity" value="10.0"/>

  <!-- (Opcional) Macro para inercia por defecto -->
  <xacro:macro name="default_inertial" params="mass">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="${mass}"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </xacro:macro>

  <!-- Links base -->
  <link name="world"/>

  <link name="base_link">
    <xacro:default_inertial mass="1.0"/>
    <visual>
      <origin xyz="-0.5 -0.5 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://arduinobot_description/meshes/basement.STL" scale="0.01 0.01 0.01"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="-0.5 -0.5 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://arduinobot_description/meshes/basement.STL" scale="0.01 0.01 0.01"/>
      </geometry>
    </collision>
  </link>

  <!-- Unión fija al mundo -->
  <joint name="virtual_joint" type="fixed">
    <parent link="world"/>
    <child link="base_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>

  <!-- Segundo link y primera articulación del robot -->
  <link name="base_plate">
    <xacro:default_inertial mass="0.1"/>
    <visual>
      <origin xyz="-0.39 -0.39 -0.56" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://arduinobot_description/meshes/base_plate.STL" scale="0.01 0.01 0.01"/>
      </geometry>
    </visual>
  </link>

  <joint name="joint_1" type="revolute">
    <parent link="base_link"/>
    <child  link="base_plate"/>
    <origin xyz="0 0 0.307" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-${PI/2}" upper="${PI/2}" effort="${effort}" velocity="${velocity}"/>
  </joint>

</robot>
```

**Claves de la clase:**

* `visual`/`geometry`/`mesh`: define **qué** malla se renderiza (y **scale**).
* `origin` dentro de `visual`: **posiciona y orienta la malla** respecto al *link* (prueba/ajusta valores hasta centrar).
* `joint type="fixed"` (virtual) une `world` → `base_link`.
* `joint type="revolute"` + `axis` establece el eje de rotación; `limit` usa propiedades Xacro (`PI`, `effort`, `velocity`) para no “quemar” números.

---

## 6) Visualizar en RViz2

1. **Construir y “sourcear”** (de nuevo, tras cambios):

   ```bash
   colcon build
   . install/setup.bash
   ```
2. **Lanzar** :

   ```bash
   ros2 launch urdf_tutorial display.launch.py model:=/home/joho/Documents/RoboticsAndROS2-LearnByDoingManipulators/arduinobot_ws/src/arduinobot_description/urdf/arduinobot.urdf.xacro
   ```

---

## 7) Consejos y errores comunes

* **Paquete no encontrado**: asegúrate de haber hecho `colcon build` y `. install/setup.bash`.
* **Malla no cargada**: confirma:

  * Rutas `package://arduinobot_description/meshes/...` (sensibles a mayúsculas).
  * Que `meshes/` y `urdf/` están **instalados** en `CMakeLists.txt`.
* **Malla descentrada**: ajusta `origin xyz/rpy` dentro de `visual`.
* **Varios workspaces**: “sourcea” en orden (underlay → overlay).

---

## 8) Próximos pasos

* Añadir más *links* (`forward_drive_arm`, `horizontal_arm`, `claw_support`, `gripper_left/right`).
* Definir *joints* (`revolute`/`fixed`), ejes (`axis xyz`), y límites (`limit lower/upper/effort/velocity`) reutilizando propiedades Xacro.
* (Opcional) Añadir `collision` e `inertial` para simulación más realista.