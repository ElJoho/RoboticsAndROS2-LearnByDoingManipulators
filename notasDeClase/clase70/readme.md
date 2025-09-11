# `arduinobot_utils` — Servicios de conversión de ángulos (ROS 2, TF2)

> **Objetivo.** Crear un **servidor de servicios en Python** que convierta orientaciones entre **cuaterniones** y **ángulos de Euler (roll, pitch, yaw)**, usando utilidades de **TF2** en ROS 2. Este utilitario se distribuye en el paquete `arduinobot_utils` y utiliza las **interfaces de servicio** definidas en `arduinobot_msgs`.

---

## 📦 Paquetes involucrados
- **`arduinobot_utils`**: contiene el nodo Python `angle_conversion.py` con dos servicios:
  - `/euler_to_quaternion` → `EulerToQuaternion.srv`
  - `/quaternion_to_euler` → `QuaternionToEuler.srv`
- **`arduinobot_msgs`**: define las interfaces `.srv` usadas por el nodo.

---

## 🧭 Resumen (TRANSCRIPTION + CLASS NOTES)
- Se crea **`arduinobot_utils`** con build type **`ament_cmake`** para mezclar **C++** y **Python**.
- Para el código Python se crea la carpeta **`arduinobot_utils/`** (mismo nombre del paquete) y `__init__.py`.
- Se implementa el nodo **`angle_conversion.py`**, que registra **dos servicios**:
  - **Euler → Cuaternión** y **Cuaternión → Euler**.
- Dependencias para las conversiones:
  - `tf-transformations` (paquete ROS): `sudo apt-get install ros-<distro>-tf-transformations`
  - `transforms3d` (PyPI): `sudo pip3 install transforms3d`
- Los servicios aparecen con `ros2 service list` y se prueban con `ros2 service call` (ver **Ejemplos**).

---

## 🚀 Instalación rápida

```bash
# 1) Crear paquete (si aún no existe)
ros2 pkg create --build-type ament_cmake arduinobot_utils

# 2) Instalar dependencias de conversión
sudo apt-get install ros-<distro>-tf-transformations
sudo pip3 install transforms3d

# 3) Compilar workspace
colcon build
. install/setup.bash    # (sourcer el workspace)

# 4) Ejecutar el nodo
ros2 run arduinobot_utils angle_conversion.py
```

> En el arranque verás un log tipo: `Angle Conversion Services are ready` y, con `ros2 service list`, los servicios `/euler_to_quaternion` y `/quaternion_to_euler`.

---

## 🧪 Ejemplos de uso

### 1) Euler → Cuaternión
```bash
ros2 service call /euler_to_quaternion arduinobot_msgs/srv/EulerToQuaternion "roll: -0.5
pitch: 0.0
yaw: 1.5"
```
Respuesta esperada (aprox.):
```text
x: -0.1810, y: -0.1686, z: 0.6604, w: 0.7089
```

### 2) Cuaternión → Euler
```bash
ros2 service call /quaternion_to_euler arduinobot_msgs/srv/QuaternionToEuler "x: 0.0
y: 0.0
z: 0.0
w: 1.0"
```
Respuesta esperada (radianes):
```text
roll: 0.0, pitch: 0.0, yaw: 0.0  # Identidad
```

> **Nota numérica:** `euler_from_quaternion` usa **radianes** y retorna ángulos envueltos (p. ej. `[-π, π]`). El cuaternión `(0,0,0,1)` es la **rotación identidad**; por eso `yaw=0`.

---

## 🗂️ Estructura mínima esperada
```
arduinobot_ws/
└── src/
    ├── arduinobot_msgs/
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   └── srv/
    │       ├── EulerToQuaternion.srv
    │       └── QuaternionToEuler.srv
    └── arduinobot_utils/
        ├── CMakeLists.txt
        ├── package.xml
        └── arduinobot_utils/
            ├── __init__.py
            └── angle_conversion.py
```

---

## 🧠 Pseudocódigo por script

### `arduinobot_utils/angle_conversion.py`
```text
INICIO
  importar rclpy, Node, servicios (EulerToQuaternion, QuaternionToEuler)
  importar utilidades TF: quaternion_from_euler, euler_from_quaternion

  clase AnglesConverter(Node):
    constructor:
      inicializar nodo ("angles_converter" o similar)
      registrar servicio "euler_to_quaternion" con callback eulerToQuaternionCallback
      registrar servicio "quaternion_to_euler" con callback quaternionToEulerCallback
      log "Angle Conversion Services are ready"

    eulerToQuaternionCallback(req, res):
      log "requested euler: (roll, pitch, yaw)"
      (x, y, z, w) = quaternion_from_euler(req.roll, req.pitch, req.yaw)
      colocar en res.x, res.y, res.z, res.w
      log "quaternion resultante"
      devolver res

    quaternionToEulerCallback(req, res):
      log "requested quaternion: (x, y, z, w)"
      (roll, pitch, yaw) = euler_from_quaternion((req.x, req.y, req.z, req.w))
      colocar en res.roll, res.pitch, res.yaw
      log "euler resultante"
      devolver res

  main():
    rclpy.init()
    crear nodo AnglesConverter
    rclpy.spin(nodo)  # atender solicitudes
    destruir nodo y shutdown
FIN
```

### `arduinobot_msgs/srv/EulerToQuaternion.srv`
```text
# Request
float64 roll
float64 pitch
float64 yaw
---
# Response
float64 x
float64 y
float64 z
float64 w
```

### `arduinobot_msgs/srv/QuaternionToEuler.srv`
```text
# Request
float64 x
float64 y
float64 z
float64 w
---
# Response
float64 roll
float64 pitch
float64 yaw
```

---

## 🛠️ Explicación de **CMakeLists.txt** (arduinobot_utils)

Puntos clave añadidos:
- `find_package(ament_cmake_python REQUIRED)`: utilidades CMake para instalar **paquetes Python** dentro de un paquete `ament_cmake`.
- `ament_python_install_package(${PROJECT_NAME})`: instala la **carpeta Python** con el mismo nombre del paquete (requiere `__init__.py`).
- `install(PROGRAMS ${PROJECT_NAME}/angle_conversion.py DESTINATION lib/${PROJECT_NAME})`: instala el **script ejecutable** (con *shebang* y permisos `+x`) en `install/lib/<paquete>` para que `ros2 run` lo encuentre.
- `find_package(rclpy REQUIRED)` y `find_package(arduinobot_msgs REQUIRED)`: dependencias del nodo.
- `ament_package()`: cierre estándar del paquete.

---

## 🛠️ Explicación de **CMakeLists.txt** (arduinobot_msgs)

Puntos clave añadidos (típicos para paquetes de interfaces):
- `find_package(rosidl_default_generators REQUIRED)`: macro generadora para `.srv`/`.msg`.
- `rosidl_generate_interfaces(${PROJECT_NAME}
    "srv/EulerToQuaternion.srv"
    "srv/QuaternionToEuler.srv"
  )`: declara y **genera** las interfaces de servicio.
- `ament_export_dependencies(rosidl_default_runtime)`: dependencias de ejecución para usar las interfaces generadas.
- `ament_package()`: cierre estándar.

> En algunos esqueletos también verás `rosidl_get_typesupport_target()` si otro target CMake requiere *linkear* contra los tipos generados.

---

## 📄 Explicación de **package.xml**

### `arduinobot_utils/package.xml`
- `buildtool_depend`
  - `ament_cmake`: build system base.
  - `ament_cmake_python`: soporte para instalar paquetes Python desde CMake.
- `depend`
  - `rclpy`: cliente ROS 2 para Python.
  - `arduinobot_msgs`: uso de las interfaces `.srv`.
- `<export><build_type>ament_cmake</build_type></export>`: declara tipo de construcción.

### `arduinobot_msgs/package.xml`
- `buildtool_depend` `ament_cmake` y `build_depend` `rosidl_default_generators`: necesarios para **generar** las interfaces.
- `exec_depend` `rosidl_default_runtime`: necesario en tiempo de ejecución para usar las interfaces generadas.
- Marca el paquete como contenedor de interfaces ROS 2.

---

## 🧰 Troubleshooting

- **Ángulos en radianes:** todas las conversiones usan radianes.
- **Cuaternión identidad:** `(0,0,0,1)` ⇒ **sin rotación** ⇒ `(0,0,0)`.
- **Normalización:** envía cuaterniones **normalizados**.
- **Permisos del script:** `chmod +x arduinobot_utils/angle_conversion.py` y *shebang* correcto.
- **Servicios no visibles:** confirma nodo corriendo y `source install/setup.bash` del workspace correcto.
