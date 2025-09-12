# Clase 71 — Servicio **Euler ⟷ Quaternion** en C++ (ROS 2)

Este documento resume lo realizado en la clase: creación de un **nodo C++** que ofrece **dos servicios ROS 2** para convertir orientaciones entre **ángulos de Euler (roll, pitch, yaw)** y **cuaterniones (x, y, z, w)** usando **TF2**. Se describen los archivos involucrados (`angle_conversion.cpp`, `CMakeLists.txt`, `package.xml`), el **pseudocódigo** del script, los **comandos** utilizados en terminal y consideraciones de compilación/ejecución.

---

## Objetivos
- Implementar un nodo C++ `angles_conversion_service` con **dos servidores de servicio**:
  - `/euler_to_quaternion`: (roll, pitch, yaw) → (x, y, z, w)
  - `/quaternion_to_euler`: (x, y, z, w) → (roll, pitch, yaw)
- Compilar e instalar el ejecutable `angle_conversion` dentro del paquete `arduinobot_utils`.
- Probar los servicios con `ros2 service call`.

---

## Estructura de archivos relevante
```
arduinobot_ws/
└── src/
    └── arduinobot_utils/
        ├── CMakeLists.txt
        ├── package.xml
        └── src/
            └── angle_conversion.cpp
```

---

## Pseudocódigo del nodo `angle_conversion.cpp`

> **Resumen:** el nodo crea dos servidores de servicio; cada callback usa TF2 para realizar la conversión y escribe mensajes informativos en el log.

```text
class AnglesConverter : rclcpp::Node
  constructor:
    Node("angles_conversion_service")
    euler_to_quaternion = create_service<EulerToQuaternion>(
      name="euler_to_quaternion",
      callback=eulerToQuaternionCallback)
    quaternion_to_euler = create_service<QuaternionToEuler>(
      name="quaternion_to_euler",
      callback=quaternionToEulerCallback)
    log("Angle conversion services are ready")

  eulerToQuaternionCallback(req, res):
    # req: roll, pitch, yaw
    log("Request to convert euler angles: roll=..., pitch=..., yaw=...")
    q = tf2::Quaternion()
    q.setRPY(req.roll, req.pitch, req.yaw)    # Construye cuaternión desde Euler (RPY)
    res.x = q.getX(); res.y = q.getY(); res.z = q.getZ(); res.w = q.getW()
    log("Corresponding quaternion x=..., y=..., z=..., w=...")

  quaternionToEulerCallback(req, res):
    # req: x, y, z, w
    log("Request to convert quaternion: x=..., y=..., z=..., w=...")
    q = tf2::Quaternion(req.x, req.y, req.z, req.w)
    R = tf2::Matrix3x3(q)                    # Matriz de rotación a partir del cuaternión
    R.getRPY(res.roll, res.pitch, res.yaw)   # Extrae Euler (roll, pitch, yaw)
    log("Corresponding Euler Angles roll=..., pitch=..., yaw=...")

main():
  rclcpp::init()
  node = new AnglesConverter()
  rclcpp::spin(node)
  rclcpp::shutdown()
```

**Notas técnicas importantes**
- TF2: se incluyen `tf2/LinearMath/Quaternion.h` y `tf2/LinearMath/Matrix3x3.h` para `tf2::Quaternion` y `tf2::Matrix3x3`.
- Convención RPY: `setRPY(roll, pitch, yaw)` usa radianes y convención intrínseca ZYX común en TF (verificar consistencia en tu proyecto).

---

## Comandos utilizados y explicación

### 1) Crear paquete y compilar
```bash
ros2 pkg create --build-type ament_cmake arduinobot_utils
colcon build
. install/setup.bash
```
- `ros2 pkg create ...`: genera la plantilla del paquete `arduinobot_utils` con tipo **ament_cmake**.
- `colcon build`: compila todos los paquetes del workspace.
- `. install/setup.bash`: **sourc ea** el workspace para exportar ejecutables, mensajes y rutas.

### 2) Ejecutar el nodo
```bash
ros2 run arduinobot_utils angle_conversion
```
- Lanza el ejecutable `angle_conversion` (nuestro nodo). En el log se espera:
```
[INFO] ... Angle conversion services are ready
```

### 3) Ver servicios disponibles
```bash
ros2 service list
```
Salida esperada (entre otros):
```
/euler_to_quaternion
/quaternion_to_euler
```

### 4) Llamar a los servicios

**Euler → Quaternion**
```bash
ros2 service call /euler_to_quaternion arduinobot_msgs/srv/EulerToQuaternion "roll: -0.5
pitch: 0.0
yaw: 1.5"
```
Respuesta típica:
```text
x: -0.1810227231, y: -0.1686401280, z: 0.6604482617, w: 0.7089424339
```

**Quaternion → Euler**
```bash
ros2 service call /quaternion_to_euler arduinobot_msgs/srv/QuaternionToEuler "x: 0.0
y: 0.0
z: 0.0
w: 1.0"
```
Respuesta típica:
```text
roll: 0.0, pitch: -0.0, yaw: 0.0
```

> **Tip:** al escribir la petición YAML, puedes usar **doble tab** después del tipo de servicio para autocompletar el prototipo del mensaje; asegúrate de las **comillas** y los **saltos de línea** correctos.

---

## Cambios en `CMakeLists.txt` (qué y para qué)

Puntos clave que deben estar presentes para compilar este nodo C++:

- **Dependencias ament y bibliotecas C++:**
  ```cmake
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(arduinobot_msgs REQUIRED)
  find_package(tf2 REQUIRED)
  ```
- **Ejecutable y dependencias:**
  ```cmake
  add_executable(angle_conversion src/angle_conversion.cpp)
  ament_target_dependencies(angle_conversion rclcpp arduinobot_msgs tf2)
  ```
- **Instalación del ejecutable:**
  ```cmake
  install(TARGETS angle_conversion DESTINATION lib/${PROJECT_NAME})
  ```

**Sobre las secciones Python (opcionales):**  
Si en tu paquete **no** hay paquete/script Python, elimina/omite:
```cmake
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS ${PROJECT_NAME}/angle_conversion.py DESTINATION lib/${PROJECT_NAME})
```
Estas líneas son válidas **solo si** existe un paquete Python `arduinobot_utils/` con `__init__.py` y un script ejecutable `angle_conversion.py` (con shebang y permisos +x).

**Sugerencia adicional (estándar C++):**
```cmake
# target_compile_features(angle_conversion PUBLIC cxx_std_17)
```
No es obligatorio, pero hace la configuración más explícita.

---

## Cambios en `package.xml` (qué y para qué)

Para un nodo C++ como este, lo mínimo recomendado es declarar:
```xml
<buildtool_depend>ament_cmake</buildtool_depend>

<depend>rclcpp</depend>
<depend>tf2</depend>
<depend>arduinobot_msgs</depend>
```
- `ament_cmake`: sistema de build.
- `rclcpp`: API de ROS 2 en C++.
- `tf2`: utilidades de transformación y tipos de rotación (Quaternion/Matrix3x3).
- `arduinobot_msgs`: paquete donde se definen los **servicios** `EulerToQuaternion` y `QuaternionToEuler`.

**Si no usas Python**, no declares `rclpy` ni `ament_cmake_python` en este paquete (no aportan a este ejecutable C++).  
**Ojo a los typos:** usar `rclcpp` (con **cpp**), no `rclpp`.

---

## Salidas de referencia (de clase)

```
[INFO] [...] Angle conversion services are ready
[INFO] [...] Request to convert euler angles roll: -0.5, pitch: 0, yaw: 1.5 into a quaternion
[INFO] [...] Corresponding quaternion x: -0.181023, y: -0.16864, z: 0.660448, w: 0.708942
[INFO] [...] Request to convert quaternions x: 0, y: 0 , z: 0 , w: 1 into euler angles
[INFO] [...] Corresponding Euler Angles roll: 0, pitch: -0, yaw: 0
```

---

## Problemas comunes y cómo evitarlos
- **Dos servicios con el mismo nombre:** asegúrate de exponer **`euler_to_quaternion`** y **`quaternion_to_euler`** con nombres **distintos**.
- **Headers TF2 incorrectos:** usa `tf2/LinearMath/Quaternion.h` y `tf2/LinearMath/Matrix3x3.h`.
- **Secciones Python en CMake sin archivos Python:** si no existe `arduinobot_utils/__init__.py` o `angle_conversion.py`, elimina las líneas de instalación Python para evitar fallos de `colcon build`.
- **Typo en dependencias XML:** revisa que sea `rclcpp` (no `rclpp`).