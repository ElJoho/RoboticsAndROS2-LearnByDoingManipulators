# Clase 62 — Servidor de Servicio en Python (`AddTwoInts`) con ROS 2

Este `README.md` documenta lo realizado en la clase: creación del **paquete de interfaces** `arduinobot_msgs`, implementación del **servidor de servicio** en Python (`simple_service_server.py`) y la ejecución/validación desde la CLI de ROS 2. Incluye también qué contiene cada archivo (`AddTwoInts.srv`, `CMakeLists.txt`, `package.xml`, `setup.py`) y **todos los comandos** utilizados.

---

## Objetivos
- Definir una **interfaz de servicio** (`AddTwoInts.srv`) para sumar dos enteros.
- Generar e instalar las interfaces con `rosidl`.
- Implementar un **nodo servidor** en Python que atienda `/add_two_ints`.
- Comprobar el servicio con la **línea de comandos**: listar, ver tipo y llamar al servicio.

---

## Comandos usados en clase (paso a paso)

### 1) Crear e instalar el paquete de interfaces
```bash
# En la raíz del workspace (arduinobot_ws)
ros2 pkg create --build-type ament_cmake arduinobot_msgs
colcon build
. install/setup.bash
```

### 2) Compilar todo el workspace (después de añadir archivos)
```bash
colcon build
. install/setup.bash
```

> Si aparece un error del tipo: `InvalidServiceSpecification: Could not find separator '---' between request and response`, revisa tu `AddTwoInts.srv` y asegúrate de que tenga exactamente **tres guiones** en una línea separando request y response (ver sección de archivos).

### 3) Ejecutar el servidor del servicio (Terminal 2)
```bash
ros2 run arduinobot_py_examples simple_service_server
```
**Salida esperada (ejemplo):**
```
[INFO] [...] [simple_service_server]: Service add_two_ints Ready
```

### 4) Inspeccionar y llamar el servicio (Terminal 3)
```bash
ros2 service list
ros2 service type /add_two_ints
ros2 service call /add_two_ints arduinobot_msgs/srv/AddTwoInts "a: 7
b: 5"
```
**Salida esperada:**
```
requester: making request: arduinobot_msgs.srv.AddTwoInts_Request(a=7, b=5)

response:
arduinobot_msgs.srv.AddTwoInts_Response(sum=12)
```

---

## ¿Qué contiene cada archivo?

### 1) `AddTwoInts.srv` (paquete `arduinobot_msgs`)
Interfaz del servicio con **dos enteros** en la solicitud y **un entero** en la respuesta:
```srv
int64 a
int64 b
---
int64 sum
```
- La línea `---` debe estar **sola** y sin espacios. Si falta, `colcon build` fallará con el error de separador.
- Ubicación típica: `arduinobot_msgs/srv/AddTwoInts.srv`.

### 2) `CMakeLists.txt` (paquete `arduinobot_msgs`)
Declara dependencias y genera las interfaces a partir del `.srv`:
```cmake
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/AddTwoInts.srv"
)

ament_package()
```
Este archivo invoca `rosidl_generate_interfaces` sobre `srv/AddTwoInts.srv`, habilitando la generación de código para C++/Python según el entorno. fileciteturn0file0

> **Sugerencia (opcional):** muchos ejemplos incluyen `ament_export_dependencies(rosidl_default_runtime)` para facilitar el uso del paquete por terceros en tiempo de ejecución.

### 3) `package.xml` (paquete `arduinobot_msgs`)
Declara metadatos y dependencias de **build** y **runtime** para las interfaces:
```xml
<buildtool_depend>ament_cmake</buildtool_depend>
<build_depend>rosidl_default_generators</build_depend>
<depend>std_msgs</depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```
Con esto, `colcon` sabe que debe **generar** e **instalar** los artefactos del servicio. fileciteturn0file1

### 4) `simple_service_server.py` (paquete `arduinobot_py_examples`)
Nodo Python que crea un **servidor** con `create_service`:
- **Tipo de servicio**: `AddTwoInts` (importado desde `arduinobot_msgs.srv`).
- **Nombre del servicio**: `/add_two_ints`.
- **Callback**: recibe `a` y `b`, calcula `sum` y la devuelve.
- **Logs**: imprime “Service add_two_ints Ready”, “New Message Received…”, “Returning sum…”.

Fragmento relevante:
```python
self.service_ = self.create_service(
    AddTwoInts,
    "add_two_ints",
    self.serviceCallback
)
```
Y el callback:
```python
def serviceCallback(self, req, res):
    res.sum = req.a + req.b
    return res
```
El `main()` inicializa ROS 2, instancia el nodo y lo mantiene con `rclpy.spin(...)`. fileciteturn0file2

### 5) `setup.py` (paquete `arduinobot_py_examples`)
Declara el **entry point** para lanzar el nodo desde `ros2 run`. Debe incluir algo como:
```python
entry_points={
    'console_scripts': [
        'simple_service_server = arduinobot_py_examples.simple_service_server:main',
    ],
}
```
- Asegúrate de que la ruta del módulo y el nombre de la función (`main`) coincidan con tu archivo Python.
- Tras modificar `setup.py`, ejecuta `colcon build` y vuelve a hacer `source` del `setup.bash`.

---

## Errores frecuentes y solución

- **`InvalidServiceSpecification: Could not find separator '---' between request and response`**  
  El `.srv` carece de la línea `---` o tiene espacios extra. Corrígelo a:
  ```
  int64 a
  int64 b
  ---
  int64 sum
  ```
  Luego **limpia y recompila**:
  ```bash
  rm -rf build/ install/ log/
  colcon build
  . install/setup.bash
  ```

- **Advertencias `AMENT_PREFIX_PATH`/`CMAKE_PREFIX_PATH` tras limpiar**  
  Son normales justo antes de recompilar: desaparecen tras `colcon build` exitoso y nuevo `source`.

- **`ros2 service call` con YAML**  
  Recuerda las comillas multilínea o una sola línea válida:
  ```bash
  ros2 service call /add_two_ints arduinobot_msgs/srv/AddTwoInts "a: 7
  b: 5"
  # o
  ros2 service call /add_two_ints arduinobot_msgs/srv/AddTwoInts "{a: 7, b: 5}"
  ```

---

## Verificación rápida

1. **Servidor activo** (Terminal 2):
   ```bash
   ros2 run arduinobot_py_examples simple_service_server
   ```
   Deberías ver: `Service add_two_ints Ready`.

2. **Listar y llamar** (Terminal 3):
   ```bash
   ros2 service list
   ros2 service type /add_two_ints
   ros2 service call /add_two_ints arduinobot_msgs/srv/AddTwoInts "{a: 7, b: 5}"
   ```

---

## Estructura mínima de carpetas (referencial)
```
arduinobot_ws/
└── src/
    ├── arduinobot_msgs/
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   └── srv/
    │       └── AddTwoInts.srv
    └── arduinobot_py_examples/
        ├── setup.py
        └── arduinobot_py_examples/
            └── simple_service_server.py
```