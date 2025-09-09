# Clase 63 — Servidor de Servicio en C++ (`AddTwoInts`) con ROS 2

Este `README.md` resume lo realizado en la clase: implementación de un **servidor de servicio en C++**
que atiende `/add_two_ints` y suma dos enteros, además de los **comandos** usados para verificarlo con la CLI de ROS 2.
También explica qué hace cada archivo relevante del paquete C++.

---

## Objetivos
- Crear un nodo C++ que expone el servicio `/add_two_ints` (tipo `arduinobot_msgs/srv/AddTwoInts`).
- Atender solicitudes y devolver `sum = a + b`.
- Verificar el servicio con `ros2 service list`, `ros2 service type` y `ros2 service call`.

---

## Comandos usados en clase

> Abre **dos terminales**. En cada una, recuerda **sourc‍ear** el workspace tras compilar:
> ```bash
> . install/setup.bash
> ```

### Terminal 1 — Ejecutar el servidor
```bash
ros2 run arduinobot_cpp_examples simple_service_server
```
Salida esperada (ejemplo):
```
[INFO] [...] [rclcpp]: Service add_two_ints is Ready
[INFO] [...] [rclcpp]: New Request received a: 7 b: 5
[INFO] [...] [rclcpp]: Returning sum: 12
```

### Terminal 2 — Inspeccionar y llamar al servicio
```bash
ros2 service list
ros2 service type /add_two_ints
ros2 service call /add_two_ints arduinobot_msgs/srv/AddTwoInts "a: 7
b: 5"
```
Salida esperada:
```
requester: making request: arduinobot_msgs.srv.AddTwoInts_Request(a=7, b=5)

response:
arduinobot_msgs.srv.AddTwoInts_Response(sum=12)
```

> **Tip YAML:** también puedes llamar en una sola línea:
> ```bash
> ros2 service call /add_two_ints arduinobot_msgs/srv/AddTwoInts "{a: 7, b: 5}"
> ```

---

## ¿Qué hace cada archivo?

### `simple_service_server.cpp` (paquete `arduinobot_cpp_examples/src/`)
- Incluye `rclcpp` y la interfaz `arduinobot_msgs/srv/AddTwoInts`.
- Declara la clase `SimpleServiceServer` que hereda de `rclcpp::Node` y, en su constructor:
  - Crea el servidor con `create_service<...>("add_two_ints", std::bind(...))`.
  - Enlaza el **callback** con `std::bind` usando `_1` y `_2` para **Request** y **Response**.
  - Emite logs informativos cuando el servicio está listo y cuando llega una solicitud.
- `serviceCallback(req, res)`:
  - Lee `req->a` y `req->b`, calcula `res->sum = a + b` y registra el resultado.
- `main()`:
  - Inicializa ROS 2 (`rclcpp::init(argc, argv)`), crea el nodo (`std::make_shared<...>()`),
    lo mantiene ejecutándose (`rclcpp::spin(node)`) y finalmente cierra (`rclcpp::shutdown()`).

### `CMakeLists.txt` (paquete `arduinobot_cpp_examples/`)
- Declara dependencias de **build** con `find_package(...)` (p. ej., `rclcpp`, `arduinobot_msgs`).  
- Define el ejecutable:
  ```cmake
  add_executable(simple_service_server src/simple_service_server.cpp)
  ament_target_dependencies(simple_service_server rclcpp arduinobot_msgs)
  ```
- Instala el binario en `lib/${PROJECT_NAME}` para que `ros2 run` lo encuentre:
  ```cmake
  install(TARGETS simple_service_server DESTINATION lib/${PROJECT_NAME})
  ```

### `package.xml` (paquete `arduinobot_cpp_examples/`)
- Metadatos y dependencias de tiempo de compilación/ejecución:
  - `ament_cmake` como build tool.
  - `rclcpp`, `std_msgs`, `rcl_interfaces`, `arduinobot_msgs` como dependencias.
- Sección `<export>` con `<build_type>ament_cmake</build_type>`.

---

## Compilación rápida
Desde la **raíz** del workspace:
```bash
colcon build
. install/setup.bash
```
> Si limpiaste el workspace, es normal ver *warnings* de rutas inexistentes **antes** de recompilar.
> Tras `colcon build` y volver a hacer `source`, desaparecen.

---

## Estructura mínima (referencial)
```
arduinobot_ws/
└── src/
    └── arduinobot_cpp_examples/
        ├── CMakeLists.txt
        ├── package.xml
        └── src/
            └── simple_service_server.cpp
```

---

## Solución de problemas
- **No aparece `/add_two_ints` en `ros2 service list`:**
  - Verifica que el nodo del servidor esté corriendo en otra terminal.
  - Asegúrate de haber hecho `. install/setup.bash` en **cada** terminal.
- **Fallo al llamar al servicio (YAML):**
  - Usa comillas y formato correcto: `"a: 7
b: 5"` o `{a: 7, b: 5}`.
- **Errores de link o dependencias:**
  - Revisa que `arduinobot_msgs` esté instalado en el workspace y que `CMakeLists.txt`/`package.xml`
    declaren las dependencias correctas. Vuelve a compilar.
