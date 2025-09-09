# Cliente de servicio en C++ (`simple_service_client.cpp`)

Este repositorio contiene el código y los pasos vistos en la **Clase 66**, donde creamos un *cliente de servicio* en ROS 2 (C++) que llama al servidor `add_two_ints` para sumar dos enteros.

---

## 🧭 Objetivo de la clase

* Implementar un **nodo cliente** en C++ que:

  * Espere a que el servicio `/add_two_ints` esté disponible.
  * Envíe una **request** con dos enteros `a` y `b`.
  * Reciba la **response** con el campo `sum`.
* Integrarlo al paquete CMake y ejecutarlo desde la CLI de ROS 2.

---

## 📦 Archivos relevantes

### `src/simple_service_client.cpp`

Cliente en C++. Puntos clave:

* Incluye `rclcpp` y la interfaz `arduinobot_msgs/srv/AddTwoInts`.
* Define la clase `SimpleServiceClient : public rclcpp::Node`.
* Crea el cliente con `create_client<AddTwoInts>("add_two_ints")`.
* **Espera** al servicio con `wait_for_service`.
* Construye la `Request` (`a`, `b`) y la envía con `async_send_request`.
* Usa un **callback** para imprimir el resultado (`sum`).
* En `main`, valida argumentos, instancia el nodo y hace `spin`.

### `CMakeLists.txt`

* Declara dependencias (`rclcpp`, `arduinobot_msgs`, etc.).
* Compila y **instala** el ejecutable `simple_service_client`.
* Fragmentos relevantes (ya presentes):

  * `add_executable(simple_service_client src/simple_service_client.cpp)`
  * `ament_target_dependencies(simple_service_client rclcpp arduinobot_msgs)`
  * Instalación bajo `lib/${PROJECT_NAME}`.&#x20;

---

## 🧩 Interfaz del servicio

* Tipo: `arduinobot_msgs/srv/AddTwoInts`
* **Request**: `int64 a`, `int64 b`
* **Response**: `int64 sum`

> Asegúrate de tener el paquete `arduinobot_msgs` correctamente compilado (el `.srv` con el separador `---`).

---

## ▶️ Comandos usados en clase

### 1) Compilar el workspace

```bash
colcon build
```

### 2) Cargar el entorno

```bash
source install/setup.bash
```

### 3) Lanzar el **servidor** (en una terminal)

```bash
ros2 run arduinobot_cpp_examples simple_service_server
```

Salida esperada (ejemplo):

```
[INFO] [rclcpp]: Service add_two_ints is Ready
```

### 4) Ejecutar el **cliente** (en otra terminal)

```bash
ros2 run arduinobot_cpp_examples simple_service_client 3 5
```

Salida esperada:

```
[INFO] [rclcpp]: Service Response8
```

> (El servidor mostrará que recibió `a: 3`, `b: 5` y devolvió `sum: 8`.)

---

## 🧠 Pseudocódigo del cliente (`simple_service_client.cpp`)

```text
Clase SimpleServiceClient(a, b):
  hereda de rclcpp::Node con nombre "simple_service_client"
  client_ ← create_client<AddTwoInts>("add_two_ints")

  mientras NO client_.wait_for_service(1s):
    log: "Service not available, waiting more time ..."
    si !rclcpp::ok():           # si ROS se detuvo
      log error y return

  request ← nueva AddTwoInts::Request
  request.a ← a
  request.b ← b

  client_.async_send_request(request, callback responseCallback)

Método responseCallback(future):
  si future válido:
    log: "Service Response: " + future.get()->sum
  si no:
    log error: "Service failure"

main(argc, argv):
  rclcpp::init(argc, argv)
  si argc != 3:
    log error de uso y return
  node ← make_shared<SimpleServiceClient>(atoi(argv[1]), atoi(argv[2]))
  rclcpp::spin(node)            # mantener el nodo para esperar la respuesta
  rclcpp::shutdown()
```

---

## 🛠️ Qué se cambió en el *build system*

En `CMakeLists.txt`:

* Se añadió el ejecutable **`simple_service_client`** y sus dependencias `rclcpp` y `arduinobot_msgs`.
* Se instaló el binario para poder ejecutar con `ros2 run`.&#x20;

> Si agregas o renombras archivos, recuerda volver a compilar con `colcon build` y **volver a hacer** `source install/setup.bash`.

---

## ✅ Verificación rápida

1. **Servidor** arriba:

   ```bash
   ros2 run arduinobot_cpp_examples simple_service_server
   ```
2. **Cliente** con argumentos:

   ```bash
   ros2 run arduinobot_cpp_examples simple_service_client 7 5
   ```
3. Debes ver la suma (`12`) en la salida del cliente y del servidor.

---

## 🧯 Problemas comunes

* **El cliente no imprime nada**: te falta `rclcpp::spin(node)` o el servidor no está corriendo.
* **Error al convertir argumentos**: asegúrate de pasar **dos enteros**: `simple_service_client A B`.
* **No encuentra el servicio**: revisa el nombre exacto `"add_two_ints"` y que el servidor esté listo.
* **Cambiaste CMake y no se ve**: vuelve a ejecutar `colcon build` y `source install/setup.bash`.

---

## 🗂️ Estructura sugerida del paquete

```
arduinobot_cpp_examples/
├─ CMakeLists.txt
└─ src/
   ├─ simple_service_server.cpp
   └─ simple_service_client.cpp
```

¡Listo! Con esto ya tienes el cliente en C++ integrándose con el servidor `add_two_ints` y ejecutándose desde la CLI de ROS 2.
