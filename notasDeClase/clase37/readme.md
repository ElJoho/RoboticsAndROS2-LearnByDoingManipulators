# Clase 37 `simple_parameter` (C++ / ROS 2)

Un nodo ROS 2 en C++ mínimo que declara parámetros configurables y reacciona a cambios de estos en tiempo de ejecución. Basado en la transcripción de clase, este README explica la estructura del nodo, las API de ROS 2 utilizadas y los cambios requeridos en **`CMakeLists.txt`** y **`package.xml`**.

---

## 1) Qué hace este nodo

* Declara dos parámetros al inicio:

  * `simple_int_param` (entero, valor por defecto `28`)
  * `simple_string_param` (cadena, valor por defecto `"Antonio"`)
* Registra un callback que se ejecuta cuando uno o más parámetros cambian en tiempo de ejecución.
* Imprime en el terminal los nuevos valores y confirma la actualización como exitosa.

---

## 2) Pseudocódigo de `simple_parameter.cpp`

```pseudocode
INICIO

INCLUIR rclcpp, rcl_interfaces/SetParametersResult, <string>, <vector>, <memory>

CLASE SimpleParameter : Node
  PÚBLICO:
    CONSTRUCTOR SimpleParameter():
      - Llama al constructor base Node con nombre "simple_parameter"
      - declare_parameter<int>("simple_int_param", 28)
      - declare_parameter<std::string>("simple_string_param", "Antonio")
      - param_callback_handle_ = add_on_set_parameters_callback(
          bind(this->paramChangeCallback, _1)
        )

  PRIVADO:
    MIEMBRO param_callback_handle_: SharedPtr a OnSetParametersCallbackHandle

    MÉTODO paramChangeCallback(parameters: vector<rclcpp::Parameter>) 
      result := rcl_interfaces::msg::SetParametersResult()
      PARA CADA param EN parameters:
        SI param.name == "simple_int_param" Y param.type == INTEGER:
          log "Param simple_int_param cambiado! Nuevo valor: <param.as_int()>"
          result.successful = true
        SI param.name == "simple_string_param" Y param.type == STRING:
          log "Param simple_string_param cambiado! Nuevo valor: <param.as_string()>"
          result.successful = true
      RETORNAR result

FUNCIÓN main(argc, argv):
  rclcpp::init(argc, argv)
  node := make_shared<SimpleParameter>()
  rclcpp::spin(node)
  rclcpp::shutdown()

FIN
```

---

## 3) Funciones y tipos utilizados (explicación detallada)

* **`rclcpp::Node`**
  Clase base para nodos en C++. Proporciona APIs para parámetros, logging y ejecución.

* **`declare_parameter<T>(nombre, valor_defecto)`**
  Declara un parámetro de tipo `T` y asigna un valor inicial.
  Usado aquí para:

  * `simple_int_param` (int, 28)
  * `simple_string_param` (string, "Antonio")

* **`add_on_set_parameters_callback(cb)`**
  Registra un callback que se ejecuta cuando se modifican parámetros. Retorna un `SharedPtr` que se debe guardar para mantener el callback activo.

* **`std::bind` y `std::placeholders::_1`**
  Permiten adaptar el método `paramChangeCallback` como callback válido para ROS 2.

* **`rclcpp::Parameter`**
  Representa un parámetro que cambia. Métodos:

  * `get_name()` → nombre
  * `get_type()` → tipo (`INTEGER`, `STRING`, etc.)
  * `as_int()`, `as_string()` → obtiene valor con el tipo correcto

* **`rcl_interfaces::msg::SetParametersResult`**
  Tipo de retorno del callback. Campo relevante:

  * `successful` → indica si la actualización fue aceptada.

* **Logging**

  * `get_logger()` obtiene el logger del nodo
  * `RCLCPP_INFO(...)` / `RCLCPP_INFO_STREAM(...)` imprimen mensajes en terminal

* **Ciclo de vida en `main`**

  * `rclcpp::init()`
  * `std::make_shared<SimpleParameter>()`
  * `rclcpp::spin(node)` → mantiene el nodo activo
  * `rclcpp::shutdown()`

---

## 4) Cambios en los archivos de construcción

### `CMakeLists.txt` (añadir nodo y dependencias)

**Cambios requeridos**:

* Incluir dependencia `rcl_interfaces`.
* Añadir ejecutable `simple_parameter`.
* Asociar dependencias correctas.
* Instalar el nuevo ejecutable.

```cmake
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)  # NUEVO

add_executable(simple_parameter src/simple_parameter.cpp)  # NUEVO

ament_target_dependencies(simple_parameter  rclcpp rcl_interfaces)

install(TARGETS
  simple_publisher
  simple_subscriber
  simple_parameter    # NUEVO
  DESTINATION lib/${PROJECT_NAME}
)
```

### `package.xml` (declarar dependencia en runtime)

**Agregar dependencia `rcl_interfaces`:**

```xml
<depend>rclcpp</depend>
<depend>std_msgs</depend>
<depend>rcl_interfaces</depend>  <!-- NUEVO -->
```

---

## 5) Compilar y ejecutar

```bash
# Desde la raíz del workspace
colcon build --packages-select arduinobot_cpp_examples
source install/setup.bash

# Ejecutar nodo
ros2 run arduinobot_cpp_examples simple_parameter
```

### Consultar y modificar parámetros desde terminal

```bash
# Listar parámetros
ros2 param list

# Obtener valores actuales
ros2 param get /simple_parameter simple_int_param
ros2 param get /simple_parameter simple_string_param

# Cambiar valores (dispara callback e imprime logs)
ros2 param set /simple_parameter simple_int_param 99
ros2 param set /simple_parameter simple_string_param "Hola"
```

---

## 6) Ubicación sugerida de archivos

```
arduinobot_ws/
└── src/
    └── arduinobot_cpp_examples/
        ├── CMakeLists.txt
        ├── package.xml
        └── src/
            └── simple_parameter.cpp
```