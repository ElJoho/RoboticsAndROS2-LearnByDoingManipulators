# Clase 29 `simple_subscriber.cpp`

## 1. Estructura de `simple_subscriber.cpp` 
```pseudocode
INICIO

CLASE SimpleSubscriber (hereda de Node)
    MÉTODO __init__():
        - Llama al constructor de Node con el nombre "simple_subscriber"
        - Define la suscripción al tópico "chatter"
            • Tipo de mensaje: std_msgs/String
            • Cola de mensajes: 10
            • Callback: msgCallback

    MÉTODO msgCallback(msg):
        - Recibe el mensaje publicado en "chatter"
        - Imprime en consola: "I heard: <contenido del mensaje>"

FUNCIÓN main():
    - Inicializa el sistema de ROS2
    - Crea un nodo de tipo SimpleSubscriber
    - Mantiene el nodo activo con spin (esperando mensajes)
    - Al finalizar, destruye el nodo y apaga ROS2
    - Retorna 0

FIN
```

## 2. Añadir el ejecutable, dependencias e instalación en CMakeLists

```txt
# 1. DEPENDENCIAS
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# 2. EJECUTABLES
add_executable(simple_publisher src/simple_publisher.cpp)
add_executable(simple_subscriber src/simple_subscriber.cpp) # <-- Añadir

# 3. DEPENDENCIAS
ament_target_dependencies(simple_publisher rclcpp std_msgs)
ament_target_dependencies(simple_subscriber rclcpp std_msgs) # <-- Añadir

# 4. INSTALACIÓN
install(TARGETS
  simple_publisher
  simple_subscriber # <-- Añadir
  DESTINATION lib/${PROJECT_NAME}
)
```

**Nota:** No es necesario modificar `package.xml` porque no se añadieron nuevas librerías.

## 3. Construir y ejecutar

```bash
colcon build --packages-select arduinobot_cpp_examples
. install/setup.bash
```

## 4. Comprobar el funcionamiento del subscriber

```bash
ros2 run arduinobot_cpp_examples simple_subscriber
```

En otro terminal:

```bash
ros2 topic list
ros2 topic info /chatter --verbose
ros2 topic pub /chatter std_msgs/msg/String "data: 'Hello World'"
```

El último comando debería hacer que en el primer terminal se imprima **Hello World**.

## 5. ROS2 es agnóstico

Un sistema operativo agnóstico es aquel en el que sus nodos no dependen de un lenguaje de programación específico para comunicarse entre ellos.
Para comprobarlo, se puede correr el nodo `simple_publisher.py` y `simple_subscriber.cpp`, y verá sus mensajes:

```bash
ros2 run arduinobot_py_examples simple_publisher
```