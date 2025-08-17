
# Nodo `simple_publisher` usando C++

## 1. Crear `simple_publisher.cpp`

Desde la carpeta raíz del *workspace*, ir a `src`, luego a la carpeta del paquete de C++ y dentro de esta a `src`.
Ahí crear el script que contendrá el nodo en C++, siguiendo la estructura:

```
workspace/src/paquete_cpp/src/nodo.cpp
```

En este caso:

```
arduoinobot_ws/src/arduinobot_cpp_examples/src/simple_publisher.cpp
```

---

## 2. Modificar `CMakeLists.txt`

Dentro de la carpeta del paquete de C++ se encuentra el archivo `CMakeLists.txt`, el cual se debe modificar cada vez que se añade un script de un nodo.

### 2.1 Añadir librerías con `find_package()`

Se añaden las librerías de las dependencias que se están usando en el paquete de C++.
Estructura:

```cmake
find_package(nombre_libreria REQUIRED)
```

En este caso, se deben añadir las librerías **rclcpp** y **std\_msgs**.

### 2.2 Añadir ejecutable con `add_executable()`

Se añade el ejecutable del nodo.
Estructura:

```cmake
add_executable(nombre_nodo src/nombre_script_nodo.cpp)
```

En este caso:

```cmake
add_executable(simple_publisher src/simple_publisher.cpp)
```

### 2.3 Añadir dependencias con `ament_target_dependencies()`

Se añaden las dependencias específicas que utiliza el nodo.
Estructura:

```cmake
ament_target_dependencies(nombre_nodo nombre_dependencia1 nombre_dependencia2 ... nombre_dependenciaN)
```

En este caso:

```cmake
ament_target_dependencies(simple_publisher rclcpp std_msgs)
```

### 2.4 Añadir instalador con `install()`

Se especifica el instalador del ejecutable.
Estructura:

```cmake
install(TARGETS
    nombre_nodo1
    nombre_nodo2
    ...
    nombre_nodoN
    DESTINATION lib/${PROJECT_NAME}
)
```

### Ejemplo completo de `CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.8)
project(arduinobot_cpp_examples) # <--- PROJECT_NAME

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. LIBRERÍAS
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# 2. EJECUTABLES
add_executable(simple_publisher src/simple_publisher.cpp)

# 3. DEPENDENCIAS
ament_target_dependencies(simple_publisher rclcpp std_msgs)

# 4. INSTALADOR
install(TARGETS
  simple_publisher
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

---

## 3. Modificar `package.xml`

En la misma ruta que `CMakeLists.txt` se encuentra el archivo `package.xml`.
Aquí se añaden también las dependencias con el tag `<depend>`.

Ejemplo:

```xml
...
  <buildtool_depend>ament_cmake</buildtool_depend>

  <!-- DEPENDENCIAS -->
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  
  <test_depend>ament_lint_auto</test_depend>
...
```

---

## 4. Construir y ejecutar el paquete

Ejecutar en la raíz del *workspace*:

```bash
colcon build
. install/setup.bash
```

---

## 5. Probar la funcionalidad del nodo y el paquete

### 5.1 Ejecutar nodo

Estructura:

```bash
ros2 run nombre_paquete nombre_nodo
```

En este caso:

```bash
ros2 run arduinobot_cpp_examples simple_publisher
```

Salida esperada:

```output
[INFO] [1755455168.488316641] [simple_publisher]: Publishing at 1 Hz
```

### 5.2 Verificar tópicos

#### Listar tópicos

```bash
ros2 topic list
```

Deberá aparecer `/chatter`.

#### Escuchar mensajes (`echo`)

```bash
ros2 topic echo /chatter
```

Salida esperada:

```output
data: 'Hello ROS 2 - counter: 96'
---
data: 'Hello ROS 2 - counter: 97'
---
```

#### Información del tópico (`info`)

```bash
ros2 topic info /chatter
ros2 topic info /chatter --verbose
```

#### Frecuencia de publicación (`hz`)

```bash
ros2 topic hz /chatter
```

Ejemplo de salida:

```output
average rate: 1.000
  min: 1.000s max: 1.000s std dev: 0.00026s window: 3
average rate: 1.000
  min: 1.000s max: 1.000s std dev: 0.00027s window: 5
```

---

¿Quieres que te lo deje ahora mismo **en un archivo `.md` descargable** para que no tengas que copiar/pegar desde aquí?
