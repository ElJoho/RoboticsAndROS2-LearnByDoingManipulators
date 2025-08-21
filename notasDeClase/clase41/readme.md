# Clase 40 – Visualizar el Robot con Launch Files (`display.launch.py`)

En esta lección creamos nuestro primer **launch file** en ROS 2 para visualizar el modelo URDF del robot en **RViz2**.  
El objetivo es arrancar todos los nodos necesarios con un solo comando y en una sola terminal.

---

## 📂 Estructura del paquete

Dentro del paquete `arduinobot_description` se añadió una carpeta llamada `launch` que contiene el archivo:

- `display.launch.py` → archivo principal para lanzar los nodos.

---

## 📜 Pseudocódigo de `display.launch.py`

```pseudocode
INICIO

Importar librerías necesarias (launch, launch_ros, os, ament_index)

FUNCIÓN generate_launch_description():
    Declarar argumento "model" con ruta por defecto al archivo `arduinobot.urdf.xacro`

    Crear variable robot_description:
        - Ejecutar comando `xacro` sobre el modelo
        - Convertir archivo Xacro en URDF
        - Guardar como parámetro

    Crear nodo robot_state_publisher:
        - Publica la descripción URDF en /robot_description
        - Publica los frames del robot en TF

    Crear nodo joint_state_publisher_gui:
        - Abre ventana con sliders para mover articulaciones

    Crear nodo rviz2:
        - Lanza RViz2
        - Carga configuración inicial desde `rviz/display.rviz`

    DEVOLVER LaunchDescription con:
        - Argumento "model"
        - Nodo robot_state_publisher
        - Nodo joint_state_publisher_gui
        - Nodo rviz2

FIN
```
---

## 📚 Librerías y dependencias

### En el código (`display.launch.py`)

* `launch` → Para crear `LaunchDescription` y manejar argumentos.
* `launch_ros` → Para lanzar nodos de ROS 2 (`Node`).
* `ament_index_python` → Para obtener rutas de los paquetes.
* `os` → Para construir rutas de archivos en Python.

### En `CMakeLists.txt`

Se añadieron nuevas librerías y directorios a instalar:

```cmake
find_package(urdf REQUIRED)

install(
  DIRECTORY meshes urdf launch rviz
  DESTINATION share/${PROJECT_NAME}
)
```

👉 Esto asegura que las carpetas `launch` y `rviz` se instalen junto con `meshes` y `urdf`.

### En `package.xml`

Se añadieron dependencias de ejecución necesarias:

```xml
<exec_depend>urdf</exec_depend>
<exec_depend>xacro</exec_depend>
<exec_depend>ros2launch</exec_depend>
<exec_depend>robot_state_publisher</exec_depend>
<exec_depend>joint_state_publisher_gui</exec_depend>
<exec_depend>rviz2</exec_depend>
```

Estas permiten ejecutar los nodos relacionados con el URDF, procesar Xacro, lanzar archivos y visualizar en RViz2.

---

## 💻 Comandos utilizados en la terminal

1. **Compilar el workspace:**

   ```bash
   colcon build
   ```

2. **Configurar el entorno en una nueva terminal:**

   ```bash
   . install/setup.bash
   ```

3. **Ejecutar el launch file:**

   ```bash
   ros2 launch arduinobot_description display.launch.py
   ```

Con este único comando se lanzan todos los nodos y se abre RViz2 mostrando el modelo del robot listo para ser manipulado.
