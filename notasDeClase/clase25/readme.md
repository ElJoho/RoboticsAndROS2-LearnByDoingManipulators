#25. Create and Activate a Workspace

## 1. Colcon build
Usar el comando
´´´
colcon build
´´´
para inicializar el workspace.
NOTA: ES IMPORNTANTE QUE COLCON BUILD SE HAGA EN LA CARPETA INICIAL DEL WORKSPACE Y NUNCA DENTRO DE SRC U OTRA CARPETA INTERNA.

## 2. Creando un paquete 
### 2.1 Creando un paquete de python
Irse a la carpeta src dentro del workspace y poner el comando:

´´´
ros2 pkg create --build-type ament_python arduinobot_py_examples 
´´´
### 2.2 Creando un paquete de C++
Irse a la carpeta src dentro del workspace y poner el comando 

´´´
ros2 pkg create --build-type ament_cmake arduinobot_cpp_examples
´´´

### 2.3 Usar colcon build para crear los paquetes
volverse a la carpeta raiz del workspace
´´´
colcon build
´´´
### 2.4 Activar workspace
Para hacer que el workspace actual (arduinobot_ws) se reconozca como un overlay para ros2 primero irse a la carpeta install y luego poner el comando de setup.bash
´´´
cd install/
. setup.bash
´´´
### 2.5 Comprobar que el paquete esta en el listado
Usar el siguiente comando:
´´´
ros2 pkg list
´´´
Revisar que los paquetes que hemos creado estan en el listado. Debera aparecer arduinobot_py_example y arduinobot_cpp_examples

Nota: El source command que se uso para hacer que el ambiente sea consciente del nuevo overlay solo se aplica para el terminal actual en que se uso el comando.