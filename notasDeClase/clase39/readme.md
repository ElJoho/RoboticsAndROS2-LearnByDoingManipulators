
# Clase 39. Visualización del modelo URDF en Rviz con ROS 2

En esta práctica se aplican los conceptos vistos en el curso para **visualizar el modelo URDF del robot en Rviz**.  
Para lograrlo, utilizamos nodos de ROS 2 que publican la descripción del robot y nos permiten manipular sus articulaciones a través de una interfaz gráfica.

---

## 1. Preparar el entorno
Antes de ejecutar cualquier comando en ROS 2 es necesario **cargar las configuraciones del workspace**:

```bash
. install/setup.bash
```

Este comando (`source`) asegura que las rutas del *workspace* queden configuradas en la sesión de terminal y que ROS 2 reconozca los paquetes y nodos instalados.

---

## 2. Publicar el modelo del robot

Ejecutamos el nodo `robot_state_publisher` que se encarga de **publicar la descripción URDF del robot en un tópico** de ROS 2.

```bash
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro ~/Documents/RoboticsAndROS2-LearnByDoingManipulators/arduinobot_ws/src/arduinobot_description/urdf/arduinobot.urdf.xacro)"
```

* `ros2 run <paquete> <ejecutable>`: corre un nodo de un paquete.
* `--ros-args -p robot_description:=...`: asigna el parámetro `robot_description`.
* `xacro ...`: convierte el archivo `.xacro` en un archivo URDF válido en tiempo de ejecución.

El nodo muestra en consola los segmentos (links) del robot que fueron cargados, confirmando que la descripción se publicó correctamente.

---

## 3. Controlar las articulaciones

Abrimos una nueva terminal, volvemos a **cargar el entorno** (`. install/setup.bash`) y ejecutamos el nodo `joint_state_publisher_gui`:

```bash
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```

Este nodo abre una interfaz gráfica con **deslizadores para mover las articulaciones** del robot.
Inicialmente espera a que el parámetro `robot_description` esté disponible, publicado por el `robot_state_publisher`.

> Nota: en caso de escribir mal el nombre del ejecutable (`join_state_publisher_gui`), ROS 2 devolverá el error *No executable found*.

---

## 4. Visualizar el robot en Rviz

En otra terminal ejecutamos:

```bash
ros2 run rviz2 rviz2
```

Esto abre la ventana de Rviz, donde:

1. Se debe configurar el **Fixed Frame** como `world` (el primer frame definido en el modelo).
2. Se añaden dos visualizaciones:

   * **TF** → muestra la posición y orientación de cada link.
   * **Robot Model** → renderiza la geometría del robot leyendo el tópico `robot_description`.

Ahora, al mover los deslizadores en la ventana de `joint_state_publisher_gui`, el modelo del robot se mueve en Rviz.

---

## 5. Guardar configuración en Rviz

Podemos guardar la configuración de visualización (TF y Robot Model) en un archivo `.rviz` para no repetir los pasos cada vez:

1. En Rviz ir a: `File → Save Config As`.
2. Guardar el archivo como:

```
arduinobot_ws/src/arduinobot_description/rviz/display.rviz
```

---

## Conclusión

* Se usaron **tres terminales**:

  1. `robot_state_publisher` → publica el modelo del robot.
  2. `joint_state_publisher_gui` → permite manipular las articulaciones.
  3. `rviz2` → visualiza el modelo y sus movimientos.

En siguientes lecciones se verá cómo simplificar todos estos pasos usando un **archivo de lanzamiento (launch file)** para ejecutar todo con un solo comando.
