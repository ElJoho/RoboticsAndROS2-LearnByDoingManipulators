# Nodo `simple_publisher` usando Python

## 1. Crear `simple_publisher.py`
Desde la carpeta raíz del *workspace*, ir a `src`, luego a la carpeta del paquete de Python y, dentro de esta, a una carpeta con el mismo nombre que el paquete. Ahí crear el script que contendrá el nodo en Python, siguiendo la estructura:

```
workspace/src/paquete_python/paquete_python/nodo.py

```

En este caso:

```
arduoinobot_ws/src/arduinobot_py_examples/arduinobot_py_examples/simple_publisher.py

```

---

## 2. Modificar `setup.py`
En el archivo `setup.py`, ubicar la sección `console_scripts` y añadir el script con la estructura:

```
nombre_script = nombre_paquete.nombre_script:main

```

Ejemplo:

```python
'console_scripts': [
    'simple_publisher = arduinobot_py_examples.simple_publisher:main',
],
```

---

## 3. Modificar `package.xml`

Añadir las dependencias usadas en el nuevo script.
Si ya estaban añadidas, no es necesario volver a ponerlas.
Para este caso, se debe incluir `rclpy` y `std_msgs`:

```xml
<license>TODO: License declaration</license>

<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>

<test_depend>ament_copyright</test_depend>
```

---

## 4. Compilar el workspace

Ir a la raíz del *workspace* y compilar con:

```bash
colcon build
```

Luego, para que ROS 2 reconozca el paquete, ejecutar:

```bash
. install/setup.bash
```

---

## 5. Ejecutar el nodo

La estructura general para ejecutar un nodo es:

```bash
ros2 run nombre_paquete nombre_nodo
```

En este caso:

```bash
ros2 run arduinobot_py_examples simple_publisher
```

> 💡 **Tip:** Usa la tecla **Tab** para autocompletar nombres de paquetes o scripts.

---

## 6. Ver los tópicos activos

En otra terminal:

```bash
ros2 topic list
```

Deberías ver el tópico `/chatter` mientras el nodo esté corriendo.

---

## 7. Observar los mensajes del nodo

Para ver los mensajes que publica el nodo:

```bash
ros2 topic echo /nombre_topico
```

En este caso:

```bash
ros2 topic echo /chatter
```

---

## 8. Información del tópico

Para ver información de un tópico:

```bash
ros2 topic info /nombre_topico
```

Con más detalles:

```bash
ros2 topic info /nombre_topico --verbose
```

Ejemplo:

```bash
ros2 topic info /chatter
ros2 topic info /chatter --verbose
```

---

## 9. Frecuencia de publicación

Para ver la frecuencia con la que publica un nodo:

```bash
ros2 topic hz /nombre_topico
```

En este caso:

```bash
ros2 topic hz /chatter
```