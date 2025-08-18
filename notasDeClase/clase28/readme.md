
# Clase 28: `simple_subscriber.py`

## 1. Crear el archivo `simple_subscriber.py`
La estructura a seguir es la siguiente:

```pseudocode
INICIO

Importar librerías ROS2 y tipo de mensaje String

CLASE SimpleSubscriber (hereda de Node)
    MÉTODO __init__():
        - Inicializa nodo "simple_subscriber"
        - Crea suscripción al tópico "chatter" con callback msgCallback

    MÉTODO msgCallback(msg):
        - Imprime en consola el contenido del mensaje recibido

FUNCIÓN main():
    - Inicializa ROS2
    - Crea nodo SimpleSubscriber
    - Ejecuta el nodo escuchando mensajes
    - Destruye nodo y cierra ROS2

SI este archivo es el principal:
    - Llamar a main()

FIN
```

---

## 2. Añadir el ejecutable en `setup.py`

En `setup.py` añadir la siguiente línea dentro de `entry_points`:

```python
...
    entry_points={
        'console_scripts': [
            'simple_publisher = arduinobot_py_examples.simple_publisher:main',
            'simple_subscriber = arduinobot_py_examples.simple_subscriber:main' # <--- AÑADIR ESTO
        ],
    },
...
```

---

## 3. Construir y ejecutar

Ejecutar los siguientes comandos en el terminal para comprobar la funcionalidad:

```bash
colcon build
. install/setup.bash
ros2 run arduinobot_py_examples simple_subscriber
```

---

## 4. Comprobar funcionamiento del suscriptor

En otro terminal ejecutar:

```bash
colcon build
. install/setup.bash
ros2 topic list
ros2 topic info /chatter --verbose
```

---

## 5. Enviar mensajes al tópico con `ros2 topic pub`

Para enviar mensajes al tópico se usa la siguiente estructura del comando:

```bash
ros2 topic pub /nombre_topico std_msgs/msg/String "data: 'mensaje'"
```

En este caso se usa `std_msgs/msg/String` porque ese es el tipo de variable que está utilizando el suscriptor.
El comando para este caso sería el siguiente:

```bash
ros2 topic pub /chatter std_msgs/msg/String "data: 'Hello ROS2'"
```

En el terminal del suscriptor se observará el mensaje.

---

## 6. Independencia de lenguaje entre nodos

En un terminal distinto al del suscriptor ejecutar:

```bash
ros2 run arduinobot_cpp_examples simple_publisher
```

Se observará que el suscriptor, pese a estar escrito en **Python**, puede recibir los mensajes de un **publisher en C++**.
Esto demuestra que los nodos no son dependientes del lenguaje en el que están escritos para comunicarse entre sí.

```
