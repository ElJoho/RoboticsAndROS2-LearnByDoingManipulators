# Clase 36: Nodo `simple_parameter.py` en ROS2

Este ejemplo muestra cómo crear un nodo en **ROS2 (Humble)** que utiliza **parámetros configurables** para modificar su comportamiento en tiempo de ejecución.  
El paquete utilizado es **`arduinobot_py_examples`** y el nodo implementado se llama `simple_parameter`.

---

## 1. Objetivo de la clase

- Aprender a **declarar parámetros** en un nodo de ROS2.
- Comprender cómo un nodo puede reaccionar cuando **se modifican parámetros en tiempo de ejecución**.
- Integrar la funcionalidad en los archivos de configuración (`setup.py` y `package.xml`).

---

## 2. Estructura en pseudocódigo

```pseudocode
INICIO

CLASE SimpleParameter (hereda de Node)
    MÉTODO __init__():
        - Llama al constructor de Node con nombre "simple_paramaeter"
        - Declara parámetro entero simple_int_param con valor por defecto 28
        - Declara parámetro string simple_string_param con valor por defecto "Anatonio"
        - Registra un callback paramChangeCallback para detectar cambios

    MÉTODO paramChangeCallback(params):
        - Crea objeto resultado SetParametersResult
        - Para cada parámetro modificado:
            • Si es simple_int_param y es entero → mostrar valor nuevo en consola y marcar éxito
            • Si es simple_string_param y es string → mostrar valor nuevo en consola y marcar éxito
        - Retornar resultado

FUNCIÓN main():
    - Inicializa ROS2
    - Crea nodo SimpleParameter
    - Mantiene el nodo en ejecución escuchando cambios
    - Al terminar, destruye el nodo y apaga ROS2

FIN
```

---

## 3. Funciones principales usadas

* **`self.declare_parameter(nombre, valor)`**
  Declara un parámetro dentro del nodo con un nombre y un valor por defecto.

* **`self.add_on_set_parameters_callback(función)`**
  Registra una función *callback* que se ejecuta cuando un parámetro declarado cambia en tiempo de ejecución.

* **`SetParametersResult`**
  Estructura de mensaje de la librería `rcl_interfaces` que indica si el cambio de un parámetro fue exitoso o no.

* **`self.get_logger().info("mensaje")`**
  Permite imprimir mensajes informativos en la consola del nodo.

* **`.destroy_node()`**
  Libera los recursos del nodo antes de finalizar su ejecución.

* **`rclpy.shutdown()`**
  Cierra el sistema de ROS2 de manera controlada al terminar el programa.

---

## 4. Archivos de configuración

### 📌 `setup.py`

En `entry_points` se añadió la entrada para el nuevo nodo:

```python
'console_scripts': [
    'simple_publisher = arduinobot_py_examples.simple_publisher:main',
    'simple_subscriber = arduinobot_py_examples.simple_subscriber:main',
    'simple_parameter = arduinobot_py_examples.simple_parameter:main'
],
```

### 📌 `package.xml`

Se debe incluir la dependencia de `rcl_interfaces` ya que se usa `SetParametersResult`:

```xml
<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
<exec_depend>rcl_interfaces</exec_depend>
```

---

## 5. Ejecución del nodo

1. Construir el workspace:

   ```bash
   colcon build
   ```

2. Sourcing del entorno:

   ```bash
   source install/setup.bash
   ```

3. Ejecutar el nodo:

   ```bash
   ros2 run arduinobot_py_examples simple_parameter
   ```

---

## 6. Probar parámetros desde la terminal

* **Ver parámetros actuales:**

  ```bash
  ros2 param list
  ```

* **Obtener el valor de un parámetro:**

  ```bash
  ros2 param get /simple_paramaeter simple_int_param
  ```

* **Cambiar el valor de un parámetro en ejecución:**

  ```bash
  ros2 param set /simple_paramaeter simple_int_param 42
  ros2 param set /simple_paramaeter simple_string_param "Robot_UNAL"
  ```