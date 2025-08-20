# Clase 38: CLI de parámetros en ROS 2 (`simple_parameter`)

Este README explica, paso a paso, **cómo ejecutar el nodo** `simple_parameter` y cómo **gestionar sus parámetros** desde la línea de comandos usando `ros2 param`.

---

## 3) Ejecutar el nodo `simple_parameter`

Puedes ejecutar la **implementación en C++** (paquete `arduinobot_cpp_examples`) o, si existe, la **implementación en Python** (paquete `arduinobot_py_examples`). Aquí usamos C++.

### 3.1 Ejecución con valores por defecto

#### Comando

```bash
ros2 run arduinobot_cpp_examples simple_parameter
```

#### ¿Qué hace?

* Lanza el nodo `simple_parameter` con los **valores por defecto** de sus parámetros (por ejemplo, `simple_int_param = 28` y `simple_string_param = "Antonio"`).

#### Notas

* Para **detener** el nodo en la terminal donde corre: `Ctrl + C`.

---

### 3.2 Ejecución pasando parámetros al arrancar

#### Comando

```bash
ros2 run arduinobot_cpp_examples simple_parameter --ros-args -p simple_int_param:=30
```

#### ¿Qué hace?

* Lanza el nodo **sobrescribiendo** el valor por defecto de `simple_int_param` a `30`.
* La opción `--ros-args` permite pasar argumentos de ROS 2; `-p` se usa para **asignar parámetros** en el arranque.

#### Ejemplo (Terminal 1)

```
ros2 run arduinobot_cpp_examples simple_parameter --ros-args -p simple_int_param:=30
[INFO] [...] [simple_parameter]: Param simple_string_param cambiado! Nuevo valor: Hi ros2
```

---

## 4) Inspeccionar y gestionar parámetros con `ros2 param`

Abre **otra terminal** (Terminal 2) y usa los siguientes comandos mientras el nodo está corriendo en la Terminal 1.

### 4.1 Listar parámetros disponibles del nodo

```bash
ros2 param list
```

**Salida:**

```
/simple_parameter:
  simple_int_param
  simple_string_param
  use_sim_time
  qos_overrides...
```

---

### 4.2 Leer (obtener) el valor de un parámetro

```bash
ros2 param get /simple_parameter simple_int_param
```

**Salida:**

```
Integer value is: 28
```

Si lo arrancaste con `-p simple_int_param:=30`:

```
Integer value is: 30
```

Otro ejemplo con string:

```bash
ros2 param get /simple_parameter simple_string_param
# String value is: Antonio
```

---

### 4.3 Cambiar el valor de un parámetro en caliente (runtime)

```bash
ros2 param set /simple_parameter simple_string_param "Hi ros2"
```

**Salida:**

```
Set parameter successful
```

Verificación:

```bash
ros2 param get /simple_parameter simple_string_param
# String value is: Hi ros2
```

---

## 5) Ayuda, subcomandos y autocompletado

* Ver ayuda general:

  ```bash
  ros2 param
  ```
* Ver ayuda de un subcomando:

  ```bash
  ros2 param get -h
  ros2 param set -h
  ```
* Autocompletado:

  * Escribe `ros2 param ` y presiona **TAB** dos veces para listar los subcomandos disponibles.

---

## 6) Resumen rápido (cheat-sheet)

| Tarea                           | Comando                                                          |
| ------------------------------- | ---------------------------------------------------------------- |
| Ejecutar nodo (C++)             | `ros2 run arduinobot_cpp_examples simple_parameter`              |
| Ejecutar nodo pasando parámetro | `ros2 run ... --ros-args -p simple_int_param:=30`                |
| Listar parámetros               | `ros2 param list`                                                |
| Leer parámetro                  | `ros2 param get /simple_parameter simple_int_param`              |
| Cambiar parámetro en caliente   | `ros2 param set /simple_parameter simple_string_param "Hi ros2"` |
| Ver ayuda                       | `ros2 param -h`                                                  |
