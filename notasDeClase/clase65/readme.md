# Clase 65 — Cliente de Servicio en Python (`AddTwoInts`) con ROS 2

Este `README.md` resume la clase donde implementamos un **cliente de servicio en Python**
que consume el servidor `/add_two_ints` para sumar dos enteros. Incluye los **comandos**
utilizados y una explicación breve de cada archivo relevante, además de **pseudocódigo**
del script `simple_service_client.py`.

---

## Objetivos
- Crear un **nodo cliente** que:
  - Espere la disponibilidad del servicio `/add_two_ints`.
  - Envíe una solicitud con dos enteros `a`, `b` (tipo `arduinobot_msgs/srv/AddTwoInts`).
  - Imprima la respuesta `sum` al finalizar la llamada.
- Verificar el flujo **cliente ↔ servidor** con la CLI de ROS 2.

---

## Comandos usados en clase

> Desde la raíz del workspace (`arduinobot_ws`), compila y *sourcea*:

```bash
colcon build
source install/setup.bash
```

### Terminal 1 — Ejecutar el **servidor**
```bash
ros2 run arduinobot_py_examples simple_service_server
```
Salida esperada (ejemplo):
```
[INFO] [...] [simple_service_server]: Service add_two_ints Ready
[INFO] [...] [simple_service_server]: New Message Received a: 5, b: 3
[INFO] [...] [simple_service_server]: Returning sum: 8
```

### Terminal 2 — Ejecutar el **cliente**
```bash
source install/setup.bash
ros2 run arduinobot_py_examples simple_service_client 5 3
```
Salida esperada:
```
[INFO] [...] [simple_service_client]: Service Response 8
```

> Si el servidor no está disponible, el cliente mostrará periódicamente:  
> `Service not available, waiting more ...` hasta que el servicio aparezca.

---

## ¿Qué hace cada archivo?

### `arduinobot_py_examples/simple_service_client.py`
Cliente asíncrono que llama al servicio `AddTwoInts`:
- Importa `rclpy`, `Node` y la interfaz `arduinobot_msgs/srv/AddTwoInts`.
- **Clase** `SimpleServiceClient(Node)`:
  - Crea el **cliente** con `create_client(AddTwoInts, "add_two_ints")`.
  - Espera al servidor con `wait_for_service(timeout_sec=1.0)` en un bucle.
  - Construye la **request** con `a` y `b` (convertidos desde `sys.argv` a `int`).
  - Llama al servicio con `call_async(...)` y registra un **callback** con `add_done_callback(...)`.
- **`main()`**:
  - Inicializa ROS 2, valida que se reciban **dos argumentos** (a, b),
  - crea el nodo y hace `rclpy.spin(...)` para procesar la respuesta,
  - destruye el nodo y hace `rclpy.shutdown()`.

### `arduinobot_py_examples/simple_service_server.py` (referencia de clase previa)
Servidor que **expone** `/add_two_ints` y retorna `sum = a + b`. Debe estar arriba para probar el cliente.

### `setup.py` (del paquete Python)
Incluye los **entry points** para instalar los ejecutables Python, por ejemplo:
- `simple_service_client = <tu_paquete>.simple_service_client:main`
- `simple_service_server = <tu_paquete>.simple_service_server:main`

> En esta clase no fue necesario añadir nuevas dependencias en `package.xml`.

---

## Pseudocódigo corto — `simple_service_client.py`

```text
función main():
  rclpy.init()
  si número_de_argumentos != 2:
    imprimir "uso"
    salir

  a = int(argv[1])
  b = int(argv[2])
  nodo = SimpleServiceClient(a, b)
  rclpy.spin(nodo)
  destruir nodo
  rclpy.shutdown()

clase SimpleServiceClient(Node):
  constructor(a, b):
    super("simple_service_client")
    client = create_client(AddTwoInts, "add_two_ints")
    mientras client.wait_for_service(1.0) == false:
      log "Service not available, waiting more ..."

    req = AddTwoInts.Request(a=a, b=b)
    future = client.call_async(req)
    future.add_done_callback(responseCallback)

  responseCallback(future):
    log "Service Response {future.result().sum}"
```

---

## Estructura mínima (referencial)

```
arduinobot_ws/
└── src/
    └── arduinobot_py_examples/
        ├── package.xml
        ├── setup.py
        └── arduinobot_py_examples/
            ├── simple_service_server.py   # servidor
            └── simple_service_client.py   # cliente (esta clase)
```

---

## Solución de problemas
- **`Service not available...` infinito**: asegúrate de que el **servidor** corre en otra terminal y
  que hiciste `source install/setup.bash` en **ambas**.
- **No encuentra el ejecutable**: revisa el `entry point` en `setup.py`, recompila y `source`.
- **Argumentos inválidos**: el cliente requiere **dos enteros**, p. ej. `5 3`.
