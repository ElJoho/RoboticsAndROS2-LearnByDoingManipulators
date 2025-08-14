# 25. Create and Activate a Workspace

## 1. Colcon Build
Usar el comando:

```bash
colcon build
```

para inicializar el workspace.

> ⚠️ **Importante:**
> Ejecuta `colcon build` **únicamente** en la carpeta raíz del workspace
> *(por ejemplo: `~/arduinobot_ws/`)* y **nunca** dentro de `src` u otra carpeta interna.

---

## 2. Creando un paquete

### 2.1 Creando un paquete de Python

Ir a la carpeta `src` dentro del workspace y ejecutar:

```bash
cd src/
ros2 pkg create --build-type ament_python arduinobot_py_examples
```

---

### 2.2 Creando un paquete de C++

Ir a la carpeta `src` dentro del workspace y ejecutar:

```bash
cd src/
ros2 pkg create --build-type ament_cmake arduinobot_cpp_examples
```

---

### 2.3 Usar Colcon Build para crear los paquetes

Volver a la carpeta raíz del workspace y ejecutar:

```bash
cd ..
colcon build
```

---

### 2.4 Activar el workspace

Para que el workspace actual (`arduinobot_ws`) se reconozca como un **overlay** para ROS 2:

```bash
cd install/
. setup.bash
```

💡 **Tip:** También puedes usar:

```bash
source install/setup.bash
```

---

### 2.5 Comprobar que el paquete está en el listado

Ejecutar:

```bash
ros2 pkg list
```

Verificar que aparecen:

* `arduinobot_py_examples`
* `arduinobot_cpp_examples`

---

> ℹ️ **Nota:**
> El comando `source install/setup.bash` **solo afecta la terminal actual**.
> Si abres una nueva terminal, deberás volver a ejecutarlo o añadirlo a tu `~/.bashrc`:
>
> ```bash
> echo "source ~/arduinobot_ws/install/setup.bash" >> ~/.bashrc
> ```

