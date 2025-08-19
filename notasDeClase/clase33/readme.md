# Clase 33 — URDF (Parte 2): nuevos *links*, ejes y **mimic** (gripper)

> Continuamos el modelo URDF/Xacro agregando los eslabones del brazo y el gripper, definiendo sus *joints* y límites. Este resumen es **corto** y listo para Github.

---

## Qué agregamos en esta clase

* **Links (con malla y `origin`)**

  * `forward_drive_arm` → `forward_drive_arm.STL`, `scale = 0.01 0.01 0.01`, `origin`:
    `rpy="0 -PI/2 PI/2"` · `xyz="0.19 0.06 -0.08"`
  * `horizontal_arm` → `horizontal_arm.STL`, `scale = 0.01 0.01 0.01`, `origin`:
    `rpy="PI/2 0 PI/2"` · `xyz="-0.03 -0.4 -0.06"`
  * `claw_support` → `claw_support.STL`, `scale = 0.01 0.01 0.01`, `origin`:
    `rpy="0 0 PI/2"` · `xyz="0 -0.05 -0.15"`
  * `gripper_right` → `right_finger.STL`, `scale = 0.01 0.01 0.01`, `origin`:
    `rpy="0 0 -PI/2"` · `xyz="-0.1 0.5 -0.1"`
  * `gripper_left` → `left_finger.STL`, `scale = 0.01 0.01 0.01`, `origin`:
    `rpy="0 0 -PI/2"` · `xyz="-0.04 0.5 -0.1"`

> Usa variables Xacro que ya tenías (`PI`, `effort`, `velocity`) para **no repetir** números.

---

## Joints añadidos

|                              Joint | Tipo     | Parent → Child                         | Axis    | Origin (rpy / xyz)                      | Límites (rad)                                          | Notas                                        |
| ---------------------------------: | -------- | -------------------------------------- | ------- | --------------------------------------- | ------------------------------------------------------ | -------------------------------------------- |
|                           joint\_2 | revolute | `base_plate` → `forward_drive_arm`     | `1 0 0` | `rpy="0 0 0"` · `xyz="-0.02 0 0.35"`    | `lower="-PI/2"` · `upper="PI/2"` · `effort`/`velocity` | Segundo DOF                                  |
|                           joint\_3 | revolute | `forward_drive_arm` → `horizontal_arm` | `1 0 0` | `rpy="0 0 0"` · `xyz="0 0 0.8"`         | `lower="-PI/2"` · `upper="PI/2"` · `effort`/`velocity` | Tercer DOF                                   |
| horizontal\_arm\_to\_claw\_support | fixed    | `horizontal_arm` → `claw_support`      | —       | `rpy="0 0 0"` · `xyz="0 0.82 0"`        | —                                                      | Unión rígida                                 |
|                           joint\_4 | revolute | `claw_support` → `gripper_right`       | `0 0 1` | `rpy="0 0 0"` · `xyz="-0.04 0.13 -0.1"` | `lower="-PI/2"` · `upper="0.0"` · `effort`/`velocity`  | Dedo derecho                                 |
|                           joint\_5 | revolute | `claw_support` → `gripper_left`        | `0 0 1` | `rpy="0 0 0"` · `xyz="-0.22 0.13 -0.1"` | `lower="0.0"` · `upper="PI/2"` · `effort`/`velocity`   | **mimic** de `joint_4` con `multiplier="-1"` |

**Mimic (gripper):**

```xml
<joint name="joint_5" type="revolute">
  ...
  <mimic joint="joint_4" multiplier="-1"/>
</joint>
```

---

## Pasos rápidos

1. **Añade** los `link` y `joint` anteriores a tu `urdf/arduinobot.urdf.xacro`.
2. **Reutiliza** las propiedades Xacro:

   ```xml
   <xacro:property name="PI" value="3.14159265359"/>
   <xacro:property name="effort" value="30.0"/>
   <xacro:property name="velocity" value="10.0"/>
   ```
3. **Compila y visualiza**:

   ```bash
   colcon build
   . install/setup.bash
   ros2 launch urdf_tutorial display.launch.py model:=/ruta/absoluta/a/arduinobot.urdf.xacro
   ```

> En RViz2 verás **sliders** para los *joints* `revolute`. El `mimic` hace que ambos dedos se muevan de forma **opuesta** con un solo control.

---

## Tips / Errores comunes

* No inviertas `xyz` y `rpy` en `<origin>`.
* Revisa **mayúsculas** en nombres de archivos (`STL` vs `stl`).
* Usa rutas `package://arduinobot_description/meshes/...` o una **variable** `${path}` para todas las mallas.
