# Clase 43 – Preparar el URDF para Simulación en Gazebo

## 🔧 Cambios principales en el URDF (`arduinobot.urdf.xacro`)

1. **Etiqueta `<collision>`**
   - Cada `link` ahora incluye una sección `<collision>`.
   - En esta sección se define el **volumen ocupado en el espacio** por el link.
   - Para simplificar, copiamos el contenido de la etiqueta `<visual>` y lo usamos también en `<collision>`.
   - Ejemplo:
     ```xml
     <collision>
       <geometry>
         <mesh filename="package://arduinobot_description/meshes/base_link.STL"/>
       </geometry>
     </collision>
     ```

   👉 De esta forma, Gazebo puede detectar colisiones y calcular las fuerzas entre las piezas del robot y el entorno.

---

2. **Etiqueta `<inertial>`**
   - Añadimos la descripción de **masa e inercia** de cada link.
   - Ejemplo básico:
     ```xml
     <inertial>
       <mass value="1.0"/>
       <inertia ixx="1.0" ixy="0.0" ixz="0.0"
                iyy="1.0" iyz="0.0"
                izz="1.0"/>
     </inertial>
     ```
   - Se usa una **matriz de inercia identidad**, suficiente para el curso.

---

3. **Uso de Macros en Xacro**
   - Para no repetir código, definimos una macro llamada `default_inertia`:
     ```xml
     <xacro:macro name="default_inertia" params="mass">
       <inertial>
         <mass value="${mass}"/>
         <inertia ixx="1.0" ixy="0.0" ixz="0.0"
                  iyy="1.0" iyz="0.0"
                  izz="1.0"/>
       </inertial>
     </xacro:macro>
     ```
   - Con esta macro podemos asignar fácilmente diferentes masas a cada link:
     ```xml
     <xacro:default_inertia mass="1.0"/>   <!-- base_link -->
     <xacro:default_inertia mass="0.1"/>   <!-- base_plate -->
     <xacro:default_inertia mass="0.05"/>  <!-- closer_support -->
     <xacro:default_inertia mass="0.01"/>  <!-- gripper -->
     ```

---

## 📋 Masas asignadas por link

- **base_link** → 1.0  
- **base_plate** → 0.1  
- **forward_drive_arm** → 0.1  
- **horizontal_arm** → 0.1  
- **closer_support** → 0.05  
- **gripper_right** → 0.01  
- **gripper_left** → 0.01  
