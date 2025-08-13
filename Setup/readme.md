# 🛠️ Setup para el Curso

Este repositorio requiere el uso de [Git LFS (Large File Storage)](https://git-lfs.com/) para gestionar archivos pesados como `.stp`, `.stl`, `.ipt`, entre otros, que superan los 100 MB y no pueden ser subidos directamente a GitHub.

---

### ✅ Instalación Git LFS (Larfe file Storage)

1. **Instalar Git LFS**  
   Abre la terminal y ejecuta:

   ```bash
   sudo apt update
   sudo apt install git-lfs
   ```

2. **Inicializar Git LFS** (solo una vez por repositorio):

   ```bash
   git lfs install
   ```

3. **Track de los distintos tipos de archivos**  
   Ejecuta el siguiente comando en la raíz del repositorio:

   ```bash
   git lfs track "*.ipt" "*.iam" "*.idw" "*.ipn" "*.stl" "*.step" "*.stp" "*.igs" "*.iges" "*.dwg" "*.dxf" "*.3ds" "*.obj" "*.fbx" "*.mp4" "*.mov" "*.avi" "*.mkv" "*.zip" "*.rar" "*.7z"
   ```

---

### 📂 Clasificación de tipos de archivos

| Categoría      | Extensiones                                                                      |
|----------------|----------------------------------------------------------------------------------|
| **Inventor**   | `.ipt`, `.iam`, `.idw`, `.ipn`                                                   |
| **CAD/3D**     | `.stl`, `.step`, `.stp`, `.igs`, `.iges`, `.dwg`, `.dxf`, `.3ds`, `.obj`, `.fbx` |
| **Video**      | `.mp4`, `.mov`, `.avi`, `.mkv`                                                   |
| **Compresión** | `.zip`, `.rar`, `.7z`                                                            |

---

### 🧩 Confirmar seguimiento con `.gitattributes`

1. Asegúrate de estar en la raíz del repositorio.
2. Agrega el archivo `.gitattributes` al repositorio:

   ```bash
   git add .gitattributes
   git commit -m "Tracking CAD, video, and archive files with Git LFS"
   ```

---

> ⚠️ **Importante:** Asegúrate de hacer `git add` a los archivos rastreados con LFS **después** de configurar el `git lfs track`, para que el seguimiento sea efectivo.

---

## Instalar ROS2 Humble en Ubuntu 22.04

Para instalar **ROS2 Humble** en **Ubuntu 22.04**, visita el siguiente enlace y sigue los pasos indicados:

👉 [Guía oficial de instalación de ROS2 Humble en Ubuntu](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

> ⚠️ **Importante:** Se debe usar la opción **"Desktop Install"** y **NO** la opción **"ros-base (Bare Bones)"**.





