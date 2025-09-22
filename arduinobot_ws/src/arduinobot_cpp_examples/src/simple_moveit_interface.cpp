#include <memory> // Incluye utilidades de memoria (std::shared_ptr, etc.)
#include <rclcpp/rclcpp.hpp> // Incluye la API de ROS 2 en C++ (nodos, logging, init/shutdown)
#include <moveit/move_group_interface/move_group_interface.h> // Incluye la interfaz MoveGroupInterface para planear/ejecutar con MoveIt

void move_robot(const std::shared_ptr<rclcpp::Node> node){ // Define función que moverá el robot; recibe un nodo ROS 2 compartido
    auto arm_move_group = moveit::planning_interface::MoveGroupInterface(node, "arm"); // Crea una interfaz de grupo para el move group "arm"
    auto gripper_move_group = moveit::planning_interface::MoveGroupInterface(node, "gripper"); // Crea una interfaz de grupo para el move group "gripper"
    
    std::vector<double> arm_joint_goal {1.57, 0.0, 0.0}; // Objetivo de juntas del brazo: base ~90° (1.57 rad), hombro 0, codo 0
    std::vector<double> gripper_joint_goal {-0.7, 0.7}; // Objetivo de juntas del gripper: abre la pinza con valores opuestos

    bool arm_within_bounds = arm_move_group.setJointValueTarget(arm_joint_goal); // Aplica el objetivo al brazo y verifica si está dentro de límites
    bool gripper_within_bounds = gripper_move_group.setJointValueTarget(gripper_joint_goal); // Aplica el objetivo al gripper y verifica límites

    if(!arm_within_bounds || !gripper_within_bounds){ // Si cualquiera de los objetivos está fuera de límites...
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"),"Target joint position were outside of limits!"); // ...avisa por consola con un warning
        return; // Sale de la función sin planear/ejecutar
    }

    moveit::planning_interface::MoveGroupInterface::Plan arm_plan; // Objeto para almacenar el plan del brazo (trayectoria + metadatos)
    moveit::planning_interface::MoveGroupInterface::Plan gripper_plan; // Objeto para almacenar el plan del gripper

    bool arm_plan_success = arm_move_group.plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS; // Intenta planear para el brazo y comprueba éxito
    bool gripper_plan_success = gripper_move_group.plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS; // Intenta planear para el gripper y comprueba éxito

    if(arm_plan_success && gripper_plan_success){ // Si ambos planes fueron exitosos...
        arm_move_group.move(); // Ejecuta el plan del brazo (usa el último plan calculado internamente)
        gripper_move_group.move(); // Ejecuta el plan del gripper
    }else{ // Si alguno de los planes falló...
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"One or more planners failed"); // ...registra un error en consola
        return; // Sale de la función
    }
}

int main(int argc, char ** argv){ // Función principal del ejecutable
    rclcpp::init(argc,argv); // Inicializa el sistema ROS 2 con argumentos de línea de comandos
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("simple_moveit_interface"); // Crea un nodo ROS 2 llamado "simple_moveit_interface"
    move_robot(node); // Llama a la función que configura objetivos, planea y ejecuta
    rclcpp::shutdown(); // Cierra limpiamente la infraestructura de ROS 2
    return 0; // Devuelve 0 indicando finalización correcta
}
