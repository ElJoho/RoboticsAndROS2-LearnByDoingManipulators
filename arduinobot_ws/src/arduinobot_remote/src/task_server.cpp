#include <rclcpp/rclcpp.hpp> // Incluye la API principal de nodos de ROS 2 (rclcpp)
#include <rclcpp_action/rclcpp_action.hpp> // Incluye la API de acciones de ROS 2
#include "arduinobot_msgs/action/arduinobot_task.hpp" // Mensaje de acción personalizado ArduinobotTask
#include <rclcpp_components/register_node_macro.hpp> // Macro para registrar el nodo como componente
#include <moveit/move_group_interface/move_group_interface.h> // Interfaz MoveGroup de MoveIt para planificar y mover
#include <memory> // Utilidades de memoria (std::shared_ptr, etc.)
#include <thread> // Para lanzar hilos (std::thread)
#include <vector>  // added // Contenedor para objetivos de articulaciones (vectores de double)

using namespace std::placeholders; // Usa _1, _2, ... para std::bind
namespace arduinobot_remote // Define el namespace del paquete/nodo
{   
    class TaskServer : public rclcpp::Node{ // Declara la clase TaskServer que hereda de rclcpp::Node
        public:
            explicit TaskServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("task_server", options){ // Constructor: crea el nodo "task_server" con opciones
                action_server_ = rclcpp_action::create_server<arduinobot_msgs::action::ArduinobotTask>( // Crea el servidor de acciones para ArduinobotTask
                    get_node_base_interface(), // Interfaz base del nodo (requerida por create_server)
                    get_node_clock_interface(), // Interfaz de reloj del nodo
                    get_node_logging_interface(), // Interfaz de logging del nodo
                    get_node_waitables_interface(), // Interfaz de waitables (para ejecución)
                    "task_server", // Nombre del servidor de acción/tópico
                    std::bind(&TaskServer::goalCallback,this,_1,_2), // Callback para metas (goal) entrantes
                    std::bind(&TaskServer::cancelCallback, this, _1), // Callback para solicitudes de cancelación
                    std::bind(&TaskServer::acceptedCallback,this,_1) // Callback cuando una meta es aceptada
                );
                
                RCLCPP_INFO( // Log informativo al iniciar
                    rclcpp::get_logger("rclcpp"), // Logger genérico de rclcpp
                    "Starting the Task Server" // Mensaje de inicio
                );
            }
        private:
            
            rclcpp_action::Server<arduinobot_msgs::action::ArduinobotTask>::SharedPtr action_server_; // Puntero compartido al servidor de acciones
            std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_move_group_, gripper_move_group_; // MoveGroup para brazo y pinza
            std::vector<double> arm_joint_goal_ , gripper_joint_goal_; // Objetivos de articulaciones para brazo y pinza

            rclcpp_action::GoalResponse goalCallback( // Maneja la recepción de una nueva meta
                const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const arduinobot_msgs::action::ArduinobotTask::Goal> goal // UUID y contenido de la meta
            ){
                (void)uuid;  // silence unused warning // Evita advertencia si uuid no se usa
                RCLCPP_INFO_STREAM( // Imprime información detallada usando stream
                    rclcpp::get_logger("rclcpp"), // Logger
                    "Received goal request with task_number: " << goal-> task_number // Muestra el número de tarea solicitado
                );
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; // Acepta y ejecuta inmediatamente la meta
            }


            rclcpp_action::CancelResponse cancelCallback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<arduinobot_msgs::action::ArduinobotTask>> goal_handle){ // Maneja solicitud de cancelación
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received request to cancel the goal"); // Log de cancelación
                if(arm_move_group_){ // Si el MoveGroup del brazo está inicializado
                    arm_move_group_->stop(); // Detiene cualquier movimiento del brazo
                }if(gripper_move_group_){ // Si el MoveGroup de la pinza está inicializado
                    gripper_move_group_->stop(); // Detiene cualquier movimiento de la pinza
                }
                (void)goal_handle; // No se usa explícitamente: evita advertencia
                return rclcpp_action::CancelResponse::ACCEPT; // Acepta la cancelación
            }

            void acceptedCallback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<arduinobot_msgs::action::ArduinobotTask>> goal_handle){ // Se llama al aceptar una meta
                std::thread{std::bind(&TaskServer::execute,this,_1), goal_handle}.detach(); // Lanza la ejecución en un hilo separado y lo desprende
            }

            void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<arduinobot_msgs::action::ArduinobotTask>> goal_handle){ // Lógica principal de ejecución de la acción

                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Executing goal"); // Log de inicio de ejecución
                if(!arm_move_group_){ // Si aún no se creó el MoveGroup del brazo
                    arm_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface> (shared_from_this(), "arm"); // Inicializa MoveGroup para el grupo "arm"
                }
                if(!gripper_move_group_){ // Si aún no se creó el MoveGroup de la pinza
                    gripper_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface> (shared_from_this(), "gripper"); // Inicializa MoveGroup para el grupo "gripper"
                }
                auto result = std::make_shared<arduinobot_msgs::action::ArduinobotTask::Result>(); // Objeto resultado que se enviará al cliente

                if(goal_handle-> get_goal() -> task_number == 0){ // Si la tarea solicitada es 0 (home)
                    arm_joint_goal_ = {0.0, 0.0, 0.0}; // Brazo a posición de inicio (todas las articulaciones en 0)
                    gripper_joint_goal_ = {-0.7, 0.7}; // Pinza abierta (valores opuestos)
                }else if(goal_handle-> get_goal() -> task_number == 1){ // Si la tarea solicitada es 1 (picking)
                    arm_joint_goal_ = {-1.14, -0.6, -0.07}; // Posición hipotética de recogida
                    gripper_joint_goal_ = {0.0, 0.0}; // Pinza cerrada
                }else if(goal_handle->get_goal() -> task_number == 2){ // Si la tarea solicitada es 2 (rest)
                    arm_joint_goal_ = {-1.57, 0.0, -0.9}; // Posición de descanso del brazo
                    gripper_joint_goal_ = {0.0, 0.0}; // Pinza cerrada
                }else{ // Si la tarea no es válida
                    RCLCPP_ERROR(get_logger(), "Invalid Task Number");  // fixed typo // Error: número de tarea inválido
                    return; // Termina la ejecución sin éxito
                }

                arm_move_group_->setStartState(*arm_move_group_->getCurrentState()); // Fija el estado inicial del brazo al estado actual
                gripper_move_group_->setStartState(*gripper_move_group_->getCurrentState());  // fixed: use gripper // Fija el estado inicial de la pinza al estado actual

                bool arm_within_bounds = arm_move_group_->setJointValueTarget(arm_joint_goal_); // Asigna objetivo de juntas del brazo y valida límites
                bool gripper_within_bounds = gripper_move_group_->setJointValueTarget(gripper_joint_goal_); // Asigna objetivo de juntas de la pinza y valida límites

                if(!arm_within_bounds || !gripper_within_bounds){ // Si alguno de los objetivos está fuera de límites
                    RCLCPP_ERROR(get_logger(),"Target position out of boundaries"); // Informa error de límites
                    return; // Aborta la ejecución
                }

                moveit::planning_interface::MoveGroupInterface::Plan arm_plan, gripper_plan; // Estructuras para almacenar los planes (trayectorias)
                bool arm_plan_success = (arm_move_group_->plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS); // Planifica para el brazo y comprueba éxito
                bool gripper_plan_success =  (gripper_move_group_->plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS); // Planifica para la pinza y comprueba éxito
                
                if(arm_plan_success && gripper_plan_success){ // Si ambos planificadores tuvieron éxito
                    arm_move_group_->move(); // Ejecuta el plan del brazo (bloqueante hasta finalizar)
                    gripper_move_group_->move(); // Ejecuta el plan de la pinza
                }else{ // Si algún plan falló
                    RCLCPP_ERROR(get_logger(), "One or more planners failed"); // Informa fallo de planificación
                    return; // Aborta la ejecución
                }
                
                result->success = true; // Marca el resultado como exitoso
                goal_handle->succeed(result); // Notifica al cliente que la meta se cumplió enviando el resultado

            }
    };// namespace arduinobot_remote // Fin de la clase TaskServer
}; // Fin del namespace arduinobot_remote

RCLCPP_COMPONENTS_REGISTER_NODE(arduinobot_remote::TaskServer); // Registra el nodo como componente para rclcpp_components
