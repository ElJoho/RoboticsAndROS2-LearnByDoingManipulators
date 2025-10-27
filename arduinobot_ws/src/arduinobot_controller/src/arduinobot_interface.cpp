#include "arduinobot_controller/arduinobot_interface.hpp"
#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <cmath>
#include <sstream>

namespace arduinobot_controller{

    std::string compensateZeros(int value){
        std::string compensate_zeros = "";
        if(value < 10){
            compensate_zeros = "00";
        }else if(value < 100 ){
            compensate_zeros = "0";
        }else{
            compensate_zeros = "";
        }
        return compensate_zeros;
    }

    ArduinobotInterface::ArduinobotInterface(){}

    ArduinobotInterface::~ArduinobotInterface(){
        if(arduino_.IsOpen()){
            try{
                arduino_.Close();
            }catch(...){
                RCLCPP_FATAL_STREAM(
                    rclcpp::get_logger("ArduinobotInterface"),
                    "Something went wrong while closing the connection with port " << port_
                );
            }
        }
    }

    ArduinobotInterface::CallbackReturn
    ArduinobotInterface::on_init(const hardware_interface::HardwareInfo &hardware_info){
        CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
        if (result != CallbackReturn::SUCCESS){
            return result;
        }

        // get serial port from params (e.g., <param name="port">/dev/ttyUSB0</param>)
        try{
            port_ = info_.hardware_parameters.at("port");
        }catch(...){
            RCLCPP_FATAL(rclcpp::get_logger("ArduinobotInterface"),
                         "Parameter 'port' not found in hardware parameters.");
            return CallbackReturn::ERROR;
        }

        // IMPORTANT: size vectors (reserve is not enough since we export pointers to elements)
        const size_t n_joints = info_.joints.size();
        position_commands_.assign(n_joints, 0.0);
        prev_position_commands_.assign(n_joints, 0.0);
        position_states_.assign(n_joints, 0.0);

        return CallbackReturn::SUCCESS;
    }

    ArduinobotInterface::CallbackReturn
    ArduinobotInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/){
        // zero all buffers based on actual joint count
        std::fill(position_commands_.begin(), position_commands_.end(), 0.0);
        std::fill(prev_position_commands_.begin(), prev_position_commands_.end(), 0.0);
        std::fill(position_states_.begin(), position_states_.end(), 0.0);

        // open serial
        try{
            if(!arduino_.IsOpen()){
                arduino_.Open(port_);
                arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
                arduino_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
            }
        }catch(const std::exception &e){
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("ArduinobotInterface"),
                                "Failed to open serial port " << port_ << ": " << e.what());
            return CallbackReturn::ERROR;
        }catch(...){
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("ArduinobotInterface"),
                                "Failed to open serial port " << port_);
            return CallbackReturn::ERROR;
        }

        return CallbackReturn::SUCCESS;
    }

    ArduinobotInterface::CallbackReturn
    ArduinobotInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/){
        try{
            if(arduino_.IsOpen()){
                arduino_.Close();
            }
        }catch(...){
            RCLCPP_WARN_STREAM(rclcpp::get_logger("ArduinobotInterface"),
                               "Error while closing port " << port_);
        }
        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface>
    ArduinobotInterface::export_state_interfaces(){
        std::vector<hardware_interface::StateInterface> state_interfaces;
        state_interfaces.reserve(info_.joints.size());
        for (size_t i = 0; i < info_.joints.size(); ++i){
            state_interfaces.emplace_back(
                hardware_interface::StateInterface(
                    info_.joints[i].name,
                    hardware_interface::HW_IF_POSITION,
                    &position_states_[i]
                )
            );
        }
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface>
    ArduinobotInterface::export_command_interfaces(){
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        command_interfaces.reserve(info_.joints.size());
        for (size_t i = 0; i < info_.joints.size(); ++i){
            command_interfaces.emplace_back(
                hardware_interface::CommandInterface(
                    info_.joints[i].name,
                    hardware_interface::HW_IF_POSITION,
                    &position_commands_[i]
                )
            );
        }
        return command_interfaces;
    }

    hardware_interface::return_type
    ArduinobotInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/){
        // if Arduino echoes measured positions back, parse here and update position_states_
        // minimal placeholder keeps controllers stable if no feedback:
        position_states_ = position_commands_;
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type
    ArduinobotInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/){
        if (!arduino_.IsOpen()){
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("ArduinobotInterface"),
                                "Serial port " << port_ << " is not open.");
            return hardware_interface::return_type::ERROR;
        }

        // send only when there is a change to reduce traffic
        if (position_commands_ == prev_position_commands_){
            return hardware_interface::return_type::OK;
        }

        try{
            // Example simple protocol: "J1:xxx;J2:yyy;J3:zzz;J4:www\n" with degrees [0..180]
            std::stringstream ss;
            for (size_t i = 0; i < position_commands_.size(); ++i){
                // radians -> degrees
                double deg = position_commands_[i] * 180.0 / M_PI;
                if (deg < 0.0) deg = 0.0;
                if (deg > 180.0) deg = 180.0;
                int ideg = static_cast<int>(std::round(deg));
                ss << "J" << (i+1) << ":" << compensateZeros(ideg) << ideg;
                if (i + 1 < position_commands_.size()) ss << ";";
            }
            ss << "\n";
            const std::string msg = ss.str();
            arduino_.Write(msg);
        }catch(const std::exception &e){
            RCLCPP_ERROR_STREAM(
                rclcpp::get_logger("ArduinobotInterface"),
                "Something went wrong while sending a command to port "  << port_ << ": " << e.what()
            );
            return hardware_interface::return_type::ERROR;
        }catch(...){
            RCLCPP_ERROR_STREAM(
                rclcpp::get_logger("ArduinobotInterface"),
                "Something went wrong while sending a command to port "  << port_
            );
            return hardware_interface::return_type::ERROR;
        }

        prev_position_commands_ = position_commands_;
        return hardware_interface::return_type::OK;
    }

} // namespace arduinobot_controller

PLUGINLIB_EXPORT_CLASS(
    arduinobot_controller::ArduinobotInterface,
    hardware_interface::SystemInterface
)
