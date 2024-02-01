/*
 * Author:        Viktoria Krimer (viktoria.krimer@de.schunk.com)
 * Maintainer:    Viktoria Krimer (viktoria.krimer@de.schunk.com)
 * Created:       DECEMBER 2023
 * 
 * Implementation for common procedures to communicate with the gripper.
 *
 * The gripper initializes at the beginning. It acts as the middleware between communication and the Ros-Wrapper.
 * 
 */

#include "schunk_gripper/schunk_gripper_lib.hpp"

//Commands for using in ROS
std::map<std::string, uint32_t> commands_str
{
    {"NO COMMAND", 0},
    {"MOVE TO ABSOLUTE POSITION", MOVE_TO_ABSOLUTE_POSITION},
    {"MOVE TO RELATIVE POSITION", MOVE_TO_RELATIVE_POSITION},
    {"REPEAT COMMAND TOGGLE", REPEAT_COMMAND_TOGGLE},
    {"STOP", STOP},
    {"FAST STOP", FAST_STOP},
    {"GRIP WORKPIECE", GRIP_WORK_PIECE},
    {"GRIP WORKPIECE WITH POSITION", GRIP_WORKPIECE_WITH_POSITION},
    {"RELEASE WORKPIECE", RELEASE_WORK_PIECE},
    {"PREPARE FOR SHUTDOWN", PREPARE_FOR_SHUTDOWN},
    {"SOFT RESET", SOFT_RESET}
};
//Start th gripper, so it is ready to operate
Gripper::Gripper(const std::string &ip): AnybusCom(ip)
{  
   try
   {  
      startGripper();
      //Get parameters
      getActualParameters();
      //Model
      getModel();
      //get Versions
      getWithInstance<uint16_t>(SW_VERSION_NUM_INST, &sw_version);
      getWithInstance<char>(COMM_VERSION_TXT_INST, NULL, 12);
      comm_version = save_data_char.data();

      ip_changed_with_all_param = true;
   }
   catch(const char* res)
   {
      std::cout << "Failed Connection to gripper: " << res << std::endl;
      start_connection = false;
   }
   catch(const nlohmann::json::parse_error &e)
   {
      std::cout << "message: " << e.what()  << "\nexception id: " << e.id << std::endl;
      start_connection = false;
   }
   catch(...)
   {
      std::cout << "Failed Connection to gripper: "  << std::endl;
      start_connection = false;
   }
}

void Gripper::getActualParameters()
{
      grp_pos_lock = gripperBitOutput(USE_GPE);
      //Get parameters
      getWithInstance<float>(GRP_POS_MARGIN_INST, &grp_pos_margin);
      getWithInstance<float>(GRP_PREPOS_DELTA_INST, &grp_prepos_delta);
      getWithInstance<float>(ZERO_POS_OFS_INST, &zero_pos_ofs);
      getWithInstance<uint16_t>(GRP_PREHOLD_TIME_INST, &grp_prehold_time);
      getWithInstance<float>(WP_RELEASE_DELTA_INST, &wp_release_delta);
      getWithInstance<float>(WP_LOST_DISTANCE_INST, &wp_lost_dst);
      //get max and mins
      getWithInstance<float>(MAX_POS_INST, &max_pos);
      getWithInstance<float>(MIN_POS_INST,&min_pos);
      getWithInstance<float>(MAX_VEL_INST,&max_vel);
      getWithInstance<float>(MIN_VEL_INST,&min_vel);
      getWithInstance<float>(MAX_GRP_FORCE_INST, &max_grip_force);
      getWithInstance<float>(MIN_GRP_FORCE_INST, &min_grip_force);
      getWithInstance<float>(MAX_GRP_VEL_INST, &max_grp_vel);
}
//Check if Errors occurred by the gripper
void Gripper::getModel()
{
      //Model
      getEnums(MODULE_TYPE_INST, module_type);
      model = json_data["string"];
      //Is it model M
      if(model.find("_M_") != std::string::npos) 
      {
         std::cout << "Grip force and position maintenance!" << std::endl;
         model_M = true;
      }
      else
      {
         std::cout << "No grip force and position maintenance!" << std::endl;
         model_M = false;
      }
      if(model.find("EGU") != std::string::npos) getWithInstance<float>(MAX_ALLOW_FORCE_INST,&max_allow_force);
      
      std::cout <<  model << " CONNECTED!" << std::endl;
      start_connection = true;
}

bool Gripper::check()
{
   if(plc_sync_input[3] == 0) return 1;
   else return 0;
}
//Split plc_sync_input[3] into 3 uint8_t
std::array<uint8_t, 3> Gripper::splitDiagnosis()
{
   std::array<uint8_t, 3> error_codes;
   error_codes[0] = (plc_sync_input[3] >> 3*8) & 0xFF;
   error_codes[1] = (plc_sync_input[3]  >> 2*8) & 0xFF;
   error_codes[2] =  plc_sync_input[3] & 0xFF;   
   return error_codes;
}
//Return false when the Gripper is in end condition
bool Gripper::endCondition()  
{
   return !(gripperBitInput(SUCCESS) || gripperBitInput(POSITION_REACHED) || gripperBitInput(NO_WORKPIECE_DETECTED) 
   || gripperBitInput(GRIPPED) || gripperBitInput(GRIPPER_ERROR) || gripperBitInput(WARNING) 
   || gripperBitInput(WORK_PIECE_LOST) || gripperBitInput(WRONG_WORKPIECE_DETECTED));
}
//Post a Command and receive Gripper response
void Gripper::runPost(uint32_t command, uint32_t position, uint32_t velocity, uint32_t effort)
{  
    updatePlcOutput(command, position, velocity, effort);
    postCommand();
    getWithInstance<uint32_t>(PLC_SYNC_INPUT_INST);
}
//Receive Gripper response and actual Data
void Gripper::runGets()
{   
    getWithOffset(ACTUAL_POS_OFFSET, 3, 3);
    getWithInstance<uint32_t>(PLC_SYNC_INPUT_INST);
}
//If the gripper is ready for shutdown, so do softreset. DO: Acknowledge and get receive
void Gripper::startGripper()
{     
        //get dataformat -> big/little endian
        getInfo();
        //Get plc Values
        getWithInstance<uint32_t>(PLC_SYNC_INPUT_INST);
        getWithInstance<uint32_t>(PLC_SYNC_OUTPUT_INST);
        //Get actual values
        getWithOffset(ACTUAL_POS_OFFSET, 3, 3);

        getWithInstance<uint16_t>(MODULE_TYPE_INST, &module_type);
        getWithInstance<uint16_t>(FIELDBUS_TYPE_INST, &fieldbus_type);

        last_command = 0;


        if(gripperBitInput(READY_FOR_SHUTDOWN))
        {
           // ros::Duration time(5);
            std::cout << ("SOFTRESET") << std::endl;

            updatePlcOutput(SOFT_RESET, plc_sync_output[1], plc_sync_output[2], plc_sync_output[3]);
            postCommand();
           // time.sleep();
            std::this_thread::sleep_for(std::chrono::seconds(7));
            getWithInstance<uint32_t>(PLC_SYNC_INPUT_INST);
            std::cout << ("SOFTRESET FINISHED") << std::endl;

        }
        
        acknowledge();
        std::cout << ("ACKNOWLEDGED!") << std::endl;
        runGets();
}
//mu to mm conversion from float to uint32_t
uint32_t Gripper::mm2mu(const float &convert_float)
{
   if(convert_float < 0)
   {
      int32_t intValue;
      intValue = int32_t(convert_float * 1000);
      return static_cast<uint32_t>(intValue);
   }
   else return static_cast<uint32_t>(convert_float * 1000);
}
//Get the Bits of plc_sync_input[0]
bool Gripper::gripperBitInput(const uint32_t &bitmakro) const
{
    return bitmakro & plc_sync_input[0];
}
//Get the Bits of plc_sync_output[0]
bool Gripper::gripperBitOutput(const uint32_t &bitmakro) const
{
    return bitmakro & plc_sync_output[0];
}
//Get to an Error the corresponding string
std::string Gripper::getErrorString(const uint8_t &error) 
{
   try
   {
   json_data.clear();
   getEnums(ERROR_CODE_INST, error);   //saves it in raw json_data
   return json_data["string"];
   }
   catch(const nlohmann::json::parse_error &e)
   {
      std::cout << "message: " << e.what()  << "\nexception id: " << e.id << std::endl;
      return e.what();
   }
}
//Do acknowledge the gripper
void Gripper::acknowledge()
{
    plc_sync_output[0] &= mask;
    plc_sync_output[0] |= FAST_STOP;
    postCommand();
    plc_sync_output[0] |= ACKNOWLEDGE;
    postCommand();
    getWithInstance<uint32_t>(PLC_SYNC_INPUT_INST);
    plc_sync_output[0] &= mask;
}
//change the ip-address
bool Gripper::changeIp(const std::string &new_ip)
{
   if(new_ip.size() > 100) return false;
   
   std::string old_ip = ip;
   ip = new_ip;
   initAddresses();
   
   //Control if it is the same gripper
   try
   {
      startGripper();
      getActualParameters();
      getModel();
   
      return true;
   }
   catch(const char* res)
   {
      ip_changed_with_all_param = false;
      std::cout << "No Gripper found. New IP: " <<  ip << std::endl;
      initAddresses();
      return true;
   }
   catch(const nlohmann::json::parse_error &e)
   {
      ip_changed_with_all_param = true;    
      ip = old_ip;
      std::cout << "message: " << e.what()  << "\nexception id: " << e.id << std::endl;
      std::cout << "Setting to old IP: " << ip << std::endl;
      initAddresses();
      return false;
   }
   catch(const std::exception &e)
   {
      ip_changed_with_all_param = true;
      ip = old_ip;
      std::cout << "Wrong data found. Setting to old IP: " << ip << std::endl;
      initAddresses();
      return false;
   }
  
}
//
Gripper::~Gripper()
{

}
