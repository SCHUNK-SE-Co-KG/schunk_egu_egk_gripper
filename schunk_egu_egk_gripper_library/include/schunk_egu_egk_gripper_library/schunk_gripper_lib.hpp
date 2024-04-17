/*
 * Author:        Viktoria Krimer (viktoria.krimer@de.schunk.com)
 * Maintainer:    Viktoria Krimer (viktoria.krimer@de.schunk.com)
 * Created:       DECEMBER 2023
 * 
 * Definitions for implementing common procedures to communicate with the gripper.
 *
 * The gripper initializes at the beginning. It acts as the middleware between 
 * communication and the Ros-Wrapper.
 * 
 */
#ifndef SCHUNK_GRIPPER_HPP
#define SCHUNK_GRIPPER_HPP

#include "schunk_egu_egk_gripper_library/communication.hpp"
#include <chrono>
#include <thread>
#include <mutex>

extern std::recursive_mutex lock_mutex;  //Locks if something is receiving or posting data
extern std::map<std::string, uint32_t> commands_str;

class Gripper : protected AnybusCom
{

   protected:

   void startGripper();                            //get all necessary
   void getActualParameters();                     //Get all necessary parameters
   void getModel();                                //Get the string of the model and set flags
   void getVersions();
   
   bool start_connection;
   bool model_M;
   bool handshake;   
   std::string model;                              

   bool gripperBitInput(const uint32_t&) const;    //retrieve individual bits from the status double word
   bool gripperBitOutput(const uint32_t&) const;   //retrieve individual bits from the control double word
   
   void acknowledge();                             //acknowledge
   bool changeIp(const std::string &);             //changeIP
   
   bool ip_changed_with_all_param;                 //IP and parameter changed

   uint32_t mm2mu(const float &);                  //Convert millimeters to micrometers. All unsigned -> For control double word

   template <typename parametertype>
   void changeOneParameter(const char[7] , parametertype, parametertype *param = NULL);                  //Post a parameter
   template <typename parametertype>
   void changeVectorParameter(const char[7] , std::vector<parametertype>);      //Post a parameter
   void runGets();                                                                                          //Get actual data
   void runPost(uint32_t command, uint32_t position = 0, uint32_t velocity = 0, uint32_t effort = 0); //Post control d
   void getParameter(const std::string& instance, const size_t&, const uint8_t&);
   std::string getErrorString(const uint8_t &);                                         //Get for the error code the error string
   
   void sendAction();
   void sendService(std::unique_lock<std::recursive_mutex> &);

   bool check();                                                                                      //Check for errors
   bool endCondition();                                                                               //Is the gripper in an endCondition? -> example Successbit is set

   std::array<uint8_t, 3> splitDiagnosis();                                             //Split the diagnosis to get error, warning and not-feasible     

   bool post_requested;
   uint32_t set_position;
   uint32_t set_speed;
   uint32_t set_gripping_force;
   uint32_t set_command;

   public:

   Gripper(const std::string &ip);                                                        //Gripper initialisation
   ~Gripper();
};

//Post a value to an Parameter
template <typename parametertype>
inline void Gripper::changeOneParameter(const char inst[7], parametertype new_value, parametertype *store)
{
   std::string value = writeValueToString<parametertype>(new_value);   //Hexstring Value
   postParameter(inst, value);                                         //Post
   std::string instance = inst;                                       

   getWithInstance<parametertype>(inst, store);
}

template <typename parametertype>
inline void Gripper::changeVectorParameter(const char inst[7], std::vector<parametertype> new_value)
{
   std::string value = "";
   for(auto elements : new_value)
   {
      value.append(writeValueToString<parametertype>(elements));          //Hexstring Value
   }
      value.shrink_to_fit();
      std::string instance = inst;
      postParameter(instance, value);                                         //Post
}
#endif
