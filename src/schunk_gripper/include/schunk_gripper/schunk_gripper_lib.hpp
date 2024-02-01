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

#include "schunk_gripper/communication.hpp"
#include <chrono>
#include <thread>


extern std::map<std::string, uint32_t> commands_str;

class Gripper : protected AnybusCom
{

   protected:

   void startGripper();                            //get all necessary
   void getActualParameters();                     //Get all necessary parameters
   void getModel();                                //Get the string of the model and set flags
   
   bool start_connection;
   bool model_M;
   std::string model;                              

   bool gripperBitInput(const uint32_t&) const;    //retrieve individual bits from the status double word
   bool gripperBitOutput(const uint32_t&) const;   //retrieve individual bits from the control double word
   
   void acknowledge();                             //acknowledge
   bool changeIp(const std::string &);             //changeIP
   
   bool ip_changed_with_all_param;                 //IP and parameter changed

   uint32_t mm2mu(const float &);                  //Convert millimeters to micrometers. All unsigned -> For control double word

   template <typename parametertype>
   void changeParameter(const char[7] , parametertype, parametertype *param = NULL);                  //Post a parameter
   void runGets();                                                                                    //Get actual data
   void runPost(uint32_t command, uint32_t position = 0, uint32_t velocity = 0, uint32_t effort = 0); //Post control d
   bool check();                                                                                      //Check for errors
   bool endCondition();                                                                               //Is the gripper in an endCondition? -> example Successbit is set

   std::string getErrorString(const uint8_t &);                                         //Get for the error code the error string
   std::array<uint8_t, 3> splitDiagnosis();                                             //Split the diagnosis to get error, warning and not-feasible     

   public:

   Gripper(const std::string &ip);                                                        //Gripper initialisation
   ~Gripper();

};

//Post a value to an Parameter
template <typename parametertype>
inline void Gripper::changeParameter(const char inst[7], parametertype new_value, parametertype *store)
{
   std::string value = writeValueToString<parametertype>(new_value);   //Hexstring Value
   postParameter(inst, value);                                         //Post
   std::string instance = inst;                                       
   //GET and store parameter
   if(std::is_same<parametertype, float>::value) getWithInstance<float>(inst, instFloats.at(instance));
   else getWithInstance<parametertype>(inst, store);
}

#endif