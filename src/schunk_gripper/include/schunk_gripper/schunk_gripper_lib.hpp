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
   private:


   protected:

   bool handshake;
   bool ip_changed_with_all_param;

   void startGripper();
   bool acknowledge();
   void getActualParameters();
   void getModel();
   bool gripperBitInput(const uint32_t&) const;
   bool gripperBitOutput(const uint32_t&) const;
   bool changeIp(const std::string &);
   uint32_t mm2mu(const float &);

   std::string getErrorString(const uint8_t &);

   std::string model;
   
   bool start_connection;
   bool model_M;
   bool grp_pos_lock;           //Lock grip and position with brake
   bool modus_m;                //Strong or basic Grip

   template <typename parametertype>
   void changeParameter(const char[7] , parametertype, parametertype *param = NULL);
   bool check();
   bool endCondition(); 
   void runGets();
   void runPost(uint32_t command, uint32_t position = 0, uint32_t velocity = 0, uint32_t effort = 0);
   
   std::array<uint8_t, 3> splitDiagnosis();

   public:

   Gripper(const std::string &ip);          //Gripper initialisation
   ~Gripper();

};

//Post a value to an Parameter
template <typename parametertype>
inline void Gripper::changeParameter(const char inst[7], parametertype new_value, parametertype *store)
{
   std::string value = writeValueToString<parametertype>(new_value);
   postParameter(inst, value);
   std::string instance = inst;

   if(std::is_same<parametertype, float>::value) getWithInstance<float>(inst, instFloats.at(instance));
   else getWithInstance<parametertype>(inst, store);
}

#endif