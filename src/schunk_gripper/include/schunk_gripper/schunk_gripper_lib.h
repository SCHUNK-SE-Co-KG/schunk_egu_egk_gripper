#ifndef SCHUNK_GRIPPER_H
#define SCHUNK_GRIPPER_H

#include "schunk_gripper/communication.h"



extern std::map<std::string, uint32_t> commands_str;

class Gripper : protected AnybusCom
{
   private:

   void startGripper();

   protected:

   bool handshake;

   void acknowledge();
   bool gripperBitInput(const uint32_t&) const;
   bool gripperBitOutput(const uint32_t&) const;

   uint32_t mm2mu(const float &);

   std::string getErrorString(const uint8_t &);

   public:

   std::string model;

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

   Gripper(std::string ip);          //Gripper initialisation
   ~Gripper();

};

//Post a value to an Parameter
template <typename parametertype>
inline void Gripper::changeParameter(const char inst[7], parametertype new_value, parametertype *store)
{
   std::string value = writeValue2Str<parametertype>(new_value);
   postParameter(inst, value);
   std::string instance = inst;

   if(std::is_same<parametertype, float>::value) getWithInstance<float>(inst, instFloats.at(instance));
   else getWithInstance<parametertype>(inst, store);
}

#endif