/*
 * Author:        Viktoria Krimer (viktoria.krimer@de.schunk.com)
 * Maintainer:    Viktoria Krimer (viktoria.krimer@de.schunk.com)
 * Created:       DECEMBER 2023
 * 
 * Definitions to communicate with the Gripper via AnybusCom 40.
 * 
 * This task involves receiving a ByteString via HTTP, interpreting it,
 * and posting a ByteString if any action needs to be performed by the gripper.
 */


#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <curl/curl.h>
#include <iostream>
#include <iomanip>
#include <type_traits>
#include <sstream>
#include <schunk_gripper/json.hpp>

//Control double word
#define FAST_STOP 0x01000000
#define STOP 0x02000000
#define ACKNOWLEDGE 0x04000000
#define PREPARE_FOR_SHUTDOWN 0x08000000
#define SOFT_RESET 0x10000000
#define EMERGENCY_RELEASE 0x20000000
#define REPEAT_COMMAND_TOGGLE 0x40000000
#define GRIP_DIRECTION 0x80000000
#define JOG_MODE_MINUS 0x00010000
#define JOG_MODE_PLUS 0x00020000
#define RELEASE_WORK_PIECE 0x00080000
#define GRIP_WORK_PIECE 0x00100000
#define MOVE_TO_ABSOLUTE_POSITION 0x00200000
#define MOVE_TO_RELATIVE_POSITION 0x00400000
#define GRIP_WORKPIECE_WITH_POSITION 0x00000100
#define USE_GPE 0x00000080

//Status double word BIT POSITIONS
#define READY_FOR_OPERATION 0x01000000
#define CONTROL_AUTHORITY 0x02000000
#define READY_FOR_SHUTDOWN 0x04000000
#define NOT_FEASIBLE 0x08000000
#define SUCCESS 0x10000000
#define COMMAND_RECEIVED_TOGGLE 0x20000000
#define WARNING 0x40000000
#define GRIPPER_ERROR 0x80000000
#define EMERGENCY_RELEASED 0x00010000
#define SOFTWARE_LIMIT 0x00020000
#define NO_WORKPIECE_DETECTED 0x00080000
#define GRIPPED 0x00100000
#define POSITION_REACHED 0x00200000
#define PRE_HOLDING 0x00400000
#define WORK_PIECE_LOST 0x00000100
#define WRONG_WORKPIECE_DETECTED 0x00000200 
#define BRAKE_ACTIVE 0x00000080

//Instance
#define PLC_SYNC_INPUT_INST "0x0040"
#define PLC_SYNC_OUTPUT_INST "0x0048"
#define GRP_PREHOLD_TIME_INST "0x0380"
#define MODULE_TYPE_INST "0x0500"
#define WP_LOST_DISTANCE_INST "0x0528"
#define WP_RELEASE_DELTA_INST "0x0540"
#define GRP_POS_MARGIN_INST "0x0580"
#define GRP_PREPOS_DELTA_INST "0x05A8"
#define ZERO_POS_OFS_INST "0x0610"
#define ERROR_CODE_INST "0x0118"
#define MAX_POS_INST "0x0608"
#define MIN_POS_INST "0x0600"
#define MIN_VEL_INST "0x0628"
#define MAX_VEL_INST "0x0630"

size_t writeCallback(void*, size_t, size_t, void*);

typedef std::array<uint32_t, 4> plc_Array;

class AnybusCom
{

    private:

        CURL *curl1;
        CURL *curl2;
        CURL *curl3;
        CURL *curl4;
        CURL *curl5;

        std::string send_data_address;
        std::string get_address;
        std::string enum_address;

        uint32_t command;

        std::map<std::string, plc_Array*> plc_pairs
        {
        {PLC_SYNC_INPUT_INST, &plc_sync_input},
        {PLC_SYNC_OUTPUT_INST, &plc_sync_output}
        };
        
        void initAddresses();
        void updatePlc(std::string &, const std::string &);
        void updateFeedback(const std::string &);
        std::vector<std::string> splitResponse(const std::string, int);

    protected:

        std::string ip;

        uint32_t last_command;
        uint16_t module_type;

        float max_allow_force;       //strong grip

        const uint32_t mask = FAST_STOP | USE_GPE | GRIP_DIRECTION | REPEAT_COMMAND_TOGGLE;

        void receiveWithOffset(const std::string &offset, int count, int elements);                           //A Function just for Gripper Feedback
       
        template<typename paramtype>
        std::string writeValue2Str(paramtype);
        template<typename paramtype>
        paramtype readParam(std::string);

    public:

        nlohmann::json json_data;

        bool pre_grip;

        uint16_t grp_prehold_time;   //Grip prehold time

        float wp_lost_dst;           //Max. distance after workpiece lost
        float wp_release_delta;      //Workpiece release delta position
        float grp_pos_margin;        //Margin for workpiece detections
        float max_pos;
        float min_pos;
        float min_vel;
        float max_vel;
        float max_grip_force;
        float min_grip_force;
        float grp_prepos_delta;      //Gripping pre-position delta
        float zero_pos_ofs;          //zero position offset
        float actual_pos, actual_vel, actual_cur;

         std::map<std::string, float*> instFloats
        {   
            {WP_LOST_DISTANCE_INST, &wp_lost_dst},
            {WP_RELEASE_DELTA_INST, &wp_release_delta},
            {GRP_POS_MARGIN_INST, &grp_pos_margin},
            {GRP_PREPOS_DELTA_INST, &grp_prepos_delta},
            {ZERO_POS_OFS_INST, &zero_pos_ofs},
            {WP_RELEASE_DELTA_INST, &wp_release_delta}
        };

        std::vector<float> save_data;

        plc_Array plc_sync_input;   // [0] -> Status double word,  [1]  ->actual Position, [2] ->reserved,      [3] ->diagnose 
        plc_Array plc_sync_output;  // [0] -> Control double word, [1]  ->set_position,    [2] -> set_velocity, [3] ->set_effort 

        void updateSavedata(std::string, const int &counts, const int &elements);

        void updatePlcOutput(uint32_t, uint32_t  = 0, uint32_t = 0, uint32_t = 0);
        void postCommand();
        void postParameter(std::string, std::string);   
        template<typename paramtype>
        void getWithInstance(const char inst[7], paramtype *param = NULL);               //Gripper Response
        void getEnums(const char[7],const uint16_t &);

        AnybusCom(std::string ip);
        ~AnybusCom();
    };
//Get something with Instance
template<typename paramtype>
inline void AnybusCom::getWithInstance(const char inst[7], paramtype *param)
{   
    std::string response;
    CURLcode res;

    std::string instance = inst;
    
    std::string address = get_address;
    address.append("inst=" + instance + "&count=1");

    if(curl1) 
    {
        curl_easy_setopt(curl1, CURLOPT_URL, address.c_str());
        curl_easy_setopt(curl1,CURLOPT_WRITEFUNCTION, writeCallback);
        curl_easy_setopt(curl1, CURLOPT_HTTPGET, 1);
        curl_easy_setopt(curl1, CURLOPT_WRITEDATA, &response);
       // curl_easy_setopt(curl, CURLOPT_VERBOSE, 1L); // Enable verbose output
        res = curl_easy_perform(curl1);
        if(res != CURLE_OK)
            fprintf(stderr, "curl_easy_perform() failed: %s\n", curl_easy_strerror(res));
        else
        {
           if (instance == PLC_SYNC_INPUT_INST || instance == PLC_SYNC_OUTPUT_INST) updatePlc(response, instance);
           else   *param = readParam<paramtype>(response);   
        }
    curl_easy_reset(curl1);
    }
}
//Interprets the received bytestring into an variable of float or int(all except int8_t)
template<typename paramtype>
inline paramtype AnybusCom::readParam(std::string hex_str)
{
    if(hex_str.find('[') != std::string::npos)
    {
        hex_str.erase(hex_str.begin(), hex_str.begin() + 2);
        hex_str.erase(hex_str.end()-2, hex_str.end());
    }
    //if float
    if (std::is_same<paramtype, float>::value) 
    {
        uint32_t intValue;
        std::stringstream stream;
        stream << std::hex << hex_str;
        stream >> intValue;

        float floatValue;
        std::memcpy(&floatValue, &intValue, sizeof(float));
        
        return floatValue;
    }
    //if an integer
    else 
    {
        uint32_t l = std::stoul(hex_str, nullptr, 16);
        return static_cast<paramtype>(l);;
    }

}
//Function which converts datatype into bytestring
template<typename paramtype>
inline std::string AnybusCom::writeValue2Str(paramtype param)
{
    std::stringstream stream;
    //if float
     if (std::is_same<paramtype, float>::value)
     {  
        int32_t* pa= reinterpret_cast<int32_t*>(&param);
        stream << std::hex << std::setw(8) << std::setfill('0') << *pa;
     }
    //if integer
    else stream << std::hex << std::setw(sizeof(paramtype) * 2) << std::setfill('0') << param;
    
    return stream.str();
}

#endif