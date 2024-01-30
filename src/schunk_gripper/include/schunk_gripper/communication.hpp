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


#ifndef COMMUNICATION_HPP
#define COMMUNICATION_HPP

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
#define BRAKE_TEST 0x00000040
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
#define FIELDBUS_TYPE_INST "0x1130"
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
#define MAX_GRP_VEL_INST "0x0650"
#define MAX_ALLOW_FORCE_INST "0x06A8"
#define MIN_GRP_FORCE_INST "0x0658"
#define MAX_GRP_FORCE_INST "0x0660"
#define UPTIME_INST "0x1400"
#define MEAS_LGC_TEMP_INST "0x0840"
#define ORDER_NO_TXT_INST "0x1008"
#define MAC_ADDR_INST "0x1138"
#define DEAD_LOAD_KG_INST "0x03A8"
#define TOOL_CENT_POINT_INST "0x03B0"
#define CENT_OF_MASS_INST "0x03B8"
#define USED_CUR_LIMIT_INST "0x0210"
#define MAX_PHYS_STROKE_INST "0x0588"

#define SERIAL_NO_NUM_INST "0x1020"
#define SW_VERSION_NUM_INST "0x1110"
#define COMM_VERSION_TXT_INST "0x1120"
#define SERIAL_NO_TXT_INST "0x1000"
#define ORDER_NO_TXT_INST "0x1008"
#define SW_BUILD_DATE_INST "0x1100"
#define SW_BUILD_TIME_INST "0x1108"
#define SW_VERSION_TXT_INST "0x1118"

//Offset (Mostly used for gripper_info)
/*
//New comm_version 2.1 & sw_version 5.3.0
#define ACTUAL_POS_OFFSET "15" //Used for Feedbacks!
#define MIN_ERR_MOT_VOLT_OFFSET "94"
#define MEAS_LGC_VOLT_OFFSET "108"
*/
//Old comm_version 1.55.1 & sw_version 5.2.0.81896
#define ACTUAL_POS_OFFSET "15" //Used for Feedbacks!
#define MIN_ERR_MOT_VOLT_OFFSET "96"
#define MEAS_LGC_VOLT_OFFSET "110"

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
        CURL *curl6;

        std::string send_data_address;
        std::string get_address;
        std::string enum_address;
        std::string info_address;

        uint32_t command;

        std::map<std::string, plc_Array*> plc_pairs
        {
        {PLC_SYNC_INPUT_INST, &plc_sync_input},
        {PLC_SYNC_OUTPUT_INST, &plc_sync_output}
        };
        
        void updatePlc(std::string &, const std::string &);
        void updateFeedback(const std::string &);

        std::string changeEndianFormat(const std::string &);

        std::vector<std::string> splitResponse(const std::string &, int);

    protected:

        std::string ip;                                                                             //IP to the connected gripper

        uint8_t endian_format;                                                                      //Flag for big/little Endian
        bool not_double_word;                                                                       //Flag if double word is requested (double words are always big Endian)

        const uint32_t mask = FAST_STOP | USE_GPE | GRIP_DIRECTION | REPEAT_COMMAND_TOGGLE;         //Mask for setting Command bits to zero 

        void initAddresses();                                                                       
        void getInfo();                                                                                    //Function to get Endian format
        void getWithOffset(const std::string &offset, int count, int elements, bool is_float = true);      //A Function just for Gripper Feedback

        uint32_t last_command;                                                                      //Saves last Command

        uint16_t sw_version;
        std::string comm_version;
        uint16_t module_type;                                                                       //module type enum number
        uint16_t fieldbus_type;                                                                     //fieldbus_type enum number
                                                                                                            
        template<typename paramtype>
        std::string writeValueToString(paramtype);                                                  //Writes a given value to a hexadecimals string
        template<typename paramtype>                                                                
        paramtype readParam(std::string);                                                           //Reads a hexadecimal string to value

        void updateSavedata(std::string, const int &counts, const int &elements, bool is_float = true);                   //Updates variable save_data
        void updatePlcOutput(uint32_t, uint32_t  = 0, uint32_t = 0, uint32_t = 0);                  //Updates plc_sync_output[4]
        void postCommand();                                                                         //Post plc_sync_output[4]
        void postParameter(std::string, std::string);                                               //Post a Parameter with instance
        template<typename paramtype>    
        void getWithInstance(const char inst[7], paramtype *param = NULL, int elements = 1);                          //Get a Parameter with Instance
        void getEnums(const char[7],const uint16_t &);                                              //Get to an enum number the string

    public:
        
        plc_Array plc_sync_input;   // [0] -> Status double word,  [1]  ->actual Position, [2] ->reserved,      [3] ->diagnose 
        plc_Array plc_sync_output;  // [0] -> Control double word, [1]  ->set_position,    [2] -> set_velocity, [3] ->set_effort 

        nlohmann::json json_data;       //Json-Data used in get... 

        std::vector<float> save_data_float;   //Can save multiple floats from getOffset
        std::vector<char> save_data_char;   //Can save multiple floats from getOffset

        uint16_t grp_prehold_time;   //Grip prehold time
        
        bool grp_pos_lock;

        float wp_lost_dst;           //Max. distance after workpiece lost
        float wp_release_delta;      //Workpiece release delta position
        float grp_pos_margin;        //Margin for workpiece detections
        float grp_prepos_delta;      //Gripping pre-position delta
        float zero_pos_ofs;          //zero position offset
        
        float max_pos;               //max absolute position of gripper
        float min_pos;               //min absolute position of gripper           
        float min_vel;               //min velocity
        float max_vel;               //max velocity
        float max_grip_force;        //max. grip force
        float min_grip_force;        //min. grip_force
        float max_allow_force;       //strong grip
        float max_grp_vel;           //max. grip velocity

        float actual_pos, actual_vel, actual_cur;       //actual values (feedback) 

         std::map<std::string, float*> instFloats       
        {   
            {WP_LOST_DISTANCE_INST, &wp_lost_dst},
            {WP_RELEASE_DELTA_INST, &wp_release_delta},
            {GRP_POS_MARGIN_INST, &grp_pos_margin},
            {GRP_PREPOS_DELTA_INST, &grp_prepos_delta},
            {ZERO_POS_OFS_INST, &zero_pos_ofs},
            {WP_RELEASE_DELTA_INST, &wp_release_delta}
        };

        AnybusCom(const std::string &ip);
        ~AnybusCom();
    };
//Get something with Instance
template<typename paramtype>
inline void AnybusCom::getWithInstance(const char inst[7], paramtype *param, int elements)
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
        curl_easy_setopt(curl1, CURLOPT_TIMEOUT, 1L);
       // curl_easy_setopt(curl, CURLOPT_VERBOSE, 1L); // Enable verbose output
        res = curl_easy_perform(curl1);
        
        if (res != CURLE_OK)
        {
            fprintf(stderr, "curl_easy_perform_failed: %s\n", curl_easy_strerror(res));
            curl_easy_reset(curl1);
            throw curl_easy_strerror(res);
        }
        else
        {
           if (instance == PLC_SYNC_INPUT_INST || instance == PLC_SYNC_OUTPUT_INST) updatePlc(response, instance);
           else if(std::is_same<paramtype, char>::value) updateSavedata(response, 1, elements, false);
           else if(std::is_same<paramtype, float>::value && elements > 1) updateSavedata(response, 1, elements);
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

        if(endian_format == 0 && not_double_word == true) hex_str = changeEndianFormat(hex_str);
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
        else if(std::is_same<paramtype, char>::value)
        {
        std::istringstream hexStream(hex_str);
        uint16_t value;
        hexStream >> std::hex >> value;
        return static_cast<char>(value);
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
inline std::string AnybusCom::writeValueToString(paramtype param)
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

    std::string value_string = stream.str();

    if(endian_format == 0 && not_double_word == true) value_string = changeEndianFormat(value_string);

    return value_string;
}

#endif