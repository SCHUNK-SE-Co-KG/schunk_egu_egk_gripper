// Copyright 2024 SCHUNK SE & Co. KG
//
// This program is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation, either version 3 of the License, or (at your option)
// any later version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
// more details.
//
// You should have received a copy of the GNU General Public License along with
// this program. If not, see <https://www.gnu.org/licenses/>.
// --------------------------------------------------------------------------------

/*
 * Author:        Viktoria Krimer (viktoria.krimer@de.schunk.com)
 * Maintainer:    Stefan Scherzinger (stefan.scherzinger@de.schunk.com)
 */


/*
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
#include <schunk_egu_egk_gripper_library/json.hpp>

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
#define COMMANDO_CODE_INST "0x0100"
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
//Datatypes in JSON
#define BOOL_DATA 0
#define INT32_DATA 3
#define UINT8_DATA 4
#define UINT16_DATA 5
#define UINT32_DATA 6
#define CHAR_DATA 7
#define ENUM_DATA 8
#define FLOAT_DATA 18
//Supported Firmware handling in schunk_gripper_lib.hpp void getVersions() && checkVersions() in schunk_gripper_wrapper
#define MIN_SUPPORTED_FIRMWARE_VERSION 501
#define MIN_SUPPORTED_COMMUNICATION_VERSION 1.55
#define MAX_SUPPORTED_FIRMWARE_VERSION 502
#define MAX_SUPPORTED_COMMUNICATION_VERSION 1.55

//Old comm_version 1.55.1 & sw_version 5.2.0.81896
#define ACTUAL_POS_OFFSET "15" //Used for Feedbacks!
#define MIN_ERR_MOT_VOLT_OFFSET "96" //Used in Gripperinfo
#define MEAS_LGC_VOLT_OFFSET "110"   //Used in Gripperinfo

size_t writeCallback(void*, size_t, size_t, void*);

typedef std::array<uint32_t, 4> plc_Array;


class AnybusCom
{

    private:

        CURL *curl;

        std::string update_address;
        std::string data_address;
        std::string enum_address;
        std::string info_address;
        std::string metadata_address;

        uint32_t command;

        std::map<std::string, plc_Array*> plc_pairs
        {
        {PLC_SYNC_INPUT_INST, &plc_sync_input},
        {PLC_SYNC_OUTPUT_INST, &plc_sync_output}
        };

        void updatePlc(std::string &, const std::string &);
        void updateFeedback(const std::string &);

        std::string changeEndianFormat(const std::string &);

        std::vector<std::string> splitResponse(const std::string &, const size_t &, const size_t & data_type_site = 4);

    protected:

        void initAddresses();

        std::string ip;                                                                             //IP to the connected gripper
        int port = 80;                                                                              //TCP/IP Port of the gripper

        uint8_t endian_format;                                                                      //Flag for big/little Endian
        bool not_double_word;                                                                       //Flag if double word is requested (double words are always big Endian)

        const uint32_t mask = FAST_STOP | USE_GPE | GRIP_DIRECTION | REPEAT_COMMAND_TOGGLE;         //Mask for setting Command bits to zero

        uint32_t last_command;                                                                      //Saves last Command
        //Software
        uint16_t sw_version;
        std::string comm_version;
        double comm_version_double;
        uint16_t module_type;                                                                       //module type enum number
        uint16_t fieldbus_type;                                                                     //fieldbus_type enum number
        //Data handling
        std::string save_response_string;

        template<typename paramtype>
        std::string writeValueToString(paramtype);                                                  //Writes a given value to a hexadecimals string
        template<typename paramtype>
        paramtype readParam(std::string);                                                           //Reads a hexadecimal string to value
        template<typename paramtype>
        void updateSaveData(std::vector<paramtype> &vector, const size_t &elements = 1, const size_t &counts = 1);   //Updates variable save_data
        void updatePlcOutput(uint32_t, uint32_t  = 0, uint32_t = 0, uint32_t = 0);                  //Updates plc_sync_output[4]
        //Curl Requests
        void postCommand();                                                                         //Post plc_sync_output[4]
        void postParameter(std::string, std::string);                                               //Post a Parameter with instance
        void getEnums(const char[7],const uint16_t &);                                                      //Get to an enum number the string
        void getMetadata(const std::string &);
        void getInfo();
        template<typename paramtype>
        void getWithInstance(const char inst[7], paramtype *param = NULL, const size_t &elements = 1);      //Get a Parameter with Instance
        template<typename paramtype>                                                                                                          //Function to get Endian format
        void getWithOffset(const std::string &offset, const size_t & count, std::vector<paramtype> &vector, const size_t & elements = 1);      //A Function just for Gripper Feedback

        bool grp_pos_lock;           //GPE

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

        float actual_position, actual_velocity, actual_motor_current;       //actual values (feedback)

    public:

        plc_Array plc_sync_input;   // [0] -> Status double word,  [1]  ->actual Position, [2] ->reserved,      [3] ->diagnose
        plc_Array plc_sync_output;  // [0] -> Control double word, [1]  ->set_position,    [2] -> set_velocity, [3] ->set_effort

        nlohmann::json json_data;       //raw Json-Data

        std::vector<bool> bool_vector;
        std::vector<uint8_t> uint8_vector;
        std::vector<uint16_t> uint16_vector;
        std::vector<uint32_t> uint32_vector;
        std::vector<int32_t> int32_vector;
        std::vector<float> float_vector;
        std::vector<char> char_vector;

        uint16_t grp_prehold_time;   //Grip prehold time

        void performCurlRequest(std::string post);

        AnybusCom(const std::string &ip, int port);
        ~AnybusCom();
    };
//Get something with Instance
template<typename paramtype>
inline void AnybusCom::getWithInstance(const char inst[7], paramtype *param, const size_t &elements)
{
    std::string response;
    CURLcode res;

    std::string instance = inst;
    //Set instance
    std::string address = data_address;
    address.append("inst=" + instance + "&count=1");

    if(curl)
    {
        //GET CURL
        curl_easy_setopt(curl, CURLOPT_URL, address.c_str());
        curl_easy_setopt(curl,CURLOPT_WRITEFUNCTION, writeCallback);
        curl_easy_setopt(curl, CURLOPT_HTTPGET, 1);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
        curl_easy_setopt(curl, CURLOPT_TIMEOUT, 1L);
        curl_easy_setopt(curl, CURLOPT_PORT, port);
        res = curl_easy_perform(curl);

        if (res != CURLE_OK)
        {
            fprintf(stderr, "curl_easy_perform_failed: %s\n", curl_easy_strerror(res));
            curl_easy_reset(curl);
            throw curl_easy_strerror(res);
        }
        else
        {
           if ((instance == PLC_SYNC_INPUT_INST || instance == PLC_SYNC_OUTPUT_INST) && elements == 1) updatePlc(response, instance);  //If double word
           else if(param != NULL && elements == 1) *param = readParam<paramtype>(response); //If single Parameter;
           else save_response_string = response;   //If an array just save the response
        }
    curl_easy_reset(curl);
    }
}
//Receive data with an offset
template<typename paramtype>
inline void AnybusCom::getWithOffset(const std::string &offset, const size_t & count, std::vector<paramtype> &vector,const size_t & elements)
{
    std::string response;
    CURLcode res;

    std::string address = data_address;

    if(offset.length() <= 5) address.append("offset=" + offset + "&count=" + std::to_string(count));
    else throw "Offset to many symbols";

        if(curl)
        {   //GET
            curl_easy_setopt(curl, CURLOPT_URL, address.c_str());
            curl_easy_setopt(curl, CURLOPT_HTTPGET, 1);
            curl_easy_setopt(curl,CURLOPT_WRITEFUNCTION, writeCallback);
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
            curl_easy_setopt(curl, CURLOPT_TIMEOUT, 1);
            curl_easy_setopt(curl, CURLOPT_PORT, port);

            res = curl_easy_perform(curl);

            if (res != CURLE_OK)
            {
            fprintf(stderr, "curl_easy_perform_failed: %s\n", curl_easy_strerror(res));
            curl_easy_reset(curl);
            throw curl_easy_strerror(res);
            }

            else
            {
                if(count == 3 && offset == ACTUAL_POS_OFFSET) updateFeedback(response); //If ROS-Feedback
                else
                {
                    save_response_string = response;
                    updateSaveData<paramtype>(vector, elements, count);
                }
            }
         curl_easy_reset(curl);
        }
}
//Interprets the received bytestring into an variable of float or int(all except int8_t)
template<typename paramtype>
inline paramtype AnybusCom::readParam(std::string hex_str)
{
        if(hex_str.find("[",0) != std::string::npos)
        {
            hex_str.erase(hex_str.begin(), hex_str.begin() + 2);
        }
        if(hex_str.find("]",hex_str.size()-4) != std::string::npos)
        {
            hex_str.erase(hex_str.end()-2, hex_str.end());
        }

        if(endian_format == 0 && not_double_word == true) hex_str = changeEndianFormat(hex_str);    //If little Endian
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
        else if(std::is_same<paramtype, char>::value || std::is_same<paramtype, uint8_t>::value || std::is_same<paramtype, bool>::value)
        {
            std::istringstream hexStream(hex_str);
            uint16_t value;
            hexStream >> std::hex >> value;
            return static_cast<paramtype>(value);
        }
        else   //if an integer
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

    if(endian_format == 0 && not_double_word == true) value_string = changeEndianFormat(value_string);  //If little Endian

    return value_string;
}

template<typename paramtype>
inline void AnybusCom::updateSaveData(std::vector<paramtype> &vector, const size_t &elements, const size_t &count)
{
    //If multiple parameter of the same size at once (saves all in a vector-> No matrix!)
    if(count >= 2)
    {
        std::vector<std::string> splitted;
        splitted = splitResponse(save_response_string, count, sizeof(paramtype)*elements);
        vector.clear();
        vector.resize(count*elements);

        for(size_t i = 0; i < count; i++) //which Parameter
        {
            for(size_t k = 0; k < elements; k++)    //If parameter array
            {
                vector[i*elements + k] = readParam<paramtype>(splitted[i].substr((k * sizeof(paramtype) * 2), sizeof(paramtype) * 2));
            }
        }
    }
    else    //Else less complex
    {
        vector.clear();
        vector.resize(elements);

        for(size_t i = 0; i < elements; i++)
        {
            vector[i] = readParam<paramtype>(save_response_string.substr((i * sizeof(paramtype) * 2) + 2 , sizeof(paramtype) * 2));
            not_double_word = true;
        }
    }
}
#endif
