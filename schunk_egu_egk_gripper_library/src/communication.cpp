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
 * Implementation to communicate with the Gripper via AnybusCom 40.
 *
 * This task involves receiving a ByteString via HTTP, interpreting it,
 * and posting a ByteString if any action needs to be performed by the gripper.
 */

#include "schunk_egu_egk_gripper_library/communication.hpp"
//Write server response for storage in the Program
size_t writeCallback(void* contents, size_t size, size_t nmemb, void* userp)
{
    size_t total_size = size * nmemb;
    std::string* response = static_cast<std::string*>(userp);
    response->append(static_cast<char*>(contents), total_size);
    return total_size;
}

//Initialize the plcs and Addresses
AnybusCom::AnybusCom(const std::string &ip, int port) : ip(ip)
{
        initAddresses();  // for post and get
        curl = curl_easy_init();
        curl_easy_setopt(curl, CURLOPT_PORT, port);
}

//Split a hexadecimal String, which represents an Array into its parts (HERE THE DATATYPE IS ALWAYS 4 Bytes)
std::vector<std::string> AnybusCom::splitResponse(const std::string &hex_str, const size_t &count, const size_t &data_type_size)
{
    std::vector<std::string> splitted;
    splitted.resize(count);
    for(size_t i = 0; i < count; i++)
    {
        splitted[i] = hex_str.substr(2+(data_type_size*2 + 3)*i, data_type_size*2);
    }
    return splitted;
}

//Post Plc_sync_output
std::string AnybusCom::changeEndianFormat(const std::string &hexString)
{
    size_t bytes = hexString.size() / 2;        //On byte are to chars
    std::string endian_changed;

    for(size_t i = bytes  ; i > 0; i--)
    {
        endian_changed.append(hexString.substr((i-1)*2,2));
    }
    return endian_changed;
}

/**
 * @brief Performs a CURL request with the given POST data.
 *
 * This function sets up the CURL options, including the URL, POST data, and write callback.
 * It then performs the CURL request and handles the response.
 * If the request fails, an exception is thrown with the corresponding error message.
 *
 * @param post The POST data to send in the request.
 * @throws const char* An exception with the error message if the request fails.
 */
void AnybusCom::performCurlRequest(std::string post)
{
    CURLcode res;
    std::string response;

    curl_easy_setopt(curl, CURLOPT_URL, update_address.c_str());
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, post.c_str());
    curl_easy_setopt(curl, CURLOPT_POST, 1);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 1L);

    res = curl_easy_perform(curl);

    if (res != CURLE_OK)
    {
        fprintf(stderr, "curl_easy_perform_failed: %s\n", curl_easy_strerror(res));
        curl_easy_reset(curl);
        throw curl_easy_strerror(res);
    }
    else
    {
        json_data.clear();
        json_data = nlohmann::json::parse(response);    //Parse server response
        if(json_data["result"] != 0)
        {
            std::string sever_res = "Server response: ";
            sever_res.append(to_string(json_data["result"]));
            std::cout << sever_res << std::endl;
            throw sever_res.c_str();
        }
    }
    curl_easy_reset(curl);
}

/**
 * @brief Posts a command to the Anybus communication module.
 *
 * This function constructs a command string and sends it to the Anybus communication module.
 * The command string is constructed by appending the values of the plc_sync_output array as hexadecimal strings.
 * If the endian_format is 0, indicating big endian, the double_words are treated as big endian.
 *
 * @note This function requires the curl library to be initialized.
 */
void AnybusCom::postCommand()
{
    std::string post = "inst=0x0048&value=";

    if(endian_format == 0) not_double_word = false;                     //Double_words are always big endian

    for(int i = 0; i < 4 ; i++)
    {
        post.append(writeValueToString<uint32_t>(plc_sync_output[i])); //Array to HexString
        not_double_word = true;
    }
    if (curl)
    {
        performCurlRequest(post);
    }
}

/**
 * @brief Posts a parameter to the Anybus communication module.
 *
 * This function posts a parameter to the Anybus communication module
 * using the specified instruction and value. The instruction and value
 * are passed as strings. The function checks the length of the instruction
 * and value to ensure they are within the allowed limits. If the length
 * of either the instruction or value exceeds the limits, an exception is thrown.
 *
 * @param inst The instruction to be posted.
 * @param value The value to be posted.
 * @throws An exception if the length of the instruction or value exceeds the limits.
 */
void AnybusCom::postParameter(std::string inst, std::string value)
{
    std::string post;
    if(inst.length() <= 20 && value.length() <= 500)
    {
        post = "inst=" + inst;
        post.append("&value=" + value);
    }
    else throw "Too many symbols.";

    if (curl)
    {
        performCurlRequest(post);
    }
}

//Inits used Addresses with the ip
void AnybusCom::initAddresses()
{
    data_address = "http:///adi/data.json?";
    update_address = "http:///adi/update.json";
    enum_address = "http:///adi/enum.json?";
    info_address = "http:///adi/info.json";
    metadata_address = "http:///adi/metadata.json?";

    if(ip.size() >= 100) ip = "0.0.0.0";

    data_address.insert(7, ip);
    update_address.insert(7, ip);
    enum_address.insert(7,ip);
    info_address.insert(7,ip);
    metadata_address.insert(7,ip);

}

//Translates the received string of double_word to an integer[4] an saves it in plc_sync_input
void AnybusCom::updatePlc(std::string &hex_str,const std::string &inst)
{
    plc_Array *plc = plc_pairs.at(inst);
    plc->fill(0);

    hex_str.erase(hex_str.begin(), hex_str.begin() + 2);
    hex_str.erase(hex_str.end()-2, hex_str.end());

    std::string save[4];
    not_double_word = false;

    for(int i = 0; i < 4; i++)
    {
       save[i] = hex_str.substr((i * sizeof(uint32_t) * 2),sizeof(uint32_t) * 2);
       plc->at(i) = readParam<uint32_t>(save[i]);
       not_double_word = true;
    }
}

//Feedback strings getting translate and stored in corresponding variables
void AnybusCom::updateFeedback(const std::string &hex_str)
{
    std::vector<std::string> splitted;
    splitted = splitResponse(hex_str, 3, sizeof(float));

    actual_position = readParam<float>(splitted[0]);
    actual_velocity = readParam<float>(splitted[1]);
    actual_motor_current=  readParam<float>(splitted[2]);
}

//Update the plcOutput if a Command is send
void AnybusCom::updatePlcOutput(uint32_t command, uint32_t position, uint32_t velocity, uint32_t effort)
{
    uint32_t actual_command = command & (~GRIP_DIRECTION);      //Command without Grip direction

    if(last_command != actual_command && command != FAST_STOP)             //If not repeating command so do this, and reset the sended Bits
    plc_sync_output[0] &= mask;

    if(GRIP_DIRECTION == (command & GRIP_DIRECTION)) plc_sync_output[0] ^= GRIP_DIRECTION;  //if command have an positive Grip direction bit
                                                                                            //Change the direction

    if(command == FAST_STOP)                                                                //If fast stop is requested
    {
        plc_sync_output[0] &= ~FAST_STOP;
        last_command = 0;
    }
    else if(command == USE_GPE)                                                             //If GPE is requested toggle
    {
        plc_sync_output[0] ^= command;
        last_command = actual_command;
    }
    else if(last_command == actual_command) plc_sync_output[0] ^= REPEAT_COMMAND_TOGGLE;    //REPEAT_COMMAND
    else if(command == EMERGENCY_RELEASE) plc_sync_output[0] |= (FAST_STOP | EMERGENCY_RELEASE);
    else
    {
        last_command = actual_command;
        plc_sync_output[0] |= actual_command;
    }

    plc_sync_output[1] = position;
    plc_sync_output[2] = velocity;
    plc_sync_output[3] = effort;
}

//Gets to enumNumber a corresponding Error String. response is saved in json_data
void AnybusCom::getEnums(const char inst[7], const uint16_t &enumNum)
{
    std::string response;
    CURLcode res;
    std::string instance = inst;
    std::string address = enum_address;
    address.append("inst=" + instance);
    address.append("&value=" + std::to_string(enumNum));    //Enum code to string

    if(curl)
    {
        //GET
        curl_easy_setopt(curl, CURLOPT_URL, address.c_str());
        curl_easy_setopt(curl,CURLOPT_WRITEFUNCTION, writeCallback);
        curl_easy_setopt(curl, CURLOPT_HTTPGET, 1);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
        curl_easy_setopt(curl, CURLOPT_TIMEOUT, 1L);

        res = curl_easy_perform(curl);

        if (res != CURLE_OK)
        {
            fprintf(stderr, "curl_easy_perform_failed: %s\n", curl_easy_strerror(res));
            curl_easy_reset(curl);
            throw curl_easy_strerror(res);
        }

        else
        {
            response.erase(0, 1);
            response.erase(response.size()-1, 1);
            json_data =  nlohmann::json::parse(response);   //Save string in raw json
        }

        curl_easy_reset(curl);
    }
}

//Used for getting dataformat
void AnybusCom::getInfo()
{
    std::string response;
    CURLcode res;

    if(curl)
    {
        //GET
        curl_easy_setopt(curl, CURLOPT_URL, info_address.c_str());
        curl_easy_setopt(curl,CURLOPT_WRITEFUNCTION, writeCallback);
        curl_easy_setopt(curl, CURLOPT_HTTPGET, 1);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
        curl_easy_setopt(curl, CURLOPT_TIMEOUT, 1L);

        res = curl_easy_perform(curl);

        if (res != CURLE_OK)
        {
            fprintf(stderr, "curl_easy_perform_failed: %s\n", curl_easy_strerror(res));
            curl_easy_reset(curl);
            throw curl_easy_strerror(res);
        }

        else
        {
            json_data =  nlohmann::json::parse(response);
            endian_format = json_data["dataformat"];    //Set up data_format
            not_double_word = true;
        }

        curl_easy_reset(curl);
    }
}

void AnybusCom::getMetadata(const std::string &instance)
{
    std::string response;
    CURLcode res;

    std::string address = metadata_address;
    address.append("inst=" + instance);
    address.append("&count=1");    //Metadata

    if(curl)
    {
        //GET
        curl_easy_setopt(curl, CURLOPT_URL, address.c_str());
        curl_easy_setopt(curl,CURLOPT_WRITEFUNCTION, writeCallback);
        curl_easy_setopt(curl, CURLOPT_HTTPGET, 1);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
        curl_easy_setopt(curl, CURLOPT_TIMEOUT, 1L);

        res = curl_easy_perform(curl);

        if (res != CURLE_OK)
        {
            fprintf(stderr, "curl_easy_perform_failed: %s\n", curl_easy_strerror(res));
            curl_easy_reset(curl);
            throw curl_easy_strerror(res);
        }

        else
        {
            response.erase(0, 1);
            response.erase(response.size()-1, 1);
            json_data =  nlohmann::json::parse(response);
        }

        curl_easy_reset(curl);
    }
}

AnybusCom::~AnybusCom()
{
   curl_easy_cleanup(curl);
}
