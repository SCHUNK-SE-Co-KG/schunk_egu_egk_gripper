/*
 * Author:        Viktoria Krimer (viktoria.krimer@de.schunk.com)
 * Maintainer:    Viktoria Krimer (viktoria.krimer@de.schunk.com)
 * Created:       DECEMBER 2023
 * 
 * Implementation to communicate with the Gripper via AnybusCom 40.
 * 
 * This task involves receiving a ByteString via HTTP, interpreting it,
 * and posting a ByteString if any action needs to be performed by the gripper.
 */

#include "schunk_gripper/communication.hpp"
//Write server response for storage in the Program
size_t writeCallback(void* contents, size_t size, size_t nmemb, void* userp)
{
    size_t total_size = size * nmemb;
    std::string* response = static_cast<std::string*>(userp);
    response->append(static_cast<char*>(contents), total_size);
    return total_size;
}
//Initialize the plcs and Addresses
AnybusCom::AnybusCom(const std::string &ip) : ip(ip)
{       
        initAddresses();                  //Init addresses for post and get

        curl1 = curl_easy_init();
        curl2 = curl_easy_init();
        curl3 = curl_easy_init();
        curl4 = curl_easy_init();
        curl5 = curl_easy_init();
        curl6 = curl_easy_init();

}
//Receive data with an offset
void AnybusCom::getWithOffset(const std::string &offset, int count, int elements)
{   
    std::string response;
    CURLcode res;

    std::string address = get_address;
    
    if(offset.length() <= 5) address.append("offset=" + offset + "&count=" + std::to_string(count));
    else throw "Offset to many symbols";

        if(curl3) 
        {   //GET
            curl_easy_setopt(curl3, CURLOPT_URL, address.c_str());
            curl_easy_setopt(curl3, CURLOPT_HTTPGET, 1);
            curl_easy_setopt(curl3,CURLOPT_WRITEFUNCTION, writeCallback);
            curl_easy_setopt(curl3, CURLOPT_WRITEDATA, &response);
            curl_easy_setopt(curl3, CURLOPT_TIMEOUT, 1);

            res = curl_easy_perform(curl3); 
            
            if (res != CURLE_OK)
            {
            fprintf(stderr, "curl_easy_perform_failed: %s\n", curl_easy_strerror(res));
            curl_easy_reset(curl3);
            throw curl_easy_strerror(res);
            }

            else
            {  
                if(count == 3 && offset == ACTUAL_POS_OFFSET) updateFeedback(response); //If ROS-Feedback
                else 
                {   
                    save_response_string = response;
                    updateSaveData<float>(count, elements, float_vector);               //Used only for
                }               
            }
         curl_easy_reset(curl3);
        }
}
//Split a hexadecimal String, which represents an Array into its parts (HERE THE DATATYPE IS ALWAYS 4 Bytes)
std::vector<std::string> AnybusCom::splitResponse(const std::string &hex_str, int count)
{
    std::vector<std::string> splitted;
    splitted.resize(count);

    for(int i = 0; i < count; i++) splitted[i] = hex_str.substr(2 + 11*i, 8);   
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

void AnybusCom::postCommand()
{  
    CURLcode res;
    std::string response;
    
    std::string post;
    post = "inst=0x0048&value=";

    if(endian_format == 0) not_double_word = false;                     //Double_words are always big endian

    for(int i = 0; i < 4 ; i++) 
    {
        post.append(writeValueToString<uint32_t>(plc_sync_output[i])); //Array to HexString
        not_double_word = true;
    }
    if (curl4) 
    {
        //POST
        curl_easy_setopt(curl4, CURLOPT_URL, send_data_address.c_str());
        curl_easy_setopt(curl4, CURLOPT_POSTFIELDS, post.c_str());
        curl_easy_setopt(curl4, CURLOPT_POST, 1);
        curl_easy_setopt(curl4,CURLOPT_WRITEFUNCTION, writeCallback);
        curl_easy_setopt(curl4, CURLOPT_WRITEDATA, &response);
        curl_easy_setopt(curl4, CURLOPT_TIMEOUT, 1L);

        res = curl_easy_perform(curl4);

        if (res != CURLE_OK)
        {
            fprintf(stderr, "curl_easy_perform_failed: %s\n", curl_easy_strerror(res));
            curl_easy_reset(curl4);
            throw curl_easy_strerror(res);
        }
        else  
        {
            json_data.clear();
            json_data = nlohmann::json::parse(response);    //Parse server response
            if(json_data["result"] != 0)  std::cout << "Server response: " << json_data["result"] << std::endl;
        }
    curl_easy_reset(curl4);
    }
}
//Post a Parameter with an instance and Value
void AnybusCom::postParameter(std::string inst, std::string value)
{
    CURLcode res;
    std::string response;
    std::string post;
    if(inst.length() <= 20 && value.length() <= 500)
    {
        post = "inst=" + inst;
        post.append("&value=" + value);
    }
    else throw "To many symbols.";

    if (curl5) 
    {
        curl_easy_setopt(curl5, CURLOPT_URL, send_data_address.c_str());
        curl_easy_setopt(curl5, CURLOPT_POSTFIELDS, post.c_str());
        curl_easy_setopt(curl5,CURLOPT_WRITEFUNCTION, writeCallback);
        curl_easy_setopt(curl5, CURLOPT_WRITEDATA, &response);
        curl_easy_setopt(curl5, CURLOPT_TIMEOUT, 1L);

        res = curl_easy_perform(curl5);

        if (res != CURLE_OK)
        {
            fprintf(stderr, "curl_easy_perform_failed: %s\n", curl_easy_strerror(res));
            curl_easy_reset(curl5);
            throw curl_easy_strerror(res);
        }
        else  
        {
            json_data.clear();
            json_data = nlohmann::json::parse(response);    //Parse server response
            if(json_data["result"] != 0)  std::cout << "Server response: " << json_data["result"] << std::endl;
        }
    }
     curl_easy_reset(curl5);

}
//Inits used Addresses with the ip
void AnybusCom::initAddresses()
{   
    get_address = "http:///adi/data.json?";
    send_data_address = "http:///adi/update.json";
    enum_address = "http:///adi/enum.json?";
    info_address = "http:///adi/info.json";

    if(ip.size() >= 100) ip = "0.0.0.0";
    
    get_address.insert(7, ip);
    send_data_address.insert(7, ip);
    enum_address.insert(7,ip);
    info_address.insert(7,ip);

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
    splitted = splitResponse(hex_str, 3);

    actual_pos = readParam<float>(splitted[0]);
    actual_vel = readParam<float>(splitted[1]);
    actual_cur=  readParam<float>(splitted[2]);
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

    if(curl2) 
    {   
        //GET
        curl_easy_setopt(curl2, CURLOPT_URL, address.c_str());
        curl_easy_setopt(curl2,CURLOPT_WRITEFUNCTION, writeCallback);
        curl_easy_setopt(curl2, CURLOPT_HTTPGET, 1);
        curl_easy_setopt(curl2, CURLOPT_WRITEDATA, &response);
        curl_easy_setopt(curl2, CURLOPT_TIMEOUT, 1L);

        res = curl_easy_perform(curl2);

        if (res != CURLE_OK)
        {
            fprintf(stderr, "curl_easy_perform_failed: %s\n", curl_easy_strerror(res));
            curl_easy_reset(curl2);
            throw curl_easy_strerror(res);
        }

        else
        {
            response.erase(0, 1);
            response.erase(response.size()-1, 1);
            json_data =  nlohmann::json::parse(response);   //Save string in raw json
        }

        curl_easy_reset(curl2);
    }
}
//Used for getting dataformat
void AnybusCom::getInfo()
{
    std::string response;
    CURLcode res;

    if(curl6) 
    {
        //GET
        curl_easy_setopt(curl6, CURLOPT_URL, info_address.c_str());
        curl_easy_setopt(curl6,CURLOPT_WRITEFUNCTION, writeCallback);
        curl_easy_setopt(curl6, CURLOPT_HTTPGET, 1);
        curl_easy_setopt(curl6, CURLOPT_WRITEDATA, &response);
        curl_easy_setopt(curl6, CURLOPT_TIMEOUT, 1L);

        res = curl_easy_perform(curl6);

        if (res != CURLE_OK)
        {
            fprintf(stderr, "curl_easy_perform_failed: %s\n", curl_easy_strerror(res));
            curl_easy_reset(curl6);
            throw curl_easy_strerror(res);
        }

        else
        {
            json_data =  nlohmann::json::parse(response);
            endian_format = json_data["dataformat"];    //Set up data_format
            not_double_word = true;
        }

        curl_easy_reset(curl6);
    }
}

AnybusCom::~AnybusCom()
{
   curl_easy_cleanup(curl1);
   curl_easy_cleanup(curl2);
   curl_easy_cleanup(curl3);
   curl_easy_cleanup(curl4);
   curl_easy_cleanup(curl5);
}
