#include "schunk_gripper/communication.h"
//Write server response for storage in the Programm
size_t writeCallback(void* contents, size_t size, size_t nmemb, void* userp)
{
    size_t total_size = size * nmemb;
    std::string* response = static_cast<std::string*>(userp);
    response->append(static_cast<char*>(contents), total_size);
    return total_size;
}
//Initialize the plcs and Adresses, get actual data and modul 
AnybusCom::AnybusCom(std::string ip) : ip(ip)
{       
        initAdresses();                  //Init adresses for post and get

        curl1 = curl_easy_init();
        curl2 = curl_easy_init();
        curl3 = curl_easy_init();
        curl4 = curl_easy_init();
        curl5 = curl_easy_init();
}
//Receive data with an offset
void AnybusCom::receiveWithOffset(const std::string &offset, int count, int elements)
{   
    std::string response;
    CURLcode res;

    std::string adress = get_adress;
    adress.append("offset=" + offset + "&count=" + std::to_string(count));
        if(curl3) 
        {
            curl_easy_setopt(curl3, CURLOPT_URL, adress.c_str());
            curl_easy_setopt(curl3, CURLOPT_HTTPGET, 1);
            curl_easy_setopt(curl3,CURLOPT_WRITEFUNCTION, writeCallback);
            curl_easy_setopt(curl3, CURLOPT_WRITEDATA, &response);
       //   curl_easy_setopt(curl2, CURLOPT_VERBOSE, 1L); // Enable verbose output
            res = curl_easy_perform(curl3); 
            
            if(res != CURLE_OK)
            {
            fprintf(stderr, "curl_easy_perform() failed: %s\n", curl_easy_strerror(res));
            }

            else
            {   
                if(count == 3 && offset == "15") updateFeedback(response);
                else updateSavedata(response, count, elements);
            }
         curl_easy_reset(curl3);
        }
}
//Savedata is a vector, which stores floats. This is used for printing out te info
void AnybusCom::updateSavedata(std::string hexStr, const int &count, const int &elements)
{
    std::vector<std::string> splitted;
    if(count != 1)
    {
        splitted = splitResponse(hexStr,count);
        savedata.clear();
        savedata.resize(count);

        for(int i = 0; i < elements; i++) savedata[i] = readParam<float>(splitted[i]);
    }
    else
    {
        savedata.clear();
        savedata.resize(elements);

        
        hexStr.erase(hexStr.begin(), hexStr.begin() + 2);
        hexStr.erase(hexStr.end()-2, hexStr.end());

        std::vector<std::string> save;
        save.resize(elements);

        for(int i = 0; i < elements; i++)
        {
            save[i] = hexStr.substr((i * sizeof(float) * 2),sizeof(float) * 2);
            savedata.at(i) = readParam<float>(save[i]);
        }

    }
}
//Split a hexadecimal String, which represents an Array into its parts (HERE THE DATAYTPE IS ALWAYS 4 Bytes)
std::vector<std::string> AnybusCom::splitResponse(const std::string hex_str, int count)
{
    std::vector<std::string> splitted;
    splitted.resize(count);

    for(int i = 0; i < count; i++) splitted[i] = hex_str.substr(2 + 11*i, 8);   
    return splitted;
}
//Post Plc_sync_output
void AnybusCom::postCommand()
{  
    CURLcode res;
    std::string response;
    
    std::string post;
    post = "inst=0x0048&value=";

    for(int i = 0; i < 4 ; i++) post.append(writeValue2Str<uint32_t>(plc_sync_output[i]));     
    if (curl4) 
    {
        //headers = curl_slist_append(headers, "Content-Type: application/x-www-form-urlencoded");
        curl_easy_setopt(curl4, CURLOPT_URL, send_data_adress.c_str());
        curl_easy_setopt(curl4, CURLOPT_POSTFIELDS, post.c_str());
        curl_easy_setopt(curl4, CURLOPT_POST, 1);
        curl_easy_setopt(curl4,CURLOPT_WRITEFUNCTION, writeCallback);
        curl_easy_setopt(curl4, CURLOPT_WRITEDATA, &response);
        curl_easy_setopt(curl4, CURLOPT_TIMEOUT, 10L);
        //curl_easy_setopt(curl3, CURLOPT_TRANSFER_ENCODING, 1L);
        //curl_easy_setopt(curl3, CURLOPT_HTTPHEADER, headers);
        res = curl_easy_perform(curl4);

        json_data.clear();
        json_data = nlohmann::json::parse(response);

        if (res != CURLE_OK)  
        {
            fprintf(stderr, "curl_easy_perform_failed: %s\n", curl_easy_strerror(res));
        }
        else if(json_data["result"] == 0);
        else  std::cout << "Server response: " << json_data["result"] << std::endl;
    
    curl_easy_reset(curl4);
    }
}
//Post a Parameter with an instance and Value
void AnybusCom::postParameter(std::string inst, std::string value)
{
    CURLcode res;
    std::string response;
    std::string post = "inst=" + inst;
    post.append("&value=" + value);

    if (curl5) 
    {
     //   headers = curl_slist_append(headers, "Content-Type: application/x-www-form-urlencoded");
        curl_easy_setopt(curl5, CURLOPT_URL, send_data_adress.c_str());
        curl_easy_setopt(curl5, CURLOPT_POSTFIELDS, post.c_str());
        curl_easy_setopt(curl5,CURLOPT_WRITEFUNCTION, writeCallback);
        curl_easy_setopt(curl5, CURLOPT_WRITEDATA, &response);
        curl_easy_setopt(curl5, CURLOPT_TIMEOUT, 10L);
     //   curl_easy_setopt(curl, CURLOPT_TRANSFER_ENCODING, 1L);
     //   curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
        res = curl_easy_perform(curl5);
        json_data.clear();
        json_data = nlohmann::json::parse(response);

        if (res != CURLE_OK)  fprintf(stderr, "curl_easy_perform_failed: %s\n", curl_easy_strerror(res));
        else if(json_data["result"] == 0);
        else  std::cout << "Server response: " << json_data["result"] << std::endl;
    }
     curl_easy_reset(curl5);

}
//Inits thec used Adresses with the ip
void AnybusCom::initAdresses()
{   
    get_adress = "http:///adi/data.json?";
    get_adress.insert(7, ip);

    send_data_adress = "http:///adi/update.json";
    send_data_adress.insert(7, ip);

    enum_adress = "http:///adi/enum.json?";
    enum_adress.insert(7,ip);
}
//Translates the received string of plc_sync_input to an integer[4] an saves it in plc_sync_input
void AnybusCom::updatePlc(std::string &hex_str,const std::string &inst)
{   
    plc_Array *plc = plc_pairs.at(inst); 
    plc->fill(0);

    hex_str.erase(hex_str.begin(), hex_str.begin() + 2);
    hex_str.erase(hex_str.end()-2, hex_str.end());

    std::string save[4];

    for(int i = 0; i < 4; i++)
    {
       save[i] = hex_str.substr((i * sizeof(uint32_t) * 2),sizeof(uint32_t) * 2);
       plc->at(i) = readParam<uint32_t>(save[i]);
    }

}
//Feedback strings getting translate and stored in coresponding varibles
void AnybusCom::updateFeedback(const std::string &hex_str)
{      
    std::vector<std::string> splitted;
    splitted = splitResponse(hex_str, 3);

    actual_pos = readParam<float>(splitted[0]);
    actual_vel = readParam<float>(splitted[1]);
    actual_cur=  readParam<float>(splitted[2]);
}
//Update the plcOutput if a Command is send
void AnybusCom::updatePlcOutput(uint32_t command,uint32_t position, uint32_t velocity, uint32_t effort)
{   
    uint32_t actual_command = command & (~GRIP_DIRECTION);      //Command without Grip direction

    if(last_command != actual_command && command != FAST_STOP)             //If not repeating command so do this, and reset the sendet Bits
    plc_sync_output[0] &= mask;
    
    if(GRIP_DIRECTION == (command & GRIP_DIRECTION)) plc_sync_output[0] ^= GRIP_DIRECTION;  //if command have an positve Grip direction bit
                                                                                            //Change the direction

    if(command == FAST_STOP) 
    {
        plc_sync_output[0] &= ~FAST_STOP;
        last_command = 0;
    }
    else if(command == USE_GPE) 
    {
        plc_sync_output[0] ^= command;
        last_command = actual_command;
    }
    else if(last_command == actual_command) plc_sync_output[0] ^= REPEAT_COMMAND_TOGGLE;
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
    std::string adress = enum_adress;
    adress.append("inst=" + instance);
    adress.append("&value=" + std::to_string(enumNum));

    if(curl2) 
    {
        curl_easy_setopt(curl2, CURLOPT_URL, adress.c_str());
        curl_easy_setopt(curl2,CURLOPT_WRITEFUNCTION, writeCallback);
        curl_easy_setopt(curl2, CURLOPT_HTTPGET, 1);
        curl_easy_setopt(curl2, CURLOPT_WRITEDATA, &response);
      //  curl_easy_setopt(curl, CURLOPT_VERBOSE, 1L); // Enable verbose output
        res = curl_easy_perform(curl2);

        if(res != CURLE_OK)
        {
        fprintf(stderr, "curl_easy_perform() failed: %s\n", curl_easy_strerror(res));
        }

        else
        {
            response.erase(0, 1);
            response.erase(response.size()-1, 1);
            json_data =  nlohmann::json::parse(response);
        }

        curl_easy_reset(curl2);
    }
}
//clean_up
AnybusCom::~AnybusCom()
{
   curl_easy_cleanup(curl1);
   curl_easy_cleanup(curl2);
   curl_easy_cleanup(curl3);
   curl_easy_cleanup(curl4);
   curl_easy_cleanup(curl5);
}