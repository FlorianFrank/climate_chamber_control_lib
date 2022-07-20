/*
 * @author Florian Frank
 */

#include "ClimateChamberWrapper.h"
#include "ctlib/Socket.hpp"
#include "ctlib/Logging.hpp"


#include <cstdint>
#include <cstdio>

PIL::Logging ClimateChamberWrapper::m_Logger(DEBUG_LVL, nullptr);

extern "C" {
#include "ctlib/ErrorHandler.h"
ClimateChamberWrapper ClimateChamberWrapper_py;

bool Initialize(const char *ipAddr, uint16_t port, uint8_t channel) {
    return ClimateChamberWrapper_py.Initialize(ipAddr, port, channel);
}
bool DeInitialize() {
    return ClimateChamberWrapper_py.DeInitialize();
}
bool RetrieveClimateChamberStatus() {
    return ClimateChamberWrapper_py.RetrieveClimateChamberStatus();
}
float GetCurrentHumidity() {
    return ClimateChamberWrapper_py.GetCurrentHumidity();
}
float GetCurrentTemperature() {
    return ClimateChamberWrapper_py.GetCurrentTemperature();
}
float GetTargetHumidity() {
    return ClimateChamberWrapper_py.GetTargetHumidity();
}
void SetTargetHumidity(float targetHumidity) {
    ClimateChamberWrapper_py.SetTargetHumidity(targetHumidity);
}
float GetTargetTemperature() {
    return ClimateChamberWrapper_py.GetTargetTemperature();
}
void SetTargetTemperature(float targetTemperature) {
    ClimateChamberWrapper_py.SetTargetTemperature(targetTemperature);
}
bool StartExecution() {
    return ClimateChamberWrapper_py.StartExecution();
}
bool StopExecution() {
    return ClimateChamberWrapper_py.StopExecution();
}
bool StartProgram(int programID) {
    return ClimateChamberWrapper_py.StartProgram(programID);
}
bool StopProgram() {
    return ClimateChamberWrapper_py.StopProgram();
}
}

#include <unistd.h> // usleep
#include <cstdarg> // va_start, va_end, va_arg

/** Maximum length of ASCII commands, which can be sent to the climate chamber. */
#define SEND_COMMAND_BUFFER_SIZE 128
/** Maximum length of ASCII responses from the climate chamber. */
#define RECEIVE_RESPONSE_BUFFER_SIZE 512

/**
 * Flag which indicates if the thread is running or not.
 * By setting this flag to false the thread is executed securely.
 */
volatile bool threadRunning = false;


ClimateChamberWrapper::~ClimateChamberWrapper()
{
    DeInitialize();
    delete m_socket;
}


bool ClimateChamberWrapper::Initialize(const char *ipAddr, uint16_t port, uint8_t channel)
{
    m_channel = channel;
    m_socket = new PIL::Socket(TCP, IPv4, "127.0.0.1", 8080, 1000);
    if (m_socket->GetLastError() != PIL_NO_ERROR)
    {
        m_Logger.LogMessage(ERROR_LVL, __FILENAME__, __LINE__, "Error could not create Socket (%d)",
                      PIL_ErrorCodeToString(m_socket->GetLastError()));
        return false;
    }

    m_Logger.LogMessage(DEBUG_LVL, __FILENAME__, __LINE__, "Connect to climate chamber at %s:%d", ipAddr, port);
    std::string ipToConnect = ipAddr;
    m_socket->Connect(ipToConnect, port);
    if (m_socket->GetLastError() != PIL_NO_ERROR)
    {
        m_Logger.LogMessage(ERROR_LVL, __FILENAME__, __LINE__, "Error could not bind helperFiles (%s)",
                            PIL_ErrorCodeToString(m_socket->GetLastError()));
        return false;
    }
    m_Logger.LogMessage(DEBUG_LVL, __FILENAME__, __LINE__, "Climate chamber initialized");
    m_Initialized = true;
    return true;
}


bool ClimateChamberWrapper::DeInitialize()
{
    if (!m_Initialized)
        m_Logger.LogMessage(DEBUG_LVL, __FILENAME__, __LINE__, "Nothing todo Climate Chamber not initialized yet");

    // Exit loop in monitor thread, to allow joining the thread.
    if (!StopMonitorThread())
    {
        m_Logger.LogMessage(ERROR_LVL, __FILENAME__, __LINE__, "Error could not stop monitor thread");
        return false;
    }
    m_Logger.LogMessage(DEBUG_LVL, __FILENAME__, __LINE__, "Deinitialization completed");
    return true;
}


bool ClimateChamberWrapper::RetrieveClimateChamberStatus()
{
    m_Logger.LogMessage(DEBUG_LVL, __FILENAME__, __LINE__, "Call RetrieveClimateChamberStatus");

    std::map<CommandReturnValues, std::string> parsedCommand;
    if (!SendCommandGetResponse(&parsedCommand, GET_TEMPERATURE_HUMIDITY, 0))
        return false;

    // TODO atof and atoi error detection
    m_TemperatureLock.lock();
    m_CurrentTemperature = atof(parsedCommand.find(CURRENT_TEMPERATURE)->second.c_str());
   // m_TargetTemperature = atof(parsedCommand.find(TARGET_TEMPERATURE)->second.c_str());
    m_TemperatureLock.unlock();

    m_HumidityLock.lock();
    m_CurrentHumidity = atof(parsedCommand.find(CURRENT_HUMIDTY)->second.c_str());
   // m_TargetHumidity = atof(parsedCommand.find(TARGET_HUMIDITY)->second.c_str());
    m_HumidityLock.unlock();

    m_Logger.LogMessage(DEBUG_LVL, __FILENAME__, __LINE__,
                   "Retrieve Climate Chamber values: (Current temperature: %f, Target temperature: %f, Current humidity: %f, Target humidity: %f",
                   m_CurrentTemperature, m_TargetTemperature, m_CurrentHumidity, m_TargetHumidity);
    return true;

/*    if (!m_Initialized)
    {
        m_Logger.LogMessage(WARNING_LVL, __FILENAME__, __LINE__, "Climate Chamber not initialized yet");
        return false;
    }
    uint8_t commandBuffer[128];
    uint32_t commandBufferSize = 128;
    if (!CommandCreator(commandBuffer, &commandBufferSize, GET_TEMPERATURE_HUMIDITY, 1, 0))
    {
        m_Logger.LogMessage(ERROR_LVL, __FILENAME__, __LINE__, "Error CommandCreator returns false");
        return false;
    }
    m_Logger.LogMessage(DEBUG_LVL, __FILENAME__, __LINE__, "RetrieveClimateChamber status => send: \n[%s])", commandBuffer);
    if (PIL_Send(m_socket, commandBuffer, &commandBufferSize) != 0)
    {
        m_Logger.LogMessage(ERROR_LVL, __FILENAME__, __LINE__, "Error could not send message");
        return false;
    }
    uint8_t receiveBuffer[512];
    memset(receiveBuffer, 0x00, 512);
    uint16_t bufferLen = 512;
    PIL_Receive(m_socket, receiveBuffer, &bufferLen);
    std::map<ClimateChamberWrapper::CommandReturnValues, std::string> parsedCommandMap;
    if(!CommandParser(receiveBuffer, bufferLen, GET_TEMPERATURE_HUMIDITY, &parsedCommandMap))
    {
        m_Logger.LogMessage(ERROR_LVL, __FUNCTION__, __LINE__, "Error could not parse GET_TEMPERATURE_HUMIDITY");
        return false;
    }



    m_Logger.LogMessage(DEBUG_LVL, __FILENAME__, __LINE__,
                   "Parse answer from. Current temperature: %0.2f, Target temperature: %0.2f, Current humidity: %0.2f, Target humidity: %0.2f",
                   m_CurrentTemperature, m_TargetTemperature, m_CurrentHumidity, m_TargetHumidity);
    m_Running = true;
    return true;*/
}


float ClimateChamberWrapper::GetCurrentTemperature()
{
    if (!m_Initialized)
    {
        m_Logger.LogMessage(WARNING_LVL, __FILENAME__, __LINE__, "Climate Chamber not initialized yet");
        return -273; // Return 0 Kelvin indicating that the chamber is not active
    }

    m_Logger.LogMessage(DEBUG_LVL, __FILENAME__, __LINE__, "Get current temperature: %d", m_CurrentTemperature);

    // Synchronize concurrent access with monitor thread.
    m_TemperatureLock.lock();
    float currentTemperature = m_CurrentTemperature;
    m_TemperatureLock.unlock();

    return currentTemperature;
}


float ClimateChamberWrapper::GetCurrentHumidity()
{
    if (!m_Initialized)
    {
        m_Logger.LogMessage(WARNING_LVL, __FILENAME__, __LINE__, "Climate Chamber not initialized yet");
        return 0;
    }

    if (!m_Running)
    {
        m_Logger.LogMessage(WARNING_LVL, __FILENAME__, __LINE__,
                       "The humidity can only be retrieved when the climate chamber is running");
        return 0;
    }

    m_Logger.LogMessage(DEBUG_LVL, __FILENAME__, __LINE__, "Get current humidity: %d", m_CurrentHumidity);

    // Synchronize concurrent access with monitor thread.
    m_HumidityLock.lock();
    float currentHumidity = m_CurrentHumidity;
    m_HumidityLock.unlock();

    return currentHumidity;
}


float ClimateChamberWrapper::GetTargetTemperature()
{
    // Synchronize concurrent access with monitor thread.
    m_TemperatureLock.lock();
    float targetTemperature = m_TargetTemperature;
    m_TemperatureLock.unlock();

    return targetTemperature;
}


float ClimateChamberWrapper::GetTargetHumidity()
{
    // Synchronize concurrent access with monitor thread.
    m_TemperatureLock.lock();
    float targetTemperature = m_TargetHumidity;
    m_TemperatureLock.unlock();

    return targetTemperature;
}


bool ClimateChamberWrapper::SetTargetTemperature(const float targetTemperature)
{
    if (!m_Initialized)
    {
        m_Logger.LogMessage(WARNING_LVL, __FILENAME__, __LINE__, "Climate Chamber not initialized yet");
        return false;
    }
    m_Logger.LogMessage(DEBUG_LVL, __FILENAME__, __LINE__, "Set target temperature %f", targetTemperature);

    m_TemperatureLock.lock();
    m_TargetTemperature = targetTemperature;
    m_TemperatureLock.unlock();

    return true;
}


bool ClimateChamberWrapper::SetTargetHumidity(const float targetHumidity)
{
    if (!m_Initialized)
    {
        m_Logger.LogMessage(WARNING_LVL, __FILENAME__, __LINE__, "Climate Chamber not initialized yet");
        return false;
    }

    if (targetHumidity > 100)
    {
        m_Logger.LogMessage(WARNING_LVL, __FILENAME__, __LINE__,
                       "Could not set humidity %d only values from [0,100] are allowed");
    }

    m_Logger.LogMessage(DEBUG_LVL, __FILENAME__, __LINE__, "Set target humidity %f", targetHumidity);

    m_HumidityLock.lock();
    m_TargetHumidity = targetHumidity;
    m_HumidityLock.unlock();

    return true;
}


bool ClimateChamberWrapper::StartExecution()
{
    m_Logger.LogMessage(DEBUG_LVL, __FILENAME__, __LINE__,
                   "Start climate chamber execution with values: (temperature: %f, humidity: %f)", m_TargetTemperature,
                   m_TargetHumidity);
    if (!StartStopExecution(100))
        return false;
    m_Running = true;
    return true;
}


bool ClimateChamberWrapper::StopExecution()
{
    m_Logger.LogMessage(DEBUG_LVL, __FILENAME__, __LINE__, "Stop climate chamber execution");

    // TODO verify
    m_Running = false;
    return StartStopExecution(0);
}

bool ClimateChamberWrapper::StartProgram(const int programID)
{
    m_Logger.LogMessage(DEBUG_LVL, __FILENAME__, __LINE__, "Call StartProgram");

    std::map<CommandReturnValues, std::string> parsedCommand;
    if (!SendCommandGetResponse(&parsedCommand, START_PROGRAM, 1, programID))
        return false;

    int ret = atoi(parsedCommand.find(COMMAND_START_PROGRAM_RET)->second.c_str());
    if (ret != 0)
    {
        m_Logger.LogMessage(ERROR_LVL, __FILENAME__, __LINE__, "Start program %d returns with error code %d", programID,
                       ret);
        return false;
    }
    return true;
}


bool ClimateChamberWrapper::StopProgram()
{
    m_Logger.LogMessage(DEBUG_LVL, __FILENAME__, __LINE__, "Call StopProgram");

    std::map<CommandReturnValues, std::string> parsedCommand;
    if (!SendCommandGetResponse(&parsedCommand, START_PROGRAM, 1, 0))
        return false;

    int ret = atoi(parsedCommand.find(COMMAND_START_PROGRAM_RET)->second.c_str());
    if (ret != 0)
    {
        m_Logger.LogMessage(ERROR_LVL, __FILENAME__, __LINE__, "Stop program returns with error code %d", 0, ret);
        return false;
    }
    return true;


    /* if (!m_Initialized)
     {
         m_Logger.LogMessage(WARNING_LVL, __FILENAME__, __LINE__, "Climate Chamber not initialized yet");
         return false;
     }
     uint8_t commandBuffer[128];
     memset(commandBuffer, 0x00, 128);
     uint32_t commandBufferSize = 128;
     if (!CommandCreator(commandBuffer, &commandBufferSize, STOP_PROGRAM, 1, 0))
     {
         m_Logger.LogMessage(ERROR_LVL, __FILENAME__, __LINE__, "Error CommandCreator returns false");
         return false;
     }
     m_Logger.LogMessage(DEBUG_LVL, __FILENAME__, __LINE__, "Stop program status => send: \n[%s])", commandBuffer);
     if (PIL_Send(m_socket, commandBuffer, &commandBufferSize) != 0)
     {
         m_Logger.LogMessage(ERROR_LVL, __FILENAME__, __LINE__, "Error could not send message");
         return false;
     }
     uint8_t receiveBuffer[512];
     memset(receiveBuffer, 0x00, 512);
     uint16_t bufferLen = 512;
     PIL_Receive(m_socket, receiveBuffer, &bufferLen);
     std::map<ClimateChamberWrapper::CommandReturnValues, std::string> parsedCommandMap;
     if(!CommandParser(receiveBuffer, bufferLen, ACKNOWLEDGE_ERRORS, &parsedCommandMap))
     {
         m_Logger.LogMessage(ERROR_LVL, __FUNCTION__, __LINE__, "Error could not parse GET_TEMPERATURE_HUMIDITY");
         return false;
     }

     int ret = atoi(parsedCommandMap.find(COMMAND_ERROR_ACK)->second.c_str());
     if(ret != 0)
     {
         m_Logger.LogMessage(ERROR_LVL, __FILENAME__, __LINE__, "Acknowledge errors returns %d", ret);
         return false;
     }

     m_Logger.LogMessage(DEBUG_LVL, __FILENAME__, __LINE__, "Errors acknowledged successfully");
     return true;*/
}


bool ClimateChamberWrapper::GetErrorCode(int *errCodeRet)
{
    m_Logger.LogMessage(DEBUG_LVL, __FILENAME__, __LINE__, "Call GetErrorCode");

    std::map<CommandReturnValues, std::string> parsedCommand;
    if (!SendCommandGetResponse(&parsedCommand, GET_ERROR, 0))
        return false;

    int ret = atoi(parsedCommand.find(COMMAND_ERR_CODE)->second.c_str());
    m_Logger.LogMessage(DEBUG_LVL, __FILENAME__, __LINE__, "GetErrorCode returns %d", ret);

    *errCodeRet = ret;
    return true;

    /* if (!m_Initialized)
      {
          m_Logger.LogMessage(WARNING_LVL, __FILENAME__, __LINE__, "Climate Chamber not initialized yet");
          return false;
      }
      uint8_t commandBuffer[128];
      uint32_t commandBufferSize = 128;
      if (!CommandCreator(commandBuffer, &commandBufferSize, GET_ERROR, 1, 0))
      {
          m_Logger.LogMessage(ERROR_LVL, __FILENAME__, __LINE__, "Error CommandCreator returns false");
          return false;
      }
      m_Logger.LogMessage(DEBUG_LVL, __FILENAME__, __LINE__, "RetrieveClimateChamber status => send: %s)", commandBuffer);
      if (PIL_Send(m_socket, commandBuffer, &commandBufferSize) != 0)
      {
          m_Logger.LogMessage(ERROR_LVL, __FILENAME__, __LINE__, "Error could not send message");
          return false;
      }
      uint8_t receiveBuffer[512];
      memset(receiveBuffer, 0x00, 512);
      uint16_t bufferLen = 512;
      PIL_Receive(m_socket, receiveBuffer, &bufferLen);
      std::map<ClimateChamberWrapper::CommandReturnValues, std::string> parsedCommandMap;
      if(!CommandParser(receiveBuffer, bufferLen, GET_ERROR, &parsedCommandMap))
      {
          m_Logger.LogMessage(ERROR_LVL, __FUNCTION__, __LINE__, "Error could not parse GET_TEMPERATURE_HUMIDITY");
          return false;
      }
      *errCodeRet = atoi(parsedCommandMap.find(COMMAND_ERR_CODE)->second.c_str());
      m_Logger.LogMessage(DEBUG_LVL, __FILENAME__, __LINE__, "GetCurrentErrorCode returns %d", *errCodeRet);
      return true;*/
}


bool ClimateChamberWrapper::AcknowledgeErrors()
{
    m_Logger.LogMessage(DEBUG_LVL, __FILENAME__, __LINE__, "Call AcknowledgeErrors");

    std::map<CommandReturnValues, std::string> parsedCommand;
    if (!SendCommandGetResponse(&parsedCommand, ACKNOWLEDGE_ERRORS, 0))
        return false;

    int ret = atoi(parsedCommand.find(COMMAND_ERROR_ACK)->second.c_str());
    if (ret != 0)
    {
        m_Logger.LogMessage(ERROR_LVL, __FILENAME__, __LINE__, "AcknowledgeErrors returns error code %d", ret);
        return false;
    }

    return true;

    /*if (!m_Initialized)
    {
        m_Logger.LogMessage(WARNING_LVL, __FILENAME__, __LINE__, "Climate Chamber not initialized yet");
        return false;
    }
    uint8_t commandBuffer[128];
    uint32_t commandBufferSize = 128;
    if (!CommandCreator(commandBuffer, &commandBufferSize, ACKNOWLEDGE_ERRORS, 1, 0))
    {
        m_Logger.LogMessage(ERROR_LVL, __FILENAME__, __LINE__, "Error CommandCreator returns false");
        return false;
    }
    m_Logger.LogMessage(DEBUG_LVL, __FILENAME__, __LINE__, "RetrieveClimateChamber status => send: %s)", commandBuffer);
    if (PIL_Send(m_socket, commandBuffer, &commandBufferSize) != 0)
    {
        m_Logger.LogMessage(ERROR_LVL, __FILENAME__, __LINE__, "Error could not send message");
        return false;
    }
    uint8_t receiveBuffer[512];
    memset(receiveBuffer, 0x00, 512);
    uint16_t bufferLen = 512;
    PIL_Receive(m_socket, receiveBuffer, &bufferLen);
    std::map<ClimateChamberWrapper::CommandReturnValues, std::string> parsedCommandMap;
    if(!CommandParser(receiveBuffer, bufferLen, ACKNOWLEDGE_ERRORS, &parsedCommandMap))
    {
        m_Logger.LogMessage(ERROR_LVL, __FUNCTION__, __LINE__, "Error could not parse GET_TEMPERATURE_HUMIDITY");
        return false;
    }

    int ret = atoi(parsedCommandMap.find(COMMAND_ERROR_ACK)->second.c_str());
    if(ret != 0)
    {
        m_Logger.LogMessage(ERROR_LVL, __FILENAME__, __LINE__, "Acknowledge errors returns %d", ret);
        return false;
    }

    m_Logger.LogMessage(DEBUG_LVL, __FILENAME__, __LINE__, "Errors acknowledged successfully");
    return true;*/
}


bool ClimateChamberWrapper::StartMonitorThread(int intervalMs)
{
    if (intervalMs < 100)
    {
        m_Logger.LogMessage(WARNING_LVL, __FILENAME__, __LINE__,
                       "Monitor Thread interval of %d not allowed value must be greater 100 milliseconds");
        return false;
    }

    m_MonitoringThreadInterval = intervalMs;
    if (!m_Initialized)
    {
        m_Logger.LogMessage(WARNING_LVL, __FILENAME__, __LINE__, "Climate Chamber not initialized yet");
        return false;
    }
    m_Logger.LogMessage(DEBUG_LVL, __FILENAME__, __LINE__,
                   "Start monitor thread with interval %d continuously retrieves humidity and temperature information from the climate chamber",
                   m_MonitoringThreadInterval);
    m_MonitorThread = new std::thread(MonitorThreadFunction, this);
    return true;
}

bool ClimateChamberWrapper::StopMonitorThread()
{
    if (!threadRunning)
    {
        m_Logger.LogMessage(WARNING_LVL, __FILENAME__, __LINE__, "No monitoring thread running");
        return true;
    }
    threadRunning = false;
    m_Logger.LogMessage(DEBUG_LVL, __FILENAME__, __LINE__, "Join thread");
    m_MonitorThread->join();
    m_Logger.LogMessage(DEBUG_LVL, __FILENAME__, __LINE__, "Thread exited");
    delete m_MonitorThread;

    return true;
}

void ClimateChamberWrapper::RegisterHumidityTemperatureCallback(void (*callbackFunc)(float, float))
{
    if (threadRunning && m_Running)
        m_Logger.LogMessage(WARNING_LVL, __FILENAME__, __LINE__,
                       "Thread and climate chamber are still running, maybe you missed some values");
    m_Logger.LogMessage(DEBUG_LVL, __FILENAME__, __LINE__, "Register temperature and humidity callback.");
    m_tempHumCallback = callbackFunc;
}

/*
  # Private Functions.
 */

bool ClimateChamberWrapper::SendCommandGetResponse(std::map<CommandReturnValues, std::string> *parsedCommand,
                                                   ClimateChamberCommand command, int nrArgs, ...)
{
    if (!m_Initialized)
    {
        m_Logger.LogMessage(WARNING_LVL, __FILENAME__, __LINE__, "Climate Chamber not initialized yet");
        return false;
    }

    if (!parsedCommand)
    {
        m_Logger.LogMessage(ERROR_LVL, __FILENAME__, __LINE__, "Map required to store the return values is NULL");
        return false;
    }

    /**
     * SEND COMMAND.
     */
    uint8_t commandBuffer[SEND_COMMAND_BUFFER_SIZE];
    memset(commandBuffer, 0x00, SEND_COMMAND_BUFFER_SIZE);
    uint32_t commandBufferSize = SEND_COMMAND_BUFFER_SIZE;

    m_Logger.LogMessage(DEBUG_LVL, __FILENAME__, __LINE__, "Create command");
    va_list vaList;
    va_start(vaList, nrArgs);
    if (!CommandCreator(commandBuffer, &commandBufferSize, command, m_channel, nrArgs, vaList))
    {
        va_end(vaList);
        m_Logger.LogMessage(ERROR_LVL, __FILENAME__, __LINE__, "Error CommandCreator returns false");
        return false;
    }
    va_end(vaList);

    m_Logger.LogMessage(DEBUG_LVL, __FILENAME__, __LINE__, "RetrieveClimateChamber status => send: \n[%s])", commandBuffer);
    m_socket->Send(commandBuffer, reinterpret_cast<int*>(&commandBufferSize)); // TODO cleanup
    if (m_socket->GetLastError() != PIL_NO_ERROR)
    {
        m_Logger.LogMessage(ERROR_LVL, __FILENAME__, __LINE__, "Error could not send message");
        return false;
    }

    /**
     * RECEIVE RESPONSE.
     */
    uint8_t receiveBuffer[RECEIVE_RESPONSE_BUFFER_SIZE];
    memset(receiveBuffer, 0x00, RECEIVE_RESPONSE_BUFFER_SIZE);
    uint16_t bufferLen = RECEIVE_RESPONSE_BUFFER_SIZE;

    int ret = -1;
   int waitRet = -1;
int ctr = 0;
    do
    {
        waitRet = m_socket->WaitTillDataAvailable();
	if(waitRet <= PIL_SOCK_SUCCESS){
        m_Logger.LogMessage(WARNING_LVL, __FUNCTION__, __LINE__, "No data avail retry %d %s", waitRet,PIL_ErrorCodeToString(m_socket->GetLastError()));
	if(ctr > 5){ // TODO what the hell is that?
        m_Logger.LogMessage(WARNING_LVL, __FUNCTION__, __LINE__, "No data available within 5 seconds");

	 return false;
}
}
ctr++;
    }    while(waitRet <= 0);

    m_Logger.LogMessage(DEBUG_LVL, __FUNCTION__, __LINE__, "Data avail ->Read");

         m_socket->Receive(receiveBuffer, reinterpret_cast<uint32_t*>(&bufferLen));
	printf("%s\n", receiveBuffer);
    m_Logger.LogMessage(DEBUG_LVL, __FUNCTION__, __LINE__, "Read finished");


    if(ret == -1)
    {
        m_Logger.LogMessage(ERROR_LVL, __FUNCTION__, __LINE__, "Error while calling read");

    }

        if (!CommandParser(receiveBuffer, bufferLen, command, parsedCommand))
    {
        m_Logger.LogMessage(ERROR_LVL, __FUNCTION__, __LINE__, "Error could not parse GET_TEMPERATURE_HUMIDITY");
        return false;
    }

    return true;
}


/*static*/ int ClimateChamberWrapper::MonitorThreadFunction(void *ptr)
{
    if (threadRunning)
    {
        m_Logger.LogMessage(WARNING_LVL, __FILENAME__, __LINE__, __FILENAME__,
                       "Monitor thread still running -> exit thread");
        return 0;
    }

    if (!ptr)
    {
        m_Logger.LogMessage(ERROR_LVL, __FILENAME__, __LINE__, __FILENAME__,
                       "No valid CliamteChamberWrapper object passed to the monitor thread -> exit thread");
        return 0;
    }

    m_Logger.LogMessage(DEBUG_LVL, __FILENAME__, __LINE__, "StartMonitorThread");
    auto *climateChamberWrapper = reinterpret_cast<ClimateChamberWrapper *>(ptr);
    threadRunning = true;
    while (threadRunning)
    {
        float previousTemperature = climateChamberWrapper->m_CurrentTemperature;
        float previousHumidity = climateChamberWrapper->m_CurrentHumidity;

        if (!climateChamberWrapper->RetrieveClimateChamberStatus())
            m_Logger.LogMessage(WARNING_LVL, __FILENAME__, __LINE__, "Error while calling RetrieveClimateChamberStatus");

        float currentTemperature = climateChamberWrapper->m_CurrentTemperature;
        float currentHumidity = climateChamberWrapper->m_CurrentHumidity;

        // Call user defined callback function if the temperature or the humidity parameter changes.
        if (climateChamberWrapper->m_tempHumCallback &&
            (previousTemperature != currentTemperature || previousHumidity != currentHumidity))
        {
            climateChamberWrapper->m_tempHumCallback(currentTemperature, currentHumidity);
        }

        // Sleep m_MonitoringThreadInterval milliseconds
        usleep(climateChamberWrapper->m_MonitoringThreadInterval * 1e3);
    }
    return 0;
}


/*static*/ bool ClimateChamberWrapper::CommandCreator(uint8_t *buffer, uint32_t *bufferLen,
                                                      ClimateChamberWrapper::ClimateChamberCommand climateChamberCommand,
                                                      uint16_t channel, int numberArguments, ...)
{
    if (!buffer)
    {
        m_Logger.LogMessage(ERROR_LVL, __FUNCTION__, __LINE__, "Buffer == nullptr");
        return false;
    }

    if (*bufferLen < 5)
    {
        m_Logger.LogMessage(WARNING_LVL, __FUNCTION__, __LINE__,
                       "CommandBuffer must be at least 5 byte long, but bufferLen is %d", *bufferLen);
        return false;
    }

    if (channel < 0 || channel > 31)
    {
        m_Logger.LogMessage(WARNING_LVL, __FUNCTION__, __LINE__, "Invalid channelID %d, allowed values: [0,31]", channel);
        return false;
    }

    // Commands always start with $
    buffer[0] = '$';

    // ChannelID must bet between 0 and 32, default is 1
    if (sprintf(reinterpret_cast<char *>(&buffer[1]), "%02d", channel) < 0)
    {
        m_Logger.LogMessage(ERROR_LVL, __FILENAME__, __LINE__, "Error while calling sprintf");
        return false;
    }

    switch (climateChamberCommand)
    {
        case GET_TEMPERATURE_HUMIDITY:
            // ASCII command $<channelByte1><channelByte0>I\r
            buffer[3] = 'I';
            buffer[4] = '\r';
            *bufferLen = 5;
            return true;
        case SET_TEMPERATURE_HUMIDITY:
        {
            // ASCII command $<channelByte1><channelByte0>E <targetTemperature> <targetHumidity> <StartStop> <additional flags (not used)>\r
            // e.g. $01E 0010.0 0080.0 0100.0 0000.0 0000.0 0000.0 0000.0 010 00000001010000000000000000000\r

            if (*bufferLen < 81)
            {
                m_Logger.LogMessage(ERROR_LVL, __FILENAME__, __LINE__,
                               "Error buffer size to short to send set temperature/humidity command required: %d, actual: %d",
                               81, *bufferLen);
                return false;
            }

            if (numberArguments < 3)
            {
                m_Logger.LogMessage(WARNING_LVL, __FILENAME__, __LINE__,
                               "SET_TEMPERATURE_HUMIDITY requires three arguments %d were given", numberArguments);
                return false;
            }

            va_list ap;
            va_start(ap, numberArguments);
            float temperature = va_arg(ap, double);
            float humidity = va_arg(ap, double);
            float time = va_arg(ap, int);
            va_end(ap);

            if (temperature < -70 || temperature > 200)
            {
                m_Logger.LogMessage(WARNING_LVL, __FILENAME__, __LINE__,
                               "Error temperature value %d not allowed. (Allowed: [0,100].");
                return false;
            }

            if (humidity < 0 || humidity > 100)
            {
                m_Logger.LogMessage(WARNING_LVL, __FILENAME__, __LINE__,
                               "Error humidity value %d not allowed. (Allowed: [0,100].");
                return false;
            }

            buffer[3] = 'E';
            buffer[4] = ' ';
            int sprintfRet = sprintf(reinterpret_cast<char *>(&buffer[5]),
                                     "%06.1f %06.1f %06.1f 0000.0 0000.0 0000.0 000.0 010 00000001010000000000000000000\r",
                                     temperature, humidity, time);
            *bufferLen = sprintfRet + 5;
            if (sprintfRet < 0)
            {
                m_Logger.LogMessage(ERROR_LVL, __FILENAME__, __LINE__, "Error while calling sprintf");
                return false;
            }
            return true;
        }
        case GET_ERROR:
            buffer[3] = 'F';
            buffer[4] = '\r';
            *bufferLen = 5;
            return true;
        case ACKNOWLEDGE_ERRORS:
            buffer[3] = 'Q';
            buffer[4] = '\r';
            *bufferLen = 5;
            return true;
        case START_PROGRAM:
        {
            if (*bufferLen < 8)
            {
                m_Logger.LogMessage(ERROR_LVL, __FILENAME__, __LINE__,
                               "Error buffer size to short to send set temperature/humidity command required: %d, actual: %d",
                               8, *bufferLen);
                return false;
            }

            if (numberArguments < 1)
            {
                m_Logger.LogMessage(WARNING_LVL, __FILENAME__, __LINE__,
                               "START_PROGRAM requires one argument (program number) but only %d were given",
                               numberArguments);
                return false;
            }

            va_list ap;
            va_start(ap, numberArguments);
            const int programID = va_arg(ap,  int);
            va_end(ap);

            if (programID > 9999 || programID < 0)
            {
                m_Logger.LogMessage(WARNING_LVL, __FILENAME__, __LINE__,
                               "Program id %d not accepted (allowed: [0,9999]", programID);
                return false;
            }
            buffer[3] = 'P';
            if (sprintf(reinterpret_cast<char *>(&buffer[4]), "%04d", programID) < 0)
            {
                m_Logger.LogMessage(WARNING_LVL, __FILENAME__, __LINE__, "Error fprintf failed");
            }
            buffer[8] = '\r';
            *bufferLen = 8;
            return true;
        }
        case STOP_PROGRAM:
            if (*bufferLen < 8)
            {
                m_Logger.LogMessage(ERROR_LVL, __FILENAME__, __LINE__,
                               "Error buffer size to short to send set temperature/humidity command required: %d, actual: %d",
                               8, *bufferLen);
                return false;
            }

            buffer[3] = 'P';
            buffer[4] = '0';
            buffer[5] = '0';
            buffer[6] = '0';
            buffer[7] = '0';
            buffer[8] = '\r';
            *bufferLen = 8;
            return true;
    }
    return false;
}


bool ClimateChamberWrapper::CommandParser(const uint8_t *buffer, uint32_t bufferLen,
                                          ClimateChamberWrapper::ClimateChamberCommand commandToParse,
                                          std::map<ClimateChamberWrapper::CommandReturnValues, std::string> *parsedCommand)
{
    if (!buffer)
    {
        m_Logger.LogMessage(ERROR_LVL, __FUNCTION__, __LINE__, "Buffer == nullptr");
        return false;
    }

    if (bufferLen == 0)
    {
        m_Logger.LogMessage(ERROR_LVL, __FUNCTION__, __LINE__, "Invalid buffer length == 0");
        return false;
    }

    std::string s(reinterpret_cast<const char *>(buffer));
    std::string::size_type prev_pos = 0, pos = 0;

    switch (commandToParse)
    {
        case GET_TEMPERATURE_HUMIDITY:
        {
            int ctr = 0;
            // Separate return value after each space
            while ((pos = s.find(' ', pos)) != std::string::npos)
            {
                std::string substring(s.substr(prev_pos, pos - prev_pos));

                // Add parsed value to return list. The order is equal to the order in the CommandReturnValues array
                parsedCommand->insert(
                        std::pair<CommandReturnValues, std::string>(static_cast<CommandReturnValues>(ctr), substring));
                prev_pos = ++pos;
                ctr++;
            }

            if (ctr < 4)
            {
                m_Logger.LogMessage(WARNING_LVL, __FILENAME__, __LINE__, "Command parser failed %d elements returned (required 4) ", ctr);
                return false;
            }
            return true;
        }
        case GET_ERROR:
            parsedCommand->insert(
                    std::pair<CommandReturnValues, std::string>(COMMAND_ERR_CODE, std::string((const char *) buffer)));
            return true;
        case ACKNOWLEDGE_ERRORS:
            parsedCommand->insert(
                    std::pair<CommandReturnValues, std::string>(COMMAND_ERROR_ACK, std::string((const char *) buffer)));
            return true;
        case SET_TEMPERATURE_HUMIDITY:
            parsedCommand->insert(std::pair<CommandReturnValues, std::string>(ACK_TEMPERATURE_HUMIDITY,
                                                                              std::string((const char *) buffer)));
            return true;
        case START_PROGRAM:
            parsedCommand->insert(std::pair<CommandReturnValues, std::string>(COMMAND_START_PROGRAM_RET,
                                                                              std::string((const char *) buffer)));
            return true;
        case STOP_PROGRAM:
            parsedCommand->insert(std::pair<CommandReturnValues, std::string>(COMMAND_STOP_PROGRAM_RET,
                                                                              std::string((const char *) buffer)));
            return true;
        default:
            return false;
    }

    m_Logger.LogMessage(WARNING_LVL, __FILENAME__, __LINE__, "CommandParser: Command not found -> return false");
    return false;
}

bool ClimateChamberWrapper::StartStopExecution(int command)
{
    if (!m_Initialized)
    {
        m_Logger.LogMessage(WARNING_LVL, __FILENAME__, __LINE__, "Climate Chamber not initialized yet");
        return false;
    }
    uint8_t commandBuffer[128];
    uint32_t commandBufferSize = 128;
    if (!CommandCreator(commandBuffer, &commandBufferSize, SET_TEMPERATURE_HUMIDITY, 1, 3, m_TargetTemperature,
                        m_TargetHumidity, command))
    {
        m_Logger.LogMessage(ERROR_LVL, __FILENAME__, __LINE__, "Error CommandCreator returns false");
        return false;
    }
    m_Logger.LogMessage(DEBUG_LVL, __FILENAME__, __LINE__, "Send command %s", commandBuffer);
    m_socket->Send(commandBuffer, reinterpret_cast<int*>(&commandBufferSize));
    if (m_socket->GetLastError() != PIL_NO_ERROR)
    {
        m_Logger.LogMessage(ERROR_LVL, __FILENAME__, __LINE__, "Error could not send message");
        return false;
    }
    uint8_t receiveBuffer[512];
    memset(receiveBuffer, 0x00, 512);
    uint16_t bufferLen = 512;
    m_socket->Receive(receiveBuffer, reinterpret_cast<uint32_t *>(&bufferLen)); // TODO
    std::map<ClimateChamberWrapper::CommandReturnValues, std::string> parsedCommandMap;
    if (!CommandParser(receiveBuffer, bufferLen, SET_TEMPERATURE_HUMIDITY, &parsedCommandMap))
    {
        m_Logger.LogMessage(ERROR_LVL, __FILENAME__, __LINE__, "Error could not parse SET_TEMPERATURE_HUMIDITY");
        return false;
    }


    /* m_Logger.LogMessage(DEBUG_LVL, __FILENAME__, __LINE__, "Climate Chamber is started with target temperature: %d, "
                                                       "target humidity: %d (Command: %s)", m_TargetTemperature,
                    m_TargetHumidity, commandBuffer);*/
    m_Running = true;
    return true;
}


