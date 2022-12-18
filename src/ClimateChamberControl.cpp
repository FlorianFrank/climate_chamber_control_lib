/**
 * @author Florian Frank
 * @copyright University of Passau - Chair of computer engineering
 */
#include "ClimateChamberControl.h"
#include "CommandCreator.h"
#include <Util.h>
#include "ctlib/Socket.hpp"
#include "ctlib/Logging.hpp"


#include <cstdint>
#include <cstdio>

extern "C" {
#include "ctlib/ErrorHandler.h"
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


ClimateChamberControl::ClimateChamberControl(): m_Logger(DEBUG_LVL, nullptr){
    m_Util = new Util;
    m_CommandCreator = new CommandCreator(m_Util);
}

ClimateChamberControl::~ClimateChamberControl()
{
    deInitialize();
    delete m_socket;
}


PIL_ERROR_CODE ClimateChamberControl::initialize(const std::string &ipAddr, uint16_t port, uint8_t channel)
{
    m_channel = channel;

    m_socket = new PIL::Socket(TCP, IPv4, "localhost", DEFAULT_PORT, DEFAULT_TIMEOUT);
    if (m_socket->GetLastError() != PIL_NO_ERROR)
        return m_Util->logMessageAndReturn(m_socket->GetLastError(), ERROR_LVL, __FILENAME__, __LINE__, "Error could not create Socket (%d)",
                                   PIL_ErrorCodeToString(m_socket->GetLastError()));

    auto errCode = establishTCPConnection(ipAddr, DEFAULT_PORT);
    if(errCode != PIL_NO_ERROR)
        return errCode;

    m_Logger.LogMessage(DEBUG_LVL, __FILENAME__, __LINE__, "Climate chamber initialized");
    m_Initialized = true;
    return PIL_NO_ERROR;
}


PIL_ERROR_CODE ClimateChamberControl::deInitialize()
{
    if (!m_Initialized)
        m_Logger.LogMessage(DEBUG_LVL, __FILENAME__, __LINE__, "Nothing todo Climate "
                                                               "Chamber not initialized yet");

    // Exit loop in monitor thread, to allow joining the thread.
    auto errCode = stopMonitorThread();
    if (errCode != PIL_NO_ERROR)
        return m_Util->logMessageAndReturn(errCode, ERROR_LVL, __FILENAME__, __LINE__,
                                   "Error could not stop monitor thread");

    m_Logger.LogMessage(DEBUG_LVL, __FILENAME__, __LINE__, "De-initialization completed");
    return PIL_NO_ERROR;
}


PIL_ERROR_CODE ClimateChamberControl::retrieveClimateChamberStatus()
{
    m_Logger.LogMessage(DEBUG_LVL, __FILENAME__, __LINE__, "Call retrieveClimateChamberStatus");

    std::map<CommandReturnValues, std::string> parsedCommand;
    auto errCode = sendCommandGetResponse(&parsedCommand, GET_TEMPERATURE_HUMIDITY, 0);
    if (errCode != PIL_NO_ERROR)
        return errCode;

    m_TemperatureLock.lock();
    m_CurrentTemperature = std::stof(parsedCommand.find(CURRENT_TEMPERATURE)->second);
    m_TemperatureLock.unlock();

    m_HumidityLock.lock();
    m_CurrentHumidity = std::stof(parsedCommand.find(CURRENT_HUMIDTY)->second);
    m_HumidityLock.unlock();

    m_Logger.LogMessage(DEBUG_LVL, __FILENAME__, __LINE__,
                   "Retrieve Climate Chamber values: (Current temperature: %f, Target temperature: %f, "
                   "Current humidity: %f, Target humidity: %f", m_CurrentTemperature, m_TargetTemperature,
                   m_CurrentHumidity, m_TargetHumidity);
    return PIL_NO_ERROR;
}


float ClimateChamberControl::getCurrentTemperature()
{
    if (!m_Initialized)
    {
        m_Logger.LogMessage(WARNING_LVL, __FILENAME__, __LINE__, "Climate Chamber not initialized yet");
        return ZERO_KELVIN; // Return 0 Kelvin indicating that the chamber is not active
    }

    m_Logger.LogMessage(DEBUG_LVL, __FILENAME__, __LINE__, "Get current temperature: %d", m_CurrentTemperature);

    // Synchronize concurrent access with monitor thread.
    const std::lock_guard<std::mutex> lock(m_TemperatureLock);
    return m_CurrentTemperature;
}


float ClimateChamberControl::getCurrentHumidity()
{
    if (!m_Initialized){
        m_Util->logMessageAndReturn(PIL_INTERFACE_CLOSED, WARNING_LVL, __FILENAME__, __LINE__, "Climate Chamber not initialized yet");
        return INVALID_HUMIDITY;
    }

    if (!m_Running)
        return m_Util->logMessageAndReturn(PIL_INTERFACE_CLOSED, WARNING_LVL, __FILENAME__, __LINE__,
                                   "The humidity can only be retrieved when the climate chamber is running");

    m_Logger.LogMessage(DEBUG_LVL, __FILENAME__, __LINE__, "Get current humidity: %d", m_CurrentHumidity);

    // Synchronize concurrent access with monitor thread.
    const std::lock_guard<std::mutex> lockGuard(m_HumidityLock);
    return m_CurrentHumidity;
}


float ClimateChamberControl::getTargetTemperature()
{
    // Synchronize concurrent access with monitor thread.
    const std::lock_guard<std::mutex> lockGuard(m_TemperatureLock);
    return m_TargetTemperature;
}


float ClimateChamberControl::getTargetHumidity()
{
    // Synchronize concurrent access with monitor thread.
    const std::lock_guard<std::mutex> lockGuard(m_HumidityLock);
    return m_TargetHumidity;
}


PIL_ERROR_CODE ClimateChamberControl::setTargetTemperature(const float targetTemperature)
{
    if (!m_Initialized)
        return m_Util->logMessageAndReturn(PIL_INTERFACE_CLOSED, WARNING_LVL, __FILENAME__,
                                   __LINE__, "Climate Chamber not initialized yet");

    m_Logger.LogMessage(DEBUG_LVL, __FILENAME__, __LINE__, "Set target temperature %f", targetTemperature);

    const std::lock_guard<std::mutex> lockGuard(m_TemperatureLock);
    m_TargetTemperature = targetTemperature;

    return PIL_NO_ERROR;
}


PIL_ERROR_CODE ClimateChamberControl::setTargetHumidity(const float targetHumidity)
{
    if (!m_Initialized)
        return m_Util->logMessageAndReturn(PIL_INTERFACE_CLOSED, WARNING_LVL, __FILENAME__, __LINE__, "Climate Chamber not initialized yet");

    if (targetHumidity > MAX_HUMIDITY)
        return m_Util->logMessageAndReturn(PIL_INVALID_ARGUMENTS, WARNING_LVL, __FILENAME__, __LINE__,
                                   "Could not set humidity %d only values from [0,100] are allowed");

    m_Logger.LogMessage(DEBUG_LVL, __FILENAME__, __LINE__, "Set target humidity %f", targetHumidity);

    const std::lock_guard<std::mutex> lockGuard(m_HumidityLock);
    m_TargetHumidity = targetHumidity;

    return PIL_NO_ERROR;
}


PIL_ERROR_CODE ClimateChamberControl::startExecution()
{
    m_Logger.LogMessage(DEBUG_LVL, __FILENAME__, __LINE__,
                   "Start climate chamber execution with values: (temperature: %f, humidity: %f)", m_TargetTemperature,
                   m_TargetHumidity);
    auto errCode = startStopExecution(100);
    if (errCode != PIL_NO_ERROR) // TODO what is command 100 don't use magic numbers
        return errCode;
    m_Running = true;
    return PIL_NO_ERROR;
}


PIL_ERROR_CODE ClimateChamberControl::stopExecution()
{
    m_Logger.LogMessage(DEBUG_LVL, __FILENAME__, __LINE__, "Stop climate chamber execution");

    // TODO verify
    m_Running = false;
    return startStopExecution(0); // TODO what is command 0 don't use magic numbers
}

PIL_ERROR_CODE ClimateChamberControl::startProgram(const int programID)
{
    m_Logger.LogMessage(DEBUG_LVL, __FILENAME__, __LINE__, "Call startProgram");

    std::map<CommandReturnValues, std::string> parsedCommand;
    auto errCode = sendCommandGetResponse(&parsedCommand, START_PROGRAM, 1, programID);
    if (errCode != PIL_NO_ERROR)
        return errCode;

    int ret = std::stoi(parsedCommand.find(COMMAND_START_PROGRAM_RET)->second);
    if (ret != 0) // TODO what is ret 0, use explicit error code enums
        return m_Util->logMessageAndReturn(PIL_INVALID_ARGUMENTS, ERROR_LVL, __FILENAME__, __LINE__,
                                   "Start program %d returns with error code %d", programID, ret);
    return PIL_NO_ERROR;
}


PIL_ERROR_CODE ClimateChamberControl::stopProgram()
{
    m_Logger.LogMessage(DEBUG_LVL, __FILENAME__, __LINE__, "Call stopProgram");

    std::map<CommandReturnValues, std::string> parsedCommand;
    auto errCode = sendCommandGetResponse(&parsedCommand, START_PROGRAM, 1, 0);
    if (errCode != PIL_NO_ERROR)
        return errCode;

    int ret = std::stoi(parsedCommand.find(COMMAND_START_PROGRAM_RET)->second);
    if (ret != 0) // TODO what is ret != 0
        return m_Util->logMessageAndReturn(PIL_INVALID_ARGUMENTS, ERROR_LVL, __FILENAME__, __LINE__, "Stop program returns with error code %d",
                                   0, ret);
    return PIL_NO_ERROR;
}


PIL_ERROR_CODE ClimateChamberControl::acknowledgeErrors()
{
    m_Logger.LogMessage(DEBUG_LVL, __FILENAME__, __LINE__, "Call acknowledgeErrors");

    std::map<CommandReturnValues, std::string> parsedCommand;
    auto errCode = sendCommandGetResponse(&parsedCommand, ACKNOWLEDGE_ERRORS, 0);
    if (errCode != PIL_NO_ERROR)
        return errCode;

    int ret = std::stoi(parsedCommand.find(COMMAND_ERROR_ACK)->second);
    if (ret != 0)
        return m_Util->logMessageAndReturn(PIL_INVALID_ARGUMENTS, ERROR_LVL, __FILENAME__, __LINE__, "acknowledgeErrors returns error code %d",
                                   ret);
    return PIL_NO_ERROR;
}


PIL_ERROR_CODE ClimateChamberControl::startMonitorThread(int intervalMs)
{
    m_MonitoringThreadInterval = intervalMs;
    if (!m_Initialized)
        return m_Util->logMessageAndReturn(PIL_INTERFACE_CLOSED, WARNING_LVL, __FILENAME__, __LINE__, "Climate Chamber not initialized yet");

    m_Logger.LogMessage(DEBUG_LVL, __FILENAME__, __LINE__,
                        "Start monitor thread with interval %d continuously retrieves humidity and temperature information from the climate chamber",
                        m_MonitoringThreadInterval);
    m_MonitorThread = new std::thread(monitorThreadFunction, this);
    return PIL_NO_ERROR;
}

PIL_ERROR_CODE ClimateChamberControl::stopMonitorThread()
{
    if (!threadRunning)
        return m_Util->logMessageAndReturn(PIL_THREAD_NOT_FOUND, WARNING_LVL, __FILENAME__,
                                   __LINE__, "No monitoring thread running");

    threadRunning = false;
    m_Logger.LogMessage(DEBUG_LVL, __FILENAME__, __LINE__, "Join thread");
    m_MonitorThread->join();
    m_Logger.LogMessage(DEBUG_LVL, __FILENAME__, __LINE__, "Thread exited");
    delete m_MonitorThread;

    return PIL_NO_ERROR;
}

PIL_ERROR_CODE ClimateChamberControl::registerHumidityTemperatureCallback(void (*callbackFunc)(float, float))
{
    if (threadRunning && m_Running)
        m_Logger.LogMessage(WARNING_LVL, __FILENAME__, __LINE__,
                       "Thread and climate chamber are still running, maybe you missed some values");

    m_Logger.LogMessage(DEBUG_LVL, __FILENAME__, __LINE__, "Register temperature and humidity callback.");
    m_tempHumCallback = callbackFunc;
    return PIL_NO_ERROR;
}

/*
  # Private Functions.
 */

PIL_ERROR_CODE ClimateChamberControl::sendCommandGetResponse(std::map<CommandReturnValues, std::string> *parsedCommand,
                                                   ClimateChamberCommand command, int nrArgs, ...)
{
    if (!m_Initialized)
        return m_Util->logMessageAndReturn(PIL_INTERFACE_CLOSED, WARNING_LVL, __FILENAME__,
                                   __LINE__, "Climate Chamber not initialized yet");

    if (!parsedCommand)
        return m_Util->logMessageAndReturn(PIL_INVALID_ARGUMENTS, ERROR_LVL, __FILENAME__, __LINE__,
                                   "Map required to store the return values is NULL");

    /**
     * SEND COMMAND.
     */
    uint8_t commandBuffer[SEND_COMMAND_BUFFER_SIZE];
    memset(commandBuffer, 0x00, SEND_COMMAND_BUFFER_SIZE);
    uint32_t commandBufferSize = SEND_COMMAND_BUFFER_SIZE;

    m_Logger.LogMessage(DEBUG_LVL, __FILENAME__, __LINE__, "Create command");
    va_list vaList;
    va_start(vaList, nrArgs);
    auto errCode = m_CommandCreator->createCommand(commandBuffer, &commandBufferSize, command, m_channel, nrArgs, vaList);
    if (errCode != PIL_NO_ERROR)
    {
        va_end(vaList); // TODO isn't there a way to clean it up automatically after leaving the scope of the function
        return m_Util->logMessageAndReturn(errCode, ERROR_LVL, __FILENAME__, __LINE__, "Error commandCreator returns false");
    }

    va_end(vaList);

    m_Logger.LogMessage(DEBUG_LVL, __FILENAME__, __LINE__, "RetrieveClimateChamber status => send: \n[%s])", commandBuffer);
    m_socket->Send(commandBuffer, reinterpret_cast<int*>(&commandBufferSize)); // TODO cleanup
    if (m_socket->GetLastError() != PIL_NO_ERROR)
        return m_Util->logMessageAndReturn(m_socket->GetLastError(), ERROR_LVL, __FILENAME__, __LINE__, "Error could not send message");

    /**
     * RECEIVE RESPONSE.
     */
    uint8_t receiveBuffer[RECEIVE_RESPONSE_BUFFER_SIZE];
    memset(receiveBuffer, 0x00, RECEIVE_RESPONSE_BUFFER_SIZE);
    uint16_t bufferLen = RECEIVE_RESPONSE_BUFFER_SIZE;

    int ret = -1;
   int waitRet;
int ctr = 0;
    do
    {
        waitRet = m_socket->WaitTillDataAvailable();
        if (waitRet <= PIL_SOCK_SUCCESS)
        {
            m_Logger.LogMessage(WARNING_LVL, __FUNCTION__, __LINE__, "No data avail retry %d %s", waitRet,
                                PIL_ErrorCodeToString(m_socket->GetLastError()));
            if (ctr > 5)
            { // TODO what the hell is that?
                m_Logger.LogMessage(WARNING_LVL, __FUNCTION__, __LINE__, "No data available within 5 seconds");

                return PIL_INVALID_ARGUMENTS;
            }
        }
        ctr++;
    } while (waitRet <= 0);

    m_Logger.LogMessage(DEBUG_LVL, __FUNCTION__, __LINE__, "Data avail ->Read");

    m_socket->Receive(receiveBuffer, reinterpret_cast<uint32_t *>(&bufferLen));
    printf("%s\n", receiveBuffer);
    m_Logger.LogMessage(DEBUG_LVL, __FUNCTION__, __LINE__, "Read finished");


    if (ret == -1)
        m_Logger.LogMessage(ERROR_LVL, __FUNCTION__, __LINE__, "Error while calling read"); // Should we return false?

    errCode = commandParser(receiveBuffer, bufferLen, command, parsedCommand);
    if (errCode != PIL_NO_ERROR)
        return m_Util->logMessageAndReturn(errCode, ERROR_LVL, __FUNCTION__, __LINE__,
                                   "Error could not parse GET_TEMPERATURE_HUMIDITY");

    return PIL_NO_ERROR;
}


/*static*/ int ClimateChamberControl::monitorThreadFunction(void *ptr)
{
    assert(ptr == nullptr);
    auto *climateChamberWrapper = reinterpret_cast<ClimateChamberControl *>(ptr);
    if (threadRunning)
        return climateChamberWrapper->m_Util->logMessageAndReturn(PIL_THREAD_NOT_FOUND, WARNING_LVL, __FILENAME__, __LINE__, __FILENAME__,
                                   "Monitor thread still running -> exit thread"); // TODO change return type


    climateChamberWrapper->getLogger().LogMessage(DEBUG_LVL, __FILENAME__, __LINE__, "startMonitorThread");

    threadRunning = true;
    while (threadRunning)
    {
        float previousTemperature = climateChamberWrapper->m_CurrentTemperature;
        float previousHumidity = climateChamberWrapper->m_CurrentHumidity;

        auto errCode = climateChamberWrapper->retrieveClimateChamberStatus();
        if (errCode != PIL_NO_ERROR)
            climateChamberWrapper->getLogger().LogMessage(WARNING_LVL, __FILENAME__, __LINE__, "Error while calling retrieveClimateChamberStatus");

        float currentTemperature = climateChamberWrapper->m_CurrentTemperature;
        float currentHumidity = climateChamberWrapper->m_CurrentHumidity;

        // Call user defined callback function if the temperature or the humidity parameter changes.
        if (climateChamberWrapper->m_tempHumCallback &&
            (previousTemperature != currentTemperature || previousHumidity != currentHumidity))
        {
            climateChamberWrapper->m_tempHumCallback(currentTemperature, currentHumidity);
        }

        // Sleep m_MonitoringThreadInterval milliseconds
        usleep(static_cast<unsigned int>(climateChamberWrapper->m_MonitoringThreadInterval * 1e3));
    }
    return 0;
}





PIL_ERROR_CODE ClimateChamberControl::commandParser(const uint8_t *buffer, uint32_t bufferLen,
                                          ClimateChamberCommand commandToParse,
                                          std::map<ClimateChamberControl::CommandReturnValues, std::string> *parsedCommand)
{
    if (!buffer)
        return m_Util->logMessageAndReturn(PIL_INVALID_ARGUMENTS, ERROR_LVL, __FUNCTION__, __LINE__, "Buffer == nullptr");

    if (bufferLen == 0)
        return m_Util->logMessageAndReturn(PIL_INVALID_ARGUMENTS, ERROR_LVL, __FUNCTION__, __LINE__, "Invalid buffer length == 0");

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

            if (ctr < 4) // TODO avoid specific numbers within the code
                return m_Util->logMessageAndReturn(PIL_INVALID_ARGUMENTS, WARNING_LVL, __FILENAME__, __LINE__,
                                           "Command parser failed %d elements returned (required 4) ", ctr);

            return PIL_NO_ERROR;
        }
        case GET_ERROR:
            parsedCommand->insert(
                    std::pair<CommandReturnValues, std::string>(COMMAND_ERR_CODE, std::string((const char *) buffer)));
            return PIL_NO_ERROR;
        case ACKNOWLEDGE_ERRORS:
            parsedCommand->insert(
                    std::pair<CommandReturnValues, std::string>(COMMAND_ERROR_ACK, std::string((const char *) buffer)));
            return PIL_NO_ERROR;
        case SET_TEMPERATURE_HUMIDITY:
            parsedCommand->insert(std::pair<CommandReturnValues, std::string>(ACK_TEMPERATURE_HUMIDITY,
                                                                              std::string((const char *) buffer)));
            return PIL_NO_ERROR;
        case START_PROGRAM:
            parsedCommand->insert(std::pair<CommandReturnValues, std::string>(COMMAND_START_PROGRAM_RET,
                                                                              std::string((const char *) buffer)));
            return PIL_NO_ERROR;
        case STOP_PROGRAM:
            parsedCommand->insert(std::pair<CommandReturnValues, std::string>(COMMAND_STOP_PROGRAM_RET,
                                                                              std::string((const char *) buffer)));
            return PIL_NO_ERROR;
        default:
            m_Logger.LogMessage(WARNING_LVL, __FILENAME__, __LINE__, "commandParser: Command not found -> return false");
            return PIL_INVALID_ARGUMENTS;
    }
}

PIL_ERROR_CODE ClimateChamberControl::startStopExecution(int command)
{
    if (!m_Initialized)
        return m_Util->logMessageAndReturn(PIL_INTERFACE_CLOSED, WARNING_LVL, __FILENAME__, __LINE__, "Climate Chamber not initialized yet");

    uint8_t commandBuffer[128];
    uint32_t commandBufferSize = 128;
    auto errCode = m_CommandCreator->createCommand(commandBuffer, &commandBufferSize, SET_TEMPERATURE_HUMIDITY, 1, 3, m_TargetTemperature,
                                  m_TargetHumidity, command);
    if (errCode != PIL_NO_ERROR)
    {
        m_Logger.LogMessage(ERROR_LVL, __FILENAME__, __LINE__, "Error commandCreator returns false");
        return errCode;
    }

    m_Logger.LogMessage(DEBUG_LVL, __FILENAME__, __LINE__, "Send command %s", commandBuffer);
    m_socket->Send(commandBuffer, reinterpret_cast<int*>(&commandBufferSize));
    if (m_socket->GetLastError() != PIL_NO_ERROR)
        return m_Util->logMessageAndReturn(m_socket->GetLastError(), ERROR_LVL, __FILENAME__, __LINE__, "Error could not send message");

    uint8_t receiveBuffer[512];
    memset(receiveBuffer, 0x00, 512);
    uint16_t bufferLen = 512;
    m_socket->Receive(receiveBuffer, reinterpret_cast<uint32_t *>(&bufferLen));
    std::map<ClimateChamberControl::CommandReturnValues, std::string> parsedCommandMap;
    errCode = commandParser(receiveBuffer, bufferLen, SET_TEMPERATURE_HUMIDITY, &parsedCommandMap);
    if (errCode != PIL_NO_ERROR)
        return m_Util->logMessageAndReturn(errCode, ERROR_LVL, __FILENAME__, __LINE__,
                                   "Error could not parse SET_TEMPERATURE_HUMIDITY");

    m_Logger.LogMessage(DEBUG_LVL, __FILENAME__, __LINE__, "Climate Chamber is started with target temperature: %d, "
                                                       "target humidity: %d (Command: %s)", m_TargetTemperature,
                    m_TargetHumidity, commandBuffer);
    m_Running = true;
    return PIL_NO_ERROR;
}



PIL_ERROR_CODE ClimateChamberControl::establishTCPConnection(const std::string &ip, uint16_t port) {
    m_Logger.LogMessage(DEBUG_LVL, __FILENAME__, __LINE__, "Connect to climate chamber at %s:%d", ip.c_str(), port);
    m_socket->Connect(const_cast<std::string &>(ip), port); // TODO change underlying lib accepting const std::strings
    if (m_socket->GetLastError() != PIL_NO_ERROR)
        return m_Util->logMessageAndReturn(m_socket->GetLastError(), ERROR_LVL, __FILENAME__, __LINE__, "Error could not bind helperFiles (%s)",
                                   PIL_ErrorCodeToString(m_socket->GetLastError()));
    return PIL_NO_ERROR;
}
