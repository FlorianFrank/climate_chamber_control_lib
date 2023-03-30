/*
 * @author Florian Frank
 */

#include "ClimateChamberControl.h"
#define PIL_CXX 1 // TODO
#include "ctlib/Socket.hpp"
#include "ctlib/Logging.hpp"
#include "ctlib/SocketDefines.h"


#include <cstdio>

PIL::Logging ClimateChamberControl::m_Logger(PIL::DEBUG, nullptr);

extern "C" {
#include "ctlib/ErrorHandler.h"
ClimateChamberControl ClimateChamberWrapper_py;

bool Initialize(const char *ipAddr, uint16_t port, uint8_t channel) {
    return ClimateChamberWrapper_py.initialize(ipAddr, port, channel);
}
bool DeInitialize() {
    return ClimateChamberWrapper_py.deInitialize();
}
bool RetrieveClimateChamberStatus() {
    return ClimateChamberWrapper_py.retrieveClimateChamberStatus();
}
float GetCurrentHumidity() {
    return ClimateChamberWrapper_py.getCurrentHumidity();
}
float GetCurrentTemperature() {
    return ClimateChamberWrapper_py.getCurrentTemperature();
}
float GetTargetHumidity() {
    return ClimateChamberWrapper_py.getTargetHumidity();
}
void SetTargetHumidity(float targetHumidity) {
    ClimateChamberWrapper_py.setTargetHumidity(targetHumidity);
}
float GetTargetTemperature() {
    return ClimateChamberWrapper_py.getTargetTemperature();
}
void SetTargetTemperature(float targetTemperature) {
    ClimateChamberWrapper_py.setTargetTemperature(targetTemperature);
}
bool StartExecution() {
    return ClimateChamberWrapper_py.startExecution();
}
bool StopExecution() {
    return ClimateChamberWrapper_py.stopExecution();
}
bool StartProgram(int programID) {
    return ClimateChamberWrapper_py.startProgram(programID);
}
bool StopProgram() {
    return ClimateChamberWrapper_py.stopProgram();
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


ClimateChamberControl::ClimateChamberControl()
{
}

ClimateChamberControl::~ClimateChamberControl()
{
    deInitialize();
    delete m_socket;
}


PIL_ERROR_CODE ClimateChamberControl::initialize(const std::string &ipAddr, uint16_t port, uint8_t channel)
{
    m_channel = channel;

    m_socket = new PIL::Socket(TCP, IPv4, "localhost", 8080 /*TODO what is 8080??*/, DEFAULT_TIMEOUT);

    // TODO maybe split function
    m_Logger.LogMessage(PIL::DEBUG, __FILENAME__, __LINE__, "Connect to climate chamber at %s:%d", ipAddr.c_str(), port);
    m_socket->Connect(const_cast<std::string &>(ipAddr), port); // TODO change underlying lib accepting const std::strings

    m_Logger.LogMessage(PIL::DEBUG, __FILENAME__, __LINE__, "Climate chamber initialized");
    m_Initialized = true;
    return PIL_NO_ERROR;
}


PIL_ERROR_CODE ClimateChamberControl::deInitialize()
{
    if (!m_Initialized)
        m_Logger.LogMessage(PIL::DEBUG, __FILENAME__, __LINE__, "Nothing todo Climate Chamber not initialized yet");

    // Exit loop in monitor thread, to allow joining the thread.
    auto errCode = stopMonitorThread();
    if (errCode != PIL_NO_ERROR)
        return logMessageAndReturn(errCode, PIL::ERROR, __FILENAME__, __LINE__, "Error could not stop monitor thread");

    m_Logger.LogMessage(PIL::DEBUG, __FILENAME__, __LINE__, "De-initialization completed");
    return PIL_NO_ERROR;
}


PIL_ERROR_CODE ClimateChamberControl::retrieveClimateChamberStatus()
{
    m_Logger.LogMessage(PIL::DEBUG, __FILENAME__, __LINE__, "Call retrieveClimateChamberStatus");

    std::map<CommandReturnValues, std::string> parsedCommand;
    auto errCode = sendCommandGetResponse(&parsedCommand, GET_TEMPERATURE_HUMIDITY, 0);
    if (errCode != PIL_NO_ERROR)
        return errCode;

    // TODO atof and atoi error detection
    m_TemperatureLock.lock();
    m_CurrentTemperature = atof(parsedCommand.find(CURRENT_TEMPERATURE)->second.c_str());
   // m_TargetTemperature = atof(parsedCommand.find(TARGET_TEMPERATURE)->second.c_str());
    m_TemperatureLock.unlock();

    m_HumidityLock.lock();
    m_CurrentHumidity = atof(parsedCommand.find(CURRENT_HUMIDTY)->second.c_str());
   // m_TargetHumidity = atof(parsedCommand.find(TARGET_HUMIDITY)->second.c_str());
    m_HumidityLock.unlock();

    m_Logger.LogMessage(PIL::DEBUG, __FILENAME__, __LINE__,
                   "Retrieve Climate Chamber values: (Current temperature: %f, Target temperature: %f, Current humidity: %f, Target humidity: %f",
                   m_CurrentTemperature, m_TargetTemperature, m_CurrentHumidity, m_TargetHumidity);
    return PIL_NO_ERROR;
}


float ClimateChamberControl::getCurrentTemperature()
{
    if (!m_Initialized)
    {
        m_Logger.LogMessage(PIL::WARNING, __FILENAME__, __LINE__, "Climate Chamber not initialized yet");
        return ZERO_KELVIN; // Return 0 Kelvin indicating that the chamber is not active
    }

    m_Logger.LogMessage(PIL::DEBUG, __FILENAME__, __LINE__, "Get current temperature: %d", m_CurrentTemperature);

    // Synchronize concurrent access with monitor thread.
    m_TemperatureLock.lock(); // TODO Maybe use constructor/destructor mutex
    float currentTemperature = m_CurrentTemperature;
    m_TemperatureLock.unlock();

    return currentTemperature;
}


float ClimateChamberControl::getCurrentHumidity()
{
    if (!m_Initialized)
        return logMessageAndReturn(PIL_INTERFACE_CLOSED, PIL::WARNING, __FILENAME__, __LINE__, "Climate Chamber not initialized yet");
         // TODO zero is misleading! Adjust return type

    if (!m_Running)
        return logMessageAndReturn(PIL_INTERFACE_CLOSED, PIL::WARNING, __FILENAME__, __LINE__,
                                   "The humidity can only be retrieved when the climate chamber is running");

    m_Logger.LogMessage(PIL::DEBUG, __FILENAME__, __LINE__, "Get current humidity: %d", m_CurrentHumidity);

    // Synchronize concurrent access with monitor thread.
    m_HumidityLock.lock(); // TODO Maybe use constructor/destructor mutex
    float currentHumidity = m_CurrentHumidity;
    m_HumidityLock.unlock();

    return currentHumidity;
}


float ClimateChamberControl::getTargetTemperature()
{
    // Synchronize concurrent access with monitor thread.
    m_TemperatureLock.lock(); // TODO Maybe use constructor/destructor mutex
    float targetTemperature = m_TargetTemperature;
    m_TemperatureLock.unlock();

    return targetTemperature;
}


float ClimateChamberControl::getTargetHumidity()
{
    // Synchronize concurrent access with monitor thread.
    m_TemperatureLock.lock(); // TODO Maybe use constructor/destructor mutex
    float targetTemperature = m_TargetHumidity;
    m_TemperatureLock.unlock();

    return targetTemperature;
}


PIL_ERROR_CODE ClimateChamberControl::setTargetTemperature(const float targetTemperature)
{
    if (!m_Initialized)
        return logMessageAndReturn(PIL_INTERFACE_CLOSED, PIL::WARNING, __FILENAME__, __LINE__, "Climate Chamber not initialized yet");

    m_Logger.LogMessage(PIL::DEBUG, __FILENAME__, __LINE__, "Set target temperature %f", targetTemperature);

    m_TemperatureLock.lock(); // TODO Maybe use constructor/destructor mutex
    m_TargetTemperature = targetTemperature;
    m_TemperatureLock.unlock();

    return PIL_NO_ERROR;
}


PIL_ERROR_CODE ClimateChamberControl::setTargetHumidity(const float targetHumidity)
{
    if (!m_Initialized)
        return logMessageAndReturn(PIL_INTERFACE_CLOSED, PIL::WARNING, __FILENAME__, __LINE__, "Climate Chamber not initialized yet");

    if (targetHumidity > MAX_HUMIDITY) // TODO this actually follows a function, e.g. at lower temperatures only lower humidity values are allowed
        return logMessageAndReturn(PIL_INVALID_ARGUMENTS, PIL::WARNING, __FILENAME__, __LINE__,
                                   "Could not set humidity %d only values from [0,100] are allowed", targetHumidity);

    m_Logger.LogMessage(PIL::DEBUG, __FILENAME__, __LINE__, "Set target humidity %f", targetHumidity);

    m_HumidityLock.lock();  // TODO Maybe use constructor/destructor mutex
    m_TargetHumidity = targetHumidity;
    m_HumidityLock.unlock();

    return PIL_NO_ERROR;
}


PIL_ERROR_CODE ClimateChamberControl::startExecution()
{
    m_Logger.LogMessage(PIL::DEBUG, __FILENAME__, __LINE__,
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
    m_Logger.LogMessage(PIL::DEBUG, __FILENAME__, __LINE__, "Stop climate chamber execution");

    // TODO verify
    m_Running = false;
    return startStopExecution(0); // TODO what is command 0 don't use magic numbers
}

PIL_ERROR_CODE ClimateChamberControl::startProgram(const int programID)
{
    m_Logger.LogMessage(PIL::DEBUG, __FILENAME__, __LINE__, "Call startProgram");

    std::map<CommandReturnValues, std::string> parsedCommand;
    auto errCode = sendCommandGetResponse(&parsedCommand, START_PROGRAM, 1, programID);
    if (errCode != PIL_NO_ERROR)
        return errCode;

    int ret = atoi(parsedCommand.find(COMMAND_START_PROGRAM_RET)->second.c_str());
    if (ret != 0) // TODO what is ret 0, use explicit error code enums
        return logMessageAndReturn(PIL_INVALID_ARGUMENTS, PIL::ERROR, __FILENAME__, __LINE__,
                                   "Start program %d returns with error code %d", programID, ret);
    return PIL_NO_ERROR;
}


PIL_ERROR_CODE ClimateChamberControl::stopProgram()
{
    m_Logger.LogMessage(PIL::DEBUG, __FILENAME__, __LINE__, "Call stopProgram");

    std::map<CommandReturnValues, std::string> parsedCommand;
    auto errCode = sendCommandGetResponse(&parsedCommand, START_PROGRAM, 1, 0);
    if (errCode != PIL_NO_ERROR)
        return errCode;

    int ret = atoi(parsedCommand.find(COMMAND_START_PROGRAM_RET)->second.c_str());
    if (ret != 0) // TODO what is ret != 0
        return logMessageAndReturn(PIL_INVALID_ARGUMENTS, PIL::ERROR, __FILENAME__, __LINE__, "Stop program returns with error code %d",
                                   0, ret);
    return PIL_NO_ERROR;
}


PIL_ERROR_CODE ClimateChamberControl::acknowledgeErrors()
{
    m_Logger.LogMessage(PIL::DEBUG, __FILENAME__, __LINE__, "Call acknowledgeErrors");

    std::map<CommandReturnValues, std::string> parsedCommand;
    auto errCode = sendCommandGetResponse(&parsedCommand, ACKNOWLEDGE_ERRORS, 0);
    if (errCode != PIL_NO_ERROR)
        return errCode;

    int ret = atoi(parsedCommand.find(COMMAND_ERROR_ACK)->second.c_str());
    if (ret != 0)
        return logMessageAndReturn(PIL_INVALID_ARGUMENTS, PIL::ERROR, __FILENAME__, __LINE__, "acknowledgeErrors returns error code %d",
                                   ret);
    return PIL_NO_ERROR;
}


PIL_ERROR_CODE ClimateChamberControl::startMonitorThread(int intervalMs)
{
    if (intervalMs < 100)
        return logMessageAndReturn(PIL_INVALID_ARGUMENTS, PIL::WARNING, __FILENAME__, __LINE__,
                                   "Monitor Thread interval of %d not allowed value must be greater 100 milliseconds"); // TODO why is this the case?

    m_MonitoringThreadInterval = intervalMs;
    if (!m_Initialized)
        return logMessageAndReturn(PIL_INTERFACE_CLOSED, PIL::WARNING, __FILENAME__, __LINE__, "Climate Chamber not initialized yet");

    m_Logger.LogMessage(PIL::DEBUG, __FILENAME__, __LINE__,
                        "Start monitor thread with interval %d continuously retrieves humidity and temperature information from the climate chamber",
                        m_MonitoringThreadInterval);
    m_MonitorThread = new std::thread(monitorThreadFunction, this);
    return PIL_NO_ERROR;
}

PIL_ERROR_CODE ClimateChamberControl::stopMonitorThread()
{
    if (!threadRunning)
        return logMessageAndReturn(PIL_THREAD_NOT_FOUND, PIL::WARNING, __FILENAME__, __LINE__, "No monitoring thread running");

    threadRunning = false;
    m_Logger.LogMessage(PIL::DEBUG, __FILENAME__, __LINE__, "Join thread");
    m_MonitorThread->join();
    m_Logger.LogMessage(PIL::DEBUG, __FILENAME__, __LINE__, "Thread exited");
    delete m_MonitorThread;

    return PIL_NO_ERROR;
}

PIL_ERROR_CODE ClimateChamberControl::registerHumidityTemperatureCallback(void (*callbackFunc)(float, float))
{
    if (threadRunning && m_Running)
        m_Logger.LogMessage(PIL::WARNING, __FILENAME__, __LINE__,
                       "Thread and climate chamber are still running, maybe you missed some values");

    m_Logger.LogMessage(PIL::DEBUG, __FILENAME__, __LINE__, "Register temperature and humidity callback.");
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
        return logMessageAndReturn(PIL_INTERFACE_CLOSED, PIL::WARNING, __FILENAME__, __LINE__, "Climate Chamber not initialized yet");

    if (!parsedCommand)
        return logMessageAndReturn(PIL_INVALID_ARGUMENTS, PIL::ERROR, __FILENAME__, __LINE__,
                                   "Map required to store the return values is NULL");

    /**
     * SEND COMMAND.
     */
    uint8_t commandBuffer[SEND_COMMAND_BUFFER_SIZE];
    memset(commandBuffer, 0x00, SEND_COMMAND_BUFFER_SIZE);
    uint32_t commandBufferSize = SEND_COMMAND_BUFFER_SIZE;

    m_Logger.LogMessage(PIL::DEBUG, __FILENAME__, __LINE__, "Create command");
    va_list vaList;
    va_start(vaList, nrArgs);
    auto errCode = commandCreator(commandBuffer, &commandBufferSize, command, m_channel, nrArgs, vaList);
    if (errCode != PIL_NO_ERROR)
    {
        va_end(vaList); // TODO isn't there a way to clean it up automatically after leaving the scope of the function
        return logMessageAndReturn(errCode, PIL::ERROR, __FILENAME__, __LINE__, "Error commandCreator returns false");
    }

    va_end(vaList);

    m_Logger.LogMessage(PIL::DEBUG, __FILENAME__, __LINE__, "RetrieveClimateChamber status => send: \n[%s])", commandBuffer);
    m_socket->Send(commandBuffer, reinterpret_cast<int*>(&commandBufferSize)); // TODO cleanup

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
        if (waitRet <= PIL_SOCK_SUCCESS)
        {
            m_Logger.LogMessage(PIL::WARNING, __FUNCTION__, __LINE__, "No data avail retry %d", waitRet);
            if (ctr > 5)
            { // TODO what the hell is that?
                m_Logger.LogMessage(PIL::WARNING, __FUNCTION__, __LINE__, "No data available within 5 seconds");

                return PIL_INVALID_ARGUMENTS;
            }
        }
        ctr++;
    } while (waitRet <= 0);

    m_Logger.LogMessage(PIL::DEBUG, __FUNCTION__, __LINE__, "Data avail ->Read");

    m_socket->Receive(receiveBuffer, reinterpret_cast<uint32_t *>(&bufferLen));
    printf("%s\n", receiveBuffer);
    m_Logger.LogMessage(PIL::DEBUG, __FUNCTION__, __LINE__, "Read finished");


    if (ret == -1)
        m_Logger.LogMessage(PIL::ERROR, __FUNCTION__, __LINE__, "Error while calling read"); // Should we return false?

    errCode = commandParser(receiveBuffer, bufferLen, command, parsedCommand);
    if (errCode != PIL_NO_ERROR)
        return logMessageAndReturn(errCode, PIL::ERROR, __FUNCTION__, __LINE__,
                                   "Error could not parse GET_TEMPERATURE_HUMIDITY");

    return PIL_NO_ERROR;
}


/*static*/ int ClimateChamberControl::monitorThreadFunction(void *ptr)
{
    if (threadRunning)
        return logMessageAndReturn(PIL_THREAD_NOT_FOUND, PIL::WARNING, __FILENAME__, __LINE__, __FILENAME__,
                                   "Monitor thread still running -> exit thread"); // TODO change return type

    if (!ptr)
        return logMessageAndReturn(PIL_INVALID_ARGUMENTS, PIL::ERROR, __FILENAME__, __LINE__, __FILENAME__,
                                   "No valid ClimateChamberWrapper object passed to the monitor thread -> exit thread");

    m_Logger.LogMessage(PIL::DEBUG, __FILENAME__, __LINE__, "startMonitorThread");
    auto *climateChamberWrapper = reinterpret_cast<ClimateChamberControl *>(ptr);
    threadRunning = true;
    while (threadRunning)
    {
        float previousTemperature = climateChamberWrapper->m_CurrentTemperature;
        float previousHumidity = climateChamberWrapper->m_CurrentHumidity;

        auto errCode = climateChamberWrapper->retrieveClimateChamberStatus();
        if (errCode != PIL_NO_ERROR)
            m_Logger.LogMessage(PIL::WARNING, __FILENAME__, __LINE__, "Error while calling retrieveClimateChamberStatus");

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


/*static*/ PIL_ERROR_CODE ClimateChamberControl::commandCreator(uint8_t *buffer, uint32_t *bufferLen,
                                                      ClimateChamberCommand climateChamberCommand,
                                                      uint16_t channel, int numberArguments, ...)
{
    if (!buffer)
        return logMessageAndReturn(PIL_INVALID_ARGUMENTS, PIL::ERROR, __FUNCTION__, __LINE__, "Buffer == nullptr");

    if (*bufferLen < 5) // TODO what is 5?
        return logMessageAndReturn(PIL_INSUFFICIENT_RESOURCES, PIL::WARNING, __FUNCTION__, __LINE__,
                                   "CommandBuffer must be at least 5 byte long, but bufferLen is %d", *bufferLen);

    if (channel > MAX_CHANNEL)
        return logMessageAndReturn(PIL_INVALID_ARGUMENTS, PIL::WARNING, __FUNCTION__, __LINE__,
                                   "Invalid channelID %d, allowed values: [0,31]", channel);

    // Commands always start with $
    buffer[0] = '$'; // TODO this is ugly

    // ChannelID must bet between 0 and 32, default is 1
    if (sprintf(reinterpret_cast<char *>(&buffer[1]), "%02d", channel) < 0)
        return logMessageAndReturn(PIL_ERRNO, PIL::ERROR, __FILENAME__, __LINE__, "Error while calling sprintf");

    switch (climateChamberCommand)
    {
        case GET_TEMPERATURE_HUMIDITY:
            // ASCII command $<channelByte1><channelByte0>I\r
            buffer[3] = 'I';
            buffer[4] = '\r';
            *bufferLen = 5;
            return PIL_NO_ERROR;
        case SET_TEMPERATURE_HUMIDITY:
        {
            // ASCII command $<channelByte1><channelByte0>E <targetTemperature> <targetHumidity> <StartStop> <additional flags (not used)>\r
            // e.g. $01E 0010.0 0080.0 0100.0 0000.0 0000.0 0000.0 0000.0 010 00000001010000000000000000000\r

            if (*bufferLen < 81) // TODO what is 81 avoid magic numbers
                return logMessageAndReturn(PIL_INSUFFICIENT_RESOURCES, PIL::ERROR, __FILENAME__, __LINE__,
                                           "Error buffer size to short to send set temperature/humidity command required: %d, actual: %d",
                                           81, *bufferLen);

            if (numberArguments < 3)
                return logMessageAndReturn(PIL_INVALID_ARGUMENTS, PIL::WARNING, __FILENAME__, __LINE__,
                                           "SET_TEMPERATURE_HUMIDITY requires three arguments %d were given",
                                           numberArguments);

            va_list ap;
            va_start(ap, numberArguments);
            float temperature = va_arg(ap, double);
            float humidity = va_arg(ap, double);
            float time = va_arg(ap, int);
            va_end(ap);

            if (temperature < MIN_TEMPERATURE || temperature > MAX_TEMPERATURE)
                return logMessageAndReturn(PIL_INVALID_ARGUMENTS, PIL::WARNING, __FILENAME__, __LINE__,
                                           "Error temperature value %d not allowed. (Allowed: [0,100].");

            if (humidity < MIN_HUMIDITY || humidity > MAX_HUMIDITY)
                return logMessageAndReturn(PIL_INVALID_ARGUMENTS, PIL::WARNING, __FILENAME__, __LINE__,
                                           "Error humidity value %d not allowed. (Allowed: [0,100].");

            buffer[3] = 'E';
            buffer[4] = ' ';
            int sprintfRet = sprintf(reinterpret_cast<char *>(&buffer[5]),
                                     "%06.1f %06.1f %06.1f 0000.0 0000.0 0000.0 000.0 010 00000001010000000000000000000\r",
                                     temperature, humidity, time);
            *bufferLen = sprintfRet + 5;
            if (sprintfRet < 0)
                return logMessageAndReturn(PIL_ERRNO, PIL::ERROR, __FILENAME__, __LINE__, "Error while calling sprintf");
            return PIL_NO_ERROR;
        }
        case GET_ERROR:
            buffer[3] = 'F';
            buffer[4] = '\r';
            *bufferLen = 5;
            return PIL_NO_ERROR;
        case ACKNOWLEDGE_ERRORS:
            buffer[3] = 'Q';
            buffer[4] = '\r';
            *bufferLen = 5;
            return PIL_NO_ERROR;
        case START_PROGRAM:
        {
            if (*bufferLen < 8) // TODO avoid specific numbers within the source code.
                return logMessageAndReturn(PIL_INSUFFICIENT_RESOURCES, PIL::ERROR, __FILENAME__, __LINE__,
                                           "Error buffer size to short to send set temperature/humidity command required: %d, actual: %d",
                                           8, *bufferLen);


            if (numberArguments < 1)
                return logMessageAndReturn(PIL_INVALID_ARGUMENTS, PIL::WARNING, __FILENAME__, __LINE__,
                                           "START_PROGRAM requires one argument (program number) but only %d were given",
                                           numberArguments);

            va_list ap;
            va_start(ap, numberArguments);
            const int programID = va_arg(ap,  int);
            va_end(ap);

            if (programID > MAX_PROGRAM_ID || programID < MIN_PROGRAM_ID)
                return logMessageAndReturn(PIL_INVALID_ARGUMENTS, PIL::WARNING, __FILENAME__, __LINE__,
                                           "Program id %d not accepted (allowed: [0,9999]", programID);

            buffer[3] = 'P';
            if (sprintf(reinterpret_cast<char *>(&buffer[4]), "%04d", programID) < 0)
                m_Logger.LogMessage(PIL::WARNING, __FILENAME__, __LINE__, "Error fprintf failed");

            buffer[8] = '\r';
            *bufferLen = 8;
            return PIL_NO_ERROR;
        }
        case STOP_PROGRAM:
            if (*bufferLen < 8)
                return logMessageAndReturn(PIL_INSUFFICIENT_RESOURCES, PIL::ERROR, __FILENAME__, __LINE__,
                                           "Error buffer size to short to send set temperature/humidity command required: %d, actual: %d",
                                           8, *bufferLen);

            buffer[3] = 'P';
            buffer[4] = '0';
            buffer[5] = '0';
            buffer[6] = '0';
            buffer[7] = '0';
            buffer[8] = '\r';
            *bufferLen = 8;
            return PIL_NO_ERROR;
    }
    return PIL_INVALID_ARGUMENTS;
}


PIL_ERROR_CODE ClimateChamberControl::commandParser(const uint8_t *buffer, uint32_t bufferLen,
                                          ClimateChamberCommand commandToParse,
                                          std::map<ClimateChamberControl::CommandReturnValues, std::string> *parsedCommand)
{
    if (!buffer)
        return logMessageAndReturn(PIL_INVALID_ARGUMENTS, PIL::ERROR, __FUNCTION__, __LINE__, "Buffer == nullptr");

    if (bufferLen == 0)
        return logMessageAndReturn(PIL_INVALID_ARGUMENTS, PIL::ERROR, __FUNCTION__, __LINE__, "Invalid buffer length == 0");

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
                return logMessageAndReturn(PIL_INVALID_ARGUMENTS, PIL::WARNING, __FILENAME__, __LINE__,
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
            m_Logger.LogMessage(PIL::WARNING, __FILENAME__, __LINE__, "commandParser: Command not found -> return false");
            return PIL_INVALID_ARGUMENTS;
    }
}

PIL_ERROR_CODE ClimateChamberControl::startStopExecution(int command)
{
    if (!m_Initialized)
        return logMessageAndReturn(PIL_INTERFACE_CLOSED, PIL::WARNING, __FILENAME__, __LINE__, "Climate Chamber not initialized yet");

    uint8_t commandBuffer[128];
    uint32_t commandBufferSize = 128;
    auto errCode = commandCreator(commandBuffer, &commandBufferSize, SET_TEMPERATURE_HUMIDITY, 1, 3, m_TargetTemperature,
                                  m_TargetHumidity, command);
    if (errCode != PIL_NO_ERROR)
    {
        m_Logger.LogMessage(PIL::ERROR, __FILENAME__, __LINE__, "Error commandCreator returns false");
        return errCode;
    }

    m_Logger.LogMessage(PIL::DEBUG, __FILENAME__, __LINE__, "Send command %s", commandBuffer);
    m_socket->Send(commandBuffer, reinterpret_cast<int*>(&commandBufferSize));

    uint8_t receiveBuffer[512];
    memset(receiveBuffer, 0x00, 512);
    uint16_t bufferLen = 512;
    m_socket->Receive(receiveBuffer, reinterpret_cast<uint32_t *>(&bufferLen)); // TODO
    std::map<ClimateChamberControl::CommandReturnValues, std::string> parsedCommandMap;
    errCode = commandParser(receiveBuffer, bufferLen, SET_TEMPERATURE_HUMIDITY, &parsedCommandMap);
    if (errCode != PIL_NO_ERROR)
        return logMessageAndReturn(errCode, PIL::ERROR, __FILENAME__, __LINE__,
                                   "Error could not parse SET_TEMPERATURE_HUMIDITY");

    m_Logger.LogMessage(PIL::DEBUG, __FILENAME__, __LINE__, "Climate Chamber is started with target temperature: %d, "
                                                       "target humidity: %d (Command: %s)", m_TargetTemperature,
                    m_TargetHumidity, commandBuffer);
    m_Running = true;
    return PIL_NO_ERROR;
}

/*static*/ PIL_ERROR_CODE ClimateChamberControl::logMessageAndReturn(PIL_ERROR_CODE returnValue, PIL::Level level, const char* fileName, unsigned int lineNumber, const char* message, ...)
{
    va_list vaList;
    va_start(vaList, message);
    char buffer[1024]; // TODO ugly
    vsprintf(buffer, message, vaList);
    m_Logger.LogMessage(level, fileName, lineNumber, buffer);
    va_end(vaList);
    return returnValue;
}


