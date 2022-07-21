/**
 * @author Florian Frank
 */

#pragma once

#include <thread> // std::thread
#include <mutex> // std::mutex
#include <map> // std::map
#include "ctlib/Logging.hpp"

/** Forward declaration avoid include of socket file*/
namespace PIL {
    class Socket;
}

/**
 * @class This class is a wrapper to control the Weiss Technik LabEvent climate simulation chamber.
 *        The functions are implemented in a python script which is called by the methods of this class.
 */
class ClimateChamberWrapper
{
public:
    /** Default constructor. */
    ClimateChamberWrapper() = default;

    /** Destructor stops the climate chamber execution and joins the monitor thread. */
    ~ClimateChamberWrapper();

    /**
     * @brief Initialize the connection to the climate chamber. Opens the TCP/IP socket to the climate chamber.
     * @param ipAddr ipAddress to reach the climate chamber.
     * @param port port on which the climate chamber listens for ASCII-2 commands.
     * @return if successful return true, else false is returned.
     */
    bool Initialize(const char *ipAddr = "192.168.139.112", uint16_t port = 2049, uint8_t channel = 1);

    /**
     * @brief Closes the socket to the climate chamber. Joins the thread, which continuously
     *        fetches humidity and temperature data from the climate chamber.
     * @return if successful return true, else false is returned.
     */
    bool DeInitialize();

    /**
     * @brief Retrieves the current temperature, the target temperature, the current humidity, such as the target humidity from the climate chamber.
     *          The values are stored as member variables in this class and can be accessed by calling GetCurrentHumidity, GetTargetTemperature, etc.
     * @return if successful return true, else false is returned.
     */
    bool RetrieveClimateChamberStatus();

    /**
     * @brief Returns the last received current temperature value of the climate chamber.
     * @return current temperature of the climate chamber.
     */
    float GetCurrentTemperature();

    /**
     * @brief Returns the last received current humidity value of the climate chamber.
     * @return current humidity of the climate chamber.
     */
    float GetCurrentHumidity();

    /**
     * @brief Returns the last received target temperature value of the climate chamber.
     * @return target temperature of the climate chamber.
     */
    float GetTargetTemperature();

    /**
     * @brief Returns the last received target humidity value of the climate chamber.
     * @return target humidity of the climate chamber.
     */
    float GetTargetHumidity();

    /**
     * @brief Sets the target temperature of the climate chamber. The climate chamber is adjusted to this value when StartExecution is called.
     * @param targetTemperature temperature to set.
     * @return true if temperature is in the acceptance range, else return 0.
     */
    bool SetTargetTemperature(float targetTemperature);

    /**
     * @brief Sets the target humidity of the climate chamber. The climate chamber is adjusted to this value when StartExecution is called.
     * @param targetTemperature temperature to set.
     * @return true if temperature is in the acceptance range, else return 0.
     */
    bool SetTargetHumidity(float targetHumidity);

    /**
     * @brief Starts the execution of the climate chamber, which the values set by SetTargetTemperature and SetTargetHumidity.
     *        Caution: This function works only if the "external mode" was enabled on the climate chamber.
     * @return true if execution could be started successfully, else return false. The error code can be retrieved by calling GetErrorCode().
     */
    bool StartExecution();

    /**
     * @brief Stops the execution of the climate chamber.
     * @return true if command could be executed successfully.
     */
    bool StopExecution();

    /**
     * @brief Starts a predefined program (stored on the climate chamber), identified by an ID.
     * @return true if execution was successful, else return false.
     */
    bool StartProgram(int programID);

    /**
     * @brief Stops the execution of the program.
     * @return true if command could be processed successfully, else return false.
     */
    bool StopProgram();

    /**
     * @brief Retrieves the last error from the climate chamber.
     * @param errCodeRet variable stores the retrieved error code.
     * @return true if command could be processed, else return false.
     */
    bool GetErrorCode(int *errCodeRet);

    /**
     * @brief Acknowledge all errors on the climate chamber.
     * @return true if command could be processed, else return false.
     */
    bool AcknowledgeErrors();

    /**
     * @brief Starts a thread, which continuously calls RetrieveClimateChamberStatus() to retrieve temperature and humidity information from the climate chamber.
     *        This function must be executed prior registering a callback function to retrieve humidity and temperature values.
     * @param intervalMs interval in milliseconds after which the humidity and temperature is refreshed.
     * @return true if thread could be started successfully, else return false.
     */
    bool StartMonitorThread(int intervalMs = 5000);

    /**
     * @brief Stops the monitor thread which continuously retrieves humidity and temperature data from the climate chamber.
     * @return true if the thread could be stopped successfully.
     */
    bool StopMonitorThread();

    /**
     * @brief Register callback function to retrieve changes in humidity and temperature values.
     * @param callbackFunc function which is called if the humidity or temperature changes.
     * @return true if registration was successful, else return false.
     */
    void RegisterHumidityTemperatureCallback(void (*callbackFunc)(float humidity, float temperature));

    /**
     * @brief Function returns if the climate chamber is currently running or not.
     * @return true if temperature chamber is active.
     */
    bool IsRunning() const { return m_Running; }

    /**
     * @brief Enum to identify the possible commands which can be sent to the climate chamber.
     */
    enum ClimateChamberCommand
    {
        /** Retrieve current and target temperature and humidity values. */
        GET_TEMPERATURE_HUMIDITY,
        /** Set target temperature and humidity values and starts the execution of the climate chamber. */
        SET_TEMPERATURE_HUMIDITY,
        /** Command returns the last error from the climate chamber. */
        GET_ERROR,
        /** Command acknowledges all errors from the climate chamber. */
        ACKNOWLEDGE_ERRORS,
        /** Command starts a program, defined on the climate chamber. */
        START_PROGRAM,
        /** Commands stops the execution of a program stored on the climate chamber. */
        STOP_PROGRAM
    };

private:
    /** Flag indicates if the climate chamber is initialized. **/
    bool m_Initialized{};
    /** Flag indicates if the climate chamber is running.  **/
    bool m_Running{};

    /** Channel of the climate chamber. Allowed are values from 0 - 32 (default: 1)*/
    uint8_t m_channel = 1;

    int m_MonitoringThreadInterval = 5000;

    /** Target temperature which is approximated after starting the chamber.
     *  A test can be started when the target temperature == the current temperature. */
    float m_TargetTemperature{};
    /** Target humidity which is approximated after starting the chamber.
     *  A test can be started when the target humidity == the current humidity. */
    float m_TargetHumidity{};

    /** Current temperature within the climate chamber. */
    float m_CurrentTemperature{};
    /** Current humidity within the climate chamber. */
    float m_CurrentHumidity{};

    /** Socket to connect with the climate chamber. */
    PIL::Socket *m_socket{};

    /** Thread object used for the asynchronous transfer of humidity and temperature values from the climate chamber. */
    std::thread *m_MonitorThread{};
    /**
     * Monitor object to synchronize the access of the current temperature value
     * from the continuous thread and the GetTemperature and GetHumidity methods.
     */
    std::mutex m_TemperatureLock;
    /**
     * Monitor object to synchronize the access of the current temperature value
     * from the continuous thread and the GetTemperature and GetHumidity methods.
    */
    std::mutex m_HumidityLock;

    static PIL::Logging m_Logger;

    /**
     * @brief Callback function is called from the monitor thread if either the humidity or the temperature changes.
     *        Caution: Do not define blocking functions or functions with a high amount of computing time.
     *        This would delay the execution of the monitor thread.
     * @param humidity
     * @param temperature
     * @return
     */
    void (*m_tempHumCallback)(float humidity, float temperature) = nullptr;

    /**
     * @brief Enum contains all possible response values, which can be returned from the climate chamber.
     */
    enum CommandReturnValues
    {
        /** Target temperature of the next test execution. */
        TARGET_TEMPERATURE,
        /** Current temperature within the climate chamber. */
        CURRENT_TEMPERATURE,
        /** Target humidity of the next test execution. */
        TARGET_HUMIDITY,
        /** Current humidity within the climate chamber.
         *  When climate chamber is inactive only 0 is returned. */
        CURRENT_HUMIDTY,
        /** Return value of the GET_TEMPERATURE_HUMIDITY command. */
        ACK_TEMPERATURE_HUMIDITY,
        /** Error code returned by the GET_ERROR command. */
        COMMAND_ERR_CODE,
        /** Return value of the ACKNOWLEDGE_ERRORS command. */
        COMMAND_ERROR_ACK,
        /** Return value of the START_PROGRAM command. */
        COMMAND_START_PROGRAM_RET,
        COMMAND_STOP_PROGRAM_RET

    };

    /**
     * @brief
     * @param parsedCommand
     * @return
     */
    bool
    SendCommandGetResponse(std::map<CommandReturnValues, std::string> *parsedCommand, ClimateChamberCommand command, int nrArgs, ...);

    /**
     * @brief
     * @param ptr
     * @return
     */
    static int MonitorThreadFunction(void *ptr);

    static bool
    CommandCreator(uint8_t *buffer, uint32_t *bufferLen, ClimateChamberCommand climateChamberCommand, uint16_t channel,
                   int numberArguments, ...);

    static bool CommandParser(const uint8_t *buffer, uint32_t bufferLen, ClimateChamberCommand commandToParse,
                              std::map<CommandReturnValues, std::string> *parsedCommand);


    bool StartStopExecution(int command);

};
