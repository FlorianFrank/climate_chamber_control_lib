/**
 * @author Florian Frank
 * @copyright University of Passau - Chair of Computer Engineering
 */

#pragma once

#include "ClimateChamberDefines.h"
#include "../common_tools_lib/Logging/include/ctlib/Logging.h"
#include "../common_tools_lib/Logging/include/ctlib/Logging.hpp"
#include "../common_tools_lib/ErrorHandling/include/ctlib/ErrorCodeDefines.h"
#include "CommandParser.h"


#include <thread> // std::thread
#include <mutex> // std::mutex
#include <map> // std::map

/** Forward declaration avoid include of socket file from PIL library*/
namespace PIL {
    class Socket;
}

class Util;
class CommandCreator;

/**
 * @class This class is a wrapper to control the Weiss Technik LabEvent climate simulation chamber.
 *        The functions are implemented in a python script which is called by the methods of this class.
 */
class ClimateChamberControl
{
public:
    /** Default constructor. */
    ClimateChamberControl();

    /** Destructor stops the climate chamber execution and joins the monitor thread. */
    ~ClimateChamberControl();

    /**
     * @brief initialize the connection to the climate chamber. Opens the TCP/IP socket to the climate chamber.
     * @param ipAddr ipAddress to reach the climate chamber.
     * @param port port on which the climate chamber listens for ASCII-2 commands.
     * @param channel TODO requires further explanation and checks
     * @return if successful return true, else false is returned.
     */
    PIL_ERROR_CODE initialize(const std::string &ipAddr, uint16_t port = DEFAULT_PORT, uint8_t channel = DEFAULT_CHANNEL); // TODO what does this channel mean??

    /**
     * @brief Closes the socket to the climate chamber. Joins the thread, which continuously
     *        fetches humidity and temperature data from the climate chamber.
     * @return if successful return true, else false is returned.
     */
    PIL_ERROR_CODE deInitialize();

    /**
     * @brief Retrieves the current temperature, the target temperature, the current humidity, such as the target humidity from the climate chamber.
     *          The values are stored as member variables in this class and can be accessed by calling getCurrentHumidity, getTargetTemperature, etc.
     * @return if successful return true, else false is returned.
     */
    PIL_ERROR_CODE retrieveClimateChamberStatus();

    /**
     * @brief Returns the last received current temperature value of the climate chamber.
     * @return current temperature of the climate chamber.
     */
    float getCurrentTemperature();

    /**
     * @brief Returns the last received current humidity value of the climate chamber.
     * @return current humidity of the climate chamber.
     */
    float getCurrentHumidity();

    /**
     * @brief Returns the last received target temperature value of the climate chamber.
     * @return target temperature of the climate chamber.
     */
    float getTargetTemperature();

    /**
     * @brief Returns the last received target humidity value of the climate chamber.
     * @return target humidity of the climate chamber.
     */
    float getTargetHumidity();

    /**
     * @brief Sets the target temperature of the climate chamber. The climate chamber is adjusted to this value when startExecution is called.
     * @param targetTemperature temperature to set.
     * @return true if temperature is in the acceptance range, else return 0.
     */
    PIL_ERROR_CODE setTargetTemperature(float targetTemperature);

    /**
     * @brief Sets the target humidity of the climate chamber. The climate chamber is adjusted to this value when startExecution is called.
     * @param targetTemperature temperature to set.
     * @return true if temperature is in the acceptance range, else return 0.
     */
    PIL_ERROR_CODE setTargetHumidity(float targetHumidity);

    /**
     * @brief Starts the execution of the climate chamber, which the values set by setTargetTemperature and setTargetHumidity.
     *        Caution: This function works only if the "external mode" was enabled on the climate chamber.
     * @return true if execution could be started successfully, else return false. The error code can be retrieved by calling getErrorCode().
     */
    PIL_ERROR_CODE startExecution();

    /**
     * @brief Stops the execution of the climate chamber.
     * @return true if command could be executed successfully.
     */
    PIL_ERROR_CODE stopExecution();

    /**
     * @brief Starts a predefined program (stored on the climate chamber), identified by an ID.
     * @return true if execution was successful, else return false.
     */
    PIL_ERROR_CODE startProgram(int StartProgram);

    /**
     * @brief Stops the execution of the program.
     * @return true if command could be processed successfully, else return false.
     */
    PIL_ERROR_CODE stopProgram();


    /**
     * @brief Acknowledge all errors on the climate chamber.
     * @return true if command could be processed, else return false.
     */
    PIL_ERROR_CODE acknowledgeErrors();

    /**
     * @brief Starts a thread, which continuously calls retrieveClimateChamberStatus() to retrieve temperature and humidity information from the climate chamber.
     *        This function must be executed prior registering a callback function to retrieve humidity and temperature values.
     * @param intervalMs interval in milliseconds after which the humidity and temperature is refreshed.
     * @return true if thread could be started successfully, else return false.
     */
    PIL_ERROR_CODE startMonitorThread(int intervalMs = 5000);

    /**
     * @brief Stops the monitor thread which continuously retrieves humidity and temperature data from the climate chamber.
     * @return true if the thread could be stopped successfully.
     */
    PIL_ERROR_CODE stopMonitorThread();

    /**
     * @brief Register callback function to retrieve changes in humidity and temperature values.
     * @param callbackFunc function which is called if the humidity or temperature changes.
     * @return true if registration was successful, else return false.
     */
    PIL_ERROR_CODE registerHumidityTemperatureCallback(void (*callbackFunc)(float humidity, float temperature));

    /**
     * @brief Function returns if the climate chamber is currently running or not.
     * @return true if temperature chamber is active.
     */
    [[nodiscard]] bool isRunning() const { return m_Running; }

    PIL::Logging& getLogger() { return m_Logger; }

private:
    /** Flag indicates if the climate chamber is initialized. **/
    bool m_Initialized{};
    /** Flag indicates if the climate chamber is running.  **/
    bool m_Running{};

    /** Channel of the climate chamber. Allowed are values from 0 - 32 (default: 1)*/
    uint8_t m_channel = 1;

    int m_MonitoringThreadInterval = DEFAULT_TIMEOUT;

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

    PIL::Logging m_Logger;

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
     * @brief
     * @param parsedCommand
     * @return
     */
    PIL_ERROR_CODE
    sendCommandGetResponse(std::map<CommandParser::CommandReturnValues, std::string> *parsedCommand, ClimateChamberCommand command, int nrArgs, ...);

    /**
     * @brief
     * @param ptr
     * @return
     */
    static int monitorThreadFunction(void *ptr);


    PIL_ERROR_CODE startStopExecution(int command);
    PIL_ERROR_CODE establishTCPConnection(const std::string &ip, uint16_t port);

    Util *m_Util;
    CommandCreator *m_CommandCreator;
    CommandParser *m_CommandParser;
};
