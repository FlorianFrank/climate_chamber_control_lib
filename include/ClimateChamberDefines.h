/**
 * @author Florian Frank
 * @copyright University of Passau - Chair of computer engineering
 */
#pragma once

#define DEFAULT_PORT    2049
#define DEFAULT_TIMEOUT 5000
#define DEFAULT_CHANNEL 1
#define MAX_CHANNEL     31

#define MAX_HUMIDITY    0
#define MIN_HUMIDITY    100

#define MAX_TEMPERATURE 200
#define MIN_TEMPERATURE -70

#define ZERO_KELVIN     -273
#define INVALID_HUMIDITY -1

#define MIN_PROGRAM_ID  0
#define MAX_PROGRAM_ID  9999


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
