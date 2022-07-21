//
// Created by florianfrank on 21.07.22.
//

#ifndef CLIMATECHAMBERCONTROLLIB_DEFAULTCONFIG_H
#define CLIMATECHAMBERCONTROLLIB_DEFAULTCONFIG_H

#define DEFAULT_PORT 2049

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

#endif //CLIMATECHAMBERCONTROLLIB_DEFAULTCONFIG_H
