//
// Created by Florian Frank on 18.12.22.
//

#ifndef CLIMATE_CHAMBER_LIB_COMMANDPARSER_H
#define CLIMATE_CHAMBER_LIB_COMMANDPARSER_H

#include "ClimateChamberDefines.h"
#include "ctlib/ErrorCodeDefines.h"

#include <map>

class Util;

class CommandParser {
public:
    /**
* @brief Enum contains all possible response values, which can be returned from the climate chamber.
*/
    enum CommandReturnValues {
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


    CommandParser(Util *util);

    PIL_ERROR_CODE parse(const uint8_t *buffer, uint32_t bufferLen,
                         ClimateChamberCommand commandToParse,
                         std::map<CommandReturnValues, std::string> *parsedCommand);

private:
    Util *m_Util;

};


#endif //CLIMATE_CHAMBER_LIB_COMMANDPARSER_H
