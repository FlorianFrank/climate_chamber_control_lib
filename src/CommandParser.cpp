//
// Created by Florian Frank on 18.12.22.
//

#include "CommandParser.h"
#include "Util.h"


CommandParser::CommandParser(Util *util): m_Util(util) {
}

PIL_ERROR_CODE
CommandParser::parse(const uint8_t *buffer, uint32_t bufferLen, ClimateChamberCommand commandToParse,
                     std::map<CommandReturnValues, std::string> *parsedCommand){
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
            m_Util->getLogger().LogMessage(WARNING_LVL, __FILENAME__, __LINE__, "parse: Command not found -> return false");
            return PIL_INVALID_ARGUMENTS;
    }
}