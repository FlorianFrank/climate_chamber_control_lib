//
// Created by Florian Frank on 18.12.22.
//

#include "CommandCreator.h"

CommandCreator::CommandCreator(Util *util) : m_Util(util) {

}


PIL_ERROR_CODE CommandCreator::createCommand(uint8_t *buffer, uint32_t *bufferLen,
                                                     ClimateChamberCommand climateChamberCommand,
                                                     uint16_t channel, int numberArguments, ...)
{
    if (!buffer)
        return m_Util->logMessageAndReturn(PIL_INVALID_ARGUMENTS, ERROR_LVL, __FUNCTION__, __LINE__, "Buffer == nullptr");

    if (*bufferLen < 5) // TODO what is 5?
        return m_Util->logMessageAndReturn(PIL_INSUFFICIENT_RESOURCES, WARNING_LVL, __FUNCTION__, __LINE__,
                                   "CommandBuffer must be at least 5 byte long, but bufferLen is %d", *bufferLen);

    if (channel > MAX_CHANNEL)
        return m_Util->logMessageAndReturn(PIL_INVALID_ARGUMENTS, WARNING_LVL, __FUNCTION__, __LINE__,
                                   "Invalid channelID %d, allowed values: [0,31]", channel);

    // Commands always start with $
    buffer[0] = '$'; // TODO this is ugly

    // ChannelID must bet between 0 and 32, default is 1
    if (sprintf(reinterpret_cast<char *>(&buffer[1]), "%02d", channel) < 0)
        return m_Util->logMessageAndReturn(PIL_ERRNO, ERROR_LVL, __FILENAME__, __LINE__, "Error while calling sprintf");

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
                return m_Util->logMessageAndReturn(PIL_INSUFFICIENT_RESOURCES, ERROR_LVL, __FILENAME__, __LINE__,
                                           "Error buffer size to short to send set temperature/humidity command required: %d, actual: %d",
                                           81, *bufferLen);

            if (numberArguments < 3)
                return m_Util->logMessageAndReturn(PIL_INVALID_ARGUMENTS, WARNING_LVL, __FILENAME__, __LINE__,
                                           "SET_TEMPERATURE_HUMIDITY requires three arguments %d were given",
                                           numberArguments);

            va_list ap;
            va_start(ap, numberArguments);
            float temperature = va_arg(ap, double);
            float humidity = va_arg(ap, double);
            float time = va_arg(ap, int);
            va_end(ap);

            if (temperature < MIN_TEMPERATURE || temperature > MAX_TEMPERATURE)
                return m_Util->logMessageAndReturn(PIL_INVALID_ARGUMENTS, WARNING_LVL, __FILENAME__, __LINE__,
                                           "Error temperature value %d not allowed. (Allowed: [0,100].");

            if (humidity < MIN_HUMIDITY || humidity > MAX_HUMIDITY)
                return m_Util->logMessageAndReturn(PIL_INVALID_ARGUMENTS, WARNING_LVL, __FILENAME__, __LINE__,
                                           "Error humidity value %d not allowed. (Allowed: [0,100].");

            buffer[3] = 'E';
            buffer[4] = ' ';
            int sprintfRet = sprintf(reinterpret_cast<char *>(&buffer[5]),
                                     "%06.1f %06.1f %06.1f 0000.0 0000.0 0000.0 000.0 010 00000001010000000000000000000\r",
                                     temperature, humidity, time);
            *bufferLen = sprintfRet + 5;
            if (sprintfRet < 0)
                return m_Util->logMessageAndReturn(PIL_ERRNO, ERROR_LVL, __FILENAME__, __LINE__, "Error while calling sprintf");
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
                return m_Util->logMessageAndReturn(PIL_INSUFFICIENT_RESOURCES, ERROR_LVL, __FILENAME__, __LINE__,
                                           "Error buffer size to short to send set temperature/humidity command required: %d, actual: %d",
                                           8, *bufferLen);


            if (numberArguments < 1)
                return m_Util->logMessageAndReturn(PIL_INVALID_ARGUMENTS, WARNING_LVL, __FILENAME__, __LINE__,
                                           "START_PROGRAM requires one argument (program number) but only %d were given",
                                           numberArguments);

            va_list ap;
            va_start(ap, numberArguments);
            const int programID = va_arg(ap,  int);
            va_end(ap);

            if (programID > MAX_PROGRAM_ID || programID < MIN_PROGRAM_ID)
                return m_Util->logMessageAndReturn(PIL_INVALID_ARGUMENTS, WARNING_LVL, __FILENAME__, __LINE__,
                                           "Program id %d not accepted (allowed: [0,9999]", programID);

            buffer[3] = 'P';
            if (sprintf(reinterpret_cast<char *>(&buffer[4]), "%04d", programID) < 0)
                m_Util->getLogger().LogMessage(WARNING_LVL, __FILENAME__, __LINE__, "Error fprintf failed");

            buffer[8] = '\r';
            *bufferLen = 8;
            return PIL_NO_ERROR;
        }
        case STOP_PROGRAM:
            if (*bufferLen < 8)
                return m_Util->logMessageAndReturn(PIL_INSUFFICIENT_RESOURCES, ERROR_LVL, __FILENAME__, __LINE__,
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
