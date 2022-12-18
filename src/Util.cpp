//
// Created by Florian Frank on 18.12.22.
//

#include <cstdio>
#include "Util.h"




PIL_ERROR_CODE
Util::logMessageAndReturn(PIL_ERROR_CODE returnValue, Level level, const char *fileName, unsigned int lineNumber,
                          const char *message, ...) {
    va_list vaList;
    va_start(vaList, message);
    char buffer[1024]; // TODO ugly
    vsprintf(buffer, message, vaList);
    m_Logger.LogMessage(level, fileName, lineNumber, buffer);
    va_end(vaList);
    return returnValue;
}

Util::Util(): m_Logger(DEBUG_LVL, nullptr) {

}
