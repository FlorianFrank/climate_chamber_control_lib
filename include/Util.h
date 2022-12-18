//
// Created by Florian Frank on 18.12.22.
//

#ifndef CLIMATE_CHAMBER_LIB_UTIL_H
#define CLIMATE_CHAMBER_LIB_UTIL_H


#include "ctlib/ErrorCodeDefines.h"
#include "ctlib/Logging.hpp"

class Util {
public:
    Util();
    PIL_ERROR_CODE logMessageAndReturn(PIL_ERROR_CODE returnValue, Level level, const char* fileName,
                                              unsigned int lineNumber, const char* message, ...);
    PIL::Logging& getLogger() { return m_Logger; };
private:
    PIL::Logging m_Logger;
};


#endif //CLIMATE_CHAMBER_LIB_UTIL_H
