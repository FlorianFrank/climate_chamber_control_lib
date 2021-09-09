/**
 * @file This file contains all the functionality used for logging.
 * @addtogroup ASOACryptMiddleware
 */
#pragma once
#define LOGGING_ENABLED 1

#include <string.h> // strrchr

/** Max size of an message to log. **/
#define LOG_BUF_SIZE 1024

/** Remove path from file name. e.g. /home/user/SampleFile.c -> SampleFile.c. */
#ifdef __WIN32__
#define __FILENAME__ (strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : __FILE__)
#else // Linux
#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)
#endif // __WIN32__

/** Enum storing the loglevel, to indicate the type of message. */
typedef enum{ DEBUG_LVL = 0, ERROR_LVL, WARNING_LVL, NONE_LVL}  Level;
#ifdef LOGGING_ENABLED
void PIL_InitializeLogging(Level level, const char *file);
                void PIL_CloseLogfile();
                void PIL_LogMessage(Level level, const char *fileName, unsigned int lineNumber,
                const char *message, ...);
#else // Logging Disabled
/** Dummy defines when logging is disabled do nothing. */
#define InitializeLogging(level, file)
#define LogMessage(level, fileName, lineNumber, message, ...)
#define CloseLogfile()
#define convertHostID(x)
#endif // LOGGING_ENABLED
/*
 * };
 * */
