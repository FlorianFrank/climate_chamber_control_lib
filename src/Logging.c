#if 1

#include "helperFiles/Logging.h"

#include <stdarg.h> // va_start, va_end
#include <malloc.h> // malloc
#include <errno.h> // errno
#include <stdio.h>
#include <sys/time.h>

#if defined(__linux__) || defined(__WIN32__)
#include "pthread.h" // mutex
/** Mutex to provide mutual exclution when log function is called by different threads. */
pthread_mutex_t logMutex;

// Color codes for different log levels
#define COLOR_DEFAULT   39 // black
#define COLOR_ERROR     31 // red
#define COLOR_WARNING   33 // yellow
#define COLOR_DEBUG     34 // blue
#endif // __linux__


/** Initialization of variables. */
Level logLevel;
FILE *logFileStream = NULL;
char *loggingBuffer;

const char *GetCurrentTime();
#ifdef __linux__
int GetColorCode(Level level);
#endif // __linux__

const char *GetLogLevelStr(Level level);


/**
 * @brief Set log level. Only messages, with that log level or higher are logged.
 * Additionally the log file is set. If the logfile is set to null, print to stdout.
 * @param level every message of that level or with a higher level is printed.
 * @param file path and filename of the logfile, if file is null, log to stdout.
 */
void PIL_InitializeLogging(const Level level, const char *file)
{
    logLevel = level;

    if (file != NULL)
    {
        // Define logfile
        logFileStream = fopen(file, "ab+");
        if(logFileStream == NULL)
        {
            printf("Error could not open logfile %s (Error %s)\n",
                    file, strerror(errno));
        }
    } else
        // Log to std::out if file is null
        logFileStream = stdout;

    loggingBuffer = (char*)malloc(LOG_BUF_SIZE * sizeof(char));
#if __linux__
    if(pthread_mutex_init(&logMutex, NULL) != 0)
        printf("Error could not initialize logMutex\n");
#endif // __linux__
}

/**
 * @brief Log to output stream. (Either to file or to stdout, depending on the initialization)
 * LogFile has format HH:MM:SS:microseconds LogLevel FileName:LineNumber Message
 * @param level Level of the message.
 * @param fileName Name of the file, from which the logging function is called. Simply set #__FILENAME__
 * @param lineNumber Line number from which the function is called. Simply use #_LINE_
 * @param message Message to send.
 * @param ... Additional parameters, which are send with the message in a printf style.
 */
void PIL_LogMessage(const Level level, const char *fileName,
                         unsigned int lineNumber, const char *message, ...)
{
if(logFileStream == NULL)
{
    printf("Logging enabled but not initialized!\n");
    return;
}
 //   Threading::AutoMutex am(&mutex);
#if defined( __linux__) || defined(__WIN32__)
    if(pthread_mutex_lock(&logMutex) != 0)
        printf("Error while calling pthread_mutex_lock\n");
#endif // __linux__
    va_list vaList;
    va_start(vaList, message);
    if (level >= logLevel)
    {
        int ret = vsprintf(&loggingBuffer[160], message, vaList);
        va_end(vaList);
        if (ret > 0)
        {
#ifdef __linux__
            fprintf(logFileStream, "%s\033[%dm %s\033[%dm %s:%d %s\n",
                    GetCurrentTime(), GetColorCode(level), GetLogLevelStr(level), COLOR_DEFAULT, fileName, lineNumber,
                    &loggingBuffer[160]);
#else // __WIN32__
            fprintf(logFileStream, "%s %s %s:%d %s\r\n",
                    GetCurrentTime(), GetLogLevelStr(level), fileName, lineNumber,
                    &loggingBuffer[160]);
#endif // __linux__
            fflush(stdout);

        }
    }
#if defined(__linux__) || defined(__WIN32__)
    if(pthread_mutex_unlock(&logMutex) != 0)
        printf("Error while calling pthread_mutex_unlock\n");
#endif // __linux__
}


/**
 * @brief This function closes the file stream if opened.
 */
void PIL_CloseLogfile()
{
    if(logFileStream != stdout && logFileStream != NULL)
        fclose(logFileStream);
    free(loggingBuffer);
}

/**
 * Return current time as string in format HH:MM:SS:microseconds.
 * @return string containing the time.
 */
const char *GetCurrentTime()
{
    struct timeval time = { 0 };
    gettimeofday(&time, NULL);

    char timeStr[80];



#ifdef __linux__
    size_t size = strftime(timeStr, 80, "%H:%M:%S", localtime(&time.tv_sec));
    if (size > 0)
        sprintf(loggingBuffer, "%s:%li", timeStr, time.tv_usec);
#else // WIN32
    struct tm today = { 0 };
    time_t ltime;
    ::time( &ltime );
    _localtime64_s( &today, &ltime );
    size_t size = strftime( timeStr, 80,
              "%H:%M:%S", &today );
    if (size > 0)
        sprintf(&loggingBuffer[80], "%s", timeStr);
#endif // __linux__

    return &loggingBuffer[80];
}

/**
 * @brief Return loglevel to string.
 * @param level Loglevel to convert to a string.
 * @return loglevel as string.
 */
const char *GetLogLevelStr(Level level)
{
    switch (level)
    {
        case (DEBUG_LVL):
            return "DEBUG";
        case (ERROR_LVL):
            return "ERROR";
        case (WARNING_LVL):
            return "WARNING";
        case (NONE_LVL): default:
            return "NONE";
    }
}

#ifdef __linux__
/**
 * @brief Return Color code depending on the Log level (Debug: blue, Error: red, Warning: yellow)
 * @param level loglevel to which the color code corresponds.
 * @return color code.
 */
int GetColorCode(Level level)
{
    switch (level)
    {
        case (DEBUG_LVL):
            return COLOR_DEBUG;
        case (ERROR_LVL):
            return COLOR_ERROR;
        case (WARNING_LVL):
            return COLOR_WARNING;
        case (NONE_LVL): default:
            return COLOR_DEFAULT;
    }
}
#endif // __linux__
#endif // LOGGING_ENABLES
