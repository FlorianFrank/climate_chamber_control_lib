//
// Created by florianfrank on 24.11.20.
//

#ifndef CLIMATECHAMBERCONTROLLIB_MAIN_CPP
#define CLIMATECHAMBERCONTROLLIB_MAIN_CPP

#include <ClimateChamberWrapper.h>
#include <unistd.h>

extern "C"{
#include <helperFiles/Logging.h>
}

int main()
{
    PIL_InitializeLogging(DEBUG_LVL, nullptr);
    ClimateChamberWrapper climateChamberWrapper;
    climateChamberWrapper.Initialize();
    climateChamberWrapper.RetrieveClimateChamberStatus();

    climateChamberWrapper.SetTargetTemperature(23);
    climateChamberWrapper.SetTargetHumidity(80);
    climateChamberWrapper.StartExecution();
    climateChamberWrapper.StopProgram();


    printf("Current humidity %f\n", climateChamberWrapper.GetCurrentHumidity());
    printf("Current temperature %f\n", climateChamberWrapper.GetCurrentTemperature());

    int errCode;
    climateChamberWrapper.GetErrorCode(&errCode);
    printf("Receive error code %d\n", errCode);

    climateChamberWrapper.AcknowledgeErrors();
    PIL_CloseLogfile();
}

#endif //CLIMATECHAMBERCONTROLLIB_MAIN_CPP
