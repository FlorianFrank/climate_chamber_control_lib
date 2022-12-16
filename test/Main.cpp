//
// Created by florianfrank on 24.11.20.
//

#ifndef CLIMATECHAMBERCONTROLLIB_MAIN_CPP
#define CLIMATECHAMBERCONTROLLIB_MAIN_CPP

#include "ClimateChamberControl.h"
#include <unistd.h>

int main()
{
    ClimateChamberControl climateChamberWrapper;
    std::string ip = "132.321.14.161";
    climateChamberWrapper.initialize(ip);
    climateChamberWrapper.retrieveClimateChamberStatus();

    climateChamberWrapper.setTargetTemperature(23);
    climateChamberWrapper.setTargetHumidity(80);
    climateChamberWrapper.startExecution();
    climateChamberWrapper.stopProgram();


    printf("Current humidity %f\n", climateChamberWrapper.getCurrentHumidity());
    printf("Current temperature %f\n", climateChamberWrapper.getCurrentTemperature());

    int errCode;
    climateChamberWrapper.getErrorCode(&errCode);
    printf("Receive error code %d\n", errCode);

    climateChamberWrapper.acknowledgeErrors();
}

#endif //CLIMATECHAMBERCONTROLLIB_MAIN_CPP
