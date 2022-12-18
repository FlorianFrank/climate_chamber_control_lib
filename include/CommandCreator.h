//
// Created by Florian Frank on 18.12.22.
//

#ifndef CLIMATE_CHAMBER_LIB_COMMANDCREATOR_H
#define CLIMATE_CHAMBER_LIB_COMMANDCREATOR_H


#include "ctlib/ErrorCodeDefines.h"
#include "Util.h"
#include "ClimateChamberDefines.h"


class CommandCreator {
public:
    CommandCreator(Util *util);

    PIL_ERROR_CODE createCommand(uint8_t *buffer, uint32_t *bufferLen,
                                  ClimateChamberCommand climateChamberCommand,
                                  uint16_t channel, int numberArguments, ...);

private:
    Util *m_Util;
};


#endif //CLIMATE_CHAMBER_LIB_COMMANDCREATOR_H
