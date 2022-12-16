# ClimateChamberControlLib

This project provides a library to control a Weisstechnik LabEvent climate chamber via an API interface. 
It allows to set the humidity and temperature, receive error messages and warnings and acknowledge them. 
The current temperature and humidity can be received by an callback function.

## 1. Installation

Download the latest release on GitHub or clone the repository and compile it by your self. 

### 1.1 Manually compile the library

#### 1.2 Required packages

- CMAKE
- make (Linux)
- Ninja (Windows)

#### On Linux
```bash
git clone git@github.com:FlorianFrank/climate_chamber_control_lib.git
./compile.sh
```

#### On Windows
```bash
git clone git@github.com:FlorianFrank/climate_chamber_control_lib.git
./compile.ps
```

## 2. Run a simple test program

### 2.1 Using the C++-interface

Link the climate_chamber_control.lib to your project and include the header file climate_chamber_control.h.
    
e.g. using CMAKE
```cmake
project(SampleProject)

add_subproject(instrument_control_lib)

add_executable(SampleProject main.cpp)
target_include_directory(SampleProject PUBLIC climate_chamber_control_lib/include)
target_link_libraries(SampleProject climate_chamber_control)
```

```c++
#include <ClimateChamberControl.h>

int main() {
    ClimateChamberControl chamber;
    chamber.connect("
```

## 3 API-description

The api supports following functions: 

| Method Name                             | Description                                                                                                                              | Return value                                                      | 
|-----------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------|
| bool Initialize(ipAddr, port, channel); | Establishes a connection to the climate chamber.                                                                                         | Returns true if the connection could be established successfully. |
| bool DeInitialize()                     | Closes the connection to the climate chamber.                                                                                            | True if command was successful.                                   |
| bool RetrieveClimateChamberStatus()     | Sends a status request to the climate chamber to get the current temperature and humidity and stores them in the climate chamber object. | True if connection was successful.                                |
| float GetCurrentTemperature()           | Get the current temperature retrieved from the last RetrieveClimateChamberStatus() call.                                                 | True if connection was successful.                                |

