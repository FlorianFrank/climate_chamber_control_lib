# ClimateChamberControlLib

This project provides a library to control a Weisstechnik LabEvent climate chamber via an API interface. 
It allows to set the humidity and temperature, receive error messages and warnings and acknowledge them. 
The current temperature and humidity can be received by a callback function.

**Some parts of this documentation are currently missing bur will be added soon.**

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

By running **compile.sh** or **compile.ps** the library, a test application as well as the underlying abstraction library 
**common_tools_lib** providing basic functionality like logging or sockets is build. Finally, the library, headers, and the 
test application is installed in the **bin** folder and has the following structure: 
```
📦 project
│     
└─── 📂 bin
│   └─── 📂 lib
│   │    │  📜 climate_chamber_control_lib
│   │    │  📜 common_tools_lib
│   └─── 📂 bin
│   │     |  📜 ClimateChamberTest.exe
│   └─── 📂 include
│        |  📜 <include files>
│
│
└─── 📂 tmp
     │   📜 <temporary files which can be deleted after the build>
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

A sample program on how to use the climate chamber is provided in following code snipped:

```c++
#include <ClimateChamberControl.h>

int main() {
    ClimateChamberControl chamber;
    if(!chamber.initialize("<your_ip_address>")) // The standard port to access the climate chamber is 8080
        std::cout << "Error: " << chamber.GetLastError() << std::endl;
        
    chamber.setTargetTemperature(26.5);
    chamber.setTargetHumidity(30.0);
    
    chamber.startMonitorThread(5000);
    
    auto callback = [](float humidity, float temperature) {
        std::cout << "Humidity: " << humidity << " Temperature: " << temperature << std::endl;
    };
    
    chamber.RegisterTemperatureCallback(callback);
    
    chamber.startExecution();
    
    /*** Do some stuff here ***/

    chamber.stopExecution();
    chamber.Deinitialize();
```

### 2.1 Using the python-interface

The library can be accessed by a python interface as well. To run the library requires a python version < ??. 
The climate_chamber_control_library can be accessed by copying the library file directly next to the python script and
by importing it using following command: 

```python
from climate_chamber_control import *
```

The api itself is identical to the C++ API described in Section 3. 

```python
from climate_chamber_control import *

chamber = ClimateChamberControl()

if not chamber.initialize("<your_ip_address>"):
    chamber.GetLastError()

chamber.setTargetTemperature(26.5)
chamber.setTargetHumidity(30.0)

chamber.startMonitorThread(5000)

callback = lambda humidity, temperature: print("Humidity: " + str(humidity) + " Temperature: " + str(temperature))

chamber.RegisterTemperatureCallback(callback)
chamber.startExecution()

# Do some stuff here

chamber.stopExecution()
chamber.Deinitialize()
```

## 3 API-description

The api supports following functions: 

| Method Name                                                 | Description                                                                                                                                | Return value                                                      | 
|-------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------|
| bool initialize(ipAddr, port, channel);                     | Establishes a connection to the climate chamber.                                                                                           | Returns true if the connection could be established successfully. |
| bool deInitialize()                                         | Closes the connection to the climate chamber.                                                                                              | True if command was successful.                                   |
| bool retrieveClimateChamberStatus()                         | Sends a status request to the climate chamber to get the current temperature and humidity and stores them in the climate chamber object.   | True if command was successful.                                   |
| float getCurrentTemperature()                               | Get the temperature retrieved from the last retrieveClimateChamberStatus() call.                                                           | Last retrieved temperature.                                       |
| float getCurrentHumidity()                                  | Get the humidity retrieved from the last retrieveClimateChamberStatus() call.                                                              | Last retrieved humidity.                                          |
| float getTargetTemperature()                                | Returns the target temperature which should finally be reached during a cool down or heat up progress.                                     | Target temperature.                                               |
| float getTargetHumidity()                                   | Returns the target humidity which should finally be reached.                                                                               | Target humidity.                                                  |
| bool setTargetTemperature()                                 | Sets the target temperature to reach after starting or stopping the chamber.                                                               | True if command was successful.                                   |
| bool setTargetHumidity()                                    | Sets the target humidity to reach after starting or stopping the chamber.                                                                  | True if command was successful.                                   |
| bool startExecution()                                       | Starts the climate chamber to heat up or cool down to the temperature and humidity adjusted by setTargetHumidity and setTargetTemperature. | True if command was successful.                                   |
| bool stopExecution()                                        | Stops the execution of the climate chamber.                                                                                                | True if command was successful.                                   |
| bool startProgram(id)                                       | Start a specific program stored on the climate chamber, identified by an id.                                                               | True if command was successful.                                   |
| bool stopProgram()                                          | Stop a currently running program.                                                                                                          | True if command was successful.                                   |
| bool getErrorCode(errCode)                                  | Starts the climate chamber to heat up or cool down to the temperature and humidity adjusted by setTargetHumidity and setTargetTemperature. | True if command was successful.                                   |
| bool acknowledgeErrors()                                    | Acknowledge all errors stored in the error buffer.                                                                                         | True if command was successful.                                   |
 | bool startMonitorThread(int intervalMs)                     | Starts a thread which periodically calls retrieveClimateChamberStatus() to update the current temperature.                                 | True if command was successful.                                   |
| bool registerHumidityTemperatureCallback(callback Function) | Registers a callback function which is called everytime the temperature or humidity of the chamber changes.                                | -                                                                 |
| bool isRunning()                                            | Returns if the chamber is currently running or not.                                                                                        | True if cahmber is running, otherwise return False.               |
