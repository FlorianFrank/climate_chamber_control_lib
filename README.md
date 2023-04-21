# climate chamber lib

This project provides a library to control a Weisstechnik LabEvent climate chamber via an API interface. 
It allows to set the humidity and temperature, receive error messages and warnings and acknowledge them. 
The current temperature and humidity can be received by a callback function.

## Continuous integration

We maintain the compatibility with different compilers and operating systems by compiling the libary 
on different systems. The current status of the continuous integration can be found here:

|       OS       |                                                          Status                                                          | 
|:--------------:|:------------------------------------------------------------------------------------------------------------------------:|
|Windows Latest  | ![Windows latest](https://github.com/FlorianFrank/instrument_control_lib/actions/workflows/windows_latest.yml/badge.svg) |
|Mac OS Latest   |  ![MAC OS latest](https://github.com/FlorianFrank/instrument_control_lib/actions/workflows/mac_os_latest.yml/badge.svg)  |
|Ubuntu Latest   |  ![Ubuntu Latest](https://github.com/FlorianFrank/instrument_control_lib/actions/workflows/ubuntu_latest.yml/badge.svg)  |

## 1. Installation

Download the latest release on GitHub or clone the repository and compile it by your self. 

### 1.1 Manually compile the library

#### 1.2 Required packages

- CMAKE
- make (Linux)
- Ninja (Windows)
- MinGW (Windows)

#### On Linux
```bash
git clone git@github.com:FlorianFrank/climate_chamber_control_lib.git
git submodule update --init --recursive
./compile.sh
```

#### On Windows
```bash
git clone git@github.com:FlorianFrank/climate_chamber_control_lib.git
git submodule update --init --recursive
./compile.ps1
```

By running **compile.sh** or **compile.ps** the library, a test application as well as the underlying abstraction library 
**common_tools_lib** providing basic functionality like logging or sockets is build. Finally, the library, headers, and the 
test application is installed in the **bin** folder and has the following structure: 
```
📦 project
│     
└─── 📂 bin
│   └─── 📂 lib
│   │    │  📜 climate_chamber_lib
│   │    │  📜 common_tools_lib
│   │    │  📜 py_climate_chamber_lib
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

Link the climate_chamber_lib to your project and include the header file climate_chamber_control.h.
    
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

The library can be accessed by a python interface as well. To run the library requires a python version > 3.10. 
The lib can be installed using pip by executing `pip install .` in the root directory of this project.
You can import it using the following command: 

```python
from py_climate_chamber_lib import *
```

The api itself is identical to the C++ API described in Section 3. 

```python
from py_climate_chamber_lib import *

chamber = ClimateChamberControl()

if not chamber.initialize("<your_ip_address>"):
    chamber.get_last_error()

chamber.set_target_temperature(26.5)
chamber.set_target_humidity(30.0)

chamber.start_monitor_thread(5000)

callback = lambda humidity, temperature: print("Humidity: " + str(humidity) + " Temperature: " + str(temperature))

chamber.register_humid_temp_callback(callback)
chamber.start_execution()

# Do some stuff here

chamber.stop_execution()
chamber.deinitialize()
```

## 3 API-description (C++-interface)

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

## 4 API-description (Python-interface)

| Method Name                                          | Description                                                                                                                                | Return value                                                      | 
|------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------|
| bool initialize(ipAddr, port, channel);              | Establishes a connection to the climate chamber.                                                                                           | Returns true if the connection could be established successfully. |
| bool deInitialize()                                  | Closes the connection to the climate chamber.                                                                                              | True if command was successful.                                   |
| bool retrieve_climate_chamber_status()               | Sends a status request to the climate chamber to get the current temperature and humidity and stores them in the climate chamber object.   | True if command was successful.                                   |
| float get_current_temperature()                      | Get the temperature retrieved from the last retrieveClimateChamberStatus() call.                                                           | Last retrieved temperature.                                       |
| float get_current_humidity()                         | Get the humidity retrieved from the last retrieveClimateChamberStatus() call.                                                              | Last retrieved humidity.                                          |
| float get_target_temperature()                       | Returns the target temperature which should finally be reached during a cool down or heat up progress.                                     | Target temperature.                                               |
| float get_target_humidity()                          | Returns the target humidity which should finally be reached.                                                                               | Target humidity.                                                  |
| bool set_target_temperature()                        | Sets the target temperature to reach after starting or stopping the chamber.                                                               | True if command was successful.                                   |
| bool set_target_humidity()                           | Sets the target humidity to reach after starting or stopping the chamber.                                                                  | True if command was successful.                                   |
| bool start_execution()                               | Starts the climate chamber to heat up or cool down to the temperature and humidity adjusted by setTargetHumidity and setTargetTemperature. | True if command was successful.                                   |
| bool stop_execution()                                | Stops the execution of the climate chamber.                                                                                                | True if command was successful.                                   |
| bool start_program(id)                               | Start a specific program stored on the climate chamber, identified by an id.                                                               | True if command was successful.                                   |
| bool stop_program()                                  | Stop a currently running program.                                                                                                          | True if command was successful.                                   |
| bool get_error_code(errCode)                         | Starts the climate chamber to heat up or cool down to the temperature and humidity adjusted by setTargetHumidity and setTargetTemperature. | True if command was successful.                                   |
| bool acknowledge_errors()                            | Acknowledge all errors stored in the error buffer.                                                                                         | True if command was successful.                                   |
| bool start_monitor_thread(int intervalMs)            | Starts a thread which periodically calls retrieveClimateChamberStatus() to update the current temperature.                                 | True if command was successful.                                   |
| bool register_humid_temp_callback(callback Function) | Registers a callback function which is called everytime the temperature or humidity of the chamber changes.                                | -                                                                 |
| bool is_running()                                    | Returns if the chamber is currently running or not.                                                                                        | True if cahmber is running, otherwise return False.               |
