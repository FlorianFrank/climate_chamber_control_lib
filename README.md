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

A sample program on how to use the climate chamber is provided in following code snipped:

```c++
#include <ClimateChamberControl.h>

int main() {
    ClimateChamberControl chamber;
    if(!chamber.Initialize("<your_ip_address>")) // The standard port to access the climate chamber is 8080
        std::cout << "Error: " << chamber.GetLastError() << std::endl;
        
    chamber.SetTargetTemperature(26.5);
    chamber.SetTargetHumidity(30.0);
    
    chamber.StartMonitorThread(5000);
    
    auto callback = [](float humidity, float temperature) {
        std::cout << "Humidity: " << humidity << " Temperature: " << temperature << std::endl;
    };
    
    chamber.RegisterTemperatureCallback(callback);
    
    chamber.StartExecution();
    
    /*** Do some stuff here ***/

    chamber.StopExecution();
    chamber.Deinitialize();
```

## 3 API-description

The api supports following functions: 

| Method Name                                                 | Description                                                                                                                                | Return value                                                      | 
|-------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------|
| bool Initialize(ipAddr, port, channel);                     | Establishes a connection to the climate chamber.                                                                                           | Returns true if the connection could be established successfully. |
| bool DeInitialize()                                         | Closes the connection to the climate chamber.                                                                                              | True if command was successful.                                   |
| bool RetrieveClimateChamberStatus()                         | Sends a status request to the climate chamber to get the current temperature and humidity and stores them in the climate chamber object.   | True if command was successful.                                   |
| float GetCurrentTemperature()                               | Get the temperature retrieved from the last RetrieveClimateChamberStatus() call.                                                           | Last retrieved temperature.                                       |
| float GetCurrentHumidity()                                  | Get the humidity retrieved from the last RetrieveClimateChamberStatus() call.                                                              | Last retrieved humidity.                                          |
| float GetTargetTemperature()                                | Returns the target temperature which should finally be reached during a cool down or heat up progress.                                     | Target temperature.                                               |
| float GetTargetHumidity()                                   | Returns the target humidity which should finally be reached.                                                                               | Target humidity.                                                  |
| bool SetTargetTemperature()                                 | Sets the target temperature to reach after starting or stopping the chamber.                                                               | True if command was successful.                                   |
| bool SetTargetHumidity()                                    | Sets the target humidity to reach after starting or stopping the chamber.                                                                  | True if command was successful.                                   |
| bool StartExecution()                                       | Starts the climate chamber to heat up or cool down to the temperature and humidity adjusted by SetTargetHumidity and SetTargetTemperature. | True if command was successful.                                   |
| bool StopExecution()                                        | Stops the execution of the climate chamber.                                                                                                | True if command was successful.                                   |
| bool StartProgram(id)                                       | Start a specific program stored on the climate chamber, identified by an id.                                                               | True if command was successful.                                   |
| bool StopProgram()                                          | Stop a currently running program.                                                                                                          | True if command was successful.                                   |
| bool GetErrorCode(errCode)                                  | Starts the climate chamber to heat up or cool down to the temperature and humidity adjusted by SetTargetHumidity and SetTargetTemperature. | True if command was successful.                                   |
| bool AcknowledgeErrors()                                    | Acknowledge all errors stored in the error buffer.                                                                                         | True if command was successful.                                   |
 | bool StartMonitorThread(int intervalMs)                     | Starts a thread which periodically calls RetrieveClimateChamberStatus() to update the current temperature.                                 | True if command was successful.                                   |
| bool RegisterHumidityTemperatureCallback(callback Function) | Registers a callback function which is called everytime the temperature or humidity of the chamber changes.                                | -                                                                 |
| bool IsRunning()                                            | Returns if the chamber is currently running or not.                                                                                        | True if cahmber is running, otherwise return False.               |
