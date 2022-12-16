#include "pybind11/pybind11.h"
#include "ClimateChamberControl.h"

using namespace pybind11;

namespace  py = pybind11;

PYBIND11_MODULE(py_climate_chamber_lib, m) {

    class_<ClimateChamberControl>(m, "ClimateChamberControl")
            .def("initialize", &ClimateChamberControl::initialize)
            .def("deinitialize", &ClimateChamberControl::deInitialize)
            .def("retrieve_climate_chamber_status", &ClimateChamberControl::retrieveClimateChamberStatus)
            .def("get_current_temperature", &ClimateChamberControl::getCurrentTemperature)
            .def("get_current_humidity", &ClimateChamberControl::getCurrentHumidity)
            .def("get_target_temperature", &ClimateChamberControl::getTargetTemperature)
            .def("get_target_humidity", &ClimateChamberControl::getTargetHumidity)
            .def("set_target_temperature", &ClimateChamberControl::setTargetTemperature)
            .def("set_target_humidity", &ClimateChamberControl::setTargetHumidity)
            .def("start_execution", &ClimateChamberControl::startExecution)
            .def("stop_execution", &ClimateChamberControl::stopExecution)
            .def("start_program", &ClimateChamberControl::startProgram)
            .def("stop_program", &ClimateChamberControl::stopProgram)
            .def("get_error_code", &ClimateChamberControl::getErrorCode)
            .def("acknowledge_errors", &ClimateChamberControl::acknowledgeErrors)
            .def("start_monitor_thread", &ClimateChamberControl::startMonitorThread)
            .def("stop_monitor_thread", &ClimateChamberControl::stopMonitorThread)
            .def("register_humid_temp_callback", &ClimateChamberControl::registerHumidityTemperatureCallback)
            .def("is_running", &ClimateChamberControl::isRunning);
}