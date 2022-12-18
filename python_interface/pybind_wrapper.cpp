#include "pybind11/pybind11.h"
#include "ClimateChamberControl.h"
#include "ctlib/ErrorCodeDefines.h"

using namespace pybind11;

namespace  py = pybind11;

PYBIND11_MODULE(py_climate_chamber_lib, m) {

    enum_<PIL_ERROR_CODE>(m, "ERROR_CODE")
        .value("NO_ERROR", PIL_NO_ERROR)
        .value("INVALID_ARGUMENTS", PIL_INVALID_ARGUMENTS)
        .value("ERRNO", PIL_ERRNO)
        .value("TIMEOUT", PIL_TIMEOUT)
        .value("INTERFACE_CLOSED", PIL_INTERFACE_CLOSED)
        .value("INVALID_BAUDRATE", PIL_INVALID_BAUDRATE)
        .value("INSUFFICIENT_RESOURCES", PIL_INSUFFICIENT_RESOURCES)
        .value("INSUFFICIENT_PERMISSIONS", PIL_INSUFFICIENT_PERMISSIONS)
        .value("DEADLOCK_DETECTED", PIL_DEADLOCK_DETECTED)
        .value("THREAD_NOT_JOINABLE", PIL_THREAD_NOT_JOINABLE)
        .value("THREAD_NOT_FOUND", PIL_THREAD_NOT_FOUND)
        .value("ONLY_PARTIALLY_READ_WRITTEN", PIL_ONLY_PARTIALLY_READ_WRITTEN)
        .value("NO_SUCH_FILE", PIL_NO_SUCH_FILE)
        .value("UNKNOWN_ERROR", PIL_UNKNOWN_ERROR)
        .value("XML_PARSING_ERROR", PIL_XML_PARSING_ERROR);

    class_<ClimateChamberControl>(m, "ClimateChamberControl")
            .def(pybind11::init<>())
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
            .def("acknowledge_errors", &ClimateChamberControl::acknowledgeErrors)
            .def("start_monitor_thread", &ClimateChamberControl::startMonitorThread)
            .def("stop_monitor_thread", &ClimateChamberControl::stopMonitorThread)
            .def("register_humid_temp_callback", &ClimateChamberControl::registerHumidityTemperatureCallback)
            .def("is_running", &ClimateChamberControl::isRunning);
}