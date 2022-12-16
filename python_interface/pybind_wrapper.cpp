#include "pybind11/pybind11.h"
#include "ClimateChamberWrapper.h"

using namespace pybind11;

namespace  py = pybind11;

PYBIND11_MODULE(py_climate_chamber_lib, m) {

    class_<ClimateChamberWrapper>(m, "ClimateChamberWrapper")
            .def("Initialize", &ClimateChamberWrapper::Initialize)
            .def("DeInitialize", &ClimateChamberWrapper::DeInitialize)
            .def("RetrieveClimateChamberStatus", &ClimateChamberWrapper::RetrieveClimateChamberStatus)
            .def("GetCurrentTemperature", &ClimateChamberWrapper::GetCurrentTemperature)
            .def("GetCurrentHumidity", &ClimateChamberWrapper::GetCurrentHumidity)
            .def("GetTargetTemperature", &ClimateChamberWrapper::GetTargetTemperature)
            .def("GetTargetHumidity", &ClimateChamberWrapper::GetTargetHumidity)
            .def("SetTargetTemperature", &ClimateChamberWrapper::SetTargetTemperature)
            .def("SetTargetHumidity", &ClimateChamberWrapper::SetTargetHumidity)
            .def("StartExecution", &ClimateChamberWrapper::StartExecution)
            .def("StopExecution", &ClimateChamberWrapper::StopExecution)
            .def("StartProgram", &ClimateChamberWrapper::StartProgram)
            .def("StopProgram", &ClimateChamberWrapper::StopProgram)
            .def("GetErrorCode", &ClimateChamberWrapper::GetErrorCode)
            .def("AcknowledgeErrors", &ClimateChamberWrapper::AcknowledgeErrors)
            .def("StartMonitorThread", &ClimateChamberWrapper::StartMonitorThread)
            .def("StopMonitorThread", &ClimateChamberWrapper::StopMonitorThread)
            .def("RegisterHumidityTemperatureCallback", &ClimateChamberWrapper::RegisterHumidityTemperatureCallback)
            .def("IsRunning", &ClimateChamberWrapper::IsRunning);
}