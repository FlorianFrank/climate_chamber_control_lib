# Python Wrapper for the Climate chamber lib
# Author: Sebastian Pretzsch
import ctypes
import pathlib
import sys

libname = pathlib.Path().absolute() / "build/libClimateChamberControlLib.so"
_climate = ctypes.CDLL(libname)

_climate.InitializeLogging.restype = ctypes.c_void_p
_climate.InitializeLogging()

_climate.Initialize.restype = ctypes.c_bool
_climate.Initialize.argtypes = [ctypes.c_char_p, ctypes.c_int16, ctypes.c_int8]
_climate.DeInitialize.restype = ctypes.c_bool

_climate.RetrieveClimateChamberStatus.restype = ctypes.c_bool

_climate.GetCurrentHumidity.restype = ctypes.c_float
_climate.GetCurrentTemperature.restype = ctypes.c_float

_climate.GetTargetTemperature.restype = ctypes.c_float
_climate.SetTargetTemperature.argtypes = [ctypes.c_float]

_climate.StartExecution.restype = ctypes.c_bool
_climate.StopExecution.restype = ctypes.c_bool

_climate.StartProgram.argtypes = [ctypes.c_int]
_climate.StartProgram.restype = ctypes.c_bool

_climate.StopProgram.restype = ctypes.c_bool

def initialize_chamber(ipAdress, port, channel):
    return _climate.Initialize(ctypes.c_char_p(ipAdress), ctypes.c_int16(port), ctypes.c_int8(channel))

def deinitialize_chamber():
    return _climate.DeInitialize()

def retrieve_climate_chamber_status():
    return _climate.RetrieveClimateChamberStatus()

def get_current_humidity():
    return _climate.GetCurrentHumidity()

def get_current_temperature():
    return _climate.GetCurrentTemperature()

def get_target_temperature():
    return _climate.GetTargetTemperature()

def set_target_temperature(targetTemperature):
    _climate.SetTargetTemperature(ctypes.c_float(targetTemperature))

def get_target_humidity():
    return _climate.GetTargetHumidity()

def set_target_humidity(targetHumidity):
    _climate.SetTargetHumidity(ctypes.c_float(targetHumidity))

def start_execution():
    return _climate.StartExecution()

def stop_execution():
    return _climate.StopExecution()

def start_program(programID):
    return _climate.StartProgram(ctypes.c_int(programID))

def stop_program():
    return _climate.StopProgram()

    