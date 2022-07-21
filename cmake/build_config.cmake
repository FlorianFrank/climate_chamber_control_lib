# Set new CMP0077 policy used to build pugixml
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)

# Definitions for the platform independent abstraction layer.
option(PIL_COMMUNICATION    "Enable PIL Sockets"          ON)
option(PIL_THREADING        "Enable PIL Threads"          ON)
option(PIL_LOGGING          "Enable Logging support"      ON)
option(PIL_SHARED           "BUILD PIL as shared library" OFF)
option(PIL_STATIC           "BUILD PIL as static library" ON)
option(PIL_CXX              "Enable PIL C++ support"      ON)

