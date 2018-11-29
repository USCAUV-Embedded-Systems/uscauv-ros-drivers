set(OSC99_SOURCES
        OscAddress.cpp
        OscBundle.cpp
        OscCommon.cpp
        OscError.cpp
        OscMessage.cpp
        OscPacket.cpp
        OscSlip.cpp)

add_library(osc99 ${OSC99_SOURCES})
target_include_directories(osc99 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})