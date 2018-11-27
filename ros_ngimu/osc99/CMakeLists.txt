set(OSC99_SOURCES
        OscAddress.c
        OscBundle.c
        OscCommon.c
        OscError.c
        OscMessage.c
        OscPacket.c
        OscSlip.c)

add_library(osc99 ${OSC99_SOURCES})
target_include_directories(osc99 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})