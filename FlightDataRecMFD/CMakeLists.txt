cmake_minimum_required(VERSION 3.10)

project(FlightDataRecMFD LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(ORBITER_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../")


include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/../../include
)

set(SOURCES
    FlightDataRecMFD.cpp
)

add_library(FlightDataRecMFD SHARED ${SOURCES})


target_compile_definitions(FlightDataRecMFD
    PRIVATE
        $<$<CONFIG:Debug>:_DEBUG>
        $<$<CONFIG:Release>:NDEBUG>
        WIN32
        _WINDOWS
)


target_link_libraries(FlightDataRecMFD
    PRIVATE
        MFDlib
        ${ORBITER_SDK_DIR}/lib/Orbiter.lib
        ${ORBITER_SDK_DIR}/lib/Orbitersdk.lib
        #${ORBITER_SDK_DIR}/lib/MFDlib.lib
        kernel32
        user32
        gdi32
        winspool
        comdlg32
        advapi32
        shell32
        ole32
        oleaut32
        uuid
        odbc32
        odbccp32
)


target_compile_options(FlightDataRecMFD
    PRIVATE
        $<$<CONFIG:Release>:/O2>
        $<$<CONFIG:Debug>:/Od>
)


set_target_properties(FlightDataRecMFD PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY_DEBUG   "${CMAKE_SOURCE_DIR}/../Modules/Plugin"
    RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_SOURCE_DIR}/../Modules/Plugin"
)
