cmake_minimum_required(VERSION 3.10)

project(FlightData LANGUAGES CXX RC)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


set(ORBITER_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../")


include_directories(
    ${ORBITER_SDK_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}
)


set(SOURCES
    FDGraph.cpp
    FlightData.cpp
    Graph.cpp
    FlightData.rc
)

set(HEADERS
    FDGraph.h
    Graph.h
    resource.h
)


add_library(FlightData SHARED ${SOURCES} ${HEADERS})


target_compile_definitions(FlightData
    PRIVATE
        $<$<CONFIG:Debug>:_DEBUG>
        $<$<CONFIG:Release>:NDEBUG>
        WIN32
        _WINDOWS
)


target_link_libraries(FlightData
    PRIVATE
        ${ORBITER_SDK_DIR}/lib/Orbiter.lib
        ${ORBITER_SDK_DIR}/lib/Orbitersdk.lib
)


if(MSVC)
    target_compile_options(FlightData PRIVATE
        $<$<CONFIG:Release>:/O2>
        $<$<CONFIG:Debug>:/Od>
    )
endif()


set_target_properties(FlightData PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/Modules/Plugin"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/Modules/Plugin"
    OUTPUT_NAME "FlightDataRecCFD"
)
