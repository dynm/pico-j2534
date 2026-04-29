cmake_minimum_required(VERSION 3.20)

project(pico_j2534_vci LANGUAGES CXX)

include(CTest)

add_library(pico_j2534 SHARED
    src/dllmain.cpp
    src/isotp.cpp
    src/j2534.cpp
    src/pico_j2534.def
    src/winusb_transport_win.cpp
)

target_compile_features(pico_j2534 PRIVATE cxx_std_17)
target_include_directories(pico_j2534 PRIVATE include)
target_compile_definitions(pico_j2534 PRIVATE UNICODE _UNICODE WIN32_LEAN_AND_MEAN)
target_link_libraries(pico_j2534 PRIVATE setupapi winusb)

set_target_properties(pico_j2534 PROPERTIES
    OUTPUT_NAME "pico_j2534"
    PREFIX ""
)

add_executable(vin_smoke_test
    tools/vin_smoke_test.cpp
)

target_compile_features(vin_smoke_test PRIVATE cxx_std_17)
target_include_directories(vin_smoke_test PRIVATE include)
target_compile_definitions(vin_smoke_test PRIVATE WIN32_LEAN_AND_MEAN)
target_link_libraries(vin_smoke_test PRIVATE setupapi)

if(BUILD_TESTING)
    add_test(
        NAME vin_smoke_test_exports
        COMMAND vin_smoke_test --check-exports $<TARGET_FILE:pico_j2534>
    )
endif()
