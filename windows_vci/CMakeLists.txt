cmake_minimum_required(VERSION 3.20)

project(pico_j2534_vci LANGUAGES CXX)

include(CTest)

set(PICO_J2534_DEF src/pico_j2534.def)
if(MINGW AND CMAKE_SIZEOF_VOID_P EQUAL 4)
    set(PICO_J2534_DEF src/pico_j2534_mingw32.def)
endif()

add_library(pico_j2534 SHARED
    src/dllmain.cpp
    src/j2534.cpp
    ${PICO_J2534_DEF}
    src/winusb_transport_win.cpp
)

target_compile_features(pico_j2534 PRIVATE cxx_std_17)
target_include_directories(pico_j2534 PRIVATE include)
target_compile_definitions(pico_j2534 PRIVATE UNICODE _UNICODE WIN32_LEAN_AND_MEAN NOMINMAX)
target_link_libraries(pico_j2534 PRIVATE setupapi winusb)
if(MINGW)
    target_link_options(pico_j2534 PRIVATE -static -static-libgcc -static-libstdc++ -s)
endif()

set_target_properties(pico_j2534 PROPERTIES
    OUTPUT_NAME "pico_j2534"
    PREFIX ""
)

add_executable(vin_smoke_test
    tools/vin_smoke_test.cpp
)

target_compile_features(vin_smoke_test PRIVATE cxx_std_17)
target_include_directories(vin_smoke_test PRIVATE include)
target_compile_definitions(vin_smoke_test PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)
if(MINGW)
    target_link_options(vin_smoke_test PRIVATE -static -static-libgcc -static-libstdc++ -s)
endif()

add_executable(dts_can_monitor
    tools/dts_can_monitor.cpp
)

target_compile_features(dts_can_monitor PRIVATE cxx_std_17)
target_include_directories(dts_can_monitor PRIVATE include)
target_compile_definitions(dts_can_monitor PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)
if(MINGW)
    target_link_options(dts_can_monitor PRIVATE -static -static-libgcc -static-libstdc++ -s)
endif()

if(BUILD_TESTING)
    add_test(
        NAME vin_smoke_test_exports
        COMMAND vin_smoke_test --check-exports $<TARGET_FILE:pico_j2534>
    )
endif()
