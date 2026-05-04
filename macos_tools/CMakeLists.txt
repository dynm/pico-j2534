cmake_minimum_required(VERSION 3.20)

project(pico_j2534_macos_tools LANGUAGES CXX)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

add_executable(uds_vin_test
    uds_vin_test.cpp
)

target_compile_features(uds_vin_test PRIVATE cxx_std_17)
target_include_directories(uds_vin_test PRIVATE
    ${LIBUSB_INCLUDE_DIRS}
    ../pico/include
)
target_link_directories(uds_vin_test PRIVATE ${LIBUSB_LIBRARY_DIRS})
target_link_libraries(uds_vin_test PRIVATE ${LIBUSB_LIBRARIES})
target_compile_options(uds_vin_test PRIVATE ${LIBUSB_CFLAGS_OTHER})
