set(SOURCES main.cpp opts.cpp device.cpp)

add_executable(test ${SOURCES})

target_link_libraries(test PRIVATE bladeRF)
