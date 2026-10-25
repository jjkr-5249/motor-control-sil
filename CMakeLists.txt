cmake_minimum_required(VERSION 3.16)
project(MotorControlSIL LANGUAGES C)

set(CMAKE_C_STANDARD 99)

add_executable(${PROJECT_NAME}
    src/main.c
    src/motor_model.c
    src/pid_controller.c
    src/hal.c
    src/simulator.c
)

target_include_directories(${PROJECT_NAME} PRIVATE include)
target_link_libraries(${PROJECT_NAME} PRIVATE m)
