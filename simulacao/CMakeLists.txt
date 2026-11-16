# Plugin do Gazebo que faz o papel do hardware do robô (firmware, motores e câmera).
# O Gazebo acha a biblioteca pela variável GZ_SIM_SYSTEM_PLUGIN_PATH (ver rodar_gazebo.sh).
add_library(robo_simulado SHARED robo_simulado.cpp)
target_link_libraries(robo_simulado PRIVATE gz-plugin2::register gz-sim8::gz-sim8)
target_compile_options(robo_simulado PRIVATE -Wall -Wextra)
