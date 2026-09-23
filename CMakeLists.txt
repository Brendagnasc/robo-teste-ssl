cmake_minimum_required(VERSION 3.10)
project(robo_teste_ssl CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Biblioteca de controle: física, FSM e camada de entrada da visão.
# É este alvo que depois vai para o firmware do robô da SSL ou para um nó ROS.
add_library(controle
  src/ssl_controle.cpp
  src/entrada_visao.cpp
)
target_include_directories(controle PUBLIC include)
target_compile_options(controle PRIVATE -Wall -Wextra -Wpedantic)

# Simulação da planta + controle, sem hardware
add_executable(simulacao src/main_simulacao.cpp)
target_link_libraries(simulacao PRIVATE controle)

# Comparação pose crua x pose filtrada e predita
add_executable(teste_entrada src/teste_entrada_visao.cpp)
target_link_libraries(teste_entrada PRIVATE controle)

# Ponte: visão (UDP) -> controle -> robô (USB serial)
add_executable(ponte_serial src/ponte_serial.cpp)
target_link_libraries(ponte_serial PRIVATE controle)
