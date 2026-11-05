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

# Visão travada, robô preso, motor invertido: o robô tem que parar
add_executable(teste_seguranca src/teste_seguranca.cpp)
target_link_libraries(teste_seguranca PRIVATE controle)

# Ponte: visão (UDP) -> controle -> robô (USB serial)
add_executable(ponte_serial src/ponte_serial.cpp)
target_link_libraries(ponte_serial PRIVATE controle)

# Entrada no formato da liga: SSL_WrapperPacket em multicast (SSL-Vision, grSim, ER-Force).
# Fica fora do 'controle' para ele continuar sem dependência e ir para o firmware.
# Sem o protobuf instalado, a ponte compila só com a entrada em texto.
find_package(Protobuf QUIET)
if(Protobuf_FOUND)
  protobuf_generate_cpp(SSL_PROTO_SRCS SSL_PROTO_HDRS
    proto/messages_robocup_ssl_detection.proto
    proto/messages_robocup_ssl_geometry.proto
    proto/messages_robocup_ssl_wrapper.proto
  )
  add_library(visao_ssl src/visao_ssl.cpp ${SSL_PROTO_SRCS})
  target_include_directories(visao_ssl
    PUBLIC include
    PRIVATE ${CMAKE_CURRENT_BINARY_DIR} ${Protobuf_INCLUDE_DIRS})
  target_link_libraries(visao_ssl PRIVATE ${Protobuf_LIBRARIES})
  set_source_files_properties(src/visao_ssl.cpp PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra;-Wpedantic")

  target_link_libraries(ponte_serial PRIVATE visao_ssl)
  target_compile_definitions(ponte_serial PRIVATE COM_VISAO_SSL)

  # Leitura dos pacotes: time, id, confiança, várias câmeras, tempo de processamento
  add_executable(teste_visao_ssl src/teste_visao_ssl.cpp)
  target_include_directories(teste_visao_ssl PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
  target_link_libraries(teste_visao_ssl PRIVATE visao_ssl controle ${Protobuf_LIBRARIES})
else()
  message(STATUS "protobuf não encontrado: ponte_serial sem a entrada SSL")
endif()

# Robô simulado no Gazebo: plugin que faz o papel do firmware, dos motores e da câmera,
# para a ponte_serial rodar contra o simulador. Só entra se o Gazebo Harmonic estiver instalado.
find_package(gz-sim8 QUIET)
find_package(gz-plugin2 QUIET COMPONENTS register)
if(gz-sim8_FOUND AND gz-plugin2_FOUND)
  add_subdirectory(simulacao gazebo)   # build/simulacao já é o executável acima
endif()
