cmake_minimum_required(VERSION 2.8.3)
project(InspectionHex)

# For use of pthreads
find_package(Threads REQUIRED)

set(CMAKE_CXX_FLAGS "-O2 -fPIC -Wall -std=c++0x")

set(KMEL_ROOT_PATH ${PROJECT_SOURCE_DIR})

include_directories(
  ${KMEL_ROOT_PATH}/include
)

set_source_files_properties(${KMEL_ROOT_PATH}/src/crc32.c PROPERTIES LANGUAGE CXX)
set_source_files_properties(${KMEL_ROOT_PATH}/src/kBotPacket2.c PROPERTIES LANGUAGE CXX)

add_library(kmeliface SHARED 
  ${KMEL_ROOT_PATH}/src/crc32.c
  ${KMEL_ROOT_PATH}/src/SerialDevice.cc
  ${KMEL_ROOT_PATH}/src/kBotPacket2.c
  ${KMEL_ROOT_PATH}/src/kBotPacketParser.cc
  ${KMEL_ROOT_PATH}/src/kQuadInterface.cc
  ${KMEL_ROOT_PATH}/src/kQuadInterfaceDataTypes.cc
)

set(LIBS kmeliface ${CMAKE_THREAD_LIBS_INIT})

add_executable(testReceiveFeedback samples/C++/testReceiveFeedback.cpp)
add_executable(testSendQuadCmd1 samples/C++/testSendQuadCmd1.cpp)

target_link_libraries(testReceiveFeedback ${LIBS})
target_link_libraries(testSendQuadCmd1 ${LIBS})
