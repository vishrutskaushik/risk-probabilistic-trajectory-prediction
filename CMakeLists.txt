cmake_minimum_required (VERSION 3.0)
project (risk-probabilistic-trajectory-prediction)

find_package(Eigen3 REQUIRED)

set(EXECUTABLE_CPP_FILE "src/main.cpp")
set(EXECUTABLE "executable")

file(GLOB_RECURSE ALL_SOURCE_FILES "src/*.cpp" "include/*.h")
file(GLOB_RECURSE INCLUDE_FILES "include/*.h")
file(GLOB_RECURSE SRC_FILES_NO_EXEC "src/*.cpp")

message("CMAKE_CURRENT_LIST_DIR: ${CMAKE_CURRENT_LIST_DIR}")
message("SRC_FILES_NO_EXEC: ${SRC_FILES_NO_EXEC}")

list(REMOVE_ITEM SRC_FILES_NO_EXEC "${CMAKE_CURRENT_LIST_DIR}/${EXECUTABLE_CPP_FILE}")

add_executable (${EXECUTABLE} ${EXECUTABLE_CPP_FILE} ${SRC_FILES_NO_EXEC} ${INCLUDE_FILES})
target_include_directories(${EXECUTABLE} PUBLIC include)
target_link_libraries(${EXECUTABLE} PUBLIC Eigen3::Eigen)