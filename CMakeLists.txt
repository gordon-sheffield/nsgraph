cmake_minimum_required(VERSION 3.10)
project(NSGraph VERSION 1.0)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic -Werror -O0 -g)

file(GLOB_RECURSE SOURCES "src/*.cpp" "src/*.h") 

add_executable(NSGraph examples/example.cpp ${SOURCES}
        src/graph_util.h)

install(TARGETS NSGraph DESTINATION lib)

target_link_libraries(NSGraph m) 