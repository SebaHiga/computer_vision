cmake_minimum_required(VERSION 2.8)
project( computer_vision )
find_package( OpenCV REQUIRED )
find_package(nlohmann_json 3.2.0 REQUIRED)
add_executable( computer_vision main.cpp )
target_link_libraries(computer_vision PRIVATE nlohmann_json::nlohmann_json)
target_link_libraries( computer_vision ${OpenCV_LIBS} m )