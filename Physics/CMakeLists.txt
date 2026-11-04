add_library(Physics STATIC src/Physics.cpp)
target_include_directories(Physics PUBLIC include)
target_link_libraries(Physics PUBLIC SceneData)