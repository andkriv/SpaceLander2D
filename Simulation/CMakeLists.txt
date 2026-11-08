add_library(Simulation STATIC src/Simulation.cpp)
target_include_directories(Simulation PUBLIC include)
target_link_libraries(Simulation PUBLIC Physics SceneData)