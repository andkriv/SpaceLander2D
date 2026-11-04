add_library(SceneIO STATIC src/SceneIO.cpp)
target_include_directories(SceneIO PUBLIC include)
target_link_libraries(SceneIO PUBLIC SceneData PRIVATE nlohmann_json::nlohmann_json)