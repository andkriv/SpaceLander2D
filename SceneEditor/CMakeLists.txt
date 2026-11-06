add_library(SceneEditor STATIC src/Editor.cpp)
target_include_directories(SceneEditor PUBLIC include)
target_link_libraries(SceneEditor PUBLIC Renderer SceneData PRIVATE SceneIO Physics)