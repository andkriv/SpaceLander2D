add_executable(Scene src/main.cpp)
target_link_libraries(Scene PRIVATE Renderer SceneEditor SceneExecution)