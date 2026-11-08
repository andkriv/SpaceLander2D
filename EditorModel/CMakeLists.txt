add_library(EditorModel STATIC src/EditorModel.cpp src/SceneFileFlow.cpp)
target_include_directories(EditorModel PUBLIC include)
target_link_libraries(EditorModel PUBLIC SceneData RenderApi PRIVATE SceneIO)