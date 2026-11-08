add_library(EditorModel STATIC src/EditorModel.cpp)
target_include_directories(EditorModel PUBLIC include)
target_link_libraries(EditorModel PUBLIC SceneData)