add_executable(Tests
    test_vec2.cpp test_simulation.cpp
    test_sceneio.cpp test_editor_model.cpp)
target_link_libraries(Tests PRIVATE
    Catch2::Catch2WithMain SceneData Simulation SceneIO EditorModel)

list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
include(Catch)
catch_discover_tests(Tests)