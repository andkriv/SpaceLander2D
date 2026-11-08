list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
include(Catch)

add_executable(Tests
    test_vec2.cpp test_simulation.cpp test_physics.cpp
    test_sceneio.cpp test_editor_model.cpp test_file_flow.cpp)
target_link_libraries(Tests PRIVATE
    Catch2::Catch2WithMain SceneData Simulation Physics SceneIO EditorModel)
catch_discover_tests(Tests)

# need SDL3, runs on the dummy video driver
add_executable(RendererTests test_renderer.cpp)
target_link_libraries(RendererTests PRIVATE Catch2::Catch2WithMain Renderer SDL3::SDL3)
catch_discover_tests(RendererTests DISCOVERY_MODE PRE_TEST)

# need ImGui (Renderer.dll), no window
add_executable(UiTests test_draw.cpp test_ui.cpp)
target_link_libraries(UiTests PRIVATE
    Catch2::Catch2WithMain SceneEditor SceneExecution Renderer SceneIO)
catch_discover_tests(UiTests DISCOVERY_MODE PRE_TEST)