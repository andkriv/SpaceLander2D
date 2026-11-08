add_executable(Tests
    test_vec2.cpp test_simulation.cpp
    test_sceneio.cpp)
target_link_libraries(Tests PRIVATE
    Catch2::Catch2WithMain SceneData Simulation SceneIO)

list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
include(Catch)
catch_discover_tests(Tests)