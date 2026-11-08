cmake_minimum_required(VERSION 3.24)
project(SpaceLander2D LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# exe + all DLLs (incl. SDL3.dll) into the same folder
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/$<CONFIG>)

include(FetchContent)
FetchContent_Declare(SDL3
    GIT_REPOSITORY https://github.com/libsdl-org/SDL.git
    GIT_TAG        release-3.2.10
    GIT_SHALLOW    TRUE)
FetchContent_Declare(imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG        v1.91.9
    GIT_SHALLOW    TRUE)
FetchContent_Declare(json
    URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz)
FetchContent_Declare(Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG        v3.7.1
    GIT_SHALLOW    TRUE)

set(SDL_TEST_LIBRARY OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(SDL3 imgui json Catch2)

add_subdirectory(RenderApi)
add_subdirectory(Renderer)
add_subdirectory(SceneData)
add_subdirectory(SceneIO)
add_subdirectory(Physics)
add_subdirectory(EditorModel)
add_subdirectory(Simulation)
add_subdirectory(SceneEditor)
add_subdirectory(SceneExecution)
add_subdirectory(Scene)

enable_testing()
add_subdirectory(Tests)

set_property(DIRECTORY PROPERTY VS_STARTUP_PROJECT Scene)