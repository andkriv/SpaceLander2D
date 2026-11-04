set(IMGUI_SRC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/imgui_demo.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_sdl3.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_sdlrenderer3.cpp)

add_library(Renderer SHARED src/Renderer.cpp ${IMGUI_SRC})

target_include_directories(Renderer
    PUBLIC  include ${imgui_SOURCE_DIR}
    PRIVATE ${imgui_SOURCE_DIR}/backends)

target_link_libraries(Renderer PRIVATE SDL3::SDL3)

target_compile_definitions(Renderer
    PRIVATE   RENDERER_EXPORTS IMGUI_API=__declspec\(dllexport\)
    INTERFACE IMGUI_API=__declspec\(dllimport\))