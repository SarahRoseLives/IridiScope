cmake_minimum_required(VERSION 3.16)
project(IridiScope C CXX)

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

if(MINGW)
  add_compile_definitions(_WIN32_WINNT=0x0601 WIN32_LEAN_AND_MEAN NOMINMAX)
  add_compile_options(-Wno-unused-parameter -Wno-unused-variable)
endif()

# ---- Dependencies ----------------------------------------------------------
find_package(Threads REQUIRED)
find_package(OpenGL REQUIRED)

# GLFW: build the vendored copy (preferred) so the build is self-contained.
set(GLFW_VENDORED ${CMAKE_SOURCE_DIR}/third_party/glfw)
if(EXISTS ${GLFW_VENDORED}/CMakeLists.txt)
  set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
  add_subdirectory(${GLFW_VENDORED} ${CMAKE_BINARY_DIR}/glfw)
  message(STATUS "Using vendored GLFW")
else()
  find_package(glfw3 QUIET)
  if(NOT glfw3_FOUND)
    find_path(GLFW_INCLUDE_DIR GLFW/glfw3.h)
    find_library(GLFW_LIBRARY NAMES glfw3 glfw)
    if(NOT GLFW_LIBRARY OR NOT GLFW_INCLUDE_DIR)
      message(FATAL_ERROR "GLFW not found (vendored copy missing and no system package).")
    endif()
    add_library(glfw INTERFACE)
    target_include_directories(glfw INTERFACE ${GLFW_INCLUDE_DIR})
    target_link_libraries(glfw INTERFACE ${GLFW_LIBRARY})
  endif()
endif()

# FFTW single-precision: strictly prefer the vendored static build, and only
# fall back to a system package if the vendored build is absent.
set(FFTW_VENDORED ${CMAKE_SOURCE_DIR}/third_party/fftw/install)
if(EXISTS ${FFTW_VENDORED}/lib/libfftw3f.a AND EXISTS ${FFTW_VENDORED}/include/fftw3.h)
  set(FFTW_INCLUDE_DIR ${FFTW_VENDORED}/include)
  set(FFTW_LIBRARY ${FFTW_VENDORED}/lib/libfftw3f.a)
  message(STATUS "Using vendored FFTW: ${FFTW_LIBRARY}")
else()
  find_path(FFTW_INCLUDE_DIR fftw3.h)
  find_library(FFTW_LIBRARY NAMES fftw3f libfftw3f)
  if(NOT FFTW_LIBRARY)
    message(WARNING "FFTW3f not found -- DSP stages (later milestones) need it. Run scripts/bootstrap.ps1.")
  else()
    message(STATUS "Using system FFTW: ${FFTW_LIBRARY} (run scripts/bootstrap.ps1 for the vendored build)")
  endif()
endif()

# ---- Vendored iridium-sniffer DSP core ------------------------------------
set(CORE_DIR ${CMAKE_SOURCE_DIR}/references/iridium-sniffer)
set(CORE_SOURCES
  ${CORE_DIR}/burst_detect.c
  ${CORE_DIR}/burst_downmix.c
  ${CORE_DIR}/qpsk_demod.c
  ${CORE_DIR}/fir_filter.c
  ${CORE_DIR}/window_func.c
  ${CORE_DIR}/simd_generic.c
  ${CORE_DIR}/simd_sse42.c
  ${CORE_DIR}/simd_avx2.c
  ${CMAKE_SOURCE_DIR}/src/iridium/engine_globals.c
  ${CMAKE_SOURCE_DIR}/src/iridium/pipeline_c.c
)

# Force-include the Windows compatibility shim into every C core file, and
# give the SIMD kernels their ISA flags.
set(WIN_COMPAT ${CMAKE_SOURCE_DIR}/src/compat/win_compat.h)
set_source_files_properties(${CORE_SOURCES} PROPERTIES
  COMPILE_OPTIONS "-include;${WIN_COMPAT}")
set_source_files_properties(${CORE_DIR}/simd_sse42.c PROPERTIES
  COMPILE_OPTIONS "-include;${WIN_COMPAT};-msse4.2")
set_source_files_properties(${CORE_DIR}/simd_avx2.c PROPERTIES
  COMPILE_OPTIONS "-include;${WIN_COMPAT};-mavx2;-mfma")

# ---- Dear ImGui + ImPlot (vendored) ---------------------------------------
set(IMGUI_DIR  ${CMAKE_SOURCE_DIR}/third_party/imgui)
set(IMPLOT_DIR ${CMAKE_SOURCE_DIR}/third_party/implot)
set(IMGUI_SOURCES
  ${IMGUI_DIR}/imgui.cpp
  ${IMGUI_DIR}/imgui_draw.cpp
  ${IMGUI_DIR}/imgui_tables.cpp
  ${IMGUI_DIR}/imgui_widgets.cpp
  ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
  ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
  ${IMPLOT_DIR}/implot.cpp
  ${IMPLOT_DIR}/implot_items.cpp
)
# implot_items.cpp is very template-heavy; -O1 keeps build times sane.
set_source_files_properties(${IMPLOT_DIR}/implot_items.cpp PROPERTIES COMPILE_OPTIONS "-O1")

# ---- Application sources ---------------------------------------------------
# osmo-ir77 AMBE vocoder (AGPL-3.0) for voice decoding.
set(IR77_DIR ${CMAKE_SOURCE_DIR}/references/osmo-ir77/codec)
set(CODEC_SOURCES
  ${IR77_DIR}/ambe.c
  ${IR77_DIR}/ecc.c
  ${IR77_DIR}/frame.c
  ${IR77_DIR}/math.c
  ${IR77_DIR}/synth.c
  ${IR77_DIR}/tables.c
  ${IR77_DIR}/tone.c
)

set(APP_SOURCES
  src/app.cpp
  src/core/file_source.cpp
  src/core/engine.cpp
  src/parser/fec.cpp
  src/parser/iridium_parser.cpp
  src/messages/acars.cpp
  src/ui/waterfall.cpp
  src/voice/voice_manager.cpp
  src/voice/audio.cpp
  src/voice/miniaudio_impl.c
  ${CODEC_SOURCES}
)

# WIN32 => Windows GUI subsystem: no console window on launch.
add_executable(IridiScope WIN32 ${APP_SOURCES} ${IMGUI_SOURCES} ${CORE_SOURCES})

target_include_directories(IridiScope PRIVATE
  src
  src/compat
  src/iridium
  ${CORE_DIR}
  ${IR77_DIR}
  ${IMGUI_DIR}
  ${IMGUI_DIR}/backends
  ${IMPLOT_DIR}
  ${CMAKE_SOURCE_DIR}/third_party
  ${CMAKE_SOURCE_DIR}/third_party/miniaudio
)

target_compile_definitions(IridiScope PRIVATE
  IRIDISCOPE_ASSETS="${CMAKE_SOURCE_DIR}/assets"
  IRIDISCOPE_VERSION_MAJOR=0
  IRIDISCOPE_VERSION_MINOR=1
)

target_link_libraries(IridiScope PRIVATE glfw OpenGL::GL Threads::Threads)

if(FFTW_LIBRARY AND FFTW_INCLUDE_DIR)
  target_include_directories(IridiScope PRIVATE ${FFTW_INCLUDE_DIR})
  target_link_libraries(IridiScope PRIVATE ${FFTW_LIBRARY})
endif()

if(MINGW)
  target_link_libraries(IridiScope PRIVATE
    gdi32 user32 shell32 comdlg32 winmm ws2_32)

  # Ship the MinGW runtime DLLs next to the executable so it runs without
  # requiring the toolchain bin directory on PATH.
  get_filename_component(MINGW_BIN_DIR "${CMAKE_C_COMPILER}" DIRECTORY)
  set(MINGW_RUNTIME_DLLS
    libgcc_s_seh-1.dll
    libgcc_s_dw2-1.dll
    libstdc++-6.dll
    libwinpthread-1.dll
    libssp-0.dll)
  foreach(dll ${MINGW_RUNTIME_DLLS})
    if(EXISTS "${MINGW_BIN_DIR}/${dll}")
      add_custom_command(TARGET IridiScope POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${MINGW_BIN_DIR}/${dll}"
                "$<TARGET_FILE_DIR:IridiScope>/${dll}"
        COMMENT "Copying ${dll}")
    endif()
  endforeach()
endif()
