cmake_minimum_required(VERSION 3.16)
project(IridiScope C CXX)

# Embed the application icon (icon.ico in the project root). Windows uses the
# "1" resource for the exe/taskbar; GLFW loads the named "GLFW_ICON" resource
# for the window/titlebar.
set(IRIDISCOPE_RC "")
if(EXISTS "${CMAKE_SOURCE_DIR}/icon.ico")
  enable_language(RC)
  set(IRIDISCOPE_ICON "${CMAKE_SOURCE_DIR}/icon.ico")
  configure_file("${CMAKE_SOURCE_DIR}/app.rc.in" "${CMAKE_BINARY_DIR}/app.rc" @ONLY)
  set(IRIDISCOPE_RC "${CMAKE_BINARY_DIR}/app.rc")
  message(STATUS "Embedding application icon: ${IRIDISCOPE_ICON}")
endif()

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

# ---- SDRplay (optional native source) -------------------------------------
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
  set(SDRPLAY_ARCH x64)
else()
  set(SDRPLAY_ARCH x86)
endif()
find_path(SDRPLAY_INCLUDE_DIR NAMES sdrplay_api.h)
find_library(SDRPLAY_LIBRARY NAMES sdrplay_api)
# The Windows SDRplay API installs under Program Files, which is not on CMake's
# default search path. Add hints there (and honour SDRPLAY_API_ROOT /
# SDRPLAY_API_PATH). On Linux the system paths above already resolve.
if(WIN32 AND (NOT SDRPLAY_INCLUDE_DIR OR NOT SDRPLAY_LIBRARY))
  find_path(SDRPLAY_INCLUDE_DIR NAMES sdrplay_api.h
    HINTS ${SDRPLAY_API_ROOT} "$ENV{SDRPLAY_API_PATH}" "$ENV{ProgramFiles}/SDRplay/API"
    PATH_SUFFIXES inc)
  find_library(SDRPLAY_LIBRARY NAMES sdrplay_api
    HINTS ${SDRPLAY_API_ROOT} "$ENV{SDRPLAY_API_PATH}" "$ENV{ProgramFiles}/SDRplay/API"
    PATH_SUFFIXES ${SDRPLAY_ARCH})
endif()
set(SDRPLAY_FOUND FALSE)
if(SDRPLAY_INCLUDE_DIR AND SDRPLAY_LIBRARY)
  set(SDRPLAY_FOUND TRUE)
  message(STATUS "SDRplay API found: ${SDRPLAY_LIBRARY}")
else()
  message(STATUS "SDRplay API not found - SDRplay source disabled")
endif()

# ---- HackRF (optional native source) --------------------------------------
find_path(HACKRF_INCLUDE_DIR NAMES libhackrf/hackrf.h)
find_library(HACKRF_LIBRARY NAMES hackrf libhackrf)
set(HACKRF_FOUND FALSE)
if(HACKRF_INCLUDE_DIR AND HACKRF_LIBRARY)
  set(HACKRF_FOUND TRUE)
  message(STATUS "HackRF library found: ${HACKRF_LIBRARY}")
else()
  message(STATUS "HackRF library not found - HackRF source disabled")
endif()

# ---- Airspy (optional native source) --------------------------------------
find_path(AIRSPY_INCLUDE_DIR NAMES airspy.h)
find_library(AIRSPY_LIBRARY NAMES airspy libairspy)
set(AIRSPY_FOUND FALSE)
if(AIRSPY_INCLUDE_DIR AND AIRSPY_LIBRARY)
  set(AIRSPY_FOUND TRUE)
  message(STATUS "Airspy library found: ${AIRSPY_LIBRARY}")
else()
  message(STATUS "Airspy library not found - Airspy source disabled")
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
  src/sdr/live_iq_source.cpp
  src/acars/aircraft_lookup.cpp
  src/update/version_check.cpp
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
add_executable(IridiScope WIN32 ${APP_SOURCES} ${IMGUI_SOURCES} ${CORE_SOURCES} ${IRIDISCOPE_RC})

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

# SDRplay: compile the backend and link only when the API is present.
if(SDRPLAY_FOUND)
  target_sources(IridiScope PRIVATE src/sdr/sdrplay_source.cpp)
  target_compile_definitions(IridiScope PRIVATE HAS_SDRPLAY=1)
  target_include_directories(IridiScope PRIVATE ${SDRPLAY_INCLUDE_DIR})
  target_link_libraries(IridiScope PRIVATE ${SDRPLAY_LIBRARY})
endif()

# HackRF: compile the backend and link only when libhackrf is present.
if(HACKRF_FOUND)
  target_sources(IridiScope PRIVATE src/sdr/hackrf_source.cpp)
  target_compile_definitions(IridiScope PRIVATE HAS_HACKRF=1)
  target_include_directories(IridiScope PRIVATE ${HACKRF_INCLUDE_DIR})
  target_link_libraries(IridiScope PRIVATE ${HACKRF_LIBRARY})
endif()

# Airspy: compile the backend and link only when libairspy is present.
if(AIRSPY_FOUND)
  target_sources(IridiScope PRIVATE src/sdr/airspy_source.cpp)
  target_compile_definitions(IridiScope PRIVATE HAS_AIRSPY=1)
  target_include_directories(IridiScope PRIVATE ${AIRSPY_INCLUDE_DIR})
  target_link_libraries(IridiScope PRIVATE ${AIRSPY_LIBRARY})
endif()

if(MINGW)
  target_link_libraries(IridiScope PRIVATE
    gdi32 user32 shell32 comdlg32 winmm ws2_32 winhttp)

  # Ship the MinGW runtime DLLs next to the executable so it runs without
  # requiring the toolchain bin directory on PATH.
  get_filename_component(MINGW_BIN_DIR "${CMAKE_C_COMPILER}" DIRECTORY)
  set(MINGW_RUNTIME_DLLS
    libgcc_s_seh-1.dll
    libgcc_s_dw2-1.dll
    libstdc++-6.dll
    libwinpthread-1.dll
    libssp-0.dll)
  if(HACKRF_FOUND)
    list(APPEND MINGW_RUNTIME_DLLS libhackrf.dll libusb-1.0.dll)
  endif()
  if(AIRSPY_FOUND)
    list(APPEND MINGW_RUNTIME_DLLS libairspy.dll)
  endif()
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

# Copy sdrplay_api.dll next to the executable so SDRplay devices work without
# the API's bin directory on PATH.
if(SDRPLAY_FOUND)
  get_filename_component(SDRPLAY_LIB_DIR "${SDRPLAY_LIBRARY}" DIRECTORY)
  find_file(SDRPLAY_RUNTIME NAMES sdrplay_api.dll
    HINTS "${SDRPLAY_LIB_DIR}"
          "$ENV{ProgramFiles}/SDRplay/API/${SDRPLAY_ARCH}"
          "${SDRPLAY_API_ROOT}/${SDRPLAY_ARCH}")
  if(SDRPLAY_RUNTIME)
    add_custom_command(TARGET IridiScope POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
              "${SDRPLAY_RUNTIME}" "$<TARGET_FILE_DIR:IridiScope>"
      COMMENT "Copying sdrplay_api.dll")
  else()
    message(WARNING "sdrplay_api.dll not found; SDRplay devices need the API installed")
  endif()
endif()
