cmake_minimum_required(VERSION 3.16)
project(IridiScope C CXX)

# Embed resources in the executable so the app does not depend on an absolute
# source-tree path at runtime. The map falls back to an external copy next to
# the executable if one is present.
set(IRIDISCOPE_ICON "${CMAKE_SOURCE_DIR}/icon.ico")
set(IRIDISCOPE_WORLDMAP "${CMAKE_SOURCE_DIR}/assets/worldmap.jpg")
set(IRIDISCOPE_DEFAULT_LAYOUT "${CMAKE_SOURCE_DIR}/assets/default_layout.ini")
set(IRIDISCOPE_ICON_RES "")
set(IRIDISCOPE_ASSET_RES "")
if(EXISTS "${IRIDISCOPE_ICON}")
  set(IRIDISCOPE_ICON_RES
      "GLFW_ICON ICON \"${IRIDISCOPE_ICON}\"\n1 ICON \"${IRIDISCOPE_ICON}\"")
endif()
if(EXISTS "${IRIDISCOPE_WORLDMAP}" AND EXISTS "${IRIDISCOPE_DEFAULT_LAYOUT}")
  set(IRIDISCOPE_ASSET_RES
      "100 RCDATA \"${IRIDISCOPE_WORLDMAP}\"\n101 RCDATA \"${IRIDISCOPE_DEFAULT_LAYOUT}\"")
endif()

set(IRIDISCOPE_RC "")
# Windows resources (.rc) only exist on Windows; on macOS/Linux the map and
# default layout are read from the assets folder (or the .app bundle).
if(WIN32 AND (IRIDISCOPE_ICON_RES OR IRIDISCOPE_ASSET_RES))
  enable_language(RC)
  configure_file("${CMAKE_SOURCE_DIR}/app.rc.in" "${CMAKE_BINARY_DIR}/app.rc" @ONLY)
  set(IRIDISCOPE_RC "${CMAKE_BINARY_DIR}/app.rc")
  message(STATUS "Embedding application resources")
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
# libairspy installs its header under include/libairspy/ on Linux and Homebrew.
find_path(AIRSPY_INCLUDE_DIR NAMES airspy.h PATH_SUFFIXES libairspy)
find_library(AIRSPY_LIBRARY NAMES airspy libairspy)
set(AIRSPY_FOUND FALSE)
if(AIRSPY_INCLUDE_DIR AND AIRSPY_LIBRARY)
  set(AIRSPY_FOUND TRUE)
  message(STATUS "Airspy library found: ${AIRSPY_LIBRARY}")
else()
  message(STATUS "Airspy library not found - Airspy source disabled")
endif()

# ---- RTL-SDR (optional native source) -------------------------------------
find_path(RTLSDR_INCLUDE_DIR NAMES rtl-sdr.h)
find_library(RTLSDR_LIBRARY NAMES rtlsdr librtlsdr)
set(RTLSDR_FOUND FALSE)
if(RTLSDR_INCLUDE_DIR AND RTLSDR_LIBRARY)
  set(RTLSDR_FOUND TRUE)
  message(STATUS "RTL-SDR library found: ${RTLSDR_LIBRARY}")
else()
  message(STATUS "RTL-SDR library not found - RTL-SDR source disabled")
endif()

# ---- libacars-2 (optional ACARS / ARINC-622 decoder) ----------------------
find_path(LIBACARS_INCLUDE_DIR NAMES libacars/libacars.h PATH_SUFFIXES libacars-2)
find_library(LIBACARS_LIBRARY NAMES acars-2 acars libacars-2 libacars)
set(LIBACARS_FOUND FALSE)
if(LIBACARS_INCLUDE_DIR AND LIBACARS_LIBRARY)
  set(LIBACARS_FOUND TRUE)
  message(STATUS "libacars found: ${LIBACARS_LIBRARY}")
else()
  message(STATUS "libacars not found - ARINC-622 decoding disabled")
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
  ${CMAKE_SOURCE_DIR}/src/iridium/engine_globals.c
  ${CMAKE_SOURCE_DIR}/src/iridium/pipeline_c.c
)

# Pick the SIMD kernels for the target CPU. simd_init() selects the best tier
# at runtime (CPUID on x86; NEON is always present on AArch64 / Apple Silicon).
# On macOS, CMAKE_OSX_ARCHITECTURES (e.g. arm64) wins over the host processor.
set(IRIDI_TARGET_ARCH "${CMAKE_SYSTEM_PROCESSOR}")
if(APPLE AND CMAKE_OSX_ARCHITECTURES)
  set(IRIDI_TARGET_ARCH "${CMAKE_OSX_ARCHITECTURES}")
endif()
set(IRIDI_SIMD_X86 FALSE)
if(IRIDI_TARGET_ARCH MATCHES "x86_64|AMD64|amd64|i[3-6]86|x86")
  set(IRIDI_SIMD_X86 TRUE)
  list(APPEND CORE_SOURCES ${CORE_DIR}/simd_sse42.c ${CORE_DIR}/simd_avx2.c)
  message(STATUS "SIMD: SSE4.2 + AVX2+FMA kernels (runtime-detected)")
elseif(IRIDI_TARGET_ARCH MATCHES "arm64|aarch64|AARCH64|ARM64")
  list(APPEND CORE_SOURCES ${CORE_DIR}/simd_neon.c)
  message(STATUS "SIMD: NEON kernels (AArch64)")
else()
  message(STATUS "SIMD: scalar kernels only (${IRIDI_TARGET_ARCH})")
endif()

# Force-include the compatibility shim into every C core file (it is empty
# outside Windows), and give the x86 SIMD kernels their ISA flags.
set(WIN_COMPAT ${CMAKE_SOURCE_DIR}/src/compat/win_compat.h)
set_source_files_properties(${CORE_SOURCES} PROPERTIES
  COMPILE_OPTIONS "-include;${WIN_COMPAT}")
if(IRIDI_SIMD_X86)
  set_source_files_properties(${CORE_DIR}/simd_sse42.c PROPERTIES
    COMPILE_OPTIONS "-include;${WIN_COMPAT};-msse4.2")
  set_source_files_properties(${CORE_DIR}/simd_avx2.c PROPERTIES
    COMPILE_OPTIONS "-include;${WIN_COMPAT};-mavx2;-mfma")
endif()

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
  src/parser/itl_tables.cpp
  src/messages/acars.cpp
  src/messages/pager.cpp
  src/messages/l3.cpp
  src/output/gsmtap.cpp
  src/ui/waterfall.cpp
  src/util/iritime.cpp
  src/voice/voice_manager.cpp
  src/voice/audio.cpp
  src/voice/miniaudio_impl.c
  ${CODEC_SOURCES}
)

# Single source for the version (used by the app and the macOS bundle).
set(IRIDISCOPE_VERSION_MAJOR 0)
set(IRIDISCOPE_VERSION_MINOR 5)

# Cross-platform HTTP (update check, aircraft lookup) uses libcurl outside
# Windows; Windows keeps its native WinHTTP code.
list(APPEND APP_SOURCES src/util/http_get.cpp)

# On macOS, build a double-clickable IridiScope.app by default.
if(APPLE)
  option(IRIDISCOPE_MACOS_BUNDLE "Build IridiScope as a macOS .app bundle" ON)
endif()

# WIN32 => Windows GUI subsystem: no console window on launch.
if(APPLE AND IRIDISCOPE_MACOS_BUNDLE)
  # App icon (icon.icns next to icon.ico), copied into Contents/Resources.
  set(IRIDISCOPE_ICNS "${CMAKE_SOURCE_DIR}/icon.icns")
  set(IRIDISCOPE_ICNS_SRC "")
  if(EXISTS "${IRIDISCOPE_ICNS}")
    set(IRIDISCOPE_ICNS_SRC "${IRIDISCOPE_ICNS}")
    set_source_files_properties("${IRIDISCOPE_ICNS}" PROPERTIES
      MACOSX_PACKAGE_LOCATION Resources)
  endif()
  add_executable(IridiScope MACOSX_BUNDLE ${APP_SOURCES} ${IMGUI_SOURCES} ${CORE_SOURCES}
                 ${IRIDISCOPE_ICNS_SRC})
  if(IRIDISCOPE_ICNS_SRC)
    set_target_properties(IridiScope PROPERTIES MACOSX_BUNDLE_ICON_FILE "icon.icns")
  endif()
  set_target_properties(IridiScope PROPERTIES
    MACOSX_BUNDLE_BUNDLE_NAME "IridiScope"
    MACOSX_BUNDLE_GUI_IDENTIFIER "dev.sarahsforge.iridiscope"
    MACOSX_BUNDLE_SHORT_VERSION_STRING "${IRIDISCOPE_VERSION_MAJOR}.${IRIDISCOPE_VERSION_MINOR}"
    MACOSX_BUNDLE_BUNDLE_VERSION "${IRIDISCOPE_VERSION_MAJOR}.${IRIDISCOPE_VERSION_MINOR}")
else()
  add_executable(IridiScope WIN32 ${APP_SOURCES} ${IMGUI_SOURCES} ${CORE_SOURCES} ${IRIDISCOPE_RC})
endif()

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
  IRIDISCOPE_VERSION_MAJOR=${IRIDISCOPE_VERSION_MAJOR}
  IRIDISCOPE_VERSION_MINOR=${IRIDISCOPE_VERSION_MINOR}
)

# Also place the assets next to the executable so users can override the map
# and so packaged builds have a visible assets folder.
if(APPLE AND IRIDISCOPE_MACOS_BUNDLE)
  # Inside the bundle: IridiScope.app/Contents/Resources/assets
  add_custom_command(TARGET IridiScope POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${CMAKE_SOURCE_DIR}/assets" "$<TARGET_FILE_DIR:IridiScope>/../Resources/assets"
    COMMENT "Copying assets into the app bundle")
else()
  add_custom_command(TARGET IridiScope POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${CMAKE_SOURCE_DIR}/assets" "$<TARGET_FILE_DIR:IridiScope>/assets"
    COMMENT "Copying assets")
endif()

target_link_libraries(IridiScope PRIVATE glfw OpenGL::GL Threads::Threads)

if(NOT WIN32)
  find_package(CURL)
  if(CURL_FOUND)
    target_compile_definitions(IridiScope PRIVATE HAS_LIBCURL=1)
    target_link_libraries(IridiScope PRIVATE CURL::libcurl)
    message(STATUS "libcurl found - update check and aircraft lookup enabled")
  else()
    message(STATUS "libcurl not found - update check and aircraft lookup disabled")
  endif()
  target_link_libraries(IridiScope PRIVATE ${CMAKE_DL_LIBS} m)
endif()

if(APPLE)
  # OpenGL is deprecated (but fully working) on macOS; silence the warnings.
  target_compile_definitions(IridiScope PRIVATE GL_SILENCE_DEPRECATION)
  # miniaudio's CoreAudio backend.
  target_link_libraries(IridiScope PRIVATE
    "-framework CoreFoundation" "-framework CoreAudio" "-framework AudioToolbox")
endif()

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

# RTL-SDR: compile the backend and link only when librtlsdr is present.
if(RTLSDR_FOUND)
  target_sources(IridiScope PRIVATE src/sdr/rtlsdr_source.cpp)
  target_compile_definitions(IridiScope PRIVATE HAS_RTLSDR=1)
  target_include_directories(IridiScope PRIVATE ${RTLSDR_INCLUDE_DIR})
  target_link_libraries(IridiScope PRIVATE ${RTLSDR_LIBRARY})
endif()

# libacars-2: optional ARINC-622 / ADS-C / CPDLC decoding.
target_sources(IridiScope PRIVATE src/acars/libacars_bridge.cpp)
if(LIBACARS_FOUND)
  target_compile_definitions(IridiScope PRIVATE HAS_LIBACARS=1)
  target_include_directories(IridiScope PRIVATE ${LIBACARS_INCLUDE_DIR})
  target_link_libraries(IridiScope PRIVATE ${LIBACARS_LIBRARY})
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
  if(RTLSDR_FOUND)
    list(APPEND MINGW_RUNTIME_DLLS librtlsdr.dll libusb-1.0.dll)
  endif()
  if(LIBACARS_FOUND)
    list(APPEND MINGW_RUNTIME_DLLS
      libacars-2.dll libjansson-4.dll libxml2-16.dll zlib1.dll
      libiconv-2.dll liblzma-5.dll)
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
# the API's bin directory on PATH (Windows only).
if(WIN32 AND SDRPLAY_FOUND)
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
