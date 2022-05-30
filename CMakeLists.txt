cmake_minimum_required(VERSION 3.15)

project(GTI320)

#--------------------------------------------------
# Use C++ 17
#--------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#--------------------------------------------------
# Sous MAC, OpenGL est Deprecated, mais toujours
# fonctionnel, on veut éviter tous les warnings
# durant la compilation.
#--------------------------------------------------
if (APPLE)
  add_definitions( -DGL_SILENCE_DEPRECATION )
endif()

#--------------------------------------------------
# Add googletest and setup build
#--------------------------------------------------
include(FetchContent)
set(FETCHCONTENT_QUIET FALSE)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG        release-1.11.0
  GIT_PROGRESS TRUE
)

# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

# No need for GMock
set( BUILD_GMOCK OFF CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(googletest)

#--------------------------------------------------
# Add nanogui and setup build
#--------------------------------------------------
FetchContent_Declare(
  nanogui
  GIT_REPOSITORY https://github.com/wjakob/nanogui.git
  GIT_TAG        e9ec8a1a9861cf578d9c6e85a6420080aa715c03
  GIT_PROGRESS TRUE
)

set(NANOGUI_BUILD_EXAMPLE OFF CACHE BOOL "" FORCE)
set(NANOGUI_BUILD_PYTHON OFF CACHE BOOL "" FORCE)
set(NANOGUI_BUILD_SHARED OFF CACHE BOOL "" FORCE)
set(NANOGUI_INSTALL OFF CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(nanogui)

FetchContent_GetProperties( nanogui SOURCE_DIR nanogui_SRC_DIR BINARY_DIR nanogui_BIN_DIR)

add_definitions(${NANOGUI_EXTRA_DEFS})
include_directories(${nanogui_SRC_DIR}/include)
include_directories(${NANOGUI_EXTRA_INCS})

#--------------------------------------------------
# Build math library (labo 1)
#--------------------------------------------------
add_subdirectory(labo-1)

#--------------------------------------------------
# Build ICP (labo 2)
#--------------------------------------------------
add_subdirectory(labo-2)