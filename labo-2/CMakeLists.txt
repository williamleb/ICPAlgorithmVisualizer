cmake_minimum_required(VERSION 3.15)

project(labo-2)

set( OBJ_DEFAULT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../obj/ )
configure_file( IcpDataPath.h.in IcpDataPath.h )

set(HEADERS Icp.h IcpApplication.h IcpGLCanvas.h SVD.h)
set(SOURCES main.cpp IcpApplication.cpp Icp.cpp IcpGLCanvas.cpp README.txt)
add_executable(labo-2 ${SOURCES} ${HEADERS})

target_include_directories( labo-2 PRIVATE ${CMAKE_CURRENT_BINARY_DIR} )
target_link_libraries(labo-2 labo-1 nanogui ${NANOGUI_EXTRA_LIBS})
