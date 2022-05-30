cmake_minimum_required(VERSION 3.15)

project(labo-1)

#--------------------------------------------------
# Define math lib
#--------------------------------------------------
set(LABO_1_HEADERS DenseStorage.h MatrixBase.h Matrix.h Math3D.h Vector.h Operators.h GTIAssert.h)
add_library(labo-1 INTERFACE)
target_sources( labo-1 INTERFACE ${LABO_1_HEADERS} )
target_include_directories(labo-1 INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

#--------------------------------------------------
# Define test executable
#--------------------------------------------------
add_executable(labo1TestsExtra labo1TestsExtra.cpp tests/DenseStorage_Test.cpp tests/Math3D_Test.cpp tests/Matrix_Test.cpp tests/MatrixBase_Test.cpp tests/Operators_Test.cpp tests/Vector_Test.cpp tests/NouveauLabo2_Test.cpp)
target_link_libraries(labo1TestsExtra gtest)
