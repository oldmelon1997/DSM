cmake_minimum_required(VERSION 3.5)

# Set the name of the Program
set(PROGRAM_NAME InteriorNetExample)

add_executable(${PROGRAM_NAME} main.cpp)

include_directories(${CMAKE_CURRENT_BINARY_DIR}
	${CMAKE_BINARY_DIR}
	${CMAKE_SOURCE_DIR}/thirdparty/Sophus
	${EIGEN3_INCLUDE_DIR}
	${QT_VISUALIZER_LIB_INCLUDE_DIR}
	${DSM_LIB_INCLUDE_DIR}
	)

target_link_libraries(${PROGRAM_NAME} ${OpenCV_LIBS} QtVisualizer dsm)

set_target_properties(${PROGRAM_NAME} PROPERTIES FOLDER "Examples")