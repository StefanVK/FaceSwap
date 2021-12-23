# References for CMakeLists with qt
# https://doc.qt.io/qt-6/cmake-get-started.html
# https://github.com/Andrew9317/qt-cmake-template
# https://www.reddit.com/r/cpp/comments/kfv6kk/qt6_cmake_quickstart_template/

# https://github.com/lukka/CppBuildTasks-Validation/blob/v10/cmakepresets/CMakeLists.txt
# Ensure to pick up the default triplet from the environment if any. This helps 
# driving the vcpkg triplet in the same way either when starting vcpkg directly,
# or when letting CMake start vcpkg at configure/generate time.
# Note: this logic must happen before PROJECT command.
if(DEFINED ENV{VCPKG_DEFAULT_TRIPLET} AND NOT DEFINED VCPKG_TARGET_TRIPLET)
  set(VCPKG_TARGET_TRIPLET "$ENV{VCPKG_DEFAULT_TRIPLET}" CACHE STRING "The vcpkg triplet")
endif()

cmake_minimum_required(VERSION 3.20)

project(FaceMorph)
include(cmake/StandardProjectSettings.cmake)
set(QT_MAJOR_VERSION 6)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt6 COMPONENTS Widgets Gui REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc photo calib3d imgcodecs)

include_directories( ${OpenCV_INCLUDE_DIRS} )

qt_add_executable(
	FaceMorph
	"src/AnnotatorWidget.cpp" "src/ImageWidget.cpp" "src/main.cpp" "src/MainWindow.cpp" "src/morph.cpp" "src/Morpher.cpp" "src/PointHandle.cpp" "src/MainWindow.ui"
)


target_include_directories(FaceMorph PRIVATE ${Qt${QT_MAJOR_VERSION}_INCLUDE_DIRS})
target_include_directories(FaceMorph PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(FaceMorph PRIVATE Qt::Widgets Qt::Core Qt::Gui ${OpenCV_LIBS})

if(MSVC)
  #https://discourse.cmake.org/t/how-to-set-warning-level-correctly-in-modern-cmake/1103
  #string(REGEX REPLACE "/W[1-3]" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
  #target_compile_options(FaceMorph PRIVATE /W4 /WX)
else()
  #target_compile_options(FaceMorph PRIVATE -Wall -Wextra -Wpedantic)
endif()


install(TARGETS FaceMorph RUNTIME DESTINATION ${CMAKE_INSTALL_FULL_BINDIR} COMPONENT applications)
install(FILES ${qms} DESTINATION ${CMAKE_INSTALL_FULL_BINDIR} COMPONENT applications)


# Set path to our custom CMAKE scripts
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake")

IF(WIN32)
	include(windeployqt)
	windeployqt(FaceMorph)
ENDIF()
