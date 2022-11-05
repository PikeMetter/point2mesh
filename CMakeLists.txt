cmake_minimum_required(VERSION 3.19.1)
project(point2mesh)

#-----------------------------------------------------------------------------
# Enable C++11
#-----------------------------------------------------------------------------
set(_msg "Setting C++ standard")
message(STATUS "${_msg}")
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
message(STATUS "${_msg} - C++${CMAKE_CXX_STANDARD}")
if(NOT CMAKE_CXX_STANDARD MATCHES "^(11|14|17)$")
  message(FATAL_ERROR "CMAKE_CXX_STANDARD must be set to 11, 14 or 17")
endif()

set(_default_vtk "8")
set(${PROJECT_NAME}_VTK_VERSION_MAJOR ${_default_vtk} CACHE STRING "VTK major version (8 or 9)")
set_property(CACHE ${PROJECT_NAME}_VTK_VERSION_MAJOR PROPERTY STRINGS "8" "9")
if(NOT "${${PROJECT_NAME}_VTK_VERSION_MAJOR}" MATCHES "^(8|9)$")
  message(FATAL_ERROR "error: ${PROJECT_NAME}_VTK_VERSION_MAJOR must be 8 or 9.")
endif()

#-----------------------------------------------------------------------------
# ${PROJECT_NAME}_VTK_COMPONENTS
#-----------------------------------------------------------------------------
if (${PROJECT_NAME}_VTK_VERSION_MAJOR VERSION_LESS "9")
  set(VTK_COMPONENT_PREFIX "vtk")
  set(VTK_TARGET_PREFIX "vtk")
else()
  set(VTK_COMPONENT_PREFIX "")
  set(VTK_TARGET_PREFIX "VTK::")
endif()
set(${PROJECT_NAME}_VTK_COMPONENTS
  ${VTK_COMPONENT_PREFIX}FiltersExtraction
  ${VTK_COMPONENT_PREFIX}FiltersFlowPaths
  ${VTK_COMPONENT_PREFIX}FiltersGeometry
  ${VTK_COMPONENT_PREFIX}FiltersParallel
  ${VTK_COMPONENT_PREFIX}IOExport
  ${VTK_COMPONENT_PREFIX}IOImage
  ${VTK_COMPONENT_PREFIX}IOLegacy
  ${VTK_COMPONENT_PREFIX}IOPLY
  ${VTK_COMPONENT_PREFIX}IOXML
  ${VTK_COMPONENT_PREFIX}ImagingMath
  ${VTK_COMPONENT_PREFIX}ImagingMorphological
  ${VTK_COMPONENT_PREFIX}ImagingStatistics
  ${VTK_COMPONENT_PREFIX}ImagingStencil
  ${VTK_COMPONENT_PREFIX}InteractionImage
  ${VTK_COMPONENT_PREFIX}RenderingVolume${${PROJECT_NAME}_VTK_RENDERING_BACKEND}
  ${VTK_COMPONENT_PREFIX}zlib
  ${VTK_COMPONENT_PREFIX}CommonCore
  ${VTK_COMPONENT_PREFIX}CommonDataModel
  ${VTK_COMPONENT_PREFIX}FiltersSources
  ${VTK_COMPONENT_PREFIX}InteractionWidgets
  ${VTK_COMPONENT_PREFIX}InteractionStyle
  ${VTK_COMPONENT_PREFIX}RenderingCore
  ${VTK_COMPONENT_PREFIX}RenderingFreeType
  ${VTK_COMPONENT_PREFIX}RenderingQt
  ${VTK_COMPONENT_PREFIX}RenderingOpenGL2  
  ${VTK_COMPONENT_PREFIX}ViewsQt
  ${VTK_COMPONENT_PREFIX}GUISupportQt
  )

if(${PROJECT_NAME}_USE_PYTHONQT)
  list(APPEND ${PROJECT_NAME}_VTK_COMPONENTS
    ${VTK_COMPONENT_PREFIX}WrappingPythonCore
    )
endif()

#-----------------------------------------------------------------------------
# VTK
#-----------------------------------------------------------------------------
find_package(VTK REQUIRED)
if (VTK_VERSION VERSION_LESS "8.90")
  find_package(VTK COMPONENTS ${${PROJECT_NAME}_VTK_COMPONENTS} REQUIRED NO_MODULE)
  include(${VTK_USE_FILE})
  set(VTK_GUI_SUPPORT_QT_TARGET_NAME "vtkGUISupportQt")
else()
  set(VTK_LIBRARIES "")
  find_package(VTK COMPONENTS ${${PROJECT_NAME}_VTK_COMPONENTS} REQUIRED)
  set(VTK_GUI_SUPPORT_QT_TARGET_NAME "VTK::GUISupportQt")
endif ()

if(NOT TARGET ${VTK_GUI_SUPPORT_QT_TARGET_NAME})
  message(FATAL_ERROR "error: VTK was not configured to use Qt, you probably need "
                    "to recompile it with VTK_USE_GUISUPPORT ON, VTK_Group_Qt ON, "
                    "Note that Qt >= ${${PROJECT_NAME}_REQUIRED_QT_VERSION} is *required*")
endif()
set(VTK_RENDERING_BACKEND "${${PROJECT_NAME}_VTK_RENDERING_BACKEND}")

#----------------------------------------------------------------------------
# PCL
#----------------------------------------------------------------------------
find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#----------------------------------------------------------------------------
# OpenCV
#----------------------------------------------------------------------------
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

#----------------------------------------------------------------------------
# OpenCASCADE 7.3.0
#----------------------------------------------------------------------------
find_package(OpenCASCADE REQUIRED )
include_directories (SYSTEM ${OpenCASCADE_INCLUDE_DIR} )

#----------------------------------------------------------------------------
# Eigen3
#----------------------------------------------------------------------------
find_package(Eigen3 REQUIRED)
include_directories(${Egien3_INCLUDE_DIRS})

# Source files
set(KIT_SRCS
    main.cpp
    LandMark.cpp
  )

# Headers that should run through moc
set(KIT_MOC_SRCS
    filters.h
    meshRec.h
    poissonRec.h
    vtkRec.h
    myICP.h
  )

# UI files
set(KIT_UI_SRCS
  )


add_executable(${PROJECT_NAME} ${KIT_SRCS} ${KIT_UI_SRCS} ${KIT_MOC_SRCS} )

# Add linker options
foreach (LIB ${OpenCASCADE_LIBRARIES})
  target_link_libraries(${PROJECT_NAME} debug ${OpenCASCADE_LIBRARY_DIR}d/${LIB}.lib)
  target_link_libraries(${PROJECT_NAME} optimized ${OpenCASCADE_LIBRARY_DIR}/${LIB}.lib)
endforeach()

target_link_libraries(${PROJECT_NAME} ${VTK_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen3_LIBRARIES}${OpenCASCADE_LIBRARIES})
