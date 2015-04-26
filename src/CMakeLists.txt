cmake_minimum_required (VERSION 2.8 FATAL_ERROR)
set(ProjectName Autonomous_RC_Car)
project(${ProjectName})

# Get DUO_SDK install path from environment variable if set
if(DEFINED ENV{DUO_SDK})
  set(DUO_SDK $ENV{DUO_SDK} )
else()
  set(DUO_SDK ${CMAKE_CURRENT_SOURCE_DIR}/DUO3D_SDK)
endif()

# Add DUO include and lib
include_directories(${DUO_SDK}/include)

# Detect Compiler / Arch and add libraries
if(MSVC)
  if(CMAKE_CL_64) 
    set(DUOLib_ARCH x64)
  else() 
    set(DUOLib_ARCH x86)
  endif()
  if(MSVC_VERSION EQUAL 1600)
    set(DUOLib_RUNTIME msvc2010)
  elseif(MSVC_VERSION EQUAL 1700)
    set(DUOLib_RUNTIME msvc2012)
  endif() 
  link_directories(${DUO_SDK}/windows/${DUOLib_RUNTIME}/${DUOLib_ARCH})
  file(TO_NATIVE_PATH ${DUO_SDK}/windows/${DUOLib_RUNTIME}/${DUOLib_ARCH}/DUOLib.dll SRC)
  file(TO_NATIVE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/bin/. DST)
  execute_process(COMMAND cmd /c copy /y ${SRC} ${DST})
  # Set output directory for binaries
  # set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/bin )
  # set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/bin )
elseif(APPLE)
  link_directories(${DUO_SDK}/osx/x64/)
  # set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin )
elseif(UNIX)
  if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(DUOLib_ARCH x64)
  else() 
    set(DUOLib_ARCH x86)
  endif()
  link_directories(${DUO_SDK}/linux/${DUOLib_ARCH}/)
  # set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin )
endif()

# find_package(VTK REQUIRED)
# include(${VTK_USE_FILE})

 find_package( PCL 1.2 REQUIRED )

 include_directories(${PCL_INCLUDE_DIRS})
 link_directories(${PCL_LIBRARY_DIRS})
 add_definitions(${PCL_DEFINITIONS})

find_package(OpenCV REQUIRED)

file(GLOB Project_SRC *.cpp *.h)
add_executable(${ProjectName} ${Project_SRC}) 
if(WIN32)
	target_link_libraries(${ProjectName} DUOlib ${OpenCV_LIBRARIES} ${VTK_LIBRARIES} ${PCL_LIBRARIES})
elseif(UNIX OR APPLE)
	target_link_libraries(${ProjectName} duo ${OpenCV_LIBRARIES} ${VTK_LIBRARIES} ${PCL_LIBRARIES})
endif()