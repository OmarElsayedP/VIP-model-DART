# Install script for directory: /mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/blasfeo

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/build/blasfeo/libblasfeo.a")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/cmake/blasfeoConfig.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/cmake/blasfeoConfig.cmake"
         "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/build/blasfeo/CMakeFiles/Export/cmake/blasfeoConfig.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/cmake/blasfeoConfig-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/cmake/blasfeoConfig.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/cmake" TYPE FILE FILES "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/build/blasfeo/CMakeFiles/Export/cmake/blasfeoConfig.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/cmake" TYPE FILE FILES "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/build/blasfeo/CMakeFiles/Export/cmake/blasfeoConfig-noconfig.cmake")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE FILE FILES
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/blasfeo/include/blasfeo.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/blasfeo/include/blasfeo_block_size.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/blasfeo/include/blasfeo_common.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/blasfeo/include/blasfeo_d_aux.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/blasfeo/include/blasfeo_d_aux_ext_dep.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/blasfeo/include/blasfeo_d_aux_ext_dep_ref.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/blasfeo/include/blasfeo_d_aux_old.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/blasfeo/include/blasfeo_d_aux_ref.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/blasfeo/include/blasfeo_d_aux_test.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/blasfeo/include/blasfeo_d_blas.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/blasfeo/include/blasfeo_d_blas_api.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/blasfeo/include/blasfeo_d_blasfeo_api.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/blasfeo/include/blasfeo_d_blasfeo_api_ref.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/blasfeo/include/blasfeo_d_kernel.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/blasfeo/include/blasfeo_i_aux_ext_dep.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/blasfeo/include/blasfeo_m_aux.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/blasfeo/include/blasfeo_naming.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/blasfeo/include/blasfeo_processor_features.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/blasfeo/include/blasfeo_s_aux.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/blasfeo/include/blasfeo_s_aux_ext_dep.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/blasfeo/include/blasfeo_s_aux_ext_dep_ref.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/blasfeo/include/blasfeo_s_aux_old.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/blasfeo/include/blasfeo_s_aux_ref.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/blasfeo/include/blasfeo_s_aux_test.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/blasfeo/include/blasfeo_s_blas.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/blasfeo/include/blasfeo_s_blas_api.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/blasfeo/include/blasfeo_s_blasfeo_api.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/blasfeo/include/blasfeo_s_blasfeo_api_ref.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/blasfeo/include/blasfeo_s_kernel.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/blasfeo/include/blasfeo_stdlib.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/blasfeo/include/blasfeo_target.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/blasfeo/include/blasfeo_timing.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/blasfeo/include/blasfeo_v_aux_ext_dep.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/blasfeo/include/d_blas.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/blasfeo/include/d_blas_64.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/blasfeo/include/s_blas.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/blasfeo/include/s_blas_64.h"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/build/blasfeo/examples/cmake_install.cmake")

endif()

