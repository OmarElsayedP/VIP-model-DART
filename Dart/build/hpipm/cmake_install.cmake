# Install script for directory: /mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/build/hpipm/libhpipm.a")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/cmake/hpipmConfig.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/cmake/hpipmConfig.cmake"
         "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/build/hpipm/CMakeFiles/Export/cmake/hpipmConfig.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/cmake/hpipmConfig-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/cmake/hpipmConfig.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/cmake" TYPE FILE FILES "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/build/hpipm/CMakeFiles/Export/cmake/hpipmConfig.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/cmake" TYPE FILE FILES "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/build/hpipm/CMakeFiles/Export/cmake/hpipmConfig-noconfig.cmake")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE FILE FILES
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_aux_mem.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_aux_string.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_common.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_d_cast_qcqp.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_d_cond.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_d_cond_aux.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_d_cond_qcqp.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_d_core_qp_ipm.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_d_core_qp_ipm_aux.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_d_dense_qcqp.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_d_dense_qcqp_dim.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_d_dense_qcqp_ipm.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_d_dense_qcqp_res.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_d_dense_qcqp_sol.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_d_dense_qcqp_utils.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_d_dense_qp.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_d_dense_qp_dim.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_d_dense_qp_ipm.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_d_dense_qp_kkt.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_d_dense_qp_res.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_d_dense_qp_sol.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_d_dense_qp_utils.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_d_ocp_qcqp.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_d_ocp_qcqp_dim.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_d_ocp_qcqp_ipm.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_d_ocp_qcqp_res.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_d_ocp_qcqp_sol.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_d_ocp_qcqp_utils.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_d_ocp_qp.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_d_ocp_qp_dim.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_d_ocp_qp_ipm.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_d_ocp_qp_kkt.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_d_ocp_qp_red.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_d_ocp_qp_res.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_d_ocp_qp_sol.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_d_ocp_qp_utils.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_d_part_cond.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_d_part_cond_qcqp.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_d_sim_erk.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_d_sim_rk.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_d_tree_ocp_qp.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_d_tree_ocp_qp_dim.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_d_tree_ocp_qp_ipm.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_d_tree_ocp_qp_kkt.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_d_tree_ocp_qp_res.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_d_tree_ocp_qp_sol.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_m_dense_qp.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_m_dense_qp_dim.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_m_ocp_qp.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_m_ocp_qp_ipm_hard.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_m_ocp_qp_kkt.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_s_cast_qcqp.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_s_cond.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_s_cond_aux.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_s_cond_qcqp.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_s_core_qp_ipm.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_s_core_qp_ipm_aux.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_s_dense_qcqp.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_s_dense_qcqp_dim.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_s_dense_qcqp_ipm.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_s_dense_qcqp_res.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_s_dense_qcqp_sol.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_s_dense_qcqp_utils.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_s_dense_qp.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_s_dense_qp_dim.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_s_dense_qp_ipm.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_s_dense_qp_kkt.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_s_dense_qp_res.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_s_dense_qp_sol.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_s_dense_qp_utils.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_s_ocp_qcqp.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_s_ocp_qcqp_dim.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_s_ocp_qcqp_ipm.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_s_ocp_qcqp_res.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_s_ocp_qcqp_sol.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_s_ocp_qcqp_utils.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_s_ocp_qp.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_s_ocp_qp_dim.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_s_ocp_qp_ipm.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_s_ocp_qp_kkt.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_s_ocp_qp_red.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_s_ocp_qp_res.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_s_ocp_qp_sol.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_s_ocp_qp_utils.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_s_part_cond.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_s_part_cond_qcqp.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_s_sim_erk.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_s_sim_rk.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_s_tree_ocp_qp.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_s_tree_ocp_qp_dim.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_s_tree_ocp_qp_ipm.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_s_tree_ocp_qp_kkt.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_s_tree_ocp_qp_res.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_s_tree_ocp_qp_sol.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_scenario_tree.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_target.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_timing.h"
    "/mnt/c/Users/hp/Desktop/masters/thesis/CODE/Dart/hpipm/include/hpipm_tree.h"
    )
endif()

