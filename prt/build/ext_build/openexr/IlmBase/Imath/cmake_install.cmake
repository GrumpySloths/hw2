# Install script for directory: /home/niujh/games202/hw2/prt/ext/openexr/IlmBase/Imath

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
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/home/niujh/games202/hw2/prt/build/ext_build/openexr/IlmBase/Imath/libImath.a")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/OpenEXR" TYPE FILE FILES
    "/home/niujh/games202/hw2/prt/ext/openexr/IlmBase/Imath/ImathBoxAlgo.h"
    "/home/niujh/games202/hw2/prt/ext/openexr/IlmBase/Imath/ImathBox.h"
    "/home/niujh/games202/hw2/prt/ext/openexr/IlmBase/Imath/ImathColorAlgo.h"
    "/home/niujh/games202/hw2/prt/ext/openexr/IlmBase/Imath/ImathColor.h"
    "/home/niujh/games202/hw2/prt/ext/openexr/IlmBase/Imath/ImathEuler.h"
    "/home/niujh/games202/hw2/prt/ext/openexr/IlmBase/Imath/ImathExc.h"
    "/home/niujh/games202/hw2/prt/ext/openexr/IlmBase/Imath/ImathExport.h"
    "/home/niujh/games202/hw2/prt/ext/openexr/IlmBase/Imath/ImathForward.h"
    "/home/niujh/games202/hw2/prt/ext/openexr/IlmBase/Imath/ImathFrame.h"
    "/home/niujh/games202/hw2/prt/ext/openexr/IlmBase/Imath/ImathFrustum.h"
    "/home/niujh/games202/hw2/prt/ext/openexr/IlmBase/Imath/ImathFrustumTest.h"
    "/home/niujh/games202/hw2/prt/ext/openexr/IlmBase/Imath/ImathFun.h"
    "/home/niujh/games202/hw2/prt/ext/openexr/IlmBase/Imath/ImathGL.h"
    "/home/niujh/games202/hw2/prt/ext/openexr/IlmBase/Imath/ImathGLU.h"
    "/home/niujh/games202/hw2/prt/ext/openexr/IlmBase/Imath/ImathHalfLimits.h"
    "/home/niujh/games202/hw2/prt/ext/openexr/IlmBase/Imath/ImathInt64.h"
    "/home/niujh/games202/hw2/prt/ext/openexr/IlmBase/Imath/ImathInterval.h"
    "/home/niujh/games202/hw2/prt/ext/openexr/IlmBase/Imath/ImathLimits.h"
    "/home/niujh/games202/hw2/prt/ext/openexr/IlmBase/Imath/ImathLineAlgo.h"
    "/home/niujh/games202/hw2/prt/ext/openexr/IlmBase/Imath/ImathLine.h"
    "/home/niujh/games202/hw2/prt/ext/openexr/IlmBase/Imath/ImathMath.h"
    "/home/niujh/games202/hw2/prt/ext/openexr/IlmBase/Imath/ImathMatrixAlgo.h"
    "/home/niujh/games202/hw2/prt/ext/openexr/IlmBase/Imath/ImathMatrix.h"
    "/home/niujh/games202/hw2/prt/ext/openexr/IlmBase/Imath/ImathNamespace.h"
    "/home/niujh/games202/hw2/prt/ext/openexr/IlmBase/Imath/ImathPlane.h"
    "/home/niujh/games202/hw2/prt/ext/openexr/IlmBase/Imath/ImathPlatform.h"
    "/home/niujh/games202/hw2/prt/ext/openexr/IlmBase/Imath/ImathQuat.h"
    "/home/niujh/games202/hw2/prt/ext/openexr/IlmBase/Imath/ImathRandom.h"
    "/home/niujh/games202/hw2/prt/ext/openexr/IlmBase/Imath/ImathRoots.h"
    "/home/niujh/games202/hw2/prt/ext/openexr/IlmBase/Imath/ImathShear.h"
    "/home/niujh/games202/hw2/prt/ext/openexr/IlmBase/Imath/ImathSphere.h"
    "/home/niujh/games202/hw2/prt/ext/openexr/IlmBase/Imath/ImathVecAlgo.h"
    "/home/niujh/games202/hw2/prt/ext/openexr/IlmBase/Imath/ImathVec.h"
    )
endif()

