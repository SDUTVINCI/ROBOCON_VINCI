# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "D:/25RC_Project/R2/R2_short_ball_v1/MDK-ARM/tmp/F4_short_ball_v1+F4_short_ball_v1")
  file(MAKE_DIRECTORY "D:/25RC_Project/R2/R2_short_ball_v1/MDK-ARM/tmp/F4_short_ball_v1+F4_short_ball_v1")
endif()
file(MAKE_DIRECTORY
  "D:/25RC_Project/R2/R2_short_ball_v1/MDK-ARM/tmp/1"
  "D:/25RC_Project/R2/R2_short_ball_v1/MDK-ARM/tmp/F4_short_ball_v1+F4_short_ball_v1"
  "D:/25RC_Project/R2/R2_short_ball_v1/MDK-ARM/tmp/F4_short_ball_v1+F4_short_ball_v1/tmp"
  "D:/25RC_Project/R2/R2_short_ball_v1/MDK-ARM/tmp/F4_short_ball_v1+F4_short_ball_v1/src/F4_short_ball_v1+F4_short_ball_v1-stamp"
  "D:/25RC_Project/R2/R2_short_ball_v1/MDK-ARM/tmp/F4_short_ball_v1+F4_short_ball_v1/src"
  "D:/25RC_Project/R2/R2_short_ball_v1/MDK-ARM/tmp/F4_short_ball_v1+F4_short_ball_v1/src/F4_short_ball_v1+F4_short_ball_v1-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/25RC_Project/R2/R2_short_ball_v1/MDK-ARM/tmp/F4_short_ball_v1+F4_short_ball_v1/src/F4_short_ball_v1+F4_short_ball_v1-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "D:/25RC_Project/R2/R2_short_ball_v1/MDK-ARM/tmp/F4_short_ball_v1+F4_short_ball_v1/src/F4_short_ball_v1+F4_short_ball_v1-stamp${cfgdir}") # cfgdir has leading slash
endif()
