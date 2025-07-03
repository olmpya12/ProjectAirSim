# ---------------------------------------------------------------------------------------------------------------------
#
# Copyright (C) Microsoft Corporation.  
# Copyright (C) 2025 IAMAI Consulting Corp.
#
# MIT License. All rights reserved.
#
# Module Name:
#
#   CMakeLists.txt
#
# Abstract:
#
#   CMake build file.
#
# ---------------------------------------------------------------------------------------------------------------------

# Add main rover_api source
add_subdirectory(src)

# Add rover unit tests
add_subdirectory(test)