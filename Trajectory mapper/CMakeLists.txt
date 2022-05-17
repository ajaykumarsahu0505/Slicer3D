cmake_minimum_required(VERSION 3.13.4)

project(Trajectory Mapper)

#-----------------------------------------------------------------------------
# Extension meta-information
set(EXTENSION_HOMEPAGE "https://www.slicer.org/wiki/Documentation/Nightly/Extensions/Trajectory Mapper")
set(EXTENSION_CATEGORY "IGT")
set(EXTENSION_CONTRIBUTORS "Ajay (Alpha corp)")
set(EXTENSION_DESCRIPTION "This extension give you the module to mark a trajectory for needle incision during surgery.")
set(EXTENSION_ICONURL "http://www.example.com/Slicer/Extensions/Pathfinader.png")
set(EXTENSION_SCREENSHOTURLS "http://www.example.com/Slicer/Extensions/Pathfinader/Screenshots/1.png")
set(EXTENSION_DEPENDS "NA") # Specified as a list or "NA" if no dependencies

#-----------------------------------------------------------------------------
# Extension dependencies
find_package(Slicer REQUIRED)
include(${Slicer_USE_FILE})

#-----------------------------------------------------------------------------
# Extension modules
add_subdirectory(Trajectory)
## NEXT_MODULE

#-----------------------------------------------------------------------------
include(${Slicer_EXTENSION_GENERATE_CONFIG})
include(${Slicer_EXTENSION_CPACK})
