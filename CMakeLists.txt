cmake_minimum_required (VERSION 2.8)
project ("biped")

set(chronodir /usr/local/chrono)
include_directories (.)
include_directories (${chronodir}/include/chrono)
include_directories (${chronodir}/include/bullet)
link_directories (${chronodir}/lib)

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
	add_defnitions(-DNOMINMAX)
endif()

file(GLOB sources "src/*.cpp")
file(GLOB headers "include/*.h")
add_executable(main ${headers} ${sources})
find_library(chronolib NAMES ChronoEngine PATHS ${chronodir}/lib)
find_library(chronoirrlichtlib NAMES ChronoEngine_IRRLICHT PATHS ${chronodir}/lib)
find_library(irrlichtlib NAMES Irrlicht)

target_link_libraries(main ${chronolib} ${chronoirrlichtlib} ${irrlichtlib})
