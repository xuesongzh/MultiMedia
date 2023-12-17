cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(FFmpegTutorial LANGUAGES CXX)

if(CMAKE_BUILD_TYPE STREQUAL Debug)
    message(STATUS "Debug Mode.")
    add_definitions(-DDebug)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -o0 -g")
else()
    message(STATUS "Release Mode.")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -o2")
endif()

set(ffmpeg_include_dir ${PROJECT_SOURCE_DIR}/../Output/include)
include_directories(${ffmpeg_include_dir})

set(ffmpeg_lib_dir ${PROJECT_SOURCE_DIR}/../Output/lib)
file(GLOB ffmpeg_dylibs ${ffmpeg_lib_dir}/*.dylib)
link_directories(${ffmpeg_lib_dir})

include_directories(${PROJECT_SOURCE_DIR}/inc)

set(demo_dir ${PROJECT_SOURCE_DIR}/demo)
file(GLOB demo_codes ${demo_dir}/*.cpp)

set(core_dir ${PROJECT_SOURCE_DIR}/src)
file(GLOB core_codes ${core_dir}/*.cpp)

foreach (demo ${demo_codes})
    get_filename_component(demo_basename ${demo} NAME_WE)
    add_executable(${demo_basename} ${demo} ${core_codes} ${src_codes})
    target_link_libraries(${demo_basename} ${ffmpeg_dylibs})
endforeach()