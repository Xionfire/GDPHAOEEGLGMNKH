cmake_minimum_required(VERSION 3.15)
project(MonProjetCpp LANGUAGES CXX VERSION 1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

file(GLOB_RECURSE SRC_FILES ${CMAKE_SOURCE_DIR}/src/*.cpp)

if (SRC_FILES)
    message(STATUS "Compilation de Custom en STATIC (cpp trouvés)")
    add_library(Custom STATIC ${SRC_FILES})
    target_include_directories(Custom PUBLIC include)
else()
    message(STATUS "Aucun .cpp dans src/ → Custom devient header-only")
    add_library(Custom INTERFACE)
    target_include_directories(Custom INTERFACE include)
endif()

add_library(HeaderOnly INTERFACE)
target_include_directories(HeaderOnly INTERFACE include)

file(GLOB APP_SOURCES apps/*.cpp)

message(STATUS "Apps à compiler :")
foreach(APP_FILE ${APP_SOURCES})
    message(STATUS "  ${APP_FILE}")
endforeach()

foreach(APP_FILE ${APP_SOURCES})
    get_filename_component(APP_NAME ${APP_FILE} NAME_WE)

    add_executable(${APP_NAME} ${APP_FILE})

    target_link_libraries(${APP_NAME}
        PRIVATE
            Custom
            HeaderOnly
    )
endforeach()
