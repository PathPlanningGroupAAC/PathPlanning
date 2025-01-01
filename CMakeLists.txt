cmake_minimum_required(VERSION 3.13.4)

project(Bounds)

set(Python3_USE_DEBUG_LIBRARIES FALSE)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

# Aggiungi il file principale per la compilazione
file(GLOB_RECURSE SOURCES
    "${CMAKE_SOURCE_DIR}/*.cpp"
    "${CMAKE_SOURCE_DIR}/*.h"
)

foreach(FILE ${SOURCES})
    string(FIND ${FILE} "${CMAKE_BINARY_DIR}" IS_IN_BUILD)
    string(FIND ${FILE} "${CMAKE_SOURCE_DIR}/vendor" IS_IN_VENDOR)
    if(IS_IN_BUILD GREATER -1 OR IS_IN_VENDOR GREATER -1)
        list(REMOVE_ITEM SOURCES ${FILE})
    endif()
endforeach()

add_executable(Bounds ${SOURCES})

# Aggiungi le librerie esterne
include_directories(vendor/glm)
add_subdirectory(vendor/matplotlib-cpp)

if(UNIX)
    if(EXISTS "$ENV{HOME}/.local/lib/python3.${Python3_VERSION_MINOR}/site-packages")
    set(Python3_SITELIB "$ENV{HOME}/.local/lib/python3.${Python3_VERSION_MINOR}/site-packages")
    endif()

    target_link_libraries( Bounds PRIVATE
        Python3::Python
        npymath.a
    )
elseif(WIN32)
    target_link_libraries( Bounds PRIVATE
        Python3::Python
        npymath.lib
    )
endif()

message(${Python3_RUNTIME_LIBRARY_DIRS}/libs)

# Aggiungi le directory di inclusione
target_include_directories(Bounds PRIVATE
    ${Python3_INCLUDE_DIRS} # Python include directory
    ${Python3_SITELIB}/numpy/_core/include
    vendor/matplotlib-cpp # Include di matplotlib-cpp
)

target_link_directories(Bounds PRIVATE
    ${Python3_RUNTIME_LIBRARY_DIRS}/libs
    ${Python3_SITELIB}/numpy/_core/lib
)
