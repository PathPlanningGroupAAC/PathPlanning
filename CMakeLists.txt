cmake_minimum_required(VERSION 3.13.4)

project(Bounds)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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
    )
elseif(WIN32)
    if(MSVC)
		set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /NODEFAULTLIB:python313_d.lib")
    else()
        message("You will need a debug library of python!")
	endif()

    target_link_libraries( Bounds PRIVATE
        Python3::Python
    )
endif()

# Aggiungi le directory di inclusione
target_include_directories(Bounds PRIVATE
    ${Python3_INCLUDE_DIRS} # Python include directory
    vendor/matplotlib-cpp # Include di matplotlib-cpp
)

target_link_directories(Bounds PRIVATE
    ${Python3_RUNTIME_LIBRARY_DIRS}/libs
)

if(EXISTS "${CMAKE_SOURCE_DIR}/m_in.csv")
    add_custom_command(TARGET Bounds POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy
                ${CMAKE_BINARY_DIR}/m_in.csv
                $<TARGET_FILE_DIR:Bounds>/m_in.csv
    )
else()
    message(WARNING "File m_in.csv not found, skipping copy.")
endif()

if(EXISTS "${CMAKE_SOURCE_DIR}/m_out.csv")
    add_custom_command(TARGET Bounds POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy
                ${CMAKE_BINARY_DIR}/m_out.csv
                $<TARGET_FILE_DIR:Bounds>/m_out.csv
    )
else()
    message(WARNING "File m_out.csv not found, skipping copy.")
endif()
