cmake_minimum_required(VERSION 3.15)
project(blinkd LANGUAGES C)

# Global setup
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Option: Enable debug output
option(BLINKD_DEBUG "Enable blinkd debug output" OFF)

if(BLINKD_DEBUG)
    add_compile_definitions(BLINKD_DEBUG)
    message(STATUS "[OK] blinkd debug output enabled")
endif()

# Include header path
include_directories(${CMAKE_SOURCE_DIR}/include)

# SDK Library (blinkd)
add_library(blinkd STATIC
    src/engine/blinkd.c
)

# Example executable (main.c)
add_executable(blinkdctl
    examples/c/main.c
)

target_link_libraries(blinkdctl PRIVATE blinkd)

# Platform-specific linking
if(WIN32)
    target_link_libraries(blinkdctl PRIVATE ws2_32)
else()
    target_link_libraries(blinkdctl PRIVATE m)
endif()

# Install targets
install(TARGETS blinkd blinkdctl
    RUNTIME DESTINATION bin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

install(DIRECTORY include/ DESTINATION include)
