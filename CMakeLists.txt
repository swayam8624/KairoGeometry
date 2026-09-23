cmake_minimum_required(VERSION 3.28)
project(KairoGeometry LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(KAIRO_GEOMETRY_BUILD_TESTS "Build KairoGeometry unit and smoke tests" ${PROJECT_IS_TOP_LEVEL})
option(KAIRO_GEOMETRY_BUILD_VISUALIZER "Build the standalone KairoGeometry visualizer" ${PROJECT_IS_TOP_LEVEL})

# Standalone consumers fetch the same KairoMath revision validated by the
# umbrella. Local source directories remain the preferred development path.
set(KAIRO_GEOMETRY_MATH_REVISION ea2945fe36ca97d665d3deeb9ee933ea8b978fc9)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang" OR CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    add_compile_options(-Wall -Wextra -Wno-unused-parameter)
elseif(MSVC)
    add_compile_options(/W4)
endif()

if(NOT TARGET KairoMath)
    set(KAIRO_MATH_SOURCE_DIR "" CACHE PATH "Path to a local KairoMath checkout")

    if(KAIRO_MATH_SOURCE_DIR AND EXISTS "${KAIRO_MATH_SOURCE_DIR}/CMakeLists.txt")
        add_subdirectory(
            "${KAIRO_MATH_SOURCE_DIR}"
            "${CMAKE_CURRENT_BINARY_DIR}/_deps/KairoMath"
        )
    elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../KairoMath/CMakeLists.txt")
        add_subdirectory(
            "${CMAKE_CURRENT_SOURCE_DIR}/../KairoMath"
            "${CMAKE_CURRENT_BINARY_DIR}/_deps/KairoMath"
        )
    else()
        include(FetchContent)
        FetchContent_Declare(
            KairoMath
            GIT_REPOSITORY https://github.com/swayam8624/KairoMath.git
            GIT_TAG ${KAIRO_GEOMETRY_MATH_REVISION}
        )
        FetchContent_MakeAvailable(KairoMath)
    endif()
endif()

add_library(KairoGeometry)

target_sources(KairoGeometry
    PUBLIC
        FILE_SET CXX_MODULES
        BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}
        FILES
            Ray.cppm
            Line.cppm
            Segment.cppm
            Plane.cppm
            Sphere.cppm
            Triangle.cppm
            AABB.cppm
            Capsule.cppm
            OBB.cppm
            Frustum.cppm
            ClosestPoint.cppm
            Distance.cppm
            Intersection.cppm
            KairoGeometry.cppm
)

target_link_libraries(KairoGeometry PUBLIC KairoMath)

if(KAIRO_GEOMETRY_BUILD_TESTS)
    add_executable(KairoGeometrySmoke tests/smoke.cpp)
    target_link_libraries(KairoGeometrySmoke PRIVATE KairoGeometry)

    add_executable(KairoGeometryTests tests/GeometryTests.cpp)
    target_link_libraries(KairoGeometryTests PRIVATE KairoGeometry)

    add_executable(KairoGeometryCertification tests/CertificationTests.cpp)
    target_link_libraries(KairoGeometryCertification PRIVATE KairoGeometry)

    enable_testing()
    add_test(NAME KairoGeometrySmoke COMMAND KairoGeometrySmoke)
    add_test(NAME KairoGeometryTests COMMAND KairoGeometryTests)
    add_test(NAME KairoGeometryCertification COMMAND KairoGeometryCertification)
endif()

if(KAIRO_GEOMETRY_BUILD_VISUALIZER)
    add_executable(KairoGeometryVisual visual/VisualTests.cpp)
    target_link_libraries(KairoGeometryVisual PRIVATE KairoGeometry)
endif()
