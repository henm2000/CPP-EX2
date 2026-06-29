# Component + integration test suite for drone_mapper.
# Filled in fully in step 18 of PLAN.md once components exist.

find_package(GTest CONFIG REQUIRED)

set(DRONE_MAPPER_TEST_SOURCES)
file(GLOB DRONE_MAPPER_COMPONENT_TEST_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/components/*.cpp")
file(GLOB DRONE_MAPPER_INTEGRATION_TEST_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/integration/*.cpp")
list(APPEND DRONE_MAPPER_TEST_SOURCES
    ${DRONE_MAPPER_COMPONENT_TEST_SOURCES}
    ${DRONE_MAPPER_INTEGRATION_TEST_SOURCES})

if(DRONE_MAPPER_TEST_SOURCES)
    add_executable(drone_mapper_simulation_test ${DRONE_MAPPER_TEST_SOURCES})
    target_link_libraries(drone_mapper_simulation_test
        PRIVATE
            drone_mapper
            GTest::gtest_main
            GTest::gmock
    )
    target_compile_features(drone_mapper_simulation_test PRIVATE cxx_std_20)
    target_include_directories(drone_mapper_simulation_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    # Integration tests locate the shipped maps via this absolute path.
    target_compile_definitions(drone_mapper_simulation_test
        PRIVATE DRONE_MAPPER_DATA_DIR="${CMAKE_SOURCE_DIR}/data_maps")
    include(GoogleTest)
    gtest_discover_tests(drone_mapper_simulation_test)
endif()
