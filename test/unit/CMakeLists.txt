set(test_sources
    main.tst.cpp
    app.tst.cpp
)
set(TEST_EXEC ${PROJECT_NAME}_unit_tests)

add_executable(${TEST_EXEC} ${test_sources})
target_link_libraries(${TEST_EXEC}
    PRIVATE
    ${PROJECT_NAME}
    gmock
    gtest
)

setup_target_for_coverage_gcovr_html(
    NAME ${PROJECT_NAME}_coverage
    EXECUTABLE ${TEST_EXEC}

    EXCLUDE
    ${PROJECT_SOURCE_DIR}/dep/internal/ipc
    ${PROJECT_SOURCE_DIR}/dep/external
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PROJECT_BINARY_DIR}
)
