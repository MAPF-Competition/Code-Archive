add_subdirectory(Catch2)

set(TESTSOURCES test_core)

FOREACH(TESTNAME IN LISTS TESTSOURCES)
    add_executable(${TESTNAME} ${TESTNAME}.cpp)
    target_link_libraries(${TESTNAME} PRIVATE Catch2::Catch2WithMain lifelonglib)
    add_test(NAME ${TESTNAME} COMMAND ${TESTNAME})
ENDFOREACH()

