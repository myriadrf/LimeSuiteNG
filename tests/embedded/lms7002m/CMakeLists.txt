target_sources(${TESTS_CONTAINER_TARGET} PRIVATE lms7002m_embedded_tests.cpp LOprecision_tests.cpp)
target_include_directories(${TESTS_CONTAINER_TARGET} PRIVATE)

# no need to actualy link, just getting include directories
target_link_libraries(${TESTS_CONTAINER_TARGET} PRIVATE lms7002m)
