set(LMS8001_SOURCES LMS8001.cpp LMS8001_parameters.cpp LMS8001_RegistersMap.cpp)

target_sources(limesuiteng PRIVATE ${LMS8001_SOURCES})

# target_include_directories(limesuiteng PRIVATE ${PROJECT_SOURCE_DIR}/embedded/include)
