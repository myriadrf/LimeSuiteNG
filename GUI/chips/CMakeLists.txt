add_subdirectory(ADF4002)
add_subdirectory(CDCM6208)
add_subdirectory(Si5351C)
add_subdirectory(LMS7002M)
add_subdirectory(LMS8001)

if(UNIX AND NOT APPLE)
    add_subdirectory(LA9310)
endif()
