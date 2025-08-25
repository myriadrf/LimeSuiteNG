target_sources(limesuiteng PRIVATE LimeSDR_Micro.cpp LimeSDR_Micro.cpp LA9310_TRX.cpp)


target_include_directories(limesuiteng PRIVATE common_headers)

# target_link_libraries(limesuiteng PRIVATE iqplayer)
# target_link_libraries(limesuiteng PRIVATE imx8-host)