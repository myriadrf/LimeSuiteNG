add_library(lms_gfir STATIC corrections.c gfir_lms.c lms.c recipes.c rounding.c)
set_property(TARGET lms_gfir PROPERTY POSITION_INDEPENDENT_CODE TRUE)

if(NOT BUILD_SHARED_LIBS)
    install(
        TARGETS lms_gfir
        EXPORT limesuitengTarget
        LIBRARY DESTINATION lib${LIB_SUFFIX} # .so file
        ARCHIVE DESTINATION lib${LIB_SUFFIX} # .lib file
        RUNTIME DESTINATION bin # .dll file
    )
endif()

set_target_properties(lms_gfir PROPERTIES OUTPUT_NAME "limesuiteng-embedded-gfir")
