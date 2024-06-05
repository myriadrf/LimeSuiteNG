add_library(lms_gfir STATIC corrections.c gfir_lms.c lms.c recipes.c rounding.c)
set_property(TARGET lms_gfir PROPERTY POSITION_INDEPENDENT_CODE TRUE)
