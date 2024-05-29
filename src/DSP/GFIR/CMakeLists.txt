set(lms_gfir_src_files corrections.c gfir_lms.c lms.c recipes.c rounding.c)

target_sources(${MAIN_LIBRARY_NAME} PRIVATE ${lms_gfir_src_files})
