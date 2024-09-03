include(${CMAKE_SOURCE_DIR}/cmake/pr_common.cmake)

set(PROJ_NAME util_smdmodel)
pr_add_library(${PROJ_NAME} STATIC)

pr_add_dependency(${PROJ_NAME} vfilesystem TARGET PUBLIC)

pr_init_module(${PROJ_NAME})

pr_add_compile_definitions(${PROJ_NAME} -DSMDMDL_STATIC PUBLIC)

pr_finalize(${PROJ_NAME})
