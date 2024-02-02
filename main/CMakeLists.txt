idf_component_register(
    SRCS
    "main.c"
    "pm1006.c"
    INCLUDE_DIRS "."
)

add_definitions(-DPM1006_TEST)
