cpack_add_component(
    runtime
    DISPLAY_NAME "Runtime libraries"
    DESCRIPTION "Runtime libraries"
    REQUIRED)

cpack_add_component(
    dev
    DISPLAY_NAME "Headers & development files"
    DESCRIPTION "Headers & development files"
    DEPENDS runtime)

cpack_add_component(
    cli
    DISPLAY_NAME "Command Line Interface tools"
    DESCRIPTION "Command Line Interface tools"
    DEPENDS runtime)

cpack_add_component(
    gui
    DISPLAY_NAME "LimeSDR hardware Graphical User Interface"
    DESCRIPTION "LimeSDR hardware Graphical User Interface"
    DEPENDS runtime)

cpack_add_component(
    legacyapi
    DISPLAY_NAME "Legacy API wrapper"
    DEPENDS runtime)

