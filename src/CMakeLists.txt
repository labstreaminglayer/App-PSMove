SET(PSMoveLSL_SRC)
LIST(APPEND PSMoveLSL_SRC
    ${CMAKE_CURRENT_LIST_DIR}/main.cpp
    ${CMAKE_CURRENT_LIST_DIR}/mainwindow.cpp
    ${CMAKE_CURRENT_LIST_DIR}/mainwindow.h
    ${CMAKE_CURRENT_LIST_DIR}/mainwindow.ui
	${CMAKE_CURRENT_LIST_DIR}/psmovethread.cpp
    ${CMAKE_CURRENT_LIST_DIR}/psmovethread.h
)

add_executable(PSMoveLSL ${PSMoveLSL_SRC})
target_include_directories(PSMoveLSL
    PRIVATE
        ${PSM_INCLUDE_DIR}
)

target_link_libraries(PSMoveLSL
    PRIVATE
        Qt5::Core
        Qt5::Gui
        Qt5::Widgets
        Qt5::Xml
        LSL::lsl
        ${PSM_LIBRARIES}
)

add_custom_command(TARGET PSMoveLSL POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${PSM_BINARIES_DIR}"
        $<TARGET_FILE_DIR:PSMoveLSL>
)
# TODO: 
# installLSLApp(${target})
# Until then, manually copy Qt dlls and LSL dlls into the build/install dir.
