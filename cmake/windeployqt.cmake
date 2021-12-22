# https://stackoverflow.com/questions/60854495/qt5-cmake-include-all-libraries-into-executable
find_package(Qt6 COMPONENTS Core REQUIRED)

# get absolute path to qmake, then use it to find windeployqt executable

get_target_property(_qmake_executable Qt6::qmake IMPORTED_LOCATION)
get_filename_component(_qt_bin_dir "${_qmake_executable}" DIRECTORY)
get_filename_component(_qt_windeployqt_dir "${_qmake_executable}" DIRECTORY)

find_program(WINDEPLOYQT_EXECUTABLE windeployqt HINTS "${_qt_bin_dir}")

function(windeployqt target)
    # POST_BUILD step
    # - after build, we have a bin/lib for analyzing qt dependencies
    # - we run windeployqt on target and deploy Qt libs
    IF(CMAKE_BUILD_TYPE MATCHES "Debug")
        find_program(WINDEPLOYQT_DEBUG windeployqt.debug.bat HINTS "${_qt_bin_dir}")
        if(WINDEPLOYQT_DEBUG)
          set(WINDEPLOYQT_EXECUTABLE = ${WINDEPLOYQT_DEBUG})
        endif()
        message(WINDEPLOYQT_EXECUTABLE)
        add_custom_command(TARGET ${target} POST_BUILD
            COMMAND ${WINDEPLOYQT_EXECUTABLE}        
                --verbose 1
                --debug
                \"$<TARGET_FILE:${target}>\"
            COMMENT "Deploying Qt libraries using windeployqt for compilation target '${target}' ..."
        )
    ELSE()
        add_custom_command(TARGET ${target} POST_BUILD
            COMMAND ${WINDEPLOYQT_EXECUTABLE}        
                --verbose 1
                --release
                \"$<TARGET_FILE:${target}>\"
            COMMENT "Deploying Qt libraries using windeployqt for compilation target '${target}' ..."
        )
    ENDIF()
    


endfunction()