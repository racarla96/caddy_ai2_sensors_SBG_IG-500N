cmake_minimum_required(VERSION 3.10)
project(sbgCom VERSION 1.0.0 LANGUAGES C)

# Opciones de compilación
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED True)

# Definir la opción SBG_PLATFORM_ENDIANNESS
set(SBG_PLATFORM_ENDIANNESS "LITTLE" CACHE STRING "Set endianness (LITTLE or BIG)")
set_property(CACHE SBG_PLATFORM_ENDIANNESS PROPERTY STRINGS LITTLE BIG)

# Añadir la definición del preprocesador según el valor de SBG_PLATFORM_ENDIANNESS
if(SBG_PLATFORM_ENDIANNESS STREQUAL "LITTLE")
    add_definitions(-DSBG_PLATFORM_LITTLE_ENDIAN)
elseif(SBG_PLATFORM_ENDIANNESS STREQUAL "BIG")
    add_definitions(-DSBG_PLATFORM_BIG_ENDIAN)
else()
    message(FATAL_ERROR "Unknown endianness type: ${SBG_PLATFORM_ENDIANNESS}")
endif()

# Incluye los directorios de headers
include_directories(src/comWrapper src/protocol src/protocol/extDevices src/time)

# Listar explícitamente los archivos fuente
set(COM_WRAPPER_SRC
    src/comWrapper/comSerialUnix.c
)

set(PROTOCOL_SRC
    src/protocol/extDevices/extIg.c
    src/protocol/extDevices/extNmea.c
    src/protocol/commands.c
    src/protocol/commandsCalib.c
    src/protocol/commandsExt.c
    src/protocol/commandsFilter.c
    src/protocol/commandsIg30.c
    src/protocol/commandsNav.c
    src/protocol/commandsOdo.c
    src/protocol/commandsOrientation.c
    src/protocol/commandsOutput.c
    src/protocol/commandsSync.c
    src/protocol/protocol.c
    src/protocol/protocolOutput.c
    src/protocol/protocolOutputMode.c
)

set(TIME_SRC
    src/time/sbgTime.c
)

set(SBG_COM_SRC
    src/sbgCom.c
    ${COM_WRAPPER_SRC}
    ${PROTOCOL_SRC}
    ${TIME_SRC}
)

# Añade el objetivo de la librería
#add_library(sbgCom SHARED ${SBG_COM_SRC})
add_library(sbgCom STATIC ${SBG_COM_SRC})

# Configura la instalación
install(TARGETS sbgCom
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

install(DIRECTORY src/comWrapper/
        DESTINATION include/sbgCom/comWrapper
        FILES_MATCHING PATTERN "*.h")

install(DIRECTORY src/protocol/
        DESTINATION include/sbgCom/protocol
        FILES_MATCHING PATTERN "*.h")

install(DIRECTORY src/protocol/extDevices/
        DESTINATION include/sbgCom/protocol/extDevices
        FILES_MATCHING PATTERN "*.h")

install(DIRECTORY src/time/
        DESTINATION include/sbgCom/time
        FILES_MATCHING PATTERN "*.h")

install(FILES src/sbgCom.h src/sbgCommon.h src/sbgComVersion.h
        DESTINATION include/sbgCom)

# Configura el archivo de configuración de CMake para encontrar la librería
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/sbgComConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion
)

configure_file(sbgComConfig.cmake.in
               "${CMAKE_CURRENT_BINARY_DIR}/sbgComConfig.cmake"
               @ONLY)

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/sbgComConfig.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/sbgComConfigVersion.cmake"
  DESTINATION lib/cmake/sbgCom
)
