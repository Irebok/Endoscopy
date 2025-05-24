# Definir los archivos fuente
set(files
    FusionAhrs.c
    FusionCompass.c
    FusionOffset.c
    FusionCalibration.c
    FusionMath.c
)

# Crear una librería compartida (DLL)
add_library(Fusion SHARED ${files})

# Definir la macro para exportar las funciones de la DLL
target_compile_definitions(Fusion PRIVATE FUSION_EXPORTS)

# Especificar las propiedades del target
target_include_directories(Fusion PRIVATE
    ${CMAKE_SOURCE_DIR}  # Directorio actual para incluir los archivos .h
)

# Especificar si estamos en un sistema UNIX, excluyendo macOS
if(UNIX AND NOT APPLE)
    target_link_libraries(Fusion m)  # Enlazar con la librería matemática en Unix
endif()

# En Windows, se establece la definición para exportar símbolos
if(WIN32)
    set_target_properties(Fusion PROPERTIES
        DEFINE_SYMBOL FUSION_EXPORTS  # Para exportar funciones
        OUTPUT_NAME Fusion        # Nombre de la DLL de salida
    )
endif()

