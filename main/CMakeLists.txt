set(nn_objs
    "lenet_mnist.o"
)

if(NOT CMAKE_BUILD_EARLY_EXPANSION)
    set_source_files_properties(
        ${nn_objs}
        PROPERTIES
        EXTERNAL_OBJECT true
        GENERATED true
    )
endif()

set(main_srcs
    "main.c" "file_server.c" "mount.c"
    "img_nn.cpp" "PNGdec.cpp" "inflate.c" "zutil.c" "crc32.c" "inftrees.c" "inffast.c" "adler32.c"
    ${nn_objs}
)

idf_component_register(SRCS ${main_srcs}
                    INCLUDE_DIRS "."
                    EMBED_FILES "favicon.ico" "upload_script.html")
