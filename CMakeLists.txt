file(GLOB_RECURSE  srcs
    "*.h"
    "*.c*"
)
 
idf_component_register(SRCS  "${srcs}"
                       INCLUDE_DIRS "." )

