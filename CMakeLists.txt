idf_component_register(SRCS "BT_CLASSIC.c" "spp_task.c"
                    INCLUDE_DIRS "include"
                    REQUIRES bt driver nvs_flash)
