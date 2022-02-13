
px4_add_library(arch_board_flash
        flash.cpp)
target_link_libraries(arch_board_flash
        PRIVATE
        hardware_regs
        hardware_flash)