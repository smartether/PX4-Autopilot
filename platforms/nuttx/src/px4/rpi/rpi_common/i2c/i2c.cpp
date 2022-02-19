//
// Created by zhengweiqian on 19/2/2022.
//

#include <board.h>
#include <systemlib/px4_macros.h>
#include <px4_platform_common/spi.h>
#include <px4_platform_common/i2c.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_arch/micro_hal.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>
#include <unistd.h>

#include <nuttx/spi/spi.h>
#include <arch/board/board.h>
#include <arm_arch.h>
#include <chip.h>

#include <nuttx/config.h>
#include <stddef.h>
#include <nuttx/fs/fs.h>

int board_i2cdev_initialize(int);

int rp2040_i2cinitialize(void) {
    int ret = 0;

#ifdef CONFIG_RP2040_I2C_DRIVER
    #ifdef CONFIG_RP2040_I2C0
    ret = board_i2cdev_initialize(0);
    if (ret < 0)
      {
        _err("ERROR: Failed to initialize I2C0.\n");
      }
#endif

#ifdef CONFIG_RP2040_I2C1
    ret = board_i2cdev_initialize(1);
    if (ret < 0)
      {
        _err("ERROR: Failed to initialize I2C1.\n");
      }
#endif
#endif
    return ret;
}

int board_i2cdev_initialize(int port){

    int ret;
    FAR struct i2c_master_s *i2c;

    i2cinfo("Initializing /dev/i2c%d..\n", port);

    /* Initialize i2c device */

    i2c = rp2040_i2cbus_initialize(port);
    if (!i2c)
    {
        i2cerr("ERROR: Failed to initialize i2c%d.\n", port);
        return -ENODEV;
    }

    ret = i2c_register(i2c, port);
    if (ret < 0)
    {
        i2cerr("ERROR: Failed to register i2c%d: %d\n", port, ret);
    }

    return ret;
}