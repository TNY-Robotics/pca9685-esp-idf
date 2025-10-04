/*
 * PCA9685 ESP-IDF Driver by TNY Robotics
 *
 * SPDX-FileCopyrightText: 2025 TNY Robotics
 * SPDX-License-Identifier: MIT
 * 
 * 
 * Copyright (C) 2025 TNY Robotics
 * 
 * This file is part of the PCA9685 ESP-IDF Driver.
 * 
 * License: MIT
 * Repository: https://github.com/tny-robotics/pca9685
 * 
 * Author: TNY Robotics
 * Date: 19/06/2025
 * Version: 1.0
 */

#include <freertos/FreeRTOS.h>
#include "pca9685.h"

#define I2C_SDA_GPIO GPIO_NUM_21
#define I2C_SCL_GPIO GPIO_NUM_47
#define I2C_HOST I2C_NUM_0

#ifdef __cplusplus
extern "C"
#endif
void app_main(void)
{
    /* I2C CONFIGURATION */

    // i2c bus configuration
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_HOST,               // I2C port number
        .sda_io_num = I2C_SDA_GPIO,         // GPIO number for I2C sda signal
        .scl_io_num = I2C_SCL_GPIO,         // GPIO number for I2C scl signal
        .clk_source = I2C_CLK_SRC_DEFAULT,  // I2C clock source, just use the default
        .glitch_ignore_cnt = 7,             // glitch filter, again, just use the default
        .flags = {
            .enable_internal_pullup = true, // enable internal pullup resistors (oled screen does not have one)
        },
    };

    // Create the i2c bus handle
    i2c_master_bus_handle_t i2c_bus_handle = NULL;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus_handle));
}