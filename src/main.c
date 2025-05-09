/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

 #include <zephyr/kernel.h>
 #include <zephyr/device.h>
 #include <zephyr/devicetree.h>
 /* STEP 3 - Include the header file of the I2C API */
 #include <zephyr/drivers/i2c.h>
 /* STEP 4.1 - Include the header file of printk() */
 #include <zephyr/sys/printK.h>
 /* 1000 msec = 1 sec */
 #define SLEEP_TIME_MS 1000
 
 /* STEP 8 - Define the I2C slave device address and the addresses of relevant registers */
 
#define MPU6050_ADDR       0x68
#define MPU6050_REG_PWR    0x6B
#define MPU6050_REG_ACCELX 0x3B

#define WHO_AM_I_REG 0x75

 /* STEP 6 - Get the node identifier of the sensor */
 #define I2C0_NODE DT_NODELABEL(mpu6050) 

 
 int main(void)
 {
 
        int ret;
        uint8_t data[6];
         /* STEP 7 - Retrieve the API-specific device structure and make sure that the device is
          * ready to use  */
         static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C0_NODE);
         printk("I2C bus %s ready!\n\r",dev_i2c.bus->name);
         if (!device_is_ready(dev_i2c.bus)) {
                return -1;
         }

          uint8_t reg = WHO_AM_I_REG;
          uint8_t val = 0;
      
          /* Send command to read id mpu6050*/
          if (i2c_write_dt(&dev_i2c, &reg, 1)) {
              printk("Lỗi ghi I2C\n");
              return -1;
          }
      
          /* Read id mpu6050 */ 
          if (i2c_read_dt(&dev_i2c, &val, 1)) {
              printk("Lỗi đọc I2C\n");
              return -1;
          }
          printk("ID of mpu6050 is 0x%x \n",val);
  
        /* 1. Wake up MPU6050 (clear sleep bit) */
        uint8_t pwr_cfg[2] = {MPU6050_REG_PWR, 0x00};
        ret = i2c_write_dt(&dev_i2c, pwr_cfg, 2);
        if (ret) {
                printk("Failed to write PWR_MGMT_1\n");
                return -1;
        }

        uint8_t cfg[2] = {0x1C, 0x08};
        ret = i2c_write_dt(&dev_i2c, cfg, 2);
        if (ret) {
                printk("Failed to write\n");
                return -1;
        }
        
        k_msleep(100);  // wait stability
 
        while (1) {
                 // 2. Read Accel X, Y, Z (6 bytes)
                ret = i2c_write_read_dt(&dev_i2c,
                        &(uint8_t){MPU6050_REG_ACCELX}, 1,
                        data, 6);
                if (ret) {
                printk("Failed to read accel data\n");
                continue;
                }

                int16_t ax = (data[0] << 8) | data[1];
                int16_t ay = (data[2] << 8) | data[3];
                int16_t az = (data[4] << 8) | data[5];

                printk("Accel X: %d, Y: %d, Z: %d\n", ax, ay, az);
                k_msleep(SLEEP_TIME_MS);
         }
 }

