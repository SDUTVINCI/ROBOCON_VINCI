/**
 ****************************************************************************************************
 * @file        atk_ms6050.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2022-06-21
 * @brief       ATK-MS6050模块驱动代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 探索者 F407开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 ****************************************************************************************************
 */

#ifndef __ATK_MS6050_H
#define __ATK_MS6050_H

#include "./SYSTEM/sys/sys.h"

/* 引脚定义 */
#define ATK_MS6050_AD0_GPIO_PORT            GPIOC
#define ATK_MS6050_AD0_GPIO_PIN             GPIO_PIN_0
#define ATK_MS6050_AD0_GPIO_CLK_ENABLE()    do{ __HAL_RCC_GPIOC_CLK_ENABLE(); \
                                            }while(0)

/* IO操作 */
#define ATK_MS6050_AD0(x)                   do{ x ?                                                                                     \
                                                HAL_GPIO_WritePin(ATK_MS6050_AD0_GPIO_PORT, ATK_MS6050_AD0_GPIO_PIN, GPIO_PIN_SET) :    \
                                                HAL_GPIO_WritePin(ATK_MS6050_AD0_GPIO_PORT, ATK_MS6050_AD0_GPIO_PIN, GPIO_PIN_RESET);   \
                                            }while(0)

/* ATK-MS6050的IIC通讯从机地址
 * 如果ATK-MS6050的AD0引脚被拉低，则其IIC通讯的地址为0x68
 * 如果ATK-MS6050的AD0引脚被拉高，则其IIC通讯的地址为0x69
 */
#define ATK_MS6050_IIC_ADDR     0x68

/* ATK_MS6050寄存器地址定义 */
#define MPU_ACCEL_OFFS_REG      0X06    // accel_offs寄存器,可读取版本号,寄存器手册未提到
#define MPU_PROD_ID_REG         0X0C    // prod id寄存器,在寄存器手册未提到
#define MPU_SELF_TESTX_REG      0X0D    // 自检寄存器X
#define MPU_SELF_TESTY_REG      0X0E    // 自检寄存器Y
#define MPU_SELF_TESTZ_REG      0X0F    // 自检寄存器Z
#define MPU_SELF_TESTA_REG      0X10    // 自检寄存器A
#define MPU_SAMPLE_RATE_REG     0X19    // 采样频率分频器
#define MPU_CFG_REG             0X1A    // 配置寄存器
#define MPU_GYRO_CFG_REG        0X1B    // 陀螺仪配置寄存器
#define MPU_ACCEL_CFG_REG       0X1C    // 加速度计配置寄存器
#define MPU_MOTION_DET_REG      0X1F    // 运动检测阀值设置寄存器
#define MPU_FIFO_EN_REG         0X23    // FIFO使能寄存器
#define MPU_I2CMST_CTRL_REG     0X24    // IIC主机控制寄存器
#define MPU_I2CSLV0_ADDR_REG    0X25    // IIC从机0器件地址寄存器
#define MPU_I2CSLV0_REG         0X26    // IIC从机0数据地址寄存器
#define MPU_I2CSLV0_CTRL_REG    0X27    // IIC从机0控制寄存器
#define MPU_I2CSLV1_ADDR_REG    0X28    // IIC从机1器件地址寄存器
#define MPU_I2CSLV1_REG         0X29    // IIC从机1数据地址寄存器
#define MPU_I2CSLV1_CTRL_REG    0X2A    // IIC从机1控制寄存器
#define MPU_I2CSLV2_ADDR_REG    0X2B    // IIC从机2器件地址寄存器
#define MPU_I2CSLV2_REG         0X2C    // IIC从机2数据地址寄存器
#define MPU_I2CSLV2_CTRL_REG    0X2D    // IIC从机2控制寄存器
#define MPU_I2CSLV3_ADDR_REG    0X2E    // IIC从机3器件地址寄存器
#define MPU_I2CSLV3_REG         0X2F    // IIC从机3数据地址寄存器
#define MPU_I2CSLV3_CTRL_REG    0X30    // IIC从机3控制寄存器
#define MPU_I2CSLV4_ADDR_REG    0X31    // IIC从机4器件地址寄存器
#define MPU_I2CSLV4_REG         0X32    // IIC从机4数据地址寄存器
#define MPU_I2CSLV4_DO_REG      0X33    // IIC从机4写数据寄存器
#define MPU_I2CSLV4_CTRL_REG    0X34    // IIC从机4控制寄存器
#define MPU_I2CSLV4_DI_REG      0X35    // IIC从机4读数据寄存器
#define MPU_I2CMST_STA_REG      0X36    // IIC主机状态寄存器
#define MPU_INTBP_CFG_REG       0X37    // 中断/旁路设置寄存器
#define MPU_INT_EN_REG          0X38    // 中断使能寄存器
#define MPU_INT_STA_REG         0X3A    // 中断状态寄存器
#define MPU_ACCEL_XOUTH_REG     0X3B    // 加速度值,X轴高8位寄存器
#define MPU_ACCEL_XOUTL_REG     0X3C    // 加速度值,X轴低8位寄存器
#define MPU_ACCEL_YOUTH_REG     0X3D    // 加速度值,Y轴高8位寄存器
#define MPU_ACCEL_YOUTL_REG     0X3E    // 加速度值,Y轴低8位寄存器
#define MPU_ACCEL_ZOUTH_REG     0X3F    // 加速度值,Z轴高8位寄存器
#define MPU_ACCEL_ZOUTL_REG     0X40    // 加速度值,Z轴低8位寄存器
#define MPU_TEMP_OUTH_REG       0X41    // 温度值高八位寄存器
#define MPU_TEMP_OUTL_REG       0X42    // 温度值低8位寄存器
#define MPU_GYRO_XOUTH_REG      0X43    // 陀螺仪值,X轴高8位寄存器
#define MPU_GYRO_XOUTL_REG      0X44    // 陀螺仪值,X轴低8位寄存器
#define MPU_GYRO_YOUTH_REG      0X45    // 陀螺仪值,Y轴高8位寄存器
#define MPU_GYRO_YOUTL_REG      0X46    // 陀螺仪值,Y轴低8位寄存器
#define MPU_GYRO_ZOUTH_REG      0X47    // 陀螺仪值,Z轴高8位寄存器
#define MPU_GYRO_ZOUTL_REG      0X48    // 陀螺仪值,Z轴低8位寄存器
#define MPU_I2CSLV0_DO_REG      0X63    // IIC从机0数据寄存器
#define MPU_I2CSLV1_DO_REG      0X64    // IIC从机1数据寄存器
#define MPU_I2CSLV2_DO_REG      0X65    // IIC从机2数据寄存器
#define MPU_I2CSLV3_DO_REG      0X66    // IIC从机3数据寄存器
#define MPU_I2CMST_DELAY_REG    0X67    // IIC主机延时管理寄存器
#define MPU_SIGPATH_RST_REG     0X68    // 信号通道复位寄存器
#define MPU_MDETECT_CTRL_REG    0X69    // 运动检测控制寄存器
#define MPU_USER_CTRL_REG       0X6A    // 用户控制寄存器
#define MPU_PWR_MGMT1_REG       0X6B    // 电源管理寄存器1
#define MPU_PWR_MGMT2_REG       0X6C    // 电源管理寄存器2 
#define MPU_FIFO_CNTH_REG       0X72    // FIFO计数寄存器高八位
#define MPU_FIFO_CNTL_REG       0X73    // FIFO计数寄存器低八位
#define MPU_FIFO_RW_REG         0X74    // FIFO读写寄存器
#define MPU_DEVICE_ID_REG       0X75    // 器件ID寄存器

/* 函数错误代码 */
#define ATK_MS6050_EOK      0   /* 没有错误 */
#define ATK_MS6050_EID      1   /* ID错误 */
#define ATK_MS6050_EACK     2   /* IIC通讯ACK错误 */

/* 操作函数 */
uint8_t atk_ms6050_write(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *dat); /* 往ATK-MS6050的指定寄存器连续写入指定数据 */
uint8_t atk_ms6050_write_byte(uint8_t addr, uint8_t reg, uint8_t dat);          /* 往ATK-MS6050的指定寄存器写入一字节数据 */
uint8_t atk_ms6050_read(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *dat);  /* 连续读取ATK-MS6050指定寄存器的值 */
uint8_t atk_ms6050_read_byte(uint8_t addr, uint8_t reg, uint8_t *dat);          /* 读取ATK-MS6050指定寄存器的值 */
void atk_ms6050_sw_reset(void);                                                 /* ATK-MS6050软件复位 */
uint8_t atk_ms6050_set_gyro_fsr(uint8_t fsr);                                   /* ATK-MS6050设置陀螺仪传感器量程范围 */
uint8_t atk_ms6050_set_accel_fsr(uint8_t fsr);                                  /* ATK-MS6050设置加速度传感器量程范围 */
uint8_t atk_ms6050_set_lpf(uint16_t lpf);                                       /* ATK-MS6050设置数字低通滤波器频率 */
uint8_t atk_ms6050_set_rate(uint16_t rate);                                     /* ATK-MS6050设置采样率 */
uint8_t atk_ms6050_get_temperature(int16_t *temp);                              /* ATK-MS6050获取温度值 */
uint8_t atk_ms6050_get_gyroscope(int16_t *gx, int16_t *gy, int16_t *gz);        /* ATK-MS6050获取陀螺仪值 */
uint8_t atk_ms6050_get_accelerometer(int16_t *ax, int16_t *ay, int16_t *az);    /* ATK-MS6050获取加速度值 */
uint8_t atk_ms6050_init(void);                                                  /* ATK-MS6050初始化 */

#endif
