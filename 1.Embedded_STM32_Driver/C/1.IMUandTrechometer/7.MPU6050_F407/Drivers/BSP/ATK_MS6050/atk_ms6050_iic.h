/**
 ****************************************************************************************************
 * @file        atk_ms6050_iic.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2022-06-21
 * @brief       ATK-MS6050模块IIC接口驱动代码
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

#ifndef __ATK_MS6050_IIC_H
#define __ATK_MS6050_IIC_H

#include "./SYSTEM/sys/sys.h"

/* 引脚定义 */
#define ATK_MS6050_IIC_SCL_GPIO_PORT            GPIOB
#define ATK_MS6050_IIC_SCL_GPIO_PIN             GPIO_PIN_10
#define ATK_MS6050_IIC_SCL_GPIO_CLK_ENABLE()    do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)
#define ATK_MS6050_IIC_SDA_GPIO_PORT            GPIOB
#define ATK_MS6050_IIC_SDA_GPIO_PIN             GPIO_PIN_11
#define ATK_MS6050_IIC_SDA_GPIO_CLK_ENABLE()    do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)

/* IO操作 */
#define ATK_MS6050_IIC_SCL(x)                   do{ x ?                                                                                             \
                                                    HAL_GPIO_WritePin(ATK_MS6050_IIC_SCL_GPIO_PORT, ATK_MS6050_IIC_SCL_GPIO_PIN, GPIO_PIN_SET) :    \
                                                    HAL_GPIO_WritePin(ATK_MS6050_IIC_SCL_GPIO_PORT, ATK_MS6050_IIC_SCL_GPIO_PIN, GPIO_PIN_RESET);   \
                                                }while(0)

#define ATK_MS6050_IIC_SDA(x)                   do{ x ?                                                                                             \
                                                    HAL_GPIO_WritePin(ATK_MS6050_IIC_SDA_GPIO_PORT, ATK_MS6050_IIC_SDA_GPIO_PIN, GPIO_PIN_SET) :    \
                                                    HAL_GPIO_WritePin(ATK_MS6050_IIC_SDA_GPIO_PORT, ATK_MS6050_IIC_SDA_GPIO_PIN, GPIO_PIN_RESET);   \
                                                }while(0)

#define ATK_MS6050_IIC_READ_SDA()               HAL_GPIO_ReadPin(ATK_MS6050_IIC_SDA_GPIO_PORT, ATK_MS6050_IIC_SDA_GPIO_PIN)

/* 操作函数 */
void atk_ms6050_iic_start(void);                /* 产生IIC起始信号 */
void atk_ms6050_iic_stop(void);                 /* 产生IIC停止信号 */
uint8_t atk_ms6050_iic_wait_ack(void);          /* 等待IIC应答信号 */
void atk_ms6050_iic_ack(void);                  /* 产生ACK应答信号 */
void atk_ms6050_iic_nack(void);                 /* 不产生ACK应答信号 */
void atk_ms6050_iic_send_byte(uint8_t dat);     /* IIC发送一个字节 */
uint8_t atk_ms6050_iic_read_byte(uint8_t ack);  /* IIC接收一个字节 */
void atk_ms6050_iic_init(void);                 /* 初始化IIC接口 */

#endif
