/**
 ****************************************************************************************************
 * @file        atk_ms6050_iic.c
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

#include "./BSP/ATK_MS6050/atk_ms6050_iic.h"
#include "./SYSTEM/delay/delay.h"

/**
 * @brief       IIC接口延时函数，用于控制IIC读写速度
 * @param       无
 * @retval      无
 */
static inline void atk_ms6050_iic_delay(void)
{
    delay_us(2);
}

/**
 * @brief       产生IIC起始信号
 * @param       无
 * @retval      无
 */
void atk_ms6050_iic_start(void)
{
    ATK_MS6050_IIC_SDA(1);
    ATK_MS6050_IIC_SCL(1);
    atk_ms6050_iic_delay();
    ATK_MS6050_IIC_SDA(0);
    atk_ms6050_iic_delay();
    ATK_MS6050_IIC_SCL(0);
    atk_ms6050_iic_delay();
}

/**
 * @brief       产生IIC停止信号
 * @param       无
 * @retval      无
 */
void atk_ms6050_iic_stop(void)
{
    ATK_MS6050_IIC_SDA(0);
    atk_ms6050_iic_delay();
    ATK_MS6050_IIC_SCL(1);
    atk_ms6050_iic_delay();
    ATK_MS6050_IIC_SDA(1);
    atk_ms6050_iic_delay();
}

/**
 * @brief       等待IIC应答信号
 * @param       无
 * @retval      0: 应答信号接收成功
 *              1: 应答信号接收失败
 */
uint8_t atk_ms6050_iic_wait_ack(void)
{
    uint8_t waittime = 0;
    uint8_t rack = 0;
    
    ATK_MS6050_IIC_SDA(1);
    atk_ms6050_iic_delay();
    ATK_MS6050_IIC_SCL(1);
    atk_ms6050_iic_delay();
    
    while (ATK_MS6050_IIC_READ_SDA())
    {
        waittime++;
        
        if (waittime > 250)
        {
            atk_ms6050_iic_stop();
            rack = 1;
            break;
        }
    }
    
    ATK_MS6050_IIC_SCL(0);
    atk_ms6050_iic_delay();
    
    return rack;
}

/**
 * @brief       产生ACK应答信号
 * @param       无
 * @retval      无
 */
void atk_ms6050_iic_ack(void)
{
    ATK_MS6050_IIC_SDA(0);
    atk_ms6050_iic_delay();
    ATK_MS6050_IIC_SCL(1);
    atk_ms6050_iic_delay();
    ATK_MS6050_IIC_SCL(0);
    atk_ms6050_iic_delay();
    ATK_MS6050_IIC_SDA(1);
    atk_ms6050_iic_delay();
}

/**
 * @brief       不产生ACK应答信号
 * @param       无
 * @retval      无
 */
void atk_ms6050_iic_nack(void)
{
    ATK_MS6050_IIC_SDA(1);
    atk_ms6050_iic_delay();
    ATK_MS6050_IIC_SCL(1);
    atk_ms6050_iic_delay();
    ATK_MS6050_IIC_SCL(0);
    atk_ms6050_iic_delay();
}

/**
 * @brief       IIC发送一个字节
 * @param       dat: 要发送的数据
 * @retval      无
 */
void atk_ms6050_iic_send_byte(uint8_t dat)
{
    uint8_t t;
    
    for (t=0; t<8; t++)
    {
        ATK_MS6050_IIC_SDA((dat & 0x80) >> 7);
        atk_ms6050_iic_delay();
        ATK_MS6050_IIC_SCL(1);
        atk_ms6050_iic_delay();
        ATK_MS6050_IIC_SCL(0);
        dat <<= 1;
    }
    ATK_MS6050_IIC_SDA(1);
}

/**
 * @brief       IIC接收一个字节
 * @param       ack: ack=1时，发送ack; ack=0时，发送nack
 * @retval      接收到的数据
 */
uint8_t atk_ms6050_iic_read_byte(uint8_t ack)
{
    uint8_t i;
    uint8_t dat = 0;
    
    for (i = 0; i < 8; i++ )
    {
        dat <<= 1;
        ATK_MS6050_IIC_SCL(1);
        atk_ms6050_iic_delay();
        
        if (ATK_MS6050_IIC_READ_SDA())
        {
            dat++;
        }
        
        ATK_MS6050_IIC_SCL(0);
        atk_ms6050_iic_delay();
    }
    
    if (ack == 0)
    {
        atk_ms6050_iic_nack();
    }
    else
    {
        atk_ms6050_iic_ack();
    }

    return dat;
}

/**
 * @brief       初始化IIC接口
 * @param       无
 * @retval      无
 */
void atk_ms6050_iic_init(void)
{
    GPIO_InitTypeDef gpio_init_struct = {0};
    
    /* 使能SCL、SDA引脚GPIO的时钟 */
    ATK_MS6050_IIC_SCL_GPIO_CLK_ENABLE();
    ATK_MS6050_IIC_SDA_GPIO_CLK_ENABLE();
    
    /* 初始化SCL引脚 */
    gpio_init_struct.Pin    = ATK_MS6050_IIC_SCL_GPIO_PIN;  /* SCL引脚 */
    gpio_init_struct.Mode   = GPIO_MODE_OUTPUT_PP;          /* 推挽输出 */
    gpio_init_struct.Pull   = GPIO_PULLUP;                  /* 上拉 */
    gpio_init_struct.Speed  = GPIO_SPEED_FREQ_HIGH;         /* 高速 */
    HAL_GPIO_Init(ATK_MS6050_IIC_SCL_GPIO_PORT, &gpio_init_struct);
    
    /* 初始化SDA引脚 */
    gpio_init_struct.Pin    = ATK_MS6050_IIC_SDA_GPIO_PIN;  /* SDA引脚 */
    gpio_init_struct.Mode   = GPIO_MODE_OUTPUT_OD;          /* 开漏输出 */
    HAL_GPIO_Init(ATK_MS6050_IIC_SDA_GPIO_PORT, &gpio_init_struct);
    
    atk_ms6050_iic_stop();
}
