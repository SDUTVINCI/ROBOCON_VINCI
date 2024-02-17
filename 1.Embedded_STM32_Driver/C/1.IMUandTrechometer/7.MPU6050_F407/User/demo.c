/**
 ****************************************************************************************************
 * @file        demo.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2022-06-21
 * @brief       ATK-MS6050模块测试实验
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

#include "demo.h"
#include "./BSP/ATK_MS6050/atk_ms6050.h"
#include "./BSP/ATK_MS6050/eMPL/inv_mpu.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/key/key.h"
#include "./BSP/lcd/lcd.h"
#include "./BSP/led/led.h"

/**
 * @brief       通过串口1发送数据至匿名地面站V4
 * @param       fun: 功能字
 *              dat: 待发送的数据（最多28字节）
 *              len: dat数据的有效位数
 * @retval      无
 */
static void demo_usart1_niming_report(uint8_t fun, uint8_t *dat, uint8_t len)
{
    uint8_t send_buf[32];
    uint8_t i;
    
    if (len > 28)
    {
        return;
    }
    
    send_buf[len+4] = 0;            /* 校验位清零 */
    send_buf[0] = 0xAA;             /* 帧头为0xAAAA */
    send_buf[1] = 0xAA;             /* 帧头为0xAAAA */
    send_buf[2] = fun;              /* 功能字 */
    send_buf[3] = len;              /* 数据长度 */
    for (i=0; i<len; i++)           /* 复制数据 */
    {
        send_buf[4 + i] = dat[i];
    }
    for (i=0; i<(len + 4); i++)     /* 计算校验和 */
    {
        send_buf[len + 4] += send_buf[i];
    }
    
    /* 发送数据 */
    HAL_UART_Transmit(&g_uart1_handle, send_buf, len + 5, HAL_MAX_DELAY);
}

/**
 * @brief       发送状态帧至匿名地面站V4
 * @param       rol     : 横滚角
 *              pit     : 俯仰角
 *              yaw     : 航向角
 *              alt     : 飞行高度，单位：cm
 *              fly_mode: 飞行模式
 *              armed   : 锁定状态，0xA0：加锁 0xA1：解锁
 * @retval      无
 */
static void demo_niming_report_status(int16_t rol, int16_t pit, int16_t yaw, uint32_t alt, uint8_t fly_mode, uint8_t armed)
{
    uint8_t send_buf[12];
    
    /* 横滚角 */
    send_buf[0] = (rol >> 8) & 0xFF;
    send_buf[1] = rol & 0xFF;
    /* 俯仰角 */
    send_buf[2] = (pit >> 8) & 0xFF;
    send_buf[3] = pit & 0xFF;
    /* 航向角 */
    send_buf[4] = (yaw >> 8) & 0xFF;
    send_buf[5] = yaw & 0xFF;
    /* 飞行高度 */
    send_buf[6] = (alt >> 24) & 0xFF;
    send_buf[7] = (alt >> 16) & 0xFF;
    send_buf[8] = (alt >> 8) & 0xFF;
    send_buf[9] = alt & 0xFF;
    /* 飞行模式 */
    send_buf[10] = fly_mode;
    /* 锁定状态 */
    send_buf[11] = armed;
    
    /* 状态帧的功能字为0x01 */
    demo_usart1_niming_report(0x01, send_buf, 12);
}

/**
 * @brief       发送传感器帧至匿名地面站V4
 * @param       acc_x : x轴上的加速度值
 *              acc_y : y轴上的加速度值
 *              acc_z : z轴上的加速度值
 *              gyro_x: x轴上的陀螺仪值
 *              gyro_y: y轴上的陀螺仪值
 *              gyro_z: z轴上的陀螺仪值
 *              mag_x : x轴上的磁力计值（ATK-MS6050不支持）
 *              mag_y : y轴上的磁力计值（ATK-MS6050不支持）
 *              mag_z : z轴上的磁力计值（ATK-MS6050不支持）
 * @retval      无
 */
static void demo_niming_report_senser(  int16_t  acc_x, int16_t  acc_y, int16_t  acc_z,
                                        int16_t gyro_x, int16_t gyro_y, int16_t gyro_z,
                                        int16_t  mag_x, int16_t  mag_y, int16_t  mag_z)
{
    uint8_t send_buf[18];
    
    /* x轴上的加速度值 */
    send_buf[0] = (acc_x >> 8) & 0xFF;
    send_buf[1] = acc_x & 0xFF;
    /* y轴上的加速度值 */
    send_buf[2] = (acc_y >> 8) & 0xFF;
    send_buf[3] = acc_y & 0xFF;
    /* z轴上的加速度值 */
    send_buf[4] = (acc_z >> 8) & 0xFF;
    send_buf[5] = acc_z & 0xFF;
    /* x轴上的陀螺仪值 */
    send_buf[6] = (gyro_x >> 8) & 0xFF;
    send_buf[7] = gyro_x & 0xFF;
    /* y轴上的陀螺仪值 */
    send_buf[8] = (gyro_y >> 8) & 0xFF;
    send_buf[9] = gyro_y & 0xFF;
    /* z轴上的陀螺仪值 */
    send_buf[10] = (gyro_z >> 8) & 0xFF;
    send_buf[11] = gyro_z & 0xFF;
    /* x轴上的磁力计值 */
    send_buf[12] = (mag_x >> 8) & 0xFF;
    send_buf[13] = mag_x & 0xFF;
    /* y轴上的磁力计值 */
    send_buf[14] = (mag_y >> 8) & 0xFF;
    send_buf[15] = mag_y & 0xFF;
    /* z轴上的磁力计值 */
    send_buf[16] = (mag_z >> 8) & 0xFF;
    send_buf[17] = mag_z & 0xFF;
    
    /* 传感器的功能字为0x02 */
    demo_usart1_niming_report(0x02, send_buf, 18);
}

/**
 * @brief       LCD UI初始化
 * @param       无
 * @retval      无
 */
static void demo_lcd_ui_init(void)
{
    lcd_show_string(20, 130, 152, 16, 16, "PIT :    .  degrees", BLUE);
    lcd_show_string(20, 151, 152, 16, 16, "ROL :    .  degrees", BLUE);
    lcd_show_string(20, 172, 152, 16, 16, "YAW :    .  degrees", BLUE);
    lcd_show_string(20, 193, 216, 16, 16, "TEMP:    .  degrees Celsius", BLUE);
}

/**
 * @brief       LCD显示相关数据信息
 * @param       无
 * @retval      无
 */
static void demo_lcd_show_msg(float pit, float rol, float yaw, int16_t temp)
{
    int16_t tmp;
    
    /* 俯仰角 */
    tmp = pit * 10;
    if (tmp < 0)
    {
        tmp = -tmp;
        lcd_show_string(60, 130, 8, 16, 16, "-", BLUE);
    }
    else
    {
        lcd_show_string(60, 130, 8, 16, 16, " ", BLUE);
    }
    lcd_show_num(68, 130, tmp / 10, 3, 16, BLUE);
    lcd_show_num(100, 130, tmp % 10, 1, 16, BLUE);
    
    /* 横滚角 */
    tmp = rol * 10;
    if (tmp < 0)
    {
        tmp = -tmp;
        lcd_show_string(60, 151, 8, 16, 16, "-", BLUE);
    }
    else
    {
        lcd_show_string(60, 151, 8, 16, 16, " ", BLUE);
    }
    lcd_show_num(68, 151, tmp / 10, 3, 16, BLUE);
    lcd_show_num(100, 151, tmp % 10, 1, 16, BLUE);
    
    /* 航向角 */
    tmp = yaw * 10;
    if (tmp < 0)
    {
        tmp = -tmp;
        lcd_show_string(60, 172, 8, 16, 16, "-", BLUE);
    }
    else
    {
        lcd_show_string(60, 172, 8, 16, 16, " ", BLUE);
    }
    lcd_show_num(68, 172, tmp / 10, 3, 16, BLUE);
    lcd_show_num(100, 172, tmp % 10, 1, 16, BLUE);
    
    /* 温度 */
    tmp = temp;
    if (tmp < 0)
    {
        tmp = -tmp;
        lcd_show_string(60, 193, 8, 16, 16, "-", BLUE);
    }
    else
    {
        lcd_show_string(60, 193, 8, 16, 16, " ", BLUE);
    }
    lcd_show_num(68, 193, tmp / 100, 3, 16, BLUE);
    lcd_show_num(100, 193, tmp % 10, 1, 16, BLUE);
}

/**
 * @brief       例程演示入口函数
 * @param       无
 * @retval      无
 */
void demo_run(void)
{
    uint8_t ret;
    uint8_t key;
    uint8_t niming_report = 0;
    float pit, rol, yaw;
    int16_t acc_x, acc_y, acc_z;
    int16_t gyr_x, gyr_y, gyr_z;
    int16_t temp;
    
    /* 初始化ATK-MS6050 */
    ret = atk_ms6050_init();
    if (ret != 0)
    {
        printf("ATK-MS6050 init failed!\r\n");
        while (1)
        {
            LED0_TOGGLE();
            delay_ms(200);
        }
    }
    
    /* 初始化ATK-MS6050 DMP */
    ret = atk_ms6050_dmp_init();
    if (ret != 0)
    {
        printf("ATK-MS6050 DMP init failed!\r\n");
        while (1)
        {
            LED0_TOGGLE();
            delay_ms(200);
        }
    }
    
    /* LCD UI初始化 */
    demo_lcd_ui_init();
    
    while (1)
    {
        key = key_scan(0);
        if (key == KEY0_PRES)
        {
            /* 当按键0按下后，切换串口的上传状态
             * 当niming_report为0时，上传相关信息至串口调试助手
             * 当niming_report为1时，上传相关信息至匿名地面站V4
             */
            niming_report = 1 - niming_report;
            if (niming_report == 0)
            {
                /* 串口调试助手的串口通讯波特率为115200 */
                usart_init(115200);
            }
            else
            {
                /* 匿名地面站V4的串口通讯波特率为500000 */
                usart_init(500000);
            }
        }
        
        /* 获取ATK-MS6050 DMP处理后的数据 */
        ret  = atk_ms6050_dmp_get_data(&pit, &rol, &yaw);
        /* 获取ATK-MS6050加速度值 */
        ret += atk_ms6050_get_accelerometer(&acc_x, &acc_y, &acc_z);
        /* 获取ATK-MS6050陀螺仪值 */
        ret += atk_ms6050_get_gyroscope(&gyr_x, &gyr_y, &gyr_z);
        /* 获取ATK-MS6050温度值 */
        ret += atk_ms6050_get_temperature(&temp);
        if (ret == 0)
        {
            if (niming_report == 0)
            {
                /* 上传相关数据信息至串口调试助手 */
                printf("pit: %.2f, rol: %.2f, yaw: %.2f, ", pit, rol, yaw);
                printf("acc_x: %d, acc_y: %d, acc_z: %d, ", acc_x, acc_y, acc_z);
                printf("gyr_x: %d, gyr_y: %d, gyr_z: %d, ", gyr_x, gyr_y, gyr_z);
                printf("temp: %d\r\n", temp);
            }
            else
            {
                /* 上传状态帧和传感器帧至匿名地面站V4 */
                demo_niming_report_status((int16_t)(rol * 100), (int16_t)((pit) * 100), (int16_t)(yaw * 100), 0, 0, 0);
                demo_niming_report_senser(acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z, 0, 0, 0);
            }
            
            /* 在LCD上显示相关数据信息 */
            demo_lcd_show_msg(pit, rol, yaw, temp);
        }
    }
}
