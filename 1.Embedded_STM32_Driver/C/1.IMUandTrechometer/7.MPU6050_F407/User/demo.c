/**
 ****************************************************************************************************
 * @file        demo.c
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2022-06-21
 * @brief       ATK-MS6050ģ�����ʵ��
 * @license     Copyright (c) 2020-2032, ������������ӿƼ����޹�˾
 ****************************************************************************************************
 * @attention
 *
 * ʵ��ƽ̨:����ԭ�� ̽���� F407������
 * ������Ƶ:www.yuanzige.com
 * ������̳:www.openedv.com
 * ��˾��ַ:www.alientek.com
 * �����ַ:openedv.taobao.com
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
 * @brief       ͨ������1������������������վV4
 * @param       fun: ������
 *              dat: �����͵����ݣ����28�ֽڣ�
 *              len: dat���ݵ���Чλ��
 * @retval      ��
 */
static void demo_usart1_niming_report(uint8_t fun, uint8_t *dat, uint8_t len)
{
    uint8_t send_buf[32];
    uint8_t i;
    
    if (len > 28)
    {
        return;
    }
    
    send_buf[len+4] = 0;            /* У��λ���� */
    send_buf[0] = 0xAA;             /* ֡ͷΪ0xAAAA */
    send_buf[1] = 0xAA;             /* ֡ͷΪ0xAAAA */
    send_buf[2] = fun;              /* ������ */
    send_buf[3] = len;              /* ���ݳ��� */
    for (i=0; i<len; i++)           /* �������� */
    {
        send_buf[4 + i] = dat[i];
    }
    for (i=0; i<(len + 4); i++)     /* ����У��� */
    {
        send_buf[len + 4] += send_buf[i];
    }
    
    /* �������� */
    HAL_UART_Transmit(&g_uart1_handle, send_buf, len + 5, HAL_MAX_DELAY);
}

/**
 * @brief       ����״̬֡����������վV4
 * @param       rol     : �����
 *              pit     : ������
 *              yaw     : �����
 *              alt     : ���и߶ȣ���λ��cm
 *              fly_mode: ����ģʽ
 *              armed   : ����״̬��0xA0������ 0xA1������
 * @retval      ��
 */
static void demo_niming_report_status(int16_t rol, int16_t pit, int16_t yaw, uint32_t alt, uint8_t fly_mode, uint8_t armed)
{
    uint8_t send_buf[12];
    
    /* ����� */
    send_buf[0] = (rol >> 8) & 0xFF;
    send_buf[1] = rol & 0xFF;
    /* ������ */
    send_buf[2] = (pit >> 8) & 0xFF;
    send_buf[3] = pit & 0xFF;
    /* ����� */
    send_buf[4] = (yaw >> 8) & 0xFF;
    send_buf[5] = yaw & 0xFF;
    /* ���и߶� */
    send_buf[6] = (alt >> 24) & 0xFF;
    send_buf[7] = (alt >> 16) & 0xFF;
    send_buf[8] = (alt >> 8) & 0xFF;
    send_buf[9] = alt & 0xFF;
    /* ����ģʽ */
    send_buf[10] = fly_mode;
    /* ����״̬ */
    send_buf[11] = armed;
    
    /* ״̬֡�Ĺ�����Ϊ0x01 */
    demo_usart1_niming_report(0x01, send_buf, 12);
}

/**
 * @brief       ���ʹ�����֡����������վV4
 * @param       acc_x : x���ϵļ��ٶ�ֵ
 *              acc_y : y���ϵļ��ٶ�ֵ
 *              acc_z : z���ϵļ��ٶ�ֵ
 *              gyro_x: x���ϵ�������ֵ
 *              gyro_y: y���ϵ�������ֵ
 *              gyro_z: z���ϵ�������ֵ
 *              mag_x : x���ϵĴ�����ֵ��ATK-MS6050��֧�֣�
 *              mag_y : y���ϵĴ�����ֵ��ATK-MS6050��֧�֣�
 *              mag_z : z���ϵĴ�����ֵ��ATK-MS6050��֧�֣�
 * @retval      ��
 */
static void demo_niming_report_senser(  int16_t  acc_x, int16_t  acc_y, int16_t  acc_z,
                                        int16_t gyro_x, int16_t gyro_y, int16_t gyro_z,
                                        int16_t  mag_x, int16_t  mag_y, int16_t  mag_z)
{
    uint8_t send_buf[18];
    
    /* x���ϵļ��ٶ�ֵ */
    send_buf[0] = (acc_x >> 8) & 0xFF;
    send_buf[1] = acc_x & 0xFF;
    /* y���ϵļ��ٶ�ֵ */
    send_buf[2] = (acc_y >> 8) & 0xFF;
    send_buf[3] = acc_y & 0xFF;
    /* z���ϵļ��ٶ�ֵ */
    send_buf[4] = (acc_z >> 8) & 0xFF;
    send_buf[5] = acc_z & 0xFF;
    /* x���ϵ�������ֵ */
    send_buf[6] = (gyro_x >> 8) & 0xFF;
    send_buf[7] = gyro_x & 0xFF;
    /* y���ϵ�������ֵ */
    send_buf[8] = (gyro_y >> 8) & 0xFF;
    send_buf[9] = gyro_y & 0xFF;
    /* z���ϵ�������ֵ */
    send_buf[10] = (gyro_z >> 8) & 0xFF;
    send_buf[11] = gyro_z & 0xFF;
    /* x���ϵĴ�����ֵ */
    send_buf[12] = (mag_x >> 8) & 0xFF;
    send_buf[13] = mag_x & 0xFF;
    /* y���ϵĴ�����ֵ */
    send_buf[14] = (mag_y >> 8) & 0xFF;
    send_buf[15] = mag_y & 0xFF;
    /* z���ϵĴ�����ֵ */
    send_buf[16] = (mag_z >> 8) & 0xFF;
    send_buf[17] = mag_z & 0xFF;
    
    /* �������Ĺ�����Ϊ0x02 */
    demo_usart1_niming_report(0x02, send_buf, 18);
}

/**
 * @brief       LCD UI��ʼ��
 * @param       ��
 * @retval      ��
 */
static void demo_lcd_ui_init(void)
{
    lcd_show_string(20, 130, 152, 16, 16, "PIT :    .  degrees", BLUE);
    lcd_show_string(20, 151, 152, 16, 16, "ROL :    .  degrees", BLUE);
    lcd_show_string(20, 172, 152, 16, 16, "YAW :    .  degrees", BLUE);
    lcd_show_string(20, 193, 216, 16, 16, "TEMP:    .  degrees Celsius", BLUE);
}

/**
 * @brief       LCD��ʾ���������Ϣ
 * @param       ��
 * @retval      ��
 */
static void demo_lcd_show_msg(float pit, float rol, float yaw, int16_t temp)
{
    int16_t tmp;
    
    /* ������ */
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
    
    /* ����� */
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
    
    /* ����� */
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
    
    /* �¶� */
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
 * @brief       ������ʾ��ں���
 * @param       ��
 * @retval      ��
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
    
    /* ��ʼ��ATK-MS6050 */
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
    
    /* ��ʼ��ATK-MS6050 DMP */
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
    
    /* LCD UI��ʼ�� */
    demo_lcd_ui_init();
    
    while (1)
    {
        key = key_scan(0);
        if (key == KEY0_PRES)
        {
            /* ������0���º��л����ڵ��ϴ�״̬
             * ��niming_reportΪ0ʱ���ϴ������Ϣ�����ڵ�������
             * ��niming_reportΪ1ʱ���ϴ������Ϣ����������վV4
             */
            niming_report = 1 - niming_report;
            if (niming_report == 0)
            {
                /* ���ڵ������ֵĴ���ͨѶ������Ϊ115200 */
                usart_init(115200);
            }
            else
            {
                /* ��������վV4�Ĵ���ͨѶ������Ϊ500000 */
                usart_init(500000);
            }
        }
        
        /* ��ȡATK-MS6050 DMP���������� */
        ret  = atk_ms6050_dmp_get_data(&pit, &rol, &yaw);
        /* ��ȡATK-MS6050���ٶ�ֵ */
        ret += atk_ms6050_get_accelerometer(&acc_x, &acc_y, &acc_z);
        /* ��ȡATK-MS6050������ֵ */
        ret += atk_ms6050_get_gyroscope(&gyr_x, &gyr_y, &gyr_z);
        /* ��ȡATK-MS6050�¶�ֵ */
        ret += atk_ms6050_get_temperature(&temp);
        if (ret == 0)
        {
            if (niming_report == 0)
            {
                /* �ϴ����������Ϣ�����ڵ������� */
                printf("pit: %.2f, rol: %.2f, yaw: %.2f, ", pit, rol, yaw);
                printf("acc_x: %d, acc_y: %d, acc_z: %d, ", acc_x, acc_y, acc_z);
                printf("gyr_x: %d, gyr_y: %d, gyr_z: %d, ", gyr_x, gyr_y, gyr_z);
                printf("temp: %d\r\n", temp);
            }
            else
            {
                /* �ϴ�״̬֡�ʹ�����֡����������վV4 */
                demo_niming_report_status((int16_t)(rol * 100), (int16_t)((pit) * 100), (int16_t)(yaw * 100), 0, 0, 0);
                demo_niming_report_senser(acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z, 0, 0, 0);
            }
            
            /* ��LCD����ʾ���������Ϣ */
            demo_lcd_show_msg(pit, rol, yaw, temp);
        }
    }
}
