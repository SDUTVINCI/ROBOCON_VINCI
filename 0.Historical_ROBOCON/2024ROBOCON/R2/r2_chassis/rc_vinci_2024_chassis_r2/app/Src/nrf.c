#include "nrf.h"

const uint8_t TX_ADDRESS[TX_ADR_WIDTH] = {0x34, 0x73, 0x10, 0x10, 0x04}; // ���͵�ַ
const uint8_t RX_ADDRESS[RX_ADR_WIDTH] = {0x34, 0x73, 0x10, 0x10, 0x04}; // ���͵�ַ

extern SPI_HandleTypeDef hspi1;

void NRF24L01_SPI_Init(void)
{
	__HAL_SPI_DISABLE(&hspi1);	           // �ȹر�SPI3
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW; // ����ͬ��ʱ�ӵĿ���״̬Ϊ�͵�ƽ
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;     // ����ͬ��ʱ�ӵĵ�1�������أ��������½������ݱ�����
	HAL_SPI_Init(&hspi1);
	__HAL_SPI_ENABLE(&hspi1); // ʹ��SPI3
}

// ��ʼ��24L01��IO��
void NRF24L01_Init(void)
{

	NRF24L01_SPI_Init(); // ���NRF���ص��޸�SPI������
	NRF24L01_CE = 0;     // ʹ��24L01
	NRF24L01_CSN = 1;    // SPIƬѡȡ��
}

// SPI3 ��дһ���ֽ�
// TxData:Ҫд����ֽ�
// ����ֵ:��ȡ�����ֽ�
uint8_t SPI3_ReadWriteByte(uint8_t TxData)
{
	uint8_t Rxdata;
	HAL_SPI_TransmitReceive(&hspi1, &TxData, &Rxdata, 1, 1000);
	return Rxdata; // �����յ�������
}

// ��ȡSPI�Ĵ���ֵ
// reg:Ҫ���ļĴ���
uint8_t NRF24L01_Read_Reg(uint8_t reg)
{
	uint8_t reg_val;
	NRF24L01_CSN = 0;	    // ʹ��SPI����
	SPI3_ReadWriteByte(reg);	    // ���ͼĴ�����
	reg_val = SPI3_ReadWriteByte(0XFF); // ��ȡ�Ĵ�������
	NRF24L01_CSN = 1;	    // ��ֹSPI����
	return (reg_val);	    // ����״ֵ̬
}

// ��ָ��λ�ö���ָ�����ȵ�����
// reg:�Ĵ���(λ��)
//*pBuf:����ָ��
// len:���ݳ���
// ����ֵ,�˴ζ�����״̬�Ĵ���ֵ
uint8_t NRF24L01_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
	uint8_t status, u8_ctr;
	NRF24L01_CSN = 0;	  // ʹ��SPI����
	status = SPI3_ReadWriteByte(reg); // ���ͼĴ���ֵ(λ��),����ȡ״ֵ̬
	for (u8_ctr = 0; u8_ctr < len; u8_ctr++)
		pBuf[u8_ctr] = SPI3_ReadWriteByte(0XFF); // ��������
	NRF24L01_CSN = 1;		         // �ر�SPI����
	return status;			         // ���ض�����״ֵ̬
}

// ��ָ��λ��дָ�����ȵ�����
// reg:�Ĵ���(λ��)
//*pBuf:����ָ��
// len:���ݳ���
// ����ֵ,�˴ζ�����״̬�Ĵ���ֵ
uint8_t NRF24L01_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
	uint8_t status, u8_ctr;
	NRF24L01_CSN = 0;	  // ʹ��SPI����
	status = SPI3_ReadWriteByte(reg); // ���ͼĴ���ֵ(λ��),����ȡ״ֵ̬
	for (u8_ctr = 0; u8_ctr < len; u8_ctr++)
		SPI3_ReadWriteByte(*pBuf++); // д������
	NRF24L01_CSN = 1;	             // �ر�SPI����
	return status;		             // ���ض�����״ֵ̬
}

// ���24L01�Ƿ����
// ����ֵ:0���ɹ�;1��ʧ��
uint8_t NRF24L01_Check(void)
{
	uint8_t buf[5] = {0XA5, 0XA5, 0XA5, 0XA5, 0XA5};
	uint8_t i;
	// SPI2_SetSpeed(SPI_BaudRatePrescaler_4); //spi�ٶ�Ϊ9Mhz��24L01�����SPIʱ��Ϊ10Mhz��
	NRF24L01_Write_Buf(NRF_WRITE_REG + TX_ADDR, buf, 5); // д��5���ֽڵĵ�ַ.
	NRF24L01_Read_Buf(TX_ADDR, buf, 5);	     // ����д��ĵ�ַ
	for (i = 0; i < 5; i++)
		if (buf[i] != 0XA5)
			break;
	if (i != 5)
		return 1; // ���24L01����
	return 0;	          // ��⵽24L01
}

// SPIд�Ĵ���
// reg:ָ���Ĵ�����ַ
// value:д���ֵ
uint8_t NRF24L01_Write_Reg(uint8_t reg, uint8_t value)
{
	uint8_t status;
	NRF24L01_CSN = 0;	  // ʹ��SPI����
	status = SPI3_ReadWriteByte(reg); // ���ͼĴ�����
	SPI3_ReadWriteByte(value);	  // д��Ĵ�����ֵ
	NRF24L01_CSN = 1;	  // ��ֹSPI����
	return (status);	  // ����״ֵ̬
}

// ����NRF24L01����һ������
// txbuf:�����������׵�ַ
// ����ֵ:�������״��
uint8_t NRF24L01_TxPacket(uint8_t *txbuf)
{
	uint8_t sta;
	// SPI1_SetSpeed(SPI_BAUDRATEPRESCALER_8); //spi�ٶ�Ϊ6.75Mhz��24L01�����SPIʱ��Ϊ10Mhz��
	NRF24L01_CE = 0;
	NRF24L01_Write_Buf(WR_TX_PLOAD, txbuf, TX_PLOAD_WIDTH); // д���ݵ�TX BUF  32���ֽ�
	NRF24L01_CE = 1;		        // ��������
	while (NRF24L01_IRQ != 0)
		;		 // �ȴ��������
	sta = NRF24L01_Read_Reg(STATUS);	 // ��ȡ״̬�Ĵ�����ֵ
	NRF24L01_Write_Reg(NRF_WRITE_REG + STATUS, sta); // ���TX_DS��MAX_RT�жϱ�־
	if (sta & MAX_TX)		 // �ﵽ����ط�����
	{
		NRF24L01_Write_Reg(FLUSH_TX, 0xff); // ���TX FIFO�Ĵ���
		return MAX_TX;
	}
	if (sta & TX_OK) // �������
	{
		return TX_OK;
	}
	return 0xff; // ����ԭ����ʧ��
}
// ����NRF24L01����һ������
// txbuf:�����������׵�ַ
// ����ֵ:0��������ɣ��������������
uint8_t NRF24L01_RxPacket(uint8_t *rxbuf)
{
	uint8_t sta;
	// SPI1_SetSpeed(SPI_BAUDRATEPRESCALER_8); //spi�ٶ�Ϊ6.75Mhz��24L01�����SPIʱ��Ϊ10Mhz��
	sta = NRF24L01_Read_Reg(STATUS);	 // ��ȡ״̬�Ĵ�����ֵ
	NRF24L01_Write_Reg(NRF_WRITE_REG + STATUS, sta); // ���TX_DS��MAX_RT�жϱ�־
	if (sta & RX_OK)		 // ���յ�����
	{
		NRF24L01_Read_Buf(RD_RX_PLOAD, rxbuf, RX_PLOAD_WIDTH); // ��ȡ����
		NRF24L01_Write_Reg(FLUSH_RX, 0xff);	       // ���RX FIFO�Ĵ���
		return 0;
	}
	return 1; // û�յ��κ�����
}
// �ú�����ʼ��NRF24L01��RXģʽ
// ����RX��ַ,дRX���ݿ��,ѡ��RFƵ��,�����ʺ�LNA HCURR
// ��CE��ߺ�,������RXģʽ,�����Խ���������
void NRF24L01_RX_Mode(void)
{
	NRF24L01_CE = 0;
	NRF24L01_Write_Buf(NRF_WRITE_REG + RX_ADDR_P0, (uint8_t *)RX_ADDRESS, RX_ADR_WIDTH); // дRX�ڵ��ַ

	NRF24L01_Write_Reg(NRF_WRITE_REG + EN_AA, 0x01);              // ʹ��ͨ��0���Զ�Ӧ��
	NRF24L01_Write_Reg(NRF_WRITE_REG + EN_RXADDR, 0x01);          // ʹ��ͨ��0�Ľ��յ�ַ
	NRF24L01_Write_Reg(NRF_WRITE_REG + RF_CH, 40);	              // ����RFͨ��Ƶ��
	NRF24L01_Write_Reg(NRF_WRITE_REG + RX_PW_P0, RX_PLOAD_WIDTH); // ѡ��ͨ��0����Ч���ݿ��
	NRF24L01_Write_Reg(NRF_WRITE_REG + RF_SETUP, 0x0f);           // ����TX�������,0db����,2Mbps,���������濪��
	NRF24L01_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0f);             // ���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ
	NRF24L01_CE = 1;		              // CEΪ��,�������ģʽ
}
// �ú�����ʼ��NRF24L01��TXģʽ
// ����TX��ַ,дTX���ݿ��,����RX�Զ�Ӧ��ĵ�ַ,���TX��������,ѡ��RFƵ��,�����ʺ�LNA HCURR
// PWR_UP,CRCʹ��
// ��CE��ߺ�,������RXģʽ,�����Խ���������
// CEΪ�ߴ���10us,����������.
void NRF24L01_TX_Mode(void)
{
	NRF24L01_CE = 0;
	NRF24L01_Write_Buf(NRF_WRITE_REG + TX_ADDR, (uint8_t *)TX_ADDRESS, TX_ADR_WIDTH);    // дTX�ڵ��ַ
	NRF24L01_Write_Buf(NRF_WRITE_REG + RX_ADDR_P0, (uint8_t *)RX_ADDRESS, RX_ADR_WIDTH); // ����TX�ڵ��ַ,��ҪΪ��ʹ��ACK

	NRF24L01_Write_Reg(NRF_WRITE_REG + EN_AA, 0x01);      // ʹ��ͨ��0���Զ�Ӧ��
	NRF24L01_Write_Reg(NRF_WRITE_REG + EN_RXADDR, 0x01);  // ʹ��ͨ��0�Ľ��յ�ַ
	NRF24L01_Write_Reg(NRF_WRITE_REG + SETUP_RETR, 0x1a); // �����Զ��ط����ʱ��:500us + 86us;����Զ��ط�����:10��
	NRF24L01_Write_Reg(NRF_WRITE_REG + RF_CH, 40);	      // ����RFͨ��Ϊ40
	NRF24L01_Write_Reg(NRF_WRITE_REG + RF_SETUP, 0x0f);   // ����TX�������,0db����,2Mbps,���������濪��
	NRF24L01_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0e);     // ���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ,���������ж�
	NRF24L01_CE = 1;		      // CEΪ��,10us����������
}
