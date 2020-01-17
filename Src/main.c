/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <ctype.h>
#include <stdbool.h>

#include "socket.h"
#include "dhcp.h"
#include "wizchip_conf.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
static const uint16_t crcTable[] = {
    0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241,
    0XC601, 0X06C0, 0X0780, 0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440,
    0XCC01, 0X0CC0, 0X0D80, 0XCD41, 0X0F00, 0XCFC1, 0XCE81, 0X0E40,
    0X0A00, 0XCAC1, 0XCB81, 0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841,
    0XD801, 0X18C0, 0X1980, 0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40,
    0X1E00, 0XDEC1, 0XDF81, 0X1F40, 0XDD01, 0X1DC0, 0X1C80, 0XDC41,
    0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0, 0X1680, 0XD641,
    0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040,
    0XF001, 0X30C0, 0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240,
    0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501, 0X35C0, 0X3480, 0XF441,
    0X3C00, 0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41,
    0XFA01, 0X3AC0, 0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840,
    0X2800, 0XE8C1, 0XE981, 0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41,
    0XEE01, 0X2EC0, 0X2F80, 0XEF41, 0X2D00, 0XEDC1, 0XEC81, 0X2C40,
    0XE401, 0X24C0, 0X2580, 0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640,
    0X2200, 0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0, 0X2080, 0XE041,
    0XA001, 0X60C0, 0X6180, 0XA141, 0X6300, 0XA3C1, 0XA281, 0X6240,
    0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480, 0XA441,
    0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41,
    0XAA01, 0X6AC0, 0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840,
    0X7800, 0XB8C1, 0XB981, 0X7940, 0XBB01, 0X7BC0, 0X7A80, 0XBA41,
    0XBE01, 0X7EC0, 0X7F80, 0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40,
    0XB401, 0X74C0, 0X7580, 0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640,
    0X7200, 0XB2C1, 0XB381, 0X7340, 0XB101, 0X71C0, 0X7080, 0XB041,
    0X5000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0, 0X5280, 0X9241,
    0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481, 0X5440,
    0X9C01, 0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40,
    0X5A00, 0X9AC1, 0X9B81, 0X5B40, 0X9901, 0X59C0, 0X5880, 0X9841,
    0X8801, 0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81, 0X4A40,
    0X4E00, 0X8EC1, 0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41,
    0X4400, 0X84C1, 0X8581, 0X4540, 0X8701, 0X47C0, 0X4680, 0X8641,
    0X8201, 0X42C0, 0X4380, 0X8341, 0X4100, 0X81C1, 0X8081, 0X4040
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define REG_VOLTAGE     0x0000
#define REG_CURRENT_L   0x0001
#define REG_CURRENT_H   0X0002
#define REG_POWER_L     0x0003
#define REG_POWER_H     0x0004
#define REG_ENERGY_L    0x0005
#define REG_ENERGY_H    0x0006
#define REG_FREQUENCY   0x0007
#define REG_PF          0x0008
#define REG_ALARM       0x0009

#define CMD_RHR         0x03
#define CMD_RIR         0X04
#define CMD_WSR         0x06
#define CMD_CAL         0x41
#define CMD_REST        0x42


#define WREG_ALARM_THR   0x0001
#define WREG_ADDR        0x0002

#define UPDATE_TIME     200

#define RESPONSE_SIZE 32
#define READ_TIMEOUT 100

#define PZEM_BAUD_RATE 9600
uint8_t _addr = 0x42;

#define DATA_BUF_SIZE   500

#define DHCP_SOCKET     0
#define HTTP_SOCKET     2
uint8_t gDATABUF[DATA_BUF_SIZE];
uint8_t dhcp_buffer[1024];
uint8_t gateWay[4] = {192, 168, 1, 1};

#define json_answer	"HTTP/1.0 200 OK\r\n"\
		"Content-Type: application/json\r\n"\
		"Access-Control-Allow-Origin: *\r\n"\
		"\r\n"\
		"{\"voltage\": %.2f, \"current\": %.6f, \"power\": %.2f, \"energy\": %.2f, \"frequency\: %.2f, \"udid\":\"%s\"}"\

#define html_answer	"HTTP/1.0 200 OK\r\n"\
		"Content-Type: text/html\r\n"\
		"\r\n"\
		"voltage: %.2f <br> current: %.6f <br> power: %.2f <br>energy: %.2f<br> frequency: %.2f <br> udid: %s"\

uint8_t udids[27];
volatile uint32_t udid[3];

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define pgm_read_word(addr) (*(const unsigned short *)(addr))

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
struct {
       float voltage;
       float current;
       float power;
       float energy;
       float frequeny;
       float pf;
       uint16_t alarms;
   }  _currentValues; // Measured values

   uint64_t _lastRead;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t CRC16(const uint8_t *data, uint16_t len)
{
    uint8_t nTemp; // CRC table index
    uint16_t crc = 0xFFFF; // Default value

    while (len--)
    {
        nTemp = *data++ ^ crc;
        crc >>= 8;
        crc ^= (uint16_t)pgm_read_word(&crcTable[nTemp]);
    }
    return crc;
}

uint8_t setCRC(uint8_t *buf, uint16_t len){
    if(len <= 2) // Sanity check
        return 1;

    uint16_t crc = CRC16(buf, len - 2); // CRC of data

    // Write high and low byte to last two positions
    buf[len - 2] = crc & 0xFF; // Low byte first
    buf[len - 1] = (crc >> 8) & 0xFF; // High byte second
    return 0;
}

uint8_t sendCmd8(uint8_t cmd, uint16_t rAddr, uint16_t val){
    uint8_t sendBuffer[8]; // Send buffer

    sendBuffer[0] = 0xf8;                   // Set slave address
    sendBuffer[1] = cmd;                     // Set command

    sendBuffer[2] = (rAddr >> 8) & 0xFF;     // Set high byte of register address
    sendBuffer[3] = (rAddr) & 0xFF;          // Set low byte =//=

    sendBuffer[4] = (val >> 8) & 0xFF;       // Set high byte of register value
    sendBuffer[5] = (val) & 0xFF;            // Set low byte =//=

    setCRC(sendBuffer, 8);                   // Set CRC of frame
    HAL_UART_Transmit(&huart1, sendBuffer, sizeof(sendBuffer), HAL_MAX_DELAY);
    return 0;
}

uint8_t sendCmd4(uint8_t cmd){
    uint8_t sendBuffer[4]; // Send buffer
    sendBuffer[0] = 0xf8;                   // Set slave address
    sendBuffer[1] = cmd;                     // Set command
    setCRC(sendBuffer, 4);                   // Set CRC of frame
    HAL_UART_Transmit(&huart1, sendBuffer, sizeof(sendBuffer), HAL_MAX_DELAY);
    return 0;
}

void updateValues()
{
    sendCmd8(CMD_RIR, 0x00, 0x0A);
}

void resetEnergy()
{
	sendCmd4(CMD_REST);
}

uint8_t byte;
int rx_bytes = 0;
uint8_t response[26];
uint8_t response[26];

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	rx_bytes++;
    // Update the current values

		HAL_GPIO_TogglePin(PIN_LED_GPIO_Port, PIN_LED_Pin);
		_currentValues.voltage = ((uint32_t)response[3] << 8 | // Raw voltage in 0.1V
								  (uint32_t)response[4])/10.0;

		_currentValues.current = ((uint32_t)response[5] << 8 | // Raw current in 0.001A
								  (uint32_t)response[6] |
								  (uint32_t)response[7] << 24 |
								  (uint32_t)response[8] << 16) / 1000.0;

		_currentValues.power =   ((uint32_t)response[9] << 8 | // Raw power in 0.1W
								  (uint32_t)response[10] |
								  (uint32_t)response[11] << 24 |
								  (uint32_t)response[12] << 16) / 10.0;

		_currentValues.energy =  ((uint32_t)response[13] << 8 | // Raw Energy in 1Wh
								  (uint32_t)response[14] |
								  (uint32_t)response[15] << 24 |
								  (uint32_t)response[16] << 16) / 1000.0;

		_currentValues.frequeny =((uint32_t)response[17] << 8 | // Raw Frequency in 0.1Hz
								  (uint32_t)response[18]) / 10.0;

		_currentValues.pf =      ((uint32_t)response[19] << 8 | // Raw pf in 0.01
								  (uint32_t)response[20])/100.0;

		_currentValues.alarms =  ((uint32_t)response[21] << 8 | // Raw alarm value
								  (uint32_t)response[22]);

}

void W5500_Select(void) {
    HAL_GPIO_WritePin(EHERNET_CS_GPIO_Port, EHERNET_CS_Pin, GPIO_PIN_RESET);
}

void W5500_Unselect(void) {
    HAL_GPIO_WritePin(EHERNET_CS_GPIO_Port, EHERNET_CS_Pin, GPIO_PIN_SET);
}

void W5500_ReadBuff(uint8_t* buff, uint16_t len) {
    HAL_SPI_Receive(&hspi1, buff, len, HAL_MAX_DELAY);
}

void W5500_WriteBuff(uint8_t* buff, uint16_t len) {
    HAL_SPI_Transmit(&hspi1, buff, len, HAL_MAX_DELAY);
}

uint8_t W5500_ReadByte(void) {
    uint8_t byte;
    W5500_ReadBuff(&byte, sizeof(byte));
    return byte;
}

void W5500_WriteByte(uint8_t byte) {
    W5500_WriteBuff(&byte, sizeof(byte));
}

volatile bool ip_assigned = false;

void Callback_IPAssigned(void) {

	  HAL_GPIO_WritePin(PIN_LED_GPIO_Port, PIN_LED_Pin, GPIO_PIN_RESET);
	  HAL_Delay(100);
	  HAL_GPIO_WritePin(PIN_LED_GPIO_Port, PIN_LED_Pin, GPIO_PIN_SET);
	  HAL_Delay(200);
	  HAL_GPIO_WritePin(PIN_LED_GPIO_Port, PIN_LED_Pin, GPIO_PIN_RESET);
	  HAL_Delay(200);
	  HAL_GPIO_WritePin(PIN_LED_GPIO_Port, PIN_LED_Pin, GPIO_PIN_SET);
	  HAL_Delay(200);
	  HAL_GPIO_WritePin(PIN_LED_GPIO_Port, PIN_LED_Pin, GPIO_PIN_RESET);
	  HAL_Delay(200);
    ip_assigned = true;
}

void Callback_IPConflict(void) {
	  HAL_GPIO_WritePin(PIN_LED_GPIO_Port, PIN_LED_Pin, GPIO_PIN_RESET);
	  HAL_Delay(100);
	  HAL_GPIO_WritePin(PIN_LED_GPIO_Port, PIN_LED_Pin, GPIO_PIN_SET);
	  HAL_Delay(100);
	  HAL_GPIO_WritePin(PIN_LED_GPIO_Port, PIN_LED_Pin, GPIO_PIN_RESET);
	  HAL_Delay(100);
	  HAL_GPIO_WritePin(PIN_LED_GPIO_Port, PIN_LED_Pin, GPIO_PIN_SET);
while(1){}

}

// 1K should be enough, see https://forum.wiznet.io/t/topic/1612/2
uint8_t dhcp_buffer[1024];
uint32_t dhcp_ctr = 10000;

void w55500init() {
//    UART_Printf("\r\ninit() called!\r\n");
//    UART_Printf("Registering W5500 callbacks...\r\n");

	uint8_t tmp;
    reg_wizchip_cs_cbfunc(W5500_Select, W5500_Unselect);
    reg_wizchip_spi_cbfunc(W5500_ReadByte, W5500_WriteByte);
    reg_wizchip_spiburst_cbfunc(W5500_ReadBuff, W5500_WriteBuff);

//    UART_Printf("Calling wizchip_init()...\r\n");
    uint8_t memsize[2][8] = {{2,2,2,2,2,2,2,2},{2,2,2,2,2,2,2,2}};


	/* WIZCHIP SOCKET Buffer initialize */
	if(ctlwizchip(CW_INIT_WIZCHIP,(void*)memsize) == -1){
		//init fail
		//printf("WIZCHIP Initialized fail.\r\n");
		while(1);
	}
	do{
		if(ctlwizchip(CW_GET_PHYLINK, (void*)&tmp) == -1){};
			//printf("Unknown PHY Link stauts.\r\n");
	}while(tmp == PHY_LINK_OFF);

	wiz_NetInfo net_info = {    .mac = {0x00, 0x08, 0xdc, 0xab, 0xcd, 0xef},
	                            .ip = {172, 168, 1, 154},
	                            .sn = {255, 255, 254, 0},
	                            .gw = {172, 168, 1, 1},
	                            .dns = {0, 0, 0, 0},
	                            .dhcp = NETINFO_DHCP };

	setSHAR(net_info.mac);			//set MAC addr
	setSIPR(net_info.ip);			//set IP addr
	setGAR(net_info.gw);			//set gate way
	setSUBR(net_info.sn);			//set subnet

///////////////////////////////////////////

    DHCP_init(DHCP_SOCKET, dhcp_buffer);
    reg_dhcp_cbfunc(
           Callback_IPAssigned,
           Callback_IPAssigned,
           Callback_IPConflict
       );

	while((!ip_assigned) && (dhcp_ctr > 0)) {
		DHCP_run();
		dhcp_ctr--;
	}
	if(!ip_assigned) {
//		UART_Printf("\r\nIP was not assigned :(\r\n");
		return;
	}

	getIPfromDHCP(net_info.ip);
	getGWfromDHCP(net_info.gw);
	getSNfromDHCP(net_info.sn);

	wizchip_setnetinfo(&net_info);

}

int32_t loopback_tcps(uint8_t sn, uint8_t* buf, uint16_t port){
	int32_t ret;
	uint16_t size = 0, sentsize=0;
	uint8_t *url;

	switch(getSn_SR(sn)){
		case SOCK_ESTABLISHED :
			if(getSn_IR(sn) & Sn_IR_CON){
//				printf("%d:Connected\r\n",sn);
				setSn_IR(sn,Sn_IR_CON);
			}
			if((size = getSn_RX_RSR(sn)) > 0){
				if(size > DATA_BUF_SIZE) size = DATA_BUF_SIZE;
				ret = recv(sn,buf,size);
				if(ret <= 0) return ret;
				sentsize = 0;
				if(memcmp(buf, "GET ", 4)==0){
					// extract URL from request header
					url = buf + 4;
					//http data fill
					if(memcmp(url, "/info", 5)==0){
						sprintf(buf, json_answer,
								 _currentValues.voltage,
							     _currentValues.current,
								 _currentValues.power,
								 _currentValues.energy,
								_currentValues.frequeny,
								udids
								);
					}else if(memcmp(url, "/reset", 6)==0){
					    HAL_Delay(100);
						HAL_UART_Receive_IT(&huart1, response, 4);
						resetEnergy();
					    HAL_Delay(100);
						sprintf(buf, html_answer,
								 _currentValues.voltage,
							     _currentValues.current,
								 _currentValues.power,
								 _currentValues.energy,
								_currentValues.frequeny,
								udids
								);
					}else if(memcmp(url, "/pause", 6)==0){
					    HAL_Delay(1000);
						sprintf(buf, html_answer,
								 _currentValues.voltage,
							     _currentValues.current,
								 _currentValues.power,
								 _currentValues.energy,
								_currentValues.frequeny,
								udids
								);
					}else{
						sprintf(buf, html_answer,
								 _currentValues.voltage,
							     _currentValues.current,
								 _currentValues.power,
								 _currentValues.energy,
								_currentValues.frequeny,
								udids
								);
					}

					size=strlen((const char*)buf);


					//sending answer
					while(size != sentsize){
						ret = send(sn,buf+sentsize,size-sentsize);
						if(ret < 0){
							close(sn);
							return ret;
						}
						sentsize += ret; // Don't care SOCKERR_BUSY, because it is zero.
					}
				}
				//ending
				disconnect(sn);

			}
			break;
		case SOCK_CLOSE_WAIT :
//			printf("%d:CloseWait\r\n",sn);
			if((ret=disconnect(sn)) != SOCK_OK) return ret;
//			printf("%d:Closed\r\n",sn);
			break;
		case SOCK_INIT :
//			printf("%d:Listen, port [%d]\r\n",sn, port);
			if( (ret = listen(sn)) != SOCK_OK) return ret;
			break;
		case SOCK_CLOSED:
//			printf("%d:LBTStart\r\n",sn);
			if((ret=socket(sn,Sn_MR_TCP,port,0x00)) != sn)
			return ret;
	//		printf("%d:Opened\r\n",sn);
			break;
		default:
			break;
	}
	return 1;
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	udid[0] =  *((unsigned long *)0x1FFFF7F0);
	udid[1] =  *((unsigned long *)0x1FFFF7EC);
	udid[2] =  *((unsigned long *)0x1FFFF7E8);
	sprintf(udids, "%08X-%08X-%08X",udid[0], udid[1],udid[2]);
	uint8_t tmp;

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */


  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  w55500init();
  /* USER CODE END 2 */
 


  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint8_t link = 1;
  uint8_t prev_link = 0;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  HAL_Delay(100);
	  if(link == 1){
		  HAL_UART_Receive_IT(&huart1, response, 25);
		  updateValues();
	  }
//	  HAL_GPIO_TogglePin(PIN_LED_GPIO_Port, PIN_LED_Pin);
	  loopback_tcps(HTTP_SOCKET,gDATABUF, 80);
	  HAL_Delay(100);
	  if(ctlwizchip(CW_GET_PHYLINK, (void*)&tmp) == 0){
		  if(tmp == PHY_LINK_OFF){
			  link = 0;
		  }else{
			  link = 1;
		  }
	  }
	  if(prev_link!=link){
		  dhcp_ctr = 10000;
		  ip_assigned = false;
		  while((!ip_assigned) && (dhcp_ctr > 0)) {
		  		DHCP_run();
		  		dhcp_ctr--;
		  	}
	  }
	  prev_link = link;
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PIN_LED_GPIO_Port, PIN_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(EHERNET_CS_GPIO_Port, EHERNET_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PIN_LED_Pin */
  GPIO_InitStruct.Pin = PIN_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PIN_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : EHERNET_CS_Pin */
  GPIO_InitStruct.Pin = EHERNET_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(EHERNET_CS_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
