/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  *
  * Demo application to connect to a cloud based MQTT broker
  * Data communication is un-encrypted using port 1883
  *
  * Before running the code: -
  * 1. open /Core/Inc/WiFi_Credentials.h and enter WiFi access point SSID and password
  * 2. open /Core/Inc/CloudBrokerCredentials.h and enter your broker credentials
  * 3. Enter the subscribe topic name in line 376
  * 4. Enter the publish topic name in line 426
  *
  * Application Functionality
  * 1. Fetches the epoch time from st.com and uses it to start the RTC with a 1-second wake-up interrupt
  * 2. Connects to wireless access point and then to cloud DNS
  * 3. Connects to MQTT broker
  * 4. Subscribes to a single topic
  * 5. Prints date and time every second
  * 6. Publishes to a topic every 5 seconds (uses Tim6 periodic interrupt)
  * 7. Subscribe callback function runs every time a publish message is received from the broker
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "wifi.h"
#include "net_internal.h"
#include "..\..\Common\Shared\Inc\net.h"
#include "stm32l475e_iot01_tsensor.h"
#include "stm32l475e_iot01_psensor.h"
#include "WiFi_Credentials.h"
#include "CloudBrokerCredentials.h"
#include "timedate.h"
#include <time.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CONNECTION_TRIAL_MAX          10

#define WIFI_WRITE_TIMEOUT 10000
#define WIFI_READ_TIMEOUT  10000

#define CONNECTION_TRIAL_MAX          10

#define MQTT_SEND_BUFFER_SIZE             600
#define MQTT_READ_BUFFER_SIZE             600
#define MQTT_TOPIC_BUFFER_SIZE            100  /**< Maximum length of the application-defined topic names. */
#define MQTT_MSG_BUFFER_SIZE              MQTT_SEND_BUFFER_SIZE /**< Maximum length of the application-defined MQTT messages. */
#define MQTT_CMD_TIMEOUT                  5000
#define MODEL_MAC_SIZE                    13
#define MODEL_DEFAULT_MAC                 "0102030405"


#define TURBIDITY_DATA_12_BIT               0x0FFF
#define TURBIDITY_VREF_5000_mV              5000.0
#define TURBIDITY_ADC_RESOLUTION            4096.0
#define TURBIDITY_ADC_VOLTAGE_MAX           4600.0
#define TURBIDITY_ADC_VOLTAGE_MIN           1600.0
/**
 * @brief Nephelometric Turbidity Units claculation values.
 * @details Definition of Nephelometric Turbidity Units claculation data.
 */
#define TURBIDITY_NTU_CALC_FACT_0           0.8871
#define TURBIDITY_NTU_CALC_FACT_1           1.0571
#define TURBIDITY_NTU_CALC_FACT_2           1.1200
#define TURBIDITY_NTU_CALC_FACT_3           1.3333
#define TURBIDITY_NTU_CALC_FACT_4           1.6000
#define TURBIDITY_NTU_MIN                   0.0000
#define TURBIDITY_NTU_MAX                   4000.0
#define TURBIDITY_VOLTAGE_LEVEL_1           2650.0
#define TURBIDITY_VOLTAGE_LEVEL_2           2930.0
#define TURBIDITY_VOLTAGE_LEVEL_3           3350.0
#define TURBIDITY_VOLTAGE_LEVEL_4           3800.0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 DFSDM_Channel_HandleTypeDef hdfsdm1_channel0;

I2C_HandleTypeDef hi2c3;

QSPI_HandleTypeDef hqspi;

RNG_HandleTypeDef hrng;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
uint8_t timeDisplay = 0, readSensor = 0;
net_hnd_t hnet;

typedef struct {
  char *HostName;
  char *HostPort;
  char *ConnSecurity;
  char *MQClientId;
  char *MQUserName;
  char *MQUserPwd;
#ifdef LITMUS_LOOP
  char *LoopTopicId;
#endif
} device_config_t;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM6_Init(void);
static void MX_RNG_Init(void);
static void MX_I2C3_Init(void);
static void MX_DFSDM1_Init(void);
/* USER CODE BEGIN PFP */
int network_wr(Network* n, unsigned char* buffer, int len, int timeout_ms);
int network_rd(Network* n, unsigned char* buffer, int len, int timeout_ms);

extern int net_if_init(void * if_ctxt);
extern int net_if_deinit(void * if_ctxt);
extern int net_if_reinit(void * if_ctxt);
extern int wifi_net_if_init(void * if_ctxt);

void SPI_WIFI_ISR(void);


int8_t turbidity_get_ntu ( uint32_t sample, float * ntu );
//void HAL_GPIO_WritePin();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len) {
	HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, HAL_MAX_DELAY);
	return len;
}

/**-------------------------------------------------------------------------------
  * @brief  EXTI line detection callback.
  * @param  GPIO_Pin: Specifies the port pin connected to corresponding EXTI line.
  * @retval None
  ------------------------------------------------------------------------------*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin)
  {
	case (GPIO_PIN_1):
	{
		SPI_WIFI_ISR();
		break;
	}

    default:
    {
      break;
    }
  }
}

/*------------------------------------------------------------------
 * RTC timer event callback function
 * Runs every second
 ------------------------------------------------------------------*/
void HAL_RTCEx_WakeUpTimerEventCallback (RTC_HandleTypeDef * hrtc) {
	timeDisplay = 1;
}

/*--------------------------------------------------------------
 * Timer 6 overflow callback function
 --------------------------------------------------------------*/
void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef * htim) {
	if(htim->Instance == TIM6) {
		readSensor = 1;
	}
}

/*--------------------------------------------------------------------------
 * Subscribe message callback function
 * Called every time a publish meassage is received from a subscribed topic
 --------------------------------------------------------------------------*/
void subscribeMessageHandler(MessageData* data)
{
	static char mqtt_msg[MQTT_MSG_BUFFER_SIZE], mqtt_topic[MQTT_TOPIC_BUFFER_SIZE];
	snprintf(mqtt_msg, data->message->payloadlen+1, "%s", (char *)data->message->payload);
	snprintf(mqtt_topic, data->topicName->lenstring.len+1, "%s", data->topicName->lenstring.data);
	printf("\r\nPublished message from MQTT broker\r\n");
	printf("Topic: %s, Payload: %s\r\n\n", mqtt_topic, mqtt_msg);
	if(strstr(mqtt_msg, "ON")) {
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
	}
	else if(strstr(mqtt_msg, "OFF")){
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	char speedMsg[50];
	float turbidity;
	int32_t ret;


//	uint8_t buffer[20];
//	uint32_t sample;
//	uint32_t voltage_mv;
//	float ntuReading;

	uint8_t buffer[20];
	uint32_t sample;
	uint32_t voltage_mv;

	//Network and MQTT variables
	device_config_t MQTT_Config;
	MQTTMessage mqmsg;
	MQTTClient client;
	Network network;
	MQTTPacket_connectData options = MQTTPacket_connectData_initializer;
	static unsigned char mqtt_send_buffer[MQTT_SEND_BUFFER_SIZE];
	static unsigned char mqtt_read_buffer[MQTT_READ_BUFFER_SIZE];
	net_sockhnd_t socket;
	net_ipaddr_t ipAddr;
	net_macaddr_t macAddr;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_QUADSPI_Init();
  MX_SPI3_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_RTC_Init();
  MX_TIM6_Init();
  MX_RNG_Init();
  MX_I2C3_Init();
  MX_DFSDM1_Init();
  /* USER CODE BEGIN 2 */
  printf("HydroSystem connecting to Ubidots STEM...\r\n\n");

  //LED off at initialisation
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

  //Initialise MQTT broker structure
  //Fill in this section with MQTT broker credentials from header file
  MQTT_Config.HostName = CloudBroker_HostName;
  MQTT_Config.HostPort = CloudBroker_Port;
  MQTT_Config.ConnSecurity = "0";	//plain TCP connection with no security
  MQTT_Config.MQUserName = CloudBroker_Username;
  MQTT_Config.MQUserPwd = CloudBroker_Password;
  MQTT_Config.MQClientId = CloudBroker_ClientID;

  //Initialise WiFi network
  if (net_init(&hnet, NET_IF, (wifi_net_if_init)) != NET_OK) {
	  printf("\n\rError");
  }
  else {
	  printf("\n\rOK");
  }
  HAL_Delay(500);

  printf("\n\rRetrieving the IP address.");

  if (net_get_ip_address(hnet, &ipAddr) != NET_OK) {
	  printf("\n\rError 2");
  }
  else
  {
    switch(ipAddr.ipv) {
      case NET_IP_V4:
        printf("\n\rIP address: %d.%d.%d.%d\n\r", ipAddr.ip[12], ipAddr.ip[13], ipAddr.ip[14], ipAddr.ip[15]);
        break;
      case NET_IP_V6:
      default:
    	  printf("\n\rError 3");
    }
  }

  if (net_get_mac_address(hnet, &macAddr) == NET_OK) {
      printf("\n\rMac Address: %02x:%02x:%02x:%02x:%02x:%02x\r\n",
               macAddr.mac[0], macAddr.mac[1], macAddr.mac[2], macAddr.mac[3], macAddr.mac[4], macAddr.mac[5]);
  }

  /*
   * Fetch the epoch time from st.com and use it to set the RTC time
   */
  if (setRTCTimeDateFromNetwork(true) != TD_OK) {
	  printf("Fail setting time\r\n");
  }
  else {
	  printf("Time set, Starting RTC\r\n");
	  //RTC started with a 1-second wake-up interrupt
	  HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 2047, RTC_WAKEUPCLOCK_RTCCLK_DIV16);
  }

  printf("Connecting to MQTT Broker\r\n\n");
  //Create network socket
  ret = net_sock_create(hnet, &socket, NET_PROTO_TCP);
  if (ret != NET_OK)
  {
    printf("\n\rCould not create the socket.\r\n");
  }
  else
  {
	  ret |= net_sock_setopt(socket, "sock_noblocking", NULL, 0);
  }

  ret = net_sock_open(socket, MQTT_Config.HostName, 1883, 0);
  if (ret != NET_OK)
	{
	  printf("\n\rCould not open the socket.");
	  HAL_Delay(1000);
	}
  else {
	  printf("\n\rConnected to server");
	  HAL_Delay(1000);
  }

  network.my_socket = socket;
  network.mqttread = (network_rd);
  network.mqttwrite = (network_wr);

  MQTTClientInit(&client, &network, MQTT_CMD_TIMEOUT, mqtt_send_buffer, MQTT_SEND_BUFFER_SIZE,
	 mqtt_read_buffer, MQTT_READ_BUFFER_SIZE);

  /* MQTT connect */
  options.clientID.cstring = MQTT_Config.MQClientId;
  options.username.cstring = MQTT_Config.MQUserName;
  options.password.cstring = MQTT_Config.MQUserPwd;

  HAL_Delay(1000);

  ret = MQTTConnect(&client, &options);
  if (ret != 0)
  {
	printf("\n\rMQTTConnect() failed: %ld\n", ret);
  }
  else
  {
	printf("\n\rConnected to MQTT Broker");
	HAL_Delay(1000);
  }
  HAL_Delay(1000);

  //Add code here to subscribe to topics
  ret = MQTTSubscribe(&client, "/v1.6/devices/hydrosystem/motor", QOS0, (subscribeMessageHandler));
  if (ret != MQSUCCESS) {
	printf("\n\rSubscribe failed: %ld\n\r", ret);
  }
  else {
	printf("\n\rSubscribed to topic \n\r");
	ret = MQTTYield(&client, 500);
  }
  HAL_Delay(1000);

  HAL_TIM_Base_Start_IT(&htim6);	//start timer 6. Used for periodic interrupt


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	/*
	 * Need to yield to test for a received publish message from broker
	 */
	ret = MQTTYield(&client, 500);


	HAL_I2C_Master_Receive(&hi2c3, 0x9B, buffer, 2, 1000);
	sample = (((uint16_t)(buffer[0]))<<8) + buffer[1];
	voltage_mv = 5000 * sample/4096;
	printf("Sample: 0x%04x %lu decimal, voltage %lumV\r\n", sample, sample, voltage_mv);
	turbidity_get_ntu(sample, &turbidity);
	printf("NTU Reading: %f\r\n\n", turbidity);


	sprintf(speedMsg, "{\"Turbidity\": {\"value\": %f}}", turbidity);
	printf("%s\n\r", speedMsg);
	memset(&mqmsg, 0, sizeof(MQTTMessage));
	mqmsg.qos = QOS0;
	mqmsg.payload = (char *) speedMsg;
	mqmsg.payloadlen = strlen(speedMsg);
	MQTTPublish(&client, "/v1.6/devices/hydrosystem", &mqmsg);
	HAL_Delay(5000);



	if(turbidity > 3050){
		printf("Feeding plants\r\n");
		printf("Motor opening feed hatch\r\n\n");
		HAL_GPIO_WritePin(IN1A_GPIO_Port, IN1A_Pin, 0);
		HAL_GPIO_WritePin(IN2A_GPIO_Port, IN2A_Pin, 1);
		HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, 1);//rotates motor
		HAL_Delay(3500);
		HAL_GPIO_WritePin(IN1A_GPIO_Port, IN1A_Pin, 0);
		HAL_GPIO_WritePin(IN2A_GPIO_Port, IN2A_Pin, 0);
		HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, 1);//stops motor
		HAL_Delay(1500);
		HAL_GPIO_WritePin(IN1A_GPIO_Port, IN1A_Pin, 1);
		HAL_GPIO_WritePin(IN2A_GPIO_Port, IN2A_Pin, 0);
		HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, 1);//reverses motor
		HAL_Delay(3500);
		HAL_GPIO_WritePin(IN1A_GPIO_Port, IN1A_Pin, 0);
		HAL_GPIO_WritePin(IN2A_GPIO_Port, IN2A_Pin, 0);
		HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, 1);//stops motor
	}else{
		HAL_GPIO_WritePin(IN1A_GPIO_Port, IN1A_Pin, 0);
		HAL_GPIO_WritePin(IN2A_GPIO_Port, IN2A_Pin, 0);
		HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, 1);//stops motor
	}
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_LSE
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_RNG;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInit.RngClockSelection = RCC_RNGCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DFSDM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DFSDM1_Init(void)
{

  /* USER CODE BEGIN DFSDM1_Init 0 */

  /* USER CODE END DFSDM1_Init 0 */

  /* USER CODE BEGIN DFSDM1_Init 1 */

  /* USER CODE END DFSDM1_Init 1 */
  hdfsdm1_channel0.Instance = DFSDM1_Channel0;
  hdfsdm1_channel0.Init.OutputClock.Activation = DISABLE;
  hdfsdm1_channel0.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel0.Init.OutputClock.Divider = 2;
  hdfsdm1_channel0.Init.Input.Multiplexer = DFSDM_CHANNEL_INTERNAL_REGISTER;
  hdfsdm1_channel0.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel0.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
  hdfsdm1_channel0.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel0.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_EXTERNAL;
  hdfsdm1_channel0.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel0.Init.Awd.Oversampling = 1;
  hdfsdm1_channel0.Init.Offset = 0x00;
  hdfsdm1_channel0.Init.RightBitShift = 0x00;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DFSDM1_Init 2 */

  /* USER CODE END DFSDM1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x10909CEC;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief QUADSPI Initialization Function
  * @param None
  * @retval None
  */
static void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  /* QUADSPI parameter configuration*/
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 2;
  hqspi.Init.FifoThreshold = 4;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
  hqspi.Init.FlashSize = 23;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */

  /* USER CODE END QUADSPI_Init 2 */

}

/**
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
static void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 8000;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 49999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, M24SR64_Y_RF_DISABLE_Pin|M24SR64_Y_GPO_Pin|ISM43362_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, IN2A_Pin|IN1A_Pin|ENABLE_Pin|VL53L0X_XSHUT_Pin
                          |LED3_WIFI__LED4_BLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ARD_D10_Pin|SPBTLE_RF_RST_Pin|ARD_D9_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ARD_D8_Pin|ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin|LED2_Pin
                          |SPSGRF_915_SDN_Pin|ARD_D5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, USB_OTG_FS_PWR_EN_Pin|PMOD_RESET_Pin|STSAFE_A100_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPBTLE_RF_SPI3_CSN_GPIO_Port, SPBTLE_RF_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPSGRF_915_SPI3_CSN_GPIO_Port, SPSGRF_915_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ISM43362_SPI3_CSN_GPIO_Port, ISM43362_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : M24SR64_Y_RF_DISABLE_Pin M24SR64_Y_GPO_Pin ISM43362_RST_Pin ISM43362_SPI3_CSN_Pin */
  GPIO_InitStruct.Pin = M24SR64_Y_RF_DISABLE_Pin|M24SR64_Y_GPO_Pin|ISM43362_RST_Pin|ISM43362_SPI3_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_OVRCR_EXTI3_Pin SPSGRF_915_GPIO3_EXTI5_Pin SPBTLE_RF_IRQ_EXTI6_Pin ISM43362_DRDY_EXTI1_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_OVRCR_EXTI3_Pin|SPSGRF_915_GPIO3_EXTI5_Pin|SPBTLE_RF_IRQ_EXTI6_Pin|ISM43362_DRDY_EXTI1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_EXTI13_Pin */
  GPIO_InitStruct.Pin = BUTTON_EXTI13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_EXTI13_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : IN2A_Pin IN1A_Pin ENABLE_Pin VL53L0X_XSHUT_Pin
                           LED3_WIFI__LED4_BLE_Pin */
  GPIO_InitStruct.Pin = IN2A_Pin|IN1A_Pin|ENABLE_Pin|VL53L0X_XSHUT_Pin
                          |LED3_WIFI__LED4_BLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D1_Pin ARD_D0_Pin */
  GPIO_InitStruct.Pin = ARD_D1_Pin|ARD_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D10_Pin SPBTLE_RF_RST_Pin ARD_D9_Pin */
  GPIO_InitStruct.Pin = ARD_D10_Pin|SPBTLE_RF_RST_Pin|ARD_D9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D4_Pin */
  GPIO_InitStruct.Pin = ARD_D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(ARD_D4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D7_Pin */
  GPIO_InitStruct.Pin = ARD_D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D7_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D13_Pin ARD_D12_Pin ARD_D11_Pin */
  GPIO_InitStruct.Pin = ARD_D13_Pin|ARD_D12_Pin|ARD_D11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D3_Pin */
  GPIO_InitStruct.Pin = ARD_D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D6_Pin */
  GPIO_InitStruct.Pin = ARD_D6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D6_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D8_Pin ISM43362_BOOT0_Pin ISM43362_WAKEUP_Pin LED2_Pin
                           SPSGRF_915_SDN_Pin ARD_D5_Pin SPSGRF_915_SPI3_CSN_Pin */
  GPIO_InitStruct.Pin = ARD_D8_Pin|ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin|LED2_Pin
                          |SPSGRF_915_SDN_Pin|ARD_D5_Pin|SPSGRF_915_SPI3_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LPS22HB_INT_DRDY_EXTI0_Pin LSM6DSL_INT1_EXTI11_Pin ARD_D2_Pin HTS221_DRDY_EXTI15_Pin */
  GPIO_InitStruct.Pin = LPS22HB_INT_DRDY_EXTI0_Pin|LSM6DSL_INT1_EXTI11_Pin|ARD_D2_Pin|HTS221_DRDY_EXTI15_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_PWR_EN_Pin SPBTLE_RF_SPI3_CSN_Pin PMOD_RESET_Pin STSAFE_A100_RESET_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_PWR_EN_Pin|SPBTLE_RF_SPI3_CSN_Pin|PMOD_RESET_Pin|STSAFE_A100_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : VL53L0X_GPIO1_EXTI7_Pin LSM3MDL_DRDY_EXTI8_Pin */
  GPIO_InitStruct.Pin = VL53L0X_GPIO1_EXTI7_Pin|LSM3MDL_DRDY_EXTI8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PMOD_SPI2_SCK_Pin */
  GPIO_InitStruct.Pin = PMOD_SPI2_SCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PMOD_SPI2_SCK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PMOD_UART2_CTS_Pin PMOD_UART2_RTS_Pin PMOD_UART2_TX_Pin PMOD_UART2_RX_Pin */
  GPIO_InitStruct.Pin = PMOD_UART2_CTS_Pin|PMOD_UART2_RTS_Pin|PMOD_UART2_TX_Pin|PMOD_UART2_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D15_Pin ARD_D14_Pin */
  GPIO_InitStruct.Pin = ARD_D15_Pin|ARD_D14_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
int8_t turbidity_get_ntu ( uint32_t sample, float * ntu )
{
    float voltage;
    float coeff_c;

    voltage = (float)sample;
    voltage *= TURBIDITY_VREF_5000_mV;
    voltage /= TURBIDITY_ADC_RESOLUTION;
    printf("Input voltage: %fmV\r\n", voltage);

    coeff_c = TURBIDITY_NTU_CALC_FACT_0;

    if ( voltage > TURBIDITY_VOLTAGE_LEVEL_1 )
    {
        coeff_c = TURBIDITY_NTU_CALC_FACT_1;
    }

    if ( voltage > TURBIDITY_VOLTAGE_LEVEL_2 )
    {
        coeff_c = TURBIDITY_NTU_CALC_FACT_2;
    }

    if ( voltage > TURBIDITY_VOLTAGE_LEVEL_3 )
    {
        coeff_c = TURBIDITY_NTU_CALC_FACT_3;
    }

    if ( voltage > TURBIDITY_VOLTAGE_LEVEL_4 )
    {
        coeff_c = TURBIDITY_NTU_CALC_FACT_4;
    }

    *ntu = TURBIDITY_ADC_VOLTAGE_MAX;
    *ntu -= voltage;
    *ntu /= coeff_c;

    if ( voltage > TURBIDITY_ADC_VOLTAGE_MAX )
    {
        *ntu = TURBIDITY_NTU_MIN;
    }

    if ( voltage < TURBIDITY_ADC_VOLTAGE_MIN )
    {
        *ntu = TURBIDITY_NTU_MAX;
    }

    return 0;
}

void Led_SetState(bool on)
{
  if (on == true)
  {
    BSP_LED_On(LED_GREEN);
  }
  else
  {
    BSP_LED_Off(LED_GREEN);
  }
}


/**
 * @brief Blink LED for 'count' cycles of 'period' period and 'duty' ON duration.
 * duty < 0 tells to start with an OFF state.
 */
void Led_Blink(int period, int duty, int count)
{
  if ( (duty > 0) && (period >= duty) )
  {
    /*  Shape:   ____
                  on |_off__ */
    do
    {
      Led_SetState(true);
      HAL_Delay(duty);
      Led_SetState(false);
      HAL_Delay(period - duty);
    } while (count--);
  }
  if ( (duty < 0) && (period >= -duty) )
  {
    /*  Shape:         ____
                __off_| on   */
    do
    {
      Led_SetState(false);
      HAL_Delay(period + duty);
      Led_SetState(true);
      HAL_Delay(-duty);
    } while (count--);
  }
}



void SPI3_IRQHandler(void)
{
  HAL_SPI_IRQHandler(&hspi);
}/** Function to read data from the socket opened into provided buffer
 * @param - Address of Network Structure
 *        - Buffer to store the data read from socket
 *        - Expected number of bytes to read from socket
 *        - Timeout in milliseconds
 * @return - Number of Bytes read on SUCCESS
 *         - -1 on FAILURE
 **/
int network_rd(Network* n, unsigned char* buffer, int len, int timeout_ms)
{
  int bytes;

  bytes = net_sock_recv((net_sockhnd_t) n->my_socket, buffer, len);
  if(bytes < 0)
  {
    msg_error("net_sock_recv failed - %d\n", bytes);
    bytes = -1;
  }

   return bytes;
}

/** Function to write data to the socket opened present in provided buffer
 * @param - Address of Network Structure
 *        - Buffer storing the data to write to socket
 *        - Number of bytes of data to write to socket
 *        - Timeout in milliseconds
 * @return - Number of Bytes written on SUCCESS
 *         - -1 on FAILURE
 **/
int network_wr(Network* n, unsigned char* buffer, int len, int timeout_ms)
{
  int rc;

  rc = net_sock_send((net_sockhnd_t) n->my_socket, buffer, len);
  if(rc < 0)
  {
    msg_error("net_sock_send failed - %d\n", rc);
    rc = -1;
  }

  return rc;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
