/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
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
#include <stdio.h>
#include <stdbool.h>
#include "ssd1306/ssd1306.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_BUF_LEN 3

#define PWM_BUFFER_SIZE 32
#define PWM_HALFBUFFER_SIZE 16
#define DITHER_RES 8

enum rotary_direction {
	DIRECTION_NONE,
	DIRECTION_CW,
	DIRECTION_CCW,
	NUM_OF_DIRECTIONS
};

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim22;
DMA_HandleTypeDef hdma_tim2_ch2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static uint8_t rotary_state;
static uint8_t rotary_counter = 0;
static int8_t rotary_direction = DIRECTION_NONE;

uint16_t adc_buf[ADC_BUF_LEN];

// Tramage (dithering)
uint16_t pwm_buffer[PWM_BUFFER_SIZE];
uint8_t pwm_pulse = 0;

//Asservissement / regulation
int32_t integration_sum = 0.0;
uint32_t pwm_pulse_BF = 0;
uint8_t Vref_BF = 0;
// sur chaque valeurs des coeficiant de calcul, on applique un facteur 10 puissance 10 pour manipuler des int
int32_t dt = 256; //la periode d'échantillonage
int32_t Kp  = 500000;
int32_t Ki = 400000;

//variables des mesures de l'ADC
float vin_value =  0.0;
float vout_value = 0.0;
float iout_value = 0.0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM22_Init(void);
/* USER CODE BEGIN PFP */

// State management
void Set_Next_Mode(void);
void Set_Previous_Mode(void);

// Human interface
void Update_Display(void);
bool Button_Pushed(){
	if(HAL_GPIO_ReadPin(GPIOA, Button_Pin) == 0){
		return true;
	}else{
		return false;
	}
}


// USB Power Delivery requests
uint8_t Read_USB_PDO(void);
void Test_STUSB_I2C(void);
void Set_STUSB_Available_Profiles(uint8_t num_profiles);

// Voltage controller
int32_t Compute_control_input(int32_t y_setpoint, int32_t y_value);
void TransferComplete(DMA_HandleTypeDef *hdma);
void TransferHalfComplete(DMA_HandleTypeDef *hdma);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

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
  MX_DMA_Init();
  MX_ADC_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM22_Init();
  /* USER CODE BEGIN 2 */

	// Initialize screen
	ssd1306_Init();
	ssd1306_Fill(Black);
	ssd1306_UpdateScreen();

	//  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
	HAL_ADC_Start_DMA(&hadc, (uint32_t*) adc_buf, ADC_BUF_LEN);

	// Start PWM timer
	HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_2, (uint32_t*) pwm_buffer, PWM_BUFFER_SIZE);

	// Start main timer
	htim2.hdma[TIM_DMA_ID_CC2]->XferCpltCallback = TransferComplete;
	htim2.hdma[TIM_DMA_ID_CC2]->XferHalfCpltCallback = TransferHalfComplete;
	HAL_TIM_Base_Start_IT(&htim22);

	// Switch off user LED
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	// TODO
	// 1.1 Menu et navigation


	//déclaration des variables utilisateur
	uint8_t Vout_BO = 0;
	uint8_t Vout_BF = 0;
	uint8_t type_commande = 2; // 0 = BO, 1 = BF, 2 = PDO

	// définition des variable pour l'affichage des mesures
	int vin_value_int_u = 0;
	int vin_value_int_d = 0;

	int vout_value_int_u = 0;
	int vout_value_int_d = 0;

	int iout_value_int_u = 0;
	int iout_value_int_d = 0;


	//déclaration des variable définissant les modes d'utilisations des différents menus (mode 0 = affichage, mode 1 = modification)
	uint8_t mode = 0;
	uint8_t menu = 0;

	//déclaration du numéro de menu max ategnable
	int menu_max = 3;

	//déclaration des variables texte que l'on affichera sur l'écran
	char line1_str[20] = {0};
	char line2_str[20] = {0};
	char line3_str[20] = {0};

	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		//les lignes de code suivante permettent de gérer les différent menus de l'IHM
		//dans l'ordre, les menus permettent de :
		//afficher les valeurs mesurée des tensions et courants
		//selectionner le type de commande que l'on veut pour notre convertisseur (BO, BF ou PDO)
		//définir les paramètre de la commande en Boucle ouverte
		//définir les paramètre de la commande en Boucle fermé

		//L'interface à 2 modes différent, un mode de sélection de menu, et un mode de définission des paramètres.

		//pour le premier mode de sélection des menus :
		if(mode == 0){
			// premier menu, affichage du courant et des tensions mesurées
			if(rotary_counter == 0){
				sprintf(line1_str, "affichage mesures ", rotary_counter);
				sprintf(line2_str, "Ve:%02u,%02u Vs:%02u,%02u ", vin_value_int_u, vin_value_int_d, vout_value_int_u, vout_value_int_d);
				sprintf(line3_str, "Io:%02u,%03u         ", iout_value_int_u, iout_value_int_d);
			}
			// deuxième menu, selection du type de commande
			if(rotary_counter == 1){

				sprintf(line1_str, "Selection commande");
				sprintf(line2_str, "                   ");

				if(type_commande == 0){
					sprintf(line3_str, "Commande = BO     ");
				}
				if(type_commande == 1){
					sprintf(line3_str, "Commande = BF     ");
				}
				if(type_commande == 2){
					sprintf(line3_str, "Commande = PDO    ");
				}
				//si le boutton est pressé, on passe au mode de modification de la valeur
				if(Button_Pushed()){
					for(int i=0; i<100000; i++);
					while(Button_Pushed());
					for(int i=0; i<100000; i++);
					mode = 1;
					menu = 1;
					rotary_counter = type_commande;
				}
			}
			// troisième menu, selection des paramètre de la commande en boucle ouverte
			if(rotary_counter == 2){

				sprintf(line1_str, "Boucle Ouverte    ");
				sprintf(line2_str, "                  ");
				sprintf(line3_str, "Vout = %02u         ", Vout_BO);
				//si le boutton est pressé, on passe au mode de modification de la valeur
				if(Button_Pushed()){
					for(int i=0; i<100000; i++);
					while(Button_Pushed());
					for(int i=0; i<100000; i++);
					mode = 1;
					menu = 2;
					rotary_counter = Vout_BO;
				}
			}
			// quatrième menu, selection des paramètre de la commande en boucle fermée
			if(rotary_counter == 3){

				sprintf(line1_str, "Boucle Fermee     ");
				sprintf(line2_str, "                  ");
				sprintf(line3_str, "Vout = %02u         ", Vout_BF);
				//si le boutton est pressé, on passe au mode de modification de la valeur
				if(Button_Pushed()){
					for(int i=0; i<100000; i++);
					while(Button_Pushed());
					for(int i=0; i<100000; i++);
					mode = 1;
					menu = 3;
					rotary_counter = Vout_BF;
				}
			}

			// si on dépasse le quatrième menu, on reviens au premier menu
			if(rotary_counter > menu_max){
				rotary_counter = 0;
			}

		}

		//pour le deuxième mode de modifications des valeurs
		if(mode == 1){
			// on peu modifier la valeur du type de la commande (deuxième menu)
			if(menu == 1){
				sprintf(line1_str, "Selection commande");
				sprintf(line2_str, "                  ");

				if(rotary_counter == 0){
					sprintf(line3_str, "Commande = [BO]   ");
					type_commande = 0;
				}
				if(rotary_counter == 1){
					sprintf(line3_str, "Commande = [BF]   ");
					type_commande = 1;
				}
				if(rotary_counter == 2){
					sprintf(line3_str, "Commande = [PDO]  ");
					type_commande = 2;
				}

				if(rotary_counter > 2){
					rotary_counter = 2;
				}
				if(Button_Pushed()){
					for(int i=0; i<100000; i++);
					while(Button_Pushed());
					for(int i=0; i<100000; i++);
					mode = 0;
					rotary_counter = menu;
				}

			}
			// on peu modifier la valeur de la tension désirée pour la commande en boucle ouverte (troisième menu)
			if(menu == 2){
				sprintf(line1_str, "Boucle Ouverte    ");
				sprintf(line2_str, "                  ");
				sprintf(line3_str, "Vout = [%02u]       ", Vout_BO);
				Vout_BO = rotary_counter;
				if(rotary_counter > 15){
					rotary_counter = 15;
				}
				if(Button_Pushed()){
					for(int i=0; i<100000; i++);
					while(Button_Pushed());
					for(int i=0; i<100000; i++);
					mode = 0;
					rotary_counter = menu;
				}
			}
			// on peu modifier la valeur de la tension désirée pour la commande en boucle fermée (quatrième menu)
			if(menu == 3){
				sprintf(line1_str, "Boucle Fermée     ");
				sprintf(line2_str, "                  ");
				sprintf(line3_str, "Vout = [%02u]       ", rotary_counter);
				if(rotary_counter > 15){
					rotary_counter = 15;
				}
				if(Button_Pushed()){
					for(int i=0; i<100000; i++);
					while(Button_Pushed());
					for(int i=0; i<100000; i++);
					Vout_BF = rotary_counter;
					mode = 0;
					rotary_counter = menu;
				}
			}
		}

		// TODO
		// 2.1 Conversion Analogique-Numérique
		// 2.2 Etalonage

		// dans les lignes suivante, on viens diviser les valeurs de type float en valeurs de type int, affichable à l'écran.
		vin_value_int_u = (int)vin_value;
		vin_value_int_d = (int)((vin_value - vin_value_int_u)*100);

		vout_value_int_u = (int)vout_value;
		vout_value_int_d = (int)((vout_value - vout_value_int_u)*100);

		iout_value_int_u = (int)iout_value;
		iout_value_int_d = (int)((iout_value - iout_value_int_u)*1000);

		// TODO
		// 3.1 Gestion PWM
		if(type_commande == 0){
			// Convertit la valeur voulu de Vout en une valeur du rapport cyclique (entre 0 et 255)
			// 16 bit pour gérer la valeur max : 255
			pwm_pulse = (((float)Vout_BO/(float)vin_value) * 255.0);
		}

		if(type_commande == 1){
			// Si nous voulons une commande en boucle fermé, nous utilisons la fonction de calcul
			// du rapport cyclique qui viens le modifier en temps réel en fonction de Vout voulu et Vout mesuré (décidé par l'utilisateur).
			Vref_BF = Vout_BF;
			pwm_pulse = ((float)pwm_pulse_BF/100000000.0)*255.0;
		}

		// TODO
		// 4.1 Lecture PDO et asservissement

		if(type_commande == 2){
			// Si nous voulons une commande en PDO, nous utilisons la fonction de calcul
			// du rapport cyclique qui viens le modifier en temps réel en fonction de Vout voulu et Vout mesuré (décidé en fonction des broches de communication de la sortie).

			//si le pin PDO5 est mis au GND, on demande 9V
			if(HAL_GPIO_ReadPin(GPIOA, PDO2_Pin) < 0.5){
				Vref_BF = 9;
				HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);
			}
			//si le pin PDO4 est mis au GND, on demande 12V
			else if(HAL_GPIO_ReadPin(GPIOA, PDO3_Pin) < 0.5){
				Vref_BF = 12;
			}
			//si le pin PDO3 est mis au GND, on demande 15V
			else if(HAL_GPIO_ReadPin(GPIOB, PDO4_Pin) < 0.5){
				Vref_BF = 15;
			}
			//si aucun pin est mis au GND, on demande 5V
			else{
				Vref_BF = 5;
			}
			pwm_pulse = ((float)pwm_pulse_BF/100000000.0)*255.0;
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
		}

//HAL_GPIO_ReadPin(PDO5_GPIO_Port, PDO5_Pin)




		// TODO
		// 2.3 Calcul énergie et Puissance

		// rafraichissement de l'écran en fonction des variable de texte modifié précédement !
		ssd1306_SetCursor(0, 0);
		ssd1306_WriteString(line1_str, Font_7x10, White);
		ssd1306_SetCursor(0, 11);
		ssd1306_WriteString(line2_str, Font_7x10, White);
		ssd1306_SetCursor(0, 21);
		ssd1306_WriteString(line3_str, Font_7x10, White);
		ssd1306_UpdateScreen();

		// Delay
		HAL_Delay(25);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.SamplingTime = ADC_SAMPLETIME_79CYCLES_5;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T22_TRGO;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerFrequencyMode = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x0060112F;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 256-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM22 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM22_Init(void)
{

  /* USER CODE BEGIN TIM22_Init 0 */

  /* USER CODE END TIM22_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM22_Init 1 */

  /* USER CODE END TIM22_Init 1 */
  htim22.Instance = TIM22;
  htim22.Init.Prescaler = 1-1;
  htim22.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim22.Init.Period = 32-1;
  htim22.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim22.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim22) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim22) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_ITR1;
  if (HAL_TIM_SlaveConfigSynchro(&htim22, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim22, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim22, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM22_Init 2 */

  /* USER CODE END TIM22_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DMA_Debug_GPIO_Port, DMA_Debug_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SHDN_GPIO_Port, SHDN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PDO5_Pin */
  GPIO_InitStruct.Pin = PDO5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(PDO5_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Button_Pin PDO2_Pin PDO3_Pin */
  GPIO_InitStruct.Pin = Button_Pin|PDO2_Pin|PDO3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DMA_Debug_Pin SHDN_Pin */
  GPIO_InitStruct.Pin = DMA_Debug_Pin|SHDN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PDO4_Pin ROT_CHA_Pin */
  GPIO_InitStruct.Pin = PDO4_Pin|ROT_CHA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ROT_CHB_Pin */
  GPIO_InitStruct.Pin = ROT_CHB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ROT_CHB_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/*** IHM ***/


/*** USB Power Delivery Objects ***/


void Test_STUSB_I2C(void)
{
	char msg[60];
	uint8_t id;
	// Device I2C address : 0x28
	// Device ID register : 0x2F
	int ret = HAL_I2C_Mem_Read(&hi2c1, (0x28 << 1), 0x70, 1, &id, 1, 0xFFFFFFFF);
	if (ret == HAL_OK)
	{
		sprintf(msg, "STUSB I2C test success\r\n");
	}
	else
	{
		sprintf(msg, "STUSB I2C test error %d\r\n", ret);
	}
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

void Set_STUSB_Available_Profiles(uint8_t num_profiles)
{
	// TODO
	// 4.3 Programmation PDOs

	/** Documentation for write number of available profiles
	 * I2C communication:
	 * Use HAL_I2C_Mem_Read and HAL_I2C_Mem_Write functions to directly read/write into register
	 * WRN: Device address is 0x28 but should be passed shifted (0x28 << 1) to those functions
	 * Cf. Test_STUSB_I2C function
	 *
	 * Register 0x70 DPM_PDO_NUMB
	 * bit [7:5] DPM_SRC_PDO_NUMB <--- Number of source profiles available
	 * bit [4:3] reserved
	 * bis [2:0] DPM_SNK_PDO_NUMB
	 *
	 * Procedure :
	 * 1) Set device into reset state : write 0x01 to register 0x23
	 * 2) Write new value to regsiter 0x70, for ex. 0xA3 to set all 5 profiles available (1010001)
	 * Or write for example 0x23 to set only one profile available (00100011)
	 * 3) Set device back to normal mode : write 0x04 to register 0x23
	 */
}

/*** Voltage controller ***/

int32_t Compute_control_input(int32_t yref, int32_t y)
{
	// TODO
	// 3.2 Régulation

	int32_t current_error = yref-y;
	integration_sum += (current_error*dt)/1000000;

	if(integration_sum >= 2147483648){
		integration_sum = 2147483648;
	}
	if(integration_sum <= -2147483648){
		integration_sum = -2147483648;
	}

	int64_t output_64;
	output_64 =  (Kp*current_error/10000 + (Ki/10000)*integration_sum);
	int32_t output = (int32_t)output_64;

	if(output >= 100*1000000){ //100000000
		output = 100*1000000;
	}
	if(output <= 0){
			output = 0;
		}

	return output;


}

/*******************************************************************
 * INTERRUPTS
 *******************************************************************/

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{

	//TODO
	// 2.1 Conversion Analogique-Numérique
	// 2.2 Etalonage

	// calcul des valeurs de tension et de courant en fonction des valeurs numérique de l'ADC.
	// on divise par 4096 car l'ADC est sur 12 bits
	// on multiplie par 3.3 car c'est la tension pleine échelle
	// les valeurs 22 et 180 coresponde, en kilo ohms, au valeurs des résistances des ponts diviseurs de tensions (ces calcul permettent de prendre en compte la modification que le pont induit)
	vin_value =  ((((22+180)*adc_buf[1]/22)*3.3)/4096)-0.35;
	vout_value = ((((22+180)*adc_buf[0]/22)*3.3)/4096)-0.45;
	//iout_value = -((adc_buf[2]/2000)*3.3/4096)+3.3;
	iout_value = (-((adc_buf[2])*3.3/4096)+3.3)*4.4;

	//TODO
	//3.2 Régulation

	//On calcule de nouveau alpha à chaque mesure des tension de sortie et d'entrée, on le place dans une variable global.
	int vout_value_int_u = (int)vout_value;
	int vout_value_int_d = (int)((vout_value - vout_value_int_u)*100);
	pwm_pulse_BF = (Compute_control_input((Vref_BF*1000000), (vout_value_int_u*100 + vout_value_int_d)*10000));


	// TODO
	// 4.2 Protection
}

void TransferComplete(DMA_HandleTypeDef *hdma)
{
	//TODO
	// 3.3 Filtrage sortie

	for(int i=0;i<PWM_HALFBUFFER_SIZE;i++)
	{
		pwm_buffer[i+PWM_HALFBUFFER_SIZE] = pwm_pulse;
	}
}

void TransferHalfComplete(DMA_HandleTypeDef *hdma)
{
	//TODO
	// 3.3 Filtrage sortie

	for(int i=0;i<PWM_HALFBUFFER_SIZE;i++)
	{
		pwm_buffer[i] =  pwm_pulse;

	}
}

void Rotary_Encoder_Interrupt_Handler(void)
{
	static int8_t rotary_buffer = 0;
	/* Check for rotary encoder turned clockwise or counter-clockwise */
	uint8_t rotary_new = HAL_GPIO_ReadPin(ROT_CHA_GPIO_Port, ROT_CHA_Pin) << 1;
	rotary_new += HAL_GPIO_ReadPin(ROT_CHB_GPIO_Port, ROT_CHB_Pin);
	if (rotary_new != rotary_state) {
		if (((rotary_state == 0b00) && (rotary_new == 0b10)) || ((rotary_state == 0b10) && (rotary_new == 0b11)) ||
				((rotary_state == 0b11) && (rotary_new == 0b01)) || ((rotary_state == 0b01) && (rotary_new == 0b00))) {
			rotary_buffer ++;
		}
		if (((rotary_state == 0b00) && (rotary_new == 0b01)) || ((rotary_state == 0b01) && (rotary_new == 0b11)) ||
				((rotary_state == 0b11) && (rotary_new == 0b10)) || ((rotary_state == 0b10) && (rotary_new == 0b00))) {
			rotary_buffer --;
		}
		rotary_state = rotary_new;
		/* Filter rotation due to high number of positions */
		if (rotary_buffer > 3) {
			rotary_counter ++;
			rotary_direction = DIRECTION_CW;
			rotary_buffer = 0;
		}
		if (rotary_buffer < -3) {
			rotary_counter --;
			rotary_direction = DIRECTION_CCW;
			rotary_buffer = 0;
		}
	}
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
#ifdef USE_FULL_ASSERT
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
