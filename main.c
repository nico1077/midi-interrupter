/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : メインプログラム
  * @author         : dango
  * @date           : 2025/07/07
  *
  * @description
  * このプログラムは、STM32マイコン上で動作する多機能シンセサイザです。
  * 以下の3つのモードを搭載しています。
  * 1. パルスモード (MODE_PULSE):
  * - 可変抵抗で周波数とパルス幅（ON時間）を制御できるパルス信号を生成します。
  * 2. MIDIモード (MODE_MIDI):
  * - MIDIキーボードからの入力に応じて、最大8和音の矩形波を生成します。
  * - Duty比は可変抵抗で一括して調整可能です。
  * 3. MIDIラウドネスモード (MODE_MIDI_LOUDNESS):
  * - MIDIモードをベースに、音高に応じてDuty比を自動調整し、
  * 聴感上の音量を均一化します。
  *
  * また、全モード共通で、可変抵抗で周波数を制御できる
  * ファンクションジェネレータ機能も搭載しています。
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd_driver.h" // I2C接続LCDを制御するための自作ドライバ
#include <stdio.h>      // sprintf関数で文字列をフォーマットするため
#include <stdlib.h>     // abs関数で絶対値を取得するため
#include <math.h>       // powf関数でべき乗計算（周波数計算）をするため
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/**
 * @brief アプリケーションの動作モードを定義する列挙型
 */
typedef enum {
  MODE_PULSE,         // パルスモード
  MODE_MIDI,          // 通常のMIDIモード
  MODE_MIDI_LOUDNESS  // 音量補正機能付きMIDIモード
} AppMode_t;

/**
 * @brief MIDIの各発音状態（ヴォイス）を管理するための構造体
 */
typedef struct {
	uint8_t active;       // ヴォイスが現在発音中か (1:使用中, 0:空き)
	uint8_t note_number;  // 発音中のMIDIノート番号 (例: C4は60)
	uint32_t arr;         // この音の周期を決めるタイマーのARR(Auto-Reload Register)値
} MidiVoice_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// --- ファンクションジェネレータ(FG)機能の定数 ---
#define FG_TIM_CLK 180000000UL // FG用タイマー(TIM1)のクロック周波数 (180MHz)
#define FG_FREQ_MIN 90000UL    // FGの最低周波数 (90kHz)
#define FG_FREQ_MAX 300000UL   // FGの最高周波数 (300kHz)
#define FG_FINE_TUNE_HZ 10000UL // 微調整用VRの可変範囲 (10kHz)

// --- パルスモード用の定数 ---
#define PULSE_FREQ_MIN 1UL      // パルスモードの最低周波数 (1Hz)
#define PULSE_FREQ_MAX 440UL    // パルスモードの最高周波数 (440Hz, ラ(A4)の音)
#define PULSE_TIM_CLK 90000000UL// パルスモード用タイマー(TIM2)の基本クロック (90MHz)
#define PULSE_TIM_PSC 8         // パルスモード時のTIM2プリスケーラ値
// プリスケーラで分周された後の、TIM2の動作クロック周波数
// 計算式: 90MHz / (8 + 1) = 10MHz。つまり1カウントが0.1us。
#define PULSE_COUNTS_PER_SEC (PULSE_TIM_CLK / (PULSE_TIM_PSC + 1))
#define PULSE_ON_TIME_MAX_US 60UL // ON時間調整VRで設定できる最大時間 (60マイクロ秒)
#define ADC_DEAD_ZONE 150       // VRの最小付近の不安定な値を無視するための閾値

// --- MIDIモード用のタイマー定数 ---
#define MIDI_TIM2_PSC  89      // MIDIモード時のTIM2プリスケーラ値
// プリスケーラで分周された後の、MIDIモード時の各タイマーの動作クロック周波数
// 計算式: 90MHz / (89 + 1) = 1MHz。周期計算がしやすくなる。
#define MIDI_COUNTS_PER_SEC 1000000UL

// --- モード選択用のADC値閾値 ---
// 起動時にA2ポートのVRの値でMIDIモードを切り替えるための閾値
#define MIDI_MODE_SELECT_MAX_ADC 2048 // このADC値より大きい場合、ラウドネスモードになる

// --- 入力安定化フィルタ用の定数 ---
#define MOVING_AVERAGE_SAMPLES 10  // 移動平均フィルタで平均を取るサンプル数
#define HYSTERESIS_THRESHOLD 12    // ヒステリシス処理の閾値。この値以下の微小な変動は無視する

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim12;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
// --- グローバル変数 ---
AppMode_t current_mode; // 現在の動作モードを保持する

volatile uint16_t adc_values[4];      // DMAから直接書き込まれる生のADC値
uint16_t stable_adc_values[4] = {0}; // フィルタ処理後の安定したADC値

// 移動平均フィルタ用の履歴バッファ
uint16_t adc_history[4][MOVING_AVERAGE_SAMPLES] = {{0}};
uint8_t adc_history_index = 0;

// LCD表示用の変数
char lcd_buffer[4][21];
float fg_frequency = 0.0f;
uint32_t pulse_frequency = 0;
float pulse_on_time_us = 0.0f;
float midi_duty_cycle = 50.0f;

// MIDI処理用の変数
uint8_t midi_rx_byte; // UARTから受信した1バイトを一時的に格納
#define NUM_MIDI_VOICES 8
MidiVoice_t midi_voices[NUM_MIDI_VOICES]; // 8和音の各音の状態を管理
volatile uint8_t midi_voices_full_flag = 0; // 全てのヴォイスが使用中かを示すフラグ

// MIDI受信用リングバッファ
#define MIDI_BUFFER_SIZE 64
uint8_t midi_buffer[MIDI_BUFFER_SIZE];
volatile uint16_t midi_buffer_head = 0; // 読み出し位置
volatile uint16_t midi_buffer_tail = 0; // 書き込み位置

// TIM2のプリスケーラ値を保持（モード切替時の再設定を効率化するため）
static int current_tim2_prescaler = -1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM12_Init(void);
static void MX_TIM14_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
// --- 自作関数のプロトタイプ宣言 ---
void Update_Inputs(void);
void Update_Function_Generator(void);
void Update_Pulse_Mode(void);
void Update_MIDI_Mode(void);
void Update_LCD(void);
void Process_MIDI_Byte(uint8_t byte);
uint8_t Midi_NoteOn(uint8_t note, uint8_t velocity);
void Midi_NoteOff(uint8_t note);
float Midi_Note_To_Frequency(uint8_t note);
float Get_Adjusted_Duty(uint8_t note_number);
void Set_TIM2_Prescaler(uint16_t new_prescaler);
void Midi_Buffer_Write(uint8_t byte);
uint8_t Midi_Buffer_Read(uint8_t *byte);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  アプリケーションのエントリーポイント（ここからプログラムが始まる）
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
  HAL_Delay(2000); // 電源安定のためのウェイト
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_TIM9_Init();
  MX_TIM12_Init();
  MX_TIM14_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  // --- 初期化処理 ---
  HAL_Delay(50);
  lcd_init(); // LCDを初期化
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_values, 4); // ADCのDMA転送を開始

  // 全てのタイマーのPWM出力を開始
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);


  // --- 起動時のモード選択 ---
  // 起動時のスイッチと可変抵抗の状態で動作モードを決定する
  HAL_Delay(10);
  // フィルタを安定させるため、複数回入力読み取りを行う
  for (int i = 0; i < MOVING_AVERAGE_SAMPLES; i++) {
      Update_Inputs();
      HAL_Delay(1);
  }

  // トグルスイッチ(PC0)の状態で、パルスモードかMIDI系モードかを判断
  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) == GPIO_PIN_SET) {
    current_mode = MODE_PULSE; // スイッチON -> パルスモード
  } else {
    // スイッチOFF -> 可変抵抗(A2)の値でMIDIモードの種類を判断
    if (stable_adc_values[1] > MIDI_MODE_SELECT_MAX_ADC) {
      current_mode = MODE_MIDI_LOUDNESS; // VR最大 -> 音圧調整MIDIモード
    } else {
      current_mode = MODE_MIDI; // それ以外 -> 通常MIDIモード
    }
  }

  // MIDI(UART)の受信割り込みを開始
  HAL_UART_Receive_IT(&huart1, &midi_rx_byte, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // --- メインループ ---
  // この中を永久に繰り返し実行する
  while (1)
  {
	  Update_Inputs(); // 1. 入力値（VRなど）を読み取り安定化

      // 2. 現在のモードに応じた処理を実行
	  if (current_mode == MODE_PULSE) {
		  Update_Pulse_Mode();
	  } else {
		  Update_MIDI_Mode();
	  }

	  Update_Function_Generator(); // 3. FGの周波数を更新
	  Update_LCD();                // 4. LCDの表示を更新
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 2;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 3;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 4;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 89;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 89;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 89;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 89;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 179;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 179;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 65535;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 89;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 65535;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */
  HAL_TIM_MspPostInit(&htim12);

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 179;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 65535;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */
  HAL_TIM_MspPostInit(&htim14);

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
  huart1.Init.BaudRate = 31250;
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

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//==============================================================================
// ユーザー定義関数
//==============================================================================

/**
  * @brief  入力値（可変抵抗）を読み取り、「移動平均フィルタ」と「ヒステリシス処理」で安定させます。
  * @note   単発のノイズは平均化で除去し、微小な振動はヒステリシスで無視することで、値のチラつきを防ぎます。
  */
void Update_Inputs(void)
{
	// ステップ1: 最新のADC値を履歴配列に保存
	for (int i = 0; i < 4; i++) {
		adc_history[i][adc_history_index] = adc_values[i];
	}

	// ステップ2: 移動平均を計算し、ヒステリシスを適用
	for (int i = 0; i < 4; i++) {
		long sum = 0;
		for (int j = 0; j < MOVING_AVERAGE_SAMPLES; j++) {
			sum += adc_history[i][j];
		}
		uint16_t moving_average = sum / MOVING_AVERAGE_SAMPLES;

		// ステップ3: ヒステリシス処理
        // 平均値と現在の安定値の差が閾値より大きい場合のみ、安定値を更新
		if (abs(stable_adc_values[i] - moving_average) > HYSTERESIS_THRESHOLD) {
			stable_adc_values[i] = moving_average;
		}
	}

	// ステップ4: 次の保存場所を示すインデックスを更新（リングバッファ）
	adc_history_index++;
	if (adc_history_index >= MOVING_AVERAGE_SAMPLES) {
		adc_history_index = 0;
	}
}

/**
  * @brief  安定化された入力値に基づき、FGの周波数を計算し、タイマー(TIM1)を更新します。
  */
void Update_Function_Generator(void)
{
	// 粗調整VR(A3)と微調整VR(A4)の安定値を取得
	uint16_t adc_coarse = stable_adc_values[2];
	uint16_t adc_fine   = stable_adc_values[3];

	// 粗調整VRの値から、大まかな周波数を決定
	uint32_t freq_range = FG_FREQ_MAX - FG_FREQ_MIN;
	uint32_t freq_coarse = FG_FREQ_MIN + (adc_coarse * freq_range / 4095);

	// 微調整VRの値から、中心を0としたオフセット値を計算
	int32_t freq_offset = (adc_fine * FG_FINE_TUNE_HZ / 4095) - (FG_FINE_TUNE_HZ / 2);

	// 粗調整と微調整の結果を合算
	int32_t final_freq = freq_coarse + freq_offset;

	// 計算結果が設定範囲外にならないようにクリップ
	if (final_freq < FG_FREQ_MIN) final_freq = FG_FREQ_MIN;
	if (final_freq > FG_FREQ_MAX) final_freq = FG_FREQ_MAX;

	// 最終的な周波数から、タイマーのARR(周期)とCCR(Duty50%)を計算して設定
	uint32_t new_arr = (FG_TIM_CLK / final_freq) - 1;
	__HAL_TIM_SET_AUTORELOAD(&htim1, new_arr);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, new_arr / 2);

	fg_frequency = final_freq; // 表示用に周波数を保存
}

/**
  * @brief  パルスモードの周波数とONタイムを計算し、タイマー(TIM2)を更新します。
  */
void Update_Pulse_Mode(void)
{
    static uint32_t last_pulse_arr = 0; // 前回のARR値を記憶（不要な更新を防ぐため）

    // TIM2のPrescalerをパルスモード用に設定（カウントクロック10MHz = 0.1μs/カウント）
    Set_TIM2_Prescaler(PULSE_TIM_PSC);

    // 周波数設定VR(A2)の値を取得（デッドゾーン処理あり）
    uint16_t adc_freq = stable_adc_values[1];
    if (adc_freq < ADC_DEAD_ZONE) adc_freq = 0;

    // 周波数を1〜440Hzの範囲で線形に計算
    uint32_t freq_range = PULSE_FREQ_MAX - PULSE_FREQ_MIN;
    pulse_frequency = PULSE_FREQ_MIN + (adc_freq * freq_range / 4095);
    if (pulse_frequency == 0) pulse_frequency = 1;

    // 周波数からタイマーのARR(周期)を計算
    uint32_t new_arr = (PULSE_COUNTS_PER_SEC / pulse_frequency) - 1;

    // ON時間設定VR(A1)の値を取得
    uint16_t adc_on_time = stable_adc_values[0];
    pulse_on_time_us = (float)adc_on_time * (float)PULSE_ON_TIME_MAX_US / 4095.0f;

    // ON時間(us)からタイマーのCCR(パルス幅)を計算（1カウント0.1usなので10倍する）
    uint32_t new_ccr = (uint32_t)(pulse_on_time_us * 10.0f);
    if (new_ccr > new_arr) new_ccr = new_arr; // ON時間が周期を超えないようにする

    // 計算結果をタイマーレジスタに設定
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, new_ccr);
    if (new_arr != last_pulse_arr) // ARRが変更された場合のみ更新
    {
        __HAL_TIM_SET_AUTORELOAD(&htim2, new_arr);
        HAL_TIM_GenerateEvent(&htim2, TIM_EVENTSOURCE_UPDATE); // 設定を即時反映
        last_pulse_arr = new_arr;
    }

    // 実際に設定されたCCR値からON時間(us)を再計算して表示用に反映
    pulse_on_time_us = (float)new_ccr / 10.0f;
}

/**
  * @brief MIDIモード時の処理。Duty比の更新とMIDIメッセージの処理を行います。
  */
void Update_MIDI_Mode(void)
{
	uint8_t byte;
	// リングバッファからMIDIデータを1バイトずつ読み出し、処理が追いつくまでループ
	while (Midi_Buffer_Read(&byte)) {
		Process_MIDI_Byte(byte);
	}

	// Duty比設定VR(A1)から、基本となるDuty比(0-25%)を計算
	uint16_t adc_duty = stable_adc_values[0];
	midi_duty_cycle = (float)adc_duty * 25.0f / 4095.0f;

	// 発音中の全ヴォイスのDuty比をリアルタイムで更新
	for (int i=0; i < NUM_MIDI_VOICES; i++) {
		if (midi_voices[i].active) {
			float final_duty;
			// モードに応じて適用するDuty比を決定
			if (current_mode == MODE_MIDI_LOUDNESS) {
				final_duty = Get_Adjusted_Duty(midi_voices[i].note_number); // 音圧補正モード
			} else {
				final_duty = midi_duty_cycle; // 通常モード
			}
			uint32_t new_ccr = midi_voices[i].arr * (final_duty / 100.0f);

			// ヴォイス番号に応じて、該当タイマーのCCRを更新
			switch(i) {
				case 0: __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, new_ccr); break;
				case 1: __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, new_ccr); break;
				case 2: __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, new_ccr); break;
				case 3: __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, new_ccr); break;
				case 4: __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, new_ccr); break;
				case 5: __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, new_ccr); break;
				case 6: __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, new_ccr); break;
				case 7: __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, new_ccr); break;
			}
		}
	}
}

/**
  * @brief  現在の状態に基づいてLCDの表示を更新します。
  */
void Update_LCD(void)
{
	// モードごとの表示を作成
	if (current_mode == MODE_PULSE) {
		sprintf(lcd_buffer[0], "Mode: PULSE         ");
		sprintf(lcd_buffer[1], "Freq: %-4luHz", pulse_frequency);
		sprintf(lcd_buffer[2], "ON-Time: %-7.1fus", pulse_on_time_us);
	} else { // MIDI系モード
		// 現在の発音数をカウント
		uint8_t active_voices = 0;
		for(int i=0; i<NUM_MIDI_VOICES; i++) {
			if (midi_voices[i].active) active_voices++;
		}

		if (current_mode == MODE_MIDI_LOUDNESS) {
			sprintf(lcd_buffer[0], "Mode: MIDI LOUDNESS ");
		} else {
			sprintf(lcd_buffer[0], "Mode: MIDI NORMAL   ");
		}
		sprintf(lcd_buffer[1], "Duty: %4.1f%%", midi_duty_cycle);
        // 発音数と、全ヴォイス使用中フラグ("F")を表示
		sprintf(lcd_buffer[2], "Notes: %d %s", active_voices, midi_voices_full_flag ? "F" : " ");
	}

	// 4行目: FG周波数は常に表示
	sprintf(lcd_buffer[3], "FG: %5.1f kHz", fg_frequency / 1000.0f);

	// 作成した文字列をLCDに書き込み
	for (int i=0; i<4; i++) {
		lcd_set_cursor(0, i);
		lcd_send_string(lcd_buffer[i]);
	}
}

/**
  * @brief MIDIリングバッファに1バイト書き込む
  */
void Midi_Buffer_Write(uint8_t byte)
{
	uint16_t next_tail = (midi_buffer_tail + 1) & (MIDI_BUFFER_SIZE - 1);
	if (next_tail != midi_buffer_head) { // バッファが満杯でなければ
		midi_buffer[midi_buffer_tail] = byte;
		midi_buffer_tail = next_tail;
	}
}

/**
  * @brief MIDIリングバッファから1バイト読み込む
  * @retval 読み込めた場合は1、バッファが空の場合は0
  */
uint8_t Midi_Buffer_Read(uint8_t *byte)
{
	if (midi_buffer_head == midi_buffer_tail) {
		return 0; // バッファは空
	}
	*byte = midi_buffer[midi_buffer_head];
	midi_buffer_head = (midi_buffer_head + 1) & (MIDI_BUFFER_SIZE - 1);
	return 1;
}

/**
  * @brief 受信したMIDIバイトを1バイトずつ解析し、NoteOn/NoteOffを呼び出す
  */
void Process_MIDI_Byte(uint8_t byte)
{
	static uint8_t status = 0; // ステータスバイトを記憶
	static uint8_t byte1 = 0;  // 1つ目のデータバイトを記憶

	if (byte >= 0x80) { // ステータスバイト(MSB=1)の場合
		status = byte;
		byte1 = 0;
	} else if (status != 0) { // データバイトの場合
		if (byte1 == 0) {
			byte1 = byte; // 1つ目のデータバイトとして記憶
		} else {
            // 2つ目のデータバイトを受信し、3バイトのメッセージが揃った
			uint8_t command = status & 0xF0;
			uint8_t note = byte1;
			uint8_t velocity = byte;

			if (command == 0x90 && velocity > 0) { // Note On
				if (Midi_NoteOn(note, velocity) == 0) {
					midi_voices_full_flag = 1; // 音を鳴らせなかったらフラグを立てる
				}
			} else { // Note Off (NoteOffメッセージ or Velocity 0のNoteOn)
				Midi_NoteOff(note);
			}
			status = 0; // 1メッセージ処理完了
		}
	}
}

/**
  * @brief 空いているヴォイスを探して音を鳴らす (8和音対応)
  * @retval 音を鳴らせたら1、空きヴォイスがなくて鳴らせなかったら0を返す
  */
uint8_t Midi_NoteOn(uint8_t note, uint8_t velocity)
{
	// 1. 空いているヴォイスを探す
	int8_t voice_index = -1;
	for (int i = 0; i < NUM_MIDI_VOICES; i++) {
		if (!midi_voices[i].active) {
			voice_index = i;
			break;
		}
	}
	if (voice_index == -1) {
		return 0; // 空きヴォイスがなければ0を返して終了
	}

    // 2. 周波数、ARR、CCRを計算
	float freq = Midi_Note_To_Frequency(note);
	if (freq <= 0) return 0;

	uint32_t arr = (MIDI_COUNTS_PER_SEC / freq) - 1;
	uint32_t ccr = 0;
	float final_duty = 0.0f;

	// 16bitタイマーの場合、ARRの上限をチェック
	switch(voice_index) {
		case 0: case 2: case 3: case 5: case 6: case 7: // TIM8,3,4,14,9,12
			if (arr > 65535) arr = 65535;
			break;
		// 32bitタイマー(TIM2,5)はチェック不要
	}

	// モードに応じてDuty比を決定
	if (current_mode == MODE_MIDI_LOUDNESS) {
		final_duty = Get_Adjusted_Duty(note);
	} else {
		final_duty = midi_duty_cycle;
	}
	ccr = arr * (final_duty / 100.0f);

    // 3. ヴォイス情報を更新し、タイマーを設定
    midi_voices[voice_index].active = 1;
	midi_voices[voice_index].note_number = note;
	midi_voices[voice_index].arr = arr;

	switch(voice_index) {
		case 0:
			__HAL_TIM_SET_AUTORELOAD(&htim8, arr);
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, ccr);
			HAL_TIM_GenerateEvent(&htim8, TIM_EVENTSOURCE_UPDATE);
			break;
		case 1:
			__HAL_TIM_SET_AUTORELOAD(&htim2, arr);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, ccr);
			HAL_TIM_GenerateEvent(&htim2, TIM_EVENTSOURCE_UPDATE);
			break;
		case 2:
			__HAL_TIM_SET_AUTORELOAD(&htim3, arr);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ccr);
			HAL_TIM_GenerateEvent(&htim3, TIM_EVENTSOURCE_UPDATE);
			break;
		case 3:
			__HAL_TIM_SET_AUTORELOAD(&htim4, arr);
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, ccr);
			HAL_TIM_GenerateEvent(&htim4, TIM_EVENTSOURCE_UPDATE);
			break;
		case 4:
			__HAL_TIM_SET_AUTORELOAD(&htim5, arr);
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, ccr);
			HAL_TIM_GenerateEvent(&htim5, TIM_EVENTSOURCE_UPDATE);
			break;
		case 5:
			__HAL_TIM_SET_AUTORELOAD(&htim14, arr);
			__HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, ccr);
			HAL_TIM_GenerateEvent(&htim14, TIM_EVENTSOURCE_UPDATE);
			break;
		case 6:
			__HAL_TIM_SET_AUTORELOAD(&htim9, arr);
			__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, ccr);
			HAL_TIM_GenerateEvent(&htim9, TIM_EVENTSOURCE_UPDATE);
			break;
		case 7:
			__HAL_TIM_SET_AUTORELOAD(&htim12, arr);
			__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, ccr);
			HAL_TIM_GenerateEvent(&htim12, TIM_EVENTSOURCE_UPDATE);
			break;
	}
	return 1; // 成功
}

/**
  * @brief 指定されたノート番号の音を探して止める (8和音版)
  */
void Midi_NoteOff(uint8_t note)
{
	for (int i = 0; i < NUM_MIDI_VOICES; i++) {
		if (midi_voices[i].active && midi_voices[i].note_number == note) {
			// ヴォイスを非アクティブにする
			midi_voices[i].active = 0;
			midi_voices[i].note_number = 0;
			midi_voices[i].arr = 0;

			// 音を止める（Duty比を0にする）
			switch(i) {
				case 0: __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0); break;
				case 1: __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0); break;
				case 2: __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0); break;
				case 3: __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0); break;
				case 4: __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0); break;
				case 5: __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, 0); break;
				case 6: __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0); break;
				case 7: __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0); break;
			}
			break; // 同じノート番号の音は1つしか無いはずなのでループを抜ける
		}
	}
}

/**
  * @brief MIDIノート番号を周波数(Hz)に変換する
  * @note  計算式: f = 440 * 2^((n-69)/12)
  */
float Midi_Note_To_Frequency(uint8_t note)
{
	return 440.0f * powf(2.0f, (float)(note - 69) / 12.0f);
}

/**
  * @brief  ノート番号に応じて音圧（聴感上の音量）が均一に近づくようにDuty比を補正計算します。
  * @param  note_number : 補正対象のMIDIノート番号
  * @retval 補正後のDuty比 (%)
  * @note   この関数は、高い音ほどDuty比を下げ、低い音ほど上げる二次関数を用いています。
  * これにより、人間の耳が敏感な高音域の音量を抑え、全体の音量バランスを整えます。
  * 係数 a は、どのノート番号で補正が最小（基本Duty比に最も近くなるか）を決定します。
  */
float Get_Adjusted_Duty(uint8_t note_number)
{
	// 補正の中心となるノート番号。この音から離れるほど補正が大きくなる。
	const float a = 150.0f; // かなり高い音に設定（実質、低音域を持ち上げる効果）
	const float x = (float)note_number;

	// 1. ノート番号に応じた補正係数を二次関数で計算
	// f(x) = (x - a)^2 / (a - 69)^2
	// ノート69(A4)の時に補正係数が1に近くなるように正規化している。
	float numerator = (x - a) * (x - a);
	const float denominator = (a - 69.0f) * (a - 69.0f);
	float correction_factor = numerator / denominator;

	// 2. VRで設定された基本Duty比に、計算した補正係数を掛ける
	float adjusted_duty = midi_duty_cycle * correction_factor;

	// 3. 計算結果が意図しない範囲にならないように上限・下限を設定
	if (adjusted_duty > 25.0f) adjusted_duty = 25.0f;
	if (adjusted_duty < 0.0f) adjusted_duty = 0.0f;

	return adjusted_duty;
}

/**
  * @brief TIM2のプリスケーラ値を安全に変更する
  * @note  モード切替時に、異なるクロック周波数設定を反映させるために使用
  */
void Set_TIM2_Prescaler(uint16_t new_prescaler)
{
    // 現在の設定値と同じなら何もしない
    if (current_tim2_prescaler == new_prescaler) return;

    // 安全に変更するため、一度PWMを停止してから設定を変更し、再開する
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
    __HAL_TIM_SET_PRESCALER(&htim2, new_prescaler);
    __HAL_TIM_SET_COUNTER(&htim2, 0); // カウンタをリセット
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

    current_tim2_prescaler = new_prescaler; // 現在値を更新
}

/**
  * @brief MIDI(UART)で1バイト受信するたびに呼ばれるコールバック関数
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	// 受信したバイトをリングバッファに入れるだけ。重い処理はメインループで行う。
	Midi_Buffer_Write(midi_rx_byte);
	// 次の1バイトを再び割り込みモードで待つ
	HAL_UART_Receive_IT(&huart1, &midi_rx_byte, 1);
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
  * where the assert_param error has occurred.
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
