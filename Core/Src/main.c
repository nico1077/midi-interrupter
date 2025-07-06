/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd_driver.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// アプリケーションの動作モードを定義
typedef enum {
  MODE_PULSE, // パルスモード
  MODE_MIDI,   // MIDIモード
  MODE_MIDI_LOUDNESS  // ★MIDIモード（音圧調整版）を追加
} AppMode_t;
// --- ↓MIDIヴォイスの状態を管理する構造体を定義↓ ---
typedef struct {
	uint8_t active;       // ヴォイスが使用中か (1:使用中, 0:空き)
	uint8_t note_number;  // 発音中のノート番号
	uint32_t arr;         // その音の周期(ARR)
} MidiVoice_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// --- ファンクションジェネレータ(FG)機能の定数定義 ---
// D7(TIM1)を使用するように修正
#define FG_TIM_CLK 180000000UL // TIM1のクロック周波数 (180MHz)
#define FG_FREQ_MIN 90000UL    // FGの最低周波数 (90kHz)
#define FG_FREQ_MAX 300000UL   // FGの最高周波数 (300kHz)
#define FG_FINE_TUNE_HZ 10000UL // ★微調整の全範囲(Hz)。例: 10kHz

// 周波数からタイマーのARR(周期)レジスタの値を計算するためのマクロ
// 180MHzクロックで再計算
#define FG_ARR_MAX ((FG_TIM_CLK / FG_FREQ_MIN) - 1) // 1999
#define FG_ARR_MIN ((FG_TIM_CLK / FG_FREQ_MAX) - 1) // 599

// 可変抵抗のADC値の合計の最大値 (A3:4095 + A4:4095)
#define FG_ADC_TOTAL_MAX (4095 + 4095)

// --- パルスモード用の定数 (TIM2 Prescaler=8 向け) ---
#define PULSE_FREQ_MIN 1UL
#define PULSE_FREQ_MAX 440UL
#define PULSE_TIM_CLK 90000000UL
#define PULSE_TIM_PSC 8
#define PULSE_CLOCKS_PER_COUNT (PULSE_TIM_PSC + 1)
#define PULSE_COUNTS_PER_SEC (PULSE_TIM_CLK / PULSE_CLOCKS_PER_COUNT) // 10,000,000
#define PULSE_ON_TIME_MAX_US 60UL
#define ADC_DEAD_ZONE 150

// --- MIDIモード用のタイマー定数 ---
#define MIDI_COUNTS_PER_SEC 1000000UL // ★★★ この行を追加 ★★★
#define PULSE_TIM2_PSC 8      // パルスモードで使うTIM2のプリスケーラー値
#define MIDI_TIM2_PSC  89     // MIDIモードで使うTIM2のプリスケーラー値

#define MIDI_MODE_SELECT_MIN_ADC 100  // この値より下なら「最低値」とみなす
#define MIDI_MODE_SELECT_MAX_ADC 2048 // この値より上なら「最大値」とみなす

// --- 移動平均フィルタ用の追加変数 ---
#define MOVING_AVERAGE_SAMPLES 10  // 平均を取るサンプル数。5は一般的な値です。
#define HYSTERESIS_THRESHOLD 12    // ヒステリシスのしきい値

// ADCの過去の測定値を保存する履歴用の2次元配列
uint16_t adc_history[4][MOVING_AVERAGE_SAMPLES] = {{0}}; // {{0}}で全要素を0に初期化
// 次にデータを書き込む履歴配列の場所を示すインデックス
uint8_t adc_history_index = 0;
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
AppMode_t current_mode; // 現在の動作モードを保存する変数
volatile uint16_t adc_values[4];
uint16_t stable_adc_values[4] = {0}; // 安定化したADC値を保存する配列
float fg_frequency = 0.0f; // FGの現在周波数を格納する変数
char lcd_buffer[4][21]; // ★ここに lcd_buffer を移動する
uint32_t pulse_frequency = 0;
float pulse_on_time_us = 0.0f;
float midi_duty_cycle = 50.0f; // ★MIDIモードのDuty比を保存する変数
uint8_t midi_rx_byte; // 受信したMIDIデータを1バイト格納する
uint32_t current_midi_arr = 0;
// --- ↓2和音の各ヴォイスの状態を保存する配列を追加↓ ---
#define NUM_MIDI_VOICES 8
MidiVoice_t midi_voices[NUM_MIDI_VOICES];

// --- ↓リングバッファ用の変数を追加↓ ---
#define MIDI_BUFFER_SIZE 64 // バッファサイズ(必ず2のべき乗)
uint8_t midi_buffer[MIDI_BUFFER_SIZE];
volatile uint16_t midi_buffer_head = 0;

static int current_tim2_prescaler = -1;  // Prescalerの現在値を保持

volatile uint16_t midi_buffer_tail = 0;
volatile uint8_t midi_voices_full_flag = 0; // ★ヴォイスが満杯になったかどうかのフラグ
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
void Update_Inputs(void);
void Update_Function_Generator(void);
void Update_Pulse_Mode(void); // ★この行を追加
void Update_LCD(void);
void Update_MIDI_Mode(void);
void Process_MIDI_Byte(uint8_t byte);
uint8_t Midi_NoteOn(uint8_t note, uint8_t velocity); // ★追加
void Midi_NoteOff(uint8_t note);                  // ★追加
float Midi_Note_To_Frequency(uint8_t note);
float Get_Adjusted_Duty(uint8_t note_number); // ★この行を追加
void Set_TIM2_Prescaler(uint16_t new_prescaler);
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
  HAL_Delay(2000);
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
  HAL_Delay(50);
  lcd_init();
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_values, 4);

  // --- 全てのタイマーのPWM出力を開始 ---
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);


  // ★★★ ここからモード選択処理 ★★★

  // ADCの最初の読み取りが完了するのを少し待つ
  HAL_Delay(10);
  // 移動平均フィルタの履歴バッファを現在のADC値で満たすために、
  // サンプル数と同じ回数だけUpdate_Inputs()を呼び出す
  for (int i = 0; i < MOVING_AVERAGE_SAMPLES; i++) {
      Update_Inputs();
      HAL_Delay(1); // ADCの更新を待つためのごく短い遅延
  }

  // トグルスイッチ(A5)の状態で、パルスモードかMIDI系モードかを判断
  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) == GPIO_PIN_SET) {
    // スイッチがONなら、パルスモード
    current_mode = MODE_PULSE;
  } else {
    // スイッチがOFFなら、可変抵抗(A2)の値でMIDIモードの種類を判断
    if (stable_adc_values[1] > MIDI_MODE_SELECT_MAX_ADC) {
      // A2が最大値なら、音圧調整版MIDIモード
      current_mode = MODE_MIDI_LOUDNESS;
    } else {
      // それ以外（最低値や中間）なら、通常MIDIモード
      current_mode = MODE_MIDI;
    }
  }

  // ★★★ ここまでモード選択処理 ★★★


  // MIDI(UART)の受信を開始
  HAL_UART_Receive_IT(&huart1, &midi_rx_byte, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  Update_Inputs();

	  if (current_mode == MODE_PULSE) {
		  Update_Pulse_Mode();
	  } else { // MIDIモードの場合
		  Update_MIDI_Mode(); // ★追加
	  }

	  Update_Function_Generator();
	  Update_LCD();
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
  * @brief  入力値を読み取り、「移動平均フィルタ」と「ヒステリシス処理」で安定させます。
  * 単発のノイズは平均化で除去し、微小な振動はヒステリシスで無視します。
  */
void Update_Inputs(void)
{
	// --- ステップ1: 最新のADC値を履歴配列に保存 ---
	// 現在の生のADC値を、リングバッファとして機能する履歴配列の次の場所に格納します。
	for (int i = 0; i < 4; i++) {
		// adc_values[i]は、DMAによってバックグラウンドで常に更新されている想定です。
		adc_history[i][adc_history_index] = adc_values[i];
	}

	// --- ステップ2: 移動平均を計算し、ヒステリシスを適用 ---
	// 4つのアナログ入力チャンネルそれぞれについてループ処理します。
	for (int i = 0; i < 4; i++) {
		// 履歴データ（過去5回分）の合計を計算します。
		// 合計値が16ビットを超える可能性があるため、32ビットのlong型変数を使います。
		long sum = 0;
		for (int j = 0; j < MOVING_AVERAGE_SAMPLES; j++) {
			sum += adc_history[i][j];
		}

		// 合計をサンプル数で割り、移動平均値を算出します。
		uint16_t moving_average = sum / MOVING_AVERAGE_SAMPLES;

		// --- ステップ3: ヒステリシス処理 ---
		// 計算した「移動平均値」と「現在の安定値」の差の絶対値が
		// 設定したしきい値(HYSTERESIS_THRESHOLD)より大きい場合のみ、安定値を更新します。
		if (abs(stable_adc_values[i] - moving_average) > HYSTERESIS_THRESHOLD) {
			stable_adc_values[i] = moving_average;
		}
	}

	// --- ステップ4: 次の保存場所を示すインデックスを更新 ---
	// インデックスを一つ進めます。
	adc_history_index++;

	// インデックスが配列の末尾（サンプル数）に達したら、先頭の0に戻します。
	// これにより、常に最新の5つのデータが履歴として保持されます。
	if (adc_history_index >= MOVING_AVERAGE_SAMPLES) {
		adc_history_index = 0;
	}
}

/**
  * @brief  安定化された入力値に基づき、FGの周波数を計算し、タイマーを更新します。
  */
void Update_Function_Generator(void)
{
	uint16_t adc_coarse = stable_adc_values[2];
	uint16_t adc_fine   = stable_adc_values[3];

	uint32_t freq_range = FG_FREQ_MAX - FG_FREQ_MIN;
	uint32_t freq_coarse = FG_FREQ_MIN + (adc_coarse * freq_range / 4095);

	int32_t freq_offset = (adc_fine * FG_FINE_TUNE_HZ / 4095) - (FG_FINE_TUNE_HZ / 2);

	int32_t final_freq = freq_coarse + freq_offset;

	if (final_freq < FG_FREQ_MIN) {
		final_freq = FG_FREQ_MIN;
	}
	if (final_freq > FG_FREQ_MAX) {
		final_freq = FG_FREQ_MAX;
	}

	uint32_t new_arr = (FG_TIM_CLK / final_freq) - 1;

	__HAL_TIM_SET_AUTORELOAD(&htim1, new_arr);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, new_arr / 2);

	fg_frequency = final_freq;
}

/**
  * @brief  現在の状態に基づいてLCDの表示を更新します。(Fフラグ表示修正版)
  */
void Update_LCD(void)
{
	char lcd_buffer[4][21];

	// モードごとの表示を作成
	if (current_mode == MODE_PULSE) {
		sprintf(lcd_buffer[0], "Mode: PULSE         ");
		sprintf(lcd_buffer[1], "Freq: %-4luHz", pulse_frequency);
		sprintf(lcd_buffer[2], "ON-Time: %-7.1fus", pulse_on_time_us);
	} else { // MIDIモードの場合
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

		// ★★★ 表示を「Notes: X F」の形式に戻します ★★★
		// midi_voices_full_flagが1なら"F"を、0なら空白" "を表示
		sprintf(lcd_buffer[2], "Notes: %d %s", active_voices, midi_voices_full_flag ? "F" : " ");
	}

	// 4行目: FG周波数は常に表示
	sprintf(lcd_buffer[3], "FG: %5.1f kHz", fg_frequency / 1000.0f);

	// LCDに書き込み
	for (int i=0; i<4; i++) {
		lcd_set_cursor(0, i);
		lcd_send_string(lcd_buffer[i]);
	}
}

/**
  * @brief  パルスモードの周波数とONタイムを計算し、TIM2を更新します。(最終修正版)
  */
void Update_Pulse_Mode(void)
{
    static uint32_t last_pulse_arr = 0;

    // TIM2のPrescalerをパルスモード用に設定（10MHz = 0.1μs/カウント）
    Set_TIM2_Prescaler(8);

    // 周波数ADC入力（A4）
    uint16_t adc_freq = stable_adc_values[1];
    if (adc_freq < ADC_DEAD_ZONE)
    {
        adc_freq = 0;
    }

    // 周波数を1〜440Hzで決定
    uint32_t freq_range = PULSE_FREQ_MAX - PULSE_FREQ_MIN;
    pulse_frequency = PULSE_FREQ_MIN + (adc_freq * freq_range / 4095);

    if (pulse_frequency == 0) {
        pulse_frequency = 1;
    }

    // 周波数からARRを計算（10MHzタイマクロック → 分解能0.1μs）
    uint32_t new_arr = (PULSE_COUNTS_PER_SEC / pulse_frequency) - 1;

    // ON時間ADC入力（A3）
    uint16_t adc_on_time = stable_adc_values[0];
    pulse_on_time_us = (float)adc_on_time * (float)PULSE_ON_TIME_MAX_US / 4095.0f;

    // CCR値を0.1μs単位で設定（CCR = ON時間 × 10）
    uint32_t new_ccr = (uint32_t)(pulse_on_time_us * 10.0f);

    if (new_ccr > new_arr) {
        new_ccr = new_arr;
    }

    // PWM設定反映
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, new_ccr);
    if (new_arr != last_pulse_arr)
    {
        __HAL_TIM_SET_AUTORELOAD(&htim2, new_arr);
        HAL_TIM_GenerateEvent(&htim2, TIM_EVENTSOURCE_UPDATE);
        last_pulse_arr = new_arr;
    }

    // 実際に使われたCCR値からON時間(us)を再計算して表示用に反映
    pulse_on_time_us = (float)new_ccr / 10.0f;
}

/**
  * @brief MIDI(UART)で1バイト受信するたびに呼ばれるコールバック関数 (最終修正版)
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	// 受信したバイトをバッファに入れるだけ
	Midi_Buffer_Write(midi_rx_byte);
	// 次の1バイトを再び割り込みモードで待つ
	HAL_UART_Receive_IT(&huart1, &midi_rx_byte, 1);
}

/**
  * @brief 受信したMIDIバイトを解析し、NoteOn/NoteOffを呼び出す (8和音対応版)
  */
void Process_MIDI_Byte(uint8_t byte)
{
	static uint8_t status = 0;
	static uint8_t byte1 = 0;

	if (byte >= 0x80) { // ステータスバイト
		status = byte;
		byte1 = 0;
	} else if (status != 0) { // データバイト
		if (byte1 == 0) {
			byte1 = byte;
		} else {
			uint8_t command = status & 0xF0;
			uint8_t note = byte1;
			uint8_t velocity = byte;

			if (command == 0x90 && velocity > 0) {
				if (Midi_NoteOn(note, velocity) == 0) {
					// もし音を鳴らせなかったら(戻り値が0なら)
					midi_voices_full_flag = 1;
				}
			} else { // Note Off または Velocity 0 の Note On
				Midi_NoteOff(note);
			}
			status = 0;
		}
	}
}

/**
  * @brief 空いているヴォイスを探して音を鳴らす (音圧調整対応・8和音版)
  * @retval 音を鳴らせたら1、空きヴォイスがなくて鳴らせなかったら0を返す
  */
uint8_t Midi_NoteOn(uint8_t note, uint8_t velocity)
{
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

	float freq = Midi_Note_To_Frequency(note);
	if (freq <= 0) {
		return 0;
	}

	midi_voices[voice_index].active = 1;
	midi_voices[voice_index].note_number = note;

	// 全タイマー1MHz動作なので、計算は共通
	uint32_t arr = (MIDI_COUNTS_PER_SEC / freq) - 1;
	uint32_t ccr = 0;
	float final_duty = 0.0f;

	// タイマーが16bitか32bitかでARRの上限をチェック
	switch(voice_index) {
		case 0: // TIM8 (16bit)
		case 2: // TIM3 (16bit)
		case 3: // TIM4 (16bit)
		case 5: // TIM14 (16bit)
		case 6: // TIM9 (16bit)
		case 7: // TIM12 (16bit)
			if (arr > 65535) arr = 65535;
			break;
		// ヴォイス1(TIM2), 4(TIM5)は32bitなのでチェック不要
	}

	midi_voices[voice_index].arr = arr;

	// モードに応じて適用するDuty比を決定
	if (current_mode == MODE_MIDI_LOUDNESS) {
		// 音圧調整モードなら、補正計算したDuty比を使う
		final_duty = Get_Adjusted_Duty(note);
	} else {
		// 通常モードなら、そのままのDuty比を使う
		final_duty = midi_duty_cycle;
	}
	ccr = arr * (final_duty / 100.0f);

	// ヴォイス番号に応じてタイマーのレジスタを更新
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
			midi_voices[i].active = 0;
			midi_voices[i].note_number = 0;
			midi_voices[i].arr = 0;


			// ヴォイス番号に応じてタイマーの音を止める
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
			break;
		}
	}
}

/**
  * @brief MIDIモード時の処理 (音圧調整対応・8和音リアルタイムDuty更新)
  */
void Update_MIDI_Mode(void)
{
	uint8_t byte;
	// リングバッファからMIDIデータを読み取り、処理する
	while (Midi_Buffer_Read(&byte)) {
		Process_MIDI_Byte(byte);
	}

	// 可変抵抗(A1)から基本となるDuty比を計算
	uint16_t adc_duty = stable_adc_values[0];
	midi_duty_cycle = (float)adc_duty * 25.0f / 4095.0f;

	// 発音中の全ヴォイスのDuty比をリアルタイムで更新
	for (int i=0; i < NUM_MIDI_VOICES; i++) {
		if (midi_voices[i].active) {
			float final_duty;
			// モードに応じて適用するDuty比を決定
			if (current_mode == MODE_MIDI_LOUDNESS) {
				// 音圧調整モードなら、補正計算したDuty比を使う
				final_duty = Get_Adjusted_Duty(midi_voices[i].note_number);
			} else {
				// 通常モードなら、そのままのDuty比を使う
				final_duty = midi_duty_cycle;
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
  * @brief MIDIノート番号を周波数(Hz)に変換する
  */
float Midi_Note_To_Frequency(uint8_t note)
{
	// f = 440 * 2^((n-69)/12)
	return 440.0f * powf(2.0f, (float)(note - 69) / 12.0f);
}

void Set_TIM2_Prescaler(uint16_t new_prescaler)
{
    if (current_tim2_prescaler == new_prescaler) return;

    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
    __HAL_TIM_SET_PRESCALER(&htim2, new_prescaler);
    __HAL_TIM_SET_COUNTER(&htim2, 0); // カウンタ初期化
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

    current_tim2_prescaler = new_prescaler;
}

/**
  * @brief  ノート番号に応じて音圧調整されたDuty比を計算して返す
  * @param  note_number : MIDIノート番号
  * @retval 調整後のDuty比(%)
  */
float Get_Adjusted_Duty(uint8_t note_number)
{
	// 1. ノート番号に応じた補正係数 f(x) を計算
	//float correction_factor = 1.0f - 0.015f * (float)(note_number - 69);
	const float x = (float)note_number;
	const float a = 150.0f;
	float numerator = (x - a) * (x - a);
	const float denominator = (a - 69.0f) * (a - 69.0f); // (171*171 = 29241)
	float correction_factor = numerator / denominator;

	// 2. 基本となるDuty比に補正係数を掛ける
	float adjusted_duty = midi_duty_cycle * correction_factor;

	// 3. 計算結果が上限(25%)や下限(0%)を超えないように制限する
	if (adjusted_duty > 25.0f) {
		adjusted_duty = 25.0f;
	}
	if (adjusted_duty < 0.0f) {
		adjusted_duty = 0.0f;
	}

	return adjusted_duty;
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
