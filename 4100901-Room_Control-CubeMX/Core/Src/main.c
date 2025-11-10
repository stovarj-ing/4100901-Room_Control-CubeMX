/* USER CODE BEGIN Header */
/**
  **********
  * @file           : main.c
  * @brief          : Main program body
  **********
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  **********
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */         
#include "led_driver.h"     // Driver para control de LEDs
#include "ring_buffer.h"    // Implementación del buffer circular
#include "keypad_driver.h"  // Driver para el teclado matricial
#include <stdio.h>          // Para funciones printf y comunicación serial
#include <string.h>         // Para funciones de manejo de strings como strncmp
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// --- CONFIGURACION DEL SISTEMA ---
#define PASSWORD "0102*" // Contraseña de 4 dígitos
#define PASSWORD_LEN 5  // Longitud de la contraseña
#define DEBOUNCE_TIME_MS 200      // Tiempo de debounce para las teclas, evita leer dos veces la misma tecla
#define FEEDBACK_LED_TIME_MS 100  // Tiempo que el LED se enciende al oprimir cualquier tecla
#define SUCCESS_LED_TIME_MS 4000  // Tiempo que el LED se enciende cuando se ingresa la contraseña correcta
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/**
 * @brief LED principal (LD2 en la Nucleo) ---
 * @note  Esta estructura se utiliza para controlar el LED integrado en la placa.
 * @param led1 Estructura que define el puerto y pin del LED.
 */
led_handle_t led1 = { .port = GPIOA, .pin = GPIO_PIN_5 }; // LD2
led_handle_t led_ext = { .port = GPIOA, .pin = GPIO_PIN_7 }; // LED externo en PA7

// Definir la conexión física del teclado matricial 4x4 - Le permite al driver saber dónde están conectadas las teclas
keypad_handle_t keypad = { 
    // Puertos y pines para las filas del teclado
    .row_ports = {KEYPAD_R1_GPIO_Port, KEYPAD_R2_GPIO_Port, KEYPAD_R3_GPIO_Port, KEYPAD_R4_GPIO_Port},
    .row_pins  = {KEYPAD_R1_Pin, KEYPAD_R2_Pin, KEYPAD_R3_Pin, KEYPAD_R4_Pin},
    // Puertos y pines para las columnas del teclado
    .col_ports = {KEYPAD_C1_GPIO_Port, KEYPAD_C2_GPIO_Port, KEYPAD_C3_GPIO_Port, KEYPAD_C4_GPIO_Port},
    .col_pins  = {KEYPAD_C1_Pin, KEYPAD_C2_Pin, KEYPAD_C3_Pin, KEYPAD_C4_Pin}
};
//Se crea un buffer circular donde se almacenan las teclas presionadas
#define KEYPAD_BUFFER_LEN 16
uint8_t keypad_buffer[KEYPAD_BUFFER_LEN];
ring_buffer_t keypad_rb; //Permite leer y escribir sin perder datos cuando llegan varias teclas seguidas

// Guardar la contraseña ingresada e indica cuántos caracteres se han digitado
char entered_password[PASSWORD_LEN + 1] = {0};
uint8_t password_index = 0;

// VARIABLES DE TEMPORIZACION
uint32_t last_key_press_time = 0; // Tiempo de la última tecla procesada (para debounce)
uint32_t led_timer_start = 0; // Tiempo en que se encendió el LED
uint32_t led_on_duration = 0; // Duración que el LED debe permanecer encendido
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void); //Inicializar pines de GPIO
static void MX_USART2_UART_Init(void);//Inicializar pines de USART2
static void MX_TIM3_Init(void);//Inicializar pines de TIM3
/* USER CODE BEGIN PFP */
void manage_led_timer(void); //Apaga los LEDs después de un tiempo
void process_key(uint8_t key);//Esta es la lógica principal al presionar una tecla
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief  Callback de la interrupción externa GPIO.
  * @note   Esta función se llama cuando se detecta un flanco en un pin configurado para interrupción.
  *         Se mantiene muy rápida: solo lee el teclado y guarda la tecla en un buffer.
  */
 //Se ejecuta cuando se presiona una tecla del keypad
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    char key = keypad_scan(&keypad, GPIO_Pin); //Identifica cuál tecla fue la presionada
    if (key != '\0') {
        ring_buffer_write(&keypad_rb, (uint8_t)key);//Guarda la tecla en el buffer circular
    }
}

/**
 * @brief Procesa una tecla recibida del buffer del keypad.
 * @note  Contiene la lógica principal de la aplicación: feedback visual,
 *        almacenamiento de la contraseña y verificación.
 * @param key La tecla presionada a procesar.
 */
void process_key(uint8_t key)
{
    // Feedback visual inmediato al presionar cualquier tecla
    led_on(&led1);
    led_timer_start = HAL_GetTick();
    led_on_duration = FEEDBACK_LED_TIME_MS;

    // Almacenar el dígito ingresado
    if (password_index < PASSWORD_LEN) {
        entered_password[password_index++] = (char)key;
        printf("Tecla presionada: %c\r\n", key);
    }

    // Verificar si se ha ingresado la contraseña completa
    if (password_index == PASSWORD_LEN) {
        if (strncmp(entered_password, PASSWORD, PASSWORD_LEN) == 0) {
            printf("Contraseña correcta. Paso autorizado.\r\n");
            // Encender el LED para indicar éxito
            led_on(&led1);
            led_on(&led_ext);
            HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // Iniciar PWM para efecto de luz
            for (int i = 0; i <= 999; i++) {
              __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, i);
              HAL_Delay(5);  // Controla la velocidad de cambio de brillo
            }
            HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1); // Detener PWM después del efecto
            led_timer_start = HAL_GetTick();
            led_on_duration = SUCCESS_LED_TIME_MS;
        } else {
            printf("Contraseña incorrecta. Intente nuevamente\r\n");
           // Apagar el LED para indicar fallo
            led_off(&led1);
            led_off(&led_ext);
            HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1); // Detener PWM
        }

        // Reiniciar para el próximo intento
        password_index = 0;
        memset(entered_password, 0, sizeof(entered_password));
        printf("\n Ingrese la contraseña de 4 digitos y un caracter especial.\r\n");
    }
}

/**
 * @brief Gestiona el apagado automático de los LEDs sin bloquear el programa.
 * @note  Esta función debe ser llamada repetidamente en el bucle principal.
 */
void manage_led_timer(void)

{
  // Manejo del temporizador del LED (no bloqueante)
    if (led_timer_start != 0 && (HAL_GetTick() - led_timer_start > led_on_duration)) {
      //// Apagar los LEDs después del tiempo definido
        led_off(&led1);
        led_off(&led_ext);
        led_timer_start = 0;
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

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config(); // Configura el reloj del sistema

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  // Inicialización de los drivers personalizados
  led_init(&led1);
  led_init(&led_ext);
  ring_buffer_init(&keypad_rb, keypad_buffer, KEYPAD_BUFFER_LEN);
  keypad_init(&keypad); // Configura las filas como entradas con pull-down y las columnas como salidas

  printf("El Sistema de Control se ha inicializado\r\n");
  printf("Ingrese la contraseña (4 dígitos y 1 caracter especial).\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint8_t key_from_buffer;
  /** * @brief Procesa las teclas del buffer con anti-rebote.
    * @note  Esta sección lee las teclas del buffer y las procesa
    *        aplicando un tiempo de anti-rebote.
  */

    
    if (ring_buffer_read(&keypad_rb, &key_from_buffer)) { //Corazón del programa: leer tecla del buffer
        uint32_t now = HAL_GetTick();
        // Procesar la tecla si ha pasado el tiempo de anti-rebote.
        if (now - last_key_press_time > DEBOUNCE_TIME_MS) {
            last_key_press_time = now;
            process_key(key_from_buffer);
        }
    }


    /**
     * @brief Gestiona el temporizador del LED (no bloqueante).
      * @note  Esta función apaga los LEDs después de un tiempo definido sin bloquear el programa.
      */
    manage_led_timer();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) //Configura el reloj del sistema
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 79;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  #ifdef GNUC
    setvbuf(stdout, NULL, _IONBF, 0);
  #endif
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
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|LED_EXT_Pin|KEYPAD_R1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, KEYPAD_R2_Pin|KEYPAD_R4_Pin|KEYPAD_R3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin LED_EXT_Pin KEYPAD_R1_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|LED_EXT_Pin|KEYPAD_R1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : KEYPAD_C1_Pin */
  GPIO_InitStruct.Pin = KEYPAD_C1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEYPAD_C1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : KEYPAD_C4_Pin */
  GPIO_InitStruct.Pin = KEYPAD_C4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEYPAD_C4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : KEYPAD_C2_Pin KEYPAD_C3_Pin */
  GPIO_InitStruct.Pin = KEYPAD_C2_Pin|KEYPAD_C3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : KEYPAD_R2_Pin KEYPAD_R4_Pin KEYPAD_R3_Pin */
  GPIO_InitStruct.Pin = KEYPAD_R2_Pin|KEYPAD_R4_Pin|KEYPAD_R3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// Sobrescribir la función _write para redirigir printf a la UART
/**
*@brief Redirige la salida de printf a través de UART.
 @param file Descriptor de archivo (no usado).
 @param ptr Puntero a los datos a enviar.
 @param len Longitud de los datos.
 @return Número de bytes enviados.
 */
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
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
