#include "main.h"
#include "util.h"
#include "usb_device.h"
#include "gba_cart.h"
#include "usbd_cdc_if.h"
#include "host_interface.h"
#include "usart.h"

void SystemClock_Config(void);

/**
 * @brief  The application entry point.
 * @retval int
 */

int main(void)
{
    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();
    /* Configure the system clock */
    SystemClock_Config();
    /* Initialize all configured peripherals */
    MX_USB_DEVICE_Init();
    MX_USART1_UART_Init();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    __HAL_RCC_AFIO_CLK_ENABLE();

    __HAL_AFIO_REMAP_SWJ_DISABLE();

    gba_cart_init();

    HAL_Delay(1000);

    uart_printf("starting loop\n");

    static int count = 0;

    while (1) {
        //usb_printf("%d ------------------------\n", count++);
        //usb_printf("GPIOA: CRL: %s\n", itox(GPIOA->CRL));
        //usb_printf("GPIOA: CRH: %s\n", itox(GPIOA->CRH));
        //usb_printf("GPIOA: IDR: %s\n", itox(GPIOA->IDR));
        //usb_printf("GPIOA: ODR: %s\n", itox(GPIOA->ODR));

        //usb_printf("GPIOB: CRL: %s\n", itox(GPIOB->CRL));
        //usb_printf("GPIOB: CRH: %s\n", itox(GPIOB->CRH));
        //usb_printf("GPIOB: IDR: %s\n", itox(GPIOB->IDR));
        //usb_printf("GPIOB: ODR: %s\n", itox(GPIOB->ODR));

        //usb_printf("GPIOC: CRL: %s\n", itox(GPIOC->CRL));
        //usb_printf("GPIOC: CRH: %s\n", itox(GPIOC->CRH));
        //usb_printf("GPIOC: IDR: %s\n", itox(GPIOC->IDR));
        //usb_printf("GPIOC: ODR: %s\n", itox(GPIOC->ODR));

        //usb_printf("GPIOD: CRL: %s\n", itox(GPIOD->CRL));
        //usb_printf("GPIOD: CRH: %s\n", itox(GPIOD->CRH));
        //usb_printf("GPIOD: IDR: %s\n", itox(GPIOD->IDR));
        //usb_printf("GPIOD: ODR: %s\n", itox(GPIOD->ODR));
        //for (uint32_t i = 0; i < 512; i++)
        //    gba_cart_rom_read(i, &gba_buf[i], 1);
        //usb_printf("starting benchmark...\n");
        //uint32_t benchmark_start = HAL_GetTick();
        //for (uint32_t i = 0; i < 0x800000; i += 512) {
        //    gba_cart_rom_read(i, gba_buf, 512);
        //}
        //uint32_t benchmark_duration = HAL_GetTick() - benchmark_start;
        //usb_printf("reading 16 MB ROM took %u milliseconds\n", benchmark_duration);

        //gba_cart_rom_read(0, gba_buf, 512);

        //for (int i = 0; i < 512; i += 8) {
        //    usb_printf("%s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s\n",
        //            itox8((uint8_t)gba_buf[i+0]), itox8((uint8_t)(gba_buf[i+0] >> 8)),
        //            itox8((uint8_t)gba_buf[i+1]), itox8((uint8_t)(gba_buf[i+1] >> 8)),
        //            itox8((uint8_t)gba_buf[i+2]), itox8((uint8_t)(gba_buf[i+2] >> 8)),
        //            itox8((uint8_t)gba_buf[i+3]), itox8((uint8_t)(gba_buf[i+3] >> 8)),
        //            itox8((uint8_t)gba_buf[i+4]), itox8((uint8_t)(gba_buf[i+4] >> 8)),
        //            itox8((uint8_t)gba_buf[i+5]), itox8((uint8_t)(gba_buf[i+5] >> 8)),
        //            itox8((uint8_t)gba_buf[i+6]), itox8((uint8_t)(gba_buf[i+6] >> 8)),
        //            itox8((uint8_t)gba_buf[i+7]), itox8((uint8_t)(gba_buf[i+7] >> 8)));
        //}

        //gba_cart_test();

        //HAL_Delay(1000);
        hostif_run();
    }
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

    /**Initializes the CPU, AHB and APB busses clocks 
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
        Error_Handler();
    /**Initializes the CPU, AHB and APB busses clocks 
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
        |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
        Error_Handler();
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
    PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
        Error_Handler();
}

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
    usb_printf("ASSERT FAILED in file <%s> line #%lu\n", file, line);
}
#endif /* USE_FULL_ASSERT */
