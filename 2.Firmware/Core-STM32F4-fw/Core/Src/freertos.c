/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "common_inc.h"
#include "communication.hpp"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

// List of semaphores
osSemaphoreId sem_usb_irq;
osSemaphoreId sem_uart4_dma;
osSemaphoreId sem_uart5_dma;
osSemaphoreId sem_usb_rx;
osSemaphoreId sem_usb_tx;
osSemaphoreId sem_can1_tx;
osSemaphoreId sem_can2_tx;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 2000,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */


/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
    // USB 延迟中断(Deferred IRQ)信号量：二值信号量，初始无令牌。
    // 由 USB 中断服务例程释放，用于唤醒延迟处理线程。
    osSemaphoreDef(sem_usb_irq);
    sem_usb_irq = osSemaphoreNew(1, 0, osSemaphore(sem_usb_irq));

    // UART4/5 DMA 传输同步：互斥式信号量，初始可用。
    osSemaphoreDef(sem_uart4_dma);
    sem_uart4_dma = osSemaphoreNew(1, 1, osSemaphore(sem_uart4_dma));
    osSemaphoreDef(sem_uart5_dma);
    sem_uart5_dma = osSemaphoreNew(1, 1, osSemaphore(sem_uart5_dma));

    // USB 接收同步：二值信号量，初始无令牌，配合 CDC_Receive 回调使用。
    osSemaphoreDef(sem_usb_rx);
    sem_usb_rx = osSemaphoreNew(1, 0, osSemaphore(sem_usb_rx));

    // USB 发送同步：互斥式信号量，防止并发写导致 BUSY。
    osSemaphoreDef(sem_usb_tx);
    sem_usb_tx = osSemaphoreNew(1, 1, osSemaphore(sem_usb_tx));

    // CAN 发送同步：互斥式信号量，序列化发包。
    osSemaphoreDef(sem_can1_tx);
    sem_can1_tx = osSemaphoreNew(1, 1, osSemaphore(sem_can1_tx));
    osSemaphoreDef(sem_can2_tx);
    sem_can2_tx = osSemaphoreNew(1, 1, osSemaphore(sem_can2_tx));

  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */

  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
    // USB 延迟中断线程：必须在 MX_USB_DEVICE_Init() 之前创建，
    // 用于将硬件 IRQ 的工作量下沉到线程上下文，避免在中断里做重逻辑。
    const osThreadAttr_t usbIrqTask_attributes = {
        .name = "usbIrqTask",
        .stack_size = 500,
        .priority = (osPriority_t) osPriorityAboveNormal,
    };
    usbIrqTaskHandle = osThreadNew(UsbDeferredInterruptTask, NULL, &usbIrqTask_attributes);

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
    /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */

    // 跳转至 C++ 版本入口 Main()：初始化通信、机器人、传感器与用户线程。
    Main();

    vTaskDelete(defaultTaskHandle);
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
