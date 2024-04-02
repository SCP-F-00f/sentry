#include "rng.h"
 
/* USER CODE BEGIN 0 */
 
/* USER CODE END 0 */
 
RNG_HandleTypeDef hrng;
 
/* RNG init function */
void MX_RNG_Init(void)
{
 
  /* USER CODE BEGIN RNG_Init 0 */
 
  /* USER CODE END RNG_Init 0 */
 
  /* USER CODE BEGIN RNG_Init 1 */
 
  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
//  if (HAL_RNG_Init(&hrng) != HAL_OK)
//  {
//    Error_Handler();
//  }
  /* USER CODE BEGIN RNG_Init 2 */
 
  /* USER CODE END RNG_Init 2 */
 
}
 
void HAL_RNG_MspInit(RNG_HandleTypeDef* rngHandle)
{
 
  if(rngHandle->Instance==RNG)
  {
  /* USER CODE BEGIN RNG_MspInit 0 */
 
  /* USER CODE END RNG_MspInit 0 */
    /* RNG clock enable */
    __HAL_RCC_RNG_CLK_ENABLE();
 
    /* RNG interrupt Init */
    HAL_NVIC_SetPriority(HASH_RNG_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(HASH_RNG_IRQn);
  /* USER CODE BEGIN RNG_MspInit 1 */
 
  /* USER CODE END RNG_MspInit 1 */
  }
}
 
void HAL_RNG_MspDeInit(RNG_HandleTypeDef* rngHandle)
{
 
  if(rngHandle->Instance==RNG)
  {
  /* USER CODE BEGIN RNG_MspDeInit 0 */
 
  /* USER CODE END RNG_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_RNG_CLK_DISABLE();
 
    /* RNG interrupt Deinit */
    HAL_NVIC_DisableIRQ(HASH_RNG_IRQn);
  /* USER CODE BEGIN RNG_MspDeInit 1 */
 
  /* USER CODE END RNG_MspDeInit 1 */
  }
}