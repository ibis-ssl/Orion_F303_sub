/* F303 subの全bonding GPIOを分類し、PWMを停止した更新用安全状態へ設定する。 */
#include "board_io.h"

#include "stm32f303xc.h"

#include <stdint.h>

#define PIN_MASK(pin) (UINT32_C(1) << (pin))

#define GPIOA_PACKAGE_MASK UINT32_C(0xFFFF)
#define GPIOB_PACKAGE_MASK UINT32_C(0xFFFF)
#define GPIOC_PACKAGE_MASK UINT32_C(0xE000)
#define GPIOF_PACKAGE_MASK UINT32_C(0x0003)

#define GPIOA_OUTPUT_MASK (PIN_MASK(0) | PIN_MASK(6) | PIN_MASK(7))
#define GPIOA_INPUT_MASK (PIN_MASK(3) | PIN_MASK(8))
#define GPIOA_ANALOG_MASK UINT32_C(0x9E36)
#define GPIOA_PRESERVE_MASK (PIN_MASK(13) | PIN_MASK(14))

/* PB0/PB1はTIM3 PWM pin。更新中はGPIO Lowへ固定する。 */
#define GPIOB_OUTPUT_MASK (PIN_MASK(0) | PIN_MASK(1) | PIN_MASK(6) | PIN_MASK(13) | PIN_MASK(14))
#define GPIOB_INPUT_MASK (PIN_MASK(12) | PIN_MASK(15))
#define GPIOB_ANALOG_MASK UINT32_C(0x0FBC)
#define GPIOB_PRESERVE_MASK UINT32_C(0)

#define GPIOC_OUTPUT_MASK UINT32_C(0xE000)
#define GPIOC_INPUT_MASK UINT32_C(0)
#define GPIOC_ANALOG_MASK UINT32_C(0)
#define GPIOC_PRESERVE_MASK UINT32_C(0)

#define GPIOF_OUTPUT_MASK UINT32_C(0)
#define GPIOF_INPUT_MASK UINT32_C(0)
#define GPIOF_ANALOG_MASK UINT32_C(0x0003)
#define GPIOF_PRESERVE_MASK UINT32_C(0)

#define ASSERT_PORT(name, package, output, input, analog, preserve) \
  _Static_assert((((output) & (input)) | ((output) & (analog)) | ((output) & (preserve)) | ((input) & (analog)) | ((input) & (preserve)) | ((analog) & (preserve))) == 0U, name " overlap"); \
  _Static_assert(((output) | (input) | (analog) | (preserve)) == (package), name " incomplete")

ASSERT_PORT("GPIOA", GPIOA_PACKAGE_MASK, GPIOA_OUTPUT_MASK, GPIOA_INPUT_MASK, GPIOA_ANALOG_MASK, GPIOA_PRESERVE_MASK);
ASSERT_PORT("GPIOB", GPIOB_PACKAGE_MASK, GPIOB_OUTPUT_MASK, GPIOB_INPUT_MASK, GPIOB_ANALOG_MASK, GPIOB_PRESERVE_MASK);
ASSERT_PORT("GPIOC", GPIOC_PACKAGE_MASK, GPIOC_OUTPUT_MASK, GPIOC_INPUT_MASK, GPIOC_ANALOG_MASK, GPIOC_PRESERVE_MASK);
ASSERT_PORT("GPIOF", GPIOF_PACKAGE_MASK, GPIOF_OUTPUT_MASK, GPIOF_INPUT_MASK, GPIOF_ANALOG_MASK, GPIOF_PRESERVE_MASK);

static void configure_port(GPIO_TypeDef * port, uint32_t package_mask, uint32_t output_mask, uint32_t input_mask, uint32_t analog_mask, uint32_t preserve_mask)
{
  port->BSRR = output_mask << 16U;
  port->OTYPER &= ~output_mask;
  for (uint32_t pin = 0; pin < 16U; pin++) {
    const uint32_t mask = PIN_MASK(pin);
    if ((package_mask & mask) == 0U || (preserve_mask & mask) != 0U) {
      continue;
    }
    const uint32_t shift = pin * 2U;
    port->OSPEEDR &= ~(UINT32_C(3) << shift);
    port->PUPDR &= ~(UINT32_C(3) << shift);
    if ((output_mask & mask) != 0U) {
      port->MODER = (port->MODER & ~(UINT32_C(3) << shift)) | (UINT32_C(1) << shift);
    } else if ((input_mask & mask) != 0U) {
      port->PUPDR |= UINT32_C(1) << shift; /* 現行MX_GPIO_Initと同じpull-up。 */
      port->MODER &= ~(UINT32_C(3) << shift);
    } else if ((analog_mask & mask) != 0U) {
      port->MODER |= UINT32_C(3) << shift;
    }
  }
}

void board_io_init_safe(void)
{
  RCC->APB1RSTR |= RCC_APB1RSTR_TIM3RST;
  RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM3RST;
  RCC->APB1ENR &= ~RCC_APB1ENR_TIM3EN;

  RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIOFEN;
  (void)RCC->AHBENR;
  configure_port(GPIOA, GPIOA_PACKAGE_MASK, GPIOA_OUTPUT_MASK, GPIOA_INPUT_MASK, GPIOA_ANALOG_MASK, GPIOA_PRESERVE_MASK);
  configure_port(GPIOB, GPIOB_PACKAGE_MASK, GPIOB_OUTPUT_MASK, GPIOB_INPUT_MASK, GPIOB_ANALOG_MASK, GPIOB_PRESERVE_MASK);
  configure_port(GPIOC, GPIOC_PACKAGE_MASK, GPIOC_OUTPUT_MASK, GPIOC_INPUT_MASK, GPIOC_ANALOG_MASK, GPIOC_PRESERVE_MASK);
  configure_port(GPIOF, GPIOF_PACKAGE_MASK, GPIOF_OUTPUT_MASK, GPIOF_INPUT_MASK, GPIOF_ANALOG_MASK, GPIOF_PRESERVE_MASK);
}

void board_status_set_validating(bool enabled)
{
  GPIOC->BSRR = enabled ? PIN_MASK(13) : (PIN_MASK(13) << 16U);
}

void board_status_set_invalid(bool enabled)
{
  GPIOC->BSRR = enabled ? PIN_MASK(14) : (PIN_MASK(14) << 16U);
}
