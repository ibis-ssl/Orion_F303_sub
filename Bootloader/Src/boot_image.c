/* metadata、vector、CRC32Cを検証し、正常なF303 subアプリへreset相当状態でjumpする。 */
#include "boot_image.h"

#include "boot_crc32c.h"
#include "stm32f303xc.h"

#include <stddef.h>
#include <stdint.h>

static const boot_image_metadata_t * const app_metadata = (const boot_image_metadata_t *)BOOT_METADATA_BASE;

static bool stack_pointer_is_valid(uint32_t stack_pointer)
{
  const bool in_sram = stack_pointer >= BOOT_SRAM_BASE && stack_pointer <= BOOT_SRAM_END;
  const bool in_ccmram = stack_pointer >= BOOT_CCMRAM_BASE && stack_pointer <= BOOT_CCMRAM_END;
  return (in_sram || in_ccmram) && (stack_pointer & 7U) == 0U;
}

bool boot_app_is_valid(void)
{
  const boot_image_metadata_t metadata = *app_metadata;
  if (metadata.magic != BOOT_IMAGE_METADATA_MAGIC || metadata.format_version != BOOT_IMAGE_METADATA_FORMAT || metadata.record_size != sizeof(metadata)) {
    return false;
  }
  if (metadata.state != BOOT_IMAGE_STATE_CONFIRMED || metadata.slot != BOOT_APP_SLOT || metadata.image_base != BOOT_APP_BASE) {
    return false;
  }
  if (metadata.image_size < 8U || metadata.image_size > BOOT_APP_SIZE) {
    return false;
  }
  if (boot_crc32c(&metadata, offsetof(boot_image_metadata_t, record_crc32c)) != metadata.record_crc32c) {
    return false;
  }
  const uint32_t image_end = metadata.image_base + metadata.image_size;
  if (image_end < metadata.image_base || image_end > BOOT_APP_BASE + BOOT_APP_SIZE) {
    return false;
  }
  const uint32_t stack_pointer = *(const uint32_t *)BOOT_APP_BASE;
  const uint32_t reset_handler = *(const uint32_t *)(BOOT_APP_BASE + 4U);
  const uint32_t handler_address = reset_handler & ~UINT32_C(1);
  if (!stack_pointer_is_valid(stack_pointer) || (reset_handler & 1U) == 0U || handler_address < BOOT_APP_BASE || handler_address >= image_end) {
    return false;
  }
  return boot_crc32c((const void *)BOOT_APP_BASE, metadata.image_size) == metadata.image_crc32c;
}

static void boot_branch(uint32_t stack_pointer, uint32_t reset_handler) __attribute__((naked, noreturn));
static void boot_branch(uint32_t stack_pointer __attribute__((unused)), uint32_t reset_handler __attribute__((unused)))
{
  __asm volatile(
    "movs r2, #0\n"
    "msr control, r2\n"
    "msr basepri, r2\n"
    "msr faultmask, r2\n"
    "isb\n"
    "msr msp, r0\n"
    "msr primask, r2\n"
    "bx r1\n");
}

void boot_jump_to_app(void)
{
  const uint32_t stack_pointer = *(const uint32_t *)BOOT_APP_BASE;
  const uint32_t reset_handler = *(const uint32_t *)(BOOT_APP_BASE + 4U);
  __disable_irq();
  SysTick->CTRL = 0U;
  SysTick->LOAD = 0U;
  SysTick->VAL = 0U;
  for (uint32_t index = 0; index < 8U; index++) {
    NVIC->ICER[index] = UINT32_MAX;
    NVIC->ICPR[index] = UINT32_MAX;
  }
  SCB->VTOR = BOOT_APP_BASE;
  __DSB();
  __ISB();
  boot_branch(stack_pointer, reset_handler);
}
