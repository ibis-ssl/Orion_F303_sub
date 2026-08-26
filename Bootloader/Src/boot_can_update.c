/* F303 subをCAN1経由で更新するためのpolled bxCAN/Flash実装を提供する。 */
#include "boot_can_update.h"

#include "boot_config.h"
#include "boot_crc32c.h"
#include "boot_image.h"
#include "stm32f303xc.h"

#include <stddef.h>
#include <stdint.h>

#define CAN_COMMAND_ID UINT32_C(0x610)
#define CAN_VERSION_ID UINT32_C(0x611)
#define CAN_DATA_ID_BASE UINT32_C(0x480)
#define CAN_DATA_ID_LAST UINT32_C(0x4FF)
#define CAN_RESPONSE_ID UINT32_C(0x654)
#define NODE_ID 4U
#define BLOCK_CAPACITY 896U
#define RX_FIFO_CAPACITY 32U

enum { CMD_HELLO = 1, CMD_BEGIN = 2, CMD_SET_CRC = 3, CMD_BLOCK_BEGIN = 4, CMD_BLOCK_END = 5, CMD_END = 6, CMD_REBOOT = 7 };
enum { STATUS_OK = 0, STATUS_COMMAND = 1, STATUS_RANGE = 2, STATUS_SEQUENCE = 3, STATUS_CRC = 4, STATUS_FLASH = 5 };

static uint8_t block[BLOCK_CAPACITY];
static uint32_t image_size;
static uint32_t image_crc;
static uint32_t received;
static uint32_t block_offset;
static uint16_t block_length;
static uint8_t block_token;
static uint32_t block_bitmap[4];
static uint8_t session;
static bool receiving;
static bool begun;
typedef struct { uint32_t id; uint8_t data[8]; } can_frame_t;
static can_frame_t rx_fifo[RX_FIFO_CAPACITY];
static uint8_t rx_head;
static uint8_t rx_tail;
static uint8_t rx_count;
static bool rx_overflow;

static void can_init(void)
{
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
  RCC->APB1ENR |= RCC_APB1ENR_CANEN;
  GPIOA->MODER = (GPIOA->MODER & ~((UINT32_C(3) << 22U) | (UINT32_C(3) << 24U))) | (UINT32_C(2) << 22U) | (UINT32_C(2) << 24U);
  GPIOA->AFR[1] = (GPIOA->AFR[1] & ~((UINT32_C(0xF) << 12U) | (UINT32_C(0xF) << 16U))) | (UINT32_C(9) << 12U) | (UINT32_C(9) << 16U);
  CAN->MCR = CAN_MCR_INRQ | CAN_MCR_ABOM;
  while ((CAN->MSR & CAN_MSR_INAK) == 0U) {}
  CAN->BTR = (UINT32_C(4) << CAN_BTR_TS1_Pos) | (UINT32_C(1) << CAN_BTR_TS2_Pos); /* HSI 8MHz / 8TQ = 1Mbps */
  CAN->FMR |= CAN_FMR_FINIT;
  CAN->FA1R = 0U;
  CAN->FS1R = 3U;
  CAN->FM1R = 0U; /* 32-bit mask filters: exact command and data-ID range. */
  CAN->FFA1R = 0U;
  CAN->sFilterRegister[0].FR1 = CAN_COMMAND_ID << 21U;
  CAN->sFilterRegister[0].FR2 = UINT32_C(0x7FE) << 21U;
  CAN->sFilterRegister[1].FR1 = CAN_DATA_ID_BASE << 21U;
  CAN->sFilterRegister[1].FR2 = UINT32_C(0x780) << 21U;
  CAN->FA1R = 3U;
  CAN->FMR &= ~CAN_FMR_FINIT;
  CAN->MCR &= ~CAN_MCR_INRQ;
  while ((CAN->MSR & CAN_MSR_INAK) != 0U) {}
}

static bool can_read_hardware(uint32_t * id, uint8_t data[8])
{
  if ((CAN->RF0R & CAN_RF0R_FMP0_Msk) == 0U) return false;
  const CAN_FIFOMailBox_TypeDef * mb = &CAN->sFIFOMailBox[0];
  *id = (mb->RIR >> 21U) & UINT32_C(0x7FF);
  const uint32_t low = mb->RDLR;
  const uint32_t high = mb->RDHR;
  for (uint32_t i = 0; i < 4U; i++) data[i] = (uint8_t)(low >> (i * 8U));
  for (uint32_t i = 0; i < 4U; i++) data[i + 4U] = (uint8_t)(high >> (i * 8U));
  CAN->RF0R |= CAN_RF0R_RFOM0;
  return true;
}

static void can_drain_hardware(void)
{
  if ((CAN->RF0R & CAN_RF0R_FOVR0) != 0U) {
    CAN->RF0R |= CAN_RF0R_FOVR0;
    rx_overflow = true;
  }
  while ((CAN->RF0R & CAN_RF0R_FMP0_Msk) != 0U) {
    if (rx_count >= RX_FIFO_CAPACITY) {
      uint32_t discarded_id;
      uint8_t discarded[8];
      (void)can_read_hardware(&discarded_id, discarded);
      rx_overflow = true;
      continue;
    }
    can_frame_t * frame = &rx_fifo[rx_tail];
    (void)can_read_hardware(&frame->id, frame->data);
    rx_tail = (uint8_t)((rx_tail + 1U) % RX_FIFO_CAPACITY);
    rx_count++;
  }
}

static bool can_fifo_pop(uint32_t * id, uint8_t data[8])
{
  if (rx_count == 0U) return false;
  const can_frame_t * frame = &rx_fifo[rx_head];
  *id = frame->id;
  for (uint32_t i = 0; i < 8U; i++) data[i] = frame->data[i];
  rx_head = (uint8_t)((rx_head + 1U) % RX_FIFO_CAPACITY);
  rx_count--;
  return true;
}

static void can_send(const uint8_t data[8])
{
  while ((CAN->TSR & CAN_TSR_TME0) == 0U) {}
  CAN_TxMailBox_TypeDef * mb = &CAN->sTxMailBox[0];
  mb->TDTR = 8U;
  mb->TDLR = (uint32_t)data[0] | ((uint32_t)data[1] << 8U) | ((uint32_t)data[2] << 16U) | ((uint32_t)data[3] << 24U);
  mb->TDHR = (uint32_t)data[4] | ((uint32_t)data[5] << 8U) | ((uint32_t)data[6] << 16U) | ((uint32_t)data[7] << 24U);
  mb->TIR = (CAN_RESPONSE_ID << 21U) | CAN_TI0R_TXRQ;
}

static void can_send_version(void)
{
  uint32_t build_id = 0U, image_crc = 0U;
  const uint32_t * descriptor = (const uint32_t *)(BOOT_APP_BASE + UINT32_C(0x400));
  if (boot_app_is_valid() && descriptor[0] == UINT32_C(0x52565746)) {
    build_id = descriptor[1];
    image_crc = *(const uint32_t *)(BOOT_METADATA_BASE + 28U);
  }
  const uint8_t data[8] = {(uint8_t)build_id,(uint8_t)(build_id>>8U),(uint8_t)(build_id>>16U),(uint8_t)(build_id>>24U),(uint8_t)image_crc,(uint8_t)(image_crc>>8U),(uint8_t)(image_crc>>16U),(uint8_t)(image_crc>>24U)};
  while ((CAN->TSR & CAN_TSR_TME0) == 0U) {}
  CAN_TxMailBox_TypeDef * mb = &CAN->sTxMailBox[0];
  mb->TDTR = 8U;
  mb->TDLR = (uint32_t)data[0]|((uint32_t)data[1]<<8U)|((uint32_t)data[2]<<16U)|((uint32_t)data[3]<<24U);
  mb->TDHR = (uint32_t)data[4]|((uint32_t)data[5]<<8U)|((uint32_t)data[6]<<16U)|((uint32_t)data[7]<<24U);
  mb->TIR = (UINT32_C(0x664) << 21U) | CAN_TI0R_TXRQ;
}

static void respond(uint8_t command, uint8_t status, uint32_t value)
{
  const uint8_t reply[8] = { (uint8_t)(command | 0x80U), status, NODE_ID, block_token, (uint8_t)value, (uint8_t)(value >> 8U), (uint8_t)(value >> 16U), (uint8_t)(value >> 24U) };
  can_send(reply);
}

static bool flash_wait(void)
{
  while ((FLASH->SR & FLASH_SR_BSY) != 0U) {}
  const uint32_t errors = FLASH->SR & (FLASH_SR_PGERR | FLASH_SR_WRPERR);
  FLASH->SR = FLASH_SR_EOP | FLASH_SR_PGERR | FLASH_SR_WRPERR;
  return errors == 0U;
}

static void flash_unlock(void)
{
  if ((FLASH->CR & FLASH_CR_LOCK) != 0U) { FLASH->KEYR = UINT32_C(0x45670123); FLASH->KEYR = UINT32_C(0xCDEF89AB); }
}

static bool flash_erase_page(uint32_t address)
{
  flash_unlock();
  if (!flash_wait()) return false;
  FLASH->CR = FLASH_CR_PER;
  FLASH->AR = address;
  FLASH->CR |= FLASH_CR_STRT;
  const bool ok = flash_wait();
  FLASH->CR = 0U;
  return ok;
}

static bool flash_program(uint32_t address, const uint8_t * data, uint32_t length)
{
  flash_unlock();
  for (uint32_t i = 0; i < length; i += 2U) {
    const uint16_t value = (uint16_t)data[i] | ((uint16_t)(i + 1U < length ? data[i + 1U] : UINT8_C(0xFF)) << 8U);
    FLASH->CR = FLASH_CR_PG;
    *(volatile uint16_t *)(address + i) = value;
    if (!flash_wait() || *(const uint16_t *)(address + i) != value) { FLASH->CR = 0U; return false; }
  }
  FLASH->CR = 0U;
  return true;
}

static uint32_t load_u32(const uint8_t * p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8U) | ((uint32_t)p[2] << 16U) | ((uint32_t)p[3] << 24U); }

static bool block_is_complete(void)
{
  const uint32_t frame_count = ((uint32_t)block_length + 6U) / 7U;
  for (uint32_t sequence = 0U; sequence < frame_count; sequence++) {
    if ((block_bitmap[sequence / 32U] & (UINT32_C(1) << (sequence % 32U))) == 0U) return false;
  }
  return true;
}

static bool write_metadata(void)
{
  boot_image_metadata_t metadata = { BOOT_IMAGE_METADATA_MAGIC, BOOT_IMAGE_METADATA_FORMAT, sizeof(metadata), 1U, BOOT_IMAGE_STATE_CONFIRMED, BOOT_APP_SLOT, BOOT_APP_BASE, image_size, image_crc, 0U };
  metadata.record_crc32c = boot_crc32c(&metadata, offsetof(boot_image_metadata_t, record_crc32c));
  return flash_erase_page(BOOT_METADATA_BASE) && flash_program(BOOT_METADATA_BASE, (const uint8_t *)&metadata, sizeof(metadata));
}

static void handle_command(const uint8_t data[8])
{
  const uint8_t command = data[0];
  const uint8_t target = data[1];
  if (target != 0U && target != NODE_ID) return;
  if (command == CMD_HELLO) { block_token = data[2]; respond(command, STATUS_OK, received); return; }
  if (command == CMD_BEGIN) {
    const uint32_t requested_size = load_u32(&data[4]);
    const uint8_t requested_session = data[2];
    block_token = requested_session;
    if (begun && requested_session == session && requested_size == image_size) { respond(command, STATUS_OK, received); return; }
    session = requested_session;
    image_size = requested_size; received = 0U; receiving = false;
    if (image_size < 8U || image_size > BOOT_APP_SIZE) { respond(command, STATUS_RANGE, 0U); return; }
    if (!flash_erase_page(BOOT_METADATA_BASE)) { respond(command, STATUS_FLASH, 0U); return; }
    for (uint32_t address = BOOT_APP_BASE; address < BOOT_APP_BASE + BOOT_APP_SIZE; address += UINT32_C(0x800)) {
      if (!flash_erase_page(address)) { respond(command, STATUS_FLASH, address); return; }
    }
    begun = true; respond(command, STATUS_OK, 0U); return;
  }
  if (command == CMD_SET_CRC) { block_token = data[2]; image_crc = load_u32(&data[4]); respond(command, STATUS_OK, image_crc); return; }
  if (command == CMD_BLOCK_BEGIN) {
    block_offset = load_u32(&data[4]);
    block_token = data[2];
    const uint32_t remaining = block_offset < image_size ? image_size - block_offset : 0U;
    block_length = (uint16_t)(remaining < BLOCK_CAPACITY ? remaining : BLOCK_CAPACITY);
    for (uint32_t i = 0U; i < 4U; i++) block_bitmap[i] = 0U;
    rx_overflow = false;
    receiving = begun && block_offset == received && block_offset < image_size;
    respond(command, receiving ? STATUS_OK : STATUS_RANGE, received); return;
  }
  if (command == CMD_BLOCK_END) {
    block_token = data[2];
    if (!receiving || rx_overflow || !block_is_complete()) { receiving = false; respond(command, STATUS_SEQUENCE, received); return; }
    if (boot_crc32c(block, block_length) != load_u32(&data[4])) { receiving = false; respond(command, STATUS_CRC, received); return; }
    if (!flash_program(BOOT_APP_BASE + block_offset, block, block_length)) { respond(command, STATUS_FLASH, received); return; }
    received += block_length; receiving = false; respond(command, STATUS_OK, received); return;
  }
  if (command == CMD_END) {
    block_token = data[2];
    if (received != image_size || boot_crc32c((const void *)BOOT_APP_BASE, image_size) != image_crc) { respond(command, STATUS_CRC, received); return; }
    if (!write_metadata()) { respond(command, STATUS_FLASH, received); return; }
    respond(command, STATUS_OK, received); return;
  }
  if (command == CMD_REBOOT) { block_token = data[2]; respond(command, STATUS_OK, received); for (volatile uint32_t delay = 0; delay < 80000U; delay++) {} NVIC_SystemReset(); }
  respond(command, STATUS_COMMAND, received);
}

bool boot_can_update_run(unsigned int idle_loops)
{
  can_init();
  for (unsigned int idle = 0; idle < idle_loops || !boot_app_is_valid(); idle++) {
    uint32_t id;
    uint8_t data[8];
    can_drain_hardware();
    if (can_fifo_pop(&id, data)) {
      idle = 0U;
      if (id == CAN_VERSION_ID && data[0] == NODE_ID) {
        can_send_version();
      } else if (id == CAN_COMMAND_ID) {
        if (rx_overflow) { block_token = data[2]; rx_overflow = false; receiving = false; respond(data[0], STATUS_SEQUENCE, received); }
        else handle_command(data);
      }
      else if (id >= CAN_DATA_ID_BASE && id <= CAN_DATA_ID_LAST && receiving && data[0] == block_token) {
        const uint32_t sequence = id - CAN_DATA_ID_BASE;
        const uint32_t position = sequence * 7U;
        if (position < block_length && (block_bitmap[sequence / 32U] & (UINT32_C(1) << (sequence % 32U))) == 0U) {
          for (uint32_t i = 1U; i < 8U && position + i - 1U < block_length; i++) block[position + i - 1U] = data[i];
          block_bitmap[sequence / 32U] |= UINT32_C(1) << (sequence % 32U);
        }
      }
    }
  }
  return boot_app_is_valid();
}
