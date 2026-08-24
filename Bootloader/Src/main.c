/* F303 sub M1 bootloaderのentry pointとして安全IO、アプリ検証、jumpを実行する。 */
#include "board_io.h"
#include "boot_image.h"

int main(void)
{
  board_io_init_safe();
  board_status_set_validating(true);
  if (boot_app_is_valid()) {
    board_status_set_validating(false);
    boot_jump_to_app();
  }
  board_status_set_validating(false);
  board_status_set_invalid(true);
  for (;;) {
    __asm volatile("wfi");
  }
}
