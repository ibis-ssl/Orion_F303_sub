/* F303 sub基板の更新待機中に使用する安全IO APIを宣言する。 */
#pragma once

#include <stdbool.h>

void board_io_init_safe(void);
void board_status_set_validating(bool enabled);
void board_status_set_invalid(bool enabled);
