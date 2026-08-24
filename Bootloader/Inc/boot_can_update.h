/* F303 sub bootloaderのCAN更新プロトコル、Flash書込み、完了metadata生成を担当する。 */
#pragma once

#include <stdbool.h>

bool boot_can_update_run(unsigned int idle_loops);
