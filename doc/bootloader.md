# F303 sub アプリケーションブートローダー

## 目的

STM32内蔵System Memory bootloaderを使わず、基板固有の安全IOを維持したままCAN経由更新へ発展できる常駐bootloaderを置く。現在のM1は安全IO、metadata、CRC32C、通常アプリへのjumpまでを実装し、OFW-CAN受信とFlash更新は次段階とする。

## Flash配置

| 領域 | アドレス | サイズ |
| --- | --- | ---: |
| Bootloader | `0x08000000`～`0x08003FFF` | 16 KB |
| Application | `0x08004000`～`0x0801F7FF` | 110 KB |
| Metadata | `0x0801F800`～`0x0801FFFF` | 2 KB |

通常アプリのVTORは`0x08004000`。metadataのmagic、形式、状態、base、size、metadata CRC32C、初期MSP、Reset_Handler、application CRC32Cが全て正常な場合だけjumpする。

## 更新中の安全IO

- TIM3をperipheral resetしclock disableする。
- PB0（dribbler ESC）とPB1（servo）をGPIO Output Lowへ固定し、PWM pulseを出さない。
- PC13～PC15、PA0/PA6/PA7、PB6/PB13/PB14は現行`MX_GPIO_Init()`と同じOutput Low。
- PA3/PA8、PB12/PB15は現行どおりInput pull-up。
- PA13/PA14はSWDを維持する。
- その他のbonding済みpinはAnalog no-pullとし、CANを含むperipheralはM1では開始しない。

## ビルド

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\Script\build_bootloader.ps1 -Rebuild
powershell -NoProfile -ExecutionPolicy Bypass -File .\Script\build_application.ps1 -Configuration Debug -Rebuild
```

## 初回導入

2台のST-Linkを接続する場合は、sub側probe serialを必ず指定する。最初は`-Execute`なしで128 KB FlashとOption Bytesを退避する。

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\Script\install_sub_bootloader.ps1 `
  -Configuration Debug `
  -ProbeSerial 003300343033510735393935
```

backup確認後だけ`-Execute`を付ける。Device IDがF303xB/Cの`0x422`でなければ書込み前に中止する。

## 現在の検証結果

- bootloader: 1,344 byte、vector `0x08000000`
- application Debug: 63,672 byte、vector `0x08004000`
- image CRC32C: `0x54A88A33`
- metadata CRC32C: `0x101D8A68`
- CRC32C既知ベクトル、metadata構造、PowerShell構文検査に合格
- G474接続中にsub書込みを試みた場合、Device ID guardが書込み前に拒否することを確認
- sub側ST-Link `003300343033510735393935`は3.28 Vを検出するが、SWD core IDを取得できていないため実機導入は未実施

実機導入前にsub側のSWDIO、SWCLK、NRST、GND配線を確認する。
