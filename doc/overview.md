# Orion_F303_sub 概要

## 役割

STM32F303CBT6（LQFP48）を使用し、次のサブ基板機能をまとめて扱う。

- ドリブラーESCへのPWM指令
- サーボへのPWM指令
- ESCテレメトリの受信
- 2系統の遮断型ボールセンサ判定
- バッテリ電圧測定
- 状態のCAN送信とUARTデバッグ出力
- I2C接続ディスプレイへの状態表示

システムクロックは72 MHz。主要処理はTIM17の2 kHz割り込みから実行し、
メインループは待機する構成である。

## CAN通信

Classical CAN、標準11 bit ID、データ長8 byteを使用する。CAN初期化値は
Prescaler=2、BS1=9 TQ、BS2=8 TQ。現在のフィルタは全IDを受信し、アプリ側で
必要なIDを選別する。

| 方向 | CAN ID | 内容 | データ形式 |
| --- | ---: | --- | --- |
| 受信 | `0x104` | ドリブラー速度指令 | byte 0..3: float32 little endian |
| 受信 | `0x105` | サーボ角度指令 | byte 0..3: float32 little endian |
| 受信 | `0x300` | 予約（現在は処理なし） | 8 byte |
| 送信 | `0x204` | ESCテレメトリ速度 | byte 0..3: float32 little endian |
| 送信 | `0x214` | バッテリ電圧 | byte 0..3: float32 little endian |
| 送信 | `0x240` | ボール検出状態 | byte 0: 検出有無、byte 1: 位置 |

ドリブラー／サーボ指令は、約100 msごとの監視カウンタが50を超えると0へ戻る。
したがって、指令が約5秒途絶えると停止側へ移行する。

## UART

- USART1（PA9/PA10）: 2,000,000 bps。`printf`によるデバッグ出力と受信確認に使用。
- USART3（PB10/PB11）: 115,200 bps。ESCテレメトリ受信に使用。

USART1ログは次のコマンドで確認できる。

```powershell
powershell -ExecutionPolicy Bypass -File .\Script\monitor_uart.ps1 -Port COM3
```

## 周期処理

TIM17の2 kHzコールバックでボールセンサ処理を3段階に分けて実行する。

1. センサ0側を駆動し、ADC値を取得して速度テレメトリをCAN送信する。
2. センサ1側を駆動し、ADC値とバッテリ電圧を取得して電圧をCAN送信する。
3. 消灯時との差分からボール検出を判定し、結果をCAN送信する。

200割り込み（約100 ms）ごとにUARTログ、LED、PWM指令、通信タイムアウトを更新する。

## ビルドと書き込み

```powershell
# bootloaderと再配置済みDebugアプリをビルド
powershell -ExecutionPolicy Bypass -File .\Script\build_bootloader.ps1 -Rebuild
powershell -ExecutionPolicy Bypass -File .\Script\build_application.ps1 -Configuration Debug -Rebuild

# 初回はdry-runでbackupし、確認後だけ-Executeを付ける
powershell -ExecutionPolicy Bypass -File .\Script\install_sub_bootloader.ps1 -Configuration Debug -ProbeSerial <SUB_STLINK_SN>
powershell -ExecutionPolicy Bypass -File .\Script\install_sub_bootloader.ps1 -Configuration Debug -ProbeSerial <SUB_STLINK_SN> -Execute
```

通常アプリは`0x08004000`へ再配置済みであり、単体CLI書込みは禁止する。初回導入後は`flash.ps1 -BootloaderInstalled -ProbeSerial <SUB_STLINK_SN>`を使用し、applicationとmetadataを連続更新する。詳細は`doc/bootloader.md`を参照する。

現在のRelease生成makefileには、移動前のリンカスクリプト絶対パス
`C:\Users\hiroy\Documents\Orion_F303_sub\STM32F303CBTX_FLASH.ld` が残っている。
Releaseを使用する前にCubeIDEでRelease構成のmakefileを再生成し、現在の
`STM32F303CBTX_FLASH.ld` を参照していることを確認する。

## CubeMX再生成時の注意

アプリケーション処理の多くは `Core/Src/main.c` の `USER CODE` 範囲内にある。
CubeMXからコードを再生成した場合は、ビルドだけでなくCAN送受信、2 kHz周期、PWM、
UART DMA出力が維持されていることを確認する。
