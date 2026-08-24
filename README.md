# Orion_F303_sub

STM32F303CBT6を使用したサブ基板用ファームウェアです。ドリブラーESC、サーボ、
ボールセンサ、バッテリ電圧、CAN通信、ESCテレメトリを扱います。

## 開発環境

- STM32CubeIDE 1.17.0
- STM32CubeProgrammer（書き込みスクリプトで使用）
- PowerShell 5.1以降

CubeIDEでプロジェクトをインポートしてビルドできます。コマンドラインでは、
類似プロジェクト `Orion_F303_BLDC` から移植したスクリプトを使用できます。

```powershell
# bootloaderと再配置済みアプリをビルド
powershell -ExecutionPolicy Bypass -File .\Script\build_bootloader.ps1 -Rebuild
powershell -ExecutionPolicy Bypass -File .\Script\build_application.ps1 -Configuration Debug -Rebuild

# 初回導入前のFlash/Option Bytes backup
powershell -ExecutionPolicy Bypass -File .\Script\install_sub_bootloader.ps1 -ProbeSerial <SUB_STLINK_SN>

# USART1（2 Mbps）のログを表示
powershell -ExecutionPolicy Bypass -File .\Script\monitor_uart.ps1 -Port COM3
```

ツールのインストール先が標準と異なる場合は、`build.ps1` の `-MakePath`、
`flash.ps1` の `-ProgrammerPath` で実行ファイルを指定してください。

ファームウェアの役割、通信ID、主要周期は [doc/overview.md](doc/overview.md) を参照してください。
