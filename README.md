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

# bootloader/application/metadataを一括書き込み（Debug、接続probeを自動選択）
powershell -ExecutionPolicy Bypass -File .\Script\flash_all.ps1

# application/metadataのみをビルドして書き込み（Debug、接続probeを自動選択）
powershell -ExecutionPolicy Bypass -File .\Script\build_and_flash.ps1

# USART1（2 Mbps）のログを表示
powershell -ExecutionPolicy Bypass -File .\Script\monitor_uart.ps1 -Port COM3
```

ツールのインストール先が標準と異なる場合は、`build.ps1` の `-MakePath`、
`flash.ps1` の `-ProgrammerPath` で実行ファイルを指定してください。

ファームウェアの役割、通信ID、主要周期は [doc/overview.md](doc/overview.md) を参照してください。

## VS Codeでのコードブラウズ

このフォルダーをVS Codeで開き、MicrosoftのC/C++拡張機能
(`ms-vscode.cpptools`) を使用します。初回やビルド設定・ソースを変更した後は、
次のコマンド、または「タスクの実行」→ `Refresh C/C++ browse configuration`
を実行してください。

```powershell
powershell -ExecutionPolicy Bypass -File .\Script\setup_vscode.ps1
```

スクリプトはDebug/ReleaseとBootloaderのMakefileをdry-runし、ファイルごとの
includeパス、定義、ARM GCCの引数を解析用データベースに反映します。
ビルドや書き込みは行いません。生成JSONはマシン固有の絶対パスを含むためGit管理対象外です。
ARM GCCはPATHから検索します。別のインストール先は `-CompilerPath` と `-MakePath`
で指定できます。

このワークスペースではC/C++ IntelliSenseを有効にし、STM32 clangdを無効にしています。
`C/C++: Select a Configuration` で `STM32 Debug` / `STM32 Release` を選択できます。
どちらもBootloaderには専用のコンパイル条件を適用します。
設定後は必要に応じて `Developer: Reload Window` を実行し、解析完了を待ってください。
`F12` で定義へ移動、`Shift+F12` で参照検索、`Ctrl+Space` で補完候補を表示できます。
