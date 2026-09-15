# F303 sub アプリケーションブートローダー

## 目的

STM32内蔵System Memory bootloaderを使わず、基板固有の安全IOを維持したままCAN経由更新できる常駐bootloaderを置く。安全IO、metadata、CRC32C、通常アプリへのjump、OFW-CAN v2受信とFlash更新を実装している。

## OFW-CAN v2

- command ID: `0x610`
- data ID: `0x480`～`0x4FF`（下位7 bitをsequenceとして使用）
- Sub response ID: `0x654`、node ID: `4`
- 1 chunk: 最大896 byte（128 frame × 7 byte）
- payload byte 0: chunk token、byte 1～7: image data
- 896 byte固定bufferと128 bit bitmapで順不同格納し、重複frameは無視する
- bxCAN hardware FIFO0を空になるまでdrainし、32 frame software ring FIFOへ格納する
- hardware overrunまたはsoftware FIFO overflow時はchunkを破棄してsequence errorを返す
- chunk CRC32C一致後だけFlashへ書込み、全体CRC32C一致後だけmetadataをconfirmedにする
- 同一session、sizeの`BEGIN`と同一offsetの照会は冪等に処理する

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
  -ProbeSerial 002D00373033510635393935
```

backup確認後だけ`-Execute`を付ける。Device IDがF303xB/Cの`0x422`でなければ書込み前に中止する。

## 現在の検証結果

- bootloader: 1,344 byte、vector `0x08000000`
- application Debug: 63,672 byte、vector `0x08004000`
- image CRC32C: `0x54A88A33`
- metadata CRC32C: `0x101D8A68`
- CRC32C既知ベクトル、metadata構造、PowerShell構文検査に合格
- G474接続中にsub書込みを試みた場合、Device ID guardが書込み前に拒否することを確認
- 2026-08-25、接続先スワップ後のsub側ST-Linkは`002D00373033510635393935`、Device IDは`0x422`
- 128 KB FlashとOption Bytesを退避後、bootloader、application、metadataを個別にprogram/verifyして初回導入に成功
- 導入後のPCはapplication内、VTORは`0x08004000`、例外maskはすべて0
- ST-Link VCPのCOM167からUSART1 2 Mbpsログを取得し、battery値、CAN受信カウンタ、センサ値の連続更新を確認
- 通常更新スクリプトでmetadata無効化、application page個別消去、application/metadataのprogram/verify/resetに成功
- v2 bootloader: Flash 3,248 byte、RAM 3,376 byte、`-Werror` build成功
- CM4→Main→Subで65,168 byte、CRC32C `0xF692FBA9`の更新に成功
- 正常更新8.287～10.437秒、UART/CAN複合故障注入更新14.186秒
- CAN欠落はchunk再送、重複はbitmapで無視、逆順はsequence位置へ格納、payload破損はchunk CRCで検出して回復
- ST-Link readback 65,168 byteのCRC32Cが`0xF692FBA9`で送信imageと一致
- 更新後VTOR=`0x08004000`、COM167で2秒間2,646 byteのログとCAN受信カウンタ更新を確認

STM32CubeProgrammer 2.22.0ではF303の複数page範囲消去が失敗し、全消去直後の大容量programでもbit不一致が発生した。通常更新と初回導入の両方で、applicationサイズから使用pageを算出し、page番号を列挙した1回の消去後にapplicationをprogramする。列挙消去が失敗した場合のみ個別消去へfallbackする。128 KB統合BINはアドレス誤配置も確認されたため使用禁止とする。
