# M5Stick-Kun 🤖

M5StickC Plus2 で動作する二輪倒立振子ロボットのファームウェア（Rust版）

このプロジェクトは `M5stick_plus_robo_v1.0.ino` (Arduino版) を Rust に移植したものです。

## 特徴

- ✅ **Rust** - 安全で高速な組み込み開発
- ✅ **ESP-IDF統合** - ESP32の豊富な機能を活用
- ✅ **Devcontainer** - 再現性の高い開発環境
- ✅ **Claude Code統合** - AIアシスト開発環境

## ハードウェア

- **ボード**: M5StickC Plus2
- **MCU**: ESP32-PICO
- **ディスプレイ**: ST7789V (135x240 SPI)
- **IMU**: MPU6886 (加速度・ジャイロセンサー)
- **PMIC**: AXP192 (電源管理)
- **モーター**: GPIO 0, 26 (PWMサーボ制御)
- **ボタン**: GPIO 37 (A), GPIO 39 (B)
- **LED**: GPIO 19

## 主要機能

### 実装済み
- [x] 基本セットアップ
- [x] LED制御 (Lチカ)

### 実装予定
- [ ] LCD初期化と描画 (顔の表情表示)
- [ ] IMU (MPU6886) 制御
- [ ] Kalmanフィルター
- [ ] PID倒立振子制御
- [ ] モーターPWM制御
- [ ] BLE通信 (RemoteXY)
- [ ] EEPROM (NVS) 読み書き
- [ ] バッテリー電圧監視

## 開発環境のセットアップ

### 必要なもの

- **Docker** - Devcontainerの実行に必要
- **VSCode** - 推奨エディタ（Remote Containers拡張が必要）
- **M5StickC Plus2** - 実機
- **USBケーブル** - フラッシュ用

### クイックスタート

1. **リポジトリをクローン**
   ```bash
   git clone <repository-url>
   cd M5Stick-Kun
   ```

2. **VSCodeでDevcontainerを起動**
   - VSCodeでプロジェクトフォルダを開く
   - コマンドパレット (`Ctrl+Shift+P` / `Cmd+Shift+P`) で
     `Dev Containers: Reopen in Container` を選択
   - 初回起動時はイメージのビルドに時間がかかります（10-20分程度）

3. **ビルドとフラッシュ**
   ```bash
   # ビルド
   cargo build --release

   # フラッシュ＆モニター
   cargo run --release
   ```

### 手動セットアップ（Devcontainer不使用の場合）

<details>
<summary>詳細を表示</summary>

```bash
# Rustのインストール
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

# espupのインストール
cargo install espup

# ESP32向けツールチェーンのセットアップ
espup install

# 環境変数の読み込み
source ~/export-esp.sh

# cargo-espflashのインストール
cargo install cargo-espflash

# ビルド＆フラッシュ
cargo run --release
```

</details>

## コマンドリファレンス

### 基本コマンド

```bash
# ビルド
cargo build
cargo build --release

# フラッシュ
cargo espflash flash

# シリアルモニター
cargo espflash monitor

# フラッシュ＆モニター（一発実行）
cargo run --release

# エイリアス（.cargo/config.toml で定義済み）
cargo flash
cargo monitor
cargo flash-and-monitor
```

### WSL2環境でのフラッシュ（推奨）

WSL2からM5StickC Plus2にフラッシュする場合、`esptool` を使用します。

**注:** このプロジェクトは `direct-boot` 形式を使用しているため、esptoolで直接アドレス0x0にフラッシュできます。

#### 初回セットアップ

1. **esptoolのインストール（WSL2内）**
   ```bash
   sudo apt install esptool python3-serial
   ```

2. **usbipd-winのインストール（Windows）**

   PowerShell（管理者権限）で実行：
   ```powershell
   winget install --interactive --exact dorssel.usbipd-win
   ```

3. **USBデバイスのバインド（初回のみ）**

   PowerShell（管理者権限）で実行：
   ```powershell
   # デバイス一覧を表示
   usbipd list

   # M5StickC Plus2をバインド（BUSIDは実際の値に置き換え）
   usbipd bind --busid 1-3
   ```

#### 毎回の使用手順

1. **ビルド（devcontainer内）**
   ```bash
   cargo build --release
   ```

2. **USBデバイスをWSL2にアタッチ（Windows PowerShell - 管理者）**
   ```powershell
   usbipd attach --wsl --busid 1-3
   ```

3. **フラッシュ（WSL2で実行）**

   **方法A: スクリプト使用（推奨）**
   ```bash
   ./flash.sh /dev/ttyACM0
   ```

   **方法B: 手動コマンド**
   ```bash
   esptool --chip esp32 \
     --port /dev/ttyACM0 \
     --baud 460800 \
     write_flash 0x0 \
     target/xtensa-esp32-none-elf/release/m5stick-kun
   ```

4. **シリアルモニター**
   ```bash
   python3 -m serial.tools.miniterm /dev/ttyACM0 115200
   ```

   終了: `Ctrl+]`

5. **使用後（オプション）**
   ```powershell
   # Windows PowerShell
   usbipd detach --busid 1-3
   ```

#### flash.sh スクリプトオプション

```bash
# デフォルトポート (/dev/ttyUSB0) でフラッシュ
./flash.sh

# カスタムポート指定
./flash.sh /dev/ttyACM0

# スクリプトの動作:
# 1. バイナリの存在確認
# 2. デバイスの接続確認
# 3. esptoolでアドレス0x0にフラッシュ実行（direct-boot形式）
# 4. 完了メッセージとシリアルモニター起動方法を表示
```

#### トラブルシューティング

**デバイスが見つからない場合:**
```bash
# WSL2内でデバイス確認
lsusb                  # USB デバイス一覧
ls -l /dev/ttyUSB*     # シリアルポート確認
dmesg | tail           # USB接続ログ確認
```

**権限エラーの場合:**
```bash
sudo chmod 666 /dev/ttyUSB0
# または
sudo usermod -a -G dialout $USER  # ログアウト/ログインが必要
```

**フラッシュが遅い場合:**
```bash
# ボーレートを上げる（デバイスが対応している場合）
esptool --chip esp32 --port /dev/ttyACM0 --baud 921600 write_flash 0x0 target/xtensa-esp32-none-elf/release/m5stick-kun
```

## Claude Code の使い方

このプロジェクトは Claude Code が統合されており、AIアシストで開発を進められます。

### CLI から使用

```bash
# Claude Code CLI を起動
claude-code

# 例: コード生成
claude "IMUのキャリブレーション関数を実装して"

# 例: コードレビュー
claude "このPID制御の実装をレビューして"
```

### VSCode 拡張から使用

1. VSCodeの左サイドバーから Claude Code アイコンをクリック
2. チャット欄で質問や指示を入力
3. Claude がコードを生成・編集・レビュー

## プロジェクト構成

```
M5Stick-Kun/
├── .devcontainer/          # Devcontainer設定
│   ├── Dockerfile          # 開発環境イメージ
│   └── devcontainer.json   # VSCode設定
├── .cargo/
│   └── config.toml         # Cargoビルド設定
├── src/
│   └── main.rs             # メインプログラム
├── Cargo.toml              # 依存関係定義
├── build.rs                # ビルドスクリプト
├── rust-toolchain.toml     # Rustバージョン指定
├── sdkconfig.defaults      # ESP-IDF設定
└── README.md               # このファイル
```

## トラブルシューティング

### Podman環境で起動できない

**症状:** `crun: chown: Operation not permitted` などの権限エラー

**解決策:** devcontainer.jsonをPodman互換に調整済みです。詳細は [Podman セットアップガイド](.devcontainer/PODMAN_SETUP.md) を参照してください。

```bash
# コンテナをリビルド
# VSCodeコマンドパレット (Ctrl+Shift+P) で:
# "Dev Containers: Rebuild Container" を実行
```

**推奨:** 問題が解決しない場合は、[Docker Desktop](https://www.docker.com/products/docker-desktop/) の使用を検討してください。

### デバイスが認識されない

```bash
# USB権限を確認
ls -l /dev/ttyUSB* /dev/ttyACM*

# 権限がない場合
sudo usermod -a -G dialout $USER
```

### ビルドエラー: `ldproxy not found`

```bash
# ldproxyを再インストール
cargo install ldproxy
```

### espflash がデバイスを検出できない

```bash
# espflash で明示的にポート指定
cargo espflash flash --port /dev/ttyUSB0
```

### Docker内でUSBデバイスが見えない

- Windows: USB/IP または WSL2 USB passthrough を設定
- Mac: Docker Desktop の設定で USB デバイスアクセスを有効化
- Linux: `--privileged` フラグ確認（devcontainer.json で設定済み）

## 参考資料

### 公式ドキュメント
- [Rust ESP Book](https://esp-rs.github.io/book/)
- [esp-idf-hal Documentation](https://docs.rs/esp-idf-hal/)
- [M5StickC Plus2 Docs](https://docs.m5stack.com/en/core/M5StickC%20PLUS2)

### 関連プロジェクト
- [esp-rs/esp-idf-hal](https://github.com/esp-rs/esp-idf-hal)
- [esp-rs/esp-idf-svc](https://github.com/esp-rs/esp-idf-svc)

## ライセンス

MIT OR Apache-2.0

## 貢献

Issue や Pull Request は大歓迎です！

---

**Made with ❤️ and 🦀 Rust**
