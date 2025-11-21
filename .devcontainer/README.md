# M5StickC Plus2 Rust 開発環境

このDev Container環境では、コンテナ内でビルドし、ホスト側でフラッシュする方式を採用しています。

## なぜこの方式？

- **デバイス名の違いを吸収**: `/dev/ttyUSB0`, `/dev/ttyACM0` など、環境によってデバイス名が異なる問題を回避
- **シンプルで確実**: Podman rootlessモードの制限を回避
- **柔軟性**: 各開発者が自分の環境に合わせて設定可能

## セットアップ手順

### 1. ホスト側の準備（初回のみ）

ホスト側（WSL/Linux）にRustとcargo-espflashをインストール：

```bash
# Rustのインストール
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
source ~/.cargo/env

# cargo-espflashのインストール
cargo install cargo-espflash
```

### 2. 開発ワークフロー

```bash
# コンテナ内でビルド
cargo build

# ホスト側（WSLターミナル）でフラッシュ
cd /path/to/M5Stick-Kun
cargo espflash flash --monitor
```

`cargo espflash` は自動的にESP32デバイスを検出します。

### 3. デバイスが検出されない場合

```bash
# デバイスを確認
ls /dev/tty*

# 手動でデバイスを指定
cargo espflash flash --port /dev/ttyUSB0 --monitor
```

## トラブルシューティング

### Permission denied エラー

ユーザーを`dialout`グループに追加：

```bash
sudo usermod -a -G dialout $USER
# ログアウト/ログインまたは再起動が必要
```

### WSL2でUSBデバイスが見えない

WSL2ではUSB passthrough が必要です：

1. Windows側に [usbipd-win](https://github.com/dorssel/usbipd-win) をインストール
2. PowerShell（管理者権限）で：
   ```powershell
   usbipd list
   usbipd bind --busid <BUSID>
   usbipd attach --wsl --busid <BUSID>
   ```

3. WSL側で確認：
   ```bash
   ls /dev/ttyUSB*
   ```

## 便利なコマンド

コンテナ内で以下のエイリアスが使えます：

- `build`: `cargo build` のエイリアス
- `monitor`: `cargo espflash monitor`（シリアルモニタのみ）

## 参考リンク

- [espflash](https://github.com/esp-rs/espflash)
- [esp-rs](https://github.com/esp-rs)
- [M5StickC Plus2 Docs](https://docs.m5stack.com/en/core/M5StickC%20PLUS2)
- [usbipd-win](https://github.com/dorssel/usbipd-win)
