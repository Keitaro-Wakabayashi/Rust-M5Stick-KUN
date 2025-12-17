#!/bin/bash
# M5StickC Plus2フラッシュスクリプト（espflash使用）

set -e

PORT="${1:-/dev/ttyUSB0}"
BINARY="target/xtensa-esp32-none-elf/release/m5stick-kun"

echo "🔍 バイナリ確認中..."
if [ ! -f "$BINARY" ]; then
    echo "❌ エラー: $BINARY が見つかりません"
    echo "先に 'cargo build --release' を実行してください"
    exit 1
fi

echo "📦 バイナリサイズ: $(du -h $BINARY | cut -f1)"

echo "🔌 デバイス確認中..."
if [ ! -e "$PORT" ]; then
    echo "❌ エラー: $PORT が見つかりません"
    echo "利用可能なポート:"
    ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo "  なし"
    exit 1
fi

echo "📤 フラッシュ中: $PORT"
echo "   (espflashが自動的にブートローダーを処理します)"

espflash flash \
    --port "$PORT" \
    --baud 460800 \
    "$BINARY"

echo ""
echo "✅ フラッシュ完了！"
echo ""
echo "📺 シリアルモニターを起動するには:"
echo "   espflash monitor --port $PORT"
echo "   または"
echo "   python3 -m serial.tools.miniterm $PORT 115200"
