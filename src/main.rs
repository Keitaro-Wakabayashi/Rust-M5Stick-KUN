//! M5StickC Plus2 倒立振子ロボット - Rust版
//!
//! このプロジェクトは M5stick_plus_robo_v1.0.ino の Rust 移植版です
//!
//! ## ハードウェア構成
//! - MCU: ESP32-PICO-V3-02 (ESP32 dual-core Xtensa LX6)
//! - LCD: ST7789V (135x240 SPI)
//! - IMU: MPU6886 (I2C)
//! - 電源: TP4057 (充電IC), GPIO4 (HOLD - 電源保持)
//! - バッテリー: 200mAh, 電圧測定 GPIO38
//! - モーター: GPIO 0, 26 (PWM servo control)
//! - ボタン: GPIO 37 (A), GPIO 39 (B)
//! - LED: GPIO 19
//!
//! ## 注意事項
//! M5StickC Plus2にはAXP192が搭載されていません。
//! バッテリー駆動を維持するには、起動後にGPIO4 (HOLD)をHIGHに保つ必要があります。
//!
//! ## TODO: 移植予定の機能
//! 1. [x] 基本セットアップ
//! 2. [ ] LCD初期化と描画
//! 3. [ ] IMU (MPU6886) 初期化とキャリブレーション
//! 4. [ ] Kalmanフィルター実装
//! 5. [ ] PID制御
//! 6. [ ] モーターPWM制御
//! 7. [ ] BLE (RemoteXY) 通信
//! 8. [ ] EEPROM (NVS) 読み書き
//! 9. [ ] バッテリー電圧監視

#![no_std]
#![no_main]

extern crate alloc;

use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    delay::Delay,
    gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull},
    i2c::master::{Config as I2cConfig, I2c},
    time::Rate,
};
use log::*;

// ESP-IDF App Descriptor (espflashが必要とする)
esp_bootloader_esp_idf::esp_app_desc!();

// ハードウェアピン定義
// const MOTOR_PIN_L: u8 = 0;
// const MOTOR_PIN_R: u8 = 26;
// const BTN_A: u8 = 37;
// const BTN_B: u8 = 39;
// const M5_LED: u8 = 19;

// LCD SPI ピン (ST7789V)
// const LCD_CS: u8 = 5;
// const LCD_DC: u8 = 14;
// const LCD_RST: u8 = 12;
// const LCD_SCLK: u8 = 13;
// const LCD_MOSI: u8 = 15;

// I2C ピン (IMU & PMIC)
// const I2C_SDA: u8 = 21;
// const I2C_SCL: u8 = 22;

/// ロボットの状態
#[derive(Debug, Clone, Copy, PartialEq)]
enum RobotState {
    /// キャリブレーション準備中
    CalibrationReady,
    /// キャリブレーション実行中
    Calibrating,
    /// 動作姿勢への移行待ち
    WaitingForUpright,
    /// 垂直姿勢で待機中（-30°～+30°）
    Ready,
    /// バランス制御実行中
    Balancing,
    /// エラー（範囲外）
    Error,
}

#[esp_hal::main]
fn main() -> ! {
    // ロガー初期化
    esp_println::logger::init_logger_from_env();

    // ペリフェラル取得
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    // ヒープアロケーター初期化（ESP-IDFブートローダーのreclaimedメモリを使用）
    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 98768);

    info!("M5StickC Plus2 倒立振子 起動中...");

    // Delay初期化
    let mut delay = Delay::new();

    // 電源HOLD (GPIO 4 - M5StickC Plus2で必須！)
    // Plus2にはAXP192がなく、起動後にGPIO4をHIGHに保持しないと電源が切れる
    let mut power_hold = Output::new(peripherals.GPIO4, Level::High, OutputConfig::default());
    power_hold.set_high(); // 電源保持
    info!("電源HOLD有効 (GPIO4)");

    // LED初期化 (GPIO 19 - M5StickC Plus2では反転ロジック)
    let mut led = Output::new(peripherals.GPIO19, Level::Low, OutputConfig::default());
    led.set_high(); // 起動中はLED点灯

    // LCDバックライト初期化 (GPIO 27)
    let mut lcd_backlight = Output::new(peripherals.GPIO27, Level::High, OutputConfig::default());
    lcd_backlight.set_high(); // バックライト点灯

    let btn_a = Input::new(
        peripherals.GPIO37,
        InputConfig::default().with_pull(Pull::Up),
    );

    info!("GPIO初期化完了");

    // SPI初期化 (SCK: GPIO13, MOSI: GPIO15, CS: GPIO5 for LCD)
    info!("SPI初期化中...");
    use embedded_hal_bus::spi::ExclusiveDevice;
    use esp_hal::spi::master::{Config as SpiConfig, Spi};

    let spi_config = SpiConfig::default().with_frequency(Rate::from_khz(40_000));
    let spi = Spi::new(peripherals.SPI2, spi_config)
        .expect("SPI init failed")
        .with_sck(peripherals.GPIO13)
        .with_mosi(peripherals.GPIO15);

    let spi_cs = Output::new(peripherals.GPIO5, Level::High, OutputConfig::default());
    let spi_device =
        ExclusiveDevice::new(spi, spi_cs, Delay::new()).expect("SPI device creation failed");

    // LCD制御ピン (DC: GPIO14, RST: GPIO12)
    let lcd_dc = Output::new(peripherals.GPIO14, Level::Low, OutputConfig::default());
    let lcd_rst = Output::new(peripherals.GPIO12, Level::Low, OutputConfig::default());

    // LCD初期化
    info!("LCD初期化中...");
    let mut display = display::init_display(spi_device, lcd_dc, lcd_rst, &mut delay);
    info!("LCD初期化完了");

    // I2C初期化 (SDA: GPIO21, SCL: GPIO22, 100kHz)
    info!("I2C初期化中...");
    let i2c_config = I2cConfig::default().with_frequency(Rate::from_khz(100));
    let i2c = I2c::new(peripherals.I2C0, i2c_config)
        .expect("I2C init failed")
        .with_sda(peripherals.GPIO21)
        .with_scl(peripherals.GPIO22);

    // MPU6886初期化
    info!("MPU6886初期化中...");
    let mut imu = imu::Mpu6886::new(i2c);

    match imu.init() {
        Ok(_) => info!("MPU6886初期化成功"),
        Err(_) => {
            error!("MPU6886初期化失敗!");
            loop {
                led.toggle();
                delay.delay_millis(100);
            }
        }
    }

    // モーターピンを先に初期化（ループ外で一度だけ）
    let motor_pin_l = Output::new(peripherals.GPIO0, Level::Low, OutputConfig::default());
    let motor_pin_r = Output::new(peripherals.GPIO26, Level::Low, OutputConfig::default());

    // 状態マシン初期化
    let mut state = RobotState::CalibrationReady;
    let mut wait_count: u16 = 0;
    let mut last_display_update: u32 = 0;
    const DT: f32 = 0.01; // 10ms = 0.01秒

    // ローパスフィルター用変数（Arduino版: fil_N = 5）
    let mut pitch_filter: f32 = 0.0;
    const FIL_N: f32 = 5.0;

    // ループ周期測定用（簡易版：ミリ秒カウンター）
    let mut loop_count: u32 = 0;
    let mut last_time_ms: u32 = 0;
    let mut total_time_ms: u32 = 0;

    // Kalman, PID, Motorを後で初期化
    let mut kalman_opt: Option<kalman::KalmanFilter> = None;
    let mut pid_opt: Option<pid::PidController> = None;
    let mut motor_opt: Option<motor::MotorController> =
        Some(motor::MotorController::new(motor_pin_l, motor_pin_r));

    let mut motor_enable = false;

    info!("セットアップ完了! 状態マシン開始");

    // メインループ: 状態マシンベース
    loop {
        if btn_a.is_low() {
            delay.delay_millis(50);
            if btn_a.is_low() {
                motor_enable = !motor_enable;
                info!("Moter: {}", if motor_enable { "ON" } else { "OFF" });
            }

            while btn_a.is_low() {
                delay.delay_millis(10);
            }
        }
        match state {
            // ==================== キャリブレーション準備 ====================
            RobotState::CalibrationReady => {
                info!("状態: CalibrationReady");

                // LCD表示
                if let Err(_) = display::draw_calibration_ready(&mut display) {
                    error!("LCD描画エラー");
                }

                led.set_high(); // LED点灯
                delay.delay_millis(3000); // 3秒待機

                // 次の状態へ
                state = RobotState::Calibrating;
            }

            // ==================== キャリブレーション実行 ====================
            RobotState::Calibrating => {
                info!("状態: Calibrating");

                // LCD表示
                if let Err(_) = display::draw_calibrating(&mut display) {
                    error!("LCD描画エラー");
                }

                // キャリブレーション実行
                match imu.calibrate(&mut delay) {
                    Ok(_) => {
                        info!("キャリブレーション完了");
                        if let (Ok(accel), Ok(gyro)) =
                            (imu.read_accel_calibrated(), imu.read_gyro_calibrated())
                        {
                            info!(
                                "  Accel: X={:.3}g Y={:.3}g Z={:.3}g",
                                accel[0], accel[1], accel[2]
                            );
                            info!(
                                "  Gyro: X={:.1}°/s Y={:.1}°/s Z={:.1}°/s",
                                gyro[0], gyro[1], gyro[2]
                            );
                        }
                        led.set_low(); // LED消灯
                        state = RobotState::WaitingForUpright;
                    }
                    Err(_) => {
                        error!("キャリブレーション失敗!");
                        loop {
                            led.toggle();
                            delay.delay_millis(100);
                        }
                    }
                }
            }

            // ==================== 動作姿勢への移行待ち ====================
            RobotState::WaitingForUpright => {
                info!("状態: WaitingForUpright");

                // LCD表示（Stand Up!）
                if let Err(_) = display::draw_stand_up(&mut display) {
                    error!("LCD描画エラー");
                }

                // LED短い点滅（3回）
                for _ in 0..3 {
                    led.set_high();
                    delay.delay_millis(200);
                    led.set_low();
                    delay.delay_millis(200);
                }

                // 5秒待機（ユーザーが姿勢を変更）
                delay.delay_millis(5000);

                // Kalman, PID初期化
                let mut kalman = kalman::KalmanFilter::new();
                if let Ok(pitch) = imu.get_pitch() {
                    kalman.set_angle(pitch);
                    info!("Kalman初期化: angle = {:.2}°", pitch);
                }
                kalman_opt = Some(kalman);

                let pid = pid::PidController::new();
                info!(
                    "PID初期化: kp={:.2}, ki={:.2}, kd={:.2}",
                    pid.kp, pid.ki, pid.kd
                );
                pid_opt = Some(pid);

                wait_count = 0;
                state = RobotState::Ready;
            }

            // ==================== Ready（垂直姿勢待機） ====================
            RobotState::Ready => {
                // センサー読み取り
                if let (Ok(_accel), Ok(gyro), Ok(pitch_raw)) = (
                    imu.read_accel_calibrated(),
                    imu.read_gyro_calibrated(),
                    imu.get_pitch(),
                ) {
                    // Kalmanフィルター更新
                    let kalman = kalman_opt.as_mut().unwrap();
                    let pitch_kalman = kalman.update(pitch_raw, gyro[0], DT);

                    // Arduino版と同じオフセット補正（Pitch_offset = 81）
                    // 安定姿勢で81°付近になるように調整
                    const PITCH_OFFSET: f32 = 74.0;
                    let pitch_corrected = pitch_kalman + PITCH_OFFSET;

                    // ローパスフィルター（無効化: 遅延により制御が不安定になるため）
                    // pitch_filter = (pitch_corrected + pitch_filter * (FIL_N - 1.0)) / FIL_N;
                    pitch_filter = pitch_corrected;

                    // 角度範囲チェック（目標角度81°を中心に±30°の範囲）
                    // 81° - 30° = 51°, 81° + 30° = 111°
                    if pitch_filter >= 51.0 && pitch_filter <= 111.0 {
                        wait_count += 1;

                        // LCD更新（100msごと = 10カウントごと）
                        if wait_count % 10 == 0 {
                            if let Err(_) =
                                display::draw_ready(&mut display, wait_count, motor_enable)
                            {
                                error!("LCD描画エラー");
                            }
                            info!(
                                "Ready: wait_count={}, pitch={:.2}°",
                                wait_count, pitch_filter
                            );
                        }

                        // 2秒間安定（200カウント）したら動作開始
                        if wait_count >= 200 {
                            info!("垂直姿勢安定！バランス制御開始");
                            if motor_enable {
                                motor_opt.as_mut().unwrap().set_enabled(true);
                                state = RobotState::Balancing;
                            }
                        }
                    } else {
                        // 範囲外 → カウントリセット
                        wait_count = 0;
                        info!("角度範囲外: {:.2}°", pitch_filter);
                    }
                } else {
                    error!("センサー読み取りエラー");
                }

                delay.delay_millis(10);
            }

            // ==================== Balancing（制御実行中） ====================
            RobotState::Balancing => {
                // 簡素化: 10ms周期で全処理を実行
                // Arduino版はセンサー読み取りが高速ループだが、
                // Rust版はシンプルに10ms周期で統一

                // センサー読み取り
                if let (Ok(_accel), Ok(gyro), Ok(pitch_raw)) = (
                    imu.read_accel_calibrated(),
                    imu.read_gyro_calibrated(),
                    imu.get_pitch(),
                ) {
                    // Kalmanフィルター更新
                    let kalman = kalman_opt.as_mut().unwrap();
                    let pitch_kalman = kalman.update(pitch_raw, gyro[0], DT);

                    // Arduino版と同じオフセット補正（Pitch_offset = 81）
                    // 安定姿勢（pitch_kalman ≈ 7°）で pitch ≈ 81° になるように
                    const PITCH_OFFSET: f32 = 74.0; // 81 - 7 = 74
                    let pitch_corrected = pitch_kalman + PITCH_OFFSET;

                    // ローパスフィルター（無効化: 遅延により制御が不安定になるため）
                    // pitch_filter = (pitch_corrected + pitch_filter * (FIL_N - 1.0)) / FIL_N;
                    pitch_filter = pitch_corrected;

                    // 角度範囲チェック（目標角度81°から±50°程度の範囲）
                    // 31° ～ 131° くらいまで許容
                    if pitch_filter < 30.0 || pitch_filter > 130.0 {
                        // 範囲外 → エラー状態へ
                        error!("範囲外に倒れた: {:.2}°", pitch_filter);
                        motor_opt.as_mut().unwrap().set_enabled(false);
                        pid_opt.as_mut().unwrap().reset();
                        state = RobotState::Error;
                        continue;
                    }

                    // Arduino版と同じ：生のジャイロ値を使用
                    let d_angle = gyro[0];

                    // PID制御（目標角度を基準とした偏差）
                    // 目標角度を大きくすると後傾姿勢になる
                    const TARGET_ANGLE: f32 = 83.0; // 81°より2°後傾
                    let pid = pid_opt.as_mut().unwrap();
                    let angle_from_target = pitch_filter - TARGET_ANGLE;
                    let power = pid.update(angle_from_target, d_angle, 0.0);

                    // デバッグ: Power値を出力（10回に1回）
                    loop_count += 1;
                    if loop_count % 10 == 0 {
                        info!("Pitch={:.2}° Power={:.0}", pitch_filter, power);
                    }

                    // モーター駆動（Arduino版: powerL = -power, powerR = power）
                    let motor = motor_opt.as_mut().unwrap();
                    motor.drive(-power, power);

                    // PWMパルス生成（高速・20ms待機なし）
                    motor.pulse_drive(&mut delay);

                    // LCD更新（100msごと）
                    last_display_update += 1;
                    if last_display_update >= 10 {
                        if let Err(_) =
                            display::draw_balancing(&mut display, pitch_filter, power as i16)
                        {
                            error!("LCD描画エラー");
                        }
                        last_display_update = 0;
                    }

                    // 10ms待機（Arduino版のms10 += 10と同等）
                    delay.delay_millis(10);
                } else {
                    error!("センサー読み取りエラー");
                    delay.delay_millis(10);
                }
            }

            // ==================== Error（範囲外） ====================
            RobotState::Error => {
                // LCD表示
                if let Err(_) = display::draw_error(&mut display) {
                    error!("LCD描画エラー");
                }

                // LED高速点滅
                for _ in 0..10 {
                    led.toggle();
                    delay.delay_millis(100);
                }

                // Ready状態に戻る
                wait_count = 0;
                state = RobotState::Ready;
            }
        }
    }
}

// ============================================================================
// 以下、今後実装予定のモジュール構成
// ============================================================================

/// LCD制御モジュール (ST7789V)
mod display {
    use display_interface_spi::SPIInterface;
    use embedded_graphics::{
        mono_font::{ascii::FONT_6X10, MonoTextStyle},
        pixelcolor::Rgb565,
        prelude::*,
        primitives::{Ellipse, Line, PrimitiveStyle, Rectangle},
        text::Text,
    };
    use embedded_hal_bus::spi::ExclusiveDevice;
    use esp_hal::{delay::Delay, gpio::Output, spi::master::Spi};
    use mipidsi::{models::ST7789, options::ColorInversion, Builder};

    // 顔の定数
    const FACE_CENTER: i32 = 65;

    pub type SpiDeviceType =
        ExclusiveDevice<Spi<'static, esp_hal::Blocking>, Output<'static>, Delay>;

    pub type DisplayType =
        mipidsi::Display<SPIInterface<SpiDeviceType, Output<'static>>, ST7789, Output<'static>>;

    /// M5StickC Plus2のLCD初期化
    ///
    /// # ハードウェア
    /// - LCD: ST7789V (135x240)
    /// - SPI: GPIO 13 (SCK), GPIO 15 (MOSI), GPIO 5 (CS)
    /// - DC: GPIO 14
    /// - RST: GPIO 12
    pub fn init_display(
        spi_device: SpiDeviceType,
        dc: Output<'static>,
        rst: Output<'static>,
        delay: &mut Delay,
    ) -> DisplayType {
        // SPIインターフェース作成
        let di = SPIInterface::new(spi_device, dc);

        // ST7789ディスプレイを初期化
        // M5StickC Plus2: 135x240, Portrait mode
        let display = Builder::new(ST7789, di)
            .display_size(135, 240)
            .display_offset(52, 40) // M5StickC Plus2のオフセット
            .color_order(mipidsi::options::ColorOrder::Rgb)
            .invert_colors(ColorInversion::Inverted)
            .reset_pin(rst)
            .init(delay)
            .expect("LCD init failed");

        display
    }

    /// キャリブレーション準備画面
    pub fn draw_calibration_ready(display: &mut DisplayType) -> Result<(), ()> {
        use embedded_graphics::mono_font::ascii::FONT_9X18_BOLD;

        // 画面クリア
        display.clear(Rgb565::BLACK).map_err(|_| ())?;

        // テキストスタイル（白文字、大きめフォント）
        let style_bold = MonoTextStyle::new(&FONT_9X18_BOLD, Rgb565::WHITE);
        let style = MonoTextStyle::new(&FONT_6X10, Rgb565::WHITE);

        // タイトル
        Text::new("Calibrating", Point::new(20, 25), style_bold)
            .draw(display)
            .map_err(|_| ())?;

        // 指示
        Text::new("Place device:", Point::new(5, 60), style)
            .draw(display)
            .map_err(|_| ())?;
        Text::new("- Screen UP", Point::new(5, 75), style)
            .draw(display)
            .map_err(|_| ())?;
        Text::new("- Horizontal", Point::new(5, 90), style)
            .draw(display)
            .map_err(|_| ())?;

        Text::new("Starting in 3s", Point::new(15, 120), style)
            .draw(display)
            .map_err(|_| ())?;

        Ok(())
    }

    /// キャリブレーション実行中画面
    pub fn draw_calibrating(display: &mut DisplayType) -> Result<(), ()> {
        use embedded_graphics::mono_font::ascii::FONT_9X18_BOLD;

        // 画面クリア
        display.clear(Rgb565::BLACK).map_err(|_| ())?;

        // テキストスタイル
        let style_bold = MonoTextStyle::new(&FONT_9X18_BOLD, Rgb565::WHITE);
        let style = MonoTextStyle::new(&FONT_6X10, Rgb565::YELLOW);

        Text::new("Calibrating", Point::new(20, 60), style_bold)
            .draw(display)
            .map_err(|_| ())?;

        Text::new("Do not move!", Point::new(20, 100), style)
            .draw(display)
            .map_err(|_| ())?;

        Ok(())
    }

    /// 姿勢変更待機画面
    pub fn draw_stand_up(display: &mut DisplayType) -> Result<(), ()> {
        use embedded_graphics::mono_font::ascii::FONT_9X18_BOLD;

        // 画面クリア
        display.clear(Rgb565::BLACK).map_err(|_| ())?;

        // テキストスタイル
        let style_bold = MonoTextStyle::new(&FONT_9X18_BOLD, Rgb565::GREEN);
        let style = MonoTextStyle::new(&FONT_6X10, Rgb565::WHITE);

        Text::new("Stand Up!", Point::new(25, 50), style_bold)
            .draw(display)
            .map_err(|_| ())?;

        Text::new("Position:", Point::new(5, 90), style)
            .draw(display)
            .map_err(|_| ())?;
        Text::new("- Screen FRONT", Point::new(5, 105), style)
            .draw(display)
            .map_err(|_| ())?;
        Text::new("- USB UP", Point::new(5, 120), style)
            .draw(display)
            .map_err(|_| ())?;
        Text::new("- Vertical", Point::new(5, 135), style)
            .draw(display)
            .map_err(|_| ())?;

        Ok(())
    }

    /// Ready画面（角度範囲内で待機中）
    pub fn draw_ready(
        display: &mut DisplayType,
        wait_count: u16,
        motor_enable: bool,
    ) -> Result<(), ()> {
        use core::fmt::Write;
        use embedded_graphics::mono_font::ascii::FONT_9X18_BOLD;
        use heapless::String;

        // 画面クリア
        display.clear(Rgb565::BLACK).map_err(|_| ())?;

        // テキストスタイル
        let style_bold = MonoTextStyle::new(&FONT_9X18_BOLD, Rgb565::CYAN);
        let style = MonoTextStyle::new(&FONT_6X10, Rgb565::WHITE);

        Text::new("Ready...", Point::new(30, 60), style_bold)
            .draw(display)
            .map_err(|_| ())?;

        // カウントダウン表示
        let mut buf: String<32> = String::new();
        write!(&mut buf, "Wait: {:.1}s", (200 - wait_count) as f32 / 100.0).ok();
        Text::new(&buf, Point::new(30, 100), style)
            .draw(display)
            .map_err(|_| ())?;

        // ボタン状態表示
        let mut buf: String<32> = String::new();
        write!(
            &mut buf,
            "Motor: {}.",
            if motor_enable { "ON" } else { "OFF" }
        )
        .ok();
        Text::new(&buf, Point::new(30, 140), style_bold)
            .draw(display)
            .map_err(|_| ())?;

        Ok(())
    }

    /// Balancing画面（制御実行中）
    pub fn draw_balancing(display: &mut DisplayType, pitch: f32, power: i16) -> Result<(), ()> {
        use core::fmt::Write;
        use embedded_graphics::mono_font::ascii::FONT_9X18_BOLD;
        use heapless::String;

        // 画面クリア
        display.clear(Rgb565::BLACK).map_err(|_| ())?;

        // テキストスタイル
        let style_bold = MonoTextStyle::new(&FONT_9X18_BOLD, Rgb565::GREEN);
        let style = MonoTextStyle::new(&FONT_6X10, Rgb565::WHITE);

        Text::new("Balancing", Point::new(20, 30), style_bold)
            .draw(display)
            .map_err(|_| ())?;

        // Pitch角度（大きく表示）
        let mut buf: String<32> = String::new();
        write!(&mut buf, "Pitch: {:.1}", pitch).ok();
        Text::new(&buf, Point::new(10, 80), style)
            .draw(display)
            .map_err(|_| ())?;

        // Power値
        buf.clear();
        write!(&mut buf, "Power: {}", power).ok();
        Text::new(&buf, Point::new(10, 110), style)
            .draw(display)
            .map_err(|_| ())?;

        draw_stick_kun(display);

        Ok(())
    }

    /// Error画面（範囲外に倒れた）
    pub fn draw_error(display: &mut DisplayType) -> Result<(), ()> {
        use embedded_graphics::mono_font::ascii::FONT_9X18_BOLD;

        // 画面クリア
        display.clear(Rgb565::BLACK).map_err(|_| ())?;

        // テキストスタイル
        let style_bold = MonoTextStyle::new(&FONT_9X18_BOLD, Rgb565::RED);
        let style = MonoTextStyle::new(&FONT_6X10, Rgb565::WHITE);

        Text::new("ERROR!", Point::new(35, 60), style_bold)
            .draw(display)
            .map_err(|_| ())?;

        Text::new("Out of range", Point::new(15, 100), style)
            .draw(display)
            .map_err(|_| ())?;

        Ok(())
    }

    /// IMUデータをLCDに表示
    pub fn draw_imu_data(
        display: &mut DisplayType,
        accel: [f32; 3],
        gyro: [f32; 3],
        pitch: f32,
    ) -> Result<(), ()> {
        use core::fmt::Write;
        use heapless::String;

        // 画面クリア
        display.clear(Rgb565::BLACK).map_err(|_| ())?;

        // テキストスタイル（白文字）
        let style = MonoTextStyle::new(&FONT_6X10, Rgb565::WHITE);

        // タイトル
        Text::new("M5Stick-Kun IMU", Point::new(10, 15), style)
            .draw(display)
            .map_err(|_| ())?;

        // 加速度表示
        let mut buf: String<64> = String::new();
        write!(&mut buf, "Accel X:{:.2}g", accel[0]).ok();
        Text::new(&buf, Point::new(5, 35), style)
            .draw(display)
            .map_err(|_| ())?;

        buf.clear();
        write!(&mut buf, "Accel Y:{:.2}g", accel[1]).ok();
        Text::new(&buf, Point::new(5, 50), style)
            .draw(display)
            .map_err(|_| ())?;

        buf.clear();
        write!(&mut buf, "Accel Z:{:.2}g", accel[2]).ok();
        Text::new(&buf, Point::new(5, 65), style)
            .draw(display)
            .map_err(|_| ())?;

        // ジャイロ表示
        buf.clear();
        write!(&mut buf, "Gyro X:{:.1}", gyro[0]).ok();
        Text::new(&buf, Point::new(5, 85), style)
            .draw(display)
            .map_err(|_| ())?;

        buf.clear();
        write!(&mut buf, "Gyro Y:{:.1}", gyro[1]).ok();
        Text::new(&buf, Point::new(5, 100), style)
            .draw(display)
            .map_err(|_| ())?;

        buf.clear();
        write!(&mut buf, "Gyro Z:{:.1}", gyro[2]).ok();
        Text::new(&buf, Point::new(5, 115), style)
            .draw(display)
            .map_err(|_| ())?;

        // Pitch角度表示（大きく）
        buf.clear();
        write!(&mut buf, "Pitch:{:.1}deg", pitch).ok();
        Text::new(&buf, Point::new(5, 130), style)
            .draw(display)
            .map_err(|_| ())?;

        Ok(())
    }

    // === 顔描画関数 ===

    /// 初期顔描画
    pub fn draw_stick_kun(display: &mut DisplayType) -> Result<(), ()> {
        display.clear(Rgb565::RED).map_err(|_| ())?;
        Rectangle::new(Point::new(0, 105), Size::new(134, 134))
            .into_styled(PrimitiveStyle::with_fill(Rgb565::WHITE))
            .draw(display)
            .map_err(|_| ())?;
        Ellipse::new(Point::new(FACE_CENTER - 27, 160), Size::new(10, 20))
            .into_styled(PrimitiveStyle::with_fill(Rgb565::BLACK))
            .draw(display)
            .map_err(|_| ())?;
        Ellipse::new(Point::new(FACE_CENTER + 27, 160), Size::new(10, 20))
            .into_styled(PrimitiveStyle::with_fill(Rgb565::BLACK))
            .draw(display)
            .map_err(|_| ())?;

        for i in 0..5 {
            Line::new(
                Point::new(FACE_CENTER - 15, 200 + i),
                Point::new(FACE_CENTER - 10, 204 + i),
            )
            .into_styled(PrimitiveStyle::with_stroke(Rgb565::BLACK, 1))
            .draw(display)
            .map_err(|_| ())?;

            Line::new(
                Point::new(FACE_CENTER - 10, 204 + i),
                Point::new(FACE_CENTER, 206 + i),
            )
            .into_styled(PrimitiveStyle::with_stroke(Rgb565::BLACK, 1))
            .draw(display)
            .map_err(|_| ())?;

            Line::new(
                Point::new(FACE_CENTER, 206 + i),
                Point::new(FACE_CENTER + 10, 204 + i),
            )
            .into_styled(PrimitiveStyle::with_stroke(Rgb565::BLACK, 1))
            .draw(display)
            .map_err(|_| ())?;

            Line::new(
                Point::new(FACE_CENTER + 10, 204 + i),
                Point::new(FACE_CENTER + 15, 200 + i),
            )
            .into_styled(PrimitiveStyle::with_stroke(Rgb565::BLACK, 1))
            .draw(display)
            .map_err(|_| ())?;
        }

        for i in 0..5 {
            Line::new(
                Point::new(FACE_CENTER - 25, 130 + i),
                Point::new(FACE_CENTER - 45, 125 + i),
            )
            .into_styled(PrimitiveStyle::with_stroke(Rgb565::BLACK, 1))
            .draw(display)
            .map_err(|_| ())?;
            Line::new(
                Point::new(FACE_CENTER + 25, 130 + i),
                Point::new(FACE_CENTER + 45, 125 + i),
            )
            .into_styled(PrimitiveStyle::with_stroke(Rgb565::BLACK, 1))
            .draw(display)
            .map_err(|_| ())?;
        }
        Ok(())
    }
}

/// IMU制御モジュール (MPU6886)
#[allow(dead_code)]
mod imu {
    use embedded_hal::i2c::I2c;

    // MPU6886 レジスタアドレス
    const MPU6886_ADDR: u8 = 0x68;
    const WHO_AM_I: u8 = 0x75;
    const PWR_MGMT_1: u8 = 0x6B;
    const GYRO_CONFIG: u8 = 0x1B;
    const ACCEL_CONFIG: u8 = 0x1C;
    const ACCEL_XOUT_H: u8 = 0x3B;
    const GYRO_XOUT_H: u8 = 0x43;

    // レンジ設定
    const GYRO_RANGE_250DPS: u8 = 0x00;
    const ACCEL_RANGE_2G: u8 = 0x00;

    // スケール係数
    const GYRO_SCALE_250DPS: f32 = 131.0; // LSB/(deg/s)
    const ACCEL_SCALE_2G: f32 = 16384.0; // LSB/g
    const RAD_TO_DEG: f32 = 57.295779513082320876798154814105; // 180/π

    pub struct Mpu6886<I2C> {
        i2c: I2C,
        gyro_offset: [f32; 3],
        accel_offset: [f32; 3],
    }

    impl<I2C, E> Mpu6886<I2C>
    where
        I2C: I2c<Error = E>,
    {
        pub fn new(i2c: I2C) -> Self {
            Self {
                i2c,
                gyro_offset: [0.0; 3],
                accel_offset: [0.0; 3],
            }
        }

        /// レジスタから1バイト読み取り
        fn read_register(&mut self, reg: u8) -> Result<u8, E> {
            let mut buf = [0u8];
            self.i2c.write_read(MPU6886_ADDR, &[reg], &mut buf)?;
            Ok(buf[0])
        }

        /// レジスタに1バイト書き込み
        fn write_register(&mut self, reg: u8, value: u8) -> Result<(), E> {
            self.i2c.write(MPU6886_ADDR, &[reg, value])
        }

        /// レジスタから複数バイト読み取り
        fn read_bytes(&mut self, reg: u8, buffer: &mut [u8]) -> Result<(), E> {
            self.i2c.write_read(MPU6886_ADDR, &[reg], buffer)
        }

        /// MPU6886を初期化
        pub fn init(&mut self) -> Result<(), E> {
            // デバイスIDを確認
            let who_am_i = self.read_register(WHO_AM_I)?;
            if who_am_i != 0x19 {
                // MPU6886のデバイスIDは0x19
                log::warn!(
                    "MPU6886 WHO_AM_I check failed: 0x{:02x} (expected 0x19)",
                    who_am_i
                );
            }

            // リセットしてデバイスをウェイクアップ
            self.write_register(PWR_MGMT_1, 0x00)?;

            // ジャイロレンジ: ±250 deg/s
            self.write_register(GYRO_CONFIG, GYRO_RANGE_250DPS)?;

            // 加速度レンジ: ±2g
            self.write_register(ACCEL_CONFIG, ACCEL_RANGE_2G)?;

            Ok(())
        }

        /// 加速度データを読み取り (x, y, z) 単位: g
        pub fn read_accel(&mut self) -> Result<[f32; 3], E> {
            let mut buf = [0u8; 6];
            self.read_bytes(ACCEL_XOUT_H, &mut buf)?;

            let ax = i16::from_be_bytes([buf[0], buf[1]]) as f32 / ACCEL_SCALE_2G;
            let ay = i16::from_be_bytes([buf[2], buf[3]]) as f32 / ACCEL_SCALE_2G;
            let az = i16::from_be_bytes([buf[4], buf[5]]) as f32 / ACCEL_SCALE_2G;

            Ok([ax, ay, az])
        }

        /// ジャイロデータを読み取り (x, y, z) 単位: deg/s
        pub fn read_gyro(&mut self) -> Result<[f32; 3], E> {
            let mut buf = [0u8; 6];
            self.read_bytes(GYRO_XOUT_H, &mut buf)?;

            let gx = i16::from_be_bytes([buf[0], buf[1]]) as f32 / GYRO_SCALE_250DPS;
            let gy = i16::from_be_bytes([buf[2], buf[3]]) as f32 / GYRO_SCALE_250DPS;
            let gz = i16::from_be_bytes([buf[4], buf[5]]) as f32 / GYRO_SCALE_250DPS;

            Ok([gx, gy, gz])
        }

        /// キャリブレーション（500サンプル平均）
        pub fn calibrate<D: embedded_hal::delay::DelayNs>(
            &mut self,
            delay: &mut D,
        ) -> Result<(), E> {
            let mut gyro_sum = [0.0f32; 3];
            let mut accel_sum = [0.0f32; 3];

            for _ in 0..500 {
                let gyro = self.read_gyro()?;
                let accel = self.read_accel()?;

                gyro_sum[0] += gyro[0];
                gyro_sum[1] += gyro[1];
                gyro_sum[2] += gyro[2];

                accel_sum[0] += accel[0];
                accel_sum[1] += accel[1];
                accel_sum[2] += accel[2];

                delay.delay_ms(2);
            }

            // ジャイロオフセット: ゼロ点補正
            self.gyro_offset[0] = gyro_sum[0] / 500.0;
            self.gyro_offset[1] = gyro_sum[1] / 500.0;
            self.gyro_offset[2] = gyro_sum[2] / 500.0;

            // 加速度オフセット（Arduino版と同じロジック）
            // キャリブレーション姿勢: 画面が上、水平 → Z軸が重力方向（-1g）
            // X軸とY軸: ゼロ点補正
            // Z軸: 重力(1g)を引いてゼロ点にする
            self.accel_offset[0] = accel_sum[0] / 500.0;
            self.accel_offset[1] = accel_sum[1] / 500.0;
            self.accel_offset[2] = accel_sum[2] / 500.0 - 1.0; // Z軸: 測定値-1gをオフセットに

            Ok(())
        }

        /// キャリブレーション済みの加速度データを取得
        pub fn read_accel_calibrated(&mut self) -> Result<[f32; 3], E> {
            let accel = self.read_accel()?;
            Ok([
                accel[0] - self.accel_offset[0],
                accel[1] - self.accel_offset[1],
                accel[2] - self.accel_offset[2],
            ])
        }

        /// キャリブレーション済みのジャイロデータを取得
        pub fn read_gyro_calibrated(&mut self) -> Result<[f32; 3], E> {
            let gyro = self.read_gyro()?;
            Ok([
                gyro[0] - self.gyro_offset[0],
                gyro[1] - self.gyro_offset[1],
                gyro[2] - self.gyro_offset[2],
            ])
        }

        /// 加速度からPitch角度を計算 (単位: 度)
        ///
        /// 角度の定義:
        /// - 垂直（USB上、画面前）: 0°
        /// - 前に傾く（画面下向き）: 負の角度（例: -10°）
        /// - 後ろに傾く（画面上向き）: 正の角度（例: +10°）
        pub fn get_pitch(&mut self) -> Result<f32, E> {
            // キャリブレーション済み値を使用
            let accel = self.read_accel_calibrated()?;
            // ロボット姿勢に合わせてatan2の引数を入れ替え + 180°オフセット補正
            // 垂直時（USB上、画面前）: Y≈-1g, Z≈0g
            // atan2(0, -1) = 180° → 180° - 180° = 0°
            let mut pitch = libm::atan2f(accel[2], accel[1]) * RAD_TO_DEG;

            // 180°オフセット補正
            pitch -= 180.0;

            // -180°～+180°の範囲に正規化
            if pitch < -180.0 {
                pitch += 360.0;
            }

            // 符号を反転（前=負、後ろ=正にする）
            Ok(-pitch)
        }
    }
}

/// Kalmanフィルターモジュール
///
/// 1次元Kalmanフィルター: ジャイロと加速度計のデータを融合して角度を推定
mod kalman {
    /// Kalmanフィルター状態
    pub struct KalmanFilter {
        /// 推定角度 (degrees)
        angle: f32,
        /// 推定バイアス (degrees/s)
        bias: f32,
        /// 推定誤差共分散行列 P[2][2]
        p: [[f32; 2]; 2],
        /// プロセスノイズ共分散 Q
        q_angle: f32,
        q_bias: f32,
        /// 測定ノイズ共分散 R
        r_measure: f32,
    }

    impl KalmanFilter {
        /// 新しいKalmanフィルターを作成
        ///
        /// デフォルトパラメータ:
        /// - Q_angle = 0.001 (角度のプロセスノイズ)
        /// - Q_bias = 0.003 (バイアスのプロセスノイズ)
        /// - R_measure = 0.03 (測定ノイズ)
        pub fn new() -> Self {
            Self {
                angle: 0.0,
                bias: 0.0,
                p: [[0.0, 0.0], [0.0, 0.0]],
                q_angle: 0.001,
                q_bias: 0.003,
                r_measure: 0.03,
            }
        }

        /// 角度を初期化
        pub fn set_angle(&mut self, angle: f32) {
            self.angle = angle;
        }

        /// 現在の推定角度を取得
        pub fn get_angle(&self) -> f32 {
            self.angle
        }

        /// Kalmanフィルター更新
        ///
        /// # Arguments
        /// * `new_angle` - 加速度計から計算した角度 (degrees)
        /// * `new_rate` - ジャイロの角速度 (degrees/s)
        /// * `dt` - 時間ステップ (seconds)
        ///
        /// # Returns
        /// フィルター後の角度 (degrees)
        pub fn update(&mut self, new_angle: f32, new_rate: f32, dt: f32) -> f32 {
            // Step 1: 予測 (Predict)
            // 角度を積分で予測
            let rate = new_rate - self.bias;
            self.angle += dt * rate;

            // 誤差共分散行列の予測
            self.p[0][0] += dt * (dt * self.p[1][1] - self.p[0][1] - self.p[1][0] + self.q_angle);
            self.p[0][1] -= dt * self.p[1][1];
            self.p[1][0] -= dt * self.p[1][1];
            self.p[1][1] += self.q_bias * dt;

            // Step 2: 更新 (Update)
            // イノベーション（測定残差）
            let y = new_angle - self.angle;

            // イノベーション共分散
            let s = self.p[0][0] + self.r_measure;

            // Kalmanゲイン
            let k = [self.p[0][0] / s, self.p[1][0] / s];

            // 状態更新
            self.angle += k[0] * y;
            self.bias += k[1] * y;

            // 誤差共分散行列の更新
            let p00_temp = self.p[0][0];
            let p01_temp = self.p[0][1];

            self.p[0][0] -= k[0] * p00_temp;
            self.p[0][1] -= k[0] * p01_temp;
            self.p[1][0] -= k[1] * p00_temp;
            self.p[1][1] -= k[1] * p01_temp;

            self.angle
        }
    }

    impl Default for KalmanFilter {
        fn default() -> Self {
            Self::new()
        }
    }
}

/// PID制御モジュール
///
/// 倒立振子の姿勢制御用PIDコントローラー
mod pid {
    /// PID制御パラメータ
    pub struct PidController {
        // PIDゲイン
        pub kp: f32,
        pub ki: f32,
        pub kd: f32,
        pub kspd: f32,   // 速度フィードバックゲイン
        pub kdst: f32,   // 速度積分ゲイン
        pub kpower: f32, // パワー係数

        // 目標角度
        pub target_angle: f32, // 目標角度オフセット (degrees, Arduino版では約-8°)

        // 状態変数
        p_angle: f32, // 比例項
        i_angle: f32, // 積分項
        d_angle: f32, // 微分項
        k_speed: f32, // 速度項
        speed: f32,   // 速度累積

        // リミット
        i_limit: f32, // 積分項のリミット
    }

    impl PidController {
        /// 新しいPIDコントローラーを作成
        ///
        /// デフォルトパラメータ（Arduino元コードより）:
        /// - kp = 6.3
        /// - ki = 1.4
        /// - kd = 0.48
        /// - kspd = 5.0
        /// - kdst = 0.14
        /// - kpower = 0.003
        pub fn new() -> Self {
            Self {
                kp: 35.0, // パワー不足対策で増加（元: 6.3）
                ki: 1.4,
                kd: 0.48,
                kspd: 5.0,
                kdst: 0.14,
                kpower: 0.003,
                target_angle: 8.0, // 目標角度: 安定姿勢（重心バランス位置）
                p_angle: 0.0,
                i_angle: 0.0,
                d_angle: 0.0,
                k_speed: 0.0,
                speed: 0.0,
                i_limit: 300.0, // Arduino版と同じ
            }
        }

        /// PID制御計算（Arduino版と同じロジック）
        ///
        /// # Arguments
        /// * `angle` - 現在の角度（絶対値、degrees）
        ///   - Arduino版と同じく、Kalman/LPFフィルター済みの角度をそのまま渡す
        ///   - target_angleとの差分計算は行わない
        /// * `d_angle` - 角速度 (degrees/s)
        /// * `power_input` - 外部パワー入力（通常0、リモコン操作時に使用）
        ///
        /// # Returns
        /// モーターパワー出力 (-1000 ~ 1000)
        pub fn update(&mut self, angle: f32, d_angle: f32, power_input: f32) -> f32 {
            // 速度更新
            self.speed += self.kpower * power_input;

            // Arduino版と完全一致: 絶対角度をそのまま使用
            // P_Angle = kp * gain[2] * Angle （Arduino版 251行目）

            // PID項計算
            self.p_angle = self.kp * angle;
            self.i_angle += self.ki * angle + self.kdst * self.speed;
            self.d_angle = self.kd * d_angle;
            self.k_speed = self.kspd * self.speed;

            // アンチワインドアップ: 積分項のリミット（Arduino版と同じ）
            // Arduino版（258-259行目）:
            // if (I_Angle > 300) { power = Speed = I_Angle = Pitch_power = 0; }
            // if (I_Angle < -300) { power = Speed = I_Angle = Pitch_power = 0; }
            // → 積分項と速度項だけリセットして制御は継続
            if self.i_angle > self.i_limit || self.i_angle < -self.i_limit {
                self.i_angle = 0.0;
                self.speed = 0.0;
                // P項とD項は有効なので制御は継続される
            }

            // 出力計算
            let power = self.p_angle + self.i_angle + self.d_angle + self.k_speed;
            power
        }

        /// PID状態をリセット
        pub fn reset(&mut self) {
            self.p_angle = 0.0;
            self.i_angle = 0.0;
            self.d_angle = 0.0;
            self.k_speed = 0.0;
            self.speed = 0.0;
        }

        /// パラメータを設定
        pub fn set_gains(&mut self, kp: f32, ki: f32, kd: f32) {
            self.kp = kp;
            self.ki = ki;
            self.kd = kd;
        }
    }

    impl Default for PidController {
        fn default() -> Self {
            Self::new()
        }
    }
}

/// モーター制御モジュール
///
/// サーボモーター(連続回転)をGPIO+Timer疑似PWMで制御
mod motor {
    use esp_hal::gpio::Output;

    /// モーター制御構造体
    pub struct MotorController<'d> {
        pin_l: Output<'d>,
        pin_r: Output<'d>,
        offset_l: i16,
        offset_r: i16,
        neutral: u32, // 1500us
        enabled: bool,
        // PWM状態保持
        pulse_l_us: u32,
        pulse_r_us: u32,
    }

    impl<'d> MotorController<'d> {
        /// モーターコントローラーを初期化
        ///
        /// # Arguments
        /// * `pin_l` - 左モーターGPIO (GPIO 0)
        /// * `pin_r` - 右モーターGPIO (GPIO 26)
        pub fn new(pin_l: Output<'d>, pin_r: Output<'d>) -> Self {
            Self {
                pin_l,
                pin_r,
                offset_l: 0,
                offset_r: 0,
                neutral: 1500,
                enabled: false,
                pulse_l_us: 1500,
                pulse_r_us: 1500,
            }
        }

        /// モーターを駆動
        ///
        /// # Arguments
        /// * `power_l` - 左モーターパワー (-1000 ~ 1000)
        /// * `power_r` - 右モーターパワー (-1000 ~ 1000)
        pub fn drive(&mut self, power_l: f32, power_r: f32) {
            if !self.enabled {
                return;
            }

            // パワーをPWMパルス幅に変換 (500us ~ 2500us)
            // Arduino版: powerL = -power + offset, powerR = power + offset
            // 両輪とも同じ方向に回転するように、同じ符号を使用
            let pulse_l = self.neutral as f32 + power_l + self.offset_l as f32;
            let pulse_r = self.neutral as f32 + power_r + self.offset_r as f32;

            // 500us ~ 2500us にクランプ
            self.pulse_l_us = pulse_l.clamp(500.0, 2500.0) as u32;
            self.pulse_r_us = pulse_r.clamp(500.0, 2500.0) as u32;
        }

        /// 疑似PWMパルスを生成（メインループから呼び出す）
        ///
        /// Arduino版のpulse_drive()と同等の処理
        /// 50Hz = 20ms周期のPWMを生成
        pub fn pulse_drive<D: embedded_hal::delay::DelayNs>(&mut self, delay: &mut D) {
            if !self.enabled {
                return;
            }

            // Arduino版と同じ: パルス幅だけ生成し、20ms周期待機はしない
            // これにより高速ループが可能になる

            // パルス開始: 両方HIGHに設定
            self.pin_l.set_high();
            self.pin_r.set_high();

            // 短い方のパルス幅まで待機
            let min_pulse = self.pulse_l_us.min(self.pulse_r_us);
            delay.delay_us(min_pulse);

            // 短い方を先にLOWに
            if self.pulse_l_us <= self.pulse_r_us {
                self.pin_l.set_low();
                if self.pulse_l_us < self.pulse_r_us {
                    delay.delay_us(self.pulse_r_us - self.pulse_l_us);
                    self.pin_r.set_low();
                } else {
                    self.pin_r.set_low();
                }
            } else {
                self.pin_r.set_low();
                delay.delay_us(self.pulse_l_us - self.pulse_r_us);
                self.pin_l.set_low();
            }

            // Arduino版に合わせて20ms周期待機は削除
            // メインループが10ms周期で制御する
        }

        /// モーターを停止（ニュートラル位置）
        pub fn stop(&mut self) {
            self.drive(0.0, 0.0);
        }

        /// モーターを有効化/無効化
        pub fn set_enabled(&mut self, enabled: bool) {
            self.enabled = enabled;
            if !enabled {
                self.stop();
                self.pin_l.set_low();
                self.pin_r.set_low();
            }
        }

        /// オフセットを設定
        pub fn set_offset(&mut self, offset_l: i16, offset_r: i16) {
            self.offset_l = offset_l;
            self.offset_r = offset_r;
        }
    }
}

/// BLE通信モジュール (RemoteXY)
#[allow(dead_code)]
mod ble {
    // TODO: esp-idf-svc の BLE API使用
    // TODO: RemoteXY プロトコル実装
    // TODO: ジョイスティック・スライダー制御
}

/// NVS (不揮発性ストレージ) モジュール
#[allow(dead_code)]
mod nvs {
    // TODO: esp-idf-svc の NVS使用
    // TODO: オフセット値の保存・読み込み
}

/// バッテリー監視モジュール (M5StickC Plus2用)
///
/// Plus2にはAXP192が搭載されておらず、以下の構成:
/// - バッテリー充電IC: TP4057
/// - バッテリー電圧測定: GPIO38 (ADC) - R40/R41分圧
/// - 電源保持: GPIO4 (HOLD) - 起動後HIGHを維持必須
#[allow(dead_code)]
mod power {
    // M5StickC Plus2の電源管理
    // GPIO4 (HOLD): 電源保持ピン（main関数で初期化済み）
    // GPIO38 (ADC): バッテリー電圧測定（R40/R41分圧経由）
    //
    // 注意: Plus2にはAXP192が搭載されていないため、
    //       バッテリー駆動を維持するにはGPIO4をHIGHに保つ必要がある
}
