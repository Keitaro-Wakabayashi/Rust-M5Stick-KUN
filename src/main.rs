//! M5StickC Plus2 倒立振子ロボット - Rust版
//!
//! このプロジェクトは M5stick_plus_robo_v1.0.ino の Rust 移植版です
//!
//! ## ハードウェア構成
//! - MCU: ESP32-PICO-V3-02 (ESP32 dual-core Xtensa LX6)
//! - LCD: ST7789V (135x240 SPI)
//! - IMU: MPU6886 (I2C)
//! - PMIC: AXP192 (I2C)
//! - モーター: GPIO 0, 26 (PWM servo control)
//! - ボタン: GPIO 37 (A), GPIO 39 (B)
//! - LED: GPIO 19
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

use esp_hal::{
    clock::CpuClock,
    delay::Delay,
    gpio::{Level, Output, OutputConfig},
    i2c::master::{Config as I2cConfig, I2c},
    time::Rate,
};
use esp_backtrace as _;
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

    // LED初期化 (GPIO 19 - M5StickC Plus2では反転ロジック)
    let mut led = Output::new(peripherals.GPIO19, Level::Low, OutputConfig::default());
    led.set_high(); // 起動中はLED点灯

    // LCDバックライト初期化 (GPIO 27)
    let mut lcd_backlight = Output::new(peripherals.GPIO27, Level::High, OutputConfig::default());
    lcd_backlight.set_high(); // バックライト点灯

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
    let spi_device = ExclusiveDevice::new(spi, spi_cs, Delay::new())
        .expect("SPI device creation failed");

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

    // キャリブレーション実行
    info!("キャリブレーション開始 (500サンプル, 約1秒)...");
    match imu.calibrate(&mut delay) {
        Ok(_) => info!("キャリブレーション完了"),
        Err(_) => {
            error!("キャリブレーション失敗!");
            loop {
                led.toggle();
                delay.delay_millis(100);
            }
        }
    }

    led.set_low(); // キャリブレーション完了、LED消灯

    // Kalmanフィルター初期化
    let mut kalman = kalman::KalmanFilter::new();
    // 初期角度を設定（起動時の加速度計から）
    if let Ok(pitch) = imu.get_pitch() {
        kalman.set_angle(pitch);
        info!("Kalmanフィルター初期化: angle = {:.2}°", pitch);
    }

    info!("セットアップ完了!");

    // メインループ: IMUデータを読み取ってKalmanフィルターで処理
    const DT: f32 = 0.01; // 10ms = 0.01秒
    loop {
        // 加速度とジャイロを取得
        if let (Ok(accel), Ok(gyro), Ok(pitch_raw)) = (
            imu.read_accel_calibrated(),
            imu.read_gyro_calibrated(),
            imu.get_pitch(),
        ) {
            // Kalmanフィルター更新（Y軸ジャイロを使用）
            let pitch_filtered = kalman.update(pitch_raw, gyro[1], DT);

            // LCDに描画（フィルター後の角度を表示）
            if let Err(_) = display::draw_imu_data(&mut display, accel, gyro, pitch_filtered) {
                error!("LCD描画エラー");
            }

            // シリアル出力（生データとフィルター後を比較）
            info!(
                "Pitch: Raw={:.2}° | Filtered={:.2}° | Gyro Y={:.1}°/s",
                pitch_raw, pitch_filtered, gyro[1]
            );
        } else {
            error!("IMUデータ取得失敗");
        }

        // 10ms待機（100Hz更新）
        delay.delay_millis(10);
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
        text::Text,
    };
    use embedded_hal_bus::spi::ExclusiveDevice;
    use esp_hal::{
        delay::Delay,
        gpio::Output,
        spi::master::Spi,
    };
    use mipidsi::{models::ST7789, options::ColorInversion, Builder};

    pub type SpiDeviceType = ExclusiveDevice<Spi<'static, esp_hal::Blocking>, Output<'static>, Delay>;

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
    const GYRO_SCALE_250DPS: f32 = 131.0;  // LSB/(deg/s)
    const ACCEL_SCALE_2G: f32 = 16384.0;   // LSB/g
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
                log::warn!("MPU6886 WHO_AM_I check failed: 0x{:02x} (expected 0x19)", who_am_i);
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
        pub fn calibrate<D: embedded_hal::delay::DelayNs>(&mut self, delay: &mut D) -> Result<(), E> {
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

            self.gyro_offset[0] = gyro_sum[0] / 500.0;
            self.gyro_offset[1] = gyro_sum[1] / 500.0;
            self.gyro_offset[2] = gyro_sum[2] / 500.0;

            self.accel_offset[0] = accel_sum[0] / 500.0;
            self.accel_offset[1] = accel_sum[1] / 500.0;
            self.accel_offset[2] = accel_sum[2] / 500.0 - 1.0; // Z軸は重力(1g)を引く

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
        pub fn get_pitch(&mut self) -> Result<f32, E> {
            let accel = self.read_accel_calibrated()?;
            let pitch = libm::atan2f(accel[1], accel[2]) * RAD_TO_DEG;
            Ok(pitch)
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

        // 状態変数
        p_angle: f32,    // 比例項
        i_angle: f32,    // 積分項
        d_angle: f32,    // 微分項
        k_speed: f32,    // 速度項
        speed: f32,      // 速度累積

        // リミット
        i_limit: f32,    // 積分項のリミット
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
                kp: 6.3,
                ki: 1.4,
                kd: 0.48,
                kspd: 5.0,
                kdst: 0.14,
                kpower: 0.003,
                p_angle: 0.0,
                i_angle: 0.0,
                d_angle: 0.0,
                k_speed: 0.0,
                speed: 0.0,
                i_limit: 300.0,
            }
        }

        /// PID制御計算
        ///
        /// # Arguments
        /// * `angle` - 現在の角度 (degrees, 0 = 垂直)
        /// * `d_angle` - 角速度 (degrees/s)
        /// * `power_input` - 外部パワー入力（通常0、リモコン操作時に使用）
        ///
        /// # Returns
        /// モーターパワー出力 (-1000 ~ 1000)
        pub fn update(&mut self, angle: f32, d_angle: f32, power_input: f32) -> f32 {
            // 速度更新
            self.speed += self.kpower * power_input;

            // PID項計算
            self.p_angle = self.kp * angle;
            self.i_angle += self.ki * angle + self.kdst * self.speed;
            self.d_angle = self.kd * d_angle;
            self.k_speed = self.kspd * self.speed;

            // アンチワインドアップ: 積分項のリミット
            if self.i_angle > self.i_limit {
                self.reset();
                return 0.0;
            }
            if self.i_angle < -self.i_limit {
                self.reset();
                return 0.0;
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
/// サーボモーター(連続回転)をPWMで制御（Phase 4で完全実装予定）
mod motor {
    /// モーター制御構造体（Phase 4で完全実装予定）
    pub struct MotorController {
        offset_l: i16,
        offset_r: i16,
        neutral: u32, // 1500us
        enabled: bool,
    }

    impl MotorController {
        /// モーターコントローラーを初期化（スタブ）
        pub fn new() -> Self {
            Self {
                offset_l: 0,
                offset_r: 0,
                neutral: 1500,
                enabled: false,
            }
        }

        /// モーターを駆動（スタブ: ログ出力のみ）
        ///
        /// # Arguments
        /// * `power_l` - 左モーターパワー (-1000 ~ 1000)
        /// * `power_r` - 右モーターパワー (-1000 ~ 1000)
        pub fn drive(&mut self, power_l: f32, power_r: f32) {
            if !self.enabled {
                return;
            }
            // TODO: Phase 4でLEDC PWM実装
            log::debug!("Motor: L={:.1}, R={:.1}", power_l, power_r);
        }

        /// モーターを停止
        pub fn stop(&mut self) {
            self.drive(0.0, 0.0);
        }

        /// モーターを有効化/無効化
        pub fn set_enabled(&mut self, enabled: bool) {
            self.enabled = enabled;
        }

        /// オフセットを設定
        pub fn set_offset(&mut self, offset_l: i16, offset_r: i16) {
            self.offset_l = offset_l;
            self.offset_r = offset_r;
        }
    }

    impl Default for MotorController {
        fn default() -> Self {
            Self::new()
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

/// バッテリー監視モジュール (AXP192)
#[allow(dead_code)]
mod power {
    // TODO: I2CでAXP192通信
    // TODO: 電圧取得 (getBatteryVoltage)
}
