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

// This creates a default app-descriptor required by the esp-idf bootloader.
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

    // ヒープアロケーター初期化
    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 98768);

    info!("M5StickC Plus2 倒立振子 起動中...");

    // Delay初期化
    let mut delay = Delay::new();

    // LED初期化 (GPIO 19 - M5StickC Plus2では反転ロジック)
    let mut led = Output::new(peripherals.GPIO19, Level::Low, OutputConfig::default());
    led.set_high(); // 起動中はLED点灯

    info!("GPIO初期化完了");

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
    info!("セットアップ完了!");

    // メインループ: IMUデータを読み取って表示
    loop {
        // 加速度とジャイロを取得
        if let (Ok(accel), Ok(gyro), Ok(pitch)) = (
            imu.read_accel_calibrated(),
            imu.read_gyro_calibrated(),
            imu.get_pitch(),
        ) {
            info!(
                "Accel: [{:.3}, {:.3}, {:.3}] g | Gyro: [{:.3}, {:.3}, {:.3}] deg/s | Pitch: {:.2}°",
                accel[0], accel[1], accel[2],
                gyro[0], gyro[1], gyro[2],
                pitch
            );
        } else {
            error!("IMUデータ取得失敗");
        }

        // 100ms待機
        delay.delay_millis(100);
    }
}

// ============================================================================
// 以下、今後実装予定のモジュール構成
// ============================================================================

/// LCD制御モジュール (ST7789V)
#[allow(dead_code)]
mod display {
    // TODO: mipidsi を使用してLCD初期化
    // TODO: embedded-graphics で顔の描画
    // TODO: draw_smile(), draw_henoji(), draw_wink() 実装
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
#[allow(dead_code)]
mod kalman {
    // TODO: Kalman構造体とロジック実装
    // TODO: setAngle(), getAngle() 実装
}

/// PID制御モジュール
#[allow(dead_code)]
mod pid {
    // TODO: PID制御パラメータ構造体
    // TODO: PID_ctrl() 実装
    // TODO: PID_reset() 実装
}

/// モーター制御モジュール
#[allow(dead_code)]
mod motor {
    // TODO: PWM初期化
    // TODO: pulse_drive() 実装
    // TODO: servo_stop() 実装
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
