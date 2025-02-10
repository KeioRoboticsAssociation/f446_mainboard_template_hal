#include "main.hpp"
#include "UartLink.hpp"
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <thread>

// ---- HALのオブジェクト ----
extern UART_HandleTypeDef huart2;

// ---- 対ROS通信 ---- (不要な場合はUartLink周りを削除)
UartLink uart_link(&huart2, 0);

//射出時に機体が向くべきpitch, yawの目標値　ROSから受け取る　[rad]
float target_pitch = 0.0;
float target_yaw = 0.0;

//PID制御用の変数
PID_Controller pid_pitch;
PID_Controller pid_yaw;
float current_pitch = 0.0;
float current_yaw = 0.0;
float pitch_velocity = 0.0;
float yaw_velocity = 0.0;
float control_period = 0.01;  // 制御周期 (10ms)


// PID制御器のクラス
class PID_Controller {
    private:
        // PIDゲイン
        float Kp, Ki, Kd;
    
        // 台形制御用パラメータ
        float max_acceleration;  // 最大加速度 (rad/s^2)
        float max_velocity;      // 最大速度 (rad/s)
    
        // 制御用変数
        float integral;
        float previous_error;
        float previous_velocity;
        std::chrono::steady_clock::time_point previous_time;
    
    public:
        // コンストラクタ
        PID_Controller(float kp, float ki, float kd, float max_acc, float max_vel)
            : Kp(kp), Ki(ki), Kd(kd), max_acceleration(max_acc), max_velocity(max_vel),
              integral(0.0f), previous_error(0.0f), previous_velocity(0.0f) {
            previous_time = std::chrono::steady_clock::now();
        }
    
        // 目標角度に基づいた回転速度を計算 (台形制御付き)
        float computeVelocity(float target_angle, float current_angle) {
            // 現在の時刻を取得
            auto current_time = std::chrono::steady_clock::now();
            float dt = std::chrono::duration<float>(current_time - previous_time).count();
            previous_time = current_time;
    
            if (dt <= 0.0f) dt = 1e-6f; // ゼロ除算防止
    
            // 角度誤差
            float error = target_angle - current_angle;
    
            // PID制御計算
            integral += error * dt;
            float derivative = (error - previous_error) / dt;
            float pid_output = Kp * error + Ki * integral + Kd * derivative;
    
            // 最大速度を適用 (PIDの出力を速度とする)
            float desired_velocity = std::clamp(pid_output, -max_velocity, max_velocity);
    
            // 台形制御による加速度制限
            float max_change = max_acceleration * dt;
            float velocity = std::clamp(desired_velocity, previous_velocity - max_change, previous_velocity + max_change);
    
            // 状態を更新
            previous_error = error;
            previous_velocity = velocity;
    
            return velocity;
        }
    };


// UART受信割り込み
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        uart_link.interrupt();
    }
}

// GPIO割り込み
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    
}

void setup() {
    uart_link.start(); // ros2との通信を開始

    //pitch, yawのデータを受信するsubscriberを作成
    UartLinkSubscriber<float, float> sub(uart_link, 0);
    sub.set_callback(ros_callback);

    // PIDパラメータと台形制御の設定
    pid_pitch(1.0, 0.1, 0.05, 0.5, 1.0);  // Kp, Ki, Kd, max_acceleration, max_velocity
    pid_yaw(1.0, 0.1, 0.05, 0.5, 1.0);    // Kp, Ki, Kd, max_acceleration, max_velocity
}

void loop() {
    pitch_velocity = pid_pitch.calcVelocity(target_pitch, current_pitch);
    yaw_velocity = pid_yaw.calcVelocity(target_yaw, current_yaw);
    current_pitch += pitch_velocity * control_period;  // ここはエンコーダーなりのコードを用意して書き換える
    current_yaw += yaw_velocity * control_period;      // ここはエンコーダーなりのコードを用意して書き換える
}

//ROSからのデータ受信時のコールバック関数
void ros_callback(float pitch, float yaw) {
    target_pitch = pitch;
    target_yaw = yaw;
}
