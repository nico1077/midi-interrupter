/*
 * lcd_driver.c
 *
 * Created on: Jun 21, 2025
 * Author: dango
 *
 * REVISED based on user feedback and web examples.
 * This version re-introduces commands from working examples
 * to enhance initialization stability.
 */

#include "lcd_driver.h"
#include <string.h>

// main.cで定義されているI2Cのハンドル（管理情報）を、このファイルでも使えるようにする宣言
extern I2C_HandleTypeDef hi2c1;

/**
 * @brief LCDに1バイトのコマンドを送信する内部関数
 * @param cmd: 送信するコマンド
 */
static void lcd_send_cmd(char cmd) {
    char data_tx[2];
    data_tx[0] = 0x00; // コントロールバイト: コマンド
    data_tx[1] = cmd;  // コマンド本体
    HAL_I2C_Master_Transmit(&hi2c1, LCD_I2C_ADDRESS, (uint8_t *)data_tx, 2, 200);
}

/**
 * @brief LCDに1バイトのデータ（文字）を送信する内部関数
 * @param data: 送信するデータ
 */
static void lcd_send_data(char data) {
    char data_tx[2];
    data_tx[0] = 0x40; // コントロールバイト: データ
    data_tx[1] = data; // データ本体
    HAL_I2C_Master_Transmit(&hi2c1, LCD_I2C_ADDRESS, (uint8_t *)data_tx, 2, 200);
}


// --- ここからヘッダファイルで宣言した関数の実装 ---

/**
 * @brief LCDを初期化します。動作実績のあるシーケンスを参考に修正。
 * Ref: Arduino sample and https://qiita.com/nakochi/items/33a48aaa729fa9e4ef9c etc.
 */
void lcd_init(void) {
    // 1. 電源投入後、LCDが安定するまで長めに待機
    HAL_Delay(100);

    // --- 動作実績のあるサンプルコードやWebサイトを参考にした初期化シーケンス ---
    // 解説：最初にFunction Setを複数回送るのは、4bitモードと8bitモードの切り替えを確実に行うための
    // HD44780互換コントローラの伝統的な手順です。I2Cでは不要な場合もありますが、安定化のため試します。

    // Function Set (8-bit mode)
    lcd_send_cmd(0x38);
    HAL_Delay(5);

    // Function Set (8-bit mode, 拡張命令セットを有効化)
    // この 0x39 コマンドが多くのサンプルでキーとなっています。
    lcd_send_cmd(0x39);
    HAL_Delay(5);

    // 提示されたサイトでは、ここからコントローラ固有の設定(OSC, Contrast等)が入りますが、
    // RW1063にはそれらのコマンドがないため、基本的な設定を続けます。

    // Display ON/OFF (ディスプレイON, カーソルOFF, 点滅OFF)
    lcd_send_cmd(0x0C);
    HAL_Delay(2);

    // Clear Display (表示をクリア)
    lcd_send_cmd(0x01);
    // クリア処理は特に時間がかかるため、長めに待機します。
    HAL_Delay(5);

    // Entry Mode Set (文字を書くごとにカーソルが右に移動する設定)
    lcd_send_cmd(0x06);
    HAL_Delay(2);
}

void lcd_clear(void) {
    lcd_send_cmd(0x01); // Clear display command
    HAL_Delay(5);       // クリア後の待機時間を少し長くします
}

void lcd_set_cursor(uint8_t col, uint8_t row) {
    uint8_t address;
    switch (row) {
        case 0:
            address = 0x80 + col;
            break;
        case 1:
            address = 0xC0 + col;
            break;
        case 2:
            address = 0x94 + col;
            break;
        case 3:
            address = 0xD4 + col;
            break;
        default:
            address = 0x80 + col;
            break;
    }
    lcd_send_cmd(address);
}

void lcd_send_string(char *str) {
    while (*str) {
        lcd_send_data(*str++);
    }
}
