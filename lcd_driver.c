/*
 * lcd_driver.c
 *
 * Created on: Jun 21, 2025
 * Author: dango
 *
 * I2C接続キャラクタ液晶(コントローラ: RW1063)用のドライバ 実装ファイル
 */

#include "lcd_driver.h"
#include <string.h>

// main.cで定義されているI2Cのハンドル(hi2c1)を、このファイルでも使えるようにするおまじない
extern I2C_HandleTypeDef hi2c1;

// --- 内部（static）関数 ---
// staticを付けると、このファイル内からのみ呼び出せるプライベートな関数になります。

/**
 * @brief LCDに1バイトの「コマンド」を送信します。
 * @note  コマンドは、表示をクリアしたり、カーソルを動かしたりする命令です。
 * @param cmd 送信するコマンドバイト
 */
static void lcd_send_cmd(char cmd) {
    char data_tx[2];
    data_tx[0] = 0x00; // コントロールバイト: 次の1バイトが「コマンド」であることを示す
    data_tx[1] = cmd;  // 送信するコマンド本体
    // I2Cマスターモードで、指定したアドレスに2バイトのデータを送信
    HAL_I2C_Master_Transmit(&hi2c1, LCD_I2C_ADDRESS, (uint8_t *)data_tx, 2, 200);
}

/**
 * @brief LCDに1バイトの「データ（文字）」を送信します。
 * @note  データは、表示させたい文字コード（'A', 'B', 'c'など）です。
 * @param data 送信するデータバイト
 */
static void lcd_send_data(char data) {
    char data_tx[2];
    data_tx[0] = 0x40; // コントロールバイト: 次の1バイトが「データ」であることを示す
    data_tx[1] = data; // 送信するデータ本体
    HAL_I2C_Master_Transmit(&hi2c1, LCD_I2C_ADDRESS, (uint8_t *)data_tx, 2, 200);
}


// --- 公開関数の実装 ---

/**
 * @brief LCDを初期化します。
 * @note  このシーケンスは、コントローラRW1063の安定動作のために調整されています。
 */
void lcd_init(void) {
    // 1. 電源投入後、LCD内部が安定するまで十分に待機
    HAL_Delay(100);

    // 2. Function Set (機能設定)
    // 8bitバスモード、拡張命令セットを有効にするため、2回送信するのが安定化のコツです。
    lcd_send_cmd(0x38);
    HAL_Delay(5);
    lcd_send_cmd(0x39);
    HAL_Delay(5);

    // 3. Display ON/OFF Control (表示制御)
    // ディスプレイをON、カーソルは表示しない、カーソルの位置で点滅しない設定
    lcd_send_cmd(0x0C);
    HAL_Delay(2);

    // 4. Clear Display (表示クリア)
    // 画面の表示をすべて消去します。この処理は時間がかかるので長めに待ちます。
    lcd_send_cmd(0x01);
    HAL_Delay(5);

    // 5. Entry Mode Set (エントリーモード設定)
    // 文字を書き込んだ後、カーソルが自動的に右へ移動するように設定
    lcd_send_cmd(0x06);
    HAL_Delay(2);
}

/**
 * @brief LCDの表示をすべて消去し、カーソルを左上の初期位置(0,0)に戻します。
 */
void lcd_clear(void) {
    lcd_send_cmd(0x01); // 表示クリアコマンド
    HAL_Delay(5);       // クリア処理には時間がかかるため、少し長めに待つ
}

/**
 * @brief カーソルの表示位置を指定します。
 * @param col 列 (0から始まる。20x4液晶なら0-19)
 * @param row 行 (0から始まる。20x4液晶なら0-3)
 */
void lcd_set_cursor(uint8_t col, uint8_t row) {
    // 4行LCDのアドレスマップに従って、カーソル位置を設定するコマンドを生成
    uint8_t address;
    switch (row) {
        case 0: address = 0x80 + col; break; // 1行目
        case 1: address = 0xC0 + col; break; // 2行目
        case 2: address = 0x94 + col; break; // 3行目
        case 3: address = 0xD4 + col; break; // 4行目
        default: address = 0x80;      break; // 不正な行が指定されたら先頭へ
    }
    lcd_send_cmd(address);
}

/**
 * @brief 現在のカーソル位置に文字列を表示します。
 * @param str 表示したい文字列へのポインタ
 */
void lcd_send_string(char *str) {
    // 文字列の終わり（NULL文字 '\0'）が来るまで、1文字ずつデータとして送信
    while (*str) {
        lcd_send_data(*str++);
    }
}
