/*
 * lcd_driver.h
 *
 * Created on: Jun 21, 2025
 * Author: dango
 *
 * I2C接続キャラクタ液晶(コントローラ: RW1063)用のドライバ ヘッダファイル
 * 対応LCD例: AQM2004-RN-GBW (20文字x4行)
 */

#ifndef INC_LCD_DRIVER_H_
#define INC_LCD_DRIVER_H_

#include "main.h" // STM32の基本定義を読み込むために必要

// --- LCDのI2Cアドレス定義 ---
// LCDのデータシートで定められたアドレスは0x3Fです。
// STM32のHALライブラリでは、7ビットのアドレスを1ビット左にシフトした値を
// 要求するため、(0x3F << 1) と記述します。
#define LCD_I2C_ADDRESS (0x3F << 1)

// --- 公開関数プロトタイプ宣言 ---
// これらは main.c など、他のファイルから呼び出すことができる関数です。

/**
 * @brief LCDを初期化します。
 * @note  電源投入後、最初に一度だけ呼び出してください。
 */
void lcd_init(void);

/**
 * @brief LCDの表示をすべて消去し、カーソルを左上の初期位置(0,0)に戻します。
 */
void lcd_clear(void);

/**
 * @brief カーソルの表示位置を指定します。
 * @param col 列 (0から始まる。20x4液晶なら0-19)
 * @param row 行 (0から始まる。20x4液晶なら0-3)
 */
void lcd_set_cursor(uint8_t col, uint8_t row);

/**
 * @brief 現在のカーソル位置に文字列を表示します。
 * @param str 表示したい文字列へのポインタ
 */
void lcd_send_string(char *str);

#endif /* INC_LCD_DRIVER_H_ */
