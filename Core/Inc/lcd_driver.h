/*
 * lcd_driver.h
 *
 *  Created on: Jun 21, 2025
 *      Author: dango
 */

#ifndef INC_LCD_DRIVER_H_
#define INC_LCD_DRIVER_H_

#include "main.h" // STM32の基本定義を読み込むために必要

// --- LCDの基本設定 ---
// データシートによると、このLCDのI2Cアドレスは0x3Eです。
// HALライブラリは7ビットアドレスを1ビット左にシフトした値を要求するため、こう書きます。
#define LCD_I2C_ADDRESS (0x3F << 1)

// --- 公開する関数（main.cから呼び出す関数）の宣言 ---

/**
 * @brief LCDを初期化します。電源投入後に一度だけ呼び出します。
 */
void lcd_init(void);

/**
 * @brief LCDの表示をすべて消去します。
 */
void lcd_clear(void);

/**
 * @brief 指定した位置にカーソルを移動します。
 * @param col: 列 (0-19)
 * @param row: 行 (0-3)
 */
void lcd_set_cursor(uint8_t col, uint8_t row);

/**
 * @brief 現在のカーソル位置に文字列を表示します。
 * @param str: 表示する文字列
 */
void lcd_send_string(char *str);

#endif /* INC_LCD_DRIVER_H_ */
