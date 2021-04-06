/*
 * bluetooth.h
 *
 *  Created on: Jan 12, 2021
 *      Author: Piotr
 */

#ifndef INC_BLUETOOTH_H_
#define INC_BLUETOOTH_H_

#define START_BALANCE 's'
#define STOP_BALANCE 'h'
#define CONFIG 'n'
#define CONTROL 'c'
#define FORWARD 'f'
#define BACKWARD 'b'
#define CONFIG_STEP 0.002f//in radiance
#define CONTROL_STEP 0.01
#define FRAME_SIZE 3
uint8_t bluetoothRxData[FRAME_SIZE];
uint8_t bluetoothTxData[]="OK\n";




#endif /* INC_BLUETOOTH_H_ */
