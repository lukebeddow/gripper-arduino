#ifndef PIN_DEFINITIONS_PCB_NEW_H_
#define PIN_DEFINITIONS_PCB_NEW_H_

/* CNC Shield Pins */

// analogue pins
#define reset A0
#define feed_hold A1
#define cycle_start A2
#define coolant_enable A3
#define not_used_1 A4
#define not_used_2 A5

// digital pins
#define xstep 2
#define ystep 3
#define zstep 4
#define xdir 5
#define ydir 6
#define zdir 7

#define motor_enable 8
#define xlim 9
#define ylim 10
#define zlim 11
#define spindle_enable 12
#define spindle_dir 13

/* PCB board 1 pins */

// button pins
#define button_gnd 32
#define blue_enable 34
#define white_forwards 36
#define green_backwards 38

// joystick pins
#define joystick_gnd 21
#define joystick_power 20
#define joystick_push 19

// analogue pins
#define joystick_x A10 //A14
#define joystick_y A11 //A15

/* Bluetooth pins */

#define bt_tx 16
#define bt_rx 17
#define bt_enable 23
#define bt_state 24

/* HX711 pins */

#define SG1_dt 52
#define SG1_sck 50
#define SG2_dt 51
#define SG2_sck 53
#define SG3_dt 41
#define SG3_sck 39
#define SG4_dt 34
#define SG4_sck 32

#endif