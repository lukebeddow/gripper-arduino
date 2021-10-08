#pragma once

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

/* Joystick and buttons pins */

// button pins
#define button_gnd 33
#define blue_enable 35
#define white_forwards 37
#define green_backwards 39

// joystick pins
#define joystick_gnd 43
#define joystick_power 45
#define joystick_push 47

// analogue pins
#define joystick_x A8
#define joystick_y A9

/* Bluetooth pins */

#define bt_tx 16
#define bt_rx 17
#define bt_enable 24
#define bt_state 25

/* HX711 pins */

#define SG1_dt 27
#define SG1_sck 26
#define SG2_dt 49
#define SG2_sck 48
#define SG3_dt 51
#define SG3_sck 50