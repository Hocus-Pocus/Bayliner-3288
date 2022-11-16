;****************************************************************************
;
;            MS_IAC.inc (header file for MS_IAC.asm)  10/12/12
;
; Manual Idle Air Controller for Ford 351W Marine engines using MS_TECA 
; hardware 
;
;                           By Robert Hiebert 
;
;****************************************************************************

*****************************************************************************
*****************************************************************************
**   M E G A S Q U I R T - 2 0 0 1 - V2.00
**
**   (C) 2002 - B. A. Bowling And A. C. Grippo
**
**   This header must appear on all derivatives of this code.
**
*****************************************************************************
*****************************************************************************
;****************************************************************************
;  - OEM Idle Air Control Solonoid information:
;
;  - IAC Solonoid resistance 6.0-13.0 ohms (internal fly back diode)
;    Observations of IAC solonoid PWM was a frequency of 161HZ.
;****************************************************************************
;****************************************************************************
; - PWM control for the idle air control valve solonoids is done at a 
;   frequency of 200 HZ (0.005 sec period). The state of the control pots  
;   are read by the ADC converter and the 8 bit value is converted to a  
;   value from 0 - 50 in the lookup table. If PWM control is commanded, the 
;   solonoid driver is turned on in the 5 ms timer section and a counter set 
;   to the lookup table value which is counted down in the 100 us timer 
;   section. In the main loop the value of the counter is polled and when it
;   reaches zero, the driver is turned off. A value of 50 will result in an
;   on period of 0.005 seconds (100% duty cycle). 
;   Duty cycle = table value x 2   
;****************************************************************************
;****************************************************************************
;
; ------------------------- MS_IAC Hardware Wiring  -----------------------
;
;****************************************************************************
;
; ----- Power connections -----
;
;  12 Volt input   - Pin 19
;  Vref 5V output  - Pin 8
;  Common ground   - Pins 22,23,24,26,27,29,32,33,34,35
;
; ----- Inputs [Port Name - Function - Pin#] -----
;
;  PTB4/AD4  - Battery Voltage/Boot Loader Entry           - No Pin
;  PTB5/AD5  - Idle Air Control Input Port                 - Pin 5
;  PTB6/AD6  - Idle Air Control Input Stbd                 - Pin 25
;
; ----- Outputs [Port Name - Function - Pin#] -----
;
;  PTD3/SPSCK - Program Loop Counter LED                   - No Pin
;  PTD4/T1CH0 - Idle Air Control Solonoid Port             - Pin 36
;  PTD5/T1CH1 - Idle Air Control Solonoid Stbd             - Pin 37
;
; ----- Spares [Port name/Pin# - Header Socket#] -----
;
;  PTD0/SS    - H1
;  PTD1/MISO  - H2
;  Pin 1      - H3
;  Pin 2      - H4
;
;****************************************************************************

;****************************************************************************
; ------------------------ Output Port Equates ------------------------------
;****************************************************************************

LoopFrq  equ 3     ; Loop frequency counter on Port D3
IACpwmP  equ 4     ; IAC PWM Port Engine on Port D4
IACpwmS  equ 5     ; IAC PWM Stbd Engine on Port D5

;****************************************************************************
; ---------------- "inputs" Operating Status bit field equates --------------
;****************************************************************************

clk100k     equ 1     ; 1 = 0.1S   clock tick flag   0 = "clk100k" cleared
adcc        equ 2     ; 1 = ADC complete flag        0 = "adcc" cleared
iaconP      equ 4     ; 1 = IAC PW Port on flag      0 = "iaconP" cleared
iaconS      equ 5     ; 1 = IAC PW Stbd on flag      0 = "iaconS" cleared

;****************************************************************************
; ----------------- RAM Variables (512 bytes available) ---------------------
; I/O registers from $0000 to $0039 (decimal 0 to 63)
; Direct page addressing from $0000 to $00FF (decimal 0 to 255)
; Ram end at $023F (decimal 575)
;****************************************************************************

;Memory Location $0040 (decimal 64)

ms_ram_start:

;****************************************************************************
; ------------ RS232 Real Time Download and Datalog Variables ---------------
;****************************************************************************

secH:           ds 1     ; Seconds counter, Hi byte
secL:           ds 1	 ; Seconds counter, Lo byte
BAT:            ds 1     ; Battery Voltage 8 bit ADC reading
IACP:           ds 1     ; Port Engine Idle Air Control Sensor 8 bit ADC reading
IACS:           ds 1     ; Stbd Engine Idle Air Control Sensor 8 bit ADC reading
Volts:          ds 1     ; Battery voltage to 0.1V resolution
IACpwP:         ds 1     ; Port IAC pulse width variable(0-60, 100uS resolution)
IACpwS:         ds 1     ; Stbd IAC pulse width variable(0-60, 100uS resolution)

;****************************************************************************
; This marks the end of the real time download variables, 8 in total
;****************************************************************************

;****************************************************************************
; --------------------- Status Bit Field Variables --------------------------
; - NOTE! ";*"  denotes that this variable is listed with RS232 variables
;****************************************************************************

inputs:         ds 1     ; Input status bit field variable(1 of 2)

;****************************************************************************
; ----------------------- Time Clock Variables ------------------------------
; - NOTE! ";*"  denotes that this variable is listed with RS232 variables
;****************************************************************************

uSx100:         ds 1	 ; 100 Microseconds counter
mS:             ds 1	 ; Milliseconds counter
mSx5:           ds 1     ; 5 Milliseconds counter
mSx20:          ds 1     ; 20 Milliseconds counter
mSx100:         ds 1     ; 100 Milliseconds counter
;*secL:           ds 1     ; Seconds counter, Lo byte
;*secH:           ds 1     ; Seconds counter, Hi byte


;****************************************************************************
; --------------------- 8 bit ADC input Variables --------------------------
; ----------------- (ordered list from "MAP" to "TPS") --------------------
; - NOTE! ";*"  denotes that this variable is listed with RS232 variables
;****************************************************************************

;*BAT:            ds 1     ; Battery Voltage 8 bit ADC reading
;*IACP:           ds 1     ; Port Engine Idle Air Control Sensor 8 bit ADC reading
;*IACS:           ds 1     ; Stbd Engine Idle Air Control Sensor 8 bit ADC reading
adsel:          ds 1     ; ADC Selector Variable

;****************************************************************************
; --------------------- ADC Conversion Variables --------------------------
; - NOTE! ";*"  denotes that this variable is listed with RS232 variables
;****************************************************************************

;*IACP:           ds 1     ; Port Engine Idle Air Control Sensor 8 bit ADC reading
;*IACS:           ds 1     ; Stbd Engine Idle Air Control Sensor 8 bit ADC reading
;*Volts:          ds 1     ; Battery voltage to 0.1V resolution

;***************************************************************************
; - Serial Communications Interface Variables
;***************************************************************************

txcnt:          ds 1     ; SCI transmitter count (incremented)

;Memory Location $0050 (decimal 80)

txgoal:         ds 1     ; SCI number of bytes to transmit
txmode:         ds 1     ; Transmit mode flag
rxoffset:       ds 1     ; Offset placeholder when receiving VE/constants
                         ; vis. SCI

;****************************************************************************
; ----------------------------- Misc. Variables -----------------------------
;****************************************************************************

LoopCntr:      ds 1     ; Loop counter for main loop frequency check
IACcntP:       ds 1     ; IAC counter for IACpwP (100uS resolution)
IACcntS:       ds 1     ; IAC counter for IACpwS (100uS resolution)
AIACcntP:      ds 1     ; Port Auto IAC duration counter value(100mS res)
AIACcntS:      ds 1     ; Stbd Auto IAC duration counter value(100mS res)
blank1:        ds 1     ; Placeholder
blank2:        ds 1     ; Placeholder
blank3:        ds 1     ; Placeholder
blank4:        ds 1     ; Placeholder
blank5:        ds 1     ; Placeholder
blank6:        ds 1     ; Placeholder
blank7:        ds 1     ; Placeholder
blank8:        ds 1     ; Placeholder

;Memory Location $0060 (decimal 96)

ms_ram_end:
ms_rf_start:
ms_rf_end:

;****************************************************************************
; ---------------------------- Misc. Equates --------------------------------
;****************************************************************************

ms_ram_size       equ {ms_ram_end-ms_ram_start}
ms_rf_size        equ {ms_rf_end-ms_rf_start}
ms_total_ram_size equ {ms_rf_end-ms_ram_start}


