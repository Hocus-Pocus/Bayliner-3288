;****************************************************************************
;
;                                MV351WM.asm
;
;        Real Time Variable Display and "digital dashboard" for MSnS351WM
;
;                         By Robert Hiebert 2010
;
; - Rev 1: 4/16/12 Add configurable bank fuel flow constants for fuel burn calculation
;          fine tuning
;
;****************************************************************************

;***********************************************************************************************
;
; ------------------------------------------- Operation ----------------------------------------
;
; On power up, the unit defaults to display screen #0.
; The user has a choice of 19,(0-18), screens which display the variables name
; abreviation on the top line, and their corresponding real time values on
; the bottom line. The lower line is updated every 250 miliseconds.
; The earlier version, MV_ECU also had a constant configuration mode which
; this version does not. This is a "display only" unit.
;
; Screen 0 displays "ERPM MAP AFR CLT MAT" ; Operating status 1 (default screen)
; "ERPM" is Engine RPM, uses calculated "rpmhrH:rpmhrL"
; "MAP"  is Manifold Absolute Pressure in KPA, uses "map"
; "AFR"  is Air Fuel Ratio x 10, uses "afr"
; "CLT"  is Engine Coolant Temp in degrees F+40, uses "clt"
; "MAT"  is Manifold Air Temperature in degrees F+40, uses "mat"
;
; Screen 1 displays "TPP EOP EFP EXGT VLT" ; Operating status 2 screen
; "TPP"  is Throttle position in percent, uses "tpsp"
; "EOP"  is Engine oil pressure in PSI, uses "oprs"
; "EFP"  is Engine fuel pressure in PSI, uses "fprs"
; "EXGT" is Engine exhaust gas temperature in degrees F, uses "egtH:egtL"
; "VLT"  is Battery Voltage x 10, uses "volts"
;
; Screen 2 displays " ERPM AFR FDSC LTHR " ; Fuel burn screen
; "ERPM" is Engine RPM, uses calculated "rpmH:rpmL"
; "AFR"  is Air Fuel Ratio x 10, uses "afr"
; "FDSC" is Fuel Delivery Injector On Time over a 1 second period in mmS, uses "fdSecH:fdSecL"
; "LTHR" is Current fuel burn in L/Hr x 10, uses "ltrHrH:ltrHrL"
;
; Screen 3 displays "ERPM MAP AFR FTC PLW" ; Fuel tune screen
; "ERPM" is Engine RPM, uses calculated "rpmhrH:rpmhrL"
; "MAP"  is Manifold Absolute Pressure in KPA, uses "map"
; "AFR"  is Air Fuel Ratio x 10, uses "afr"
; "FTC"  is Fuel Trim Correction in percent, uses "Ftrimcor"
; "PLW"  is Pulse Width Lo Res in mmS, uses "pw"
;
; Screen 4 displays "ERPM MAP TRA CTA MTA" ; Ignition tune screen
; "ERPM" is Engine RPM, uses calculated "rpmhrH:rpmhrL"
; "MAP"  is Manifold Absolute Pressure in KPA, uses "map"
; "TRA"  is Ignition Trim Angle in degrees BTDC, uses "trmAng"
; "CTA"  is Calculated Timing Angle in degrees BTDC, uses "timAng"
; "MTA"  is Monitored Ignition Timing Angle in degrees BTDC, uses "monTimAng"
;
; Screen 5 displays " GAM WCR ACR BCR TCR" ; Corrections screen
; "GAM"  is Gamma Correction in percent, uses gammae
; "WCR"  is Engine Coolant Correction in percent, uses "warmcor
; "ACR"  is Manifold Air Temperature Correction in percent, uses "aircor"
; "BCR"  is Barometric Pressure Correction in percent, uses "barocor"
; "TCR"  is Acceleration Correction in percent, uses "tpsaccel"
;
; Screen 6 displays " ego EGV LMD AFR FTC" ; EGO screen
; "ego"  is Exhaust Gas Oxygen ADC, uses "egoADC"
; "EGV"  is Exhaust Gas Oxygen voltage x 10, uses "egoV"
; "LMD"  is Exhaust Gas Oxygen Lambda x 10, uses "lambda"
; "AFR"  is Air Fuel Ratio x 10, uses "afr"
; "FTC"  is Fuel Trim Correction in percent, uses "Ftrimcor"
;
; Screen 7 displays " map MAP     BAR BCR" ; ADC screen 1
; "map"  is Manifold Pressure ADC, uses "mapADC"
; "MAP"  is Manifold Absolute Pressure in KPA, uses "map"
; "BAR"  is Barometric Pressure in KPA, uses "barometer"
; "BCR"  is Barometric Pressure Correction in percent, uses "barocor"
;
; Screen 8 displays " vlt VLT clt CLT WCR" ; ADC screen 2
; "vlt"  is Battery Voltage ADC, uses "battADC"
; "VLT"  is Battery Voltage x 10, uses "volts"
; "clt"  is Engine Coolant Temperature ADC, uses "cltADC"
; "CLT"  is Engine Coolant Temperature in degrees F+40, uses "clt"
; "WCR"  is Engine Coolant Correction in percent, uses "warmcor"
;
; Screen 9 displays "egt EXGT mat MAT ACR" ; ADC screen 3
; "egt"  is Exhaust Gas Temperature ADC, uses "egtADC"
; "EXGT" is Engine exhaust gas temperature in degrees F, uses "egtH:egtL"
; "mat"  is Manifold Air Temperature ADC, uses "matADC"
; "MAT"  is Manifold Air Temperature in degrees F+40, uses "mat"
; "ACR"  is Manifold Air Temperature Correction in percent, uses "aircor"
;
; Screen 10 displays " ftc  FTC  itc  TAF " ; ADC screen 4
; "ftc"  is Fuel trim ADC, uses "ftrimADC"
; "FTC"  is Fuel Trim Correction in percent, uses "Ftrimcor"
; "itc"  is Ignition Trim Correction ADC, uses "itrimADC"
; "TAF"  is Ignition Trim Angle Factor, uses "trmAngFac"
;
; Screen 11 displays " eop  EOP  efp  EFP " ; ADC screen 5
; "eop"  is Engine oil pressure ADC, uses "opADC"
; "EOP"  is Engine oil pressure in PSI, uses "oprs"
; "efp"  is Engine fuel pressure ADC, uses "fpADC"
; "EFP"  is Engine fuel pressure in PSI, uses "fprs"
;
; Screen 12 displays "PWHR PLW FLD VEC DTY" ; Pulsewidth screen
; "PWHR" is Pulse Width Hi Res in mmmS, uses "pwcalcH:pwcalcL"
; "PLW"  is Pulse Width Lo Res in mmS, uses "pw"
; "FLD"  is Fuel Deleivery Pulse Width Lo Res in mmS, uses "fd"
; "VEC   is Volumetric Efficiency(Current VE table value in percent), uses "vecurr"
; "DTY"  is Injector Duty Cycle in percent, uses "duty"
;
; Screen 13 displays " ERPM PIPD MNPD SEC " ; Period screen
; "ERPM" is Engine RPM, uses calculated "rpmhrH:rpmhrL"
;*; "PIPD" is PIP Period Predicted in mmmS, uses "pippprdH:pippprdL"
; "PIPD" is PIP Period in mmmS, uses "tachH:tachL"
; "MNPD" is Ignition Monitor Period in mmmS, uses "monpH:monpL"
; "SEC"  is Seconds Counter Lo byte, uses "secL"
;
; Screen 14 displays "ERPM MAP SAF DAF TAF" ; Ignition factor screen
; "ERPM" is Engine RPM, uses calculated "rpmH:rpmL"
; "MAP"  is Manifold Absolute Pressure in KPA, uses "map"
; "SAF"  is Spark Angle Factor(Current ST table value), uses "spkAngFac"
; "DAF"  is Delay Angle Factor, uses "dlyAngFac"
; "TAF"  is Ignition Trim Angle Factor, uses "trmAngFac"
;
; Screen 15 displays "ERPM TRA DLA CTA MTA" ; Ignition angle screen
; "ERPM" is Engine RPM, uses calculated "rpmH:rpmL"
; "TRA"  is Ignition Trim Angle in degrees BTDC, uses "trmAng"
; "DLA"  is Ignition Delay Angle in degrees BTDC, uses "dlyAng"
; "CTA"  is Calculated Timing Angle in degrees BTDC, uses "timAng"
; "MTA"  is Monitored Ignition Timing Angle in degrees BTDC, uses "monTimAng"
;
; Screen 16 displays "FP RN CR SW RW AC DC" ; "portAbits", "Engine" bit field screen
; "FP"   is Fuel Pump(portAbits 0)
; "RN"   is Engine Running(engine 0)
; "CR"   is Engine Cranking(engine 1)
; "SW"   is Start Warmup(engine 2)
; "RW"   is Run Warmup(engine 3)
; "AC"   is Accellerating(engine 4)
; "DC"   is Decelerating(engine 5)
;
; Screen 17 displays "EA RL FC       FT IT" ; "portCbits", "Alarmbits", "portAbits" screen
; "EA"   is Engine Alarm(portCbits 0)
; "RL"   is Rev Limiter(alarmbits 5)
; "FC"   is Flood Clear(alarmbits 6)
; "FT"   is Fuel Trim Enable(portAbits 5)
; "IT"   is Ignition Trim Enable(portAbits 2)
;
; Screen 18 displays "EA HT LO LF HF ET KN" ; "portCbits", "Alarmbits" screen
; "EA"   is Engine Alarm(portCbits 0)
; "HT"   is High engine temperature(alarmbits 1)
; "LO"   is Low engine oil pressure(alarmbits 0)
; "LF"   is Low fuel pressure(alarmbits 2)
; "HF"   is High fuel pressure(alarmbits 3)
; "ET"   is High exhaust gas temperature(alarmbits 4)
; "KN"   is Engine knock detected(alarmbits 7)
;
;***********************************************************************************************

;****************************************************************************

;****************************************************************************
;
; -------------------------- MV351WM Hardware Wiring  -----------------------
;
;****************************************************************************
;
; ----- Inputs [port name - function] -----
;
;  PTA0      - Display Screen next
;  PTA1      - Display Screen previous
;  PTA2      - not used
;  PTA3      - not used
;
; ----- Outputs [port name - function] -----
;
;  PTB4       - VFD Display Enable
;  PTB5       - VFD Display R/W
;  PTB6       - VFD Display RS
;  PTC0       - VFD Display data DB4
;  PTC1       - VFD Display data DB5
;  PTC2       - VFD Display data DB6
;  PTC3       - VFD Display data DB7
;
;****************************************************************************

;****************************************************************************
; ------------------------ Input Port Equates -------------------------------
;****************************************************************************
;
;Sw0     equ 0     ; PTA0 - Display Screen next
;Sw1     equ 1     ; PTA1 - Display Screen previous
;Sw2     equ 2     ; PTA2 - not used
;Sw3     equ 3     ; PTA3 - not used
;
; ----- Outputs [port name - function] -----
;
;  PTB4       - VFD Display Enable
;  PTB5       - VFD Display R/W
;  PTB6       - VFD Display RS
;  PTC0       - VFD Display data DB4
;  PTC1       - VFD Display data DB5
;  PTC2       - VFD Display data DB6
;  PTC3       - VFD Display data DB7
;
;****************************************************************************

.header 'MV351WM'           ; Listing file title
.pagewidth 130              ; Listing file width
.pagelength 90              ; Listing file height

.nolist                     ; Turn off listing file
     include "gp32.equ"     ; Include HC 908 equates
.list                       ; Turn on listing file
     org	   ram_start    ; Origin  Memory location $0040=64
                                ;(start of RAM)
     include "MV351WM.inc"  ; Include definitions for MV351WM.asm

;****************************************************************************
;
; ----------------- Configure system and set up clock ----------------------
;
;****************************************************************************

     org     rom_start              ; Origin at memory location
                                    ; ($8000 = 32,768)(start of ROM)

START:

;****************************************************************************
; - Set Configuration Register 1
;****************************************************************************

     mov     #$3B,CONFIG1     ; Move %00111011 into Configuration Register 1
                              ;(COP timeout period=2p18-2p4 CGMXCLK cycles)
                              ;(LVI disabled during stop mode)
                              ;(LVI module resets disabled)
                              ;(LVI module power disabled)
                              ;(LVI operates in 5-V mode)
                              ;(Stop mode recovery after 4096 CGMXCLKC cycls)
                              ;(Stop instruction enabled)
                              ;(COP module disabled)

;****************************************************************************
; - Set Configuration Register 2
;****************************************************************************

     mov     #$01,CONFIG2     ; Move %00000001 into Configuration Register 2
                              ;(Oscillator disabled during stop mode)
                              ;(Internal data bus clock used as clock source
                              ; for SCI)

;****************************************************************************
; - Set The Stack Pointer to the bottom of RAM
;****************************************************************************

     ldhx     #ram_last+1           ; Load index register with value in
                                    ; "ram_last" +1 ($023F+1=$0240=576)
     txs                            ; Transfer value in index register Lo
                                    ; byte to stack

;****************************************************************************
; - Initialize the PLL Control Register for a bus frequency of 8.003584mhz
;****************************************************************************

     mov     #$02,PCTL      ; Move %00000010 into PLL Control Register
                            ;(PLL Interrupts Disabled)
                            ;(No change in lock condition(flag))
                            ;(PLL off)
                            ;(CGMXCLK divided by 2 drives CGMOUT)
                            ;(VCO pwr of 2 mult = 1(E=0))
                            ;(Prescale mult = 4(P=2))
     mov     #$03,PMSH      ; Move %00000011 into PLL Multiplier Select
                            ; Register Hi (Set N MSB)
     mov     #$D1,PMSL      ; Move %11010001 into PLL Multiplier Select
                            ; Register Lo (Set N LSB)($84 for 7.37 MHz)
     mov     #$D0,PMRS      ; Move %11010000 into PLL VCO Range Select
                            ; Register (Set L) ($C0 for 7.37 MHz)
     mov     #$01,PMDS      ; Move %00000001 into Reference Divider Select
                            ; Register (Set "RDS0" bit (default value of 1)
     bset    AUTO,PBWC      ; Set "Auto" bit of PLL Bandwidth Control Register
     mov     #$32,PCTL      ; Move %00100000 into PLL Control Register
                            ;(PLL On)

PLL_WAIT:
     brclr   LOCK,PBWC,PLL_WAIT     ; If "Lock" bit of PLL Bandwidth Control
                                    ; Register is clear, branch to PLL_WAIT:
     bset    BCS,PCTL               ; Set "BCS" bit of PLL Control Register
                                    ;(CGMXCLK divided by 2 drives CGMOUT)
                                    ;(Select VCO as base clock)


;****************************************************************************
;
; ----------------------------- Set up RS 232 ------------------------------
;
;****************************************************************************

;****************************************************************************
; - Enable/disable loop mode
;****************************************************************************

     mov     #$40,SCC1     ; Move %01000000 into SCI Control Register 1
                           ;(Normal operation enabled)
                           ;(Set "ENSCI" bit)(SCI enabled)
                           ;(Transmitter output not inverted)
                           ;(8 bit SCI characters)
                           ;(Idle line wakeup)
                           ;(Idle character bit countbegins after start bit)
                           ;(Parity function disabled)
                           ;(Even Parity)
     bset    PTY,SCC1      ; Set "PTY" bit of SCI Control Register 1
                           ;(Odd Parity)???

;****************************************************************************
; - This register initialize interrupts request and activates the
;   transmitter and receiver and wakeup mode
;****************************************************************************

     mov     #$0C,SCC2     ; Move %00001100 into SCI Control Register 2
                           ;(SCTIE not enabled to generate CPU interrupt)
                           ;(TCIE not enabled to generate CPU interrupt)
                           ;(SCRIE not enabled to generate CPU interrupt)
                           ;(ILIE not enabled to generate CPU interrupt)
                           ;(Set "TE" bit)(Transmitter enabled)
                           ;(Set "RE" bit)(Receiver enabled)
                           ;(Normal Operation)
                           ;(No break characters being transmitted)

;****************************************************************************
; - This register initialize the DMA services and error interrupts
;****************************************************************************

     clr     SCC3          ; Clear SCI Control Register 3
                           ;(DMA not enabled to service SCI receiver)
                           ;(SCTE DMA service requests disabled)
                           ;(SCI error CPU interrupt requests for OR bit
                           ; disabled)
                           ;(SCI error CPU interrupt requests for NE bit
                           ; disabled)
                           ;(SCI error CPU interrupt requests for FE bit
                           ; disabled)
                           ;(SCI error CPU interrupt requests for PE bit
                           ; disabled)

;****************************************************************************
; - This register sets baud rate
;****************************************************************************

     lda      #$30           ; Load accumulator with %00110000
     sta      SCBR           ; Copy to SCI Baud Rate Register
                             ; 8003584mhz/(64*13*1)=9619.7 baud

;****************************************************************************
; ------------- Set up the port data-direction registers --------------------
;               Set directions,
;               Preset state of pins to become outputs
;               Set all unused pins to outputs initialized Lo
;****************************************************************************

;****************************************************************************
; - Set up VFD control line I/Os
;****************************************************************************

; Port B
     clr     PORTB           ; Clear Port B Data Register
                             ;(Preinit all pins low)
     lda     #$FF            ; Load accumulator with %11111111
                             ;(port direction setup 1 = output)
     sta     DDRB            ; Copy to Port A Data Direction Register
                             ; Set all as outputs
                             ; NA,RS,R/W,En,NA,NA,NA,NA

; Port C
     clr     PORTC           ; Clear Port C Data Register
                             ;(Preinit all pins low)
     lda     #$FF            ; Load accumulator with %11111111
                             ; (set up port directions, 1 = out)
     sta     DDRC            ; Copy to Port C Data Direction Register
                             ; Set all as outputs
                             ; NA,NA,NA,NA,DB7,DB6,DB5,DB4

;****************************************************************************
; - Set up for push button inputs
;****************************************************************************

; Port A
     mov     #$FF,PTAPUE     ; Move %11111111 into Port A pullup register
                             ;(Set all pullups)
     clr     PORTA           ; Clear Port A Data Regisister
                             ;(preinit all pins Lo)
     lda     #$F0            ; Load accumulator with %11110000
                             ;(port direction setup 1 = output)
     sta     DDRA            ; Copy to Port A Data Direction Register
                             ; Inputs on PTA3,2,1,0
                             ; Tog Mode,Frz/Sel,Scrl Rt/Inc.Scrl Lft/Dec
                             ; Outputs on PTA7,6,5,4 (not used)

;****************************************************************************
; - Set up Ports D and E.(The Motorola manual states that it is not
;   necessarry to set up Port E when SCI is enabled, but we'll do it anyway).
;****************************************************************************

; Port D
     clr     PORTD           ; Clear Port D Data Register
                             ;(Preinit all pins low)
     lda     #$FF            ; Load accumulator with %11111111
                             ; (init port directions 1 = out)
     sta     DDRD            ; Copy to Port D Data Direction Register
                             ; Set all as outputs
                             ; NA,NA,NA,NA,NA,NA,NA,NA

; Port E
     clr     PORTE           ; Clear Port E Data Register (to avoid glitches)
     lda     #$01            ; Load accumulator with %00000001
                             ; (set up port directions, 1 = out)
                             ; (Serial Comm Port)
     sta     DDRE            ; Copy to Port E Data Direction Register


;***************************************************************************
; - Initialize the variables
;***************************************************************************

     clra                   ; Clear accumulator
     clrh                   ; Clear index register Hi byte
     clrx                   ; Clear index register Lo byte
     clr     Sw0DB          ; Switch #0 de-bounce timer counter variable
     clr     Sw0ARC         ; Switch #0 auto-repeat command timer counter var
     clr     Sw0AR          ; Switch #0 auto-repeat timer counter variable
     clr     Sw1DB          ; Switch #1 de-bounce timer counter variable
     clr     Sw1ARC         ; Switch #1 auto-repeat command timer counter var
     clr     Sw1AR          ; Switch #1 auto-repeat timer counter variable
     clr     Sw2DB          ; Switch #2 de-bounce timer counter variable
     clr     Sw2ARC         ; Switch #2 auto-repeat command timer counter var
     clr     Sw2AR          ; Switch #2 auto-repeat timer counter variable
     clr     Sw3DB          ; Switch #3 de-bounce timer counter variable
     clr     Sw3ARC         ; Switch #3 auto-repeat command timer counter var
     clr     Sw3AR          ; Switch #3 auto-repeat timer counter variable
     clr     LPflags        ; Switch last pass status bit field variable
     clr     ARCflags       ; Switch auto-repeat command status bit field
     clr     ARflags        ; Switch auto-repeat status bit field variable
     clr     Swflags        ; Switch status bit field variable
     clr     ModeCntr       ; Counter for determining "mode" bit status
     clr     FrzCntr        ; Counter for determining "frz" bit status
     clr     SelCntr        ; Counter for determining "sel" bit status
     clr     flags          ; Bit field for operating status flags (1 of 2)
     clr     ScrnCnt        ; Counter for display screen numbers
     clr     ScrnCnt_prv    ; Screen count number previous
     clr     ConCnt         ; Counter for Constant numbers
     clr     ConCnt_prv     ; Constant number previous
     clr     ComVal         ; Value for VFD command data
     clr     TopVal         ; Value for VFD top line data
     clr     DisVal         ; Value for VFD bottom line variable data
     clr     ConVal         ; Value for VFD bottom line constant data
     clr     CurCon         ; Value of current selected constant
     clr     ColNum         ; Value of  Column number for VFD
     clr     value          ; Value for VFD data
     clr     DatVal         ; Data value for VFD
     clr     ByteCnt        ; Count of bytes to receive via SCI
     clr     ByteGoal       ; Desired number of bytes to receive via SCI
     clr     readbuf        ; Buffer for temporary storage of received byte
     clr     mS             ; Milliseconds counter
     clr     mSx5           ; 5 Milliseconds counter
     clr     ByteCnt        ; Count of bytes to receive via SCI
     clr     ByteGoal       ; Desired number of bytes to receive via SCI
     clr     readbuf        ; Buffer for temporary storage of received byte
     clr     value          ; Value sent to VFD(instruction or data)
     clr     LineNum        ; Line number for VFD(for instruction)
     clr     ColNum         ; Column number for VFD(for instruction)
     clr     DatVal         ; Data value for VFD
     clr     ComVal         ; Value for VFD command data
     clr     TopVal         ; Value for VFD top line data
     clr     DisVal         ; Value for VFD bottom line variable data
     clr     BotLin0        ; Bottom Line Column 0
     clr     BotLin1        ; Bottom Line Column 1
     clr     BotLin2        ; Bottom Line Column 2
     clr     BotLin3        ; Bottom Line Column 3
     clr     BotLin4        ; Bottom Line Column 4
     clr     BotLin5        ; Bottom Line Column 5
     clr     BotLin6        ; Bottom Line Column 6
     clr     BotLin7        ; Bottom Line Column 7
     clr     BotLin8        ; Bottom Line Column 8
     clr     BotLin9        ; Bottom Line Column 9
     clr     BotLin10       ; Bottom Line Column 10
     clr     BotLin11       ; Bottom Line Column 11
     clr     BotLin12       ; Bottom Line Column 12
     clr     BotLin13       ; Bottom Line Column 13
     clr     BotLin14       ; Bottom Line Column 14
     clr     BotLin15       ; Bottom Line Column 15
     clr     BotLin16       ; Bottom Line Column 16
     clr     BotLin17       ; Bottom Line Column 17
     clr     BotLin18       ; Bottom Line Column 18
     clr     BotLin19       ; Bottom Line Column 19
     clr     BotLin0L       ; Bottom Line Column 0 last pass
     clr     BotLin1L       ; Bottom Line Column 1 last pass
     clr     BotLin2L       ; Bottom Line Column 2 last pass
     clr     BotLin3L       ; Bottom Line Column 3 last pass
     clr     BotLin4L       ; Bottom Line Column 4 last pass
     clr     BotLin5L       ; Bottom Line Column 5 last pass
     clr     BotLin6L       ; Bottom Line Column 6 last pass
     clr     BotLin7L       ; Bottom Line Column 7 last pass
     clr     BotLin8L       ; Bottom Line Column 8 last pass
     clr     BotLin9L       ; Bottom Line Column 9 last pass
     clr     BotLin10L      ; Bottom Line Column 10 last pass
     clr     BotLin11L      ; Bottom Line Column 11 last pass
     clr     BotLin12L      ; Bottom Line Column 12 last pass
     clr     BotLin13L      ; Bottom Line Column 13 last pass
     clr     BotLin14L      ; Bottom Line Column 14 last pass
     clr     BotLin15L      ; Bottom Line Column 15 last pass
     clr     BotLin16L      ; Bottom Line Column 16 last pass
     clr     BotLin17L      ; Bottom Line Column 17 last pass
     clr     BotLin18L      ; Bottom Line Column 18 last pass
     clr     BotLin19L      ; Bottom Line Column 19last pass
     clr     AC_100         ; 8 bit ASCII conversion 100s column
     clr     AC_10          ; 8 bit ASCII conversion 10s column
     clr     AC_1           ; 8 bit ASCII conversion 1s column
     clr     map            ; Manifold Absolute Pressure in KPA
     clr     mat            ; Manifold Temperature in degrees F + 40
     clr     clt            ; Engine Coolant Temperature in degrees F + 40
     clr     volts          ; Battery (system) voltage
     clr     egoV           ; Exhaust Gas Oxygen sensor voltage
     clr     lambda         ; Exhaust Gas Oxygen lambda
     clr     afr            ; Exhaust Gas Oxygen Air/Fuel Ratio
     clr     duty           ; Injector duty cycle
     clr     trmAng         ; Ignition Trim Angle
     clr     dlyAng         ; Ignition Delay Angle
     clr     timAng         ; Calculated Ignition Timing Angle
     clr     monTimAng      ; Monitored Ignition Timing Angle
     clr     ltrHrH         ; Fuel burn in Litres per Hour x 10 Hi byte
     clr     ltrHrL         ; Fuel burn in Litres per Hour x 10 Lo byte
     clr     oprs           ; Engine oil pressure in PSI
     clr     fprs           ; Fuel pressure in PSI
     clr     egtH           ; Exhaust temperature in degrees F Hi byte
     clr     egtL           ; Exhaust temperature in degrees F Lo byte
     clr     rpmH           ; Engine RPM Lo Res Hi Byte
     clr     rpmL           ; Engine RPM Lo Res Lo byte
     clr     rpmhrH         ; Engine RPM Hi Res Hi Byte
     clr     rpmhrL         ; Engine RPM Hi Res Lo byte
     clr     monDlyAngFac   ; Monitored Delay Angle Factor
     clr     monDlyAng      ; Monitored Delay Angle
     clr     monFrngAng     ; Monitored Firing Angle
     clr     BnkFloH        ; Injector bank flow rate in cc/sec x 1,000,000 Hi byte
     clr     BnkFloL        ; Injector bank flow rate in cc/sec x 1,000,000 Lo byte




     mov     #$FF,ScrnCnt_Lst     ; Move decimal 255 into "ScrnCnt_Lst"
     mov     #$FF,ConCnt_Lst      ; Move decimal 255 into "ConCnt_Lst"

;***************************************************************************
; - Delay while power stabilizes, allow MS and VFD to come up.
;   One pass through the primary loop takes ~1.5uS, so this delay is ~300mS
;   (minimum delay is 260mS)
;***************************************************************************

     jsr     DELAY300     ; Jump to subroutine at DELAY300:

;****************************************************************************
; Set up TIM2 as a free running ~1us counter. Set Channel 0 output compare
; to generate the ~1000us(1.0mS) clock tick interupt vector "TIM2CH0_ISR:"
;****************************************************************************

     mov     #$33,T2SC       ; Move %00110011 into Timer2
                             ; Status and Control Register
                             ;(Disable interrupts, stop timer)
                             ;(Prescale and counter cleared))
                             ;(Prescale for bus frequency / 8)
     mov     #$FF,T2MODH     ; Move decimal 255 into T2 modulo reg Hi
     mov     #$FF,T2MODL     ; Move decimal 255 into T2 modulo reg Lo
                             ;(free running timer)
     mov     #$03,T2CH0H     ; Move decimal 3 into T1CH0 O/C register Hi
     mov     #$E8,T2CH0L     ; Move decimal 232 into T1CH0 O/C register Lo
                             ;(~1000uS)=(~1.0mS)
     mov     #$54,T2SC0      ; Move %01010100 into Timer2
                             ; channel 0 status and control register
                             ; (Output compare, interrupt enabled)
     mov     #$03,T2SC       ; Move %00000011 into Timer2
                             ; Status and Control Register
                             ; Disable interrupts, counter Active
                             ; Prescale for bus frequency / 8
                             ; 8,003584hz/8=1000448hz
                             ; = .0000009995sec

;****************************************************************************
; - Enable Interrupts
;****************************************************************************

     cli              ; Clear interrupt mask ( Turn on all interrupts now )

;***************************************************************************
; ------------------------------ Initialize VFD ---------------------------
;
;  PTB4       - VFD Display Enable
;  PTB5       - VFD Display R/W
;  PTB6       - VFD Display RS
;  PTC0       - VFD Display data DB4
;  PTC1       - VFD Display data DB5
;  PTC2       - VFD Display data DB6
;  PTC3       - VFD Display data DB7
;
;***************************************************************************

;***************************************************************************
; - Clear EN, R/W, and RS
;***************************************************************************

     mov     #$00,PORTB     ; Move 0 into PortB(Clear all Port B)
                            ;("En"=0,"R/W"=0,"RS"=0)

;***************************************************************************
; - Initialize for 8 bit mode (Function Set)(do this 3 times)
;***************************************************************************

     mov     #$03,PORTC       ; Move %00000011 into PortC
                              ;(Set bit 0=DB4, and bit1=DB5)
     bset    Enable,PORTB     ; Set "Enable" bit of Port B (PTB4)("En"=1)
     jsr     LONG_DELAY       ; Jump to subroutine at LONG_DELAY:
     bclr    Enable,PORTB     ; Clear "Enable" bit of Port B (PTB4)("En"=0)
     jsr     SHORT_DELAY      ; Jump to subroutine at SHORT_DELAY:

     mov     #$03,PORTC       ; Move %00000011 into PortC
                              ;(Set bit 0=DB4, and bit1=DB5)
     bset    Enable,PORTB     ; Set "Enable" bit of Port B (PTB4)("En"=1)
     jsr     LONG_DELAY       ; Jump to subroutine at LONG_DELAY:
     bclr    Enable,PORTB     ; Clear "Enable" bit of Port B (PTB4)("En"=0)
     jsr     SHORT_DELAY      ; Jump to subroutine at SHORT_DELAY:

     mov     #$03,PORTC       ; Move %00000011 into PortC
                              ;(Set bit 0=DB4, and bit1=DB5)
     bset    Enable,PORTB     ; Set "Enable" bit of Port B (PTB4)("En"=1)
     jsr     LONG_DELAY       ; Jump to subroutine at LONG_DELAY:
     bclr    Enable,PORTB     ; Clear "Enable" bit of Port B (PTB4)("En"=0)
     jsr     SHORT_DELAY      ; Jump to subroutine at SHORT_DELAY:

;***************************************************************************
; - Set 4 bit bus mode Hi nibble (Function Set)
;***************************************************************************

     mov     #$02,PORTC       ; Move %00000010 into PortC
                              ;(Set bit1=DB5)
     bset    Enable,PORTB     ; Set "Enable" bit of Port B (PTB4)("En"=1)
     jsr     LONG_DELAY       ; Jump to subroutine at LONG_DELAY:
     bclr    Enable,PORTB     ; Clear "Enable" bit of Port B (PTB4)("En"=0)
     jsr     SHORT_DELAY      ; Jump to subroutine at SHORT_DELAY:

;***************************************************************************
; - Set 4 bit bus mode Hi nibble (Function Set)
;***************************************************************************

     mov     #$02,PORTC       ; Move %00000010 into PortC
                              ;(Set bit1=DB5)
     bset    Enable,PORTB     ; Set "Enable" bit of Port B (PTB4)("En"=1)
     jsr     SHORT_DELAY      ; Jump to subroutine at SHORT_DELAY:
     bclr    Enable,PORTB     ; Clear "Enable" bit of Port B (PTB4)("En"=0)


;***************************************************************************
; - Set 4 bit bus mode Lo nibble (Function Set)
;***************************************************************************

     mov     #$08,PORTC       ; Move %00001000 into PortC(Set bit7=DB3)
     bset    Enable,PORTB     ; Set "Enable" bit of Port B (PTB4)("En"=1)
     jsr     LONG_DELAY       ; Jump to subroutine at LONG_DELAY:
     bclr    Enable,PORTB     ; Clear "Enable" bit of Port B (PTB4)("En"=0)
     jsr     SHORT_DELAY      ; Jump to subroutine at SHORT_DELAY:

;***************************************************************************
; - Set brightness to 100% Hi nibble (Brightness Set)
;***************************************************************************

     bset    Reg_Sel,PORTB    ; Set "Reg_Sel" bit of PortB(RS=1)
     jsr     SHORT_DELAY      ; Jump to subroutine at SHORT_DELAY:
     mov     #$00,PORTC       ; Move %00000000 into PortC (Clear all Port C)
     bset    Enable,PORTB     ; Set "Enable" bit of Port B (PTB4)("En"=1)
     jsr     SHORT_DELAY      ; Jump to subroutine at SHORT_DELAY:
     bclr    Enable,PORTB     ; Clear "Enable" bit of Port B (PTB4)("En"=0)

;***************************************************************************
; - Set brightness to 100% Lo nibble (Brightness Set)
;***************************************************************************

     mov     #$00,PORTC       ; Move %00000000 into PortC (Clear all Port C)
     bset    Enable,PORTB     ; Set "Enable" bit of Port B (PTB4)("En"=1)
     jsr     LONG_DELAY       ; Jump to subroutine at LONG_DELAY:
     bclr    Enable,PORTB     ; Clear "Enable" bit of Port B (PTB4)("En"=0)
     jsr     SHORT_DELAY      ; Jump to subroutine at SHORT_DELAY:
     bclr    Reg_Sel,PORTB    ; Clear "Reg_Sel" bit of PortB(RS=0)

;***************************************************************************
; - Set display off, cursor off, blinking off Hi nibble
;   (Display On/Off control)
;***************************************************************************

     mov     #$00,PORTC       ; Move %00000000 into PortC (Clear all Port C)
     bset    Enable,PORTB     ; Set "Enable" bit of Port B (PTB4)("En"=1)
     jsr     SHORT_DELAY      ; Jump to subroutine at SHORT_DELAY:
     bclr    Enable,PORTB     ; Clear "Enable" bit of Port B (PTB4)("En"=0)

;***************************************************************************
; - Set display off, cursor off, blinking off Lo nibble
;   (Display On/Off control)
;***************************************************************************

     mov     #$08,PORTC       ; Move %00001000 into PortC(Set bit7=DB3)
     bset    Enable,PORTB     ; Set "Enable" bit of Port B (PTB4)("En"=1)
     jsr     LONG_DELAY       ; Jump to subroutine at LONG_DELAY:
     bclr    Enable,PORTB     ; Clear "Enable" bit of Port B (PTB4)("En"=0)
     jsr     SHORT_DELAY      ; Jump to subroutine at SHORT_DELAY:

;***************************************************************************
; - Set display clear Hi nibble(Display Clear)
;***************************************************************************

     mov     #$00,PORTC       ; Move %00000000 into PortC (Clear all Port C)
     bset    Enable,PORTB     ; Set "Enable" bit of Port B (PTB4)("En"=1)
     jsr     SHORT_DELAY      ; Jump to subroutine at SHORT_DELAY:
     bclr    Enable,PORTB     ; Clear "Enable" bit of Port B (PTB4)("En"=0)

;***************************************************************************
; - Set display clear Lo nibble(Display Clear)
;***************************************************************************

     mov     #$01,PORTC       ; Move %00000001 into PortC (Set bit0=DB0))
     bset    Enable,PORTB     ; Set "Enable" bit of Port B (PTB4)("En"=1)
     jsr     LONG_DELAY       ; Jump to subroutine at LONG_DELAY:
     bclr    Enable,PORTB     ; Clear "Enable" bit of Port B (PTB4)("En"=0)

;***************************************************************************
; - Delay for command to execute (min 2.3mS)
;   One pass through the primary loop takes ~1.5uS, bus frequency of ~8mHZ
;***************************************************************************

     clr     tmp2       ; Clear tmp2 variable

WAIT_6:
     clr     tmp1       ; Clear tmp1 variable

WAIT_5:
     lda     tmp1       ; Load accumulator with value in tmp1 variable
     inca               ; Increment value in accumulator
     sta     tmp1       ; Copy to tmp1 variable
     cmp     #$FF       ; Compare value in accumulator with decimal 255
     blo     WAIT_5     ; If C bit of CCR is set, (A<M), branch to WAIT_5:
     lda     tmp2       ; Load accumulator with value in tmp2 variable
     inca               ; Increment value in accumulator
     sta     tmp2       ; Copy to tmp2 variable
     cmp     #$07       ; Compare value in accumulator with decimal 7
     blo     WAIT_6     ; If C bit of CCR is set, (A<M), branch to WAIT_6:
                        ;(~2.6mS delay)

;***************************************************************************
; - Set display on, cursor off, blinking off Hi nibble
;   (Display On/Off control)
;***************************************************************************

     mov     #$00,PORTC       ; Move %00000000 into PortC (Clear all Port C)
     bset    Enable,PORTB     ; Set "Enable" bit of Port B (PTB4)("En"=1)
     jsr     SHORT_DELAY      ; Jump to subroutine at SHORT_DELAY:
     bclr    Enable,PORTB     ; Clear "Enable" bit of Port B (PTB4)("En"=0)

;***************************************************************************
; - Set display on, cursor off, blinking off Lo nibble
;   (Display On/Off control)
;***************************************************************************

     mov     #$0C,PORTC       ; Move %00001100 into PortC
                              ;(Set bit7=DB3 and bit6=DB2)
     bset    Enable,PORTB     ; Set "Enable" bit of Port B (PTB4)("En"=1)
     jsr     LONG_DELAY       ; Jump to subroutine at LONG_DELAY:
     bclr    Enable,PORTB     ; Clear "Enable" bit of Port B (PTB4)("En"=0)
     jsr     SHORT_DELAY      ; Jump to subroutine at SHORT_DELAY:

;***************************************************************************
; - Set cursor increment Hi nibble(Entry Mode Set)
;***************************************************************************

     mov     #$00,PORTC       ; Move %00000000 into PortC (Clear all Port C)
     bset    Enable,PORTB     ; Set "Enable" bit of Port B (PTB4)("En"=1)
     jsr     SHORT_DELAY      ; Jump to subroutine at SHORT_DELAY:
     bclr    Enable,PORTB     ; Clear "Enable" bit of Port B (PTB4)("En"=0)

;***************************************************************************
; - Set cursor increment Lo nibble(Entry Mode Set)
;***************************************************************************

     mov     #$06,PORTC       ; Move %00000110 into PortC
                              ;(Set bit6=DB2 and bit5=DB1)
     bset    Enable,PORTB     ; Set "Enable" bit of Port B (PTB4)("En"=1)
     jsr     LONG_DELAY       ; Jump to subroutine at LONG_DELAY:
     bclr    Enable,PORTB     ; Clear "Enable" bit of Port B (PTB4)("En"=0)
     jsr     SHORT_DELAY      ; Jump to subroutine at SHORT_DELAY:

;****************************************************************************
;****************************************************************************
;*                        M A I N  E V E N T  L O O P                       *
;****************************************************************************
;****************************************************************************

LOOPY:

;****************************************************************************
; - Check control button states
;****************************************************************************

     jsr     SW0_CHK     ; Jump to subroutine at SW0_CHK:
                         ;(Check he state of the Display Screen next
                         ; button on PTA0)
     jsr     SW1_CHK     ; Jump to subroutine at SW1_CHK:
                         ;(Check he state of the Display Screen previous
                         ; button on PTA1)

     brset   Sw1cls,Swflags,SCRL_D_RT   ; If "Sw1cls" bit of "Swflags"
                                        ; variable is set, branch to
                                        ; SCRL_D_RT
                                        ;(Scroll Right/Increment button is
                                        ; pressed)
     brset   Sw0cls,Swflags,SCRL_D_LFT  ; If "Sw0cls" bit of "Swflags"
                                        ; variable is set, branch to
                                        ; SCRL_D_LFT
                                        ;(Scroll Left/Decrement button is
                                        ; pressed)
     jmp     DSPLY_MODE                 ; Jump to DSPLY_MODE:

SCRL_D_RT:

;****************************************************************************
; - Increment the Display Screen number.
;****************************************************************************

INC_SCRNCNT:
     lda     ScrnCnt_prv     ; Load accumulator with value in "ScrnCnt_prv"
     cmp     #$12            ; Compare with decimal 18
     beq     RTN_TO_0_D      ; If Z bit of CCR is set, branch to RTN_TO_0_D
                             ;("ScrnCnt_prv" = 18 so return to screen 0)
     inc     ScrnCnt         ; Increment "ScrnCnt" variable
     bra     SCRL_D_RT_DONE  ; Branch to SCRL_D_RT_DONE:

RTN_TO_0_D:
    clr     ScrnCnt          ; Clear "ScrnCnt" variable(ScrnCnt = 0)

SCRL_D_RT_DONE:
     mov     ScrnCnt,ScrnCnt_prv  ; Move value in "ScrnCnt" to ScrnCnt_prv"
     bclr    Sw1cls,Swflags       ; Clear "Sw1cls" bit of "Swflags" variable
     jmp     DSPLY_MODE           ; Jump to DSPLY_MODE:


SCRL_D_LFT:

;****************************************************************************
; - Decrement the Display Screen number.
;****************************************************************************


DEC_SCRNCNT:
     lda     ScrnCnt_prv      ; Load accumulator with value in "ScrnCnt_prv"
     beq     RTN_TO_18        ; If Z bit of CCR is set, branch to RTN_TO_18
                              ;("ScrnCnt_prv" = 0 so return to screen 18)
     dec     ScrnCnt          ; Decrement "ScrnCnt" variable
     bra     SCRL_D_LFT_DONE  ; Branch to SCRL_D_LFT_DONE:

RTN_TO_18:
    mov     #$12,ScrnCnt      ; Move decimal 18 into "ScrnCnt" variable

SCRL_D_LFT_DONE:
     mov     ScrnCnt,ScrnCnt_prv  ; Move value in "ScrnCnt" to ScrnCnt_prv"
     bclr    Sw0cls,Swflags       ; Clear "Sw0cls" bit of "Swflags" variable


DSPLY_MODE:

;****************************************************************************
; - Using the screen count number, determine the appropriate array for the
;   top line of the display and print it.
;
; - First, compare the desired screen count number with the screen count
;   number on the last pass through the main loop. If it is the same, skip
;   over, otherwise update the top line of the display. This is to eliminate
;   "digit rattle" caused by rapid screen updates.
;****************************************************************************

TOPLIN_SUB:
     lda     ScrnCnt              ; Load accumulator with value in "ScrnCnt"
     cmp     ScrnCnt_Lst          ; Compare with "ScrnCnt_Lst"
     beq     NO_CHNG_TOP          ; If Z bit of CCR is clear, branch to
                                  ; NO_CHNG_TOP:(ScrnCnt_Lst = ScrnCnt)
     jsr     PRNT_TOPLN_DSP       ; Jump to subroutine at PRNT_TOPLN_DSP:
     mov     ScrnCnt,ScrnCnt_Lst  ; Copy value in "ScrnCnt" to ScrnCnt_Lst"

NO_CHNG_TOP:

DISP_BOT:

;****************************************************************************
; - We have 20 variables in RAM in ordered list(BotLin0 through BotLin19)
;   which have been initialized to ASCII $20(blank space). The variable
;   "DisVal" contains the offset value from the entry point of the ordered
;   list of variables, beginning at the variable "secl". "DisVal" matches
;   the variable's abbreviation on the top line on the display. Using
;   "DisVal", we do an ASCII conversion of each variable, and overwrite the
;   3 blank spaces on the bottom line beneath the matching variable
;   abreviation, with the appropriate numbers.
;****************************************************************************


;****************************************************************************
; - Load the over-write values for the bottom line string.
;****************************************************************************

     jsr     LOAD_SPACE             ; Jump to subroutine at LOAD_SPACE:

;****************************************************************************
; - Determine which screen we are in, and prepare the appropriate string for
;   the bottom line of the VFD
;****************************************************************************

     lda     ScrnCnt              ; Load accumulator with value in "ScrnCnt"
     cbeqa   #$00,SCRN_0_JMP      ; Compare and branch to SCRN_0_JMP:,
                                  ; if equal to decimal 0
     cbeqa   #$01,SCRN_1_JMP      ; Compare and branch to SCRN_1_JMP:,
                                  ; if equal to decimal 1
     cbeqa   #$02,SCRN_2_JMP      ; Compare and branch to SCRN_2_JMP:,
                                  ; if equal to decimal 2
     cbeqa   #$03,SCRN_3_JMP      ; Compare and branch to SCRN_3_JMP:,
                                  ; if equal to decimal 3
     cbeqa   #$04,SCRN_4_JMP      ; Compare and branch to SCRN_4_JMP:,
                                  ; if equal to decimal 4
     cbeqa   #$05,SCRN_5_JMP      ; Compare and branch to SCRN_5_JMP:,
                                  ; if equal to decimal 5
     cbeqa   #$06,SCRN_6_JMP      ; Compare and branch to SCRN_6_JMP:,
                                  ; if equal to decimal 6
     cbeqa   #$07,SCRN_7_JMP      ; Compare and branch to SCRN_7_JMP:,
                                  ; if equal to decimal 7
     cbeqa   #$08,SCRN_8_JMP      ; Compare and branch to SCRN_8_JMP:,
                                  ; if equal to decimal 8
     cbeqa   #$09,SCRN_9_JMP      ; Compare and branch to SCRN_9_JMP:,
                                  ; if equal to decimal 9
     cbeqa   #$0A,SCRN_10_JMP     ; Compare and branch to SCRN_10_JMP:,
                                  ; if equal to decimal 10
     cbeqa   #$0B,SCRN_11_JMP     ; Compare and branch to SCRN_11_JMP:,
                                  ; if equal to decimal 11
     cbeqa   #$0C,SCRN_12_JMP     ; Compare and branch to SCRN_12_JMP:,
                                  ; if equal to decimal 12
     cbeqa   #$0D,SCRN_13_JMP     ; Compare and branch to SCRN_13_JMP:,
                                  ; if equal to decimal 13
     cbeqa   #$0E,SCRN_14_JMP     ; Compare and branch to SCRN_14_JMP:,
                                  ; if equal to decimal 14
     cbeqa   #$0F,SCRN_15_JMP     ; Compare and branch to SCRN_15_JMP:,
                                  ; if equal to decimal 15
     cbeqa   #$10,SCRN_16_JMP     ; Compare and branch to SCRN_16_JMP:,
                                  ; if equal to decimal 16
     cbeqa   #$11,SCRN_17_JMP     ; Compare and branch to SCRN_17_JMP:,
                                  ; if equal to decimal 17
     cbeqa   #$12,SCRN_18_JMP     ; Compare and branch to SCRN_18_JMP:,
                                  ; if equal to decimal 18
     jmp     LOOPY                ; Jump to LOOPY(sanity check)


SCRN_0_JMP:
     jmp     SCRN_0         ; Jump to SCRN_0:(Long branch)

SCRN_1_JMP:
     jmp     SCRN_1         ; Jump to SCRN_1:(Long branch)

SCRN_2_JMP:
     jmp     SCRN_2         ; Jump to SCRN_2:(Long branch)

SCRN_3_JMP:
     jmp     SCRN_3         ; Jump to SCRN_3:(Long branch)

SCRN_4_JMP:
     jmp     SCRN_4         ; Jump to SCRN_4:(Long branch)

SCRN_5_JMP:
     jmp     SCRN_5         ; Jump to SCRN_5:(Long branch)

SCRN_6_JMP:
     jmp     SCRN_6         ; Jump to SCRN_6:(Long branch)

SCRN_7_JMP:
     jmp     SCRN_7         ; Jump to SCRN_7:(Long branch)

SCRN_8_JMP:
     jmp     SCRN_8         ; Jump to SCRN_8:(Long branch)

SCRN_9_JMP:
     jmp     SCRN_9         ; Jump to SCRN_9:(Long branch)

SCRN_10_JMP:
     jmp     SCRN_10        ; Jump to SCRN_10:(Long branch)

SCRN_11_JMP:
     jmp     SCRN_11        ; Jump to SCRN_11:(Long branch)

SCRN_12_JMP:
     jmp     SCRN_12        ; Jump to SCRN_12:(Long branch)

SCRN_13_JMP:
     jmp     SCRN_13        ; Jump to SCRN_13:(Long branch)

SCRN_14_JMP:
     jmp     SCRN_14        ; Jump to SCRN_14:(Long branch)

SCRN_15_JMP:
     jmp     SCRN_15        ; Jump to SCRN_15:(Long branch)

SCRN_16_JMP:
     jmp     SCRN_16        ; Jump to SCRN_16:(Long branch)

SCRN_17_JMP:
     jmp     SCRN_17        ; Jump to SCRN_17:(Long branch)

SCRN_18_JMP:
     jmp     SCRN_18        ; Jump to SCRN_18:(Long branch)



SCRN_0:
     jsr     CALC_RPM_HR       ; Jump to subroutine at CALC_RPM_HR:
     lda     rpmhrH            ; Load accumulator with value in "rpmhrH"
     sta     intacc1+2         ; Copy to "intacc1+2"
     lda     rpmhrL            ; Load accumulator with value in "rpmhrL"
     sta     intacc1+3         ; Copy to "intacc1+3"
     jsr     CONV_16BIT_ASCII  ; Jump to subroutine at CONV_16BIT_ASCII:
     lda     thousands         ; Load accumulator with value in "thousands"
     sta     Botlin0           ; Copy to "BotLin0"(1st column on left, bottom line)
     lda     hundreds          ; Load accumulator with value in "hundreds"
     sta     Botlin1           ; Copy to "BotLin1"(2nd column on left, bottom line)
     lda     tens              ; Load accumulator with value in "tens"
     sta     Botlin2           ; Copy to "BotLin4"(3d column on left, bottom line)
     lda     ones              ; Load accumulator with value in "ones"
     sta     Botlin3           ; Copy to "BotLin3"(4th column on left, bottom line)
     lda     #$20              ; Load accumulator with ASCII ' '(space)
     sta     Botlin4           ; Copy to "BotLin4"(5th column on left, bottom line)
     jsr     CALC_MAP          ; Jump to subroutine at CALC_MAP:
     lda     map               ; Load accumulator with value in "map"
     sta     DisVal            ; Copy to "DisVal" variable
     jsr     CONV_8BIT_ASCII   ; Jump to subroutine at CONV_8BIT_ASCII:
     lda     AC_100            ; Load accumulator with value in "AC_100"
     sta     Botlin5           ; Copy to "BotLin5"(6th column on left, bottom line)
     lda     AC_10             ; Load accumulator with value in "AC_10"
     sta     Botlin6           ; Copy to "BotLin6"(7th column on left, bottom line)
     lda     AC_1              ; Load accumulator with value in "AC_1"
     sta     Botlin7           ; Copy to "BotLin7"(8th column on left, bottom line)
     lda     #$20              ; Load accumulator with ASCII ' '(space)
     sta     Botlin8           ; Copy to "BotLin8"(9th column on left, bottom line)
     jsr     CALC_LAMBDA       ; Jump to subroutine at CALC_LAMBDA:
     jsr     CALC_AFR          ; Jump to subroutine at CALC_AFR:
     lda     afr               ; Load accumulator with value in "afr"
     sta     DisVal            ; Copy to "DisVal" variable
     jsr     CONV_8BIT_ASCII   ; Jump to subroutine at CONV_8BIT_ASCII:
     lda     AC_100            ; Load accumulator with value in "AC_100"
     sta     Botlin9           ; Copy to "BotLin9"(10th column on left, bottom line)
     lda     AC_10             ; Load accumulator with value in "AC_10"
     sta     Botlin10          ; Copy to "BotLin10"(11th column on left, bottom line)
     lda     AC_1              ; Load accumulator with value in "AC_1"
     sta     Botlin11          ; Copy to "BotLin11"(12th column on left, bottom line)
     lda     #$20              ; Load accumulator with ASCII ' '(space)
     sta     Botlin12          ; Copy to "BotLin12"(13th column on left, bottom line)
     jsr     CALC_CLT          ; Jump to subroutine at CALC_CLT:
     lda     clt               ; Load accumulator with value in "clt"
     sta     DisVal            ; Copy to "DisVal" variable
     jsr     CONV_8BIT_ASCII   ; Jump to subroutine at CONV_8BIT_ASCII:
     lda     AC_100            ; Load accumulator with value in "AC_100"
     sta     Botlin13          ; Copy to "BotLin13"(14th column on left, bottom line)
     lda     AC_10             ; Load accumulator with value in "AC_10"
     sta     Botlin14          ; Copy to "BotLin14"(15th column on left, bottom line)
     lda     AC_1              ; Load accumulator with value in "AC_1"
     sta     Botlin15          ; Copy to "BotLin15"(16th column on left, bottom line)
     lda     #$20              ; Load accumulator with ASCII ' '(space)
     sta     Botlin16          ; Copy to "BotLin16"(17th column on left, bottom line)
     jsr     CALC_MAT          ; Jump to subroutine at CALC_MAT:
     lda     mat               ; Load accumulator with value in "mat"
     sta     DisVal            ; Copy to "DisVal" variable
     jsr     CONV_8BIT_ASCII   ; Jump to subroutine at CONV_8BIT_ASCII:
     lda     AC_100            ; Load accumulator with value in "AC_100"
     sta     Botlin17          ; Copy to "BotLin17"(18th column on left, bottom line)
     lda     AC_10             ; Load accumulator with value in "AC_10"
     sta     Botlin18          ; Copy to "BotLin18"(19th column on left, bottom line)
     lda     AC_1              ; Load accumulator with value in "AC_1"
     sta     Botlin19          ; Copy to "BotLin19"(20th column on left, bottom line)
     jmp     CHK_FRZ_DISP      ; Jump to CHK_FRZ_DISP:

SCRN_1:
     lda     tpsp              ; Load accumulator with value in "tpsp"
     sta     DisVal            ; Copy to "DisVal" variable
     jsr     CONV_8BIT_ASCII   ; Jump to subroutine at CONV_8BIT_ASCII:
     lda     AC_100            ; Load accumulator with value in "AC_100"
     sta     Botlin0           ; Copy to "BotLin0"(1st column on left, bottom line)
     lda     AC_10             ; Load accumulator with value in "AC_10"
     sta     Botlin1           ; Copy to "BotLin1"(2nd column on left, bottom line)
     lda     AC_1              ; Load accumulator with value in "AC_1"
     sta     Botlin2           ; Copy to "BotLin2"(3d column on left, bottom line)
     lda     #$20              ; Load accumulator with ASCII ' '(space)
     sta     Botlin3           ; Copy to "BotLin3"(4th column on left, bottom line)
     jsr     CALC_OP           ; Jump to subroutine at CALC_OP:
     lda     oprs              ; Load accumulator with value in "oprs"
     sta     DisVal            ; Copy to "DisVal" variable
     jsr     CONV_8BIT_ASCII   ; Jump to subroutine at CONV_8BIT_ASCII:
     lda     AC_100            ; Load accumulator with value in "AC_100"
     sta     Botlin4           ; Copy to "BotLin4"(5th column on left, bottom line)
     lda     AC_10             ; Load accumulator with value in "AC_10"
     sta     Botlin5           ; Copy to "BotLin5"(6th column on left, bottom line)
     lda     AC_1              ; Load accumulator with value in "AC_1"
     sta     Botlin6           ; Copy to "BotLin6"(7th column on left, bottom line)
     lda     #$20              ; Load accumulator with ASCII ' '(space)
     sta     Botlin7           ; Copy to "BotLin7"(8th column on left, bottom line)
     jsr     CALC_FP           ; Jump to subroutine at CALC_FP:
     lda     fprs              ; Load accumulator with value in "fprs"
     sta     DisVal            ; Copy to "DisVal" variable
     jsr     CONV_8BIT_ASCII   ; Jump to subroutine at CONV_8BIT_ASCII:
     lda     AC_100            ; Load accumulator with value in "AC_100"
     sta     Botlin8           ; Copy to "BotLin8"(9th column on left, bottom line)
     lda     AC_10             ; Load accumulator with value in "AC_10"
     sta     Botlin9           ; Copy to "BotLin9"(10th column on left, bottom line)
     lda     AC_1              ; Load accumulator with value in "AC_1"
     sta     Botlin10          ; Copy to "BotLin10"(11th column on left, bottom line)
     lda     #$20              ; Load accumulator with ASCII ' '(space)
     sta     Botlin11          ; Copy to "BotLin11"(12th column on left, bottom line)
     jsr     CALC_EGT          ; Jump to subroutine at CALC_EGT:
     lda     egtH              ; Load accumulator with value in "egtH"
     sta     intacc1+2         ; Copy to "intacc1+2"
     lda     egtL              ; Load accumulator with value in "egtL"
     sta     intacc1+3         ; Copy to "intacc1+3"
     jsr     CONV_16BIT_ASCII  ; Jump to subroutine at CONV_16BIT_ASCII:
     lda     thousands         ; Load accumulator with value in "thousands"
     sta     Botlin12          ; Copy to "BotLin12"(13th column on left, bottom line)
     lda     hundreds          ; Load accumulator with value in "hundreds"
     sta     Botlin13          ; Copy to "BotLin13"(14th column on left, bottom line)
     lda     tens              ; Load accumulator with value in "tens"
     sta     Botlin14          ; Copy to "BotLin14"(15th column on left, bottom line)
     lda     ones              ; Load accumulator with value in "ones"
     sta     Botlin15          ; Copy to "BotLin15"(16th column on left, bottom line)
     lda     #$20              ; Load accumulator with ASCII ' '(space)
     sta     Botlin16          ; Copy to "BotLin16"(17th column on left, bottom line)
     jsr     CALC_VOLTS        ; Jump to subroutine at CALC_VOLTS:
     lda     volts             ; Load accumulator with value in "volts"
     sta     DisVal            ; Copy to "DisVal" variable
     jsr     CONV_8BIT_ASCII   ; Jump to subroutine at CONV_8BIT_ASCII:
     lda     AC_100            ; Load accumulator with value in "AC_100"
     sta     Botlin17           ; Copy to "BotLin17"(18th column on left, bottom line)
     lda     AC_10             ; Load accumulator with value in "AC_10"
     sta     Botlin18           ; Copy to "BotLin18"(19th column on left, bottom line)
     lda     AC_1              ; Load accumulator with value in "AC_1"
     sta     Botlin19          ; Copy to "BotLin19"(20th column on left, bottom line)
     jmp     CHK_FRZ_DISP      ; Jump to CHK_FRZ_DISP:

SCRN_2:
     lda     #$20              ; Load accumulator with ASCII ' '(space)
     sta     Botlin0           ; Copy to "BotLin0"(1st column on left, bottom line)
     jsr     CALC_RPM          ; Jump to subroutine at CALC_RPM:
     lda     rpmH              ; Load accumulator with value in "rpmH"
     sta     intacc1+2         ; Copy to "intacc1+2"
     lda     rpmL              ; Load accumulator with value in "rpmL"
     sta     intacc1+3         ; Copy to "intacc1+3"
     jsr     CONV_16BIT_ASCII  ; Jump to subroutine at CONV_16BIT_ASCII:
     lda     thousands         ; Load accumulator with value in "thousands"
     sta     Botlin1           ; Copy to "BotLin1"(2nd column on left, bottom line)
     lda     hundreds          ; Load accumulator with value in "hundreds"
     sta     Botlin2           ; Copy to "BotLin2"(3d column on left, bottom line)
     lda     tens              ; Load accumulator with value in "tens"
     sta     Botlin3           ; Copy to "BotLin3"(4th column on left, bottom line)
     lda     ones              ; Load accumulator with value in "ones"
     sta     Botlin4           ; Copy to "BotLin4"(5th column on left, bottom line)
     lda     #$20              ; Load accumulator with ASCII ' '(space)
     sta     Botlin5           ; Copy to "BotLin5"(6th column on left, bottom line)
     jsr     CALC_AFR          ; Jump to subroutine at CALC_AFR:
     lda     afr               ; Load accumulator with value in "afr"
     sta     DisVal            ; Copy to "DisVal" variable
     jsr     CONV_8BIT_ASCII   ; Jump to subroutine at CONV_8BIT_ASCII:
     lda     AC_100            ; Load accumulator with value in "AC_100"
     sta     Botlin6           ; Copy to "BotLin6"(7th column on left, bottom line)
     lda     AC_10             ; Load accumulator with value in "AC_10"
     sta     Botlin7           ; Copy to "BotLin7"(8th column on left, bottom line)
     lda     AC_1              ; Load accumulator with value in "AC_1"
     sta     Botlin8           ; Copy to "BotLin8"(9th column on left, bottom line)
     lda     #$20              ; Load accumulator with ASCII ' '(space)
     sta     Botlin9           ; Copy to "BotLin9"(10th column on left, bottom line)
     lda     fdSecH            ; Load accumulator with value in "fdSecH"
     sta     intacc1+2         ; Copy to "intacc1+2"
     lda     fdSecL            ; Load accumulator with value in "fdSecLL"
     sta     intacc1+3         ; Copy to "intacc1+3"
     jsr     DIV_BY_TEN        ; Jump to subroutine at DIV_BY_TEN:
     jsr     CONV_16BIT_ASCII  ; Jump to subroutine at CONV_16BIT_ASCII:
     lda     thousands         ; Load accumulator with value in "thousands"
     sta     Botlin10          ; Copy to "BotLin10"(11th column on left, bottom line)
     lda     hundreds          ; Load accumulator with value in "hundreds"
     sta     Botlin11          ; Copy to "BotLin11"(12th column on left, bottom line)
     lda     tens              ; Load accumulator with value in "tens"
     sta     Botlin12          ; Copy to "BotLin12"(13th column on left, bottom line)
     lda     ones              ; Load accumulator with value in "ones"
     sta     Botlin13          ; Copy to "BotLin13"(14th column on left, bottom line)
     lda     #$20              ; Load accumulator with ASCII ' '(space)
     sta     Botlin14          ; Copy to "BotLin14"(15th column on left, bottom line)
     jsr     CALC_L_HR         ; Jump to subroutine at CALC_L_HR:
     lda     ltrHrH            ; Load accumulator with value in "ltrHrH"
     sta     intacc1+2         ; Copy to "intacc1+2"
     lda     ltrHrL            ; Load accumulator with value in "ltrHrL"
     sta     intacc1+3         ; Copy to "intacc1+3"
     jsr     CONV_16BIT_ASCII  ; Jump to subroutine at CONV_16BIT_ASCII:
     lda     thousands         ; Load accumulator with value in "thousands"
     sta     Botlin15          ; Copy to "BotLin15"(16th column on left, bottom line)
     lda     hundreds          ; Load accumulator with value in "hundreds"
     sta     Botlin16          ; Copy to "BotLin16"(17th column on left, bottom line)
     lda     tens              ; Load accumulator with value in "tens"
     sta     Botlin17          ; Copy to "BotLin17"(18th column on left, bottom line)
     lda     ones              ; Load accumulator with value in "ones"
     sta     Botlin18          ; Copy to "BotLin18"(19th column on left, bottom line)
     lda     #$20              ; Load accumulator with ASCII ' '(space)
     sta     Botlin19          ; Copy to "BotLin19"(20th column on left, bottom line)
     jmp     CHK_FRZ_DISP      ; Jump to CHK_FRZ_DISP:

SCRN_3:
     jsr     CALC_RPM_HR       ; Jump to subroutine at CALC_RPM_HR:
     lda     rpmhrH            ; Load accumulator with value in "rpmhrH"
     sta     intacc1+2         ; Copy to "intacc1+2"
     lda     rpmhrL            ; Load accumulator with value in "rpmhrL"
     sta     intacc1+3         ; Copy to "intacc1+3"
     jsr     CONV_16BIT_ASCII  ; Jump to subroutine at CONV_16BIT_ASCII:
     lda     thousands         ; Load accumulator with value in "thousands"
     sta     Botlin0           ; Copy to "BotLin0"(1st column on left, bottom line)
     lda     hundreds          ; Load accumulator with value in "hundreds"
     sta     Botlin1           ; Copy to "BotLin1"(2nd column on left, bottom line)
     lda     tens              ; Load accumulator with value in "tens"
     sta     Botlin2           ; Copy to "BotLin4"(3d column on left, bottom line)
     lda     ones              ; Load accumulator with value in "ones"
     sta     Botlin3           ; Copy to "BotLin3"(4th column on left, bottom line)
     lda     #$20              ; Load accumulator with ASCII ' '(space)
     sta     Botlin4           ; Copy to "BotLin4"(5th column on left, bottom line)
     jsr     CALC_MAP          ; Jump to subroutine at CALC_MAP:
     lda     map               ; Load accumulator with value in "map"
     sta     DisVal            ; Copy to "DisVal" variable
     jsr     CONV_8BIT_ASCII   ; Jump to subroutine at CONV_8BIT_ASCII:
     lda     AC_100            ; Load accumulator with value in "AC_100"
     sta     Botlin5           ; Copy to "BotLin5"(6th column on left, bottom line)
     lda     AC_10             ; Load accumulator with value in "AC_10"
     sta     Botlin6           ; Copy to "BotLin6"(7th column on left, bottom line)
     lda     AC_1              ; Load accumulator with value in "AC_1"
     sta     Botlin7           ; Copy to "BotLin7"(8th column on left, bottom line)
     lda     #$20              ; Load accumulator with ASCII ' '(space)
     sta     Botlin8           ; Copy to "BotLin8"(9th column on left, bottom line)
     jsr     CALC_AFR          ; Jump to subroutine at CALC_AFR:
     lda     afr               ; Load accumulator with value in "afr"
     sta     DisVal            ; Copy to "DisVal" variable
     jsr     CONV_8BIT_ASCII   ; Jump to subroutine at CONV_8BIT_ASCII:
     lda     AC_100            ; Load accumulator with value in "AC_100"
     sta     Botlin9           ; Copy to "BotLin9"(10th column on left, bottom line)
     lda     AC_10             ; Load accumulator with value in "AC_10"
     sta     Botlin10          ; Copy to "BotLin10"(11th column on left, bottom line)
     lda     AC_1              ; Load accumulator with value in "AC_1"
     sta     Botlin11          ; Copy to "BotLin11"(12th column on left, bottom line)
     lda     #$20              ; Load accumulator with ASCII ' '(space)
     sta     Botlin12          ; Copy to "BotLin12"(13th column on left, bottom line)
     lda     Ftrimcor          ; Load accumulator with value in "Ftrimcor"
     sta     DisVal            ; Copy to "DisVal" variable
     jsr     CONV_8BIT_ASCII   ; Jump to subroutine at CONV_8BIT_ASCII:
     lda     AC_100            ; Load accumulator with value in "AC_100"
     sta     Botlin13          ; Copy to "BotLin13"(14th column on left, bottom line)
     lda     AC_10             ; Load accumulator with value in "AC_10"
     sta     Botlin14          ; Copy to "BotLin14"(15th column on left, bottom line)
     lda     AC_1              ; Load accumulator with value in "AC_1"
     sta     Botlin15          ; Copy to "BotLin15"(16th column on left, bottom line)
     lda     #$20              ; Load accumulator with ASCII ' '(space)
     sta     Botlin16          ; Copy to "BotLin16"(17th column on left, bottom line)
     lda     pw                ; Load accumulator with value in "pw"
     sta     DisVal            ; Copy to "DisVal" variable
     jsr     CONV_8BIT_ASCII   ; Jump to subroutine at CONV_8BIT_ASCII:
     lda     AC_100            ; Load accumulator with value in "AC_100"
     sta     Botlin17          ; Copy to "BotLin17"(18th column on left, bottom line)
     lda     AC_10             ; Load accumulator with value in "AC_10"
     sta     Botlin18          ; Copy to "BotLin18"(19th column on left, bottom line)
     lda     AC_1              ; Load accumulator with value in "AC_1"
     sta     Botlin19          ; Copy to "BotLin19"(20th column on left, bottom line)
     jmp     CHK_FRZ_DISP      ; Jump to CHK_FRZ_DISP:

SCRN_4:
     jsr     CALC_RPM_HR       ; Jump to subroutine at CALC_RPM_HR:
     lda     rpmhrH            ; Load accumulator with value in "rpmhrH"
     sta     intacc1+2         ; Copy to "intacc1+2"
     lda     rpmhrL            ; Load accumulator with value in "rpmhrL"
     sta     intacc1+3         ; Copy to "intacc1+3"
     jsr     CONV_16BIT_ASCII  ; Jump to subroutine at CONV_16BIT_ASCII:
     lda     thousands         ; Load accumulator with value in "thousands"
     sta     Botlin0           ; Copy to "BotLin0"(1st column on left, bottom line)
     lda     hundreds          ; Load accumulator with value in "hundreds"
     sta     Botlin1           ; Copy to "BotLin1"(2nd column on left, bottom line)
     lda     tens              ; Load accumulator with value in "tens"
     sta     Botlin2           ; Copy to "BotLin4"(3d column on left, bottom line)
     lda     ones              ; Load accumulator with value in "ones"
     sta     Botlin3           ; Copy to "BotLin3"(4th column on left, bottom line)
     lda     #$20              ; Load accumulator with ASCII ' '(space)
     sta     Botlin4           ; Copy to "BotLin4"(5th column on left, bottom line)
     jsr     CALC_MAP          ; Jump to subroutine at CALC_MAP:
     lda     map               ; Load accumulator with value in "map"
     sta     DisVal            ; Copy to "DisVal" variable
     jsr     CONV_8BIT_ASCII   ; Jump to subroutine at CONV_8BIT_ASCII:
     lda     AC_100            ; Load accumulator with value in "AC_100"
     sta     Botlin5           ; Copy to "BotLin5"(6th column on left, bottom line)
     lda     AC_10             ; Load accumulator with value in "AC_10"
     sta     Botlin6           ; Copy to "BotLin6"(7th column on left, bottom line)
     lda     AC_1              ; Load accumulator with value in "AC_1"
     sta     Botlin7           ; Copy to "BotLin7"(8th column on left, bottom line)
     lda     #$20              ; Load accumulator with ASCII ' '(space)
     sta     Botlin8           ; Copy to "BotLin8"(9th column on left, bottom line)
     jsr     CALC_TRIM_ANG     ; Jump to subroutine at CALC_TRIM_ANG:
     lda     trmAng            ; Load accumulator with value in "trmAng"
     sta     DisVal            ; Copy to "DisVal" variable
     jsr     CONV_8BIT_ASCII   ; Jump to subroutine at CONV_8BIT_ASCII:
     lda     AC_100            ; Load accumulator with value in "AC_100"
     sta     Botlin9           ; Copy to "BotLin9"(10th column on left, bottom line)
     lda     AC_10             ; Load accumulator with value in "AC_10"
     sta     Botlin10          ; Copy to "BotLin10"(11th column on left, bottom line)
     lda     AC_1              ; Load accumulator with value in "AC_1"
     sta     Botlin11          ; Copy to "BotLin11"(12th column on left, bottom line)
     lda     #$20              ; Load accumulator with ASCII ' '(space)
     sta     Botlin12          ; Copy to "BotLin12"(13th column on left, bottom line)
     jsr     CALC_DLY_ANG      ; Jump to subroutine at CALC_DELA_ANG:
     jsr     CALC_TIM_ANG      ; Jump to subroutine at CALC_TIM_ANG:
     lda     timAng            ; Load accumulator with value in "timAng"
     sta     DisVal            ; Copy to "DisVal" variable
     jsr     CONV_8BIT_ASCII   ; Jump to subroutine at CONV_8BIT_ASCII:
     lda     AC_100            ; Load accumulator with value in "AC_100"
     sta     Botlin13          ; Copy to "BotLin13"(14th column on left, bottom line)
     lda     AC_10             ; Load accumulator with value in "AC_10"
     sta     Botlin14          ; Copy to "BotLin14"(15th column on left, bottom line)
     lda     AC_1              ; Load accumulator with value in "AC_1"
     sta     Botlin15          ; Copy to "BotLin15"(16th column on left, bottom line)
     lda     #$20              ; Load accumulator with ASCII ' '(space)
     sta     Botlin16          ; Copy to "BotLin16"(17th column on left, bottom line)
     jsr     CALC_MON_TIM_ANG  ; Jump to subroutine at CALC_MON_TIM_ANG:
     lda     monTimAng         ; Load accumulator with value in "monTimAng"
     sta     DisVal            ; Copy to "DisVal" variable
     jsr     CONV_8BIT_ASCII   ; Jump to subroutine at CONV_8BIT_ASCII:
     lda     AC_100            ; Load accumulator with value in "AC_100"
     sta     Botlin17          ; Copy to "BotLin17"(18th column on left, bottom line)
     lda     AC_10             ; Load accumulator with value in "AC_10"
     sta     Botlin18          ; Copy to "BotLin18"(19th column on left, bottom line)
     lda     AC_1              ; Load accumulator with value in "AC_1"
     sta     Botlin19          ; Copy to "BotLin19"(20th column on left, bottom line)
     jmp     CHK_FRZ_DISP      ; Jump to CHK_FRZ_DISP:

SCRN_5:
     lda     #$20              ; Load accumulator with ASCII ' '(space)
     sta     Botlin0           ; Copy to "BotLin0"(1st column on left, bottom line)
     lda     gammae            ; Load accumulator with value in "gammae"
     sta     DisVal            ; Copy to "DisVal" variable
     jsr     CONV_8BIT_ASCII   ; Jump to subroutine at CONV_8BIT_ASCII:
     lda     AC_100            ; Load accumulator with value in "AC_100"
     sta     Botlin1           ; Copy to "BotLin1"(2nd column on left, bottom line)
     lda     AC_10             ; Load accumulator with value in "AC_10"
     sta     Botlin2           ; Copy to "BotLin2"(3d column on left, bottom line)
     lda     AC_1              ; Load accumulator with value in "AC_1"
     sta     Botlin3           ; Copy to "BotLin3"(4th column on left, bottom line)
     lda     #$20              ; Load accumulator with ASCII ' '(space)
     sta     Botlin4           ; Copy to "BotLin4"(5th column on left, bottom line)
     lda     warmcor            ; Load accumulator with value in "warmcor"
     sta     DisVal            ; Copy to "DisVal" variable
     jsr     CONV_8BIT_ASCII   ; Jump to subroutine at CONV_8BIT_ASCII:
     lda     AC_100            ; Load accumulator with value in "AC_100"
     sta     Botlin5           ; Copy to "BotLin5"(6th column on left, bottom line)
     lda     AC_10             ; Load accumulator with value in "AC_10"
     sta     Botlin6           ; Copy to "BotLin6"(7th column on left, bottom line)
     lda     AC_1              ; Load accumulator with value in "AC_1"
     sta     Botlin7           ; Copy to "BotLin7"(8th column on left, bottom line)
     lda     #$20              ; Load accumulator with ASCII ' '(space)
     sta     Botlin8           ; Copy to "BotLin8"(9th column on left, bottom line)
     lda     aircor            ; Load accumulator with value in "aircor"
     sta     DisVal            ; Copy to "DisVal" variable
     jsr     CONV_8BIT_ASCII   ; Jump to subroutine at CONV_8BIT_ASCII:
     lda     AC_100            ; Load accumulator with value in "AC_100"
     sta     Botlin9           ; Copy to "BotLin9"(10th column on left, bottom line)
     lda     AC_10             ; Load accumulator with value in "AC_10"
     sta     Botlin10          ; Copy to "BotLin10"(11th column on left, bottom line)
     lda     AC_1              ; Load accumulator with value in "AC_1"
     sta     Botlin11          ; Copy to "BotLin11"(12th column on left, bottom line)
     lda     #$20              ; Load accumulator with ASCII ' '(space)
     sta     Botlin12          ; Copy to "BotLin12"(13th column on left, bottom line)
     lda     barocor           ; Load accumulator with value in "barocor"
     sta     DisVal            ; Copy to "DisVal" variable
     jsr     CONV_8BIT_ASCII   ; Jump to subroutine at CONV_8BIT_ASCII:
     lda     AC_100            ; Load accumulator with value in "AC_100"
     sta     Botlin13          ; Copy to "BotLin13"(14th column on left, bottom line)
     lda     AC_10             ; Load accumulator with value in "AC_10"
     sta     Botlin14          ; Copy to "BotLin14"(15th column on left, bottom line)
     lda     AC_1              ; Load accumulator with value in "AC_1"
     sta     Botlin15          ; Copy to "BotLin15"(16th column on left, bottom line)
     lda     #$20              ; Load accumulator with ASCII ' '(space)
     sta     Botlin16          ; Copy to "BotLin16"(17th column on left, bottom line)
     lda     tpsaccel          ; Load accumulator with value in "tpsaccel"
     sta     DisVal            ; Copy to "DisVal" variable
     jsr     CONV_8BIT_ASCII   ; Jump to subroutine at CONV_8BIT_ASCII:
     lda     AC_100            ; Load accumulator with value in "AC_100"
     sta     Botlin17          ; Copy to "BotLin17"(18th column on left, bottom line)
     lda     AC_10             ; Load accumulator with value in "AC_10"
     sta     Botlin18          ; Copy to "BotLin18"(19th column on left, bottom line)
     lda     AC_1              ; Load accumulator with value in "AC_1"
     sta     Botlin19          ; Copy to "BotLin19"(20th column on left, bottom line)
     jmp     CHK_FRZ_DISP      ; Jump to CHK_FRZ_DISP:

SCRN_6:
     lda     #$20              ; Load accumulator with ASCII ' '(space)
     sta     Botlin0           ; Copy to "BotLin0"(1st column on left, bottom line)
     lda     egoADC            ; Load accumulator with value in "egoADC"
     sta     DisVal            ; Copy to "DisVal" variable
     jsr     CONV_8BIT_ASCII   ; Jump to subroutine at CONV_8BIT_ASCII:
     lda     AC_100            ; Load accumulator with value in "AC_100"
     sta     Botlin1           ; Copy to "BotLin1"(2nd column on left, bottom line)
     lda     AC_10             ; Load accumulator with value in "AC_10"
     sta     Botlin2           ; Copy to "BotLin2"(3d column on left, bottom line)
     lda     AC_1              ; Load accumulator with value in "AC_1"
     sta     Botlin3           ; Copy to "BotLin3"(4th column on left, bottom line)
     lda     #$20              ; Load accumulator with ASCII ' '(space)
     sta     Botlin4           ; Copy to "BotLin4"(5th column on left, bottom line)
     jsr     CALC_EGOV         ; Jump to subroutine at CALC_EGOV:
     lda     egoV              ; Load accumulator with value in "egoV"
     sta     DisVal            ; Copy to "DisVal" variable
     jsr     CONV_8BIT_ASCII   ; Jump to subroutine at CONV_8BIT_ASCII:
     lda     AC_100            ; Load accumulator with value in "AC_100"
     sta     Botlin5           ; Copy to "BotLin5"(6th column on left, bottom line)
     lda     AC_10             ; Load accumulator with value in "AC_10"
     sta     Botlin6           ; Copy to "BotLin6"(7th column on left, bottom line)
     lda     AC_1              ; Load accumulator with value in "AC_1"
     sta     Botlin7           ; Copy to "BotLin7"(8th column on left, bottom line)
     lda     #$20              ; Load accumulator with ASCII ' '(space)
     sta     Botlin8           ; Copy to "BotLin8"(9th column on left, bottom line)
     jsr     CALC_AFR          ; Jump to subroutine at CALC_AFR:
     jsr     CALC_LAMBDA       ; Jump to subroutine at CALC_LAMBDA:
     lda     lambda            ; Load accumulator with value in "lambda"
     sta     DisVal            ; Copy to "DisVal" variable
     jsr     CONV_8BIT_ASCII   ; Jump to subroutine at CONV_8BIT_ASCII:
     lda     AC_100            ; Load accumulator with value in "AC_100"
     sta     Botlin9           ; Copy to "BotLin9"(10th column on left, bottom line)
     lda     AC_10             ; Load accumulator with value in "AC_10"
     sta     Botlin10          ; Copy to "BotLin10"(11th column on left, bottom line)
     lda     AC_1              ; Load accumulator with value in "AC_1"
     sta     Botlin11          ; Copy to "BotLin11"(12th column on left, bottom line)
     lda     #$20              ; Load accumulator with ASCII ' '(space)
     sta     Botlin12          ; Copy to "BotLin12"(13th column on left, bottom line)
     jsr     CALC_AFR          ; Jump to subroutine at CALC_AFR:
     lda     afr               ; Load accumulator with value in "afr"
     sta     DisVal            ; Copy to "DisVal" variable
     jsr     CONV_8BIT_ASCII   ; Jump to subroutine at CONV_8BIT_ASCII:
     lda     AC_100            ; Load accumulator with value in "AC_100"
     sta     Botlin13          ; Copy to "BotLin13"(14th column on left, bottom line)
     lda     AC_10             ; Load accumulator with value in "AC_10"
     sta     Botlin14          ; Copy to "BotLin14"(15th column on left, bottom line)
     lda     AC_1              ; Load accumulator with value in "AC_1"
     sta     Botlin15          ; Copy to "BotLin15"(16th column on left, bottom line)
     lda     #$20              ; Load accumulator with ASCII ' '(space)
     sta     Botlin16          ; Copy to "BotLin16"(17th column on left, bottom line)
     lda     Ftrimcor          ; Load accumulator with value in "Ftrimcor"
     sta     DisVal            ; Copy to "DisVal" variable
     jsr     CONV_8BIT_ASCII   ; Jump to subroutine at CONV_8BIT_ASCII:
     lda     AC_100            ; Load accumulator with value in "AC_100"
     sta     Botlin17          ; Copy to "BotLin17"(18th column on left, bottom line)
     lda     AC_10             ; Load accumulator with value in "AC_10"
     sta     Botlin18          ; Copy to "BotLin18"(19th column on left, bottom line)
     lda     AC_1              ; Load accumulator with value in "AC_1"
     sta     Botlin19          ; Copy to "BotLin19"(20th column on left, bottom line)
     jmp     CHK_FRZ_DISP      ; Jump to CHK_FRZ_DISP:

SCRN_7:
     lda     #$20              ; Load accumulator with ASCII ' '(space)
     sta     Botlin0           ; Copy to "BotLin0"(1st column on left, bottom line)
     lda     mapADC            ; Load accumulator with value in "mapADC"
     sta     DisVal            ; Copy to "DisVal" variable
     jsr     CONV_8BIT_ASCII   ; Jump to subroutine at CONV_8BIT_ASCII:
     lda     AC_100            ; Load accumulator with value in "AC_100"
     sta     Botlin1           ; Copy to "BotLin1"(2nd column on left, bottom line)
     lda     AC_10             ; Load accumulator with value in "AC_10"
     sta     Botlin2           ; Copy to "BotLin2"(3d column on left, bottom line)
     lda     AC_1              ; Load accumulator with value in "AC_1"
     sta     Botlin3           ; Copy to "BotLin3"(4th column on left, bottom line)
     lda     #$20              ; Load accumulator with ASCII ' '(space)
     sta     Botlin4           ; Copy to "BotLin4"(5th column on left, bottom line)
     jsr     CALC_MAP          ; Jump to subroutine at CALC_MAP:
     lda     map               ; Load accumulator with value in "map"
     sta     DisVal            ; Copy to "DisVal" variable
     jsr     CONV_8BIT_ASCII   ; Jump to subroutine at CONV_8BIT_ASCII:
     lda     AC_100            ; Load accumulator with value in "AC_100"
     sta     Botlin5           ; Copy to "BotLin5"(6th column on left, bottom line)
     lda     AC_10             ; Load accumulator with value in "AC_10"
     sta     Botlin6           ; Copy to "BotLin6"(7th column on left, bottom line)
     lda     AC_1              ; Load accumulator with value in "AC_1"
     sta     Botlin7           ; Copy to "BotLin7"(8th column on left, bottom line)
     lda     #$20              ; Load accumulator with ASCII ' '(space)
     sta     Botlin8           ; Copy to "BotLin8"(9th column on left, bottom line)
     sta     Botlin9           ; Copy to "BotLin9"(10th column on left, bottom line)
     sta     Botlin10          ; Copy to "BotLin10"(11th column on left, bottom line)
     sta     Botlin11          ; Copy to "BotLin11"(12th column on left, bottom line)
     sta     Botlin12          ; Copy to "BotLin12"(13th column on left, bottom line)
     lda     barometer         ; Load accumulator with value in "barometer"
     sta     DisVal            ; Copy to "DisVal" variable
     jsr     CONV_8BIT_ASCII   ; Jump to subroutine at CONV_8BIT_ASCII:
     lda     AC_100            ; Load accumulator with value in "AC_100"
     sta     Botlin13          ; Copy to "BotLin13"(14th column on left, bottom line)
     lda     AC_10             ; Load accumulator with value in "AC_10"
     sta     Botlin14          ; Copy to "BotLin14"(15th column on left, bottom line)
     lda     AC_1              ; Load accumulator with value in "AC_1"
     sta     Botlin15          ; Copy to "BotLin15"(16th column on left, bottom line)
     lda     #$20              ; Load accumulator with ASCII ' '(space)
     sta     Botlin16          ; Copy to "BotLin16"(17th column on left, bottom line)
     lda     barocor           ; Load accumulator with value in "barocor"
     sta     DisVal            ; Copy to "DisVal" variable
     jsr     CONV_8BIT_ASCII   ; Jump to subroutine at CONV_8BIT_ASCII:
     lda     AC_100            ; Load accumulator with value in "AC_100"
     sta     Botlin17          ; Copy to "BotLin17"(18th column on left, bottom line)
     lda     AC_10             ; Load accumulator with value in "AC_10"
     sta     Botlin18          ; Copy to "BotLin18"(19th column on left, bottom line)
     lda     AC_1              ; Load accumulator with value in "AC_1"
     sta     Botlin19          ; Copy to "BotLin19"(20th column on left, bottom line)
     jmp     CHK_FRZ_DISP      ; Jump to CHK_FRZ_DISP:

SCRN_8:
     lda     #$20              ; Load accumulator with ASCII ' '(space)
     sta     Botlin0           ; Copy to "BotLin0"(1st column on left, bottom line)
     lda     battADC           ; Load accumulator with value in "battADC"
     sta     DisVal            ; Copy to "DisVal" variable
     jsr     CONV_8BIT_ASCII   ; Jump to subroutine at CONV_8BIT_ASCII:
     lda     AC_100            ; Load accumulator with value in "AC_100"
     sta     Botlin1           ; Copy to "BotLin1"(2nd column on left, bottom line)
     lda     AC_10             ; Load accumulator with value in "AC_10"
     sta     Botlin2           ; Copy to "BotLin2"(3d column on left, bottom line)
     lda     AC_1              ; Load accumulator with value in "AC_1"
     sta     Botlin3           ; Copy to "BotLin3"(4th column on left, bottom line)
     lda     #$20              ; Load accumulator with ASCII ' '(space)
     sta     Botlin4           ; Copy to "BotLin4"(5th column on left, bottom line)
     jsr     CALC_VOLTS        ; Jump to subroutine at CALC_VOLTS:
     lda     volts             ; Load accumulator with value in "volts"
     sta     DisVal            ; Copy to "DisVal" variable
     jsr     CONV_8BIT_ASCII   ; Jump to subroutine at CONV_8BIT_ASCII:
     lda     AC_100            ; Load accumulator with value in "AC_100"
     sta     Botlin5           ; Copy to "BotLin5"(6th column on left, bottom line)
     lda     AC_10             ; Load accumulator with value in "AC_10"
     sta     Botlin6           ; Copy to "BotLin6"(7th column on left, bottom line)
     lda     AC_1              ; Load accumulator with value in "AC_1"
     sta     Botlin7           ; Copy to "BotLin7"(8th column on left, bottom line)
     lda     #$20              ; Load accumulator with ASCII ' '(space)
     sta     Botlin8           ; Copy to "BotLin8"(9th column on left, bottom line)
     lda     cltADC            ; Load accumulator with value in "cltADC"
     sta     DisVal            ; Copy to "DisVal" variable
     jsr     CONV_8BIT_ASCII   ; Jump to subroutine at CONV_8BIT_ASCII:
     lda     AC_100            ; Load accumulator with value in "AC_100"
     sta     Botlin9           ; Copy to "BotLin9"(10th column on left, bottom line)
     lda     AC_10             ; Load accumulator with value in "AC_10"
     sta     Botlin10          ; Copy to "BotLin10"(11th column on left, bottom line)
     lda     AC_1              ; Load accumulator with value in "AC_1"
     sta     Botlin11          ; Copy to "BotLin11"(12th column on left, bottom line)
     lda     #$20              ; Load accumulator with ASCII ' '(space)
     sta     Botlin12          ; Copy to "BotLin12"(13th column on left, bottom line)
     jsr     CALC_CLT          ; Jump to subroutine at CALC_CLT:
     lda     clt               ; Load accumulator with value in "clt"
     sta     DisVal            ; Copy to "DisVal" variable
     jsr     CONV_8BIT_ASCII   ; Jump to subroutine at CONV_8BIT_ASCII:
     lda     AC_100            ; Load accumulator with value in "AC_100"
     sta     Botlin13          ; Copy to "BotLin13"(14th column on left, bottom line)
     lda     AC_10             ; Load accumulator with value in "AC_10"
     sta     Botlin14          ; Copy to "BotLin14"(15th column on left, bottom line)
     lda     AC_1              ; Load accumulator with value in "AC_1"
     sta     Botlin15          ; Copy to "BotLin15"(16th column on left, bottom line)
     lda     #$20              ; Load accumulator with ASCII ' '(space)
     sta     Botlin16          ; Copy to "BotLin16"(17th column on left, bottom line)
     lda     warmcor           ; Load accumulator with value in "warmcor"
     sta     DisVal            ; Copy to "DisVal" variable
     jsr     CONV_8BIT_ASCII   ; Jump to subroutine at CONV_8BIT_ASCII:
     lda     AC_100            ; Load accumulator with value in "AC_100"
     sta     Botlin17          ; Copy to "BotLin17"(18th column on left, bottom line)
     lda     AC_10             ; Load accumulator with value in "AC_10"
     sta     Botlin18          ; Copy to "BotLin18"(19th column on left, bottom line)
     lda     AC_1              ; Load accumulator with value in "AC_1"
     sta     Botlin19          ; Copy to "BotLin19"(20th column on left, bottom line)
     jmp     CHK_FRZ_DISP      ; Jump to CHK_FRZ_DISP:

SCRN_9:
     lda     egtADC            ; Load accumulator with value in "egtADC"
     sta     DisVal            ; Copy to "DisVal" variable
     jsr     CONV_8BIT_ASCII   ; Jump to subroutine at CONV_8BIT_ASCII:
     lda     AC_100            ; Load accumulator with value in "AC_100"
     sta     Botlin0           ; Copy to "BotLin0"(1st column on left, bottom line)
     lda     AC_10             ; Load accumulator with value in "AC_10"
     sta     Botlin1           ; Copy to "BotLin1"(2nd column on left, bottom line)
     lda     AC_1              ; Load accumulator with value in "AC_1"
     sta     Botlin2           ; Copy to "BotLin2"(3d column on left, bottom line)
     lda     #$20              ; Load accumulator with ASCII ' '(space)
     sta     Botlin3           ; Copy to "BotLin3"(4th column on left, bottom line)
     jsr     CALC_EGT          ; Jump to subroutine at CALC_EGT:
     lda     egtH              ; Load accumulator with value in "egtH"
     sta     intacc1+2         ; Copy to "intacc1+2"
     lda     egtL              ; Load accumulator with value in "egtL"
     sta     intacc1+3         ; Copy to "intacc1+3"
     jsr     CONV_16BIT_ASCII  ; Jump to subroutine at CONV_16BIT_ASCII:
     lda     thousands         ; Load accumulator with value in "thousands"
     sta     Botlin4           ; Copy to "BotLin4"(5th column on left, bottom line)
     lda     hundreds          ; Load accumulator with value in "hundreds"
     sta     Botlin5           ; Copy to "BotLin5"(6th column on left, bottom line)
     lda     tens              ; Load accumulator with value in "tens"
     sta     Botlin6           ; Copy to "BotLin6"(7th column on left, bottom line)
     lda     ones              ; Load accumulator with value in "ones"
     sta     Botlin7           ; Copy to "BotLin7"(8th column on left, bottom line)
     lda     #$20              ; Load accumulator with ASCII ' '(space)
     sta     Botlin8           ; Copy to "BotLin8"(9th column on left, bottom line)
     lda     matADC            ; Load accumulator with value in "matADC"
     sta     DisVal            ; Copy to "DisVal" variable
     jsr     CONV_8BIT_ASCII   ; Jump to subroutine at CONV_8BIT_ASCII:
     lda     AC_100            ; Load accumulator with value in "AC_100"
     sta     Botlin9           ; Copy to "BotLin9"(10th column on left, bottom line)
     lda     AC_10             ; Load accumulator with value in "AC_10"
     sta     Botlin10          ; Copy to "BotLin10"(11th column on left, bottom line)
     lda     AC_1              ; Load accumulator with value in "AC_1"
     sta     Botlin11          ; Copy to "BotLin11"(12th column on left, bottom line)
     lda     #$20              ; Load accumulator with ASCII ' '(space)
     sta     Botlin12          ; Copy to "BotLin12"(13th column on left, bottom line)
     jsr     CALC_MAT          ; Jump to subroutine at CALC_MAT:
     lda     mat               ; Load accumulator with value in "mat"
     sta     DisVal            ; Copy to "DisVal" variable
     jsr     CONV_8BIT_ASCII   ; Jump to subroutine at CONV_8BIT_ASCII:
     lda     AC_100            ; Load accumulator with value in "AC_100"
     sta     Botlin13          ; Copy to "BotLin13"(14th column on left, bottom line)
     lda     AC_10             ; Load accumulator with value in "AC_10"
     sta     Botlin14          ; Copy to "BotLin14"(15th column on left, bottom line)
     lda     AC_1              ; Load accumulator with value in "AC_1"
     sta     Botlin15          ; Copy to "BotLin15"(16th column on left, bottom line)
     lda     #$20              ; Load accumulator with ASCII ' '(space)
     sta     Botlin16          ; Copy to "BotLin16"(17th column on left, bottom line)
     lda     aircor            ; Load accumulator with value in "aircor"
     sta     DisVal            ; Copy to "DisVal" variable
     jsr     CONV_8BIT_ASCII   ; Jump to subroutine at CONV_8BIT_ASCII:
     lda     AC_100            ; Load accumulator with value in "AC_100"
     sta     Botlin17          ; Copy to "BotLin17"(18th column on left, bottom line)
     lda     AC_10             ; Load accumulator with value in "AC_10"
     sta     Botlin18          ; Copy to "BotLin18"(19th column on left, bottom line)
     lda     AC_1              ; Load accumulator with value in "AC_1"
     sta     Botlin19          ; Copy to "BotLin19"(20th column on left, bottom line)
     jmp     CHK_FRZ_DISP      ; Jump to CHK_FRZ_DISP:

SCRN_10:
     lda     #$20              ; Load accumulator with ASCII ' '(space)
     sta     Botlin0           ; Copy to "BotLin0"(1st column on left, bottom line)
     lda     ftrmADC           ; Load accumulator with value in "ftrimADC"
     sta     DisVal            ; Copy to "DisVal" variable
     jsr     CONV_8BIT_ASCII   ; Jump to subroutine at CONV_8BIT_ASCII:
     lda     AC_100            ; Load accumulator with value in "AC_100"
     sta     Botlin1           ; Copy to "BotLin1"(2nd column on left, bottom line)
     lda     AC_10             ; Load accumulator with value in "AC_10"
     sta     Botlin2           ; Copy to "BotLin2"(3d column on left, bottom line)
     lda     AC_1              ; Load accumulator with value in "AC_1"
     sta     Botlin3           ; Copy to "BotLin3"(4th column on left, bottom line)
     lda     #$20              ; Load accumulator with ASCII ' '(space)
     sta     Botlin4           ; Copy to "BotLin4"(5th column on left, bottom line)
     sta     Botlin5           ; Copy to "BotLin5"(6th column on left, bottom line)
     lda     Ftrimcor          ; Load accumulator with value in "Ftrimcor"
     sta     DisVal            ; Copy to "DisVal" variable
     jsr     CONV_8BIT_ASCII   ; Jump to subroutine at CONV_8BIT_ASCII:
     lda     AC_100            ; Load accumulator with value in "AC_100"
     sta     Botlin6           ; Copy to "BotLin6"(7th column on left, bottom line)
     lda     AC_10             ; Load accumulator with value in "AC_10"
     sta     Botlin7           ; Copy to "BotLin7"(8th column on left, bottom line)
     lda     AC_1              ; Load accumulator with value in "AC_1"
     sta     Botlin8           ; Copy to "BotLin8"(9th column on left, bottom line)
     lda     #$20              ; Load accumulator with ASCII ' '(space)
     sta     Botlin9           ; Copy to "BotLin9"(10th column on left, bottom line)
     sta     Botlin10          ; Copy to "BotLin10"(11th column on left, bottom line)
     lda     itrmADC           ; Load accumulator with value in "itrmADC"
     sta     DisVal            ; Copy to "DisVal" variable
     jsr     CONV_8BIT_ASCII   ; Jump to subroutine at CONV_8BIT_ASCII:
     lda     AC_100            ; Load accumulator with value in "AC_100"
     sta     Botlin11          ; Copy to "BotLin11"(12th column on left, bottom line)
     lda     AC_10             ; Load accumulator with value in "AC_10"
     sta     Botlin12          ; Copy to "BotLin12"(13th column on left, bottom line)
     lda     AC_1              ; Load accumulator with value in "AC_1"
     sta     Botlin13          ; Copy to "BotLin13"(14th column on left, bottom line)
     lda     #$20              ; Load accumulator with ASCII ' '(space)
     sta     Botlin14          ; Copy to "BotLin14"(15th column on left, bottom line)
     sta     Botlin15          ; Copy to "BotLin15"(16th column on left, bottom line)
     lda     trmAngFac         ; Load accumulator with value in "trmAngFac"
     sta     DisVal            ; Copy to "DisVal" variable
     jsr     CONV_8BIT_ASCII   ; Jump to subroutine at CONV_8BIT_ASCII:
     lda     AC_100            ; Load accumulator with value in "AC_100"
     sta     Botlin16          ; Copy to "BotLin16"(17th column on left, bottom line)
     lda     AC_10             ; Load accumulator with value in "AC_10"
     sta     Botlin17          ; Copy to "BotLin17"(18th column on left, bottom line)
     lda     AC_1              ; Load accumulator with value in "AC_1"
     sta     Botlin18          ; Copy to "BotLin18"(19th column on left, bottom line)
     lda     #$20              ; Load accumulator with ASCII ' '(space)
     sta     Botlin19          ; Copy to "BotLin19"(20th column on left, bottom line)
     jmp     CHK_FRZ_DISP      ; Jump to CHK_FRZ_DISP:

SCRN_11:
     lda     #$20              ; Load accumulator with ASCII ' '(space)
     sta     Botlin0           ; Copy to "BotLin0"(1st column on left, bottom line)
     lda     opADC             ; Load accumulator with value in "opADC"
     sta     DisVal            ; Copy to "DisVal" variable
     jsr     CONV_8BIT_ASCII   ; Jump to subroutine at CONV_8BIT_ASCII:
     lda     AC_100            ; Load accumulator with value in "AC_100"
     sta     Botlin1           ; Copy to "BotLin1"(2nd column on left, bottom line)
     lda     AC_10             ; Load accumulator with value in "AC_10"
     sta     Botlin2           ; Copy to "BotLin2"(3d column on left, bottom line)
     lda     AC_1              ; Load accumulator with value in "AC_1"
     sta     Botlin3           ; Copy to "BotLin3"(4th column on left, bottom line)
     lda     #$20              ; Load accumulator with ASCII ' '(space)
     sta     Botlin4           ; Copy to "BotLin4"(5th column on left, bottom line)
     sta     Botlin5           ; Copy to "BotLin5"(6th column on left, bottom line)
     jsr     CALC_OP           ; Jump to subroutine at CALC_OP:
     lda     oprs              ; Load accumulator with value in "oprs"
     sta     DisVal            ; Copy to "DisVal" variable
     jsr     CONV_8BIT_ASCII   ; Jump to subroutine at CONV_8BIT_ASCII:
     lda     AC_100            ; Load accumulator with value in "AC_100"
     sta     Botlin6           ; Copy to "BotLin6"(7th column on left, bottom line)
     lda     AC_10             ; Load accumulator with value in "AC_10"
     sta     Botlin7           ; Copy to "BotLin7"(8th column on left, bottom line)
     lda     AC_1              ; Load accumulator with value in "AC_1"
     sta     Botlin8           ; Copy to "BotLin8"(9th column on left, bottom line)
     lda     #$20              ; Load accumulator with ASCII ' '(space)
     sta     Botlin9           ; Copy to "BotLin9"(10th column on left, bottom line)
     sta     Botlin10          ; Copy to "BotLin10"(11th column on left, bottom line)
     lda     fpADC             ; Load accumulator with value in "fpADC"
     sta     DisVal            ; Copy to "DisVal" variable
     jsr     CONV_8BIT_ASCII   ; Jump to subroutine at CONV_8BIT_ASCII:
     lda     AC_100            ; Load accumulator with value in "AC_100"
     sta     Botlin11          ; Copy to "BotLin11"(12th column on left, bottom line)
     lda     AC_10             ; Load accumulator with value in "AC_10"
     sta     Botlin12          ; Copy to "BotLin12"(13th column on left, bottom line)
     lda     AC_1              ; Load accumulator with value in "AC_1"
     sta     Botlin13          ; Copy to "BotLin13"(14th column on left, bottom line)
     lda     #$20              ; Load accumulator with ASCII ' '(space)
     sta     Botlin14          ; Copy to "BotLin14"(15th column on left, bottom line)
     sta     Botlin15          ; Copy to "BotLin15"(16th column on left, bottom line)
     jsr     CALC_FP           ; Jump to subroutine at CALC_FP:
     lda     fprs              ; Load accumulator with value in "fprs"
     sta     DisVal            ; Copy to "DisVal" variable
     jsr     CONV_8BIT_ASCII   ; Jump to subroutine at CONV_8BIT_ASCII:
     lda     AC_100            ; Load accumulator with value in "AC_100"
     sta     Botlin16          ; Copy to "BotLin16"(17th column on left, bottom line)
     lda     AC_10             ; Load accumulator with value in "AC_10"
     sta     Botlin17          ; Copy to "BotLin17"(18th column on left, bottom line)
     lda     AC_1              ; Load accumulator with value in "AC_1"
     sta     Botlin18          ; Copy to "BotLin18"(19th column on left, bottom line)
     lda     #$20              ; Load accumulator with ASCII ' '(space)
     sta     Botlin19          ; Copy to "BotLin19"(20th column on left, bottom line)
     jmp     CHK_FRZ_DISP      ; Jump to CHK_FRZ_DISP:

SCRN_12:
     lda     pwcalcH           ; Load accumulator with value in "pwcalcH"
     sta     intacc1+2         ; Copy to "intacc1+2"
     lda     pwcalcL           ; Load accumulator with value in "pwcalcL"
     sta     intacc1+3         ; Copy to "intacc1+3"
     jsr     DIV_BY_TEN        ; Jump to subroutine at DIV_BY_TEN:
     jsr     CONV_16BIT_ASCII  ; Jump to subroutine at CONV_16BIT_ASCII:
     lda     thousands         ; Load accumulator with value in "thousands"
     sta     Botlin0           ; Copy to "BotLin0"(1st column on left, bottom line)
     lda     hundreds          ; Load accumulator with value in "hundreds"
     sta     Botlin1           ; Copy to "BotLin1"(2nd column on left, bottom line)
     lda     tens              ; Load accumulator with value in "tens"
     sta     Botlin2           ; Copy to "BotLin4"(3d column on left, bottom line)
     lda     ones              ; Load accumulator with value in "ones"
     sta     Botlin3           ; Copy to "BotLin3"(4th column on left, bottom line)
     lda     #$20              ; Load accumulator with ASCII ' '(space)
     sta     Botlin4           ; Copy to "BotLin4"(5th column on left, bottom line)
     lda     pw               ; Load accumulator with value in "pw"
     sta     DisVal            ; Copy to "DisVal" variable
     jsr     CONV_8BIT_ASCII   ; Jump to subroutine at CONV_8BIT_ASCII:
     lda     AC_100            ; Load accumulator with value in "AC_100"
     sta     Botlin5           ; Copy to "BotLin5"(6th column on left, bottom line)
     lda     AC_10             ; Load accumulator with value in "AC_10"
     sta     Botlin6           ; Copy to "BotLin6"(7th column on left, bottom line)
     lda     AC_1              ; Load accumulator with value in "AC_1"
     sta     Botlin7           ; Copy to "BotLin7"(8th column on left, bottom line)
     lda     #$20              ; Load accumulator with ASCII ' '(space)
     sta     Botlin8           ; Copy to "BotLin8"(9th column on left, bottom line)
     lda     fd                ; Load accumulator with value in "fd"
     sta     DisVal            ; Copy to "DisVal" variable
     jsr     CONV_8BIT_ASCII   ; Jump to subroutine at CONV_8BIT_ASCII:
     lda     AC_100            ; Load accumulator with value in "AC_100"
     sta     Botlin9           ; Copy to "BotLin9"(10th column on left, bottom line)
     lda     AC_10             ; Load accumulator with value in "AC_10"
     sta     Botlin10          ; Copy to "BotLin10"(11th column on left, bottom line)
     lda     AC_1              ; Load accumulator with value in "AC_1"
     sta     Botlin11          ; Copy to "BotLin11"(12th column on left, bottom line)
     lda     #$20              ; Load accumulator with ASCII ' '(space)
     sta     Botlin12          ; Copy to "BotLin12"(13th column on left, bottom line)
     lda     vecurr            ; Load accumulator with value in "vecurr"
     sta     DisVal            ; Copy to "DisVal" variable
     jsr     CONV_8BIT_ASCII   ; Jump to subroutine at CONV_8BIT_ASCII:
     lda     AC_100            ; Load accumulator with value in "AC_100"
     sta     Botlin13          ; Copy to "BotLin13"(14th column on left, bottom line)
     lda     AC_10             ; Load accumulator with value in "AC_10"
     sta     Botlin14          ; Copy to "BotLin14"(15th column on left, bottom line)
     lda     AC_1              ; Load accumulator with value in "AC_1"
     sta     Botlin15          ; Copy to "BotLin15"(16th column on left, bottom line)
     lda     #$20              ; Load accumulator with ASCII ' '(space)
     sta     Botlin16          ; Copy to "BotLin16"(17th column on left, bottom line)
     jsr     CALC_DUTY         ; Jump to subroutine at CALC_DUTY:
     lda     duty              ; Load accumulator with value in "duty"
     sta     DisVal            ; Copy to "DisVal" variable
     jsr     CONV_8BIT_ASCII   ; Jump to subroutine at CONV_8BIT_ASCII:
     lda     AC_100            ; Load accumulator with value in "AC_100"
     sta     Botlin17          ; Copy to "BotLin17"(18th column on left, bottom line)
     lda     AC_10             ; Load accumulator with value in "AC_10"
     sta     Botlin18          ; Copy to "BotLin18"(19th column on left, bottom line)
     lda     AC_1              ; Load accumulator with value in "AC_1"
     sta     Botlin19          ; Copy to "BotLin19"(20th column on left, bottom line)
     jmp     CHK_FRZ_DISP      ; Jump to CHK_FRZ_DISP:

SCRN_13:
     lda     #$20              ; Load accumulator with ASCII ' '(space)
     sta     Botlin0           ; Copy to "BotLin0"(1st column on left, bottom line)
     jsr     CALC_RPM_HR       ; Jump to subroutine at CALC_RPM_HR:
     lda     rpmhrH            ; Load accumulator with value in "rpmhrH"
     sta     intacc1+2         ; Copy to "intacc1+2"
     lda     rpmhrL            ; Load accumulator with value in "rpmhrL"
     sta     intacc1+3         ; Copy to "intacc1+3"
     jsr     CONV_16BIT_ASCII  ; Jump to subroutine at CONV_16BIT_ASCII:
     lda     thousands         ; Load accumulator with value in "thousands"
     sta     Botlin1           ; Copy to "BotLin1"(2nd column on left, bottom line)
     lda     hundreds          ; Load accumulator with value in "hundreds"
     sta     Botlin2           ; Copy to "BotLin2"(3d column on left, bottom line)
     lda     tens              ; Load accumulator with value in "tens"
     sta     Botlin3           ; Copy to "BotLin3"(4th column on left, bottom line)
     lda     ones              ; Load accumulator with value in "ones"
     sta     Botlin4           ; Copy to "BotLin4"(5th column on left, bottom line)
     lda     #$20              ; Load accumulator with ASCII ' '(space)
     sta     Botlin5           ; Copy to "BotLin5"(6th column on left, bottom line)
     lda     tachH             ; Load accumulator with value in "tachH"
     sta     intacc1+2         ; Copy to "intacc1+2"
     lda     tachL             ; Load accumulator with value in "tachL"
     sta     intacc1+3         ; Copy to "intacc1+3"
     jsr     DIV_BY_TEN        ; Jump to subroutine at DIV_BY_TEN:
     jsr     CONV_16BIT_ASCII  ; Jump to subroutine at CONV_16BIT_ASCII:
     lda     thousands         ; Load accumulator with value in "thousands"
     sta     Botlin6           ; Copy to "BotLin6"(7th column on left, bottom line)
     lda     hundreds          ; Load accumulator with value in "hundreds"
     sta     Botlin7           ; Copy to "BotLin7"(8th column on left, bottom line)
     lda     tens              ; Load accumulator with value in "tens"
     sta     Botlin8           ; Copy to "BotLin8"(9th column on left, bottom line)
     lda     ones              ; Load accumulator with value in "ones"
     sta     Botlin9           ; Copy to "BotLin9"(10th column on left, bottom line)
     lda     #$20              ; Load accumulator with ASCII ' '(space)
     sta     Botlin10          ; Copy to "BotLin10"(11th column on left, bottom line)
     lda     monpH             ; Load accumulator with value in "monpH"
     sta     intacc1+2         ; Copy to "intacc1+2"
     lda     monpL             ; Load accumulator with value in "monpL"
     sta     intacc1+3         ; Copy to "intacc1+3"
     jsr     DIV_BY_TEN        ; Jump to subroutine at DIV_BY_TEN:
     jsr     CONV_16BIT_ASCII  ; Jump to subroutine at CONV_16BIT_ASCII:
     lda     thousands         ; Load accumulator with value in "thousands"
     sta     Botlin11          ; Copy to "BotLin11"(12th column on left, bottom line)
     lda     hundreds          ; Load accumulator with value in "hundreds"
     sta     Botlin12          ; Copy to "BotLin12"(13th column on left, bottom line)
     lda     tens              ; Load accumulator with value in "tens"
     sta     Botlin13          ; Copy to "BotLin13"(14th column on left, bottom line)
     lda     ones              ; Load accumulator with value in "ones"
     sta     Botlin14          ; Copy to "BotLin14"(15th column on left, bottom line)
     lda     #$20              ; Load accumulator with ASCII ' '(space)
     sta     Botlin15          ; Copy to "BotLin15"(16th column on left, bottom line)
     lda     secl              ; Load accumulator with value in "secl"
     sta     DisVal            ; Copy to "DisVal" variable
     jsr     CONV_8BIT_ASCII   ; Jump to subroutine at CONV_8BIT_ASCII:
     lda     AC_100            ; Load accumulator with value in "AC_100"
     sta     Botlin16          ; Copy to "BotLin16"(17th column on left, bottom line)
     lda     AC_10             ; Load accumulator with value in "AC_10"
     sta     Botlin17          ; Copy to "BotLin17"(18th column on left, bottom line)
     lda     AC_1              ; Load accumulator with value in "AC_1"
     sta     Botlin18          ; Copy to "BotLin18"(19th column on left, bottom line)
     lda     #$20              ; Load accumulator with ASCII ' '(space)
     sta     Botlin19          ; Copy to "BotLin19"(20th column on left, bottom line)
     jmp     CHK_FRZ_DISP      ; Jump to CHK_FRZ_DISP:

SCRN_14:
     jsr     CALC_RPM          ; Jump to subroutine at CALC_RPM:
     lda     rpmH              ; Load accumulator with value in "rpmH"
     sta     intacc1+2         ; Copy to "intacc1+2"
     lda     rpmL              ; Load accumulator with value in "rpmL"
     sta     intacc1+3         ; Copy to "intacc1+3"
     jsr     CONV_16BIT_ASCII  ; Jump to subroutine at CONV_16BIT_ASCII:
     lda     thousands         ; Load accumulator with value in "thousands"
     sta     Botlin0           ; Copy to "BotLin0"(1st column on left, bottom line)
     lda     hundreds          ; Load accumulator with value in "hundreds"
     sta     Botlin1           ; Copy to "BotLin1"(2nd column on left, bottom line)
     lda     tens              ; Load accumulator with value in "tens"
     sta     Botlin2           ; Copy to "BotLin4"(3d column on left, bottom line)
     lda     ones              ; Load accumulator with value in "ones"
     sta     Botlin3           ; Copy to "BotLin3"(4th column on left, bottom line)
     lda     #$20              ; Load accumulator with ASCII ' '(space)
     sta     Botlin4           ; Copy to "BotLin4"(5th column on left, bottom line)
     jsr     CALC_MAP          ; Jump to subroutine at CALC_MAP:
     lda     map               ; Load accumulator with value in "map"
     sta     DisVal            ; Copy to "DisVal" variable
     jsr     CONV_8BIT_ASCII   ; Jump to subroutine at CONV_8BIT_ASCII:
     lda     AC_100            ; Load accumulator with value in "AC_100"
     sta     Botlin5           ; Copy to "BotLin5"(6th column on left, bottom line)
     lda     AC_10             ; Load accumulator with value in "AC_10"
     sta     Botlin6           ; Copy to "BotLin6"(7th column on left, bottom line)
     lda     AC_1              ; Load accumulator with value in "AC_1"
     sta     Botlin7           ; Copy to "BotLin7"(8th column on left, bottom line)
     lda     #$20              ; Load accumulator with ASCII ' '(space)
     sta     Botlin8           ; Copy to "BotLin8"(9th column on left, bottom line)
     lda     spkAngFac         ; Load accumulator with value in "spkAngFac"
     sta     DisVal            ; Copy to "DisVal" variable
     jsr     CONV_8BIT_ASCII   ; Jump to subroutine at CONV_8BIT_ASCII:
     lda     AC_100            ; Load accumulator with value in "AC_100"
     sta     Botlin9           ; Copy to "BotLin9"(10th column on left, bottom line)
     lda     AC_10             ; Load accumulator with value in "AC_10"
     sta     Botlin10          ; Copy to "BotLin10"(11th column on left, bottom line)
     lda     AC_1              ; Load accumulator with value in "AC_1"
     sta     Botlin11          ; Copy to "BotLin11"(12th column on left, bottom line)
     lda     #$20              ; Load accumulator with ASCII ' '(space)
     sta     Botlin12          ; Copy to "BotLin12"(13th column on left, bottom line)
     lda     dlyAngFac         ; Load accumulator with value in "dlyAngFac"
     sta     DisVal            ; Copy to "DisVal" variable
     jsr     CONV_8BIT_ASCII   ; Jump to subroutine at CONV_8BIT_ASCII:
     lda     AC_100            ; Load accumulator with value in "AC_100"
     sta     Botlin13          ; Copy to "BotLin13"(14th column on left, bottom line)
     lda     AC_10             ; Load accumulator with value in "AC_10"
     sta     Botlin14          ; Copy to "BotLin14"(15th column on left, bottom line)
     lda     AC_1              ; Load accumulator with value in "AC_1"
     sta     Botlin15          ; Copy to "BotLin15"(16th column on left, bottom line)
     lda     #$20              ; Load accumulator with ASCII ' '(space)
     sta     Botlin16          ; Copy to "BotLin16"(17th column on left, bottom line)
     lda     trmAngFac         ; Load accumulator with value in "trmAngFac"
     sta     DisVal            ; Copy to "DisVal" variable
     jsr     CONV_8BIT_ASCII   ; Jump to subroutine at CONV_8BIT_ASCII:
     lda     AC_100            ; Load accumulator with value in "AC_100"
     sta     Botlin17          ; Copy to "BotLin17"(18th column on left, bottom line)
     lda     AC_10             ; Load accumulator with value in "AC_10"
     sta     Botlin18          ; Copy to "BotLin18"(19th column on left, bottom line)
     lda     AC_1              ; Load accumulator with value in "AC_1"
     sta     Botlin19          ; Copy to "BotLin19"(20th column on left, bottom line)
     jmp     CHK_FRZ_DISP      ; Jump to CHK_FRZ_DISP:

SCRN_15:
     jsr     CALC_RPM          ; Jump to subroutine at CALC_RPM:
     lda     rpmH              ; Load accumulator with value in "rpmH"
     sta     intacc1+2         ; Copy to "intacc1+2"
     lda     rpmL              ; Load accumulator with value in "rpmL"
     sta     intacc1+3         ; Copy to "intacc1+3"
     jsr     CONV_16BIT_ASCII  ; Jump to subroutine at CONV_16BIT_ASCII:
     lda     thousands         ; Load accumulator with value in "thousands"
     sta     Botlin0           ; Copy to "BotLin0"(1st column on left, bottom line)
     lda     hundreds          ; Load accumulator with value in "hundreds"
     sta     Botlin1           ; Copy to "BotLin1"(2nd column on left, bottom line)
     lda     tens              ; Load accumulator with value in "tens"
     sta     Botlin2           ; Copy to "BotLin4"(3d column on left, bottom line)
     lda     ones              ; Load accumulator with value in "ones"
     sta     Botlin3           ; Copy to "BotLin3"(4th column on left, bottom line)
     lda     #$20              ; Load accumulator with ASCII ' '(space)
     sta     Botlin4           ; Copy to "BotLin4"(5th column on left, bottom line)
     jsr     CALC_TRIM_ANG     ; Jump to subroutine at CALC_TRIM_ANG:
     lda     trmAng            ; Load accumulator with value in "trmAng"
     sta     DisVal            ; Copy to "DisVal" variable
     jsr     CONV_8BIT_ASCII   ; Jump to subroutine at CONV_8BIT_ASCII:
     lda     AC_100            ; Load accumulator with value in "AC_100"
     sta     Botlin5           ; Copy to "BotLin5"(6th column on left, bottom line)
     lda     AC_10             ; Load accumulator with value in "AC_10"
     sta     Botlin6           ; Copy to "BotLin6"(7th column on left, bottom line)
     lda     AC_1              ; Load accumulator with value in "AC_1"
     sta     Botlin7           ; Copy to "BotLin7"(8th column on left, bottom line)
     lda     #$20              ; Load accumulator with ASCII ' '(space)
     sta     Botlin8           ; Copy to "BotLin8"(9th column on left, bottom line)
     jsr     CALC_DLY_ANG      ; Jump to subroutine at CALC_DLY_ANG:
     lda     dlyAng            ; Load accumulator with value in "dlyAng"
     sta     DisVal            ; Copy to "DisVal" variable
     jsr     CONV_8BIT_ASCII   ; Jump to subroutine at CONV_8BIT_ASCII:
     lda     AC_100            ; Load accumulator with value in "AC_100"
     sta     Botlin9           ; Copy to "BotLin9"(10th column on left, bottom line)
     lda     AC_10             ; Load accumulator with value in "AC_10"
     sta     Botlin10          ; Copy to "BotLin10"(11th column on left, bottom line)
     lda     AC_1              ; Load accumulator with value in "AC_1"
     sta     Botlin11          ; Copy to "BotLin11"(12th column on left, bottom line)
     lda     #$20              ; Load accumulator with ASCII ' '(space)
     sta     Botlin12          ; Copy to "BotLin12"(13th column on left, bottom line)
     jsr     CALC_TIM_ANG      ; Jump to subroutine at CALC_TIM_ANG:
     lda     timAng            ; Load accumulator with value in "timAng"
     sta     DisVal            ; Copy to "DisVal" variable
     jsr     CONV_8BIT_ASCII   ; Jump to subroutine at CONV_8BIT_ASCII:
     lda     AC_100            ; Load accumulator with value in "AC_100"
     sta     Botlin13          ; Copy to "BotLin13"(14th column on left, bottom line)
     lda     AC_10             ; Load accumulator with value in "AC_10"
     sta     Botlin14          ; Copy to "BotLin14"(15th column on left, bottom line)
     lda     AC_1              ; Load accumulator with value in "AC_1"
     sta     Botlin15          ; Copy to "BotLin15"(16th column on left, bottom line)
     lda     #$20              ; Load accumulator with ASCII ' '(space)
     sta     Botlin16          ; Copy to "BotLin16"(17th column on left, bottom line)
     jsr     CALC_MON_TIM_ANG  ; Jump to subroutine at CALC_MON_TIM_ANG:
     lda     monTimAng         ; Load accumulator with value in "monTimAng"
     sta     DisVal            ; Copy to "DisVal" variable
     jsr     CONV_8BIT_ASCII   ; Jump to subroutine at CONV_8BIT_ASCII:
     lda     AC_100            ; Load accumulator with value in "AC_100"
     sta     Botlin17          ; Copy to "BotLin17"(18th column on left, bottom line)
     lda     AC_10             ; Load accumulator with value in "AC_10"
     sta     Botlin18          ; Copy to "BotLin18"(19th column on left, bottom line)
     lda     AC_1              ; Load accumulator with value in "AC_1"
     sta     Botlin19          ; Copy to "BotLin19"(20th column on left, bottom line)
     jmp     CHK_FRZ_DISP      ; Jump to CHK_FRZ_DISP:

SCRN_16:
;CHK_FP:
     brset   FPon,portAbits,SET_FP  ; If "FPon" bit of "portAbits" is set, branch to SET_FP:
     mov     #'N',Botlin0           ; Move "N" into "Botlin0"(1st column on left, bottom line)
     bra     SET_FP_DONE            ; Branch to SET_FP_DONE:

SET_FP:
     mov     #'Y',Botlin0           ; Move "Y" into "Botlin0"(1st column on left, bottom line)

SET_FP_DONE:
     lda     #$20                   ; Load accumulator with ASCII ' '(space)
     sta     Botlin1                ; Copy to "BotLin1"(2nd column on left, bottom line)
     sta     Botlin2                ; Copy to "BotLin2"(3d column on left, bottom line)

;CHK_RN:
     brset   running,engine,SET_RN  ; If "running" bit of "engine" is set, branch to SET_RN:
     mov     #'N',Botlin3           ; Move "N" into "Botlin3"(4th column on left, bottom line)
     bra     SET_RN_DONE            ; Branch to SET_RN_DONE:

SET_RN:
     mov     #'Y',Botlin3           ; Move "Y" into "Botlin3"(4th column on left, bottom line)

SET_RN_DONE:
     lda     #$20                   ; Load accumulator with ASCII ' '(space)
     sta     Botlin4                ; Copy to "BotLin4"(5th column on left, bottom line)
     sta     Botlin5                ; Copy to "BotLin5"(6th column on left, bottom line)

;CHK_CR:
     brset   crank,engine,SET_CR    ; If "crank" bit of "engine" is set, branch to SET_CR:
     mov     #'N',Botlin6           ; Move "N" into "Botlin6"(7th column on left, bottom line)
     bra     SET_CR_DONE            ; Branch to SET_CR_DONE:

SET_CR:
     mov     #'Y',Botlin6           ; Move "Y" into "Botlin6"(7th column on left, bottom line)

SET_CR_DONE:
     lda     #$20                   ; Load accumulator with ASCII ' '(space)
     sta     Botlin7                ; Copy to "BotLin7"(8th column on left, bottom line)
     sta     Botlin8                ; Copy to "BotLin8"(9th column on left, bottom line)

;CHK_SW:
     brset   startw,engine,SET_SW   ; If "startw" bit of "engine" is set, branch to SET_SW:
     mov     #'N',Botlin9           ; Move "N" into "Botlin9"(10th column on left, bottom line)
     bra     SET_SW_DONE            ; Branch to SET_SW_DONE:

SET_SW:
     mov     #'Y',Botlin9           ; Move "Y" into "Botlin9"(10th column on left, bottom line)

SET_SW_DONE:
     lda     #$20                   ; Load accumulator with ASCII ' '(space)
     sta     Botlin10               ; Copy to "BotLin10"(11th column on left, bottom line)
     sta     Botlin11               ; Copy to "BotLin11"(12th column on left, bottom line)

;CHK_RW:
     brset   warmup,engine,SET_RW   ; If "warmup" bit of "engine" is set, branch to SET_RW:
     mov     #'N',Botlin12          ; Move "N" into "Botlin12"(13th column on left, bottom line)
     bra     SET_RW_DONE            ; Branch to SET_RW_DONE:

SET_RW:
     mov     #'Y',Botlin12           ; Move "Y" into "Botlin12"(13th column on left, bottom line)

SET_RW_DONE:
     lda     #$20                   ; Load accumulator with ASCII ' '(space)
     sta     Botlin13               ; Copy to "BotLin13"(14th column on left, bottom line)
     sta     Botlin14               ; Copy to "BotLin14"(15th column on left, bottom line)

;CHK_AC:
     brset   tpsaen,engine,SET_AC   ; If "tpsaen" bit of "engine" is set, branch to SET_AC:
     mov     #'N',Botlin15          ; Move "N" into "Botlin15"(16th column on left, bottom line)
     bra     SET_AC_DONE            ; Branch to SET_AC_DONE:

SET_AC:
     mov     #'Y',Botlin15          ; Move "Y" into "Botlin15"(16th column on left, bottom line)

SET_AC_DONE:
     lda     #$20                   ; Load accumulator with ASCII ' '(space)
     sta     Botlin16               ; Copy to "BotLin16"(17th column on left, bottom line)
     sta     Botlin17               ; Copy to "BotLin17"(18th column on left, bottom line)

;CHK_DC:
     brset   tpsden,engine,SET_DC   ; If "tpsden" bit of "engine" is set, branch to SET_DC:
     mov     #'N',Botlin18          ; Move "N" into "Botlin18"(19th column on left, bottom line)
     bra     SET_DC_DONE            ; Branch to SET_DC_DONE:

SET_DC:
     mov     #'Y',Botlin18          ; Move "Y" into "Botlin18"(19th column on left, bottom line)

SET_DC_DONE:
     lda     #$20                   ; Load accumulator with ASCII ' '(space)
     sta     Botlin19               ; Copy to "BotLin19"(20th column on left, bottom line)
     jmp     CHK_FRZ_DISP            ; Jump to CHK_FRZ_DISP:

SCRN_17:
;CHK_EA:
     brset   EAon,portCbits,SET_EA   ; If "EAon" bit of "portCbits" is set, branch to SET_EA:
     mov     #'N',Botlin0            ; Move "N" into "Botlin0"(1st column on left, bottom line)
     bra     SET_EA_DONE             ; Branch to SET_EA_DONE:

SET_EA:
     mov     #'Y',Botlin0            ; Move "Y" into "Botlin0"(1st column on left, bottom line)

SET_EA_DONE:
     lda     #$20                    ; Load accumulator with ASCII ' '(space)
     sta     Botlin1                 ; Copy to "BotLin1"(2nd column on left, bottom line)
     sta     Botlin2                 ; Copy to "BotLin2"(3d column on left, bottom line)

;CHK_RL:
     brset   REVL,alarmbits,SET_RL   ; If "REVL" bit of "alarmbits" is set, branch to SET_RL:
     mov     #'N',Botlin3            ; Move "N" into "Botlin3"(4th column on left, bottom line)
     bra     SET_RL_DONE             ; Branch to SET_RL_DONE:

SET_RL:
     mov     #'Y',Botlin3            ; Move "Y" into "Botlin3"(4th column on left, bottom line)

SET_RL_DONE:
     lda     #$20                    ; Load accumulator with ASCII ' '(space)
     sta     Botlin4                 ; Copy to "BotLin4"(5th column on left, bottom line)
     sta     Botlin5                 ; Copy to "BotLin5"(6th column on left, bottom line)

;CHK_FC:
     brset   fldClr,alarmbits,SET_FC ; If "fldClr" bit of "alarmbits" is set, branch to SET_FC:
     mov     #'N',Botlin6            ; Move "N" into "Botlin6"(7th column on left, bottom line)
     bra     SET_FC_DONE             ; Branch to SET_FC_DONE:

SET_FC:
     mov     #'Y',Botlin6            ; Move "Y" into "Botlin6"(7th column on left, bottom line)

SET_FC_DONE:
     lda     #$20                    ; Load accumulator with ASCII ' '(space)
     sta     Botlin7                 ; Copy to "BotLin7"(8th column on left, bottom line)
     sta     Botlin8                 ; Copy to "BotLin8"(9th column on left, bottom line)
     sta     Botlin9                 ; Copy to "BotLin9"(10th column on left, bottom line)
     sta     Botlin10                ; Copy to "BotLin10"(11th column on left, bottom line)
     sta     Botlin11                ; Copy to "BotLin11"(12th column on left, bottom line)
     sta     Botlin12                ; Copy to "BotLin12"(13th column on left, bottom line)
     sta     Botlin13                ; Copy to "BotLin13"(14th column on left, bottom line)
     sta     Botlin14                ; Copy to "BotLin14"(15th column on left, bottom line)

 ;CHK_FT:
     brset   FTen,portAbits,SET_FT   ; If "FTen" bit of "portAbits" is set, branch to SET_FT:
     mov     #'N',Botlin15            ; Move "N" into "Botlin15"(16th column on left, bottom line)
     bra     SET_FT_DONE             ; Branch to SET_FT_DONE:

SET_FT:
     mov     #'Y',Botlin15            ; Move "Y" into "Botlin15"(16th column on left, bottom line)

SET_FT_DONE:
     lda     #$20                    ; Load accumulator with ASCII ' '(space)
     sta     Botlin16                 ; Copy to "BotLin16"(17th column on left, bottom line)
     sta     Botlin17                 ; Copy to "BotLin17"(18th column on left, bottom line)

;CHK_IT:
     brset   ITen,portAbits,SET_IT   ; If "ITen" bit of "portAbits" is set, branch to SET_IT:
     mov     #'N',Botlin18           ; Move "N" into "Botlin18"(19th column on left, bottom line)
     bra     SET_IT_DONE             ; Branch to SET_IT_DONE:

SET_IT:
     mov     #'Y',Botlin18           ; Move "Y" into "Botlin18"(19th column on left, bottom line)

SET_IT_DONE:
     lda     #$20                    ; Load accumulator with ASCII ' '(space)
     sta     Botlin19                ; Copy to "BotLin19"(20th column on left, bottom line)
     jmp     CHK_FRZ_DISP            ; Jump to CHK_FRZ_DISP:

SCRN_18:
;CHK_EA1:
     brset   EAon,portCbits,SET_EA1  ; If "EAon" bit of "portCbits" is set, branch to SET_EA1:
     mov     #'N',Botlin0            ; Move "N" into "Botlin0"(1st column on left, bottom line)
     bra     SET_EA1_DONE            ; Branch to SET_EA1_DONE:

SET_EA1:
     mov     #'Y',Botlin0            ; Move "Y" into "Botlin0"(1st column on left, bottom line)

SET_EA1_DONE:
     lda     #$20                    ; Load accumulator with ASCII ' '(space)
     sta     Botlin1                 ; Copy to "BotLin1"(2nd column on left, bottom line)
     sta     Botlin2                 ; Copy to "BotLin2"(3d column on left, bottom line)

;CHK_HT:
     brset   HET,alarmbits,SET_HT    ; If "HET" bit of "alarmbits" is set, branch to SET_HT:
     mov     #'N',Botlin3            ; Move "N" into "Botlin3"(4th column on left, bottom line)
     bra     SET_HT_DONE             ; Branch to SET_HT_DONE:

SET_HT:
     mov     #'Y',Botlin3            ; Move "Y" into "Botlin3"(4th column on left, bottom line)

SET_HT_DONE:
     lda     #$20                    ; Load accumulator with ASCII ' '(space)
     sta     Botlin4                 ; Copy to "BotLin4"(5th column on left, bottom line)
     sta     Botlin5                 ; Copy to "BotLin5"(6th column on left, bottom line)

;CHK_LO:
     brset   LOP,alarmbits,SET_LO    ; If "LOP" bit of "alarmbits" is set, branch to SET_LO:
     mov     #'N',Botlin6            ; Move "N" into "Botlin6"(7th column on left, bottom line)
     bra     SET_LO_DONE             ; Branch to SET_LO_DONE:

SET_LO:
     mov     #'Y',Botlin6            ; Move "Y" into "Botlin6"(7th column on left, bottom line)

SET_LO_DONE:
     lda     #$20                    ; Load accumulator with ASCII ' '(space)
     sta     Botlin7                 ; Copy to "BotLin7"(8th column on left, bottom line)
     sta     Botlin8                 ; Copy to "BotLin8"(9th column on left, bottom line)

;CHK_LF:
     brset   LFP,alarmbits,SET_LF    ; If "LFP" bit of "alarmbits" is set, branch to SET_LF:
     mov     #'N',Botlin9            ; Move "N" into "Botlin9"(10th column on left, bottom line)
     bra     SET_LF_DONE             ; Branch to SET_LF_DONE:

SET_LF:
     mov     #'Y',Botlin9            ; Move "Y" into "Botlin9"(10th column on left, bottom line)

SET_LF_DONE:
     lda     #$20                    ; Load accumulator with ASCII ' '(space)
     sta     Botlin10                ; Copy to "BotLin10"(11th column on left, bottom line)
     sta     Botlin11                ; Copy to "BotLin11"(12th column on left, bottom line)

;CHK_HF:
     brset   HFP,alarmbits,SET_HF    ; If "HFP" bit of "alarmbits" is set, branch to SET_HF:
     mov     #'N',Botlin12           ; Move "N" into "Botlin12"(13th column on left, bottom line)
     bra     SET_HF_DONE             ; Branch to SET_HF_DONE:

SET_HF:
     mov     #'Y',Botlin12           ; Move "Y" into "Botlin12"(13th column on left, bottom line)

SET_HF_DONE:
     lda     #$20                    ; Load accumulator with ASCII ' '(space)
     sta     Botlin13                ; Copy to "BotLin13"(14th column on left, bottom line)
     sta     Botlin14                ; Copy to "BotLin14"(15th column on left, bottom line)

;CHK_ET:
     brset   HEGT,alarmbits,SET_ET   ; If "HEGT" bit of "alarmbits" is set, branch to SET_ET:
     mov     #'N',Botlin15           ; Move "N" into "Botlin15"(16th column on left, bottom line)
     bra     SET_ET_DONE             ; Branch to SET_ET_DONE:

SET_ET:
     mov     #'Y',Botlin15           ; Move "Y" into "Botlin15"(16th column on left, bottom line)

SET_ET_DONE:
     lda     #$20                    ; Load accumulator with ASCII ' '(space)
     sta     Botlin16                ; Copy to "BotLin16"(17th column on left, bottom line)
     sta     Botlin17                ; Copy to "BotLin17"(18th column on left, bottom line)

;CHK_KN:
     brset   knock,alarmbits,SET_KN  ; If "knock" bit of "alarmbits" is set, branch to SET_KN:
     mov     #'N',Botlin18           ; Move "N" into "Botlin18"(19th column on left, bottom line)
     bra     SET_KN_DONE             ; Branch to SET_KN_DONE:

SET_KN:
     mov     #'Y',Botlin18           ; Move "Y" into "Botlin18"(19th column on left, bottom line)

SET_KN_DONE:
     lda     #$20                    ; Load accumulator with ASCII ' '(space)
     sta     Botlin19                ; Copy to "BotLin19"(20th column on left, bottom line)


;***************************************************************************
; - Check to see if we have had a "display freeze" command and branch
;   accordingly. (This feature was from a previous version but left in
;   for ease of programming)
;***************************************************************************

CHK_FRZ_DISP:
     brset    frz,flags,NO_CHNG_DB     ; If "frz" bit of "flags" variable
                                       ; is set, branch to NO_CHNG_DB:


;****************************************************************************
; - Compare all the characters on the bottom line commanded, to those of the
;   previous bottom line. If they are different, update the bottom line,
;   otherwise, skip over. This is to eliminate display  "digit rattle"
;   caused by rapid screen updates.
;****************************************************************************

BOTLIN_CHK_D:
     jsr     CMP_BOTLIN                   ; Jump to subroutine at CMP_BOTLIN:
     brclr   LinChng,flags,NO_CHNG_DB     ; If "LinChng" bit of "flags"
                                          ; variable is clear, branch to
                                          ; NO_CHNG_DB:

;****************************************************************************
; - Set up the VFD to place the first character in the bottom line, extreme
;   left hand position
;****************************************************************************

     jsr     VFD_START_BOT      ; Jump to subroutine at VFD_START_BOT:

;***************************************************************************
; - Print the bottom line of the VFD display
;***************************************************************************

PRINT_BOT_D:
     ldhx    #BotLin0       ; Load index register with address of
                            ; entry point for bottom line of VFD
     jsr     PRINT_LINE     ; Jump to subroutine at PRINT_LINE:

NO_CHNG_DB:
     jmp     LOOPY          ; Jump to LOOPY:(End of program loop while in
                            ; "Display" mode)


;****************************************************************************
;
; * * * * * * * * * * * * * * Interrupt Section * * * * * * * * * * * * * *
;
; NOTE!!! If the interrupt service routine modifies the H register, or uses
; the indexed addressing mode, save the H register (pshh) and then restore
; it (pulh) prior to exiting the routine
;
;****************************************************************************

;****************************************************************************
;
; -------- Following interrupt service routines in priority order ----------
;
; TIM2CH0_ISR: - TIM2 CH0 Interrupt (1000uS clock tick)(1.0mS)
;
; SCIRCV_ISR:  - SCI receive
;
; SCITX_ISR:   - SCI transmit (Not used)
;
;
;***************************************************************************

;****************************************************************************
;============================================================================
; - TIM2 CH0 Interrupt (1000uS clock tick)(1.0mS)
; - Generate time rates:
;   Milliseconds,(for contact de-bounce counters)
;   5 Milleseconds,(for auto-repeat and auto-repeat command counters)
;   250 Milliseconds,(for real time variable display updates)
;============================================================================
;****************************************************************************

TIM2CH0_ISR:
     pshh                  ; Push value in index register Hi byte to stack
     lda     T2SC0         ; Load accumulator with value in TIM2 CH0 Status
                           ; and Control Register (Arm CHxF flag clear)
     bclr    CHxF,T2SC0    ; Clear CHxF bit of TIM2 CH0 Status and
                           ; Control Register
     lda     T2CH0L        ; Load accumulator with value in TIM2 CH0 OC
                           ; register Lo byte
     add     #$E8          ; Add (A)<-(A)+(#) decimal 232
     tax                   ; Transfer value in accumulator to index
                           ; register Lo byte
     lda     T2CH0H        ; Load accumulator with value in TIM2 CH0 OC
                           ; register Hi byte
     adc     #$03          ; Add with carry decimal 768 (A)<-(A)+(#)+(C)
                           ;(total = ~1000uS)
     sta     T2CH0H        ; Copy result to TIM2 CH0 OC register Hi byte
     stx     T2CH0L        ; Copy value in index register Lo byte
                           ; to TIM2 CH0 OC register Lo byte
                           ;(new output compare value)

;============================================================================
;*********************** 1.0 millisecond section ****************************
;============================================================================


;****************************************************************************
; - Check the value of the contact de-bounce counter variables, if other
;   than zero, decrement them.
;****************************************************************************

     lda     Sw0DB              ; Load accumulator with value in "Sw0DB"
                                ; variable
     beq     Sw0DB_CHK_DONE     ; If "Z" bit of "CCR is set, branch to
                                ; Sw0DB_CHK_DONE:
     dec     Sw0DB              ; Decrement "Sw0DB" variable

Sw0DB_CHK_DONE:

     lda     Sw1DB              ; Load accumulator with value in "Sw1DB"
                                ; variable
     beq     Sw1DB_CHK_DONE     ; If "Z" bit of "CCR is set, branch to
                                ; Sw1DB_CHK_DONE:
     dec     Sw1DB              ; Decrement "Sw1DB" variable

Sw1DB_CHK_DONE:

     lda     Sw2DB              ; Load accumulator with value in "Sw2DB"
                                ; variable
     beq     Sw2DB_CHK_DONE     ; If "Z" bit of "CCR is set, branch to
                                ; Sw2DB_CHK_DONE:
     dec     Sw2DB              ; Decrement "Sw2DB" variable

Sw2DB_CHK_DONE:

     lda     Sw3DB              ; Load accumulator with value in "Sw3DB"
                                ; variable
     beq     Sw3DB_CHK_DONE     ; If "Z" bit of "CCR is set, branch to
                                ; Sw3DB_CHK_DONE:
     dec     Sw3DB              ; Decrement "Sw3DB" variable

Sw3DB_CHK_DONE:


;****************************************************************************
; - Increment millisecond counter
;****************************************************************************

INC_mS:
     inc     mS                  ; Increment Millisecond counter
     lda     mS                  ; Load accumulator with value in
                                 ; Millisecond counter
     cmp     #$05                ; Compare it with decimal 5
     bne     TIM2CH0_ISR_DONE    ; If the Z bit of CCR is clear,
                                 ; branch to TIM2CH0_ISR_DONE:

;============================================================================
;************************** 5 Millisecond section ***************************
;============================================================================

;****************************************************************************
; - Check the value of the contact auto-repeat command counter variables,
;   if other than zero, decrement them.
;****************************************************************************

     lda     Sw0ARC             ; Load accumulator with value in "Sw0ARC"
                                ; variable
     beq     SW0ARC_CHK_DONE    ; If "Z" bit of "CCR is set, branch to
                                ; SW0ARC_CHK_DONE:
     dec     Sw0ARC             ; Decrement "Sw0ARC" variable

SW0ARC_CHK_DONE:

     lda     Sw1ARC             ; Load accumulator with value in "Sw1ARC"
                                ; variable
     beq     SW1ARC_CHK_DONE    ; If "Z" bit of "CCR is set, branch to
                                ; SW1ARC_CHK_DONE:
     dec     Sw1ARC             ; Decrement "Sw1ARC" variable

SW1ARC_CHK_DONE:

     lda     Sw2ARC             ; Load accumulator with value in "Sw2ARC"
                                ; variable
     beq     SW2ARC_CHK_DONE    ; If "Z" bit of "CCR is set, branch to
                                ; SW2ARC_CHK_DONE:
     dec     Sw2ARC             ; Decrement "Sw2ARC" variable

SW2ARC_CHK_DONE:

     lda     Sw3ARC             ; Load accumulator with value in "Sw3ARC"
                                ; variable
     beq     SW3ARC_CHK_DONE    ; If "Z" bit of "CCR is set, branch to
                                ; SW3ARC_CHK_DONE:
     dec     Sw3ARC             ; Decrement "Sw3ARC" variable

SW3ARC_CHK_DONE:


;****************************************************************************
; - Check the value of the contact auto-repeat counter variables, if other
;   than zero, decrement them.
;****************************************************************************

     lda     Sw0AR              ; Load accumulator with value in "Sw0AR"
                                ; variable
     beq     SW0AR_CHK_DONE     ; If "Z" bit of "CCR is set, branch to
                                ; SW0AR_CHK_DONE:
     dec     Sw0AR              ; Decrement "Sw0AR" variable

SW0AR_CHK_DONE:

     lda     Sw1AR              ; Load accumulator with value in "Sw1AR"
                                ; variable
     beq     SW1AR_CHK_DONE     ; If "Z" bit of "CCR is set, branch to
                                ; SW1AR_CHK_DONE:
     dec     Sw1AR              ; Decrement "Sw1AR" variable

SW1AR_CHK_DONE:

     lda     Sw2AR              ; Load accumulator with value in "Sw2AR"
                                ; variable
     beq     SW2AR_CHK_DONE     ; If "Z" bit of "CCR is set, branch to
                                ; SW2AR_CHK_DONE:
     dec     Sw2AR              ; Decrement "Sw2AR" variable

SW2AR_CHK_DONE:

     lda     Sw3AR              ; Load accumulator with value in "Sw3AR"
                                ; variable
     beq     SW3AR_CHK_DONE     ; If "Z" bit of "CCR is set, branch to
                                ; SW3AR_CHK_DONE:
     dec     Sw3AR              ; Decrement "Sw3AR" variable

SW3AR_CHK_DONE:


;****************************************************************************
; - Increment 5 millisecond counter
;****************************************************************************

INC_mSx5:
     clr     mS                  ; Clear Millisecond counter
     inc     mSx5                ; Increment 5 Millisecond counter
     lda     mSx5                ; Load accumulator with value in
                                 ; 5 Millesecond counter
     cmp     #$32                ; Compare it with decimal 50
     bne     TIM2CH0_ISR_DONE    ; If the Z bit of CCR is clear,
                                 ; branch to TIM2CH0_ISR_DONE:

;============================================================================
;************************* 250 Millisecond section **************************
;============================================================================

;***************************************************************************
; - Send the letter 'A' command to update the real time variables.
;***************************************************************************

     lda     SCS1                    ; Load accumulator with value in SCI
                                     ; Control Register 1
                                     ;(Clear transmitter empty bit)
                                     ;(Clears all by read)
     mov     #'A',SCDR               ; Move ASCII'A' to SCI Data Register
                                     ;(Transmitter is running, so data
                                     ; shift starts now!)
     clr     ByteCnt                 ; Clear "ByteCnt" variable
     mov     #$2A,ByteGoal           ; Move decimal 42 into "ByteGoal"
                                     ;(one more than the number of real
                                     ; time variables for display,41)

;**************************************************************************
; - Enable receiver full interrupt.
;**************************************************************************

     bset    SCRIE,SCC2     ; Set "SCRIE" bit of SCI Control Register 2
                            ;(SCRF enabled to generate CPU Interrupt)

UPDATE_DONE:
     clr     mSx5           ; Clear 5 Millisecond counter

TIM2CH0_ISR_DONE:
     pulh                  ; Pull value from stack to index register Hi byte
     rti                   ; Return from interrupt


;***************************************************************************
;
; ---------------- MS_ECU Serial Communications Interface -----------------
;
; Communications are established when a command character is sent, the
; particular character sets the mode:
;
; "A" = Receive realtime variables via txport.(53 bytes)(MS->MV)
;
;***************************************************************************

;***************************************************************************
;===========================================================================
; - SCI Receive Interrupt
;===========================================================================
;***************************************************************************

;***************************************************************************
; - Enter here when have received RS 232 byte
;   (SCRF bit of SCS1 set)
;***************************************************************************

SCIRCV_ISR:
     pshh                 ; Push value in index register Hi byte to Stack

;***************************************************************************
; - Clear status register to allow next interrupt
;***************************************************************************

     lda     SCS1     ; Load accumulator with value in SCI Register 1

;***************************************************************************
; - Transfer received byte from register to buffer
;***************************************************************************

     lda     SCDR        ; Load accumulator with value in SCI Data Register
     sta     readbuf     ; Copy to "readbuf" variable

;***************************************************************************
; - We are in "Display" mode, transfer received byte from buffer to real
;   time variable.
;***************************************************************************

RCVNG_VAR:
     ldx     ByteCnt           ; Load index register Lo byte with value in
                               ; "ByteCnt" variable
     clrh                      ; Clear index register hi byte
     lda     readbuf           ; Load accumulator with value in "readbuf"
     sta     secl,x            ; Copy to address at "secl", offset in index
                               ; register Lo byte

NEXT_RCV_BYTE:
     inc     ByteCnt          ; Increment value in "ByteCnt"(ByteCnt=ByteCnt+1)
     lda     ByteCnt          ; Load accumulator w3ith value in "ByteCnt"
     cmp     ByteGoal         ; Compare value in accumulator (ByteCnt")to
                              ; value in "ByteGoal" variable
     bls     SCIRCV_ISR_DONE  ; If C or Z bits of CCR are set,(A<=M),
                              ; branch to SCIRCV_ISR_DONE:

;***************************************************************************
; - Done receiving - kill receive interrupt enable
;***************************************************************************

     clr     ByteCnt           ; Clear "ByteCnt" variable
     bclr    SCRIE,SCC2        ; Clear "SCRIE" bit of SCI Control Register 2
                               ;(SCRF not enabled to generate CPU interrupt)

SCIRCV_ISR_DONE:
     pulh                ; Pull value from stack to index register Hi byte
     rti                 ; Return from interrupt


;**************************************************************************
;==========================================================================
; - SCI Transmit Interrupt
;==========================================================================
;**************************************************************************

;**************************************************************************
; - Enter here when the RS232 transmit buffer is empty
;   (SCTE bit of SCS1 is set)(Not used)
;**************************************************************************

SCITX_ISR:
     rti                ; Return from interrupt


;**************************************************************************
;==========================================================================
;- Dummy ISR vector ( This should never be called, but, just in case.)
;==========================================================================
;**************************************************************************

Dummy:
     rti     ; Return from interrupt


;***************************************************************************
;
; ---------------------------- SUBROUTINES --------------------------------
;
; - CALC_MAP
; - CALC_MAT
; - CALC_CLT
; - CALC_VOLTS
; - CALC_EGOV
; - CALC_AFR
; - CALC_LAMBDA
; - CALC_OP
; - CALC_FP
; - CALC_EGT
; - CALC_DUTY
; - CALC_RPM
; - CALC_RPM_HR
; - CALC_TRIM_ANG
; - CALC_DLY_ANG
; - CALC_TIM_ANG
; - CALC_MON_TIM_ANG
; - CALC_L_HR
; - DIV_BY_TEN
;
;-------------------------------------------------------------------------
;
; - CONV_8BIT_ASCII
; - CONV_16BIT_ASCII
; - SW0_CHK
; - SW1_CHK
; - DELAY300
; - PRNT_TOPLN_DSP
; - LOAD_SPACE
; - PRINT_LINE
; - GET_VECT_ADDR
; - ADD_A_TO_HX
; - LDA_W_HX_PL_A
; - VFD_START_TOP
; - VFD_START_BOT
; - VFD_SEND
; - Long Delay      (for VFD instruction/data transfer)
; - Short Delay     (for VFD instruction/data transfer)
; - VFD Display
; - CMP_BOTLIN
; - ORD_TABLE_FIND (Ordered Table Search)
; - LININTERP      (Linear Interpolation)
; - UDVD32         (32 x 16 divide)
; - DIVROUND       (Round after 16 x 8 division)
; - UMUL32         (16 x 16 multiply)
;
;***************************************************************************

;***********************************************************************************************
; - This subroutine uses "mapADC" and the "KPAFACTOR4250rjh" table to look up the value of "kpa"
;***********************************************************************************************

CALC_MAP:
     clrh                         ; Clear index register Hi byte
     lda     mapADC               ; Load accumulator with value in MAP ADC
     tax                          ; Copy to index register Lo byte
     lda     KPAFACTOR4250rjh,x   ; Load accumulator with value in KPAFACTOR4250rjh table
                                  ; (offset in index register Lo byte)
     sta     map                  ; Copy to Manifold Air Pressure in Kilopascals
     rts                          ; Return from subroutine

;***********************************************************************************************
; - This subroutine uses "matADC" and the "thermfactor" table to look up the value of "mat"
;***********************************************************************************************

CALC_MAT:
     clrh                      ; Clear index register Hi byte
     lda     matADC            ; Load accumulator with value in Manifold Air Temperature ADC
     tax                       ; Copy to index register Lo byte
     lda     thermfactor,x     ; Load accumulator with value in thermfactor table,
                               ; (offset in index register Lo byte(modified for Ford sensor)
     sta     mat               ; Copy to Manifold Air Temp in degreesF+40
     rts                       ; Return from subroutine

;***********************************************************************************************
; - This subroutine uses "cltADC" and the "thermfactor" table to look up the value of "clt"
;***********************************************************************************************

CALC_CLT:
     clrh                      ; Clear index register Hi byte
     lda     cltADC            ; Load accumulator with value in EngineTemperature ADC
     tax                       ; Copy to index register Lo byte
     lda     thermfactor,x     ; Load accumulator with value in thermfactor table,
                               ; (offset in index register Lo byte(modified for Ford sensor))
     sta     clt               ; Copy to Engine Temp in degreesF+40
     rts                       ; Return from subroutine

;***********************************************************************************************
; - This subroutine calculates battery voltage * 10 from battery
;   voltage ADC
;   Batt ADC/255 * 30 = battery voltage
;   (battADC * 300)/255  = volts(battery voltage * 10)(for integer math)
;**********************************************************************************************

CALC_VOLTS:

;***************************************************************************
; - Load variables and multiply to obtain the dividend
;***************************************************************************

     clr     tmp4       ; Clear tmp4
     lda     battADC    ; Load accumulator with value in Battery voltage ADC
     sta     tmp3       ; Copy to tmp3
     lda     #$1        ; Load accumulator with decimal 300 Hi byte(1 x 256 = 256)
     sta     tmp2       ; Copy to tmp4
     lda     #$2C       ; Load accumulator with decimal 300 Lo byte (44)
     sta     tmp1       ; Copy to tmp1
     jsr     UMUL32     ; Jump to subroutine at UMUL32: (16x16 multiply)

;****************************************************************************
; - Load 16 bit dividend ("battADC" x 300) and divide by 255
;****************************************************************************

     lda     tmp6       ; Load accumulator with value in tmp6 (result Mid Lo byte)
     psha               ; Push value in accumulator to stack
     pulh               ; Pull value from stack to index register Hi byte((A)to(H))
     lda     tmp5       ; Load accumulator with value in tmp5 (result Lo byte)
     ldx     #$FF       ; Load index register Lo byte with decimal 255
     div                ; Divide A<-(H:A)/(X);H<-Remainder
     jsr     DIVROUND   ; Jump to subroutine at DIVROUND:(round result)
     sta     volts      ; Copy to "volts"(battery voltage x 10)
     rts                ; Return from subroutine

;***********************************************************************************************
; - This subroutine calculates Exhaust Gas Oxygen sensor voltage * 10 from Exhaust Gas Oxygen
;   sensor ADC
;   egoADC/255 * 5 = ego voltage
;   (egoADC * 50)/255  = egoV(ego voltage * 10)(for integer math)
;**********************************************************************************************

CALC_EGOV:
     lda     #$32       ; Load accumulator with decimal 50(5 x 10 for integer math)
     tax                ; Transfer value in accumulator to index register Lo byte
     lda     egoADC     ; Load accumulator with value in "egoADC"
     mul                ; Multiply X:A<-(X)x(A)
     pshx               ; Push value in index register Lo byte to stack
     pulh               ; Pull value from stack to index register Hi byte((X)to(H))
     ldx     #$FF       ; Load index register Lo byte with decimal 255
     div                ; Divide A<-(H:A)/(X);H<-Remainder
     jsr     DIVROUND   ; Jump to subroutine at DIVROUND:(round result)
     sta     egoV       ; Copy to "egoV"(ego voltage  x 10)
     rts                ; Return from subroutine

;***********************************************************************************************
; - This subroutine uses "egoADC" to calculate "afr"(Air/Fuel ratio x 10) using the Innovate
;   LC-1 WB unit
;   AFR = 7.35 + (egoADC * 0.05898040)
;   For integer math:
;   AFR = (7.35*100 + (((egoADC*1000)*59)/10000))/10
;   AFR = (735 + (((egoADC*59000)/10000))/10
;***********************************************************************************************

CALC_AFR:

;***********************************************************************************************
; - Load variables and multiply to obtain the dividend(egoADC*59,000)
;***********************************************************************************************

     clr     tmp4       ; Clear tmp4
     lda     egoADC     ; Load accumulator with value in "egoADC"
     sta     tmp3       ; Copy to tmp3
     lda     #$E6       ; Load accumulator with decimal 59000 Hi byte(230 x 256 = 58880)
     sta     tmp2       ; Copy to tmp2
     lda     #$78       ; Load accumulator with decimal 59000 Lo byte(120)
     sta     tmp1       ; Copy to tmp1
     jsr     UMUL32     ; Jump to subroutine at UMUL32: (16x16 multiply)

;***********************************************************************************************
; - Load 24 bit dividend ("egoADC" x 59,000)
;***********************************************************************************************

     clr     intacc1    ; Clear intacc1
     lda     tmp7       ; Load accumulator with value in tmp7 (result Mid Hi byte)
     sta     intacc1+1  ; Copy to intacc1+1
     lda     tmp6       ; Load accumulator with value in tmp6 (result Mid Lo byte)
     sta     intacc1+2  ; Copy to intacc1+2
     lda     tmp5       ; Load accumulator with value in tmp5 (result Lo byte)
     sta     intacc1+3  ; Copy to intacc1+3

;***********************************************************************************************
; - Load 16 bit divisor (10,000) and divide
;***********************************************************************************************

     lda     #$27       ; Load accumulator with decimal 10,000 Hi byte(39 x 256 = 9984)
     sta     intacc2    ; Copy to intacc2
     lda     #$10       ; Load accumulator with decimal 10,000 Lo byte(16)
     sta     intacc2+1  ; Copy to intacc2+1 variable
     jsr     udvd32     ; jump to udvd32 subroutine(intacc1:32 = intacc1:32 / intacc2:16)

;***********************************************************************************************
; - Add 735 to the result
;   02:DF + intacc1+2:intacc1+3 = tmp1:tmp2(maximum 64762)
;***********************************************************************************************

     lda     intacc1+2  ; Load accumulator with value in intacc1+2 (result mid Lo byte)
     cmp     #$FC       ; Compare with decimal 252
     bhi     RAIL_AFR   ; If (A)>(M) branch to RAIL_AFR:
     bra     CONT_AFR   ; Branch to CONT_AFR:

RAIL_AFR:
     lda     #$FC       ; Load accumulator with decimal 252
     sta     intacc1+2  ; Copy to "intacc1+2"

CONT_AFR:
     lda     intacc1+3  ; Load accumulator with value in intacc1+3(result Lo byte)
     add     #$DF       ; Add (A)<-)(A)+(M) (decimal 735 Lo byte)(223)
     tax                ; Transfer value in accumulator to index register Lo byte
                        ; (temporary store result)
     lda     intacc1+2  ; Load accumulator with value in intacc1+3(result Mid Lo byte)
     adc     #$02       ; Add with carry (A)<-(A)+(M)+(C)(decimal 735 Hi byte(2 x 256 = 512)
     sta     tmp1       ; Copy result to "tmp1"(result Hi byte)
     stx     tmp2       ; Result Lo byte

;***********************************************************************************************
; - Divide the result by 10
;   afr(AFR x 10) = tmp1:tmp2 / 10
;***********************************************************************************************

     lda     tmp1       ; Load accumulator with value in "tmp1" result Hi byte)
     psha               ; Push value in accumulator onto stack
     pulh               ; Pull value from stack to index register Hi byte(A to H)
     lda     tmp2       ; Load accumulator with value in "tmp2"(result Lo byte)
     ldx     #$0A       ; Load index register Lo byte with decimal 10
     div                ; Divide A<-(H:A)/(X);H<-Remainder
     jsr     DIVROUND   ; Jump to subroutine at DIVROUND:(round result)
     sta     afr        ; Copy result to "afr"(AFR x 10)
     rts                ; Return from subroutine


;***********************************************************************************************
; - This subroutine uses "afr" to calculate the value of "lambda"(LAMBDA x 100)
;   LAMBDA = AFR / 14.7
;   For integer math:
;   lambda = ((AFR*10) / (14.7*10))*100
;   lambda = (afr/147)*100
;   lambda = (afr*100)/147
;***********************************************************************************************

CALC_LAMBDA:
     lda     #$64       ; Load accumulator with decimal 100
     tax                ; Transfer value in accumulator to index register Lo byte
     lda     afr        ; Load accumulator with value in "afr"
     mul                ; Multiply X:A<-(X)x(A)
     pshx               ; Push value in index register Lo byte to stack(result Hi byte)
     pulh               ; Pull value from stack to index register Hi byte((X)to(H))
     ldx     #$93       ; Load index register Lo byte  with decimal 147(14.7*10)
     div                ; Divide A<-(H:A)/(X);H<-Remainder
     jsr     DIVROUND   ; Jump to subroutine at DIVROUND:(round result)
     sta     lambda     ; Copy to "lambda"(LAMBDA x 10)
     rts                ; Return from subroutine


;***********************************************************************************************
; - This subroutine uses "opADC" to calculate the value of "oprs"(engine oil pressure in PSI)
;   Oil pressure and Fuel pressure transducers 1-5V, 0-100 PSI
;***********************************************************************************************

CALC_OP:

;***********************************************************************************************
; - Linear Interpolation of ADC value:
;   tmp1 = 51 (1 volt)
;   tmp2 = 255 ( 5 volts)
;   tmp3 = Lo end of scale
;   tmp4 = Hi end of scale
;   tmp5 = ADC value
;   tmp6 = Result
;***********************************************************************************************

     mov     #$33,tmp1         ; Move decimal 51 to "tmp1"(1 volt)
     mov     #$FF,tmp2         ; Move decimal 255 to "tmp2"(5 volts)
     mov     #$64,tmp4         ; Move decimal 100 to "tmp4" (100 PSI)
     clr     tmp3              ; Clear "tmp3"(0 PSI)
     mov     opADC,tmp5        ; Move value in "opADC" to "tmp5"(ADC value)
     jsr     lininterp         ; Jump to subroutine at linenterp:
     lda     tmp6              ; Load accumulator with value in "tmp6"(Result)
     sta     oprs              ; Copy to "oprs"(engine oil pressure in PSI)
     rts                       ; Return from subroutine


;***********************************************************************************************
; - This subroutine uses "fpADC" to calculate the value of "fprs"(Fuel pressure in PSI)
;   Oil pressure and Fuel pressure transducers 1-5V, 0-100 PSI
;***********************************************************************************************

CALC_FP:

;***********************************************************************************************
; - Linear Interpolation of ADC value:
;   tmp1 = 51 (1 volt)
;   tmp2 = 255 ( 5 volts)
;   tmp3 = Lo end of scale
;   tmp4 = Hi end of scale
;   tmp5 = ADC value
;   tmp6 = Result
;***********************************************************************************************

     mov     #$33,tmp1         ; Move decimal 51 to "tmp1"(1 volt)
     mov     #$FF,tmp2         ; Move decimal 255 to "tmp2"(5 volts)
     mov     #$64,tmp4         ; Move decimal 100 to "tmp4" (100 PSI)
     clr     tmp3              ; Clear "tmp3"(0 PSI)
     mov     fpADC,tmp5        ; Move value in "fpADC" to "tmp5"(ADC value)
     jsr     lininterp         ; Jump to subroutine at linenterp:
     lda     tmp6              ; Load accumulator with value in "tmp6"(Result)
     sta     fprs              ; Copy to "fprs"(Fuel pressure in PSI)
     rts                       ; Return from subroutine

;***********************************************************************************************
; - This subroutine uses "egtADC" to calculate the value of "egtH:egtL"(Exhaust Gas Temperature
;   in degrees F)
;   EGT amplifier 0.0027v @ 0C (32F), 5.022v @ 720C (1328F) Resolution ~5 degrees F
;   egtH:egtL = (((egtADC-0)*(1328-32))/(255-0))+32
;   egtH:egtL = ((egtADC*(1296)/255)+32)
;   egtH:egtL = (egtADC*5.082352941)+32)
;   For integer math:
;   egtH:egtL = (egtADC*50824/10000)+32
;***********************************************************************************************

CALC_EGT:

;***********************************************************************************************
; - Load variables and multiply to obtain the dividend("egtADC" x 50824)
;***********************************************************************************************

     clr     tmp4       ; Clear tmp4
     lda     egtADC     ; Load accumulator with value in "egtADC"
     sta     tmp3       ; Copy to tmp3
     lda     #$C6       ; Load accumulator with decimal 50824 Hi byte(198 x 256 = 50688)
     sta     tmp2       ; Copy to tmp2
     lda     #$88       ; Load accumulator with decimal 50824 Lo byte(136)
     sta     tmp1       ; Copy to tmp1
     jsr     UMUL32     ; Jump to subroutine at UMUL32: (16x16 multiply)

;***********************************************************************************************
; - Load 24 bit dividend ("egtADC" x 50824)
;***********************************************************************************************

     clr     intacc1    ; Clear intacc1
     lda     tmp7       ; Load accumulator with value in tmp7 (result Mid Hi byte)
     sta     intacc1+1  ; Copy to intacc1+1
     lda     tmp6       ; Load accumulator with value in tmp6 (result Mid Lo byte)
     sta     intacc1+2  ; Copy to intacc1+2
     lda     tmp5       ; Load accumulator with value in tmp5 (result Lo byte)
     sta     intacc1+3  ; Copy to intacc1+3

;***********************************************************************************************
; - Load 16 bit divisor (10,000) and divide
;***********************************************************************************************

     lda     #$27       ; Load accumulator with decimal 10,000 Hi byte(39 x 256 = 9984)
     sta     intacc2    ; Copy to intacc2
     lda     #$10       ; Load accumulator with decimal 10,000 Lo byte(16)
     sta     intacc2+1  ; Copy to intacc2+1 variable
     jsr     udvd32     ; jump to udvd32 subroutine(intacc1:32 = intacc1:32 / intacc2:16)

;***********************************************************************************************
; - Add 32 to the result
;   #$20 + intacc1+2:intacc1+3 = egtH:egtL
;***********************************************************************************************


     lda     intacc1+3  ; Load accumulator with value in "intacc1+3"(result Lo byte)
     add     #$20       ; Add (A)<-)(A)+(M) (decimal 32)
     sta     egtL       ; copy to "egtL"(exhaust gas temperature Lo byte)
     lda     intacc1+2  ; Load accumulator with value in "intacc1+2"(result Hi byte)
     adc     #$00       ; Add with carry (A)<-(A)+(M)+(C)(just the carry)
     sta     egtH       ; copy to "egtH"(exhaust gas temperature Hi byte)
     rts                ; Return from subroutine

;***********************************************************************************************
; - Duty cycle % integer rounded, "Duty", is calculated by multiplying the pulse width in mmsec
;   by 10000 and dividing the product by the period between injector bank firing events.
;   the time period between Injector bank firing events is determined from the injector firing
;   strategy. MSnS351WM uses 2 squirts alternating which fires a bank every 8 ignition events.
;   The period between injector firing events will then be tachH:tachL x 2. Because tachH:tachL
;   x 8 can become too large for UDV32 pw x 10000 is divided by 2 instead.
;
;   ((pw x 10000) / 8) / tachH:tachL = duty
;   or
;   (pw x 1250) / tachH:tachL = duty
;
;***********************************************************************************************

CALC_DUTY:

;***************************************************************************
; - Load variables and multiply to obtain the dividend
;***************************************************************************

     clr     tmp4       ; Clear tmp4
     lda     pw         ; Load accumulator with value in "pw"
     sta     tmp3       ; Copy to tmp3
     lda     #$4        ; Load accumulator with decimal 4 (4 x 256 = 1024)
     sta     tmp2       ; Copy to tmp4
     lda     #$E2       ; Load accumulator with decimal 226 (226 + 1024 = 1250)
     sta     tmp1       ; Copy to tmp1
     jsr     UMUL32     ; Jump to subroutine at UMUL32: (16x16 multiply)

;****************************************************************************
; - Load 24 bit dividend (1250 x "pw")
;****************************************************************************

     clr     intacc1    ; Clear intacc1
     lda     tmp7       ; Load accumulator with value in tmp7 (result Mid Hi byte)
     sta     intacc1+1  ; Copy to intacc1+1
     lda     tmp6       ; Load accumulator with value in tmp6 (result Mid Lo byte)
     sta     intacc1+2  ; Copy to intacc1+2
     lda     tmp5       ; Load accumulator with value in tmp5 (result Lo byte)
     sta     intacc1+3  ; Copy to intacc1+3

;****************************************************************************
; - Load 16 bit divisor (tachH:tachL) and divide
;****************************************************************************

     lda     tachH      ; Load accumulator with "tachH"(Pip period) Hi byte
     sta     intacc2    ; Copy to intacc2
     lda     tachL      ; Load accumulator with "tachL"(Pip period) Lo byte
     sta     intacc2+1  ; Copy to intacc2+1 variable
     jsr     udvd32     ; jump to udvd32 subroutine
                        ; (intacc1:32 = intacc1:32 / intacc2:16)
     lda     intacc1+3  ; Load accumulator with value in intacc1+3 variable
                        ; (8-bit result)
     ldx     intacc2+1  ; Load index retgister Lo byte with value in intacc2+1
                        ; (8 bit remainder)
     jsr     DIVROUND   ; Jump to "DIVROUND" subroutine (round result)


;****************************************************************************
; - Save the result as "duty"
;****************************************************************************

     sta     duty       ; Copy to "duty"(DUTY CYCLE%)
     rts                ; Return from subroutine


;***********************************************************************************************
; - This subroutine uses "rpm20" to calculate engine RPM to 4 digits for display
;***********************************************************************************************

CALC_RPM:
     ldx     rpm20        ; Load index register Lo byte with value in engine RPM/20
     lda     #$14         ; Load accumulator with decimal 20
     mul                  ; Multiply (X:A)<-(X)x(A)
     sta     rpmL         ; Copy value in accumulator to engine RPM Hi Byte
     pshx                 ; Push value in index register Lo byte to stack
     pula                 ; Pull value in stack to accumulator(X to A)
     sta     rpmH         ; Copy to engine RPM Hi byte
     rts                  ; Return from subroutine

;***********************************************************************************************
; - This subroutine uses "tachH:tachL" to calculate engine RPM to 4 digits for display
;   "tachH:tachL" is the 1uS time period between PIP(IRQ) signals. Rollover occurs at
;   305.18 RPM 6 cyl, and 228.885 RPM 8 cyl so we ignore any engine speeds less than 320 RPM
;   for 6 cyl($140) and 240 RPM for 8 cyl($F0)
;***********************************************************************************************

CALC_RPM_HR:

;***********************************************************************************************
;
; -------------------------------------- RPM CALCULATION ---------------------------------------
;
; RPM = CONSTANT/PERIOD
; Where:
; RPM     = Engine RPM
; RPM_K   = 16 bit constant using 1uS clock tick (1mhz)
;           ((1,000,000 tick per second * 60 seconds per minute)/number of cylinders
; RPM_P   = 16 bit period count between IRQ pulsed lines in 1uS resolution
;
;   RPM_K
;   ----- = RPM
;   RPM_P
;

; Where:
; rpmhrH:rpmhrL             = Engine RPM
; rpmK:rpmK+1:rpmK+2:rpmK+3 = 32 bit constant using 1uS clock tick
; tachH:tachL               = 16 bit period count between IRQ pulsed lines in 1uS resolution
;
;   rpmk:rpmk+1:rpmk+2:rpmk+3
;   ------------------------- = rpmhrH:rpmhrL
;       tachH:tachL
;
; 6cyl 4stroke RPMK = ((1,000,000*60)/3) = 20,000,000($01312D00)
; 8cyl 4stroke RPMK = ((1,000,000*60)/4) = 15,000,000($00E4E1C0)
; 6cyl RPM resolution is ~.05@~1000, ~.20@~2000, ~.76@~3000, and ~1.28@~5000
; 8cyl RPM resolution is ~.06@~1000, ~.27@~2000, ~.61@~3000, and ~1.72@~5000
;***********************************************************************************************

     lda     rpm20             ; Load accumulator with value in "rpm20"
     cmp     #$10              ; Compare with decimal 16(320 RPM)
     bhs     RPM_COMP          ; If (A)=>(M) branch to RPM_COMP
     clr     rpmhrL            ; Clear "rpmhrL"
     clr     rpmhrH            ; Clear "rpmhrH"
     bra     RPM_CALC_DONE     ; Branch to RPM_CALC_DONE:

RPM_COMP:

     lda     #$00          ; Load accumulator with $00 ("rpmK" 8 cyl)
     sta     intacc1       ; Copy to intacc1 variable
     lda     #$E4          ; Load accumulator with $E4 ("rpmK+1" 8 cyl)
     sta     intacc1+1     ; Copy to intacc1+1 variable
     lda     #$E1          ; Load accumulator with $E1 ("rpmK+2" 8 cyl)
     sta     intacc1+2     ; Copy to "intacc1+2
     lda     #$C0          ; Load accumulator with $C0 ("rpmK+3" 8 cyl)
     sta     intacc1+3     ; Copy to "intacc1+3"
     ldhx    tachH         ; Load index register with value in tachH:tachL
     sthx    intacc2       ; Copy value in index register to intacc2 variable
     jsr     udvd32        ; Jump to subroutine udvd32 (32x16 divide)
     lda     intacc1+3     ; Load accumulator with value in intacc1+3(result Lo byte)
     sta     rpmhrL        ; Copy to "rpmhrL"
     lda     intacc1+2     ; Load accumulator with value in intacc1+2(result Mid Lo byte)
     sta     rpmhrH        ; Copy to "rpmhrH"

RPM_CALC_DONE:
     rts                   ; Return from subroutine


;***********************************************************************************************
; - This subroutine calculates "trmAng"(Ignition Trim Angle). Ignition Trim Angle of 10 degrees
;   (pot mid point) is 0 degrees ignition trim (no change). Ignition Trim Angle of 20 degrees
;   (pot full CW)is 10 degrees ignition trim advance. Ignition Trim Angle of 0 degrees
;   (pot full CCW) is 10 degrees ignition trim retard.
;
;    "trmAngFac" is 57 to 0 from itrmADC 0 to 255, 28 being mid point. "trmAng" can be
;    calculated by inverting "trmAngFac" and interpolating.
;
;***********************************************************************************************

;***********************************************************************************************
; - Linear Interpolation of ADC value:
;   tmp1 = 0 (trmAngFac = 0)
;   tmp2 = 57 (trmAngFac = 57)
;   tmp3 = Lo end of scale (0 degrees)
;   tmp4 = Hi end of scale (20 degrees)
;   tmp5 = inverted "trmAngFac" value
;   tmp6 = Result
;***********************************************************************************************

CALC_TRIM_ANG:
     mov     #$0,tmp1          ; Move decimal 0 to "tmp1"(trmAngFac = 0)
     mov     #$39,tmp2         ; Move decimal 57 to "tmp2"(trmAngFac = 57)
     mov     #$14,tmp4         ; Move decimal 20 to "tmp4" (20 degrees)
     clr     tmp3              ; Clear "tmp3"(0 degrees)
     lda     #$39              ; Load accumulator with decimal 57
     sub     trmAngFac         ; Subtract A<(A)-(M)(invert "trmAngFac")
     sta     tmp5              ; Store result in "tmp5"
     jsr     lininterp         ; Jump to subroutine at linenterp:
     lda     tmp6              ; Load accumulator with value in "tmp6"(Result)
     sta     trmAng            ; Copy to "trmAng"(Ignition Trim Angle)
     rts                       ; Return from subroutine


;***********************************************************************************************
; - This subroutine calculates "dlyAng"(Ignition Delay Angle)
;   dlyAng = dlyAngFac / 255 x 90
;   dlyAng = dlyAngFac * 90 /255 (for integer math)
;***********************************************************************************************

CALC_DLY_ANG:
     lda     #$5A           ; Load accumulator with decimal 90
     tax                    ; Transfer value in accumulator to index register Lo byte
     lda     dlyAngFac      ; Load accumulator with value in "dlyAngFac"
     mul                    ; Multiply X:A<-(X)x(A)
     pshx                   ; Push value in index register Lo byte to stack
     pulh                   ; Pull value from stack to index register Hi byte((X)to(H))
     ldx     #$FF           ; Load index register Lo byte with decimal 255
     div                    ; Divide A<-(H:A)/(X);H<-Remainder
     jsr     DIVROUND       ; Jump to subroutine at DIVROUND:(round result)
     sta     dlyAng         ; Copy to "dlyAng"(Ignition Delay Angle)
     rts                    ; Return from subroutine


;***********************************************************************************************
; - This subroutine calculates "timAng"(Ignition Timing Angle)
;   timAng = 90 - dlyAng + 10
;***********************************************************************************************

CALC_TIM_ANG:
     lda     #$5A       ; Load accumulator with decimal 90
     sub     dlyAng     ; Subtract A<(A)-(M)
     add     #$0A       ; Add A<-(A)+(M) decimal 10
     sta     timAng     ; Copy result to "timAng"(Ignition Timing Angle)
     rts                ; Return from subroutine

;***********************************************************************************************
;
; ------------------------- Computation of Monitored Ignition Timing ---------------------------
;
; In the Keyboard interrupt section, we grabbed a timestamp when the coil was actually fired.
; From this, we subtracted the timestamp of the PIP signal to determine the Monitor Delay
; Period. (this should be the same as the Delay Period.) We can do the spark calculations
; in reverse to determine the Monitor Delay Angle Factor.
; The formula for determining the Delay Period is:
;
; PIP Period(tachH:tachL) * Delay Angle Factor / PIP Angle Factor = Delay Period.
; So, the Monitor Period * PIP Angle Factor / PIP Period = Monitor Delay Angle Factor.
; The Monitor Delay Angle Factor * PIP Angle  / PIP Angle Factor = Monitor Delay Angle.
; The PIP Angle - Monitor Delay Angle = Monitor Firing Angle
; The Monitor Firing Angle + Static Timing Angle(10 degrees) = Monitored Ignition Timing
; in degrees BTDC.
;
; monpH:monpL * $01:00 / tachH:tachL = monDlyAngFac
; monDlyAngFac * PIP_ANGLE / $01:00 = monDlyAng
; PIP_ANGLE - monDlyAng = monFrngAng
; monFrngAng + STATIC_TIMING_ANGLE = monTimAng
;
;***********************************************************************************************

;***********************************************************************************************
; - Calculate Monitored ignition timing in degrees BTDC.
;***********************************************************************************************

CALC_MON_TIM_ANG:

;***********************************************************************************************
; - Check for minimum RPM for ignition periods to be meaningfull for ignition calculations
;***********************************************************************************************

     lda     rpm20             ; Load accumulator with value in "rpm20"
     cmp     #$10              ; Compare with decimal 16(320 RPM)
     bhs     LOAD_PIP_PRD      ; If (A)=>(M) branch to LOAD_PIP_PRD
     clr     monTimAng         ; Clear "monTimAng"
     bra     CALC_MTA_DONE     ; Branch to CALC_MTA_DONE:


;***********************************************************************************************
; - Load 16 bit divisor(PIP Period (tachH:tachL))
;***********************************************************************************************

LOAD_PIP_PRD:
     lda     tachH        ; Load accumulator with value in "tachH"(PIP period Hi byte)
     sta     intacc2      ; Copy to intacc2 variable
     lda     tachL        ; Load accumulator with value in "tachL"(PIP period Lo byte)
     sta     intacc2+1    ; Copy to intacc2+1 variable
     clr     intacc2+2    ; Clear intacc2+2 variable
     clr     intacc2+3    ; Clear intacc2+3 variable

;***********************************************************************************************
; - Load 24 bit dividend(Monitor Period * 256) and divide
;***********************************************************************************************

     clr     intacc1         ; Clear intacc1 variable
     lda     monpH           ; Load accumulator with value in "monpH"
     sta     intacc1+1       ; Copy  to intacc1+1 variable
     lda     monpL           ; Load accumulator with value in "monpL"
     sta     intacc1+2       ; Copy to intacc1+2 variable
     clr     intacc1+3       ; Clear intacc1+3 variable(multiply by 256)
     jsr     udvd32          ; jump to udvd32 subroutine (intacc1:32 = intacc1:32 / intacc2:16)
     lda     intacc1+3       ; Load accumulator with value in intacc1+3 variable
                             ; (8-bit result)
     ldx     intacc2+1       ; Load index retgister Lo byte with value in intacc2+1
                             ; (8 bit remainder)
     sta     monDlyAngFac    ; Copy result to Monitor Delay Angle Factor variable

;***********************************************************************************************
; - Multiply by PIP Angle and divide by 256(8cyl PIP Angle = 90)
;***********************************************************************************************

     ldx     #$5A            ; Load index register Lo byte with "PIP Angle"(decimal 90)
     mul                     ; Multiply (X:A)<-(X)*(A)
     stx     monDlyAng       ; Copy to Monitor Delay Angle variable

;***********************************************************************************************
; - Calculate "monTimAng"
;***********************************************************************************************

     lda     #$5A            ; Load index register Lo byte with "PIP Angle"(decimal 90)
     sub     monDlyAng       ; Subtract (A<-(A)-(M))
     sta     monFrngAng      ; Copy to Monitor Firing Angle variable
     add     #$0A            ; Add (A)<-(A)+(M) Static Timing Angle (always 10 degrees)
     sta     monTimAng       ; Copy to Measured ignition timing in degrees BTDC variable

CALC_MTA_DONE:
     rts                     ; Return from subroutine

;***********************************************************************************************
; - This subroutine calculates "ltrHrH:ltrHrL"(Fuel burn in Litres/Hour over a 1 second period
;   x 10) using the variables "fdSecH:fdSecL" which is the accumulated fuel delivery injector
;   pulse width over a 1 second period, and time.
;   Fuel Burn Rate = inj on time per sec * inj bank flow litres per sec * 3600
;   Fuel Burn Rate = (fdSecH:fdSecL) * .000013313 * 3600
;   Fuel Burn Rate = (fdSecH:fdSecL) * .0479268
;   ltrHr = (fdSecH:fdSecL) * .0479268
;   ltrHr = (fdSecH:fdSecL) * 479 / 1000(for integer math)
;
; REVISON!!
;   WitchHunter Performance actual flow data:
;   Bank CC/min = 828.5
;   Average pressure across the injectors (calculated from data logs) = 45PSI
;   Adjusted flow rate Bank CC/min = 842
;   Fuel Burn Rate = inj on time per sec * inj bank flow litres per sec * 3600
;   Fuel Burn Rate = (fdSecH:fdSecL) * .000014033 * 3600
;   Fuel Burn Rate = (fdSecH:fdSecL) * .0505
;   ltrHr = (fdSecH:fdSecL) * .0505
;   ltrHr = (fdSecH:fdSecL) * 505 / 1000(for integer math)
;
; - REVISED 4/13/12: Configurable constants from Megasquirt, BnkflowHmv:BnkflowLmv, are now
;   used instead of hard coded flow rate of 359 to fine tune calculations.
;
;***********************************************************************************************

CALC_L_HR:

;***************************************************************************
; - Load variables and multiply to obtain the dividend
;***************************************************************************

     lda     fdSecH     ; Load accumulator with value in "fdSecH"
     sta     tmp4       ; Copy to tmp4
     lda     fdSecL     ; Load accumulator with value in "fdSecL"
     sta     tmp3       ; Copy to tmp3
;*     lda     #$01       ; Load accumulator with decimal 505 Hi byte
;*                        ; (1 x 256 = 256)
     lda     BnkflowHmv ; Load accumulator with bank flow rate Hi byte
     sta     tmp2       ; Copy to tmp4
;*     lda     #$F9       ; Load accumulator with decimal 505 Lo byte
;*                        ; (249)
     lda     BnkflowLmv ; Load accumulator with bank flow rate Lo byte
     sta     tmp1       ; Copy to tmp1
     jsr     UMUL32     ; Jump to subroutine at UMUL32: (16x16 multiply)

;****************************************************************************
; - Load 32 bit dividend (fdSecH:fdSecL x 355)
;****************************************************************************

     lda     tmp8       ; Load accumulator with value in tmp8 (result Hi byte)
     sta     intacc1    ; Copy to intacc1
     lda     tmp7       ; Load accumulator with value in tmp7 (result Mid Hi byte)
     sta     intacc1+1  ; Copy to intacc1+1
     lda     tmp6       ; Load accumulator with value in tmp6 (result Mid Lo byte)
     sta     intacc1+2  ; Copy to intacc1+2
     lda     tmp5       ; Load accumulator with value in tmp5 (result Lo byte)
     sta     intacc1+3  ; Copy to intacc1+3

;****************************************************************************
; - Load 16 bit divisor (1000) and divide
;****************************************************************************

     lda     #$03       ; Load accumulator with decimal 1000 Hi byte
                        ; (3 x 256 = 768)
     sta     intacc2    ; Copy to intacc2
     lda     #$E8       ; Load accumulator with decimal 1000 Lo byte
                        ; (232)
     sta     intacc2+1  ; Copy to intacc2+1 variable
     jsr     udvd32     ; jump to udvd32 subroutine
                        ; (intacc1:32 = intacc1:32 / intacc2:16)

;****************************************************************************
; - Save the result as "ltrHrH:ltrHrL"
;****************************************************************************

     lda     intacc1+3  ; Load accumulator with value in intacc1+3
                        ; (quotient Lo byte)
     sta     ltrHrL     ; Copy to "ltrHrH"(Litre/Hr x10 Hi byte)
     lda     intacc1+2  ; Load accumulator with value in intacc1+2
                        ; (quotient Mid Lo byte)
     sta     ltrHrH     ; Copy to "ltrHrH"(Litre/Hr x10 Lo byte)
     rts                ; Return from subroutine


;***********************************************************************************************
; - This subroutine divides a 16 bit variable loaded in "intacc1+2:intacc1+3" by 10 so the
;   result can fit in the 4 digit display. ( uS resolution is now in mmS)
;***********************************************************************************************

DIV_BY_TEN:

;****************************************************************************
; - Clear the dividend Hi and Mid Hi bytes
;****************************************************************************

     clr     intacc1     ; Clear intacc1
     clr     intacc1+1   ; Clear intacc1+1

;****************************************************************************
; - Load the 8 bit divisor and divide by 10. The 16 bit quotient remains in
;   "intacc1+2:intacc1+3" for direct use in the "CONV_16BIT_ASCII" subroutine
;****************************************************************************

     clr     intacc2       ; Clear "intacc2"
     lda     #$0A          ; Load accumulator with decimal 10
     sta     intacc2+1     ; Copy to "intacc2+1
     jsr     udvd32        ; Jump to udv32:
     rts                   ; Return from subroutine

;****************************************************************************
; - This subroutine takes a byte value in the accumulator, transfers it to
;   the index register Lo byte, and converts it to a 3 variable string,
;   stored temporarily in variables "AC_100", AC_10", and "AC_1"
;****************************************************************************

CONV_8BIT_ASCII:
     clrh                      ; Clear index register hi byte
     lda     disVal            ; Load accumulator with value in "disVal"
     tax                       ; Transfer value in accumulator to index
                               ; register Lo byte(8 bit value)
     lda     ASCII_CONV_100,x  ; Load accumulator with value in
                               ; ASCII_CONV_100 table, offset in index
                               ; register Lo byte(ASCII 100s value)
     sta     AC_100            ; Copy to "AC_100" variable
     lda     ASCII_CONV_10,x   ; Load accumulator with value in
                               ; ASCII_CONV_10 table, offset in index
                               ; register Lo byte(ASCII 10s value)
     sta     AC_10             ; Copy to "AC_10" variable
     lda     ASCII_CONV_1,x    ; Load accumulator with value in
                               ; ASCII_CONV_1 table, offset in index
                               ; register Lo byte(ASCII 1s value)
     sta     AC_1              ; Copy to "AC_1" variable
     rts                       ; Return from subroutine


;***********************************************************************************************
;  - This subroutine takes a 16 bit value previously stored in "intacc1+2:intacc1+3"
;    and converts it to a 4 variable string, stored temporarily as "thousands", "hundreds",
;    "tens", and "ones"
;***********************************************************************************************

CONV_16BIT_ASCII:

     clr     intacc1    ; Clear intacc1
     clr     intacc1+1  ; Clear intacc1+1

;***********************************************************************************************
; - Load 16 bit divisor (decimal 1000) and divide (1000s digit)
;***********************************************************************************************

     lda     #$3        ; Load accumulator with decimal 1000 Hi byte(3 x 256 = 768)
     sta     intacc2    ; Copy to intacc2
     lda     #$E8       ; Load accumulator with decimal 1000 Lo byte(232)
     sta     intacc2+1  ; Copy to intacc2+1 variable
     jsr     udvd32     ; jump to udvd32 subroutine
                        ; (intacc1:32 = intacc1:32 / intacc2:16)

;***********************************************************************************************
; - Convert the result to ASCII and save the 1000s digit
;***********************************************************************************************

     lda     intacc1+3         ; Load accumulator with value in intacc1+3(result  Lo byte)
     sta     disval            ; Copy to "disVal"
     jsr     CONV_8BIT_ASCII   ; Jump to subroutine at CONV_8BIT_ASCII:
     lda     AC_1              ; Load accumulator with value in "AC_1"(1s col)
     sta     thousands         ; Copy to "thousands"

;***********************************************************************************************
; - Load remainder as 16 bit dividend and divide by 100 (100s digit)
;***********************************************************************************************

     clrh               ; Clear index register Hi byte
     lda     intacc2    ; Load accumulator with value in intacc2(remainder Hi byte)
     psha               ; push value in accumulator onto stack
     pulh               ; pull value from stack to index register Hi byte(A to H)
     lda     intacc2+1  ; Load accumulator with value in intacc2+1(remainder Lo byte)
     ldx     #$64       ; Load index register Lo byte with decimal 100
     div                ; A<-(H:A)/(X):(H)<-Rem
     sta     disval     ; Copy to "disVal"


;***********************************************************************************************
; - Save remainder for next calculation
;***********************************************************************************************

     pshh               ; Push value in index register Hi byte to stack
     pulx               ; Pull value from stack to index register Lo byte(H to A)
     stx     tmp1       ; Copy value in index register Lo byte to tmp1 variable

;***********************************************************************************************
; - Convert the result to ASCII and save the 100s digit
;***********************************************************************************************

     jsr     CONV_8BIT_ASCII   ; Jump to subroutine at CONV_8BIT_ASCII:
     lda     AC_1              ; Load accumulator with value in "AC_1"(1s col)
     sta     hundreds          ; Copy to "hundreds"

;***********************************************************************************************
; - Load remainder as 8 bit dividend and divide by 10 (10s digit)
;***********************************************************************************************

     clrh               ; clear index register Hi byte
     lda     tmp1       ; Load accumulator with value in tmp1(remainder)
     ldx     #$A        ; Load index register Lo byte with decimal 10
     div                ; A<-(H:A)/(X):(H)<-Rem
     sta     disval     ; Copy to "disVal"

;***********************************************************************************************
; - Save remainder for next calculation
;***********************************************************************************************

     pshh               ; Push value in index register Hi byte to stack
     pulx               ; Pull value from stack to index register Lo byte (H to X)
     stx     tmp1       ; Copy value in index register Lo byte to tmp1 variable

;***********************************************************************************************
; - Convert the result to ASCII and save the 10s digit
;***********************************************************************************************

     jsr     CONV_8BIT_ASCII   ; Jump to subroutine at CONV_8BIT_ASCII:
     lda     AC_1              ; Load accumulator with value in "AC_1"(1s col)
     sta     tens              ; Copy to "tens"

;***********************************************************************************************
; - Convert the remainder to ASCII and save the 1s digit
;***********************************************************************************************

     lda     tmp1              ; Load accumulator with value in tmp1(remainder)
     sta     disval            ; Copy to "disVal"
     jsr     CONV_8BIT_ASCII   ; Jump to subroutine at CONV_8BIT_ASCII:
     lda     AC_1              ; Load accumulator with value in "AC_1"(1s col)
     sta     ones              ; Copy to "ones"
     rts                       ; Return from subroutine


;***********************************************************************************************
; - This subroutine checks the state of the Display Screen next
;   button on PTA0 and updates the switch status flag.
;   The switch status flag is cleared every pass through the main loop after
;   the routine relevent to that flag is completed.
;   Edge detection is provided from both open to closed, and closed to open.
;   Auto-repeat at 2HZ is commanded as long as the contacts remain closed
;   for a period of 1 second or more.
;   Auto-repeat is prevented in the open state.
;***********************************************************************************************

SW0_CHK:
     lda     Sw0DB                      ; Load accumulator with value in Switch
                                        ; #0 de-bounce counter variable
     bne     SW0_CHK_DONE               ; If Z bit of CCR is clear, branch to
                                        ; SW0_CHK_DONE: ("Sw0DB" not = 0,
                                        ; de-bounce in progress, skip over)
     brset   Sw0,porta,SW0_OPN          ; If "Sw0" bit of Port A is set,(Hi)
                                        ; branch to Sw0_OPN: (contacts are open)
     brset   Sw0LP,LPflags,SW0_ARC_CHK  ; If "Sw0LP" bit of "LPflags"
                                        ; variable is set, branch to SW0_ARC_CHK:
                                        ; (contacts closed, bit is already set,
                                        ; check for auto-repeat command)
     mov     #$64,Sw0DB                 ; Move decimal 100 into Switch #0
                                        ; de-bounce counter variable (100mS)
     bset    Sw0LP,LPflags              ; Set "Sw0LP" bit of "LPflags" variable
     jmp     SW0_CLS                    ; Jump to SW0_CLS:

SW0_ARC_CHK:
     brset   Sw0LP,ARCflags,SW0_ARC_PROG  ; If "Sw0LP" bit of "ARCflags"
                                          ; variable is set, branch to SW0_ARC_PROG:
                                          ;(auto-repeat command check in progress)
     mov     #$C8,Sw0ARC                  ; Move decimal 200 into Switch #0
                                          ; auto-repeat command counter variable(1Sec)
     bset    Sw0LP,ARCflags               ; Set "Sw0LP" bit of "ARCflags" variable
     jmp     SW0_CHK_DONE                 ; Jump to SW0_CHK_DONE:

SW0_ARC_PROG:
     lda     Sw0ARC                      ; Load accumulator with value in Switch
                                         ; #0 auto repeat command timer counter
     bne     SW0_CHK_DONE                ; If Z bit of CCR is clear, branch to
                                         ; SW0_CHK_DONE: ("Sw0ARC" not = 0,
                                         ; auto-repeat command check in progress,
                                         ; skip over)
     brset   Sw0LP,ARflags,SW0_AR_PROG   ; If "Sw0LP" bit of "ARflags"
                                         ; variable is set, branch to SW0_AR_PROG:
                                         ;(auto-repeat check in progress)
     mov     #$64,Sw0AR                  ; Move decimal 100 into Contact Set #0
                                         ; auto-repeat counter variable(500mS)
     bset    Sw0LP,ARflags               ; Set "Sw0LP" bit of "ARflags" variable

SW0_AR_PROG:
     lda     Sw0AR                       ; Load accumulator with value in Contact
                                         ; Set #0 auto repeat timer counter var
     bne     SW0_CHK_DONE                ; If Z bit of CCR is clear, branch to
                                         ; SW0_CHK_DONE: ("Sw0DB" not = 0,
                                         ; auto-repeat check in progress,
                                         ; skip over)
SW0_CLS:
     bset    Sw0cls,Swflags              ; Set "Sw0cls" bit of "Swflags" variable
     bclr    Sw0LP,ARflags               ; Clear "Sw0LP" bit of "ARflags" variable
     jmp     SW0_CHK_DONE                ; Jump to SW0_CHK_DONE:

SW0_OPN:
     brclr   Sw0LP,LPflags,SW0_CHK_DONE  ; If "Sw0LP" bit of "LPflags"
                                         ; variable is clear, branch to SW0_CHK_DONE:
                                         ; (contact set open, and bit is already
                                         ; clear, so skip over)
     mov     #$64,Sw0DB                  ; Move decimal 100 into Contact Set #0
                                         ; de-bounce counter variable (100mS)
     bclr    Sw0LP,LPflags               ; Clear "Sw0LP" bit of "LPflags" variable
     clr     Sw0AR                       ; Clear Sw0 auto-repeat timer counter
     bclr    Sw0LP,ARflags               ; Clear "Sw0LP" bit of "ARflags" variable
     clr     Sw0ARC                      ; Clear "Sw0" auto-repeat command timer counter
     bclr    Sw0LP,ARCflags              ; Clear "Sw0LP" bit of "ARCflags" variable
     bclr    Sw0cls,Swflags              ; Clear "Sw0cls" bit of "Swflags" variable

SW0_CHK_DONE:
     rts                                 ; Return from subroutine


;****************************************************************************
; - This subroutine checks the state of the Display Screen previous
;   button on PTA1 and updates the switch status flag.
;   The switch status flag is cleared every pass through the main loop after
;   the routine relevent to that flag is completed.
;   Edge detection is provided from both open to closed, and closed to open.
;   Auto-repeat at 2HZ is commanded as long as the contacts remain closed
;   for a period of 1 second or more.
;   Auto-repeat is prevented in the open state.
;****************************************************************************

SW1_CHK:
     lda     Sw1DB                ; Load accumulator with value in Switch
                                  ; #1 de-bounce counter variable
     bne     SW1_CHK_DONE         ; If Z bit of CCR is clear, branch to
                                  ; SW1_CHK_DONE: ("Sw0DB" not = 0,
                                  ; de-bounce in progress, skip over)
     brset   Sw1,porta,SW1_OPN    ; If "Sw1" bit of Port A is set,(Hi)
                                  ; branch to Sw1_OPN: (contacts are open)
     brset   Sw1LP,LPflags,SW1_ARC_CHK  ; If "Sw1LP" bit of "LPflags"
                             ; variable is set, branch to SW1_ARC_CHK:
                             ; (contacts closed, bit is already set,
                             ; check for auto-repeat command)
     mov     #$64,Sw1DB      ; Move decimal 100 into Switch #1
                             ; de-bounce counter variable (100mS)
     bset    Sw1LP,LPflags   ; Set "Sw1LP" bit of "LPflags" variable
     jmp     SW1_CLS         ; Jump to SW1_CLS:

SW1_ARC_CHK:
     brset   Sw1LP,ARCflags,SW1_ARC_PROG  ; If "Sw1LP" bit of "ARCflags"
                             ; variable is set, branch to SW1_ARC_PROG:
                             ;(auto-repeat command check in progress)
     mov     #$C8,Sw1ARC     ; Move decimal 200 into Switch #1
                             ; auto-repeat command counter variable(1Sec)
     bset    Sw1LP,ARCflags  ; Set "Sw1LP" bit of "ARCflags" variable
     jmp     SW1_CHK_DONE    ; Jump to SW1_CHK_DONE:

SW1_ARC_PROG:
     lda     Sw1ARC              ; Load accumulator with value in Switch
                                 ; #0 auto repeat command timer counter
     bne     SW1_CHK_DONE        ; If Z bit of CCR is clear, branch to
                                 ; SW1_CHK_DONE: ("Sw1ARC" not = 0,
                                 ; auto-repeat command check in progress,
                                 ; skip over)
     brset   Sw1LP,ARflags,SW1_AR_PROG   ; If "Sw1LP" bit of "ARflags"
                                 ; variable is set, branch to SW1_AR_PROG:
                                 ;(auto-repeat check in progress)
     mov     #$64,Sw1AR          ; Move decimal 100 into Contact Set #1
                                 ; auto-repeat counter variable(500mS)
     bset    Sw1LP,ARflags       ; Set "Sw1LP" bit of "ARflags" variable

SW1_AR_PROG:
     lda     Sw1AR               ; Load accumulator with value in Contact
                                 ; Set #1 auto repeat timer counter var
     bne     SW1_CHK_DONE        ; If Z bit of CCR is clear, branch to
                                 ; SW1_CHK_DONE: ("Sw1DB" not = 0,
                                 ; auto-repeat check in progress,
                                 ; skip over)
SW1_CLS:
     bset    Sw1cls,Swflags      ; Set "Sw1cls" bit of "Swflags" variable
     bclr    Sw1LP,ARflags       ; Clear "Sw1LP" bit of "ARflags" variable
     jmp     SW1_CHK_DONE        ; Jump to SW1_CHK_DONE:

SW1_OPN:
     brclr   Sw1LP,LPflags,SW1_CHK_DONE  ; If "Sw1LP" bit of "LPflags"
                              ; variable is clear, branch to SW1_CHK_DONE:
                              ; (contact set open, and bit is already
                              ; clear, so skip over)
     mov     #$64,Sw1DB       ; Move decimal 100 into Contact Set #1
                              ; de-bounce counter variable (100mS)
     bclr    Sw1LP,LPflags    ; Clear "Sw1LP" bit of "LPflags" variable
     clr     Sw1AR            ; Clear "Sw1" auto-repeat timer counter
     bclr    Sw1LP,ARflags    ; Clear "Sw1LP" bit of "ARflags" variable
     clr     Sw1ARC           ; Clear "Sw1" auto-repeat command timer counter
     bclr    Sw1LP,ARCflags   ; Clear "Sw1LP" bit of "ARCflags" variable
     bclr    Sw1cls,Swflags   ; Clear "Sw1cls" bit of "Swflags" variable

SW1_CHK_DONE:
     rts                      ; Return from subroutine


;***************************************************************************
; - This subroutine is a ~300mS delay used at start up for power
;   stabilization and between transmit bytes for downloading VE and ST
;   constants.
;***************************************************************************

DELAY300:
     clr     tmp1     ; Clear tmp1 variable

WAIT_1:
     clr     tmp2     ; Clear tmp2 variable

WAIT_2:
     clr     tmp3     ; Clear tmp3 variable

WAIT_3:
     lda     tmp3     ; Load accumulator with value in tmp3
     inca             ; Increment value in accumulator
     sta     tmp3     ; Copy to tmp3
     cmp     #$C8     ; Compare value in accumulator with decimal 200
     blo     WAIT_3   ; If C bit of CCR is set, (A<M), branch to WAIT_3:
     lda     tmp2     ; Load accumulator with value in tmp2
     inca             ; Increment value in accumulator
     sta     tmp2     ; Copy to tmp2
     cmp     #$C8     ; Compare value in accumulator with decimal 200
     blo     WAIT_2   ; If C bit of CCR is set, (A<M), branch to WAIT_2:
     lda     tmp1     ; Load accumulator with value in tmp1
     inca             ; Increment value in accumulator
     sta     tmp1     ; Copy to tmp1
     cmp     #$05     ; Compare value in accumulator with decimal 5
     blo     WAIT_1   ; If C bit of CCR is set, (A<M), branch to WAIT_1:
     rts              ; Return from subroutine


;****************************************************************************
; - This subroutine prints the top line of the VFD while in "Display" mode
;****************************************************************************

PRNT_TOPLN_DSP:

;****************************************************************************
; - Set up the VFD to place the first character in the top line, extreme
;   left hand position
;****************************************************************************

     jsr     VFD_START_TOP     ; Jump to subroutine at VFD_START_TOP:

;***************************************************************************
; - Using the Variables Top Line Table Index vector table, and the "ScrnCnt"
;   offset value, load H:X with the address of the desired Variables Top
;   Line Table.
;***************************************************************************

     ldhx    #VARS_TL_TB_IND    ; Load index register with the address of
                                ; the first value in the Variables Top Line
                                ; Table Index vector table
     lda     ScrnCnt            ; Load accumulator with the value in "ScrnCnt"
     jsr     GET_VECT_ADDR      ; Jump to subroutine at GET_VECT_ADDR:

;***************************************************************************
; - Print the top line of the VFD display
;***************************************************************************

     jsr     PRINT_LINE         ; Jump to subroutine at PRINT_LINE:
     rts                        ; Return from subroutine


;***************************************************************************
; - This subroutine initializes bottom line of the VFD with blank spaces
;***************************************************************************

LOAD_SPACE:
     lda     #$20         ; Load accumulator with ASCII ' '(space)
     sta     BotLin0      ; Copy to "BotLin0" variable
     sta     BotLin1      ; Copy to "BotLin0" variable
     sta     BotLin2      ; Copy to "BotLin0" variable
     sta     BotLin3      ; Copy to "BotLin0" variable
     sta     BotLin4      ; Copy to "BotLin0" variable
     sta     BotLin5      ; Copy to "BotLin0" variable
     sta     BotLin6      ; Copy to "BotLin0" variable
     sta     BotLin7      ; Copy to "BotLin0" variable
     sta     BotLin8      ; Copy to "BotLin0" variable
     sta     BotLin9      ; Copy to "BotLin0" variable
     sta     BotLin10     ; Copy to "BotLin0" variable
     sta     BotLin11     ; Copy to "BotLin0" variable
     sta     BotLin12     ; Copy to "BotLin0" variable
     sta     BotLin13     ; Copy to "BotLin0" variable
     sta     BotLin14     ; Copy to "BotLin0" variable
     sta     BotLin15     ; Copy to "BotLin0" variable
     sta     BotLin16     ; Copy to "BotLin0" variable
     sta     BotLin17     ; Copy to "BotLin0" variable
     sta     BotLin18     ; Copy to "BotLin0" variable
     sta     BotLin19     ; Copy to "BotLin0" variable
     rts                  ; Return from subroutine


;***************************************************************************
; - This subroutine takes the address of the desired Line Table loaded
;   in H:X. Using the value in "ColNum" offset value, load the "value"
;   variable with the contents of the appropriate ASCCI value in the table
;   and display them on the top line of the VFD.
;***************************************************************************

PRINT_LINE:
     clr     ColNum            ; Clear "ColNum" variable
                               ;(ColNum = 0 = 1st column on left)
     lda     ColNum            ; Load accumulator with value in "ColNum"

NEXT_CHAR:
     jsr     LDA_W_HX_PL_A     ; Jump to subroutine at LDA_W_HX_PL_A:
     sta     value             ; Copy to "value" variable
     jsr     VFD_SEND          ; Jump to subroutine at VFD_SEND:
     inc     ColNum            ; Increment "ColNum" (ColNum=ColNum+1)
     lda     ColNum            ; Load accumulator with value in "ColNum"
     cmp     #$14              ; Compare (A) with decimal 20
     beq     CHARS_DONE        ; If Z bit of CCR is set, branch to
                               ; CHARS_DONE:
                               ;(finished sending all display characters)
     bra     NEXT_CHAR         ; Branch to NEXT_CHAR:

CHARS_DONE:
     rts                       ; Return from subroutine


;***************************************************************************
; - This subroutine loads H:X with the desired vectored address found in a
;   vector address table. (H:X) originally holds the address of beginning of
;   the vector address table. (A) holds the offset value to the desired
;   vector address.
;   The accumulator has to be multiplied by 2 before addition to the index
;   register H:X, since each entry in the vector table is of 2 byte length.
;   Since the indexed addressing mode for LDHX is missing, we cannot load
;   H:X with the content of memory that H:X is pointing to. To do so, we
;   load (A) with the Hi byte of the vector address, using indexed
;   addressing with zero offset, and load (X) with the Lo byte of the vector
;   address, again, using indexed addressing, but, with an offset of 1.
;   After copying (A) to (H) via push/pull operations, (H:X) contains the
;   vector address.
;   NOTE! After the final "pulh" instruction, a "jmp ,x" will jump the
;   program to the desired vector address.
;***************************************************************************

GET_VECT_ADDR:
     lsla             ; Logical shift left accumulator(multiply by 2)
     pshx             ; Push value in index register Lo byte to stack
     pshh             ; Push value in index register Hi byte to stack
     add     2,SP     ; Add ((A)<-(A)+(M)) In this case, 2=2nd location
                      ; on stack, and SP=A, so (A=X+A)
     tax              ; Transfer value in accumulator to index register Lo
                      ; byte(Copy result in to index register Lo byte)
     pula             ; Pull value from stack to accumulator((H)->(A))
     adc     #$00     ; Add with carry ((A)<-(A)+(M)+(C))
                      ;(This just adds the carry, if applicable)
     psha             ; Push value in accumulator to stack
                      ;(modified (H) -> stack)
     pulh             ; Pull value from stack to index register Hi byte
                      ;(modified (H)->(H)
     pula             ; Pull value from stack to accumulator
                      ;(clean up stack)
     lda     ,x       ; Load accumulator with value in index register Lo
                      ;(vector Hi byte)
     ldx     1,x      ; Load index register Lo byte with value in 1st
                      ; location on stack(vector Lo byte)
     psha             ; Push value in accumulator to stack
     pulh             ; Pull value from stack to accumulator((A)->(H)
                      ;((H:X) now contains the desired vector address)
     rts              ; Return from subroutine


;***************************************************************************
; - This subroutine does an effective address calculation, adding the
;   unsigned 8 bit value in the accumulator, to the index register (H:X).
;   Since there is no instruction available which can add the contents of
;   A to H:X, the contents of H:X must first be saved to memory(stack), to
;   allow a memory to register addition operation. H:X is modified.
;   (A) contains the value of the offset from address at (H:X)
;***************************************************************************

ADD_A_TO_HX:
     pshx            ; Push value in index register Lo byte to stack
     pshh            ; Push value in index register Hi byte to stack
     psha            ; push value in accumulator to stack
     tsx             ; Transfer value in stack to index register Lo byte
                     ;((A)->(X))
     add     2,x     ; Add ((A)<-(A)+(M)) In this case, 2=2nd location on
                     ; stack, and x=A, so (A=X+A)
     sta     2,x     ; Copy result to 2nd location on stack
     clra            ; Clear accumulator(A=0)
     adc     1,x     ; Add with carry ((A)<-(A)+(M)+(C)) In this case
                     ; 1=1st location on stack, and x=A=0, so (A=H+C+A)
     sta     1,x     ; Copy result to 1st location on stack
     pula            ; Pull value from stack to accumulator
     pulh            ; Pull value from stack to index register Hi byte
     pulx            ; Pull value from stack to index register Lo byte
                     ;(H:X) now contains ((H:X+(A))
     rts             ; return from subroutine


;***************************************************************************
; - This subroutine loads into A, the contents of a location pointed to by
;   H:X plus A. H:X is preserved. This operation emulates a "lda A,X"
;   instruction, so called "accumulator-offset indexed addressing mode",
;   which is not available on the HC08 family instruction set.
;   (A) contains the value of the offset from address at (H:X)
;***************************************************************************

LDA_W_HX_PL_A:
     pshx             ; Push value in index register Lo byte to stack
     pshh             ; Push value in index register Hi byte to stack
                      ;(These 2 instructions save the original (H:X))
     pshx             ; Push value in index register Lo byte to stack
     pshh             ; Push value in index register Hi byte to stack
                      ;(These 2 instructions are for the working H:X
     add     2,SP     ; Add ((A)<-(A)+(M)) In this case, 2=2nd location
                      ; on stack, and SP=A, so (A=X+A)
     tax              ; Transfer value in accumulator to index register Lo
                      ; byte(Copy result in to index register Lo byte)
     pula             ; Pull value from stack to accumulator((H)->(A))
     adc     #$00     ; Add with carry ((A)<-(A)+(M)+(C))
                      ;(This just adds the carry, if applicable)
     psha             ; Push value in accumulator to stack
                      ;(modified (H) to stack)
     pulh             ; Pull value from stack to index register Hi byte
                      ;(modified (H) to (H)
     ais     #$01     ; Add immediate value of 1 to SP register
                      ;(clean up stack)
     lda     ,x       ; Load accumulator with value in index register Lo
                      ; byte
                      ;(A now contains the value in the location at H:X+A)
     pulh             ; Pull value from stack to index register Hi byte
     pulx             ; Pull value from stack to index register Lo byte
                      ;(these 2 instructions restore (H:X))
     rts              ; return from subroutine

;***************************************************************************
; - This subroutine sends an instruction byte to position the cursor in the
;   top left corner of the display.
;***************************************************************************

VFD_START_TOP:

;***************************************************************************
; - Set up to send an instruction  byte.
;***************************************************************************

     bclr    Enable,PORTB     ; Clear "Enable" bit of Port B(PTB4)
     bclr    Rd_Wrt,PORTB     ; Clear "Rd_Wrt" bit of Port B(PTB5)
                              ;(Write Operation)
     bclr    Reg_Sel,PORTB    ; Clear "Reg_Sel" bit of Port B(PTB6)
                              ;(Send an instruction value)


;***************************************************************************
; - Set cursor top left Hi nibble
;***************************************************************************

     mov     #$08,PORTC       ; Move %00001000 into PortC
                              ;(Set bit4=DB7)
     bset    Enable,PORTB     ; Set "Enable" bit of Port B (PTB4)("En"=1)
     jsr     SHORT_DELAY      ; Jump to subroutine at SHORT_DELAY:
     bclr    Enable,PORTB     ; Clear "Enable" bit of Port B (PTB4)("En"=0)

;***************************************************************************
; - Set cursor top left Lo nibble
;***************************************************************************

     mov     #$00,PORTC       ; Move %00000000 into PortC
     bset    Enable,PORTB     ; Set "Enable" bit of Port B (PTB4)("En"=1)
     jsr     LONG_DELAY       ; Jump to subroutine at LONG_DELAY:
     bclr    Enable,PORTB     ; Clear "Enable" bit of Port B (PTB4)("En"=0)
     jsr     SHORT_DELAY      ; Jump to subroutine at SHORT_DELAY:
     rts                      ; Return from subroutine


;***************************************************************************
; - This subroutine sends an instruction byte to position the cursor in the
;   bottom left corner of the display.
;***************************************************************************

VFD_START_BOT:

;***************************************************************************
; - Set up to send an instruction  byte.
;***************************************************************************

     bclr    Enable,PORTB     ; Clear "Enable" bit of Port B(PTB4)
     bclr    Rd_Wrt,PORTB     ; Clear "Rd_Wrt" bit of Port B(PTB5)
                              ;(Write Operation)
     bclr    Reg_Sel,PORTB    ; Clear "Reg_Sel" bit of Port B(PTB6)
                              ;(Send an instruction value)


;***************************************************************************
; - Set cursor bottom left Hi nibble
;***************************************************************************

     mov     #$0C,PORTC       ; Move %00001100 into PortC
                              ;(Set bit4=DB7 and bit3=DB6)
     bset    Enable,PORTB     ; Set "Enable" bit of Port B (PTB4)("En"=1)
     jsr     SHORT_DELAY      ; Jump to subroutine at SHORT_DELAY:
     bclr    Enable,PORTB     ; Clear "Enable" bit of Port B (PTB4)("En"=0)

;***************************************************************************
; - Set cursor bottom left Lo nibble
;***************************************************************************

     mov     #$00,PORTC       ; Move %00000000 into PortC
     bset    Enable,PORTB     ; Set "Enable" bit of Port B (PTB4)("En"=1)
     jsr     LONG_DELAY       ; Jump to subroutine at LONG_DELAY:
     bclr    Enable,PORTB     ; Clear "Enable" bit of Port B (PTB4)("En"=0)
     jsr     SHORT_DELAY      ; Jump to subroutine at SHORT_DELAY:
     rts                      ; Return from subroutine


;***************************************************************************
; - This subroutine takes a single ASCII value, held in "value" variable
;   and sets the appropriate bits of Port C. Interface with the VFD display
;   is 4 bit, so, to send an 8 bit value, bits 4,5,6 and 7 are sent first,
;   then the bits 0,1,2 and 3 are sent.
;***************************************************************************

VFD_SEND:

;***************************************************************************
; - Data Bit 4 (PTC0)
;***************************************************************************

CHK_DB4:
     lda     value       ; Load accumulator with value in "value" variable
     bit     #DB4m       ; Logical AND (A)&(M)%00010000
     bne     SET_DB4     ; If Z bit of CCR is clear, branch to SET_DB4:
     bclr    DB4,PORTC   ; Clear "DB4" bit of Port C(bit0)
     bra     CHK_DB5     ; Branch to CHK_DB5

SET_DB4:
     bset    DB4,PORTC   ; Set "DB4" bit of Port C(bit0)

;***************************************************************************
; - Data Bit 5 (PTC1)
;***************************************************************************

CHK_DB5:
     lda     value       ; Load accumulator with value in "value" variable
     bit     #DB5m       ; Logical AND (A)&(M)%00100000
     bne     SET_DB5     ; If Z bit of CCR is clear, branch to SET_DB5:
     bclr    DB5,PORTC   ; Clear "DB5" bit of Port C(bit1)
     bra     CHK_DB6     ; Branch to CHK_DB6

SET_DB5:
     bset    DB5,PORTC   ; Set "DB5" bit of Port C(bit1)

;***************************************************************************
; - Data Bit 6 (PTC2)
;***************************************************************************

CHK_DB6:
     lda     value       ; Load accumulator with value in "value" variable
     bit     #DB6m       ; Logical AND (A)&(M)%01000000
     bne     SET_DB6     ; If Z bit of CCR is clear, branch to SET_DB6:
     bclr    DB6,PORTC   ; Clear "DB6" bit of Port C(bit2)
     bra     CHK_DB7     ; Branch to CHK_DB7

SET_DB6:
     bset    DB6,PORTC   ; Set "DB6" bit of Port C(bit2)

;***************************************************************************
; - Data Bit 7 (PTC3)
;***************************************************************************

CHK_DB7:
     lda     value       ; Load accumulator with value in "value" variable
     bit     #DB7m       ; Logical AND (A)&(M)%10000000
     bne     SET_DB7     ; If Z bit of CCR is clear, branch to SET_DB7:
     bclr    DB7,PORTC   ; Clear "DB7" bit of Port C(bit3)
     bra     HI_NIB      ; Branch to HI_NIB:

SET_DB7:
     bset    DB7,PORTC   ; Set "DB7" bit of Port C(bit3)

;***************************************************************************
; - Send the Hi nibble
;***************************************************************************

HI_NIB:
     bset    Reg_Sel,PORTB    ; Set "Reg_Sel" bit of PortB(RS=1)
     jsr     SHORT_DELAY      ; Jump to subroutine at SHORT_DELAY:
     bset    Enable,PORTB     ; Set "Enable" bit of Port B (PTB4)("En"=1)
     jsr     SHORT_DELAY      ; Jump to subroutine at SHORT_DELAY:

;***************************************************************************
; - Clear Enable line to set up Lo nibble
;***************************************************************************

     bclr    Enable,PORTB     ; Clear "Enable" bit of Port B (PTB4)("En"=0)

;***************************************************************************
; - Data Bit 0 (PTC0)
;***************************************************************************

CHK_DB0:
     lda     value       ; Load accumulator with value in "value" variable
     bit     #DB0m       ; Logical AND (A)&(M)%00000001
     bne     SET_DB0     ; If Z bit of CCR is clear, branch to SET_DB0:
     bclr    DB0,PORTC   ; Clear "DB0" bit of Port C(bit0)
     bra     CHK_DB1     ; Branch to CHK_DB1

SET_DB0:
     bset    DB0,PORTC   ; Set "DB0" bit of Port C(bit0)

;***************************************************************************
; - Data Bit 1 (PTC1)
;***************************************************************************

CHK_DB1:
     lda     value       ; Load accumulator with value in "value" variable
     bit     #DB1m       ; Logical AND (A)&(M)%00000010
     bne     SET_DB1     ; If Z bit of CCR is clear, branch to SET_DB1:
     bclr    DB1,PORTC   ; Clear "DB1" bit of Port C(bit1)
     bra     CHK_DB2     ; Branch to CHK_DB2

SET_DB1:
     bset    DB1,PORTC   ; Set "DB1" bit of Port C(bit1)

;***************************************************************************
; - Data Bit 2 (PTC2)
;***************************************************************************

CHK_DB2:
     lda     value       ; Load accumulator with value in "value" variable
     bit     #DB2m       ; Logical AND (A)&(M)%00000100
     bne     SET_DB2     ; If Z bit of CCR is clear, branch to SET_DB2:
     bclr    DB2,PORTC   ; Clear "DB2" bit of Port C(bit2)
     bra     CHK_DB3     ; Branch to CHK_DB3

SET_DB2:
     bset    DB2,PORTC   ; Set "DB2" bit of Port C(bit2)

;***************************************************************************
; - Data Bit 3 (PTC3)
;***************************************************************************

CHK_DB3:
     lda     value       ; Load accumulator with value in "value" variable
     bit     #DB3m       ; Logical AND (A)&(M)%00001000
     bne     SET_DB3     ; If Z bit of CCR is clear, branch to SET_DB3:
     bclr    DB3,PORTC   ; Clear "DB3" bit of Port C(bit3)
     bra     LO_NIB      ; Branch to LO_NIB:

SET_DB3:
     bset    DB3,PORTC   ; Set "DB3" bit of Port C(bit3)

;***************************************************************************
; - Send the Lo nibble
;***************************************************************************

LO_NIB:
     bset    Enable,PORTB     ; Set "Enable" bit of Port B (PTB4)("En"=1)
     jsr     LONG_DELAY       ; Jump to subroutine at LONG_DELAY:
                              ;(timing requirement)

;***************************************************************************
; - Clear Enable and Register Select to set up for next transmit
;***************************************************************************

     bclr    Enable,PORTB     ; Clear "Enable" bit of Port B (PTB4)("En"=0)
     jsr     SHORT_DELAY      ; Jump to subroutine at SHORT_DELAY:
     bclr    Reg_Sel,PORTB    ; Clear "Reg_Sel" bit of PortB(RS=0)
     jsr     SHORT_DELAY      ; Jump to subroutine at SHORT_DELAY:
     rts                      ; Return from subroutine


;****************************************************************************
; - This is the delay time from the point at which the data bits have been
;   configured, and the "enable" bit set, to the point at which the "enable"
;   bit is cleared.(min 0.45uS)
;   One pass through the loop takes ~1.5uS, bus frequency of ~8mHZ
;****************************************************************************

LONG_DELAY:
     clr     tmp1       ; Clear tmp1 variable

WAIT_4:
     lda     tmp1       ; Load accumulator with value in tmp1 variable
     inca               ; Increment value in accumulator
     sta     tmp1       ; Copy to tmp1 variable
     cmp     #$02       ; Compare value in accumulator with decimal 2
     blo     WAIT_4     ; If C bit of CCR is set, (A<M), branch to
                        ; WAIT_4:(~3uS delay for timing requirements)
     rts                ; Return from subroutine


;****************************************************************************
; - This is the delay time from the point at which the "enable" bit has been
;   cleared, to the point where the data bits can be re-configured.
;   (min 0.01uS) One NOP takes ~0.125uS, bus frequency of ~8mHZ
;****************************************************************************

SHORT_DELAY:
     nop                ; No operation(1 bus cycle)
     rts                ; Return from subroutine


;****************************************************************************
; - This subroutine compares all the characters on the bottom line commanded,
;   to those of the previous bottom line. If they are different, update the
;   bottom line, otherwise, skip over. This is to eliminate display
;   "digit rattle" caused by rapid screen updates.
;****************************************************************************

CMP_BOTLIN:
     lda     Botlin0         ; Load accumulator with value in "Botlin0"
     cmp     Botlin0L        ; Compare it with the value in "Botlin0L"
     beq     NO_CHNG_BL0     ; If Z bit of CCR is set, branch to NO_CHNG_BL0
                             ;(A=M)
     sta     Botlin0L        ; Copy "Botlin0" to "Botlin0L"
     jmp     BOTLIN_CHNG     ; Jump to BOTLIN_CHNG:

NO_CHNG_BL0:
     lda     Botlin1         ; Load accumulator with value in "Botlin1"
     cmp     Botlin1L        ; Compare it with the value in "Botlin1L"
     beq     NO_CHNG_BL1     ; If Z bit of CCR is set, branch to NO_CHNG_BL1
                             ;(A=M)
     sta     Botlin1L        ; Copy "Botlin1" to "Botlin1L"
     jmp     BOTLIN_CHNG     ; Jump to BOTLIN_CHNG:

NO_CHNG_BL1:
     lda     Botlin2         ; Load accumulator with value in "Botlin2"
     cmp     Botlin2L        ; Compare it with the value in "Botlin2L"
     beq     NO_CHNG_BL2     ; If Z bit of CCR is set, branch to NO_CHNG_BL2
                             ;(A=M)
     sta     Botlin2L        ; Copy "Botlin2" to "Botlin2L"
     jmp     BOTLIN_CHNG     ; Jump to BOTLIN_CHNG:

NO_CHNG_BL2:
     lda     Botlin3         ; Load accumulator with value in "Botlin3"
     cmp     Botlin3L        ; Compare it with the value in "Botlin3L"
     beq     NO_CHNG_BL3     ; If Z bit of CCR is set, branch to NO_CHNG_BL3
                             ;(A=M)
     sta     Botlin3L        ; Copy "Botlin3" to "Botlin3L"
     jmp     BOTLIN_CHNG     ; Jump to BOTLIN_CHNG:

NO_CHNG_BL3:
     lda     Botlin4         ; Load accumulator with value in "Botlin4"
     cmp     Botlin4L        ; Compare it with the value in "Botlin4L"
     beq     NO_CHNG_BL4     ; If Z bit of CCR is set, branch to NO_CHNG_BL4
                             ;(A=M)
     sta     Botlin4L        ; Copy "Botlin4" to "Botlin4L"
     jmp     BOTLIN_CHNG     ; Jump to BOTLIN_CHNG:

NO_CHNG_BL4:
     lda     Botlin5         ; Load accumulator with value in "Botlin5"
     cmp     Botlin5L        ; Compare it with the value in "Botlin5L"
     beq     NO_CHNG_BL5     ; If Z bit of CCR is set, branch to NO_CHNG_BL5
                             ;(A=M)
     sta     Botlin5L        ; Copy "Botlin5" to "Botlin5L"
     jmp     BOTLIN_CHNG     ; Jump to BOTLIN_CHNG:

NO_CHNG_BL5:
     lda     Botlin6         ; Load accumulator with value in "Botlin6"
     cmp     Botlin6L        ; Compare it with the value in "Botlin6L"
     beq     NO_CHNG_BL6     ; If Z bit of CCR is set, branch to NO_CHNG_BL6
                             ;(A=M)
     sta     Botlin6L        ; Copy "Botlin6" to "Botlin6L"
     jmp     BOTLIN_CHNG     ; Jump to BOTLIN_CHNG:

NO_CHNG_BL6:
     lda     Botlin7         ; Load accumulator with value in "Botlin7"
     cmp     Botlin7L        ; Compare it with the value in "Botlin7L"
     beq     NO_CHNG_BL7     ; If Z bit of CCR is set, branch to NO_CHNG_BL7
                             ;(A=M)
     sta     Botlin7L        ; Copy "Botlin7" to "Botlin7L"
     jmp     BOTLIN_CHNG     ; Jump to BOTLIN_CHNG:

NO_CHNG_BL7:
     lda     Botlin8         ; Load accumulator with value in "Botlin8"
     cmp     Botlin8L        ; Compare it with the value in "Botlin8L"
     beq     NO_CHNG_BL8     ; If Z bit of CCR is set, branch to NO_CHNG_BL8
                             ;(A=M)
     sta     Botlin8L        ; Copy "Botlin8" to "Botlin8L"
     jmp     BOTLIN_CHNG     ; Jump to BOTLIN_CHNG:

NO_CHNG_BL8:
     lda     Botlin9         ; Load accumulator with value in "Botlin9"
     cmp     Botlin9L        ; Compare it with the value in "Botlin9L"
     beq     NO_CHNG_BL9     ; If Z bit of CCR is set, branch to NO_CHNG_BL9
                             ;(A=M)
     sta     Botlin9L        ; Copy "Botlin9" to "Botlin9L"
     jmp     BOTLIN_CHNG     ; Jump to BOTLIN_CHNG:

NO_CHNG_BL9:
     lda     Botlin10         ; Load accumulator with value in "Botlin10"
     cmp     Botlin10L        ; Compare it with the value in "Botlin10L"
     beq     NO_CHNG_BL10     ; If Z bit of CCR is set, branch to NO_CHNG_BL10
                              ;(A=M)
     sta     Botlin10L        ; Copy "Botlin10" to "Botlin10L"
     jmp     BOTLIN_CHNG      ; Jump to BOTLIN_CHNG:

NO_CHNG_BL10:
     lda     Botlin11         ; Load accumulator with value in "Botlin11"
     cmp     Botlin11L        ; Compare it with the value in "Botlin11L"
     beq     NO_CHNG_BL11     ; If Z bit of CCR is set, branch to NO_CHNG_BL11
                              ;(A=M)
     sta     Botlin11L        ; Copy "Botlin11" to "Botlin11L"
     jmp     BOTLIN_CHNG      ; Jump to BOTLIN_CHNG:

NO_CHNG_BL11:
     lda     Botlin12         ; Load accumulator with value in "Botlin12"
     cmp     Botlin12L        ; Compare it with the value in "Botlin12L"
     beq     NO_CHNG_BL12     ; If Z bit of CCR is set, branch to NO_CHNG_BL12
                              ;(A=M)
     sta     Botlin12L        ; Copy "Botlin12" to "Botlin12L"
     jmp     BOTLIN_CHNG      ; Jump to BOTLIN_CHNG:

NO_CHNG_BL12:
     lda     Botlin13         ; Load accumulator with value in "Botlin13"
     cmp     Botlin13L        ; Compare it with the value in "Botlin13L"
     beq     NO_CHNG_BL13     ; If Z bit of CCR is set, branch to NO_CHNG_BL13
                              ;(A=M)
     sta     Botlin13L        ; Copy "Botlin13" to "Botlin13L"
     jmp     BOTLIN_CHNG      ; Jump to BOTLIN_CHNG:

NO_CHNG_BL13:
     lda     Botlin14         ; Load accumulator with value in "Botlin14"
     cmp     Botlin14L        ; Compare it with the value in "Botlin14L"
     beq     NO_CHNG_BL14     ; If Z bit of CCR is set, branch to NO_CHNG_BL14
                              ;(A=M)
     sta     Botlin14L        ; Copy "Botlin14" to "Botlin14L"
     jmp     BOTLIN_CHNG      ; Jump to BOTLIN_CHNG:

NO_CHNG_BL14:
     lda     Botlin15         ; Load accumulator with value in "Botlin15"
     cmp     Botlin15L        ; Compare it with the value in "Botlin15L"
     beq     NO_CHNG_BL15     ; If Z bit of CCR is set, branch to NO_CHNG_BL15
                              ;(A=M)
     sta     Botlin15L        ; Copy "Botlin15" to "Botlin15L"
     jmp     BOTLIN_CHNG      ; Jump to BOTLIN_CHNG:

NO_CHNG_BL15:
     lda     Botlin16         ; Load accumulator with value in "Botlin16"
     cmp     Botlin16L        ; Compare it with the value in "Botlin16L"
     beq     NO_CHNG_BL16     ; If Z bit of CCR is set, branch to NO_CHNG_BL16
                              ;(A=M)
     sta     Botlin16L        ; Copy "Botlin16" to "Botlin16L"
     jmp     BOTLIN_CHNG      ; Jump to BOTLIN_CHNG:

NO_CHNG_BL16:
     lda     Botlin17         ; Load accumulator with value in "Botlin17"
     cmp     Botlin17L        ; Compare it with the value in "Botlin17L"
     beq     NO_CHNG_BL17     ; If Z bit of CCR is set, branch to NO_CHNG_BL17
                              ;(A=M)
     sta     Botlin17L        ; Copy "Botlin17" to "Botlin17L"
     jmp     BOTLIN_CHNG      ; Jump to BOTLIN_CHNG:

NO_CHNG_BL17:
     lda     Botlin18         ; Load accumulator with value in "Botlin18"
     cmp     Botlin18L        ; Compare it with the value in "Botlin18L"
     beq     NO_CHNG_BL18     ; If Z bit of CCR is set, branch to NO_CHNG_BL18
                              ;(A=M)
     sta     Botlin18L        ; Copy "Botlin18" to "Botlin18L"
     jmp     BOTLIN_CHNG      ; Jump to BOTLIN_CHNG:

NO_CHNG_BL18:
     lda     Botlin19         ; Load accumulator with value in "Botlin19"
     cmp     Botlin19L        ; Compare it with the value in "Botlin19L"
     beq     NO_CHNG_BL19     ; If Z bit of CCR is set, branch to NO_CHNG_BL19
                              ;(A=M)
     sta     Botlin19L        ; Copy "Botlin19" to "Botlin19L"
     jmp     BOTLIN_CHNG      ; Jump to BOTLIN_CHNG:

NO_CHNG_BL19:
     bra     NO_CHNG_BOTLIN   ; Branch to NO_CHNG_BOTLIN:

BOTLIN_CHNG:
     bset    LinChng,flags    ; Set "Linchng" bit of "flags" variable
     bra     CMP_BOTLIN_DONE  ; Branch to CMP_BOTLIN_DONE:

NO_CHNG_BOTLIN:
     bclr    LinChng,flags    ; Clear "Linchng" bit of "flags" variable

CMP_BOTLIN_DONE:
     rts                      ; Return from subroutine

;***************************************************************************
;
; -------------------- Ordered Table Search Subroutine ---------------------
;
;  X is pointing to the start of the first value in the table
;  tmp1:2 initially hold the start of table address,
;  then they hold the bound values
;  tmp3 is the end of the table ("n" elements - 1)
;  tmp4 is the comparison value
;  tmp5 is the index result - if zero then comp value is less
;  than beginning of table, and if equal to "n" elements then it is
;  rail-ed at upper end
;
;***************************************************************************

ORD_TABLE_FIND:
     clr     tmp5     ; Clear tmp5 variable
     ldhx    tmp1     ; Load high part of index register with value in tmp1
     lda     ,x	      ; Load accumulator with low part of index register???
     sta     tmp1     ; Copy to tmp1 variable
     sta     tmp2     ; Copy to tmp2 variable

REENT:
     incx                    ; Increment low part of index register
     inc     tmp5            ; Increment tmp5 variable
     mov     tmp2,tmp1       ; Move value in tmp2 variable to tmp1 variable
     lda     ,x              ; Load accumulator with value in index reg Lo??
     sta     tmp2            ; Copy to tmp2 variable
     cmp     tmp4            ; Compare it with tmp4 variable
     bhi     GOT_ORD_NUM     ; If higher, branch to GOT_ORD_NUM lable
     lda     tmp5            ; Load accumulator with value in tmp5 variable
     cmp     tmp3            ; Compare it with value in tmp3 variable
     bne     REENT           ; If the Z bit of CCR is clesr, branch to REENT:

GOT_ORD_NUM:
     rts                     ; Return from subroutine


;****************************************************************************
;
; ------------------ Linear Interpolation - 2D Subroutine -------------------
;
; Graph Plot         Z2
;                   Y2
;               X
;               Y
;         X1
;         Y1
;            (y2 - y1)
;  Y = Y1 +  --------- * (x - x1)
;            (x2 - x1)
;
;   tmp1 = x1
;   tmp2 = x2
;   tmp3 = y1
;   tmp4 = y2
;   tmp5 = x
;   tmp6 = y
;***************************************************************************

LININTERP:
     clr     tmp7          ; Clear tmp7 variable (This is the negative slope
                           ; detection bit) (tmp7 = 0)
     mov     tmp3,tmp6     ; Move value in tmp3 variable to tmp6 variable
                           ; (Y1 to tmp6)

CHECK_LESS_THAN:
     lda     tmp5               ; Load accumulator with value in tmp5 variable
                                ; (x)
     cmp     tmp1               ; Compare it with value in tmp1 variable
                                ; (x1)
     bhi     CHECK_GREATER_THAN ; If higher, branch to CHECK_GREATER_THAN:
                                ; (X>X1)
     bra     DONE_WITH_INTERP	; Branch to DONE_WITH_INTERP: (else (Y=Y1))

CHECK_GREATER_THAN:
     lda     tmp5             ; Load accumulator with value in tmp5 variable
                              ; (x)
     cmp     tmp2             ; Compare it with value in tmp2 variable
                              ; (X2)
     blo     DO_INTERP        ; If lower, branch to DO_INTERP lable
                              ; (X<X2)
     mov     tmp4,tmp6        ; Move value in tmp4 variable to tmp6 variable
                              ; (Y2 to tmp6)
     bra     DONE_WITH_INTERP ; Branch to DONE_WITH_INTERP lable (else (Y=Y2))

DO_INTERP:
     mov     tmp3,tmp6        ; Move value in tmp3 variable to tmp6 variable
                              ; (Y1 to tmp6)
     lda     tmp2             ; Load accumulator with value in tmp2 variable
                              ; (X2)
     sub     tmp1             ; Subtract tmp1 from tmp2 (A=X2-X1)
     beq     DONE_WITH_INTERP ; If the Z bit of CCR is set, branch to
                              ;DONE_WITH_INTERP:  else (Y=Y1)
     psha                     ; Push value in accumulator to stack
                              ; (X2-X1)(stack 1)
     lda     tmp4             ; Load accumulator with value in tmp4 variable
                              ; (Y2)
     sub     tmp3             ; Subtract tmp3 from tmp4 (A=Y2-Y1)
     bcc     POSINTERP        ; If C bit of CCR is clear, branch to POSINTERP:
     nega                     ; Negate accumulator      ??????????
     inc     tmp7             ; Increment tmp7 variable (tmp7 = 1)

POSINTERP:
     psha                     ; Push value in accumulator to stack
                              ; (negated Y2-Y1) (stack 2)
     lda     tmp5             ; Load accumulator with value in tmp5 variable
                              ; (X)
     sub     tmp1             ; Subtract tmp1 from tmp5 (A=X-X1)
     beq     ZERO_SLOPE	      ; If the Z bit of CCR is set,
                              ; branch to ZERO_SLOPE lable  (Y=Y1)
     pulx                     ; Pull value from stack to index register Lo
                              ;(negated Y2-Y1) (stack 2)
     mul                      ; Multiply it by the value in the accumulator
                              ; A=(negated Y2-Y1)*(X-X1)
     pshx                     ; Push the index register L to the stack
                              ; (stack 2)
     pulh                     ; Pull this value to index register Hi(stack 2)
     pulx                     ; Pull the next value to index register Lo
                              ;(stack 1)
     div                      ; Divide A<-(H:A)/(X);H<-Remainder
     psha                     ; Push the value in the accumulator onto stack
                              ; (stack 1)
     lda     tmp7             ; Load accumulator with value in tmp7 variable
     bne     NEG_SLOPE        ; If the Z bit of CCR is clear,
                              ; branch to NEG_SLOPE: (Y=Y1)
     pula                     ; Pull value from stack to accumulator (stack 1)
     add     tmp3             ; Add it with value in tmp3 variable
     sta     tmp6             ; Copy it to tmp6 variable
     bra     DONE_WITH_INTERP ; Branch to  DONE_WITH_INTERP:

NEG_SLOPE:
     pula                     ; Pull value from stack to accumulator(stack 1)
     sta     tmp7             ; Copy to tmp7 variable
     lda     tmp3             ; Load accumulator with value in tmp3  Y1)
     sub     tmp7             ; Subtract tmp7 from tmp3
     sta     tmp6             ; Copy result to tmp6 variable
     bra     DONE_WITH_INTERP ; Branch to  DONE_WITH_INTERP:

ZERO_SLOPE:
        pula    ; Pull value from stack to accumulator (clean stack)(stack 2)
        pula    ; Pull value from stack to accumulator (clean stack)(stack 1)

DONE_WITH_INTERP:
        rts      ; Return from subroutine

;****************************************************************************
;
; ----------------- 32 x 16 Unsigned Divide Subroutine ---------------------
;
; This routine takes the 32-bit dividend stored in INTACC1.....INTACC1+3
; and divides it by the 16-bit divisor stored in INTACC2:INTACC2+1.
; The quotient replaces the dividend and the remainder replaces the divisor.
; INTACC1:INTAAC1+1:INTAAC1+2:INTAAC1+3 / INTAAC2:INTAAC2+1
; = INTACC1:INTAAC1+1:INTAAC1+2:INTAAC1+3 rem INTAAC2:INTAAC2+1
;
;***************************************************************************

UDVD32    EQU     *
*
DIVIDEND  EQU     INTACC1+2
DIVISOR   EQU     INTACC2
QUOTIENT  EQU     INTACC1
REMAINDER EQU     INTACC1
*
        PSHH                            ;save h-reg value
        PSHA                            ;save accumulator
        PSHX                            ;save x-reg value
        AIS     #-3                     ;reserve three bytes of temp storage
        LDA     #!32                    ;
        STA     3,SP                    ;loop counter for number of shifts
        LDA     DIVISOR                 ;get divisor msb
        STA     1,SP                    ;put divisor msb in working storage
        LDA     DIVISOR+1               ;get divisor lsb
        STA     2,SP                    ;put divisor lsb in working storage

****************************************************************************
*     Shift all four bytes of dividend 16 bits to the right and clear
*     both bytes of the temporary remainder location
****************************************************************************

        MOV     DIVIDEND+1,DIVIDEND+3   ;shift dividend lsb
        MOV     DIVIDEND,DIVIDEND+2     ;shift 2nd byte of dividend
        MOV     DIVIDEND-1,DIVIDEND+1   ;shift 3rd byte of dividend
        MOV     DIVIDEND-2,DIVIDEND     ;shift dividend msb
        CLR     REMAINDER               ;zero remainder msb
        CLR     REMAINDER+1             ;zero remainder lsb

****************************************************************************
*     Shift each byte of dividend and remainder one bit to the left
****************************************************************************

SHFTLP  LDA     REMAINDER               ;get remainder msb
        ROLA                            ;shift remainder msb into carry
        ROL     DIVIDEND+3              ;shift dividend lsb
        ROL     DIVIDEND+2              ;shift 2nd byte of dividend
        ROL     DIVIDEND+1              ;shift 3rd byte of dividend
        ROL     DIVIDEND                ;shift dividend msb
        ROL     REMAINDER+1             ;shift remainder lsb
        ROL     REMAINDER               ;shift remainder msb

*****************************************************************************
*     Subtract both bytes of the divisor from the remainder
*****************************************************************************

        LDA     REMAINDER+1          ;get remainder lsb
        SUB     2,SP                 ;subtract divisor lsb from remainder lsb
        STA     REMAINDER+1          ;store new remainder lsb
        LDA     REMAINDER            ;get remainder msb
        SBC     1,SP                 ;subtract divisor msb from remainder msb
        STA     REMAINDER            ;store new remainder msb
        LDA     DIVIDEND+3           ;get low byte of dividend/quotient
        SBC     #0                   ;dividend low bit holds subtract carry
        STA     DIVIDEND+3           ;store low byte of dividend/quotient

*****************************************************************************
*     Check dividend/quotient lsb. If clear, set lsb of quotient to indicate
*     successful subraction, else add both bytes of divisor back to remainder
*****************************************************************************

        BRCLR   0,DIVIDEND+3,SETLSB     ;check for a carry from subtraction
                                        ;and add divisor to remainder if set
        LDA     REMAINDER+1             ;get remainder lsb
        ADD     2,SP                    ;add divisor lsb to remainder lsb
        STA     REMAINDER+1             ;store remainder lsb
        LDA     REMAINDER               ;get remainder msb
        ADC     1,SP                    ;add divisor msb to remainder msb
        STA     REMAINDER               ;store remainder msb
        LDA     DIVIDEND+3              ;get low byte of dividend
        ADC     #0                      ;add carry to low bit of dividend
        STA     DIVIDEND+3              ;store low byte of dividend
        BRA     DECRMT                  ;do next shift and subtract

SETLSB  BSET    0,DIVIDEND+3            ;set lsb of quotient to indicate
                                        ;successive subtraction
DECRMT  DBNZ    3,SP,SHFTLP             ;decrement loop counter and do next
                                        ;shift

*****************************************************************************
*     Move 32-bit dividend into INTACC1.....INTACC1+3 and put 16-bit
*     remainder in INTACC2:INTACC2+1
*****************************************************************************

        LDA     REMAINDER               ;get remainder msb
        STA     1,SP                    ;temporarily store remainder msb
        LDA     REMAINDER+1             ;get remainder lsb
        STA     2,SP                    ;temporarily store remainder lsb
        MOV     DIVIDEND,QUOTIENT       ;
        MOV     DIVIDEND+1,QUOTIENT+1   ;shift all four bytes of quotient
        MOV     DIVIDEND+2,QUOTIENT+2   ; 16 bits to the left
        MOV     DIVIDEND+3,QUOTIENT+3   ;
        LDA     1,SP                    ;get final remainder msb
        STA     INTACC2                 ;store final remainder msb
        LDA     2,SP                    ;get final remainder lsb
        STA     INTACC2+1               ;store final remainder lsb

*****************************************************************************
*     Deallocate local storage, restore register values, and return from
*     subroutine
*****************************************************************************

        AIS     #3                      ;deallocate temporary storage
        PULX                            ;restore x-reg value
        PULA                            ;restore accumulator value
        PULH                            ;restore h-reg value
        RTS                             ;return

*****************************************************************************


;****************************************************************************
; ----------  ----- ROUND after div (unsigned) Subroutine -------------------
;
;  1)  check for div overflow (carry set), rail result if detected
;  2)  if (remainder * 2) > divisor then     ; was remainder > (divisor / 2)
;  2a)    increment result, rail if over-flow
;
;****************************************************************************

DIVROUND:
     bcs     DIVROUND0     ; If C bit of CCR is set, branch to DIVROUND0:
                           ; (div overflow? yes, branch)
     stx     local_tmp     ; Copy value in index register Lo byte to
                           ; local_tmp variable (divisor)
     pshh                  ; Push value in index register Hi byte onto
                           ; stack (retrieve remainder)
     pulx                  ; Pull value on stack to index register Lo byte
     lslx                  ; Logical shift left index register lo byte (* 2)
     bcs     DIVROUND2     ; If C bit of CCR is set, branch to DIVROUND2:
                           ;(over-flow on left-shift, (remainder * 2) > $FF)
     cpx     local_tmp     ; Compare value in local_tmp variable with value
                           ; in index register Lo byte
                           ;(compare (remainder * 2) to divisor)
     blo     DIVROUND1     ; If lower, branch to DIVROUND1:


DIVROUND2:
     inca                   ; Increment accumulator (round-up result)
     bne      DIVROUND1     ; If Z bit of CCR is clear, branch to DIVROUND1:
                            ; (result roll over? no, branch)


DIVROUND0:
     lda     #$FF     ; Load accumulator with decimal 255 (rail result)


DIVROUND1:
     rts              ; return from subroutine


;****************************************************************************
;
; ------------------- 16 x 16 Unsigned Multiply Subroutine -----------------
;
;     tmp8:tmp7:tmp6:tmp5 = tmp4:tmp3 * tmp2:tmp1
;
;               tmp3*tmp1
;   +      tmp4*tmp1
;   +      tmp3*tmp2
;   + tmp4*tmp2
;   = ===================
;     tmp8 tmp7 tmp6 tmp5
;
;****************************************************************************

UMUL32:
     lda     tmp1        ; Load accumulator with value in tmp1 variable
     ldx     tmp3        ; Load index register Lo byte with value in tmp3
     mul                 ; Multiply X:A<-(X)*(A)
     sta     tmp5        ; Ccopy result to tmp5
     stx     tmp6        ; Copy value in index register Lo byte to tmp6
;
     lda     tmp2        ; Load accumulator with value in tmp2
     ldx     tmp4        ; Load index register Lo byte with value in tmp4
     mul                 ; Multiply X:A<-(X)*(A)
     sta     tmp7        ; Copy result to tmp7
     stx     tmp8        ; Copy value in index register Lo byte to tmp8
;
     lda     tmp1        ; Load accumulator with value in tmp1
     ldx     tmp4        ; Load index register Lo byte with value in tmp4
     mul                 ; Multiply X:A<-(X)*(A)
     add     tmp6        ; Add without carry, A<-(A)+(M)
     sta     tmp6        ; Copy result to tmp6
     txa                 ; Transfer value in index register Lo byte
                         ; to accumulator
     adc     tmp7        ; Add with carry, A<-(A)+(M)+(C)
     sta     tmp7        ; Copy result to tmp7
     bcc     UMUL32a     ; If C bit of CCR is clear, branch to UMUL32a:
     inc     tmp8        ; Increment value in tmp8


UMUL32a:
     lda     tmp2        ; Load accumulator with value in tmp2
     ldx     tmp3        ; Load index register Lo byte with value in tmp3
     mul                 ; Multiply X:A<-(X)*(A)
     add     tmp6        ; Add without carry, A<-(A)+(M)
     sta     tmp6        ; Copy result to tmp6
     txa                 ; Transfer value in index register Lo byte
                         ; to accumulator
     adc     tmp7        ; Add with carry, A<-(A)+(M)+(C)
     sta     tmp7        ; Copy result to tmp7
     bcc     UMUL32b     ; If C bit of CCR is clear, branch to UMUL32b:
     inc     tmp8        ; increment value in tmp8 variable


UMUL32b:
      rts                ; return from subroutine

;***************************************************************************
; ----------------------------- Include Files -----------------------------
;***************************************************************************


     org     $F000       ; Origin at Memory Location $F000 = 61440
                         ; (uses 1536) to Memory location $E500 = 62976

     include "ASCII_Conv_100.inc"     ; Converts 8 bit value to ASCII,
                                      ; 100s column
     include "ASCII_Conv_10.inc"      ; Converts 8 bit value to ASCII,
                                      ; 10s column
     include "ASCII_Conv_1.inc"       ; Converts 8 bit value to ASCII,
                                      ; 1s column
     include "DIYWBlambda.inc"        ; Converts WB O2 sensor voltage Raw
                                      ; Reading to Lambda * 10
     include "kpafactor4250rjh.inc"   ; Converts mapADC and baroADC
                                      ; to kpa
     include "thermfactor.inc"        ; Converts matADC and cltADC
                                      ; to degrees F+40

;***************************************************************************
; --------------------------- VFD Lookup Tables ---------------------------
;***************************************************************************


     org     $E000       ; Origin at Memory Location $E000 = 57334
                         ;(uses 357) to memory location $E15B = 57691
                         ; Flash ends at $FDFF = 65023

;***************************************************************************
; - These tables are the character strings for the top lines of the VFD
;   while in "Display" mode.
;***************************************************************************

VARS0_TL_TB:
     db     'ERPM MAP AFR CLT MAT'
            ; "ERPM" is Engine RPM, uses calculated "rpmH:rpmL"
            ; "MAP"  is Manifold Absolute Pressure in KPA, uses "map"
            ; "AFR"  is Air Fuel Ratio x 10, uses "afr"
            ; "CLT"  is Engine Coolant Temp in degrees F+40, uses "clt"
            ; "MAT"  is Manifold Air Temperature in degrees F+40, uses "mat"

VARS1_TL_TB:
     db     'TPP EOP EFP EXGT VLT'
            ; "TPP"  is Throttle position in percent, uses "tpsp"
            ; "EOP"  is Engine oil pressure in PSI, uses "oprs"
            ; "EFP"  is Engine fuel pressure in PSI, uses "fprs"
            ; "EXGT" is Engine exhaust gas temperature in degrees F, uses "egtH:egtL"
            ; "VLT"  is Battery Voltage x 10, uses "volts"

VARS2_TL_TB:
     db     ' ERPM AFR FDSC LTHR '
            ; "ERPM" is Engine RPM, uses calculated "rpmH:rpmL"
            ; "AFR"  is Air Fuel Ratio x 10, uses "afr"
            ; "FDSC" is Fuel Delivery Injector On Time over a 1 second period in mmS, uses "fdSecH:fdSecL"
            ; "LTHR" is Current fuel burn in L/Hr x 10, uses "ltrHrH:ltrHrL"

VARS3_TL_TB:
     db     'ERPM MAP AFR FTC PLW'
            ; "ERPM" is Engine RPM, uses calculated "rpmH:rpmL"
            ; "MAP"  is Manifold Absolute Pressure in KPA, uses "map"
            ; "AFR"  is Air Fuel Ratio x 10, uses "afr"
            ; "FTC"  is Fuel Trim Correction in percent, uses "Ftrimcor"
            ; "PLW"  is Pulse Width Lo Res in mmS, uses "pw"

VARS4_TL_TB:
     db     'ERPM MAP TRA CTA MTA'
            ; "ERPM" is Engine RPM, uses calculated "rpmH:rpmL"
            ; "MAP"  is Manifold Absolute Pressure in KPA, uses "map"
            ; "TRA"  is Ignition Trim Angle in degrees BTDC, uses "trmAng"
            ; "CTA"  is Calculated Timing Angle in degrees BTDC, uses "timAng"
            ; "MTA"  is Monitored Ignition Timing Angle in degrees BTDC, uses "monTimAng"

VARS5_TL_TB:
     db     ' GAM WCR ACR BCR TCR'
            ; "GAM" is Gamma Correction in percent, uses "gammae"
            ; "WCR" is Engine Coolant Correction in percent, uses "warmcor"
            ; "ACR" is Manifold Air Temperature Correction in percent, uses "aircor"
            ; "BCR" is Barometric Pressure Correction in percent, uses "barocor"
            ; "TCR" is Acceleration Correction in percent, uses "tpsaccel"

VARS6_TL_TB:
     db     ' ego EGV LMD AFR FTC'
            ; "ego" is Exhaust Gas Oxygen ADC, uses "egoADC"
            ; "EGV" is Exhaust Gas Oxygen voltage x 10, uses "egoV"
            ; "LMD" is Exhaust Gas Oxygen Lambda x 10, uses "lambda"
            ; "AFR"  is Air Fuel Ratio x 10, uses "afr"
            ; "FTC"  is Fuel Trim Correction in percent, uses "Ftrimcor"

VARS7_TL_TB:
     db     ' map MAP     BAR BCR'
            ; "map" is Manifold Pressure ADC, uses "mapADC"
            ; "MAP" is Manifold Absolute Pressure in KPA, uses "map"
            ; "BAR" is Barometric Pressure in KPA, uses "barometer"
            ; "BCR" is Barometric Pressure Correction in percent, uses "barocor"

VARS8_TL_TB:
     db     ' vlt VLT clt CLT WCR'
            ; "vlt" is Battery Voltage ADC, uses "battADC"
            ; "VLT" is Battery Voltage x 10, uses "volts"
            ; "clt" is Engine Coolant Temperature ADC, uses "cltADC"
            ; "CLT" is Engine Coolant Temperature in degrees F+40, uses "clt"
            ; "WCR" is Engine Coolant Correction in percent, uses "warmcor"

VARS9_TL_TB:
     db     'egt EXGT mat MAT ACR'
            ; "egt"  is Exhaust Gas Temperature ADC, uses "egtADC"
            ; "EXGT" is Engine exhaust gas temperature in degrees F, uses "egtH:egtL"
            ; "mat"  is Manifold Air Temperature ADC, uses "matADC"
            ; "MAT"  is Manifold Air Temperature in degrees F+40, uses "mat"
            ; "ACR"  is Manifold Air Temperature Correction in percent, uses "aircor"

VARS10_TL_TB:
     db      ' ftc  FTC  itc  TAF '
             ; "ftc"  is Fuel trim ADC, uses "ftrimADC"
             ; "FTC"  is Fuel Trim Correction in percent, uses "Ftrimcor"
             ; "itc"  is Ignition Trim Correction ADC, uses "itrimADC"
             ; "TAF"  is Ignition Trim Angle Factor, uses "trmAngFac"

VARS11_TL_TB:
     db      ' eop  EOP  efp  EFP '
             ; "eop"  is Engine oil pressure ADC, uses "eopADC"
             ; "EOP"  is Engine oil pressure in PSI, uses "oprs"
             ; "efp"  is Engine fuel pressure ADC, uses "efpADC"
             ; "EFP"  is Engine fuel pressure in PSI, uses "fprs"

VARS12_TL_TB:
     db      'PWHR PLW FLD VEC DTY'
             ; "PWHR" is Pulse Width Hi Res in mmmS, uses "pwcalcH:pwcalcL"
             ; "PLW"  is Pulse Width Lo Res in mmS, uses "pw"
             ; "FLD"  is Fuel Deleivery Pulse Width Lo Res in mmS, uses "fd"
             ; "VEC   is Volumetric Efficiency(Current VE table value in percent),
             ; uses "vecurr"
             ; "DTY"  is Injector Duty Cycle in percent, uses "duty"

VARS13_TL_TB:
     db      ' ERPM PIPD MNPD SEC '
             ; "ERPM" is Engine RPM, uses calculated "rpmhrH:rpmhrL"
             ;*; "PIPD" is PIP Period Predicted in mmmS, uses "pippprdH:pippprdL"
             ; "PIPD" is PIP Period in mmmS, uses "tachH:tachL"
             ; "MNPD" is Ignition Monitor Period in mmmS, uses "monpH:monpL"
             ; "SEC"  is Seconds Counter Lo byte, uses "secL"

VARS14_TL_TB:
     db      'ERPM MAP SAF DAF TAF'
             ; "ERPM" is Engine RPM, uses calculated "rpmH:rpmL"
             ; "MAP"  is Manifold Absolute Pressure in KPA, uses "map"
             ; "SAF"  is Spark Angle Factor(Current ST table value), uses "spkAngFac"
             ; "DAF"  is Delay Angle Factor, uses "dlyAngFac"
             ; "TAF"  is Ignition Trim Angle Factor, uses "trmAngFac"

VARS15_TL_TB:
     db      'ERPM TRA DLA CTA MTA'
             ; "ERPM" is Engine RPM, uses calculated "rpmH:rpmL"
             ; "TRA"  is Ignition Trim Angle in degrees BTDC, uses "trmAng"
             ; "DLA"  is Ignition Delay Angle in degrees BTDC, uses "dlyAng"
             ; "CTA"  is Calculated Timing Angle in degrees BTDC, uses "timAng"
             ; "MTA"  is Monitored Ignition Timing Angle in degrees BTDC, uses "monTimAng"


VARS16_TL_TB:
     db      'FP RN CR SW RW AC DC'
             ; "FP"   is Fuel Pump(portAbits 0)
             ; "RN"   is Engine Running(engine 0)
             ; "CR"   is Engine Cranking(engine 1)
             ; "SW"   is Start Warmup(engine 2)
             ; "RW"   is Run Warmup(engine 3)
             ; "AC"   is Accellerating(engine 4)
             ; "DC"   is Decelerating(engine 5)



VARS17_TL_TB:
     db      'EA RL FC       FT IT'
             ; "EA"   is Engine Alarm(portCbits 0)
             ; "RL"   is Rev Limiter(alarmbits 5)
             ; "FC"   is Flood Clear(alarmbits 6)
             ; "FT"   is Fuel Trim Enable(portAbits 5)
             ; "IT"   is Ignition Trim Enable(portAbits 2)


VARS18_TL_TB:
     db      'EA HT LO LF HF ET KN'
             ; "EA"   is Engine Alarm(portCbits 0)
             ; "HT"   is High engine temperature(alarmbits 1)
             ; "LO"   is Low engine oil pressure(alarmbits 0)
             ; "LF"   is Low fuel pressure(alarmbits 2)
             ; "HF"   is High fuel pressure(alarmbits 3)
             ; "ET"   is High exhaust gas temperature(alarmbits 4)
             ; "KN"   is Engine knock detected(alarmbits 7)


;***************************************************************************
; - This table is the 16 bit vector address index, for the tables of the
;   character strings, for the top lines of the VFD while in "Display" mode.
;***************************************************************************

VARS_TL_TB_IND:
     dw     VARS0_TL_TB         ; ScrnCnt=0  'ERPM MAP AFR CLT MAT'
     dw     VARS1_TL_TB         ; ScrnCnt=1  'TPP EOP EFP EXGT VLT'
     dw     VARS2_TL_TB         ; ScrnCnt=2  ' ERPM AFR FDSC LTHR '
     dw     VARS3_TL_TB         ; ScrnCnt=3  'ERPM MAP AFR FTC PLW'
     dw     VARS4_TL_TB         ; ScrnCnt=4  'ERPM MAP TRA CTA MTA'
     dw     VARS5_TL_TB         ; ScrnCnt=5  ' GAM WCR ACR BCR TCR'
     dw     VARS6_TL_TB         ; ScrnCnt=6  ' ego EGV LMD AFR FTC'
     dw     VARS7_TL_TB         ; ScrnCnt=7  ' map MAP     BAR BCR'
     dw     VARS8_TL_TB         ; ScrnCnt=8  ' vlt VLT clt CLT WCR'
     dw     VARS9_TL_TB         ; ScrnCnt=9  'egt EXGT mat MAT ACR'
     dw     VARS10_TL_TB        ; ScrnCnt=10 ' ftc  FTC  itc  TAF '
     dw     VARS11_TL_TB        ; ScrnCnt=11 ' eop  EOP  efp  EFP '
     dw     VARS12_TL_TB        ; ScrnCnt=12 'PWHR PLW FLD VEC DTY'
     dw     VARS13_TL_TB        ; ScrnCnt=13 ' ERPM PIPD MNPD SEC '
     dw     VARS14_TL_TB        ; ScrnCnt=14 'ERPM MAP SAF DAF TAF'
     dw     VARS15_TL_TB        ; ScrnCnt=15 'ERPM TRA DLA CTA MTA'
     dw     VARS16_TL_TB        ; ScrnCnt=16 'FP RN CR SW RW AC DC'
     dw     VARS17_TL_TB        ; ScrnCnt=17 'EA RL FC       FT IT'
     dw     VARS18_TL_TB        ; ScrnCnt=18 'EA HT LO LF HF ET KN'

;****************************************************************************
; - Interrupt Vector table
;****************************************************************************

     org     vec_timebase  ; Origin at $FFDC = 65500


	dw	Dummy          ;Time Base Vector
	dw	Dummy          ;ADC Conversion Complete
	dw	Dummy          ;Keyboard Vector
	dw	SCITX_ISR      ;SCI Transmit Vector
	dw	SCIRCV_ISR     ;SCI Receive Vector
	dw	Dummy          ;SCI Error Vecotr
	dw    Dummy          ;SPI Transmit Vector
	dw	Dummy          ;SPI Receive Vector
	dw    Dummy          ;TIM2 Overflow Vector
	dw	Dummy          ;TIM2 Ch1 Vector
	dw	TIM2CH0_ISR    ;TIM2 Ch0 Vector
	dw	Dummy          ;TIM1 Overflow Vector
	dw	Dummy          ;TIM1 Ch1 Vector
	dw	Dummy          ;TIM1 Ch0 Vector
	dw    Dummy          ;PLL Vector
	dw	Dummy          ;IRQ Vector
	dw    Dummy          ;SWI Vector
	dw	Start          ;Reset Vector

	end

