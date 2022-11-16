;***********************************************************************************************
;
;   MSnS351WM.inc
;
;   Based on the work of Bruce Bowling and Al Grippo for the original B&G and Hi Res code.
;   Also Magnus Bjelk for his megasquirtnspark and the msnsextra team as well as all the
;   unknown contributors to the MS project.
;   Special thanks to Eric Fahlgren with the early help integrating with Megatune, and
;   Phil Tobin for assisting in integrating with Tuner Studio
;
;   Robert Hiebert 2010
;
;***********************************************************************************************
;***********************************************************************************************
;   This version of the MSnS300/460 code is designed specifically for use on a Ford 351
;   Windsor marine engine, converted from carburation to EFI. The conversion was done using EFI
;   components from the 5.8L truck engines dated ~1992. The upper intake manifold was modified
;   to fit in the engine space, otherwise all stock components were used including 19lb/hr
;   injectors and TFI igntion. The system is speed density and fires the injectors in banks of
;   four, alternately, every ignition event. A WB O2 sensor is installed and is used for tuning
;   and reference only as the engine runs open loop in all conditions.
;***********************************************************************************************
;***********************************************************************************************
;
; Rev 1: 1/1/11:  - Added Hi Res tach feature, crankshaft trigger detection feature and
;                   crankshaft trigger error compensation feature
;
; Rev 2: 4/12/11: - Put ADC averaging back in.
;
; Rev 2: 7/07/11: - Add knock detection code
;
; Rev 2: 8/17/11: - Change fuel delivery calculations to try reduce deadband error
;
; Rev 2: 9/25/11: - Comment out Loop Counter code. Add code to knock sensor to fall through if
;                   bit is already set or cleared. Add configureable constants for all alarms
;                   and "runon", "startedon", and "warmoff". Modify AFR2 table. Update VE and
;                   ST with latest tune.
;
; Rev 2: 4/16/12: - ASE counter and bank flow rate variables and code changes
;
; Rev 2: 7/02/12: - Modify code to average Hi Res Tach period over 16 samples to smooth out display
;
; Rev 2: 8/01/12: - Revise period averaging for Hi Res Tach calcs. Total periods for 250 ms
;                   then divide by "tachcnt". Code is slower but works well.
;                   Average "batt", "ego", "OP_ADC", "FP_ADC", and "EGT_ADC"  over 16 iterations
;                   to smooth things out (~176 ms update)
;
; Rev 9: 8/02/12: - Eliminate "aseHi", aseLo", "trigErr", "dbcor" and "warmoff" from .asm
;
;
; Rev 10: 8/03/12 - Modify code to allow knock detection to sound alarm only when ignition trim
;                   or fuel trim is active.
;
; Rev 11: 8/08/12 - Add code for hot start ASE.
;
;***********************************************************************************************
;***********************************************************************************************
;
; ---------------------------------- MSnS351WM Hardware Wiring ---------------------------------
;
;***********************************************************************************************
;
; ----- Inputs [port name - function] -----
;
;  IRQ	    - PIP (Profile Ignition Pickup)Input (Rising edge at 10 degrees BTDC on Pin 24,
;                 inverted to IRQ)
;  PTB0/AD0     - Manifold Absolute Pressure
;  PTB1/AD1     - Manifold Air Temperature
;  PTB2/AD2     - Coolant Temperature
;  PTB3/AD3     - Throttle Position Sensor
;  PTB4/AD4     - Battery Voltage
;  PTB5/AD5     - Exhaust Gas Oxygen
;  PTB6/AD6     - Fuel Trim
;  PTB7/AD7     - MC14051B analog multiplexer common output
;  MC14051B Ch0 - Ignition Trim
;  MC14051B Ch1 - Engine Oil Pressure
;  MC14051B Ch2 - Fuel Presure
;  MC14051B Ch3 - Exhaust Gas Temperature
;  PTA2         - Ignition Trim Enable
;  PTA3         - Knock Sensor
;  PTA4         - Ignition Monitor
;  PTA5         - Fuel Trim Enable
;
; ----- Outputs [port name - function] -----
;
;  PTD4/T1CH0   - Injector 1
;  PTD5/T1CH1   - Injector 2
;  PTA0         - Fuel Pump Relay driver transistor, sinking
;  PTA1	    - SPOUT (Spark Output) driver transistor, sinking, with pullup, for TFI module,
;                 50% duty cycle, fires on rising edge
;  PTA6         - MC14051B input "B"
;  PTA7         - MC14051B input "A"
;  PTC0	    - Accumulated Injector On Time(open collector)
;  PTC1	    - Engine Alarm
;  PTC2	    - Program loop counter LED (Spare Output)
;
;***********************************************************************************************
;***********************************************************************************************
; ram_start         = memory location $0040(64T)(GP32equ)
; ram_last          = memory location $023F(575T)(GP32equ)
; rom_start         = memory location $8000(32768)(GP32equ)
; rom_last          = memory location $FDFF(65023)(GP32equ)
; init_stack        = ram_exec = memory location $01ED(493)(bootr12)
; ms_ram_start      = memory location $0040(64)(MS_ECU_HP.h)
; ms_ram_end        = memory location $00D0(208)(MS_ECU_HP.h)
; ms_rf_start       = memory location $00D0(208)(MS_ECU_HP.h)
; ms_rf_end         = memory location $01B0(432)(MS_ECU_HP.h)
; ms_ram_size       = {ms_ram_end-ms_ram_start}($00D0-$0040=$0090)(208-64=144)(MS_ECU_HP.h)
; ms_rf_size        = {ms_rf_end-ms_rf_start}}($01B0-$00D0=$00E0)(432-208=224)(MS_ECU_HP.h)
; ms_total_ram_size = {ms_rf_end-ms_ram_start}}($01B0-$0040=$0170)(432-64=368)(MS_ECU_HP.h)
;***********************************************************************************************

.header 'MegaSquirt'    	            ; Listing file title
.pagewidth 130          	            ; Listing file width
.pagelength 90          	            ; Listing file height

.nolist                                   ; Turn off listing file
     include "gp32.equ"                   ; Include HC 908 equates
.list                                     ; Turn on listing file
     org	   ram_start                  ; Origin  Memory location $0040(64)
     include "MSnS351WM.inc"              ; Include definitions for
                                          ; MSnS351WM.asm


;***********************************************************************************************
;
; Main Routine Here - Initialization and main loop
;
; Note: Org down 128 bytes below the "rom_start" point because of erase bug in bootloader
;       routine. Only B&G HiRes and Megasquirtnspark use the 256 offset, all others use 128???
;
;***********************************************************************************************

	org	{rom_start + 128}     ; Origin at memory location $8000 + $80 = $8080
                                  ; (32768 + 128 = 32896)
Start:
	ldhx	#init_stack+1    ; Load index register with value in init_stack+1
                             ;(Set the stack Pointer)($01ED + 1 = $01EE = 494
	txs                    ; Transfer value in index register Lo byte to stack
                             ;(Move before burner to avoid conflict)


;***********************************************************************************************
; - Set the phase lock loop for a bus frequency of 8.003584mhz
;  (Boot loader initially sets it at 7.3728mhz)
;***********************************************************************************************

;PllSet:
	bclr	BCS,pctl		; Select external Clock Reference
	bclr	PLLON,pctl		; Turn Of PLL
	mov	#$02,pctl		; Set P and E Bits
	mov	#$D0,pmrs		; Set L ($C0 for 7.37 MHz)
	mov	#$03,pmsh		; Set N (MSB)
	mov	#$D1,pmsl		; Set N (LSB) ($84 for 7.37 MHz)
	bset	AUTO,pbwc         ; Enable automatic bandwidth control
	bset	PLLON,pctl		; Turn back on PLL

PLL_wait:
     brclr   LOCK,pbwc,PLL_wait  ; Wait for PLL to lock
     bset    BCS,pctl            ; Select VCO as base clock

;***********************************************************************************************
; - Set up the port data-direction registers, Set directions
;   Preset state of pins to become outputs
;***********************************************************************************************

; Port A
     mov     #$FF,PTAPUE     ; Move %11111111 to Port A pullup enable reg Set all pullups to
                             ; (connect unused pins to VDD, output direction clears pullup)
     mov     #$02,PORTA      ; Move %00000010 to Port A Data Regisister (preinit) Loopchk Lo,
                             ; Spark Hi, fuelp Lo,(SPOUT Lo, fuel pump off)
     mov     #$C3,DDRA       ; Move %11000011 to Port A Data Direction Register
                             ; Inputs on PTA5,4,3,2, = NA,NA,Ftrm en,Ign_Mon,Knk_Det,Itrm en
                             ; Outputs on PTA7,6,1,0 = MVa,MVb,Spout,Fuel Pump

; Port B
     clr     DDRB            ; Clear Port B Data Direction Register (Set as inputs,
                             ; "ADSEL" will select channel)

; Port C
     mov     #$FF,PTCPUE     ; Move %11111111 to Port C pullup enable reg (Set all pullups to
                             ; connect unused pins to VDD, output direction clears pullup)
     clr          PORTC      ; Move %00000000 to Port C Data Register(Preinit all low)
     mov     #$07,DDRC       ; Move %00000111 to Port C Data Direction Register,
                             ; Inputs on PTC7,6,5,4,3 = NA,NA,NA,NA,NA
                             ; Outputs on PTC2,1,0 = spare(loop chk),Eng Alarm,AIOT

; Port D
     mov     #$FF,PTDPUE     ; Move %11111111 to Port C pullup enable reg (Set all pullups to
                             ; connect unused pins to VDD, output direction clears pullup)
     mov     #$30,PORTD      ; Move %00110000 to Port D Data Direction Register (preinit output
                             ; pins 5,4 Hi)(Turn off injectors (inverted output)
     mov     #$F0,DDRD       ; Move %11110000 to Port D Data Direction Register (Injectors on
                             ; PTD4,5)(PTD6,7 not available, set as outputs)

; Port E
     clr     PORTE         ; Clear Port E Data Register (to avoid glitches)
     mov     #$01,DDRE     ; Move %00000001 to Port E Data Direction Register(Serial Comm Port)


;***********************************************************************************************
; - Set up TIM2 as a free running ~1us counter. Set Channel 0 output compare
;   to generate the ~100us(.1ms) clock tick. Interupt vector "TIM2CH0_ISR:"
;   Set Channel 1 output compare for SPOUT control.
;   Interupt vector "TIM2CH1_ISR:"
; - NOTE! The channels on this timer are not connected to any pins so pin
;   state is irrelevant.
;***********************************************************************************************

     mov     #$33,T2SC       ; Move %00110011 into Timer2 Status and Control Register
                             ;(Disable interupts, stop timer)(Prescale and counter cleared))
                             ;(Prescale for bus frequency / 8)
     mov     #$FF,T2MODH     ; Move dec 255 into T2 modulo register Hi
     mov     #$FF,T2MODL     ; Move dec 255 into T2 modulo register Lo(free running timer)
     clr     T2CH0H          ; Clear T2CH0 O/C register Hi byte
     mov     #$64,T2CH0L     ; Move decimal 100 into T2CH0 O/C register Lo byte(~100uS)
     mov     #$54,T2SC0      ; Move %01010100 into T2CH0 Status and Control Register.
                             ;(Output compare, interrupt enabled)
     clr     T2CH1H          ; Clear T2CH1 O/C register Hi byte
     clr     T2CH1L          ; Clear T2CH1 O/C register Lo byte
     mov     #$14,T2SC1      ; Move %00010100 into T2CH1 Status and Control Register.
                             ; (Output compare, interrupt disabled)
     mov     #$13,T2SC       ; Move %00010011 into Timer2 Status and Control Register.
                             ; (Disable interupts, reset counter, counter Active,
                             ; Prescale for bus frequency /8. 8,003,584hz/8=1,000,448hz
                             ; = .0000009995sec


;***********************************************************************************************
; Set up TIM1 as a free running ~1us counter. Set channels 0 and 1 for output
; compare, pulsewidth control of injectors. INJ1 on T1SC0 and INJ2 on T1SC1
; Initialize output pins logic state 1 (injectors off)
;***********************************************************************************************

     mov     #$33,T1SC       ; Move %00110011 into Timer1 Status and Control Register
                             ;(Disable interupts, stop timer)(Prescale and counter cleared))
                             ;(Prescale for bus frequency / 8)
     mov     #$FF,T1MODH     ; Move decimal 255 into T1 modulo reg Hi
     mov     #$FF,T1MODL     ; Move decimal 255 into T1 modulo reg Lo
     mov     #$00,T1SC0      ; Move %00000000 into Timer1 Channel 0 Status and Control Register
                             ;(Normal port output initialized logic 1)(injector off)
     mov     #$00,T1SC1      ; Move %00000000 into Timer1 Channel 1 Status and Control Register
                             ;(Normal port output initialized logic 1)(injector off)
     mov     #$00,T1CH0H     ; Move decimal 0 to T1CH0 register Hi
     mov     #$01,T1CH0L     ; Move decimal 1 to T1CH0 register Lo
     mov     #$00,T1CH1H     ; Move decimal 0 to T1CH0 register Hi
     mov     #$01,T1CH1L     ; Move decimal 1 to T1CH0 register Lo


;***********************************************************************************************
; - Set up Serial Communications Interface Module
;***********************************************************************************************

     mov     #$30,SCBR       ; Move %0110000 into SCI Baud Rate Register
                             ; (8003584/(64*13*1)=9619.7 baud)
     bset     ENSCI,SCC1     ; Set enable SCI bit of SCI Control Register 1(Enable SCI)
     bset     RE,SCC2        ; Set receiver enable bit of SCI Control Reg 2(Enable receiver)
     bset     SCRIE,SCC2     ; Set SCI receive interrupt enable bit of SCI Control Reg 2
                             ;(Enable Rcv. Interrupt)
     lda      SCS1           ; Load accumulator with SCI Status Register 1
                             ;(Clear SCI transmitter Empty Bit)
     clr      TXCNT          ; Clear SCI transmitter count(incremented)(characters transmitted)
     clr      TXGOAL         ; Clear SCI number of bytes to transmit
                             ;(characters to be transmitted)


;***********************************************************************************************
; - Set up IRQ Interrupt (Rising edge of PIP on Pin 24, hardware inverted)
;***********************************************************************************************

     mov     #$04,INTSCR     ; Move %00000100 into IRQ Status and Control Register
                             ; (Enable IRQ (turn on interrupts))(falling edge only)


;***********************************************************************************************
; - Set up Keyboard Interrupt, Ign Mon on PTA4
;***********************************************************************************************

     mov     #$02,INTKBSCR       ; Move %00000010 into Keyboard Status and Control Register
                                 ;(Keyboard interrupts masked)(interrupts on falling edges only)
     mov     #$10,INTKBIER       ; Move %0010000 into Keyboard Interrupt Enable Register
                                 ;(Keyboard interrupts on PTA4)
     bset    ackk,INTKBSCR       ; Set the Keyboard Acknowledge bit of Keyboard Status and
                                 ; Control Register(clear any false interrupts)
     bset    MONen,inputs        ; Set "MONen" bit of "inputs" variable
     bclr    imaskk,INTKBSCR     ; Clear the Interrupt Mask bit of Keyboard Status and Control
                                 ; Register(enable interrupts)

;***********************************************************************************************
;   Set all RAM to known value - for code runaway protection.
;   If there is ever a code runaway, and processor tries executing this as an opcode ($32)
;   then a reset will occur.
;***********************************************************************************************

	ldhx   #ram_start      ;Point to start of RAM $0040(64)

ClearRAM:
	lda	#$32             ; This is an illegal op-code - cause reset if executed
	sta    ,x              ;Set RAM location
	aix    #1              ;advance pointer
	cphx   #ram_last+1     ;$23F + 1 = $240(576)  done ?
	bne    ClearRAM        ;loop back if not

;***********************************************************************************************
; - Clear all variables
;***********************************************************************************************

     clr     secl         ; low seconds - from 0 to 255, then rollover
     clr     map          ; Manifold Absolute Pressure ADC Raw Reading - KPa (0 - 255)(AD0)
     clr     mat          ; Manifold Air Temp ADC Raw Reading - counts (0 - 255)(AD1)
     clr     clt          ; Coolant Temperature ADC Raw Reading - counts (0 - 255)(AD2)
     clr     tps          ; Throttle Position Sensor ADC Raw Reading - counts (0 - 255)(AD3)
     clr     batt         ; Battery Voltage ADC Av Reading - counts (0 - 255)(AD4)
     clr     ego          ; Exhaust Gas Oxygen ADC Av Reading - counts (0 - 255)(AD5)
     clr     Ftrm_ADC     ; Fuel Trim ADC Raw Reading - counts (0 - 255)(AD6)
     clr     Itrm_ADC     ; Ignition Trim ADC Raw Reading - counts (0 - 255)(AD7, Mplx ch0)
     clr     OP_ADC       ; Engine Oil Pressure ADC Av Reading - counts (0 - 255)(AD7, Mplx ch1)
     clr     FP_ADC       ; Fuel Pressure ADC Av Reading - counts (0 - 255)(AD7, Mplx ch2)
     clr     EGT_ADC      ; Exhaust Gas Temperature ADC Av Reading - counts (0 - 255)(AD7, Mplx ch3)
     clr     barometer    ; Current barometer reading in KPA (for MV)
     clr     barocor      ; Barometer Lookup Correction - percent
     clr     warmcor      ; Total Warmup Correction - percent
     clr     aircor       ; Air Density Correction lookup - percent
     clr     Ftrimcor     ; Fuel Trim Correction Factor (85% - 115%)
     clr     gammae       ; Total Gamma Enrichments - percent
     clr     tpsaccel     ; Acceleration enrichment - percent
     clr     rpm20        ; Computed engine RPM - rpm/20
     clr     vecurr       ; Current VE value from lookup table - percent
     clr     pwcalcH      ; high order of calculated pulsewith (16 bits)
     clr     pwcalcL      ; low order of calculated pulsewidth (16 bits)
     clr     pw           ; injector squirt time in 1/10 millesec (0 to 25.5 millisec) - applied
     clr     fd           ; Fuel Delivery PW Lo Res
     clr     fdSecH       ; Fuel Delivery PW Lo Res over 1 second Hi byte
     clr     fdSecL       ; Fuel Delivery PW Lo Res over 1 second Lo byte
     clr     tachH        ; Tachometer period averaged over 16 periods Hi byte
     clr     tachL        ; Tachometer period averaged over 16 periods Lo byte
     clr     Spk_Ang_Fac  ; Spark Angle Factor (from ST table)
     clr     Trm_Ang_Fac  ; Trim Angle Factor (from IgnTrimcor table)
     clr     Dly_Ang_Fac  ; Delay Angle Factor (Spk_Ang_Fac + Trm_Ang_Fac)
     clr     MON_pH       ; MON period Hi byte (MON_tsH - PIP_tsH)
     clr     MON_pL       ; MON period Lo byte (MON_tsL - PIP_tsL)
     clr     tpsp         ; Throttle position percent
     clr     engine       ; Variable bit-field to hold engine current status
     clr     alarmbits    ; Engine Alarm Status Bit Field
     clr     portAbits    ; Port A status Bit field
     clr     portCbits    ; Port C Status Bit Field
     clr     engine2      ; Variable bit-field to hold engine current status #2
     clr     squirt       ; Event variable bit field for Injector Firing
     clr     inputs       ; Bit field variable for input status flags
     clr     kpa          ; MAP value in units of KPa
     clr     coolant      ; Coolant temperature in Degrees F plus 40 (allows -40 degress to fit in int)
     clr     adsel        ; ADC Selector Variable
     clr     adsel2       ; ADC Selector Variable 2
     clr     pwnadH       ; Calculated PW without Accel and deadband Hi byte
     clr     pwnadL       ; Calculated PW without Accel and deadband Lo byte
     clr     fdhrH        ; Calc PW without deadband Hi byte (fuel delivery)
     clr     fdhrL        ; Calc PW without deadband Lo byte (fuel delivery)
     clr     deadband     ; Injector deadband in 100uS resolution
     clr     rpmph        ; High part of RPM Period
     clr     rpmpl        ; Low part of RPM Period
     clr     rpmch        ; Counter for high part of RPM
     clr     rpmcl        ; Counter for low part of RPM
     clr     mSx250       ; 250 Milliseconds counter
     clr     mSx100       ; 100 Milliseconds counter
     clr     mS           ; Milliseconds counter
     clr     uSx100       ; 100 Microseconds counter
     clr     sech         ; high seconds - rollover at 65536 secs (1110.933 minutes, 18.51 hours)
     clr     tpsaclk      ; TPS enrichment timer clock in 0.1 second resolution
     clr     asecount     ; Counter value for after-start enrichment counter - every ignition pulse
     clr     last_tps     ; TPS reading updated every 0.1 seconds
     clr     igncount     ; Ignition pulse counter
     clr     altcount     ; Alternate count selector
     clr     tpsaclkcmp   ; Comparison value for TPS acceleration time - from lookup table
     clr     tpsfuelcut   ; TPS Fuel Cut (percent)
     clr     txcnt        ; SCI transmitter count (incremented)
     clr     txgoal       ; SCI number of bytes to transmit
     clr     txmode       ; Transmit mode flag
     clr     rxoffset     ; offset placeholder when receiving VE/constants vis. SCI
     clr     INTACC1
     clr     INTACC2
     clr     tmp1
     clr     tmp2
     clr     tmp3
     clr     tmp4
     clr     tmp5
     clr     tmp6
     clr     tmp7
     clr     tmp8
     clr     tmp9
     clr     tmp10
     clr     tmp11
     clr     tmp12
     clr     tmp13
     clr     tmp14
     clr     tmp15
     clr     tmp16
     clr     tmp17
     clr     tmp18
     clr     tmp19
     clr     tmp20
     clr     tmp21
     clr     tmp22
     clr     PIP_tsH         ; PIP timestamp current reading Hi byte
     clr     PIP_tsL         ; PIP timestamp current reading Lo byte
     clr     PIP_tsH_prv     ; PIP timestamp previous reading Hi byte
     clr     PIP_tsL_prv     ; PIP timestamp previous reading Lo byte
     clr     PIP_pH_prv      ; PIP period Hi byte previous
     clr     PIP_pL_prv      ; PIP period Lo byte previous
     clr     PIP_pH_dif      ; PIP period difference Hi byte
                             ;(PIP_pH - PIP_pH_prv)
     clr     PIP_pL_dif      ; PIP period difference Lo byte
                             ;(PIP_pL - PIP_pL_prv)
     clr     PIP_pH          ; PIP period Hi byte (PIP_tsH - PIP_tsH_prv)
     clr     PIP_pL          ; PIP period Lo byte (PIP_tsL - PIP_tsL_prv)
     clr     PIP_pH_pred     ; Predicted PIP period Hi byte(PIP_pH + PIP_pH_dif)
     clr     PIP_pL_pred     ; Predicted PIP period Lo byte(PIP_pL + PIP_pL_dif)
     clr     Delay_pH        ; Delay period from PIP to SPOUT, Hi byte
     clr     Delay_pL        ; Delay period from PIP to SPOUT, Lo byte
                             ;(16 bit result of Mul_Hi:Mul_Mid:Mul_Lo / $FF)
     clr     CH1_onH         ; T2CH1 output compare for SPOUT rising, Hi byte
     clr     CH1_onL         ; T2CH1 output compare for SPOUT rising, Lo byte
                             ;(PIP_tsH:PIP_tsL + Delay_pH:Delay_pL)
     clr     SPOUT_tsH       ; SPOUT timestamp current reading Hi byte
     clr     SPOUT_tsL       ; SPOUT timestamp current reading Lo byte
     clr     PIP_periodH     ; PIP period Hi byte (for SPOUT off calcs)
     clr     PIP_periodL     ; PIP period Lo byte (for SPOUT off calcs)
     clr     SPOUTon_pH      ; SPOUT on period Hi byte
     clr     SPOUTon_pL      ; SPOUT on period Lo byte
                             ;(50% duty cycle, half of PIP period)
     clr     CH1_offH        ; T2CH1 output compare for SPOUT falling, Hi byte
     clr     CH1_offL        ; T2CH1 output compare for SPOUT falling, Lo byte
                             ;(SPOUT_tsH:SPOUT_tsL + SPOUTon_pH:SPOUTon_pL)
     clr     MON_tsH         ; MON timestamp Hi byte current reading
     clr     MON_tsL         ; MON timestamp Lo byte current reading
     clr     LoopCounter     ; Loop counter for main loop frequency check
     clr     MinPIP          ; Minimum PIP signals required for period calcs
     clr     page            ; Page selection variable
     clr     flocker         ; Burner locking semaphor
     clr     burnSrc         ; Burn routine variable
     clr     burnDst         ; Burn routine variable
     clr     burnCount       ; Burn routine variable
     clr     fdcntr          ; Counter for AIOT trigger on
     clr     aiotcntr        ; Counter for AIOT trigger off
     clr     fdtH            ; Fuel deleivery Lo Res total Hi byte
     clr     fdtL            ; Fuel deleivery Lo Res total Lo byte
     clr     tachcurH        ; Tachometer period current Hi byte
     clr     tachcurL        ; Tachometer period current Lo byte
     clr     tachTH          ; Tachometer period total Hi byte
     clr     tachTM          ; Tachometer period total Mid byte
     clr     tachTL          ; Tachometer period total Lo byte
     clr     BnkflowHmv      ; Injector bank flow rate L/hr x 10 Hi byte for MV
     clr     BnkflowLmv      ; Injector bank flow rate L/hr x 10 Lo byte for MV
     clr     tachcnt         ; Tachometer counter for tachometer period averaging time
     clr     battTH          ; Battery ADC total Hi Byte
     clr     battTL          ; Battery ADC total Lo Byte
     clr     battcnt         ; Counter for battery ADC averaging
     clr     egoTH           ; EGO ADC total Hi Byte
     clr     egoTL           ; EGO ADC total Lo Byte
     clr     egocnt          ; Counter for EGO ADC averaging
     clr     OP_ADCTH        ; Engine Oil Pressure ADC total Hi Byte
     clr     OP_ADCTL        ; Engine Oil Pressure ADC total Lo Byte
     clr     OP_ADCcnt       ; Counter for Engine Oil Pressure ADC averaging
     clr     FP_ADCTH        ; Fuel Pressure ADC total Hi Byte
     clr     FP_ADCTL        ; Fuel Pressure ADC total Lo Byte
     clr     FP_ADCcnt       ; Counter for Fuel Pressure ADC averaging
     clr     EGT_ADCTH       ; Exhaust Gas Temperature ADC total Hi Byte
     clr     EGT_ADCTL       ; Exhaust Gas Temperature ADC total Lo Byte
     clr     EGT_ADCcnt      ; Counter for Exhaust Gas Temperature ADC averaging
     clr     mapcur          ; Manifold Absolute Pressure ADC Raw Reading - KPa (0 - 255)(AD0)
     clr     matcur          ; Manifold Air Temp ADC Raw Reading - counts (0 - 255)(AD1)
     clr     cltcur          ; Coolant Temperature ADC Raw Reading - counts (0 - 255)(AD2)
     clr     tpscur          ; Throttle Position Sensor ADC Raw Reading - counts (0 - 255)(AD3)
     clr     battcur         ; Battery Voltage ADC Raw Reading - counts (0 - 255)(AD4)
     clr     egocur          ; Exhaust Gas Oxygen ADC Raw Reading - counts (0 - 255)(AD5)
     clr     Ftrm_ADCcur     ; Fuel Trim ADC Raw Reading - counts (0 - 255)(AD6)
     clr     Itrm_ADCcur     ; Ignition Trim ADC Raw Reading - counts (0 - 255)(AD7, Mplx ch0)
     clr     OP_ADCcur       ; Engine Oil Pressure ADC Raw Reading - counts (0 - 255)(AD7, Mplx ch1)
     clr     FP_ADCcur       ; Fuel Pressure ADC Raw Reading - counts (0 - 255)(AD7, Mplx ch2)
     clr     EGT_ADCcur      ; Exhaust Gas Temperature ADC Raw Reading - counts (0 - 255)(AD7, Mplx ch3)
     clr     ADCcnt          ; ADC channel counter
     clr     airtemp         ; Manifold Air Temperature in degrees F + 40

;***********************************************************************************************
; - Initialize critical variables to some other acceptable starting value
;***********************************************************************************************

     lda     #$03        ; Decimal 3
     sta     MinPip

     lda     #$32        ; Decimal 50
     sta     tpscur

     lda     #$64        ; Decimal 100
     sta     mapcur
     sta     aircor
     sta     vecurr
     sta     barocor
     sta     warmcor
     sta     Ftrimcor

     lda     #$70        ; Decimal 112
     sta     battcur

     lda     #$73        ; Decimal 115
     sta     matcur
     sta     cltcur

     lda     #$80        ; Decimal 128
     sta     Ftrm_ADCcur
     sta     Itrm_ADCcur

     lda     #$82        ; Decimal 130
     sta     OP_ADCcur
     sta     FP_ADCcur

     lda     #$FF
     sta     last_tps
     sta     PIP_pH_prv     ; 305 RPM 6 cyl, 229 RPM 8 cyl
     sta     PIP_pL_prv     ; 305 RPM 6 cyl, 229 RPM 8 cyl
     sta     tachH          ; 305 RPM 6 cyl, 229 RPM 8 cyl
     sta     tachL          ; 305 RPM 6 cyl, 229 RPM 8 cyl

;*********************************************************************************************
; Copy injector bank flow rate configurable constants from Flash memory to RAM for use by
; Megaview in fuel burn calculations.
;*********************************************************************************************

     lda     BnkflowH     ; Load accumulator with value in "BnkflowH"
     sta     BnkflowHmv   ; Copy to "BnkflowHmv"
     lda     BnkflowL     ; Load accumulator with value in "BnkflowL"
     sta     BnkflowLmv   ; Copy to "BnkflowLmv"

;***********************************************************************************************
; - Fire up the ADC, and perform one conversion, Set up clock source for ADC
;   Do an initial conversion for barometric pressure and barometric pressure correction
;***********************************************************************************************

;GetBaro:
     lda     #%01110000     ; Load accumulator with %01110000
     sta     ADCLK          ; Copy to ADC Clock Register( bus clock/8 = ~1mhz )
     lda     #%00000000	    ; Load accumulator with %0000000(one conversion, no interrupt on
                            ; channel AD0)
     sta     ADSCR          ; Copy to ADC Status and Control Register

ADCWait:
     brclr   coco,ADSCR,ADCWait     ; If "conversions complete flag" bit of ADC Status and
                                    ; Control Register is clear, branch to ADCWait:
                                    ;(keep looping while COnversion COmplete flag = 0)
     clrh                           ; Clear index register Hi Byte
     lda     ADR                    ; Load accumulator with value in ADC Result Variable
     tax                            ; Copy to index register Lo byte
     lda     KPAFACTOR4250rjh,x     ; Load accumulator with value in KPAFACTOR4250rjh table
                                    ; offset in index register Lo byte
     sta     barometer              ; Copy to Barometric Pressure  in Kilopascals
     lda     BAROFAC4250rjh,x       ; Load accumulator with value in BAROFAC4250rjh  table
                                    ; offset in index register Lo byte
     sta     barocor                ; Copy to Barometric Pressure correction factor



;***********************************************************************************************
;  Do the ADC conversion again for Coolant Temperature ADC, then determine the
;  actual temperature from the Thermfac table
;***********************************************************************************************

     lda     #%00000010	    ; Load accumulator with %00000010
                            ;(one conversion, no interrupt on channel AD2)
     sta     ADSCR          ; Copy to ADC Status and Control Register

ADCWait2:
     brclr   coco,ADSCR,ADCWait2 ; If "conversions complete flag" bit of ADC Status and Control
                                 ; Register is clear, branch to ADCWait2:
                                 ;(keep looping while COnversion COmplete flag = 0)
     lda     ADR                 ; Load accumulator with value in ADC Result Variable
     sta     clt                 ; Copy to Coolant Temperature 8 bit ADC
     clr     adsel               ; Clear the ADC channel selector variable
     clrh                        ; Clear index register Hi byte
     lda     clt                 ; Load accumulator with value in Coolant Temperature ADC
     tax                         ; Copy to index register Lo byte
     lda     thermfactor,x       ; Load accumulator with value in thermfactor table, offset in
                                 ; index register Lo byte
     sta     coolant             ; Copy to Coolant Temp in degreesF+40


;***********************************************************************************************
; - Primer Pulse Width is directly set by the coolant temperature value of
;   "pru" (at -40 degrees) and "prh" (at 165 degrees) - value is interpolated
;   Load the Final Pulse Width variable with the Prime Pulse value * 100 and
;   schedule the injectors to be fired on the first 100uS clock tick.
;
;   Note! In order for the Prime Pulse to be effective, the fuel system must
;   be at normal operating pressure. If there is insufficient residual
;   pressure, it may be necessary to turn the ignition switch off after the
;   fuel pump stops(1 second with no IRQ signal), then switch on and start.
;***********************************************************************************************

INTERP_PRIME_PW:
     lda     #$00             ; Load accumulator with decimal 0
     sta     tmp1             ; Copy to tmp1 variable
     lda     #$CD             ; Load accumulator with decimal 205
                              ; 165 + 40 degrees (offset in lookup table)
     sta     tmp2             ; Copy to tmp2 variable
     lda     pru              ; Load accumulator with value in Primer Pulse
                              ; Width at -40 F)
     sta     tmp3             ; Copy to "tmp3"
     lda     prh              ; Load accumulator with value in Primer Pulse
                              ; Width at 165 F)
     sta     tmp4             ; Copy to "tmp4"
     mov     coolant,tmp5     ; Move value at current coolant temp to tmp5
     jsr     lininterp        ; Jump to Lininterp: (interpolation subroutine)
     lda     tmp6             ; Load accumulator with value in tmp6 variable
                              ; (result of calculation)
     sta     pw               ; Copy to "pw"
     sta     fd               ; Copy to "fd"
     tax                      ; Tansfer value in accumulator to index register Lo byte
     lda     #$64             ; Load accumulator with decimal 100
     mul                      ; Multiply X:A<-(X)x(A)
     stx     pwcalch          ; Copy result Hi byte to "pwcalch"
     sta     pwcalcl          ; Copy result Lo byte to "pwcalcl"
     bset    fuelp,porta      ; Set "fuelp" bit of Port A (energise fuel pump relay)
     bset    FPon,portAbits   ; Set "FPon" bit of "portAbits"
     bset    sched1,squirt    ; Set "sched1" bit of "squirt"
     bset    inj1,squirt      ; Set "inj1" bit of "squirt"
     bset    sched2,squirt    ; Set "sched2" bit of "squirt"
     bset    inj2,squirt      ; Set "inj2" bit of "squirt"
     bset    running,engine   ; Set "running" bit of "engine"


;***********************************************************************************************
; - Enable interrupts
;***********************************************************************************************

     cli                      ; Clear interrupt mask

;***********************************************************************************************
;***********************************************************************************************
;***************************    M A I N  E V E N T  L O O P     ********************************
;***********************************************************************************************
;***********************************************************************************************

LOOPER:


;*;***********************************************************************************************
;*; - Toggle "Loopchk" bit on Port C each program loop so frequency can
;*;   be checked with a scope or frequency meter.(for program developement)
;*;***********************************************************************************************
;*
;*     com     LoopCounter      ; Ones compliment "LoopCounter"(flip state of "LoopCounter")
;*     bne     SET_LOOPCHK      ; If the Z bit of CCR is clear, branch to SET_LOOPCHK:
;*     bclr    Loopchk,PORTC    ; Clear "Loopchk" bit of Port C (LED off)
;*     bra     LOOPCHK_DONE     ; Branch to LOOPCHK_DONE:
;*
;*SET_LOOPCHK:
;*     bset    Loopchk,PORTC    ; Set "Loopchk" bit of Port C (LED on)
;*
;*LOOPCHK_DONE:

;***********************************************************************************************
; - Check the state of the Fuel Trim and Ignition Trim enable switches and act accordingly
;***********************************************************************************************

;CHK_TRM:
     brclr   Ign_Trm_En,porta,ITRM_ON    ; If "Ign_Trm_En" (PTA2) is low, branch to ITRM_ON:
     mov     #$1C,Trm_Ang_Fac            ; Move decimal 28 into Trim Angle Factor
                                         ; 90/256=.352 counts per degree, 28 counts is mid
                                         ; point(10 degrees, no correction 8 cyl)
     bclr    ITen,portAbits              ; Clear "ITen" bit of "portAbits"
     bra     CHK_FTRM                    ; Branch to CHK_FTRM:

ITRM_ON:
     bset    ITen,portAbits              ; Set "ITen" bit of "portAbits"

;***********************************************************************************************
; - Linear Interpolation of ADC value:
;   tmp1 = 0
;   tmp2 = 255
;   tmp3 = High end of scale
;   tmp4 = Lo end of scale
;   tmp5 = ADC value
;   tmp6 = Result
;***********************************************************************************************

     clr     tmp1              ; Clear "tmp1"(0 volts)
     mov     #$FF,tmp2         ; Move decimal 255 to "tmp2"(5 volts)
     mov     #$39,tmp3         ; Move decimal 57 to "tmp3" (20 degrees 8 cyl)
     clr     tmp4              ; Clear "tmp4"(0 degrees)
     mov     Itrm_ADC,tmp5     ; Move value in "Itrm_ADC" to "tmp5"(ADC value)
     jsr     lininterp         ; Jump to subroutine at linenterp:
     lda     tmp6              ; Load accumulator with value in "tmp6"(Result)
     sta     Trm_Ang_Fac       ; Copy to Trim Angle Factor variable

CHK_FTRM:
     brclr   Fuel_Trm_En,porta,FTRM_ON   ; If "Fuel_Trm_En" (PTA5) is low, branch to FTRM_ON:
     mov     #$64,Ftrimcor               ; Move decimal 100 into "Ftrimcor" (no fuel trim)
     bclr    FTen,portAbits              ; Clear "FTen" bit of "portAbits"
     bra     CHK_TRM_DONE

FTRM_ON:
     bset    FTen,portAbits       ; Set "FTen" bit of "portAbits"

;***********************************************************************************************
; - Linear Interpolation of ADC value:
;   tmp1 = 0
;   tmp2 = 255
;   tmp3 = High end of scale
;   tmp4 = Lo end of scale
;   tmp5 = ADC value
;   tmp6 = Result
;***********************************************************************************************

     clr     tmp1              ; Clear "tmp1"(0 volts)
     mov     #$FF,tmp2         ; Move decimal 255 to "tmp2"(5 volts)
     mov     #$73,tmp3         ; Move decimal 115 to "tmp3" (115%)
     mov     #$55,tmp4         ; Move decimal 85 to "tmp4"(85%)
     mov     Ftrm_ADC,tmp5     ; Move value in "Ftrm_ADC" to "tmp5"(ADC value)
     jsr     lininterp         ; Jump to subroutine at linenterp:
     lda     tmp6              ; Load accumulator with value in "tmp6"(Result)
     sta     Ftrimcor          ; Copy to Fuel Trim Correction Factor

CHK_TRM_DONE:

;***********************************************************************************************
;
; - KnockSenseMS output "TIM” is normally high (+5 volts), and goes low (0 volts) when it
;   detects a knock. “TIM” is connected to MS DB37 pin 4 which is traced on the MS board to
;   JP1-2 (X9). X9 is a header socket to the daughter board header pin X9 which connects to R4
;   and C5 to header pin X4 to header socket X4 on the MS board. X4 is traced to HC908 pin 36
;   (PTA3) which was a spare input. PTA3 is polled every time through the main loop. If it is
;   high “knock” (bit 7) of “alarmbits” is cleared. If it is low, the bit is set. The code
;   poles “alarmbits” to see if any of the bits are set. If all are clear, no alarm is sounded.
;   If any are set, the alarm is sounded and TS or MV displays which bit, or bits are causing
;   the alarm.
;
;***********************************************************************************************

;**********************************************************************************************
; - Check the state of the Knock Sensor Input and act accordingly
;**********************************************************************************************

;CHK_KNOCK:
     brclr   Knk_Det,porta,KNOCK_ON        ; If "Knk" (PTA3) is low, branch to KNOCK_ON:
     brclr   knocking,alarmbits,KNOCK_DONE ; If "knocking" bit of "alarmbits" is already clear,
                                           ; branch to KNOCK_DONE:
     bclr    knocking,alarmbits            ; Clear "knocking" bit of "alarmbits"
     bra     KNOCK_DONE                    ; Branch to KNOCK_DONE:

KNOCK_ON:
     brset   knocking,alarmbits,KNOCK_DONE ; If "knocking" bit of "alarmbits" is already set,
                                           ; branch to KNOCK_DONE:
     bset    knocking,alarmbits            ; Set "knocking" bit of "alarmbits"

KNOCK_DONE:

;*;***********************************************************************************************
;*; - Set the "trigHi" bit of "engine" variable when the IRQ signal goes low (PIP Hi) and clear
;*;   it when the IRQ signal goes Hi (PIP Lo). This is used to test the crank trigger signal for
;*;   initial engine set up only.
;*;***********************************************************************************************

;*     bil     SET_TRIG       ; If IRQ pin is low (PIP Hi), branch to SET_TRIG:
;*     bclr    trigHi,engine  ; Clear "trigHi" bit of "engine" variable
;*     bra     TRIG_DONE      ; Branch to TRIG_DONE:

;*SET_TRIG:
;*     bset    trigHi,engine  ; Set "trigHi" bit of "engine" variable

;*TRIG_DONE:

;***********************************************************************************************
; - On start up, until we have received a minimum of 3 PIP signals so
;   that 0.1uS period calculations will be valid, prohibit "Run" mode.
;***********************************************************************************************

     lda     MinPIP       ; Load accumulator with value in "MinPIP" variable
     beq     MINPIP_DONE  ; If Z bit of CCR is set, branch to MINPIP_DONE:
     bclr    run,engine2  ; Clear "run" bit of "engine2" variable

MINPIP_DONE:

;***********************************************************************************************
; - Check to see if Ignition Monitor on PTA4 is high so it can be re-enabled for
;   keyboard interrupts
;***********************************************************************************************

     brclr   MONen,inputs,CHK_MON_HI    ; If "MONen" bit of "inputs" var is clear,
                                        ; branch to CHK_MON_HI:
     bra     EN_KYBD_DONE               ; Branch to EN_KYBD_DONE:

CHK_MON_HI:
     bclr     Ign_Mon,DDRA               ; Clear "Ign_Mon" bit of Port Data Direction Register
                                         ;(Make sure PTA4 is configured as an input
                                         ; so it can be read)
     brclr   Ign_Mon,porta,EN_KYBD_DONE  ; If "Ign_Mon" bit of Port A is clear, branch to
                                         ;EN_KYBD_DONE:(pin is Lo, continue loop)

;***********************************************************************************************
; ------------------- Re enable Ignition Monitor KBI ------------------------
; To avoid false interrupts, configure pin as output, set, logic Hi, enable
; KBI, and restore pin to input.
;***********************************************************************************************

     bset     Ign_Mon,DDRA       ; Set "Ign_Mon" bit of Port A Data Direction Register
                                 ; (pin is output)
     bset     Ign_Mon,porta      ; Set "Ign_Mon" pin of Port A (pin is Hi)
     bset     Ign_Mon,INTKBIER   ; Set "Ign_Mon" bit of Keyboard Interrupt
                                 ; Enable Register (enable pin for interrupt)
     bclr     Ign_Mon,DDRA       ; Clear "Ign_Mon" bit of Port A Data Direction Register
                                 ; (pin is input)
     bset     MONen,inputs       ; Set "MONen" bit of "inputs" variable

EN_KYBD_DONE:

;***********************************************************************************************
; - At cranking and low engine speeds, PIP "mirrors" SPOUT. If we are not in "Run" mode,
;   control SPOUT here.
;***********************************************************************************************

;SLOW_SPOUT:
     brset   run,engine2,SLOW_SPOUT_DONE  ; If "run" bit of "engine2 variable is set,
                                          ; branch to SLOW_SPOUT_DONE:
     bil     SET_SPOUT_HI                 ; If IRQ pin is low (PIP Hi), branch to SET_SPOUT_HI
     bset    spark,PORTA                  ; Set "spark" bit of Port A(SPOUT falling edge)
     bra     SLOW_SPOUT_DONE              ; Branch to SLOW_SPOUT_DONE:

SET_SPOUT_HI:
     bclr    spark,PORTA                  ; Clear "spark" bit of Port A (SPOUT rising edge)

SLOW_SPOUT_DONE:

;***********************************************************************************************
; - If we are in "Run" mode, and if a spark is not in progress, and if a
;   spark has been scheduled, set and arm TIM1 CH0 with new output compare
;   value to drive SPOUT Hi and fire coil.
;***********************************************************************************************

     brclr   run,engine2,DELAY_DONE       ; If "run" bit of "engine2 variable is clear,
                                          ; branch to DELAY_DONE:
     brset   sparking,engine2,DELAY_DONE  ; If "sparking" bit of "engine2" variable is set,
                                          ; branch to DELAY_DONE:
     brclr   schedspk,engine2,DELAY_DONE  ; If "schedspk" bit of "engine2" variable is clear,
                                          ; branch to DELAY_DONE:


;***********************************************************************************************
; - Set and arm TIM2 Chan 1 output compare value to fire coil
;   CH1_ocH:CH1_ocL = T2CH1H:T2CH1L
;***********************************************************************************************

;SPARK_DELAY:
     clrh                      ; Clear index register Hi byte
     ldhx    CH1_onH           ; Load index register with value in CH1 on O/C value H:L
     sthx    T2CH1H            ; Copy to TIM2 CH1 register H:L
                               ;(output compare value for SPOUT rising edge)
     bset    CHxIE,T2SC1       ; Enable interrupt requests TIM2 channel 1
                               ; Status and Control Register
     bclr    schedspk,engine2  ; Clear "schedspk" bit of "engine2" variable

DELAY_DONE:


;***********************************************************************************************
; ----------------------------------------- ADC Section ----------------------------------------
;
; - All tables are pre-computed for all 256 different values
;   and stored in FLASH.
; - Manifold Air and Coolant temperatures are in degrees F plus 40 ("airtemp"
;   and "coolant"). This allows unsigned numbers for full temperature range
; - Manifold Air pressures is in Kilopascals ("kpa")
;   Air density correction factors is in percent,
;   with 100 being no correction ("barocor" and "aircor")
;   "Ftrimcor" is the fuel correction factor from -15% to +15% (85 to 115)
;   Centre position of the pot is 0 correction (100%).
;   "Itrimcor" is the ignition trim correction factor, 10 degrees advance
;   or retard. 0=10 degrees retard, 10=no trim, 20=10 degrees advance.
;***********************************************************************************************

;***********************************************************************************************
; - Update the ADC readings and conversions. This is done only once per ADC
;   conversion complete interrupt, in the first pass through the main loop
;   after the interrupt routine has been completed.
;***********************************************************************************************

     brset   adcc,inputs,ADC_LOOKUPS  ; If "adcc" bit of "inputs" variable is set, branch to
                                      ; ADC_LOOKUPS:
     jmp     NO_ADC_PASS              ; Jump to NO_ADC_PASS:

ADC_LOOKUPS:

;***********************************************************************************************
; - First determine which is the current ADC reading and branch accordingly
;***********************************************************************************************

     lda     ADCcnt        ; Load accumulator with value in "ADCcnt"
     cbeqa   #$00,ADC_0    ; Compare with decimal 0 and if equal, branch to ADC_0:(map)
     cbeqa   #$01,ADC_1    ; Compare with decimal 1 and if equal, branch to ADC_1:(mat)
     cbeqa   #$02,ADC_2    ; Compare with decimal 2 and if equal, branch to ADC_2:(clt)
     cbeqa   #$03,ADC_3    ; Compare with decimal 3 and if equal, branch to ADC_3:(tps)
     cbeqa   #$04,ADC_4    ; Compare with decimal 4 and if equal, branch to ADC_4:(batt)
     cbeqa   #$05,ADC_5_J  ; Compare with decimal 5 and if equal, branch to ADC_5_J:(ego)
     cbeqa   #$06,ADC_6_J  ; Compare with decimal 6 and if equal, branch to ADC_6_J:(Ftrm_ADC)
     cbeqa   #$07,ADC_7_J  ; Compare with decimal 7 and if equal, branch to ADC_7_J:(ITrm_ADC)
     cbeqa   #$08,ADC_8_J  ; Compare with decimal 8 and if equal, branch to ADC_8_J:(OP_ADC)
     cbeqa   #$09,ADC_9_J  ; Compare with decimal 9 and if equal, branch to ADC_9_J:(FP_ADC)
     cbeqa   #$A,ADC_10_J  ; Compare with decimal 10 and if equal, branch to ADC_10_J:(EGT_ADC)

;***********************************************************************************************
; - "map" section
;***********************************************************************************************

ADC_0:
     lda     mapcur               ; Load accumulator with value in "mapcur"
     sta     map                  ; Copy to "map"
     clrh                         ; Clear index register Hi byte
     lda     map                  ; Load accumulator with value in MAP ADC
     tax                          ; Copy to index register Lo byte
     lda     KPAFACTOR4250rjh,x   ; Load accumulator with value in KPAFACTOR4250rjh table
                                  ; offset in index register Lo byte
     sta     kpa                  ; Copy to Manifold Air Pressure in Kilopascals
     jmp     ADC_DONE             ; Jump to ADC_DONE:

;***********************************************************************************************
; - "mat" section
;***********************************************************************************************

ADC_1:
     lda     matcur          ; Load accumulator with value in "matcur"
     sta     mat             ; Copy to "mat"
     clrh                    ; Clear index register Hi byte
     lda     mat             ; Load accumulator with value in Manifold Air Temperatue ADC
     tax                     ; Copy to index register Lo byte
     lda     airdenfactor,x  ; Load accumulator with value in airdenfactor
                             ; table, offset in index register Lo byte
     sta     aircor          ; Copy to Air Density Correction
     lda     mat             ; Load accumulator with value in Manifold Air Temperatue ADC
     tax                     ; Copy to index register Lo byte
     lda     thermfactor,x   ; Load accumulator with value in thermfactor table,
                             ; offset in index register Lo byte
     sta     airtemp         ; Copy to Manifold Air Temperature in degrees F + 40
     jmp     ADC_DONE        ; Jump to ADC_DONE:

;***********************************************************************************************
; - "clt" section
;***********************************************************************************************

ADC_2:
     lda     cltcur          ; Load accumulator with value in "cltcur"
     sta     clt             ; Copy to "clt"
     clrh                    ; Clear index register Hi byte
     lda     clt             ; Load accumulator with value in Coolant Temperature ADC
     tax                     ; Copy to index register Lo byte
     lda     thermfactor,x   ; Load accumulator with value in thermfactor table,
                             ; offset in index register Lo byte
     sta     coolant         ; Copy to Coolant Temp in degreesF+40
     jmp     ADC_DONE        ; Jump to ADC_DONE:

;***********************************************************************************************
; - "tps" section
;***********************************************************************************************

ADC_3:
     lda     tpscur          ; Load accumulator with value in "tpscur"
     sta     tps             ; Copy to "tps"

;***********************************************************************************************
; - Linear Interpolation of ADC value:
;   tmp1 = Lo ADC(CT_cnt)
;   tmp2 = Hi ADC(WOT_cnt)
;   tmp3 = Lo end of scale(0%)
;   tmp4 = Hi end of scale(100%)
;   tmp5 = ADC value
;   tmp6 = Result
;***********************************************************************************************

     lda     CT_cnt            ; Load accumulator with value in "CT_cnt"
     sta     tmp1              ; Copy to "tmp1"
     lda     WOT_cnt           ; Load accumulator with value in "WOT_cnt"
     sta     tmp2              ; Copy to "tmp2"
     mov     #$00,tmp3         ; Move zero to "tmp3"(0%)
     mov     #$64,tmp4         ; Move decimal 100 to "tmp4"(100%)
     mov     tps,tmp5          ; Move value in "tps" to "tmp5"(ADC value)
     jsr     lininterp         ; Jump to subroutine at linenterp:
     lda     tmp6              ; Load accumulator with value in "tmp6"(Result)
     sta     tpsp              ; Copy to "tpsp"(Throttle percent opening)
     jmp     ADC_DONE          ; Jump to ADC_DONE:

;***********************************************************************************************
; - Long branches
;***********************************************************************************************

ADC_5_J:
     bra     ADC_5

ADC_6_J:
     bra     ADC_6

ADC_7_J:
     bra     ADC_7

ADC_8_J:
     bra     ADC_8

ADC_9_J:
     bra     ADC_9_J1

ADC_10_J:
     bra     ADC_10_J1

;***********************************************************************************************
; - "batt" Section
;***********************************************************************************************

ADC_4:
     lda     battcur      ; Load accumulator with value in "battcur"
     add     battTL       ; Add without Carry A<-(A)+(M) "ADR" + "battTL" (total Lo Byte)
     sta     battTL       ; Copy to "battTL" (update total Lo Byte))
     lda     battTH       ; Load accumulator with value in "battTH" (total Hi Byte)
     adc     #$0          ; Add with carry decimal zero A<-(A)+(M)(just the carry)
     sta     battTH       ; Copy to "battT" (update total Hi Byte))
     inc     battcnt      ; Increment "battcnt" variable

;***********************************************************************************************
; - Check the value of "battcnt" to see if it's time to average the values
;***********************************************************************************************

     lda     battcnt      ; Load accumulator with value in "battcnt"
     cmp     #$10         ; Compare value in "battcnt" to decimal 16
     beq     BATT_AV      ; If Z bit of CCR is set (operands equal) branch to BATT_AV:
     bra     NO_BATT_AV   ; Branch to NO_BATT_AV:

BATT_AV:
     lda     battTL       ; Lod accumulator with value in "battTL"
     sta     tmp1         ; Copy to "tmp1"
     lda     battTH       ; Load accumulator with value in "battTH"
     sta     tmp2         ; Copy to "tmp2"
     jsr     DIV_BY_16    ; Jump to subroutine at DIV_BY_16:
     lda     tmp1         ; Load accumulator with value in "tmp1"
     sta     batt         ; Copy to "batt"
     clr     battTH       ; Clear "battTH"
     clr     battTL       ; Clear "battTL"
     clr     battcnt      ; Clear "battcnt"

NO_BATT_AV:
     jmp     ADC_DONE     ; Jump to ADC_DONE:

;***********************************************************************************************
; - "ego" Section
;***********************************************************************************************

ADC_5:
     lda     egocur       ; Load accumulator with value in "egocur"
     add     egoTL        ; Add without Carry A<-(A)+(M) "ADR" + "egoTL" (total Lo Byte)
     sta     egoTL        ; Copy to "egoTL" (update total Lo Byte))
     lda     egoTH        ; Load accumulator with value in "egoTH" (total Hi Byte)
     adc     #$0          ; Add with carry decimal zero A<-(A)+(M)(just the carry)
     sta     egoTH        ; Copy to "egoTH" (update total Hi Byte))
     inc     egocnt       ; Increment "battcnt" variable

;***********************************************************************************************
; - Check the value of "egocnt" to see if it's time to average the values
;***********************************************************************************************

     lda     egocnt       ; Load accumulator with value in "egocnt"
     cmp     #$10         ; Compare value in "battcnt" to decimal 16
     beq     EGO_AV      ; If Z bit of CCR is set (operands equal) branch to EGO_AV:
     bra     NO_EGO_AV   ; Branch to NO_EGO_AV:

EGO_AV:
     lda     egoTL        ; Lod accumulator with value in "egoTL"
     sta     tmp1         ; Copy to "tmp1"
     lda     egoTH        ; Load accumulator with value in "egoTH"
     sta     tmp2         ; Copy to "tmp2"
     jsr     DIV_BY_16    ; Jump to subroutine at DIV_BY_16:
     lda     tmp1         ; Load accumulator with value in "tmp1"
     sta     ego          ; Copy to "ego"
     clr     egoTH        ; Clear "egoTH"
     clr     egoTL        ; Clear "egoTL"
     clr     egocnt       ; Clear "egocnt"

NO_EGO_AV:
     jmp     ADC_DONE     ; Jump to ADC_DONE:

;***********************************************************************************************
; - "Ftrm_ADC" section
;***********************************************************************************************

ADC_6:
     lda     Ftrm_ADCcur     ; Load accumulator with value in "Ftrm_ADCcur"
     sta     Ftrm_ADC        ; Copy to "Ftrm_ADC"
     jmp     ADC_DONE        ; Jump to ADC_DONE:

;***********************************************************************************************
; - "Itrm_ADC" section
;***********************************************************************************************

ADC_7:
     lda     Itrm_ADCcur     ; Load accumulator with value in "Itrm_ADCcur"
     sta     Itrm_ADC        ; Copy to "Itrm_ADC"
     jmp     ADC_DONE        ; Jump to ADC_DONE:

;***********************************************************************************************
; - Long branches
;***********************************************************************************************

ADC_9_J1:
     bra     ADC_9

ADC_10_J1:
     bra     ADC_10

;***********************************************************************************************
; - "OP_ADC" Section
;***********************************************************************************************

ADC_8:
     lda     OP_ADCcur      ; Load accumulator with value in "OP_ADCcur"
     add     OP_ADCTL       ; Add without Carry A<-(A)+(M) "ADR" + "OP_ADCTL" (total Lo Byte)
     sta     OP_ADCTL       ; Copy to "OP_ADCTL" (update total Lo Byte))
     lda     OP_ADCTH       ; Load accumulator with value in "OP_ADCTH" (total Hi Byte)
     adc     #$0            ; Add with carry decimal zero A<-(A)+(M)(just the carry)
     sta     OP_ADCTH       ; Copy to "OP_ADCT" (update total Hi Byte))
     inc     OP_ADCcnt      ; Increment "OP_ADCcnt" variable

;***********************************************************************************************
; - Check the value of "OP_ADCcnt" to see if it's time to average the values
;***********************************************************************************************

     lda     OP_ADCcnt      ; Load accumulator with value in "OP_ADCcnt"
     cmp     #$10           ; Compare value in "battcnt" to decimal 16
     beq     OP_ADC_AV;     ; If Z bit of CCR is set (operands equal) branch to OP_ADC_AV:
     bra     NO_OP_ADC_AV   ; Branch to NO_OP_ADC_AV:

OP_ADC_AV:
     lda     OP_ADCTL       ; Lod accumulator with value in "OP_ADCTL"
     sta     tmp1           ; Copy to "tmp1"
     lda     OP_ADCTH       ; Load accumulator with value in "OP_ADCTH"
     sta     tmp2           ; Copy to "tmp2"
     jsr     DIV_BY_16      ; Jump to subroutine at DIV_BY_16:
     lda     tmp1           ; Load accumulator with value in "tmp1"
     sta     OP_ADC         ; Copy to "OP_ADC"
     clr     OP_ADCTH       ; Clear "OP_ADCTH"
     clr     OP_ADCTL       ; Clear "OP_ADCTL"
     clr     OP_ADCcnt      ; Clear "OP_ADCcnt"

NO_OP_ADC_AV:
     jmp     ADC_DONE       ; Jump to ADC_DONE:

;***********************************************************************************************
; - "FP_ADC" Section
;***********************************************************************************************

ADC_9:
     lda     FP_ADCcur      ; Load accumulator with value in "FP_ADCcur"
     add     FP_ADCTL       ; Add without Carry A<-(A)+(M) "ADR" + "FP_ADCTL" (total Lo Byte)
     sta     FP_ADCTL       ; Copy to "FP_ADCTL" (update total Lo Byte))
     lda     FP_ADCTH       ; Load accumulator with value in "FP_ADCTH" (total Hi Byte)
     adc     #$0            ; Add with carry decimal zero A<-(A)+(M)(just the carry)
     sta     FP_ADCTH       ; Copy to "FP_ADCT" (update total Hi Byte))
     inc     FP_ADCcnt      ; Increment "FP_ADCcnt" variable

;***********************************************************************************************
; - Check the value of "FP_ADCcnt" to see if it's time to average the values
;***********************************************************************************************

     lda     FP_ADCcnt      ; Load accumulator with value in "FP_ADCcnt"
     cmp     #$10           ; Compare value in "battcnt" to decimal 16
     beq     FP_ADC_AV;     ; If Z bit of CCR is set (operands equal) branch to FP_ADC_AV:
     bra     NO_FP_ADC_AV   ; Branch to NO_FP_ADC_AV:

FP_ADC_AV:
     lda     FP_ADCTL       ; Lod accumulator with value in "FP_ADCTL"
     sta     tmp1           ; Copy to "tmp1"
     lda     FP_ADCTH       ; Load accumulator with value in "FP_ADCTH"
     sta     tmp2           ; Copy to "tmp2"
     jsr     DIV_BY_16      ; Jump to subroutine at DIV_BY_16:
     lda     tmp1           ; Load accumulator with value in "tmp1"
     sta     FP_ADC         ; Copy to "FP_ADC"
     clr     FP_ADCTH       ; Clear "FP_ADCTH"
     clr     FP_ADCTL       ; Clear "FP_ADCTL"
     clr     FP_ADCcnt      ; Clear "FP_ADCcnt"

NO_FP_ADC_AV:
     jmp     ADC_DONE       ; Jump to ADC_DONE:

;***********************************************************************************************
; - "EGT_ADC" Section
;***********************************************************************************************

ADC_10:
     lda     EGT_ADCcur      ; Load accumulator with value in "EGT_ADCcur"
     add     EGT_ADCTL       ; Add without Carry A<-(A)+(M) "ADR" + "EGT_ADCTL" (total Lo Byte)
     sta     EGT_ADCTL       ; Copy to "EGT_ADCTL" (update total Lo Byte))
     lda     EGT_ADCTH       ; Load accumulator with value in "EGT_ADCTH" (total Hi Byte)
     adc     #$0            ; Add with carry decimal zero A<-(A)+(M)(just the carry)
     sta     EGT_ADCTH       ; Copy to "EGT_ADCT" (update total Hi Byte))
     inc     EGT_ADCcnt      ; Increment "EGT_ADCcnt" variable

;***********************************************************************************************
; - Check the value of "EGT_ADCcnt" to see if it's time to average the values
;***********************************************************************************************

     lda     EGT_ADCcnt     ; Load accumulator with value in "EGT_ADCcnt"
     cmp     #$10           ; Compare value in "battcnt" to decimal 16
     beq     EGT_ADC_AV;    ; If Z bit of CCR is set (operands equal) branch to EGT_ADC_AV:
     bra     NO_EGT_ADC_AV  ; Branch to NO_EGT_ADC_AV:

EGT_ADC_AV:
     lda     EGT_ADCTL      ; Lod accumulator with value in "EGT_ADCTL"
     sta     tmp1           ; Copy to "tmp1"
     lda     EGT_ADCTH      ; Load accumulator with value in "EGT_ADCTH"
     sta     tmp2           ; Copy to "tmp2"
     jsr     DIV_BY_16      ; Jump to subroutine at DIV_BY_16:
     lda     tmp1           ; Load accumulator with value in "tmp1"
     sta     EGT_ADC        ; Copy to "EGT_ADC"
     clr     EGT_ADCTH      ; Clear "EGT_ADCTH"
     clr     EGT_ADCTL      ; Clear "EGT_ADCTL"
     clr     EGT_ADCcnt     ; Clear "EGT_ADCcnt"

NO_EGT_ADC_AV:

ADC_DONE:
     bclr    adcc,inputs                    ; Clear "adcc" bit of "inputs" variable

NO_ADC_PASS:

;***********************************************************************************************
; - Alarm section
;***********************************************************************************************

;HET_ALARM:
     lda     coolant                      ; Load accumulator with value in "coolant"
     cmp     hetoff                       ; Compare with "hetoff" (235=195 degrees F)
     bls     CLEAR_HET                    ; If A lower or same as M, branch to CLEAR_HET:
     bra     CHK_HET_HI                   ; Branch to CHK_HET_HI:

CLEAR_HET:
     brclr   HET,alarmbits,HET_ALARM_DONE ; If "HET" bit of "alarmbits" is clear,
                                          ; branch to HET_ALARM_DONE:
     bclr    HET,alarmbits                ; Clear "HET" bit of "alarmbits"
     bra     HET_ALARM_DONE               ; Branch to HET_ALARM_DONE:

CHK_HET_HI:
     cmp     heton                        ; Compare with "heton" (240=200 degrees F)
     bhs     SET_HET                      ; If A greater or same as M, branch to SET_HET:
     bra     HET_ALARM_DONE               ; Branch to HET_ALARM_DONE:

SET_HET:
     brset   HET,alarmbits,HET_ALARM_DONE ; If "HET" bit of "alarmbits" is set, branch to
                                          ; HET_ALARM_DONE:
     bset    HET,alarmbits                ; Set "HET" bit of "alarmbits"

HET_ALARM_DONE:

;LOP_ALARM:
     lda     OP_ADC                       ; Load accumulator with value in "OP_ADC"
     cmp     lopon                        ; Compare with "lopon" (92=20 PSI)
     bls     SET_LOP                      ; If A lower or same as M, branch to SET_LOP:
     bra     CHK_LOP_HI                   ; Branch to CHK_LOP_HI:

SET_LOP:
     brset   LOP,alarmbits,LOP_ALARM_DONE ; If "LOP" bit of "alarmbits" is set, branch to
                                          ; LOP_ALARM_DONE:
     bset    LOP,alarmbits                ; Set "LOP" bit of "alarmbits"
     bra     LOP_ALARM_DONE               ; Branch to LOP_ALARM_DONE:

CHK_LOP_HI:
     cmp     lopoff                       ; Compare with "lopoff" (102=25 PSI)
     bhs     CLEAR_LOP                    ; If A greater or same as M, branch to CLEAR_LOP:
     bra     LOP_ALARM_DONE               ; Branch to LOP_ALARM_DONE:

CLEAR_LOP:
     brclr   LOP,alarmbits,LOP_ALARM_DONE ; If "LOP" bit of "alarmbits" is clear, branch to
                                          ; LOP_ALARM_DONE:
     bclr    LOP,alarmbits                ; Clear "LOP" bit of "alarmbits"

LOP_ALARM_DONE:

;HEGT_ALARM:
     lda     EGT_ADC                        ; Load accumulator with value in "EGT_ADC"
     cmp     hegtoff                        ; Compare with "hegtoff" (181=950 degrees F)
     bls     CLEAR_HEGT                     ; If A lower or same as M, branch to CLEAR_HEGT:
     bra     CHK_HEGT_HI                    ; Branch to CHK_HEGT_HI:

CLEAR_HEGT:
     brclr   HEGT,alarmbits,HEGT_ALARM_DONE ; If "HEGT" bit of "alarmbits" is clear,
                                            ; branch to HEGT_ALARM_DONE:
     bclr    HEGT,alarmbits                 ; Clear "HEGT" bit of "alarmbits"
     bra     HEGT_ALARM_DONE                ; Branch to HEGT_ALARM_DONE:

CHK_HEGT_HI:
     cmp     hegton                         ; Compare with "hegton" (191=1000 degrees F)
     bhs     SET_HEGT                       ; If A greater or same as M, branch to SET_HEGT:
     bra     HEGT_ALARM_DONE                ; Branch to HEGT_ALARM_DONE:

SET_HEGT:
     brset   HEGT,alarmbits,HEGT_ALARM_DONE ; If "HET" bit of "alarmbits" is set, branch to
                                            ; HEGT_ALARM_DONE:
     bset    HEGT,alarmbits                 ; Set "HEGT" bit of "alarmbits"

HEGT_ALARM_DONE:

;FP_ALARM
     lda     FP_ADC                       ; Load accumulator with value in "EGT_ADC"
     cmp     lfpon                        ; Compare with "lfpon" (102=25 PSI)
     bls     SET_LFP                      ; If A lower or same as M, branch to SET_LFP:
     bra     CHK_LFP_HI                   ; Branch to CHK_LFP_HI:

SET_LFP
     brset   LFP,alarmbits,LFP_ALARM_DONE ; If "LFP: bit of "alarmbits" is set, branch to
                                          ; LFP_ALARM_DONE:
     bset    LFP,alarmbits                ; Set "LFP" bit of "alarmbits"
     bra     LFP_ALARM_DONE               ; Branch to LFP_ALARM_DONE:

CHK_LFP_HI:
     cmp     lfpoff                       ; Compare with "lfpoff" (112=30 PSI)
     bhs     CLEAR_LFP                    ; If A higher or same as M, branch to CLEAR_LFP:
     bra     LFP_ALARM_DONE               ; Branch to LFP_ALARM_DONE:

CLEAR_LFP:
     brclr   LFP,alarmbits,LFP_ALARM_DONE ; If "LFP" bit of "alarmbits" is clear, branch to
                                          ; LFP_ALARM_DONE:
     bclr    LFP,alarmbits                ; Clear "LFP" bit of "alarmbits"

LFP_ALARM_DONE:

     cmp     hfpoff                       ; Compare with "hfpoff" (143=45 PSI)
     bls     CLEAR_HFP                    ; If A lower or same as M, branch to CLEAR_HFP:
     bra     CHK_HFP_HI                   ; Branch to CHK_HFP_HI:

CLEAR_HFP
     brclr   HFP,alarmbits,HFP_ALARM_DONE ; If "HFP: bit of "alarmbits" is clear, branch to
                                          ; HFP_ALARM_DONE:
     bclr    HFP,alarmbits                ; Clear "HFP" bit of "alarmbits"
     bra     HFP_ALARM_DONE               ; Branch to HFP_ALARM_DONE:

CHK_HFP_HI:
     cmp     hfpon                        ; Compare with "hfpon" (153=50 PSI)
     bhs     SET_HFP                      ; If A higher or same as M, branch to SET_HFP:
     bra     HFP_ALARM_DONE               ; Branch to HFP_ALARM_DONE:

SET_HFP:
     brset   HFP,alarmbits,HFP_ALARM_DONE ; If "HFP" bit of "alarmbits" is set, branch to
                                          ; HFP_ALARM_DONE:
     bset    HFP,alarmbits                ; Set "HFP" bit of "alarmbits"

HFP_ALARM_DONE:

;***********************************************************************************************
; Check to see if any of the engine alarm flags are set and energise or de-energise the
; alarm relay as required. If the cause of the alarm is a knock detection only, do not energise
; the alarm relay unless ignition trim or fuel trim has been activated. This is a band aid
; measure to get around knock detector nuisance trips.
;
; LOP      = 00000001 = 1T   = #$01
; HET      = 00000010 = 2T   = #$02
; LFP      = 00000100 = 4T   = #$04
; HFP      = 00001000 = 8T   = #$08
; HEGT     = 00010000 = 16T  = #$10
; REVL     = 00100000 = 32T  = #$20
; fldclr   = 01000000 = 64T  = #$40
; knocking = 10000000 = 128T = #$80
;
;***********************************************************************************************

;ALARM_CHK:
     lda     alarmbits                      ; Load accumulaotr with value in "alarmbits"
     cbeqa   #$00,ALARM_OFF                 ; If equal to "0" branch to ALARM_OFF:
     brset   EAon,portCbits,ALARM_CHK_DONE  ; If "EAon" bit of "portCbits" is already set,
                                            ; branch to ALARM_CHK_DONE:
     cbeqa   #$80,KNOCK_ALARM               ; If equal to decimal 128 branch to KNOCK_ALARM:

ALARM_ON:
     bset    Eng_Alrm,PORTC                 ; Energise alarm relay
     bset    EAon,portCbits                 ; Set "EAon" bit of "portCbits"
     bra     ALARM_CHK_DONE                 ; Branch to ALARM_CHK_DONE:

KNOCK_ALARM:
     brset   ITen,portAbits,ALARM_ON        ; If "ITen" bit of "portAbits" is set, branch
                                            ; to ALARM_ON:
     brset   FTen,portAbits,ALARM_ON        ; If "FTen" bit of "portAbits" is set, branch
                                            ; to ALARM_ON:
     bset    EAon,portCbits                 ; Set "EAon" bit of "portCbits"
     bra     ALARM_CHK_DONE                 ; Branch to ALARM_CHK_DONE:

ALARM_OFF:
     brclr   EAon,portCbits,ALARM_CHK_DONE  ; If "EAon" bit of "portCbits" is already clear,
                                            ; branch to ALARM_CHK_DONE:
     bclr    Eng_Alrm,PORTC                 ; De-Energise alarm relay
     bclr    EAon,portCbits                 ; Clear "EAon" bit of "portCbits"

ALARM_CHK_DONE:

;***********************************************************************************************
; - Spark scheduling, as well as RPM calculations are done only once in the
;   main loop after the PIP rising edge signal has been received, the
;   interrupt routine has been completed, and relevent variables updated.
;***********************************************************************************************

     brset   piprise,inputs,PIP_PASS          ; If "piprise" bit of "inputs" variable is set,
                                              ; branch to PIP_PASS:
     jmp     NO_PIP_PASS                      ; Jump to NO_PIP_PASS:

PIP_PASS:

;***********************************************************************************************
;
; ------------- IGNITION CALCULATION and SCHEDULING SECTION -----------------
;
; To simplify the igntion calculations, the angles are scaled by a factor of
; 256. The PIP Angle Factor is 256, and all other angles are scaled to this.
;
;***********************************************************************************************

;***********************************************************************************************
; - Calculate Period
;   PIP_tsH:PIP_tsL - PIP_tsH_prv:PIP_tsL_prv = PIP_pH:PIP_pL
;***********************************************************************************************

IGNITION_CALCS:
     lda     PIP_tsL        ; Load accumulator with value in PIP timestamp Lo byte
     sub     PIP_tsL_prv    ; Subtract A<-(A)-(M) PIP Timestamp previous Lo byte
     sta     PIP_pL         ; Copy result to PIP period Lo byte variable
     sta     tachcurL       ; Copy to "tachcurL"
     lda     PIP_tsH        ; Load accumulator with value in PIP timestamp Hi byte
     sbc     PIP_tsH_prv    ; Subtract with carry A<-(A)-(M)-(C)
                            ;(PIP Timestamp previous Hi byte)
     sta     PIP_pH         ; Copy result to PIP period Hi byte variable
     sta     tachcurH       ; Copy to "tachcurH"

;***********************************************************************************************
; - 1/02/11 Skip the "predicted period" thing to see if it makes any real world difference
;***********************************************************************************************

     lda     PIP_pL          ; Load accumulator with value in "PIP_pL"
     sta     PIP_pL_pred     ; Copy to "PIP_pL_pred"(for SPOUT on calcs)
     sta     PIP_PeriodL     ; Copy to "PIP_PeriodL"(for SPOUT off calcs)
     lda     PIP_pH          ; Load accumulator with value in "PIP_pH"
     sta     PIP_pH_pred     ; Copy to "PIP_pH_pred"(for SPOUT on calcs)
     sta     PIP_PeriodH     ; Copy to "PIP_PeriodH"(for SPOUT off calcs)


;*;***********************************************************************************************
;*; - Calculate "predicted" period to compensate for acceleration/deceleration
;*;***********************************************************************************************

;*     lda     PIP_pL          ; Load accumulator with value in "PIP_pL"
;*     sub     PIP_pL_prv      ; Subtract A<-(A)-(M)(PIP_pL - PIP_pL_prv = PIP_pL_dif)
;*     sta     PIP_pL_dif      ; Copy result to "PIP_pL_dif" variable
;*     lda     PIP_pH          ; Load accumulator with value in "PIP_pH"
;*     sbc     PIP_pH_prv      ; Subtract with carry A<-(A)-(M)-(C)
                             ;(PIP_pH - PIP_pH_prv = PIP_pH_dif)
;*     sta     PIP_pH_dif      ; Copy result to "PIP_pH_dif" variable
;*     lda     PIP_pL          ; Load accumulator with value in "PIP_pL"
;*     add     PIP_pL_dif      ; Add without Carry A<-(A)+(M)(PIP_pL + PIP_pL_dif = PIP_pL_pred)
;*     sta     PIP_pL_pred     ; Copy to "PIP_pL_pred" variable(for SPOUT on calcs)
;*     sta     PIP_PeriodL     ; Copy to PIP Period Lo byte(for SPOUT off calcs)
;*     lda     PIP_pH          ; Load accumulator with value in "PIP_pH"
;*     adc     PIP_pH_dif      ; Add with Carry A<-(A)+(M)
                             ; (PIP_pH + PIP_pH_dif = PIP_pH_pred)
;*     sta     PIP_pH_pred     ; Copy to "PIP_pH_pred" variable(for SPOUT on calcs)
;*     sta     PIP_PeriodH     ; Copy to PIP Period Hi byte(for SPOUT off calcs)

;***********************************************************************************************
; - Calculate delay time from PIP rising edge to drive SPOUT Hi (on).
;   This is a magical bit of code that Magnus uses and took me the longest
;   time to understand. I originally developed the long math routine version
;   with UMUL32 and UDVD32,,which worked, but got my program loop frequency
;   down to about 768HZ @ ~6450RPM 8cyl, where the ignition starts to falter.
;   This code runs at ~973HZ @ ~6450, and 963HZ @ ~8205RPM 8cyl until falter.
;   (The falter may be due to the duty cycle of my "superstim" IRQ input.)
;   It is simply an 8 bit by 16 bit multiply, which yields a 24 bit result.
;   By ignoring the least significant byte, you effectively divide the 24
;   bit result by 256, rounded down.
;   So, the PIP Period Predicted * Delay Angle Factor / PIP Angle Factor
;   = Delay Period
;
;   PIP_pH_pred:PIP_pL_pred * Dly_Ang_Fac / $01:00 = Delay_pH:Delay_pL.
;
;***********************************************************************************************

;DELAY_CALC:
     lda     Dly_Ang_Fac     ; Load accumulator with value in Delay Angle Factor
     ldx     PIP_pH_pred     ; Load index register Lo byte with value in PIP period predicted Hi
     mul                     ; Multiply (X:A)<-(X)*(A)
     stx     tmp1            ; Copy value in index register Lo byte to tmp1 variable
                             ; (result Hi byte)
     sta     tmp2            ; Copy value in accumulator to tmp2 variable
                             ;(result Lo byte)
     lda     Dly_Ang_Fac     ; Load accumulator with value in Delay
     ldx     PIP_pL_pred     ; Load index register Lo byte with value in PIP period predicted Lo
     mul                     ; Multiply (X:A)<-(X)*(A)
     txa                     ; Transfer value in index register Lo byte to accumulator
                             ;(result Hi byte to accumulator)
     add     tmp2            ; Add (A)<-(A)+(M)(Lo byte result of Hi byte
                             ; mul + Hi byte result of Lo byte mul)
     sta     Delay_pL        ; Copy result to Delay Period Lo byte variable
     lda     tmp1            ; Load accumulator with value in tmp1 variable
                             ;(Hi byte result of Hi byte mul)
     adc     #$0             ; Add with carry decimal 0 (A)<-(A)+(M)+(C)(just add the carry)
     sta     Delay_pH        ; Copy result to Delay Period Hi byte variable


;***********************************************************************************************
; - Calculate the output compare value for delay from PIP to SPOUT
;   PIP_tsH:PIP_tsL + Delay_pH:Delay_pL = CH1_onH:CH1_onL
;***********************************************************************************************

;CALC_OC:
     lda     PIP_tsL            ; Load accumulator with value in "PIP_tsL"
     add     Delay_pL           ; Add without Carry A<-(A)+(M)(PIP_tsL + Delay_pL)
     tax                        ; Transfer result in accumulator to index register Lo byte
     lda     PIP_tsH            ; Load accumulator with value in "PIP_tsH"
     adc     Delay_pH           ; Add with Carry A<-(A)+(M)
     sta     CH1_onH            ; Copy to CH1 on O/C value Hi byte
     stx     CH1_onL            ; Copy to CH1 on O/C value Lo byte
     bset    schedspk,engine2   ; Set "schedspk" bit of "engine2" variable


;***********************************************************************************************
; - Calculate SPOUT on time (50% duty cycle, half PIP predicted period)
;***********************************************************************************************

     ldx     PIP_periodL      ; Load index register Lo byte with value in PIP Period Lo byte
     lda     PIP_periodH      ; Load accumulator with value in PIP Period Hi byte
     lsra                     ; Logical shift right accumulator
     rorx                     ; Rotate right through carry index register Lo byte
                              ; (these 2 steps = A:X / 2)
     sta     SPOUTon_pH       ; Copy to Spout on period Hi byte
     stx     SPOUTon_pL       ; Copy to Spout on period Lo byte


;***********************************************************************************************
; - Save the current timestamp as timestamp previous for next calculations
;***********************************************************************************************

     mov     PIP_tsH,PIP_tsH_prv  ; Copy value in PIP_tsH to PIP_tsH_prv
     mov     PIP_tsL,PIP_tsL_prv  ; Copy value in PIP_tsL to PIP_tsL_prv


;***********************************************************************************************
; - Save the current period as period previous for next calculations
;***********************************************************************************************

     mov     PIP_pH,PIP_pH_prv  ; Copy value in PIP_pH to PIP_pH_prv
     mov     PIP_pL,PIP_pL_prv  ; Copy value in PIP_pL to PIP_pL_prv

;IGN_CALCS_DONE:

;***********************************************************************************************
;
; ---------------------------------- RPM CALCULATION SECTION -----------------------------------
;
; RPM = CONSTANT/PERIOD
; Where:
; RPM         = Engine RPM
; RPM_K = 16 bit constant using .1ms clock tick (10khz)
;               ((10,000tickpsec*60secpmin)/(number of cyl/(stroke/2)))
; RPM_P = 16 bit period count between IRQ pulsed lines in 0.1ms
;               resolution
;   RPM_K
;   ----- = RPM
;   RPM_P
;
; 6cyl 4stroke RPMK = ((10,000*60)/3) = 200,000
; 8cyl 4stroke RPMK = ((10,000*60)/4) = 150,000
;
; We use the 100uS clock tick to calculate RPM/20. This allows us to use an
; 8 bit variable with a range of 0 to 5,100 RPM
; Our formula is now:
;
; rpm = constant/period
; Where:
; rpm         = Engine RPM/20 (0 to 5,100 RPM to fit in 8 bit variable)
; RPMk:RPMk+1 = 16 bit constant using 100uS clock tick (10khz)
;               ((10,000tickpsec*60secpmin)/(number of cyl/(stroke/2)))/20
; RPMpH:RPMpL = 16 bit period count between IRQ pulsed lines in 100uS
;               resolution
;   rpmk:rpmk+1
;   ----------- = rpm
;   RPMpH:RPMpL
;
; 6cyl 4stroke rpmK = ((10,000*60)/3)/20 = 10,000 = $2710
; 8cyl 4stroke rpmK = ((10,000*60)/4)/20 = 7,500  = $1D4C
; 6cyl RPM resolution is ~05@~1000, ~20@~2000, ~76@~3000, and ~128@~5000
; 8cyl RPM resolution is ~06@~1000, ~27@~2000, ~61@~3000, and ~172@~5000
;***********************************************************************************************

;***********************************************************************************************
; - Calculate Engine RPM/20.
;***********************************************************************************************

;RPM_COMP:
     ldhx    RPMph               ; Load index register with value in RPM Period Hy byte
     beq     RPM_CALC_DONE       ; If Z bit of CCR is set, branch to RPM_CALC_DONE:
     pshh                        ; Push value in index register Hi byte to stack
     pula                        ; Pull value in stack to accumulator
     tsta                        ; Test accumulator for Z or N
     beq     FAST_RPM_CALC       ; If the Z bit of CCR is set, branch to FAST_RPM_CALC:

;SLOW_RPM_CALC:
     clr     intacc1             ; Clear intacc1 variable
     clr     intacc1+1           ; Clear intacc1+1 variable
     sthx    intacc2             ; Copy value in index register to intacc2 variable
     lda     #$1D                ; Load accumulator with $1D ("RPMk")
     sta     intacc1+2           ; Copy to "intacc1+2
     lda     #$4C                ; Load accumulator with $4C ("RPMk+1")
     sta     intacc1+3           ; Copy to "intacc1+3"
     jsr     udvd32              ; Jump to subroutine udvd32 (32x16 divide)
     lda     intacc1+3           ; Load accumulator with value in intacc1+3(8-bit RPM result)
     sta     rpm20               ; Copy to "rpm20"
     bra     RPM_CALC_DONE       ; Branch to RPM_CALC_DONE:

FAST_RPM_CALC:
     lda     #$1D          ; Load accumulator with $1D (RPMk)
     psha                  ; Push value in accumulator to stack
     pulh                  ; Pull value from stack to index register Hi byte
     lda     #$4C          ; Load accumulator with $4C (rpmk+1)
     div                   ; Divide (A = (H:A) / X)
     jsr     DIVROUND      ; Jump to "DIVROUND" subroutine,(round result)
     sta     rpm20         ; Copy to "rpm20"

RPM_CALC_DONE:

;**********************************************************************************************
; - Check for over speed. If overspeed, set the alarm bit which will skip over the Injector
;   Firing Control in the 100uS section which should slow things down in a hurry
;;**********************************************************************************************

     lda     rpm20                       ; Load accumulator with value in "rpm20"
     cmp     revloff                     ; Compare with "revloff" (175=3500RPM)
     bls     NO_REV_LIMIT                ; If (A=<(M), branch to NO_REV_LIMIT:
     bra     CHK_REV_LIMIT               ; Branch to CHK_REV_LIMIT:

NO_REV_LIMIT:
     brclr   REVL,alarmbits,REV_LIM_DONE ; If "REVL" bit of "alarmbits" is clear, branch to
                                         ; REV_LIM_DONE:(bit already clear, skip over)
     bclr    REVL,alarmbits              ; Clear "REVL" bit of "alarmbits"
     bra     REV_LIM_DONE                ; Branch to REV_LIM_DONE

CHK_REV_LIMIT:
     cmp     revlon                      ; Compare with "revlon" (225=4500RPM)
     bhs     REV_LIMIT                   ; If (A=>(M), branch to REV_LIMIT:
     bra     REV_LIM_DONE                ; Branch to REV_LIM_DONE

REV_LIMIT:
     brset   REVL,alarmbits,REV_LIM_DONE ; If "REVL" bit of "alarmbits" is set, branch to
                                         ; REV_LIM_DONE:(bit already set, skip over)
     bset    REVL,alarmbits              ; Set "REVL" bit of "alarmbits"

REV_LIM_DONE:

;***********************************************************************************************
; - Determine if engine RPMs are high enough that the ~1uS TIM2 counter
;   won't overflow between PIP signals, and that the engine has actually
;   started.(for ignition calcs)
;   At ~1uS, 65535 rollover occurs at 305.18 RPM 6cyl, and 228.88 RPM 8cyl
;   Start/Run break point for fuel is at 320 RPM, so this Lo limit is
;   set to 340 RPM
;***********************************************************************************************

     lda     rpm20             ; Load accumulator with value in RPM/20
     cmp     runon             ; Compare it with "runon" (17=340 RPM)
     bhi     SET_RUN_E2        ; If (A)>(M), branch to SET_RUN_E2:
     bclr    run,engine2       ; Clear "run" bit of "engine2" variable
     bra     CHK_LO_SPD_DONE   ; Branch to CHK_LO_SPD_DONE:

SET_RUN_E2:
     bset    run,engine2       ; Set "run" bit of "engine2" variable

CHK_LO_SPD_DONE:

;***********************************************************************************************
; - The high resolution tach uses a time stamp from the ~1uS counters T2CNTH:T2CNTL which
;   overflows at 306.044 RPM for 6 cyl, and 227.84 RPM for 8 cyl engines.
;   It is saved as tachcurH:tachcurL
;
;   Resolution @ 5000 RPM is 1.25 RPM 6 cyl, 1.67 RPM 8 cyl
;   Resolution @ 2500 RPM is 0.31 RPM 6 cyl, 0.42 RPM 8 cyl
;   Resolution @ 1000 RPM is 0.05 RPM 6 cyl, 0.07 RPM 8 cyl
;
;   tachcurH:tachcurL is added to tachTH:tachTM:TachTL for 250 ms, then divided by the value of
;   "tachcnt" which was incemented at every trigger signal, to smooth the read out.
;
;***********************************************************************************************

;***********************************************************************************************
; - First make sure that our engine speed is above roll over point for the 16 bit period value.
;   If it is, do the Hi Res tach period averaging calculations, if not rail tachH:tachL, clear
;   "tachcnt" so it will be at zero when the revs are within range and skip over.
;***********************************************************************************************

     brset   run,engine2,HI_RES_TACH     ; If "run" bit of "engine2" variable is set,
                                         ; branch to HI_RES_TACH:
     lda     #$FF                        ; Load accumulator with decimal 255
     sta     tachH                       ; Copy to "tachH"
     sta     tachL                       ; Copy to "tachL"
     clr     tachcnt                     ; Clear "tachcnt"
     jmp     HI_RES_TACH_DONE            ; Branch to HI_RES_TACH_DONE:

HI_RES_TACH:
     lda     tachTL       ; Load accumulator with value in "tachTL" (total Lo byte)
     add     tachcurL     ; Add "tachcurL" A<-(A)+(M)(current period Lo Byte)
     sta     tachTL       ; Copy to "tachTL"(result Lo byte)(updated total Lo byte
     lda     tachTM       ; Load accumulator with value in "tachTM" (total Mid byte)
     adc     tachcurH     ; Add with carry "tachcurH" A<-(A)+(M)(current period Hi Byte)
     sta     tachTM       ; Copy to "tachTM"(result Mid byte)(updated total Mid byte
     lda     tachTH       ; Load accumulator with value in "tachTH" (total Hi Byte)
     adc     #$0          ; Add with carry decimal zero A<-(A)+(M)(just the carry)
     sta     tachTH       ; Copy to "tachTH"(result Hi byte)(updated total Hi byte

;***********************************************************************************************
; - Check "clock250,inputs" to see if it's time to average the period values
;***********************************************************************************************

     brset   clock250,inputs,AV_HI_RES_PERIOD     ; If "clock250" bit of "inputs variable is
                                                  ; set, branch to AV_HO_RES_PERIOD:
     bra     HI_RES_TACH_DONE                     ; Branch to HI_RES_TACH_DONE:

AV_HI_RES_PERIOD:
     clr     intacc1             ; Clear intacc1 (divident Hi Byte)
     clr     intacc1+1           ; Clear intacc1+1 (dividend Mid Hi Byte)
     clr     intacc2             ; Clear intacc2 (divisor Hi Byte)
     lda     tachTH              ; Load accumulator with value in "tachTH"
     sta     intacc1+1           ; Copy to intacc1+1 (dividend Mid Hi Byte)
     lda     tachTM              ; Load accumulator with value in "tachTM"
     sta     intacc1+2           ; Copy to intacc1+2 (dividend Mid Lo Byte)
     lda     tachTL              ; Load accumulator with value in "tachTL"
     sta     intacc1+3           ; Copy to intacc1+3 (dividend Lo Byte)
     lda     tachcnt             ; Load accumulator with value in "tachcnt"
     sta     intacc2+1           ; Copy to intacc2+1 (divisor Lo Byte)
     jsr     udvd32              ; Jump to "udcd32" subroutine (32x16 divide)
     lda     intacc1+3           ; Load accumulator with value in intacc1+3 (result Lo Byte)
     sta     tachL               ; Copy to "tachL"
     lda     intacc1+2           ; Load accumulator with value in intacc1+2 (result Hi Byte)
     sta     tachH               ; Copy to "tachH"
     clr     tachTH              ; Clear "tachTH" (total Hi Byte)
     clr     tachTM              ; Clear "tachTM" (total Mid Byte)
     clr     tachTL              ; Clear "tachTL" (total Lo Byte)
     clr     tachcnt             ; Clear "tachcnt" (tachometer period averaging counter)
     bclr    clock250,inputs     ; Clear "clock250" bit of "inputs" variable

HI_RES_TACH_DONE:

;***********************************************************************************************
; - Determine if engine RPMs are high enough that we have finished cranking,
;   and that the engine has actually started. (for fuel calcs)
;***********************************************************************************************

;CHK_CRANK:
     lda     rpm20             ; Load accumulator with value in RPM/20
     cmp     startedon         ; Compare it with "startedon" (17=340 RPM)
     bhi     ENG_START         ; If (A)>(M), branch to ENG_START:
     bclr    started,engine2   ; Clear "started" bit of "engine2" variable
     bra     PIP_PASS_DONE     ; Branch to PIP_PASS_DONE:

ENG_START:
     bset    started,engine2   ; Set "started" bit of "engine2" variable

PIP_PASS_DONE:
     bclr    piprise,inputs    ; Clear "piprise" bit of "inputs" variable

NO_PIP_PASS:
     brset   cant_crank,engine2,WARM_UP_ENRICH ; If "cant_crank" bit of "engine2" variable is
                                               ; set, branch to WARM_UP_ENRICH:
     brset   started,engine2,WARM_UP_ENRICH    ; If "started" bit of "engine2" variable is
                                               ; set, branch to WARM_UP_ENRICH:


;***********************************************************************************************
;
; ------------------------------------- Cranking Mode ------------------------------------------
;
; Pulsewidth is directly set by the coolant temperature value of
;  CWU (at -40 degrees) and CWH (at 165 degrees) - value is interpolated
;
;***********************************************************************************************

;CRANKING_SET:
     bset    crank,engine	; Set crank bit of engine variable
     bclr    startw,engine	; Clear startw bit of engine variable
     bclr    warmup,engine	; Clear warmup bit of engine variable
     lda     floodClear       ; Load accumulator with value in "floodClear"(flood clear trigger)
     cmp     tps              ; Compare with current throttle position ADC reading
     bhi     INTERP_CRANK_PW	; If (A)>(M), branch to INTERP_CRANK_PW:

FLOOD_CLEAR:
        lda     #$01             ; Load accumulator with decimal 1(0.001 ms pulsewidth)
        sta     pwcalcl          ; Copy to "pwcalcl"
        clr     pwcalch          ; Clear "pwcalch"
        clr     pw               ; Clear "pw"
        clr     fd               ; Clear "fd"
        bset    fldClr,alarmbits ; Set "fldClr" bit of "alarmbits"
        jmp     LOOPER           ; Jump to LOOPER: (Keep looping until "floodClear">"tps"

INTERP_CRANK_PW:
     bclr    fldClr,alarmbits ; Clear "fldClr" bit of "alarmbits"
     lda     #$00             ; Load accumulator with decimal 0
     sta     tmp1             ; Copy to tmp1 variable
     lda     #$CD             ; Load accumulator with decimal 205 165 + 40 degrees
                              ; (offset in lookup table)
     sta     tmp2             ; Copy to tmp2 variable
     lda     cwu              ; Load accumulator with value in "cwu"
     sta     tmp3             ; Copy to "tmp3"
     lda     cwh              ; Load accumulator with value in "cwh"
     sta     tmp4             ; Copy to "tmp4"
     mov     coolant,tmp5     ; Move value at current coolant temp to tmp5
     jsr     lininterp        ; Jump to Lininterp: (interpolation subroutine)
     lda     tmp6             ; Load accumulator with value in tmp6 variable
                              ; (result of calculation)
     sta     pw               ; Copy to "pw"
     sta     fd               ; Copy to "fd"
     tax                      ; Transfer value in accumulator to index reg Lo
     lda     #$64             ; Load accumulator with decimal 100
     mul                      ; Multiply X:A <-(X)*(A)
     stx     pwcalch          ; Copy value in index register to
                              ; Calculated pulsewidth Hi byte
     sta     pwcalcl          ; Copy value in acumulator to
                              ; Calculated pulsewidth Lo byte
     jmp     LOOPER           ; Jump to LOOPER: (loop until engine starts)


;***********************************************************************************************
;
; ----------------------- Warm Up and After Start Enrichment section ---------------------------
;
; The Warm-up enrichment is a linear interpolated value from WWU (10 points)
; which are placed at configurabletemperatures in the WWURANGE table originally set at
; -40, -20, 0, +20, +40, +60, +80, +100, +130, and +160 degrees F
;
; The After Start enrichment is active below 160 degrees F and is added to the warm up
; enrichment percent value.
;
; Method:
;
; 1) If (warmup, engine is set) then:
; 2) Perform ordered table search of WWU (using coolant variable) to
;    determine which bin.
; 3) Perform linear interpolation to get interpolated warmup enrichment
;    percent
; 4)  else clear warmup bit in engine
;
; Also, the after-start enrichment value is calculated and applied here - it
;  is an added percent value on top of the warmup enrichment, and is applied
;  for the number of INJ1 firings or time specified in AWC. This enrichment starts
;  at a value of AWEV at first, then it linearly interpolates down to zero
;  after AWC cycles.
;
; where:
; warmup, engine = "warm up" bit of "engine" variable
; startw, engine = "after start enrichment" bit of "engine" variable
; awc            = After-start number of cycles
; asecount       = Counter value for after-start enrichment counter,
; AWEV           = After-start Warmup Percent enrichment add-on value
; warmcor        = After Start and Warmup Enrichment percent value
;
; 1) If (startw, engine is set) then:
; 2)  compare if (awc < asecount) then:
; 3)  x1=0, x2=AWC, y1=AWEV, y2=0, x=asecount, y=tmp6(ASE%)
; 4)  warmcor + tmp6 = warmcor
; 5)  else clear startw bit in engine
;
; - Rev 11 added ASE enrichment above "coolanton" set point to try eliminate hot start lean
;   conditions which may be brought on by heat soak of the manifold air temperature sensor,
;   heating of fuel in the rails, or a combination of both. The method is somewhat similar
;   to cold ASE but simpler in that under starting conditions of coolant temps of "coolanton"
;   set point  or greater, and air temps of "airtempon" set point or greater, the "awevh"
;   value is decayed by the "awch" value until it times out.
;
;***********************************************************************************************

WARM_UP_ENRICH:
     brclr   crank,engine,WUE1     ; If the "crank" bit of "engine" variable is clear,
                                   ; Branch to WUE1:
     bclr    crank,engine          ; Clear "crank" bit of "engine" variable
     bclr    fldClr,alarmbits      ; Clear "fldClr" bit of "alarmbits"
     bset    warmup,engine         ; Set "warmup" bit of "engine" variable
     bset    startw,engine         ; Set "startw" bit of "engine" variable
     clr     asecount              ; Clear the after-start enrichment counter variable

;***********************************************************************************************
; - Determine if we are starting cold or at operating temperature
;***********************************************************************************************

WUE1:
     brclr   warmup,engine,WUE2    ; If "warmup" bit of "engine" variable is clear
                                   ; Branch to WUE2:
     bra     CHK_WUE_RNG           ; Branch to CHK_WUE_RNG:

WUE2:
     brclr   startw,engine,WUE_DONE_J     ; If "startw" bit of "engine" variable is
                                          ; clear, branch to WUE_DONE_J:(long branch)
     bra     CHK_ASEH_RNG                 ; Branch to CHK_ASEH_RNG:

CHK_WUE_RNG:
     lda     coolant               ; Load accumulator with value in "coolant"
     cmp     coolanton             ; Compare value in accumulator with "coolanton"
     bhs     NOT_WUE_RNG           ; If "coolant" >= "coolanton", branch to NOT_WUE_RNG:
     bra     WUE_RNG               ; Branch to WUE_RNG:

NOT_WUE_RNG:
     brclr   warmup,engine,CHK_ASEH_RNG     ; If "warmup" bit of "engine" variable is
                                            ; already clear branch to CHK_ASEH_RNG:
     bclr    warmup,engine                  ; Clear "warmup" bit of "engine" variable
     lda     #$64                           ; Load accumulator with decimal 100
     sta     warmcor                        ; Copy value to Warmup Correction variable

;***********************************************************************************************
; - Determine if we are starting at operating temperature and a potential heat soak condition
;  (high Manifold Air Temperature)
;***********************************************************************************************

CHK_ASEH_RNG:
     brclr   startw,engine,WUE_DONE_J     ; If "startw" bit of "engine" variable is already
                                          ; clear, branch to WUE_DONE:(long branch)
     lda     airtemp            ; Load accumulator with value in Manifold Air Temperature F+40
     cmp     airtempon          ; Compare value in accumulator with "airtempon"
     bhs     ASEH_RNG           ; If "mat" >= "airtempon", branch to ASEH_RNG:
     bclr    startw,engine      ; Clear "startw" bit of "engine" variable
     bra     WUE_DONE_J         ; Branch to WUE_DONE_J:(long branch)


;***********************************************************************************************
; - Hot start ASE section
;***********************************************************************************************

ASEH_RNG:
     lda     asecount           ; Load accumulator with Counter value for
                                ; after-start enrichment counter
     cmp     awch               ; Compare to After-start Hot number of cycles
     bhi     ASE_ASEH_END       ; If higher, branch to ASE_ASEH_END:(hot start ASE finished)
     clr     tmp1               ; Clear tmp1 variable
     lda     awch               ; Load accumulator with value in After-start hot number of cycles
     sta     tmp2               ; Copy to tmp2 variable
     lda     awevh              ; Load accumulator with value in After-start Hot Warmup Percent
                                ; enrichment add-on value
     sta     tmp3               ; Copy to tmp3 variable
     clr     tmp4               ; Clear tmp4 variable
     mov     asecount,tmp5      ; Move Counter value for after-start enrichment counter to
                                ; tmp5 variable
     jsr     lininterp          ; Jump to Liniterp subroutine, (result in tmp6)
     lda     #$64               ; Load accumulator with decimal 100
     sta     warmcor            ; Copy value to Warmup Correction variable
     lda     tmp6               ; Load accumulator with value in tmp6
     add     warmcor            ; Add it to Warmup Correction variable (result in accumulator)
     sta     warmcor            ; Copy new value to Warmup Correction variable
     bra     WUE_ASE_ASEH_DONE  ; Branch to WUE_ASE_ASEH_DONE:

;***********************************************************************************************
; - Long Branch
;***********************************************************************************************

WUE_DONE_J:
     bra     WUE_ASE_ASEH_DONE     ; Branch to WUE_ASE_ASEH_DONE:

;***********************************************************************************************
; - Cold start warm up and ASE Section
;***********************************************************************************************

WUE_RNG:
     ldhx    #WWURANGE		  ; Load Index register with address of first byte of WWURANGE
     sthx    tmp1               ; Copy value into tmp1 variable
     lda     #$09               ; Load accumulator with decimal 9
     sta     tmp3               ; Copy value into tmp3 variable
     lda     coolant		  ; Load accumulator with current coolant temp
     sta     tmp4               ; Copy value into tmp4 variable
     jsr     ORD_TABLE_FIND     ; jump to subroutine ORD_TABLE_FIND (result in tmp5)
     clrh                       ; Clear index register hi byte
     lda     tmp5               ; Load accumulator with result from ORD_TABLE_FIND
     tax                        ; Copy it to index register Lo byte
     lda     WWU,x              ; Load the accumulator with the warmup bin from index
                                ; register Lo byte
     sta     tmp4               ; Copy it to the tmp4 variable
     decx                       ; Decrement index reg Lo byt(move down 1 bin)
     lda     WWU,x              ; Load accumulator with new bin value
     sta     tmp3               ; Copy it to the tmp3 variable
     mov     coolant,tmp5       ; Move current coolant temp to tmp5 variable
     jsr     lininterp          ; Jump to Lininterp subroutine(result in temp6)
     mov     tmp6,warmcor       ; Copy to Total Warmup Correction variable
     lda     warmcor            ; Load accumulator with Warmup Correction variable
     cmp     #$64               ; Compare it to decimal 100
     bne     ASE_RNG            ; If not equal, branch to ASE_RNG:
     bra     WUE_END            ; Branch to WUE_END:

ASE_RNG:
     lda     asecount           ; Load accumulator with Counter value for
                                ; after-start enrichment counter
     cmp     awc                ; Compare to After-start number of cycles
     bhi     ASE_ASEH_END       ; If higher, branch to ASE_ASEH_END:
     clr     tmp1               ; Clear tmp1 variable
     lda     awc                ; Load accumulator with value in After-start number of cycles
     sta     tmp2               ; Copy to tmp2 variable
     lda     awev               ; Load accumulator with value in After-start Warmup Percent
                                ; enrichment add-on value
     sta     tmp3               ; Copy to tmp3 variable
     clr     tmp4               ; Clear tmp4 variable
     mov     asecount,tmp5      ; Move Counter value for after-start enrichment counter to
                                ; tmp5 variable
     jsr     lininterp          ; Jump to Liniterp subroutine, (result in tmp6)
     lda     tmp6               ; Load accumulator with value in tmp6
     add     warmcor            ; Add it to Warmup Correction variable (result in accumulator)
     sta     warmcor            ; Copy new value to Warmup Correction variable
     bra     WUE_ASE_ASEH_DONE  ; Branch to WUE_ASE_ASEH_DONE:

;***********************************************************************************************
; Outside of ASE or ASEH range, clear ASE or ASEH  mode
;***********************************************************************************************

ASE_ASEH_END:
     bclr    startw,engine         ; Clear "startw" bit of "engine" variable
     bra     WUE_ASE_ASEH_DONE     ; Branch to WUE_ASE_ASEH_DONE:

;***********************************************************************************************
; Outside of WUE range - clear WUE and ASE modes
;***********************************************************************************************

WUE_END:
     bclr    warmup,engine      ; Clear "warmup" bit of "engine" variable
     bclr    startw,engine      ; Clear "startw" bit of "engine" variable

WUE_ASE_ASEH_DONE:

;***********************************************************************************************
;
; ------------------------- Throttle Acceleration Enrichment section ---------------------------
;
; Method is the following, where:
;
; tps         = Throttle Position Sensor ADC Raw Reading, counts, 0-5 volts
; last_tps    = TPS reading updated every 0.1 seconds
; tpsthresh   = Accel TPS difference over time threshold
; TPSAEN      = "engine" bit4, 0 = not in TPS accel mode 1 = TPS accel mode
; TPSDEN      = "engine" bit5, 0 = not in deaccel mode   1 = in deaccel mode
; TPSAQ       = TPS acceleration amount (function of TPSDOT) in 0.1 ms units
; TPSDQ       = Deceleration fuel cut  100 = no fuel cut, 1 = 99% fuel cut
; TPSACLK     = TPS enrichment timer clock in 0.1 second resolution
; TPSACLKCMP  = Comparison value for TPS acceleration time - from lookup table
; TPSACCEL    = Acceleration enrichment % (0 = 0% enrich, 255 = 255% enrich)
; TPSFUELCUT  = TPS fuel cut % variable ( 1 = 99% cut, 100 = 0% cut)
; rpm         = Computed engine RPM - rpm/20 (20 RPM resolution)
; DFC_en      = Decel Fuel Cut Enable on PTA2
;
;
;   ACCELERATION ENRICHMENT:
;   If (tps < lasr_tps) go to DEACCELERATION ENRICHMENT
;   If (tps - last_tps) > tpsthresh and TPSAEN = 0 then (accel enrichemnt):
;   {
;    1) Set acceleration mode
;    2) Continuously determine rate-of-change of throttle, and perform
;        interpolation of TPSAQ values to determine acceleration
;        enrichment amount to apply.
;   }
;   If (TPSACLK > TPSACLKCMP) and TPSAEN = 1, then:
;   {
;    1) Clear TPSAEN bit in engine
;    2) Set TPSACCEL to 100% (no enrichment)
;    3) Go to end of section
;   }
;
;
;   DEACCELERATION ENRICHMENT:
;    If (lasr_tps - tps > tpsthresh then (deaccelleration fuel cut)
;    {
;     If (TPSAEN = 1) then:
;     {
;      1) TPSACCEL = 100 percent (no acceleration)
;      2) Clear TPSAEN bit in ENGINE
;      3) Go to end of section
;    }
;    If (RPM > 60 (1200 RPM) then (fuel cut mode):
;    {
;      1) Set TPSACCEL value to TPSDQ
;      2) Set TPSDEN bit in ENGINE
;      3) Go to end of section
;    }
;   }
;   else
;   {
;    If (TPSDEN = 1) then
;    {
;      1) Clear TPSACCEL value in ENGINE
;      2) TPSACCEL = 100%
;      3) Go to end of section
;    }
;   }
;
;***********************************************************************************************

;***********************************************************************************************
; - Acceleration enrichment calculations are done only once in the first
;   pass through the main loop after the 100mS clock tick signal has been
;   received, the interrupt routine has been completed, and relevent
;   variables updated.
;   NOTE! This doesn't appear to work so it's commented out
;***********************************************************************************************

;     brset   clock100,inputs,TAE    ; If "clock100" bit of "inputs" variable is
                                     ; clear, branch to TAE:
;     jmp     NO_CLOCK_PASS          ; Jump to NO_CLOCK_PASS:


TAE:
     sei                       ; Set interrupt mask bit
     mov     tps,tmp1          ; Move Throttle Position reading to tmp1 var
     mov     last_tps,tmp2     ; Move Last Throttle Position reading to tmp2
     cli                       ; Clear interupt mask bit
     lda     tmp1              ; Load accumulator with value in tmp1
     cmp     tmp2              ; Compare accumulator with tmp2
     blo     TDE2              ; If (A)<(M) branch to TDE2: (deacelleration section)

;AE_CHK:
     lda     tmp1                             ; Load accumulator with value in tmp1 (TPS)
     sub     tmp2                             ; Subtract, A<-(A)-(M) (TPS - Last_TPS)
     cmp     tpsthresh                        ; Compare result to Accel TPS DOT threshold value
     blo     B_LO_CONT                        ; If (A)<(M), branch to B_LO_CONT:
     brset   tpsaen,engine,AE_COMP_SHOOT_AMT  ; If "TPSAEN" bit of "engine" is set branch to
                                              ; AE_COMP_SHOOT_AMT:

;***********************************************************************************************
; Add in accelleration  enrichement
;***********************************************************************************************

     lda     tpsaq           ; Load accumulator with value in first element
                             ;(start out using first element)
                             ;(will determine actual next time around)
     sta     tpsaccel        ; Copy to Acceleration percent amount
                             ; (used in later calculations)
     clr     tpsaclk         ; Clear Comparison value for TPS accel time var
     lda     tpsasync	     ; Load accumulator with TPS Accel clock value
     sta     tpsaclkcmp      ; Copy to Comparison value for TPS accel time
                             ;(Shoot time comparison value)
     bset    tpsaen,engine   ; Set "tpsaen" bit of "engine" variable
     bclr    tpsden,engine   ; Clear "tpsden" bit of "engine" variable
     jmp     TDE_DONE        ; Jump to TDE_DONE:


;***********************************************************************************************
; First, calculate Cold temperature add-on enrichment value from coolant
;  value TPSACOLD, from -40 degrees to 165 degrees.
;
; Then determine cold temperature multiplier value ACCELMULT (in percent),
;  from -40 degrees to 165 degrees.
;
; Next, Calculate Shoot amount (quantity) for accel enrichment from table.
;  Find bins (between) for corresponding TPSDOT, and linear interpolate
;  to find enrichment amount (from TPSAQ). This is continuously
;  checked every time thru main loop while in acceleration mode,
;  and the highest value is latched and used.
;
; The final acceleration applied is
;  AE = Alookup(TPSDOT) * (ACCELMULT/100) + TPSACOLD
;
;***********************************************************************************************

AE_COMP_SHOOT_AMT:

;***********************************************************************************************
; First, the "added" amount based on cold temperatures
;***********************************************************************************************

     lda     #$00              ; Load accumulator with 0 (0 -> - 40 degrees)
     sta     tmp1              ; Copy to tmp1 variable
     lda     #$CD              ; Load accumulator with decimal 205
                               ; 165 + 40 degrees (offset in lookup table)
     sta     tmp2              ; Copy to to tmp2 variable (this is the amount at coldest)
     lda     tpsacold          ; Load accumulator with value in "tpsacold"
     sta     tmp3              ; Copy to tmp3 variable
     mov     #$00,tmp4         ; Move decimal 0 to tmp4 variable (no enrichemnt add on at warm
                               ; temperature)
     mov     coolant,tmp5      ; Move current coolant temp value to tmp5 var
     jsr     lininterp         ; Jump to liniterp subroutine (result in tmp6)
     mov     tmp6,tmp13        ; Move result to tmp13 (save here temporarily)

;***********************************************************************************************
; Second, find the multiplier (ACCELMULT) amount based on cold temperatures
;***********************************************************************************************

     lda     #$00             ; Load accumulator with 0 (0 -> - 40 degrees)
     sta     tmp1             ; Copy to tmp1 variable
     lda     #$CD             ; Load accumulator with decimal 205 (165 + 40 degrees)
                              ; (offset in lookup table)
     sta     tmp2             ; Copy to to tmp2 variable
     lda     ACMULT           ; load accumulator with Acceleration cold multiplication factor
                              ; (percent/100)
     sta     tmp3             ; Copy to tmp3 variable (This is the amount at coldest)
     mov     #$64,tmp4        ; Move decimal 100 to tmp4 variable
                              ;(1.00 multiplier at 165 degrees)
     mov     coolant,tmp5     ; Move coolant temperature to tmp5 variable
     jsr     lininterp        ; Jump to liniterp subroutine (result in tmp6)
     mov     tmp6,tmp14       ; Move result to tmp14 (save here temporarily)

;***********************************************************************************************
; This is here to extend the branch range for the instructions earlier
; (long-jumps)
;***********************************************************************************************

     bra     END_AE_LONGBRANCH    ; Branch to END_AE_LONGBRANCH:

B_LO_CONT:
     bra     TAE_CHK_TIME         ; Branch to TAE_CHK_TIME:

TDE2:
     bra     TDE                  ; Branch to TDE:

END_AE_LONGBRANCH:


;***********************************************************************************************
; Now the lookup table amount based on TPSDOT (TPS difference over time)
;***********************************************************************************************

     ldhx    #tpsdotrate     ; Load Index register with value in "tpsdotrate"
     sthx    tmp1            ; Copy to tmp1 variable
     lda     #$0F            ; Load accumulator with decimal 15 (300RPM)
     sta     tmp3            ; Copy to TMP3 variable
     lda     tps             ; Load accumulator with value in Throttle Position Sensor ADC
     sub     last_tps        ; Subract it from the last TPS value
     sta     tmp4            ; Copy result to tmp4 variable ( TPSDOT )
     sta     tmp10           ; Copy result in tmp10 variable as well (Save away for later
                             ; use below)
     jsr     ord_table_find  ; Jump to subroutine ord_table_find( result in tmp5 )
     clrh                    ; Clear the Index Register
     lda     tmp5            ; Load accumulator with value in tmp5
     tax                     ; Transfer Index Register Lo byte
     lda     TPSAQ,x         ; Load accumulator with TPS acceleration amount
                             ; (TPSAQ is entry, offset in index register Lo
     sta     tmp4            ; Copy to tmp4 variable
     decx                    ; Decrement index register Lo byte
     lda     TPSAQ,x         ; Load accumulator with this new value
     sta     tmp3            ; Copy it to tmp3 variable
     lda     tmp10           ; Load accumulator with value in tmp10
     sta     tmp5            ; Copy it to tmp5 variable
     jsr     lininterp       ; Jump to subroutine lininterp (result in tmp6)

;***********************************************************************************************
; Now, the final applied acceleration enrichment amount is
; ((TMP6 * tmp14)/100) + tmp13
;  AE = Alookup(TPSDOT) * (ACCELMULT/100) + TPSACOLD
;***********************************************************************************************

;FIND_TOTAL_A:
     lda     tmp6            ; Load accumulator with value in tmp6( result of lininterp)
     tax                     ; Transfer to index register Lo byte
     lda     tmp14           ; Load accumulator with value in tmp14
     mul                     ; Multiply tmp6 by tmp14
     pshx                    ; Push index register Lo onto stack
     pulh                    ; Pull index register L to index register Hi
     ldx     #$64            ; Load index register low with decimal 100
     div                     ; Divide A<-(H:A)/(X);H<-Remainder
     bcs     UPPER_RAIL_AE   ; If the C bit of CCR is set, branch to UPPER_RAIL_AE:
     psha                    ; Push accumulator onto stack
     pshh                    ; Push index register high onto stack
     pula                    ; Pull index register high into accumulator
     cmp     #$32            ; Compare accumulator with decimal 50
     ble     NDA1            ; If less than or equal to, branch to NDA1:
     pula                    ; Pull value from stack to accumulator
     inca                    ; Increment accumulator
     bra     ADD_TO_AE       ; Branch to ADD_TO_AE:

NDA1:
     pula                    ; Pull value from stack to accumulator
     bra     ADD_TO_AE       ; Branch to ADD_TO_AE:

UPPER_RAIL_AE:
     lda     #$C8            ; Load accumulator with decimal 200(Set to 20 milliseconds railed)
     bra     ADD_TO_AE       ; Branch to ADD_TO_AE:

ADD_TO_AE:
     add     tmp13            ; Add value in accumulator with value in tmp13(Add on the amount
                              ;(computed in col temperature enrich above)
     sta     tmp6             ; Copy result to tmp6 variable
     cmp     TPSACCEL         ; Compare with Acceleration percent amount
     blo     TAE_CHK_TIME     ; If lower, branch to TAE_CHK_TIME:
     lda     tmp6             ; Load accumulator with value in tmp6
                              ; (Replace with this higher value)
     sta     TPSACCEL         ; Copy value to Acceleration percent amount


;***********************************************************************************************
; Check if acceleration is done
;***********************************************************************************************

TAE_CHK_TIME:
     brset   tpsden,engine,RST_ACCEL ; If deceleration bit of "Engine" variable is set,
                                     ; branch to RST_ACCEL:
     lda     tpsaclk                 ; Load accumulator with value in TPS enrichment timer clock
     cmp     tpsaclkcmp              ; Compare it to Comparison value for TPS acceleration time
     blo     TDE_DONE                ; If lower, branch to TDE_DONE:

RST_ACCEL:
     bclr    tpsaen,engine     ; Clear "tpsaen" bit of "engine" variable
     lda     #$64              ; Load accumulator with decimal 100
     sta     tpsfuelcut        ; Copy to TPS Fuel Cut % variable (0% cut)
     clr     tpsaccel          ; Clear Acceleration enrich % var (0% enrich)
     bclr    tpsden,engine     ; Clear "tpsden" bit of "engine" variable

;***********************************************************************************************
; - Deceleration fuel cut mode
;***********************************************************************************************

TDE:
     lda     tmp2                            ; Load accumulator with value in tmp2(last_tps)
     sub     tmp1                            ; A<-(A)-(M)(last_tps - tps)
     cmp     tpsthresh                       ; Compare result with value in "tpsthresh"
     blo     TDE_CHK_DONE                    ; If A<M, branch to TDE_CHK_DONE:
     brclr   tpsaen,engine,TDE_CHK_FUEL_CUT  ; If "tpsaen" bit of "engine" variable is set,
                                             ; branch to TDE_CHK_DONE:
     lda     #$64                            ; Load accumulator with decimal 100
     sta     tpsfuelcut                      ; Copy to "tpsfuelcut" variable
     clr     tpsaccel                        ; Clear "tpsaccel" variable
     bclr    tpsaen,engine                   ; Clear "tpsaen" bit of "engine" variable
     bclr    tpsden,engine                   ; Clear "tpsden" bit of "engine" variable
     bra     TDE_DONE                        ; Branch to TDE_DONE:

TDE_CHK_FUEL_CUT:
     lda     rpm20             ; Load accumulator with value in "rpm20"
     cmp     #$4B              ; Compare with decimal 75(1500 RPM)
     blo     TDE_DONE          ; If (A)<(M) branch to TDE_DONE(Revs too low, no fuel cut)
     lda     tpsdq             ; Load accumulator with value in "tpsdq"(fuel cut %)
     sta     tpsfuelcut        ; Copy to "tpsfuelcut"
     bset    tpsden,engine     ; Set "tpsden" bit of "engine"
     bclr    tpsaen,engine     ; Clear "tpsaen" bit of "engine"
     bra     TDE_DONE          ; Branch to TDE_DONE:

TDE_CHK_DONE:
     brclr   tpsden,engine,TDE_DONE  ; If "tpsden" bit of "engine" is clear, branch to
                                     ; TDE_DONE:
     bclr    tpsden,engine           ; Clear "tpsden" bit of "engine" variable
     lda     #$64                    ; Load accumulator with decimal 100
     sta     tpsfuelcut              ; Copy to "tpsfuelcut" variable
     clr     tpsaccel                ; Clear "tpsaccel" variable

TDE_DONE:
     bclr    clock100,inputs         ; Clear "clock100" bit of "inputs" variable

NO_CLOCK_PASS:

;***********************************************************************************************
;
; VE 3-D Table Lookup
;
;  This is used to determine value of VE based on RPM and MAP
;  The table looks like:
;
;     105 +....+....+....+....+....+....+....+
;         ....................................
;      00 +....+....+....+....+....+....+....+
;                    ...
;  KPA                ...
;                        ...
;      35 +....+....+....+....+....+....+....+
;         5    15   25   35   45   55   65   75 RPM/20
;
;
; Steps:
;  1) Find the bracketing KPA positions via ORD_TABLE_FIND, put index in
;      tmp8 and bounding values in tmp9(kpa1) and tmp10(kpa2)
;  2) Find the bracketing RPM positions via ORD_TABLE_FIND, store index in
;      tmp11 and bounding values in tmp13(rpm1) and tmp14(rpm2)
;  3) Using the VE table, find the table VE values for tmp15=VE(kpa1,rpm1),
;      tmp16=VE(kpa1,rpm2), tmp17 = VE(kpa2,rpm1), and tmp18 = VE(kpa2,rpm2)
;  4) Find the interpolated VE value at the lower KPA range :
;      x1=rpm1, x2=rpm2, y1=VE(kpa1,rpm1), y2=VE(kpa1,rpm2) - put in tmp19
;  5) Find the interpolated VE value at the upper KPA range :
;      x1=f1, x2=rpm2, y1=VE(kpa2,rpm1), y2=VE(kpa2,rpm2) - put in tmp11
;  6) Find the final VE value using the two interpolated VE values:
;      x1=kpa1, x2=kpa2, y1=VE_FROM_STEP_4, y2=VE_FROM_STEP_5
;
;***********************************************************************************************

;VETABLELOOKUP:

;VE_STEP_1:
     ldhx    #KPARANGEVE_f       ; Load index register with value in KPARANGEVE table
     sthx    tmp1                ; Copy value to tmp1 variable
     mov     #$0B,tmp3           ; Move decimal 12 into "tmp3"
     mov     kpa,tmp4            ; Move value in "kpa" to "tmp4"
     jsr     ORD_TABLE_FIND	   ; Jump to subroutine ORD_TABLE_FIND,(result in tmp5)
     lda     tmp1                ; Load accumulator with value in tmp1
     lda     tmp2                ; Load accumulator with value in tmp2
     mov     tmp5,tmp8		   ; move value from tmp5 to tmp8 (Index)
     mov     tmp1,tmp9		   ; move value from tmp1 to tmp9 (X1)
     mov     tmp2,tmp10		   ; move value from tmp2 to tmp10 (X2)

;VE_STEP_2:
     ldhx    #RPMRANGEVE_f       ; Load index register with value from RPMRANGEVE table
     sthx    tmp1                ; Copy value to tmp1 variable
     mov     #$0B,tmp3           ; Move decimal 12 into "tmp3"
     mov     rpm20,tmp4          ; Move value in "rpm20" to "tmp4"
     jsr     ORD_TABLE_FIND	   ; Jump to subroutine ORD_TABLE_FIND,(result in tmp5)
     mov     tmp5,tmp11		   ; Move value from tmp5 to tmp11 (Index)
     mov     tmp1,tmp13		   ; Move value from tmp1 to tmp13 (X1)
     mov     tmp2,tmp14		   ; Move value from tmp2 to tmp14 (X2)

;VE_STEP_3:
        clrh              ; Clear index register Hi byte
        ldx     #$0C      ; Load index register Lo byte with decimal 13
        lda     tmp8      ; Load accumulator with value in "tmp8"
        deca              ; Decrement value in accumulator
        mul               ; Multiply X:A<-(X)x(A)
        add     tmp11     ; Add "tmp11" to result A<-(A)+(M)
        deca              ; Decrement value in accumulator
        tax               ; Transfer value in accumulator to index register Lo byte
        jsr     VE1X      ; Jump to subroutine at VE1X:
        sta     tmp15     ; Copy result to "tmp15"
        incx              ; Increment value in index rgister Lo byte
        jsr     VE1X      ; Jump to subroutine at VE1X:
        sta     tmp16     ; Copy result to "tmp16"
        ldx     #$0C      ; Load index register Lo byte with decimal 13
        lda     tmp8      ; Load accumulator with value in "tmp8"
        mul               ; Multiply X:A<-(X)x(A)
        add     tmp11     ; Add "tmp11" to result A<-(A)+(M)
        deca              ; Decrement value in accumulator
        tax               ; Transfer value in accumulator to index register Lo byte
        jsr     VE1X      ; Jump to subroutine at VE1X:
        sta     tmp17     ; Copy result to "tmp17"
        incx              ; Increment value in index rgister Lo byte
        jsr     VE1X      ; Jump to subroutine at VE1X:
        sta     tmp18     ; Copy result to "tmp18"

;VE_STEP_4:
     mov     tmp13,tmp1	; Move value from tmp13 variable to tmp1 variable
     mov     tmp14,tmp2	; Move value from tmp14 variable to tmp2 variable
     mov     tmp15,tmp3	; Move value from tmp15 variable to tmp3 variable
     mov     tmp16,tmp4	; Move value from tmp16 variable to tmp4 variable
     mov     rpm20,tmp5	; Move RPM/20 to tmp5 variable
     jsr     lininterp	; Jump to subroutine lininterp (result in tmp6)
     mov     tmp6,tmp19	; Move value from tmp6 variable to tmp19 variable

;VE_STEP_5:
     mov     tmp13,tmp1	; Move value from tmp13 variable to tmp1 variable
     mov     tmp14,tmp2	; Move value from tmp14 variable to tmp2 variable
     mov     tmp17,tmp3	; Move value from tmp17 variable to tmp3 variable
     mov     tmp18,tmp4	; Move value from tmp18 variable to tmp4 variable
     mov     rpm20,tmp5	; Move RPM/20 to tmp5 variable
     jsr     lininterp	; Jump to subroutine lininterp (result in tmp6)
     mov     tmp6,tmp11	; Move value from tmp6 variable to tmp11 variable

;VE_STEP_6:
     mov     tmp9,tmp1	 ; Move value from tmp9 variable to tmp1 variable
     mov     tmp10,tmp2	 ; Move value from tmp10 variable to tmp2 variable
     mov     tmp19,tmp3	 ; Move value from tmp19 variable to tmp3 variable
     mov     tmp11,tmp4	 ; Move value from tmp11 variable to tmp4 variable
     mov     kpa,tmp5	 ; Move value in MAP value in units of KPa to tmp5
     lda     kpa         ; Load accumulator with value in MAP value in KPa
     jsr     lininterp	 ; Jump to subroutine lininterp (result in tmp6)
     mov     tmp6,vecurr ; Move value from tmp6 variable to Current VE
                         ; value from lookup table - percent


;***********************************************************************************************
;
; ---------------------------- Computation of Fuel Parameters ----------------------------------
;
; Remainders are maintained for hi-resolution mode
;
; Where:
;  Ftrimcor   = Fuel trim correction factor (85 to 115)(-15% to +15%)
;  warmcor    = Total warmup enrichment,(Warmup and After Start)
;               (0 to 255)(0% to 255%)(prctical values 100 to 175)
;  Tpsfuelcut = TPS fuel cut amount (1 = 99% cut, 100 = 0% cut)
;  barocorr   = Barometric pressure correction (27 to 141)(27% to 141%)
;               (practical value 100 to 110)
;  aircorr    = Air Density correction ( 90 to 105) (90% to 105%)
;  kpa        = Manifold Absolute Pressure (12 to 255kpa)
;               (practical value naturally aspirated 15 to 98)
;  vecurr     = Volumetric Efficiency (0 to 255)
;               (practical value naturally aspirated 25 to 150)
;               (Actually, the VE table is a combination of
;                volumetric efficiency and Lambda. Both can change with
;                RPM and load, so they are combined in a common 3D Table)
;  REQ_FUEL   = Required fuel pulse width (0 - 25.5mS) at 100%VE, 68
;               degrees F Manifold Air Temp and atmospheric pressure to
;               achieve stoichiometric air fuel ratio for 1 cylinder per
;               combustion cycle (720 crank degrees)
;
; Step 1 (warmcor * Tpsfuelcut)/100 = R1 + rem1/100
; Step 2 (barocor * Aircor)/100 = R2 + rem2/100
; Step 3 ((R1 + rem1/100) * (R2 + rem2/100)) / 100 = R3 + rem3/100
; Step 4 (Ftrimcor * vecurr)/100 = R4 + rem4/100
; Step 5 ((R3 + rem3/100) * (R4 + rem4/100)) /100 = R5 + rem5/100
; Step 6 (kpa * REQ_FUEL)/100 = R6 + rem6/100
; Step 7 ((R5 + rem5/100) * (R6 + rem6/100))  = R7 + rem7/100
;
;  The result in R7 is the 1 microsecond result
;
; NOTE!! It is important to note that the results of all steps except
; step 7 cannot exceed 25500, or the calculations will not be accurate
;
;***********************************************************************************************

;***********************************************************************************************
;
; -------------------------- Computation of Normalized Variables -------------------------------
;
;  The following is the form of the evaluation for the normalized variables:
;
;  (A rem A * B)
;  -------------  = C rem C
;      100
;
;  Where A = Whole part of the percentage,
;        rem A = Remainder of A from previous calculation (range 0 to 99)
;        B = Percentage multiplied (this always has a zero remainder)
;        C = Whole part of result
;        rem C = remainder of result
;
;
;  Calculation is performed by the following method:
;
;     |(A * B) + (rem A * B)|
;     |          -----------|
;     |              100    |
;     ----------------------- = C rem C
;             100
;
;
;   Inputs:  tmp10 = A
;            tmp11 = rem A
;            tmp12 = B
;            tmp13 = rem B
;
;   Outputs: tmp10 = C
;            tmp11 = rem C
;            tmp13 = high order part of (A rem A) * B
;            tmp14 = low order part of (A rem A) * B
;
;***********************************************************************************************

;WARMACCEL_COMP:

     mov     warmcor,tmp10      ; Warmup Correction in tmp10
     clr     tmp11              ; tmp11 is zero
     mov     tpsfuelcut,tmp12   ; tpsfuelcut in tmp12
     clr     tmp13              ; tmp13 is zero
     jsr     Supernorm          ; Multiply and divide by 100(warmcor and tpsfuelcut)
                                ;(result in tmp10:tmp11)=R1+rem1/100
     mov     tmp10,tmp1	        ; save whole result in tmp1
     mov     tmp11,tmp2	        ; save remainder in tmp2(tmp1:tmp2 is R1+rem1/100)
     mov     barocor,tmp10      ; tmp10 is barometer percent
     clr     tmp11              ; zero to tmp11
     mov     aircor,tmp12       ; air temp correction % in tmp12
     clr     tmp13              ; tmp13 is zero
     jsr     Supernorm          ; Multiply and divide by 100(barocor and aircor)
                                ;(result in tmp10:tmp11)=R2+rem2/100
     mov     tmp1,tmp12	        ; move saved tmp1 into tmp12
     mov     tmp2,tmp13	        ; move saved tmp2 into tmp13
     jsr     Supernorm          ; Multiply and divide by 100(R1+rem1/100 and R2+rem2/100)
                                ;(result in tmp10:tmp11)=R3+rem3/100
     mov     tmp10,tmp5	        ; save whole result into tmp5
     mov     tmp11,tmp6	        ; save remainder into tmp6(R3+rem3/100)
     lda     tmp10              ;(R3)
     sta     gammae             ; Copy to "Gammae" variable
     mov     Ftrimcor,tmp10     ; Fuel Trim correction percent into tmp10
     clr     tmp11              ; Remainder is zero
     mov     vecurr,tmp12       ; VE into tmp12
     clr     tmp13              ; Remainder is zero
     jsr     Supernorm          ; Multiply and divide by 100(Ftrimcor and vecurr)
                                ;(result in tmp10:tmp11)=R4+rem4/100
     mov     tmp5,tmp12	        ; move saved tmp5 into tmp12
     mov     tmp6,tmp13	        ; move saved tmp6 into tmp13
     jsr     Supernorm          ; Multiply and divide by 100(R3+rem3/100 and R4+rem4/100)
                                ;(result in tmp10:tmp11)=R5+rem5/100
     mov     tmp10,tmp3	        ; save whole result into tmp3
     mov     tmp11,tmp4	        ; save remainder into tmp4(R5+rem5/100)
     mov     kpa,tmp10          ; MAP into tmp10
     clr     tmp11              ; tmp11 is zero
     lda     REQ_FUEL           ; Load accumulator with value in "REQ_FUEL"
     sta     tmp12              ; Copy to tmp12 variable
     clr     tmp13              ; tmp13 is zero
     jsr     Supernorm          ; Multiply and divide by 100(kpa and REQ_FUEL)
                                ;(result in tmp10:tmp11)=R6+rem6/100
     mov     tmp3,tmp12	        ; move saved tmp3 into tmp12
     mov     tmp4,tmp13	        ; move saved tmp4 into tmp13
     jsr     Supernorm	        ; Multiply and divide by 100(R5+rem5/100 and R6+rem6/100)
                                ;(result in tmp13:tmp14)=R7+rem7/100
     mov     tmp13,pwnadH       ; Move value in tmp13 to "pwnadH"
     mov     tmp14,pwnadL       ; Move value in tmp14 to "pwnadL"


************************************************************************************************
**
** Calculation of Battery Voltage Correction for Injector Opening Time
**
** Injector open time is implemented as a linear function of
**  battery voltage, from 7.2 volts (61 ADC counts) to 19.2 volts (164 counts),
**  with 13.2 volts (113 counts) being the nominal operating voltage
**
** INJOPEN = injector open time at 13.2 volts in mms
** BATTFAC = injector open adjustment factor 6 volts from 13.2V in mms
**
**
** + (INJOPEN + BATTFAC)
** +   *
** +                     (INJOPEN)
** +                         *
** +                                       (INJOPEN - BATTFAC)
** +                                               *
** +
** ++++++++++++++++++++++++++++++++++++++++++++++++++++++
**    6.0V                 13.2V                16.2
**
***********************************************************************************************

BATT_CORR_COMP:
     lda     #61T        ; Load accumulator with decimal 61
     sta     tmp1        ; Copy to "tmp1"
     lda     #164T       ; Load accumulator with decimal 164
     sta     tmp2        ; Copy to "tmp2"
     lda     injopen     ; Load accumulator with value in "injopen"
     add     battfac     ; Add A<-(A)+(M)
     sta     tmp3        ; Copy result to "tmp3"
     lda     injopen     ; Load accumulator with value in "injopen"
     sub     battfac     ; Subtract A<-(A)-(M)
     sta     tmp4        ; Copy result to "tmp4"
     bpl     MBFF        ; Branch to MBFF: if plus(Check if minus condition)
     clr     tmp4        ; Clear "tmp4"
MBFF:
     mov     batt,tmp5   ; Move value in "batt" to "tmp5"
     jsr     lininterp   ; Jump to subroutine at lininterp:(injector open time in tmp6)
     lda     tmp6        ; Load accumulator with value in tmp6
     sta     deadband    ; Copy to Injector Deadband variable.


;***********************************************************************************************
;
; --------------------------- Calculation of Final Pulse Width ---------------------------------
;-----------
; The following equation is evaluated here:
;
; Where:
;  pwnadH:pwnadL   = Calculated Pulse Width without accelleration enrichment
;                    and injector deadband, in 1uS resolution
;  tpsaccel        = Accelleration Enrichment in 100uS resolution
;  fdhrH:fdhrL     = Calculated Pulse Width without injector deadband,
;                    in 1uS resolution (Fuel delivery)
;  deadband        = Injector deadband in 100uS resolution
;  pwcalch:pwcalcl = Final pulse width in 1uS resolution
;
;  pwnadH:pwnadL + (tpsaccel*100) = fdH:fdL
;
;  fdhrH:fdhrL + (deadband*100) = pwcalcH:pwcalcL
;
;***********************************************************************************************

FINAL_PW_CALC:
     lda     tpsaccel     ; Load accumulator with value in "tpsaccel"
     ldx     #$64         ; Load index register Lo byte with decimal 100
     mul                  ; Multiply (X:A <-(X)*(A)
     add     pwnadL       ; Add (A)<-(A)+(X) Calculated PW Lo byte
     sta     fdhrL        ; Copy result to Fuel delivery PW Hi Res Lo byte
     txa                  ; Transfer value in index register Lo byte to accumulator
     adc     pwnadH       ; Add with carry (A)<-(A)+(X)+(C) Calculated PW Hi byte
     sta     fdhrH        ; Copy result to Fuel Delivery PW Hi Res Hi byte
     lda     deadband     ; Load accumulator with value in "Deadband"
     ldx     #$64         ; Load index register Lo byte with decimal 100
     mul                  ; Multiply (X:A <-(X)*(A)
     add     fdhrL        ; Add (A)<-(A)+(X) Fuel Delivery PW Hi Res Lo byte
     sta     pwcalcL      ; Copy result to Final Pulse Width Lo byte
     txa                  ; Transfer value in index register Lo byte to accumulator
     adc     fdhrH        ; Add with carry (A)<-(A)+(X)+(C) Fuel Delivery Pulse Width Hi Res
                          ; Hi byte
     sta     pwcalcH      ; Copy result to Final Pulse Width Hi byte

;***********************************************************************************************
; - Divide pwcalcH:pwcalcL by 100 to produce Lo Res version ("pw") for TS and MT display,
;   and Fuel Burn Calcs
;***********************************************************************************************

     ldhx     pwcalcH          ; Load index register with value in pwcalcH:pwcalcL
     txa                       ; Transfer value in index register Lo byte to accumulator
     ldx      #$64             ; Load index register Lo byte with decimal 100
     div                       ; Divide (A)<-(H:A)/(X);(H)rem
     jsr      DIVROUND         ; Jump to "DIVROUND" subroutine(round result)
     sta      pw               ; Copy result to Final Pulse Width for display variable
                               ;(0.1mS resolution)

;***********************************************************************************************
; - Divide fdhrH:fdhrL by 100, then add to ("dbCor") variable to produce Lo Res version ("fd")
;   for MV and MT display, and Fuel Burn Calcs
;   It is assumed that even with a 0.9 ms dead band the actual deadband may be less which can be
;   tuned around, but skews the fuel burn calcultaions especially at smaller pulse widths.
;   Adding the "dbCor" value will attempt to solve this problem by making the fuel delivery
;   variable larger
;
;   8/02/12 - Eliminate "dbcor"
;
;***********************************************************************************************

     ldhx     fdhrH          ; Load index register with value in fdhrH:fdhrL
     txa                     ; Transfer value in index register Lo byte to accumulator
     ldx      #$64           ; Load index register Lo byte with decimal 100
     div                     ; Divide (A)<-(H:A)/(X);(H)rem
     jsr      DIVROUND       ; Jump to "DIVROUND" subroutine(round result)
;*     add      dbCor          ; Add A<-(A)+(M) Deadband correction
     sta      fd             ; Copy result to Fuel Delivery Lo Res variable

;***********************************************************************************************
;
; ST 3-D Table Lookup
;
;  This is used to determine value of Spark Angle minus 10 degrees ST,
;   based on RPM and MAP
;  The table looks like:
;
;     105 +....+....+....+....+....+....+....+
;         ....................................
;      00 +....+....+....+....+....+....+....+
;                    ...
;  KPA                ...
;                        ...
;      35 +....+....+....+....+....+....+....+
;         5    15   25   35   45   55   65   75 RPM/20
;
;
; Steps:
;  1) Find the bracketing KPA positions via ORD_TABLE_FIND, put index in
;      tmp8 and bounding values in tmp9(kpa1) and tmp10(kpa2)
;  2) Find the bracketing RPM positions via ORD_TABLE_FIND, store index in
;      tmp11 and bounding values in tmp13(rpm1) and tmp14(rpm2)
;  3) Using the ST table, find the table ST values for tmp15=ST(kpa1,rpm1),
;      tmp16=ST(kpa1,rpm2), tmp17 = ST(kpa2,rpm1), and tmp18 = ST(kpa2,rpm2)
;  4) Find the interpolated ST value at the lower KPA range :
;      x1=rpm1, x2=rpm2, y1=ST(kpa1,rpm1), y2=ST(kpa1,rpm2) - put in tmp19
;  5) Find the interpolated ST value at the upper KPA range :
;      x1=rpm1, x2=rpm2, y1=ST(kpa2,rpm1), y2=ST(kpa2,rpm2) - put in tmp11
;  6) Find the final ST value using the two interpolated ST values:
;      x1=kpa1, x2=kpa2, y1=ST_FROM_STEP_4, y2=ST_FROM_STEP_5
;
;***********************************************************************************************

;STTABLELOOKUP:

;ST_STEP_1:
     ldhx    #KPARANGEST_f       ; Load index register with value in KPARANGEST table
     sthx    tmp1                ; Copy value to tmp1 variable
     mov     #$0B,tmp3           ; Move decimal 12 into "tmp3"
     mov     kpa,tmp4            ; Move value in "kpa" to "tmp4"
     jsr     ORD_TABLE_FIND	   ; Jump to subroutine ORD_TABLE_FIND,(result in tmp5)
     lda     tmp1                ; Load accumulator with value in tmp1
     lda     tmp2                ; Load accumulator with value in tmp2
     mov     tmp5,tmp8		   ; move value from tmp5 to tmp8 (Index)
     mov     tmp1,tmp9		   ; move value from tmp1 to tmp9 (X1)
     mov     tmp2,tmp10		   ; move value from tmp2 to tmp10 (X2)

;ST_STEP_2:
     ldhx    #RPMRANGEST_f       ; Load index register with value from RPMRANGEST table
     sthx    tmp1                ; Copy value to tmp1 variable
     mov     #$0B,tmp3           ; Move decimal 12 into "tmp3"
     mov     rpm20,tmp4          ; Move value in "rpm20" to "tmp4"
     jsr     ORD_TABLE_FIND	   ; Jump to subroutine ORD_TABLE_FIND,(result in tmp5)
     mov     tmp5,tmp11		   ; Move value from tmp5 to tmp11 (Index)
     mov     tmp1,tmp13		   ; Move value from tmp1 to tmp13 (X1)
     mov     tmp2,tmp14		   ; Move value from tmp2 to tmp14 (X2)

;ST_STEP_3:
        clrh              ; Clear index register Hi byte
        ldx     #$0C      ; Load index register Lo byte with decimal 13
        lda     tmp8      ; Load accumulator with value in "tmp8"
        deca              ; Decrement value in accumulator
        mul               ; Multiply X:A<-(X)x(A)
        add     tmp11     ; Add "tmp11" to result A<-(A)+(M)
        deca              ; Decrement value in accumulator
        tax               ; Transfer value in accumulator to index register Lo byte
        jsr     VE2X      ; Jump to subroutine at VE2X:
        sta     tmp15     ; Copy result to "tmp15"
        incx              ; Increment value in index rgister Lo byte
        jsr     VE2X      ; Jump to subroutine at VE2X:
        sta     tmp16     ; Copy result to "tmp16"
        ldx     #$0C      ; Load index register Lo byte with decimal 13
        lda     tmp8      ; Load accumulator with value in "tmp8"
        mul               ; Multiply X:A<-(X)x(A)
        add     tmp11     ; Add "tmp11" to result A<-(A)+(M)
        deca              ; Decrement value in accumulator
        tax               ; Transfer value in accumulator to index register Lo byte
        jsr     VE2X      ; Jump to subroutine at VE2X:
        sta     tmp17     ; Copy result to "tmp17"
        incx              ; Increment value in index rgister Lo byte
        jsr     VE2X      ; Jump to subroutine at VE2X:
        sta     tmp18     ; Copy result to "tmp18"

;ST_STEP_4:
     mov     tmp13,tmp1	; Move value from tmp13 variable to tmp1 variable
     mov     tmp14,tmp2	; Move value from tmp14 variable to tmp2 variable
     mov     tmp15,tmp3	; Move value from tmp15 variable to tmp3 variable
     mov     tmp16,tmp4	; Move value from tmp16 variable to tmp4 variable
     mov     rpm20,tmp5	; Move Throttle Position Sensor ADC to tmp5 variable
     jsr     lininterp	; Jump to subroutine lininterp (result in tmp6)
     mov     tmp6,tmp19	; Move value from tmp6 variable to tmp19 variable

;ST_STEP_5:
     mov     tmp13,tmp1	; Move value from tmp13 variable to tmp1 variable
     mov     tmp14,tmp2	; Move value from tmp14 variable to tmp2 variable
     mov     tmp17,tmp3	; Move value from tmp17 variable to tmp3 variable
     mov     tmp18,tmp4	; Move value from tmp18 variable to tmp4 variable
     mov     rpm20,tmp5	; Move RPM/20 to tmp5 variable
     jsr     lininterp	; Jump to subroutine lininterp (result in tmp6)
     mov     tmp6,tmp11	; Move value from tmp6 variable to tmp11 variable

;ST_STEP_6:
     mov     tmp9,tmp1	 ; Move value from tmp9 variable to tmp1 variable
     mov     tmp10,tmp2	 ; Move value from tmp10 variable to tmp2 variable
     mov     tmp19,tmp3	 ; Move value from tmp19 variable to tmp3 variable
     mov     tmp11,tmp4	 ; Move value from tmp11 variable to tmp4 variable
     mov     kpa,tmp5	 ; Move value in MAP value in units of KPa to tmp5
     jsr     lininterp	 ; Jump to subroutine lininterp (result in tmp6)
     mov     tmp6,Spk_Ang_Fac   ; Move result to "Spk_Ang_Fac"

;*;***********************************************************************************************
;*; - MSnS351WM uses a crank triggered hall effect sensor to trigger the IRQ, as opposed to the
;*;   distributor hall effect PIP signal used in MSnS300 and MSnS460. This is done to retain
;*;   the ignition protected marine certified distributor. Static timing is still set to 10
;*;   degrees BTDC to retain the remote TFI module. The crank triggered PIP signal is 12 volt
;*;   square wave 50% duty cycle, the same as the signals used in MSnS300 and MSnS460. In case
;*;   there was an error in positioning the crank sensor, the following code is used to correct
;*;   for that error by using the variable "trigErr". A "trigErr" value of 128 commands no
;*;   correction. Values <128 will advance the timing, values >128 will retard the timing.
;*;   1 count = .353 degrees, or 1 degree = 2.83 counts.
;*;
;*;   8/02/12 - Eliminate "trigErr"
;*;
;*;***********************************************************************************************

;*     lda     trigErr        ; Load accumulator with value in "trigErr"
;*     cmp     #$80           ; Compare with decimal 128
;*     bhi     RETARD         ; If (A)>(M) branch to RETARD:
;*     blo     ADVANCE        ; If (A)<(M) branch to ADVANCE:
;*     bra     NO_TRIGERR     ; Branch to NO_TRIGERR:

;*RETARD:
;*     lda     trigErr        ; Load accumulator with value in "trigErr"
;*     sub     #$80           ; Subtract (A<-(A)-(M))(trigErr - decimal 128)
;*     add     tmp6           ; Add (A<-(A)+(M))(result + trigErr)
;*     sta     Spk_Ang_Fac    ; Copy to Spk_Ang_Fac
;*     bra     TRIGERR_DONE   ; Branch to TRIGERR_DONE:

;*ADVANCE:
;*     lda     #$80           ; Load accumulator with decimal 128
;*     sub     trigErr        ; Subtract (A<-(A)-(M))(decimal 128 - trigErr)
;*     sta     tmp1           ; Copy result to "tmp1"
;*     lda     tmp6           ; Load accumulator with value in "tmp6"(St table lookup value)
;*     sub     tmp1           ; Subtract (A<-(A)-(M))
                            ; (St table lookup value - (decimal 128 - trigErr))
;*     sta     Spk_Ang_Fac    ; Copy to Spk_Ang_Fac
;*     bra     TRIGERR_DONE   ; Branch to TRIGERR_DONE:

;*NO_TRIGERR:
;*     mov     tmp6,Spk_Ang_Fac   ; Move ST table lookup value to "Spk_Ang_Fac"

;*TRIGERR_DONE:

;***********************************************************************************************
; - Calculate the Delay Angle Factor from PIP to SPOUT
;***********************************************************************************************

     lda     Spk_Ang_Fac         ; Load acumulator with value in Spark Angle Factor variable
     add     Trm_Ang_Fac         ; Add with value in Trim Angle Factor(A<-(A)+(M))
     bcs     RAIL_DL_ANG_FAC     ; If C bit of CCR is set, branch to RAIL_DL_ANG_FAC
     sta     Dly_Ang_Fac         ; Copy Result to Delay Angle Factor var
     bra     DLY_ANG_CALC_DONE   ; Branch to DLY_ANG_CALC_DONE

RAIL_DL_ANG_FAC:
     lda     #$FF            ; Load accumultor with decimal 255 (maximum delay = 255/256*period)
     sta     Dly_Ang_Fac     ; Copy Result to Delay Angle variable

DLY_ANG_CALC_DONE:

;***********************************************************************************************
;***********************************************************************************************

LOOP_DONE:
     jmp     LOOPER     ; Jump to LOOPER: (End of Main Loop!!!)

;***********************************************************************************************
;***********************************************************************************************


;***********************************************************************************************
;
;* * * * * * * * * * * * * * * * * *   Interrupt Section * * * * * * * * * * * * * * * * * * * *
;
; NOTE!!! If the interrupt service routine modifies the H register, or uses the indexed
; addressing mode, save the H register (pshh) and then restore it (pulh) prior to exiting
; the routine
;
;***********************************************************************************************

;***********************************************************************************************
;
; -------------------- Following interrupt service routines in priority order ------------------
;
; IRQ_ISR:     - PIP input (PIP input Hi, IRQ Lo, hardware inverted)
;
; TIM2CH0_ISR: - TIM2 CH0 output compare ($0064 * 1uS) (100us Timer Tick)
;
; TIM2CH1_ISR: - TIM2 CH1 output compare (SPOUT control)
;
; SCIRCV_ISR:  - SCI receive
;
; SCITX_ISR:   - SCI transmit
;
; KYBD_ISR:    - Keyboard interrupt (Ignition Monitor)
;                from High to Low (hardware inverted)
;
; ADC_ISR:     - ADC Conversion Complete
;
;***********************************************************************************************

;***********************************************************************************************
;===============================================================================================
; - IRQ - Input trigger (PIP rising edge,IRQ falling edge, hardware inverted)
;   SPOUT signal scheduling time stamps,
;   Decrement minPIP counter.
;   Increment afterstart enrichment counter.
;   Determine RPM period.
;   Injector scheduling.
;===============================================================================================
;***********************************************************************************************

IRQ_ISR:
     pshh             ; Push value in index register Hi byte to stack


;***********************************************************************************************
; - Get 1uS Timestamp for SPOUT "on" calculations
;   T2CNTH:T2CNTL = PIP_tsH:PIP_tsL
;***********************************************************************************************

GET_TIMESTAMP:
     lda     T2CNTL      ; Load accumulator with value in TIM2 Counter Register Lo byte
                         ;(Unlatch any previous reads)
     ldx     T2CNTH      ; Load index register Lo byte with value in TIM2 Counter Register Hi
     stx     PIP_tsH     ; Copy to PIP Timestamp Hi byte variable
     lda     T2CNTL      ; Load accumulator with value in TIM2 Counter Register Lo byte
     sta     PIP_tsL     ; Copy to PIP Timestamp Lo byte variable

;***********************************************************************************************
; - Check to see if we have received a minimum of 3 PIP signals so that
;   period calculations will be valid
;***********************************************************************************************

CHECK_PIPn:
     lda   MinPIP         ; Load accumulator with value in Minimum PIP signals required for
                          ; period calculations
     beq   PIP_OK         ; If Z bit of CCR is set, branch to PIP_OK:
     dec   MinPIP         ; Decrement "MinPIP" variable
     bra   RPM_PERIOD     ; Branch to RPM_PERIOD:

PIP_OK:

;***********************************************************************************************
; - Determine the RPM period in 100uS resolution from the RPM counter values
;***********************************************************************************************

RPM_PERIOD:
     lda     RPMcH          ; Load accumulator with value in RPM counter Hi byte
     sta     RPMpH          ; Copy to RPM Period Hi byte
     lda     RPMcL          ; Load accumulator with value in RPM counter Lo byte
     sta     RPMpL          ; Copy to RPM Period Low byte
     clr     RPMcH          ; Clear RPM counter Hi byte
     clr     RPMcL          ; Clear RPM counter Lo byte

;***********************************************************************************************
; - Increment the counter for the Hi Res tachometer period averageing.
;***********************************************************************************************

     inc     tachcnt        ; Increment the tachometer averaging counter

;***********************************************************************************************
; If we are in ASE mode, and ignition cycle counter has been selected, Increment after-start
; enrichment counter
;***********************************************************************************************

     brclr   startw,engine,NO_IGN_COUNT     ; If "startw" bit of "engine" variable is clear,
                                            ; branch to NO_IGN_COUNT:
     brclr   warmup,engine,HOT_IGN          ; If "startw" bit of "engine" variable is clear,
                                            ; branch to HOT_IGN:
     lda     ASEtype           ; Load accumulator with value in ASE type configuration variable
     cmp     #$0               ; Compare it with decimal 0
     beq     DO_IGN_COUNT      ; IF Z bit of CCR is set, branch to DO_IGN_COUNT: (ASEtype = 0)
     bra     NO_IGN_COUNT      ; Branch to NO_IGN_COUNT: (ASEtype not = 0)

HOT_IGN:
     lda     ASEHtype          ; Load accumulator with value in ASE Hot type config variable
     cmp     #$0               ; Compare it with decimal 0
     beq     DO_IGN_COUNT      ; IF Z bit of CCR is set, branch to DO_IGN_COUNT: (ASEHtype = 0)
     bra     NO_IGN_COUNT      ; Branch to NO_IGN_COUNT: (ASEHtype not = 0)

DO_IGN_COUNT:
     inc    asecount

NO_IGN_COUNT:

;***********************************************************************************************
; ------------------------------------- Fuel Section -------------------------------------------
; - "divider" is the "ignition counts per injection" constant set in Tuner Studio
; - "alternate" is the "Injection Mode" constant set in Tuner Studio(0 = sim,1 = alternate)
;***********************************************************************************************

DO_FUEL:
     brset    running,engine,RUN_SET    ; If "running" bit of"engine" variable is set,
                                        ; branch to RUN_SET:
     bset     fuelp,PORTA               ; Set Fuel Pump Relay on Port A0(Turn on fuel Pump)
     bset     FPon,portAbits            ; Set "FPon" bit of "PortAbits"
     bset     running,engine            ; Set "running" bit of "engine" variable

RUN_SET:
     brset    crank,engine,SCHED_SQUIRT ; If "crank" bit of "engine" variable is set,
                                        ; branch to SCHED_SQUIRT:
                                        ; (Inject on every pulse if cranking)
     inc      igncount                  ; Increment Ignition pulse counter
     lda      igncount                  ; Load accumulator with value in Ign pulse counter
     cmp      divider                   ; Compare with Ignition counts per injection
     beq      SCHED_SQUIRT              ; If Z bit in CCR is set((A)=(M))
                                        ; branch to SCHED_SQUIRT:
     lda      igncount	                ; Load accumulator with Ignition pulse counter
     cmp      #$08                      ; Compare with decimal 8
                                        ;(The maximum allowed - reset if match)
     bne      IRQ_DONE	                ; If not equal, branch to IRQ_DONE:
     clr      igncount                  ; Clear Ignition pulse counter variable
     bra      IRQ_DONE                  ; Branch to IRQ_DONE:

SCHED_SQUIRT:
     clr      igncount                  ; Clear Ignition pulse counter variable

CRANK_CHK:
     brset     crank,engine,SCHED_BOTH   ; If crank bit of engine variable is set, branch
                                         ; to SCHED_BOTH:
     lda       alternate                 ; Load accumulator with value in "alternate"(0 or 1)
     beq       SCHED_BOTH                ; If Z bit of CCR is set(A)=(M), branch to
                                         ; SCHED_BOTH:
     inc       altcount                  ; Increment Alternate count selector variable
     brset     0,altcount,SCHED_INJ2     ; If 0 bit of "altcount" variable is set, branch to
                                         ; SCHED_INJ2:

SCHED_INJ1:
     bset    sched1,squirt     ; Set "sched1" bit of "squirt" variable
     bclr    sched2,squirt     ; Clear "sched2" bit of "squirt" variable
     bra     IRQ_DONE          ; Branch to IRQ_DONE:

SCHED_INJ2:
     bset    sched2,squirt	 ; Set "sched2" bit of "squirt" variable
     bclr    sched1,squirt     ; Clear "sched1" bit of "squirt" variable
     bra     IRQ_DONE          ; Branch to IRQ_DONE:

SCHED_BOTH:
     bset    sched1,squirt     ; Set "sched1" bit of "squirt" variable
     bset    sched2,squirt     ; Set "sched2" bit of "squirt" variable

IRQ_DONE:
     bset    piprise,inputs    ; Set "piprise" bit of "inputs" variable
     pulh                      ; Pull value from stack to index register Hi
     rti                       ; Return from interrupt


;***********************************************************************************************
;===============================================================================================
; - TIM2 CH0 Interrupt (100 uS clock tick)
; - Generate time rates:
;   100 Microseconds,(for AIOT,RPM counters, Overspeed check,Stall check, Injector firing
;                     control, Update fuel burn variables)
;   Milliseconds,(for ADC conversions)
;   100 Milliseconds,(for TPS DOT calculations and ASE counter)
;   250 Milliseconds,(for ASE counter)
;   500 Milliseconds,(for ASE counter)
;   Seconds,(for fuel burn calc variables for TS and MV and ASE counter)
;===============================================================================================
;***********************************************************************************************

;***********************************************************************************************
; - Read the current TIM2 CH0 counter value and add decimal 100 for the new
;   100uS clock tick O/C value
;***********************************************************************************************

TIM2CH0_ISR:
     pshh                  ; Push value in index register Hi byte to stack
     lda     T2SC0         ; Load accumulator with value in TIM2 CH0 Status and Control Register
                           ; (Arm CHxF flag clear)
     bclr    CHxF,T2SC0    ; Clear CHxF bit of TIM2 CH0 Status and Control Register
     ldhx    T2CH0H        ; Load index register with value in TIM2 CH0 register H:L
                           ; (output compare value)
     aix     #$64          ; Add decimal 100 (100 uS)
     sthx    T2CH0H        ; Copy result to TIM2 CH0 register(new output compare value)

;==============================================================================================
;******************************** 100 Microsecond section *************************************
;==============================================================================================

;**********************************************************************************************
; - Check the value of the AIOT trigger off counter variable, if other than zero, decrement it.
;   When it reaches zero, shut the AIOT trigger off(open collector output)
;**********************************************************************************************

     lda     aiotcntr          ; Load accumulator with value in "aiotcntr"
     beq     AIOT_CHK_DONE     ; If "Z" bit of "CCR is set, branch to AIOT_CHK_DONE:
     dec     aiotcntr          ; Decrement "aiotcntr"
     lda     aiotcntr          ; load accumulator with value in "aiotcntr"
     beq     AIOT_OFF          ; If "Z" bit of "CCR is set, branch to AIOT_OFF:
     bra     AIOT_CHK_DONE     ; Branch to AIOT_CHK_DONE:

AIOT_OFF:
     bclr    aiot,portC        ; Clear "aiot" bit of Port C (AIOT trigger off)

AIOT_CHK_DONE:

;**********************************************************************************************
; - Increment the RPM counter
;**********************************************************************************************

INC_RPM_CNTR:
     inc      rpmcl           ; Increment counter for RPM Lo byte
     bne      RPM_CNTR_DONE	; If the Z bit of the CCR is clear, branch to RPM_CNTR_DONE:
                              ;(Increment "rpmch" when "rpmcl" rolls over)
     inc      rpmch           ; Increment counter for RPM Hi byte

RPM_CNTR_DONE:
     brset   REVL,alarmbits,INC_CUS_JMP ; If "REVL" bit of "alarmbits" is set, branch to
                                        ; INC_CUS_JMP:(Over speed situation. Skip over firing
                                        ; control to slow engine down in a hurry)

;**********************************************************************************************
; - Check for stall and take appropriate action
;**********************************************************************************************

CHECK_STALL:
     lda      rpmch              ; Load accumulator with Counter for RPM Hi byte
     cmp      #$64               ; Compare with decimal 100 (If rpmch is 100
                                 ; (RPM Period = 2.56 sec) then engine not turning over)
     bne      INJ_FIRE_CNTL      ; If (A)not=(M), branch to INJ_FIRE_CNTL:
     clr      engine             ; Clear "engine" variable
                                 ;(Engine is stalled, clear all in engine)
     bclr     fuelp,PORTA        ; Clear "Fuel Pump" output Port A(Turn off fuel pump)
     bclr     FPon,portAbits     ; Clear "FPon" bit of "portAbits"
     bset     spark,PORTA        ; Set "spark" bit of Port A (SPOUT falling edge)
     clr      rpmch              ; Clear Counter for high part of RPM
     clr      rpmcl              ; Clear Low part of RPM Period
     clr      rpm20              ; Clear Computed engine RPM - rpm/20 variable
     bclr     cant_crank,engine2 ; Clear "cant_crank" bit in "engine"(OK to crank again)
     bra      INC_CUS_JMP        ; Branch to INC_CUS_JMP:
                                 ;(engine not turning over, skip Injector Firing Control)


;**********************************************************************************************
;
;=================== Injector Firing Control = Main Injector Control Logic ====================
;
; An injector is scheduled to be fired - because the other bank *may* be
; firing, we have to do a little monkey business with the timer. Here we stop
; the timer counting, but we do not reset the count to zero - this lets us
; subtract both channel's OC compare value from the current timer value and
; re-write back in. Later on, we stop the timer again (already stopped) but
; this time the count is reset to zero. Because we do the following below,
; the other channel's count is still valid for a future OC. If there is not
; a OC compare pending in the other channel, we still do this because it does
; not hurt anything and it takes less time to do the subtraction than it does
; to determine if there is a OC compare pending.....
; This code allows pulsewidths to overlap - up to 65.535ms PW
;
;**********************************************************************************************

INJ_FIRE_CNTL:
     brset   sched1,squirt,INJSTRT   ; If "sched1" bit of "squirt" variable is set,
                                     ; branch to INJSTRT:
     brset   sched2,squirt,INJSTRT   ; If "sched2" bit of "squirt" variable is set,
                                     ; branch to INJSTRT:
     jmp     INJ_FIRE_CNTL_DONE      ; Jump to INJ_FIRE_CNTL_DONE:

;**********************************************************************************************
; - Stop Timer so it can be set up - no timer clock reset
;   Subtract OC values from current timer count, and re-write back in.
;**********************************************************************************************

INJSTRT:
     mov      #$23,T1SC     ; Move %00100011 into Timer1 Status and Control Register
                            ; (Stop timer, reset timer counter, prescale for 1uS)
     lda      T1CH0H        ; Load accumulator with value in Timer1 Channel 0 Register Hi byte
     sub      T1CNTH        ;Subtract value in Timer1 Counter Register Hi (A<-(A)-(M))
     sta      T1CH0H        ; Copy to Timer1 Channel 0 Register Hi byte
     lda      T1CH0L        ; Load accumulator with value in Timer1 Channel 0 Register Lo byte
     sub      T1CNTL        ; Subtract value in Timer1 Counter Register Lo byte(A<-(A)-(M))
     sta      T1CH0L        ; Copy to Timer1 Channel 0 Register Lo bytr
     lda      T1CH1H        ; Load accumulator with value in Timer1 Channel 1 Register Hi byte
     sub      T1CNTH        ; Subtract value in Timer1 Counter Register Hi byte(A<-(A)-(M))
     sta      T1CH1H        ; Copy to Timer1 Channel 1 Register Hi byte
     lda      T1CH1L        ; Load accumulator with value in Timer1 Channel 1 Register Lo byte
     sub      T1CNTL        ; Subtract value in Timer1 Counter Register Lo byte(A<-(A)-(M))
     sta      T1CH1L        ; Copy to Timer1 Channel 1 Register Lo byte

     brset   sched1,squirt,NEW_SQUIRT1  ; If "sched1" bit of "squirt" is set, branch to
                                        ; NEW_SQUIRT1:

INJF1:
     brset   sched2,squirt,NEW_SQUIRT2  ; If "sched2" bit of "squirt" is set, branch to
                                        ; NEW_SQUIRT2:

INJF2:
     bra     INJ_FIRE_CNTL_DONE         ; Branch to INJ_FIRE_CNTL_DONE:


;**********************************************************************************************
;=========================== Injector #1 - Start New Injection ================================
;**********************************************************************************************

NEW_SQUIRT1:
     bclr    sched1,squirt     ; Clear "sched1" bit of "squirt"(is now current operation)
     mov     #$33,T1SC         ; Move %00110011 into TIM1 Status and Control Register
                               ; (Disable interrupts, stop timer, reset counter,
                               ; prescale for 1uS)
     mov     pwcalcH,T1CH0H    ; Move value in Final Pulse Width Hi byte
                               ; to TIM1 CHO O/C Register Hi byte
     mov     pwcalcL,T1CH0L    ; Move value in Final Pulse Width Lo byte
                               ; to TIM1 CHO O/C Register Lo byte
                               ; pulsewidth to TIM1 CH0 register Lo byte
     mov     #$10,T1SC0        ; Move %00010000 into TIM1 CH0 Status and Control Register
                               ; (Make this normal port output init logic 1)
     mov     #$1C,T1SC0        ; Move %00011100 into TIM1 CH0 Status and Control register
                               ; (Disable interupts,set output on compare)
     mov     #$03,T1SC         ; Move %00000011 into TIM1 Status and Control Register
                               ; (Disable interrupts, start timer, prescale for 1uS)

;***********************************************************************************************
; - Update Fuel Delivery Pulse Width Lo Res Total so the results can be used by TS and MV to
;   calculate current fuel burn.
;***********************************************************************************************

     lda     fd        ; Load accumulator with value in Fuel Delivery PW Lo Res
     add     fdtL      ; Add (A)<-(A)+(M) (Fuel Delivery PW Lo Res + Fuel
                       ; Delivery Pulse width Lo Res total Lo byte)
     sta     fdtL      ; Copy result to "fdtL" (update fdtL)
     lda     fdtH      ; Load accumulator with value in Fuel Delivery Pulse
                       ; width Lo Res Total Hi byte
     adc     #$00      ; Add with carry (A)<-(A)+(M)+(C) (just the carry)
     sta     fdtH      ; Copy result to "fdtH" (update fdtH)

;***********************************************************************************************
; - Update the Fuel Delivery Lo Res counter so that on roll over a pulsed signal can be sent
;   to the totalizer(open collector output)
;***********************************************************************************************

     lda     fd               ; Load accumulator with value in "fd"
     add     fdcntr           ; Add (A)<-(A)+(M)
     bcs     TOTALIZER1       ; If the carry bit of CCR is set, branch to TOTALIZER1:
     sta     fdcntr           ; Copy to "fdcntr"
     bra     TOTALIZER_DONE1  ; Branch to TOTALIZER_DONE1:

TOTALIZER1:
     sta     fdcntr           ; Copy to "fdcntr"
     bset    aiot,portC       ; Set "aiot" bit of port C(Totalizer trigger on)
     lda     #$1E             ; Load accumulator with decimal 30(3mS)
     sta     aiotcntr         ; Copy to "aiotcntr"(3mS on time)

TOTALIZER_DONE1:
     bra     INJF1            ; Branch to INJFI:

;***********************************************************************************************
; - Long branch for INC_CUS:
;***********************************************************************************************

INC_CUS_JMP:
     bra     INC_CUS     ; Branch to INC_CUS_:

;***********************************************************************************************
;============================= Injector #2 - Start New Injection ===============================
;***********************************************************************************************

NEW_SQUIRT2:
     bclr    sched2,squirt     ; Clear "sched2" bit of "squirt"(is now current operation)
     mov     #$33,T1SC         ; Move %00110011 into TIM1 Status and Control Register
                               ;(Disable interrupts, stop timer, reset counter,
                               ; prescale for 1uS)
     mov     pwcalcH,T1CH1H    ; Move value in Final Pulse Width Hi byte
                               ; to TIM1 CH1 O/C Register Hi byte
     mov     pwcalcL,T1CH1L    ; Move value in Final Pulse Width Lo byte
                               ; to TIM1 CH1 O/C Register Lo byte
                               ; pulsewidth to TIM1 CH0 register Lo byte
     mov     #$10,T1SC1        ; Move %00010000 into TIM1 CH1 Status and Control Register
                               ;(Make this normal port output init logic 1
     mov     #$1C,T1SC1        ; Move %00011100 into TIM1 CH1 Status and Control register
                               ;(Disable interupts,set output on compare)
     mov     #$03,T1SC         ; Move %00000011 into TIM1 Status and Control Register
                               ; (Disable interrupts, start timer, prescale for 1uS)


;***********************************************************************************************
; - Update Fuel Delivery Pulse Width Lo Res Total so the results can be used by TS and MV to
;   calculate current fuel burn.
;***********************************************************************************************

     lda     fd        ; Load accumulator with value in Fuel Delivery PW Lo Res
     add     fdtL      ; Add (A)<-(A)+(M) (Fuel Delivery PW Lo Res + Fuel
                       ; Delivery Pulse width Lo Res total Lo byte)
     sta     fdtL      ; Copy result to "fdtL" (update fdtL)
     lda     fdtH      ; Load accumulator with value in Fuel Delivery Pulse
                       ; width Lo Res Total Hi byte
     adc     #$00      ; Add with carry (A)<-(A)+(M)+(C) (just the carry)
     sta     fdtH      ; Copy result to "fdtH" (update fdtH)

;***********************************************************************************************
; - Update the Fuel Delivery Lo Res counter so that on roll over a pulsed signal can be sent
;   to the totalizer(open collector output)
;***********************************************************************************************

     lda     fd               ; Load accumulator with value in "fd"
     add     fdcntr           ; Add (A)<-(A)+(M)
     bcs     TOTALIZER2       ; If the carry bit of CCR is set, branch to TOTALIZER2:
     sta     fdcntr           ; Copy to "fdcntr"
     bra     TOTALIZER_DONE2  ; Branch to TOTALIZER_DONE2:

TOTALIZER2:
     sta     fdcntr           ; Copy to "fdcntr"
     bset    aiot,portC       ; Set "aiot" bit of port C(Totalizer trigger on)
     lda     #$1E             ; Load accumulator with decimal 30(3mS)
     sta     aiotcntr         ; Copy to "aiotcntr"(3mS on time)

TOTALIZER_DONE2:
     bra     INJF2            ; Branch to INJF2:

INJ_FIRE_CNTL_DONE:

;***********************************************************************************************
; - Increment 100 Microsecond counter
;***********************************************************************************************

INC_CUS:
     inc     uSx100            ; Increment 100 Microsecond counter
     lda     uSx100            ; Load accumulator with 100 Microsecond counter
     cmp     #$0A              ; Compare it with decimal 10
     bne     TIM2CH0_ISR_JMP   ; If not equal, branch to TIM2CH0_ISR_JMP:

;===============================================================================================
;********************************* millisecond section *****************************************
;===============================================================================================


;***********************************************************************************************
; - Fire off another ADC conversion, channel is pointed to by "adsel"
;***********************************************************************************************

FIRE_ADC:
     lda     adsel          ; Load accumulator with ADC Selector Variable
     ora     #%01000000     ; Inclusive "or" with %01000000 and ADC Selector
                            ; Variable ( result in accumulator )
                            ;(Enables interupt with channel selected)
     sta     adscr          ; Copy result to ADC Status and Control Register

;***********************************************************************************************
; - Increment millisecond counter
;***********************************************************************************************

INC_mS:
     clr     uSx100              ; Clear 100 Microsecond counter
     inc     mS                  ; Increment Millisecond counter
     lda     mS                  ; Load accumulator with value in Millisecond counter
     cmp     #$64                ; Compare it with decimal 100
     beq     DO_100MS            ; IF Z bit of CCR is set, branch to DO_100MS: (mS=100)
     cmp     #$C8                ; Compare it with decimal 200
     beq     DO_100MS            ; IF Z bit of CCR is set, branch to DO_100MS: (mS=200)
     cmp     #$FA                ; Compare it with decimal 250
     beq     DO_250MS            ; IF Z bit of CCR is set, branch to DO_250MS: (mS=250)
     bra     TIM2CH0_ISR_JMP     ; Branch to TIM2CH0_ISR_JMP:

;===============================================================================================
;*********************************** 100 Millisecond section ***********************************
;===============================================================================================

DO_100MS:

;**********************************************************************************************
; If we are in ASE mode, and 100 ms counter has been selected, Increment after-start
; enrichment counter
;**********************************************************************************************

     brclr   startw,engine,NO_100MS_COUNT   ; If "startw" bit of "engine" variable is clear,
                                            ; branch to NO_100MS_COUNT:
     brclr   warmup,engine,HOT_100MS        ; If "startw" bit of "engine" variable is clear,
                                            ; branch to HOT_100MS:
     lda     ASEtype           ; Load accumulator with value in ASE type configuration variable
     cmp     #$1               ; Compare it with decimal 1
     beq     DO_100MS_COUNT    ; IF Z bit of CCR is set, branch to DO_100MS_COUNT: (ASEtype = 1)
     bra     NO_100MS_COUNT    ; Branch to NO_100MS_COUNT: (ASEtype not = 1)

HOT_100MS:
     lda     ASEHtype          ; Load accumulator with value in ASE Hot type config variable
     cmp     #$1               ; Compare it with decimal 1
     beq     DO_100MS_COUNT    ; IF Z bit of CCR is set, branch to DO_100MS_COUNT: (ASEHtype = 1)
     bra     NO_100MS_COUNT    ; Branch to NO_100MS_COUNT: (ASEHtype not = 1)

DO_100MS_COUNT:
     inc    asecount

NO_100MS_COUNT:

;***********************************************************************************************
; - Increment "tpsaclk" counter for TPS DOT calculations
;***********************************************************************************************

     inc     tpsaclk     ; Increment "tpsaclk" counter

;***********************************************************************************************
; - Save current TPS reading in last_tps variable to compute TPSDOT in
;   acceleration enrichment section
;***********************************************************************************************

     lda     tps                 ; Load accumulator with value in TPS ADC
     sta     last_tps            ; Copy to last TPS variable
     bset    clock100,inputs     ; Set "clock100" bit of "inputs" variable
     bra     TIM2CH0_ISR_JMP     ; Branch to TIM2CH0_ISR_JMP:

;===============================================================================================
;********************************* 250 Millisecond section *************************************
;===============================================================================================

DO_250MS:
     bset    clock250,inputs     ; Set "clock250" bit of "inputs" variable

;**********************************************************************************************
; If we are in ASE mode, and 250 ms counter has been selected, Increment after-start
; enrichment counter
;**********************************************************************************************

     brclr   startw,engine,NO_250MS_COUNT   ; If "startw" bit of "engine" variable is clear,
                                            ; branch to NO_250MS_COUNT:
     brclr   warmup,engine,HOT_250MS        ; If "startw" bit of "engine" variable is clear,
                                            ; branch to HOT_250MS:
     lda     ASEtype           ; Load accumulator with value in ASE type configuration variable
     cmp     #$2               ; Compare it with decimal 2
     beq     DO_250MS_COUNT    ; IF Z bit of CCR is set, branch to DO_250MS_COUNT: (ASEtype = 2)
     bra     NO_250MS_COUNT    ; Branch to NO_250MS_COUNT: (ASEtype not = 2)

HOT_250MS:
     lda     ASEHtype          ; Load accumulator with value in ASE Hot type config variable
     cmp     #$2               ; Compare it with decimal 2
     beq     DO_250MS_COUNT    ; IF Z bit of CCR is set, branch to DO_250MS_COUNT: (ASEHtype = 2)
     bra     NO_250MS_COUNT    ; Branch to NO_250MS_COUNT: (ASEHtype not = 2)

DO_250MS_COUNT:
     inc    asecount

NO_250MS_COUNT:

;***********************************************************************************************
; - Increment 250 Millisecond counter and check to see if it's time to do the "500mS" section
;***********************************************************************************************

     clr     mS                  ; Clear Millisecond counter
     inc     mSx250              ; Increment 250 Millisecond counter
     lda     mSx250              ; Load accumulator with value in 250 Millisecond counter
     cmp     #$02                ; Compare with decimal 2
     beq     DO_500MS            ; If the Z bit of CCR is set, branch to DO_500MS:
     cmp     #$04                ; Compare with decimal 4
     beq     DO_500MS            ; If the Z bit of CCR is set,branch to DO_500MS:
     bra     TIM2CH0_ISR_DONE    ; Branch to TIM2CH0_ISR_DONE:

;***********************************************************************************************
; - Long branch for TIM2CH0_ISR_DONE:
;***********************************************************************************************

TIM2CH0_ISR_JMP:
     bra     TIM2CH0_ISR_DONE    ; Branch to TIM2CH0_ISR_DONE:

;==============================================================================================
;********************************* 500 Millisecond section ************************************
;==============================================================================================

DO_500MS:
;*     bset    clock500,inputs     ; Set "clock500" bit of "inputs" variable

;**********************************************************************************************
; If we are in ASE mode, and 500 ms counter has been selected, Increment after-start
; enrichment counter
;**********************************************************************************************

     brclr   startw,engine,NO_500MS_COUNT   ; If "startw" bit of "engine" variable is clear,
                                            ; branch to NO_500MS_COUNT:
     brclr   warmup,engine,HOT_500MS        ; If "startw" bit of "engine" variable is clear,
                                            ; branch to HOT_500MS:
     lda     ASEtype           ; Load accumulator with value in ASE type configuration variable
     cmp     #$3               ; Compare it with decimal 3
     beq     DO_500MS_COUNT    ; IF Z bit of CCR is set, branch to DO_500MS_COUNT: (ASEtype = 3)
     bra     NO_500MS_COUNT    ; Branch to NO_500MS_COUNT: (ASEtype not = 3)

HOT_500MS:
     lda     ASEHtype          ; Load accumulator with value in ASE Hot type config variable
     cmp     #$3               ; Compare it with decimal 3
     beq     DO_500MS_COUNT    ; IF Z bit of CCR is set, branch to DO_500MS_COUNT: (ASEHtype = 3)
     bra     NO_500MS_COUNT    ; Branch to NO_500MS_COUNT: (ASEHtype not = 3)

DO_500MS_COUNT:
     inc    asecount

NO_500MS_COUNT:

;***********************************************************************************************
; - Check to see if it's time to do the "Seconds" section
;***********************************************************************************************

     lda     mSx250              ; Load accumulator with value in 250 Millisecond counter
     cmp     #$04                ; Compare with decimal 4
     beq     Do_Sec              ; If the Z bit of CCR is set,  branch to DO_Sec:
     bra     TIM2CH0_ISR_DONE    ; Branch to TIM2CH0_ISR_DONE:

;===============================================================================================
;************************************* Seconds section *****************************************
;===============================================================================================

Do_Sec:

;**********************************************************************************************
; If we are in ASE mode, and Seconds counter has been selected, Increment after-start
; enrichment counter
;**********************************************************************************************

     brclr   startw,engine,NO_SEC_COUNT   ; If "startw" bit of "engine" variable is clear,
                                          ; branch to NO_SEC_COUNT:
     brclr   warmup,engine,HOT_SEC        ; If "startw" bit of "engine" variable is clear,
                                          ; branch to HOT_SEC:

     lda     ASEtype           ; Load accumulator with value in ASE type configuration variable
     cmp     #$4               ; Compare it with decimal 4
     beq     DO_SEC_COUNT      ; IF Z bit of CCR is set, branch to DO_SEC_COUNT: (ASEtype = 4)
     bra     NO_SEC_COUNT      ; Branch to NO_SEC_COUNT: (ASEtype not = 4)

HOT_SEC:
     lda     ASEHtype          ; Load accumulator with value in ASE Hot type config variable
     cmp     #$4               ; Compare it with decimal 4
     beq     DO_SEC_COUNT      ; IF Z bit of CCR is set, branch to DO_SEC_COUNT: (ASEHtype = 4)
     bra     NO_SEC_COUNT      ; Branch to NO_SEC_COUNT: (ASEHtype not = 4)

DO_SEC_COUNT:
     inc    asecount

NO_SEC_COUNT:

;***********************************************************************************************
; - Save the current fuel delivery lo res total as "fdSecH:fdSecL" so it can be used by TS
;   and MV in fuel burn calculations
;***********************************************************************************************

     lda     fdtL       ; Load accumulator with value in "fdtL"
     sta     fdSecL     ; Copy to "fdSecL"
     lda     fdtH       ; Load accumulator with value in "fdtH"
     sta     fdSecH     ; Copy to "fdSecH"
     clr     fdtL       ; Clear "fdtL"
     clr     fdtH       ; Clear "fdtH"

;***********************************************************************************************
; - Crank Mode Inhibit. Make a 1 to 2 second delay after engine starts. Then inhibit crank
;    mode unless engine completely stalls. This is to prohibit re-entry to crank mode under
;    abnormally low idle RPMs
;    * If running and !cranking and !cant_delay, then set cant_delay
;    * If running and !cranking and cant_delay, then set cant_crank
;    * Else clear cant_delay
;***********************************************************************************************

     brset   crank,engine,CANT_OFF        ; If "crank" bit of "engine" is set, branch to
                                          ; CANT_OFF
     brclr   running,engine,CANT_OFF      ; If "running" bit of "engine" variable is clear,
                                          ; branch to CANT_OFF:
     brset   cant_delay,engine2,CANT_SET  ; If "cant_delay" bit of "engine2" is set, branch to
                                          ; CANT_SET:
     bset    cant_delay,engine2           ; Set "cant_off" bit of "engine2"(set timer bit)
     bra     INC_SEC                     ; Branch to INC_SEC:

CANT_SET:
     bset    cant_crank,engine2     ; Set "cant_crank" bit of "engine2"(inhibit crank mode)
     bra     INC_SEC                ; Branch to SEC_DONE:

CANT_OFF:
     bclr    cant_delay,engine2     ; Clear "cant_delay" bit of "engine2"(clear timer bit)

;***********************************************************************************************
; - Increment Seconds counter
;***********************************************************************************************

INC_SEC:
     clr     mSx250              ; Clear 0.25 Second variable
     inc     secl                ; Increment "Seconds" Lo byte variable
     bne     TIM2CH0_ISR_DONE    ; If the Z bit of CCR is clear, branch to TIM2CH0_ISR_DONE:
     inc     sech                ; Increment "Seconds" Hi byte variable

TIM2CH0_ISR_DONE:
     pulh                        ; Pull value from stack to index register Hi byte
     rti                         ; Return from interrupt



;****************************************************************************
;
; ----------------- T2SC1 Interrupt (SPOUT Control) --------------------------
;
;****************************************************************************


;****************************************************************************
; - TIM2 CH1 interrupts twice per ignition event. The first is SPOUT rising
;   edge. This is the delay from PIP rising edge to SPOUT rising edge, which
;   signals the TFI module to fire the coil. TIM2 CH1 was armed and set with
;   the delay,(O/C value) in the main loop after the preceding ignition
;   sequence was finished.
;   In this first interrupt, the SPOUT signal is driven Hi, and the interrupt
;   flag is cleared. The TIM2 counter value is read to get a new timestamp
;   to calculate the new O/C value for the next interrupt which will be to
;   generate the SPOUT falling edge.
;   When the second interrupt is received, the SPOUT signal is driven Lo,
;   the interrupt flag is cleared, and the O/C interrupts are disabled.
;   This prepares TIM2 CH1 to be armed and set with the new O/C value for
;   rising SPOUT, which has already been calculated in the main loop.
;****************************************************************************

TIM2CH1_ISR:
     pshh                    ; Push value in index register hi byte to stack
     brclr   sparking,engine2,SET_LO_OC   ;If "sparking" bit of "engine2"
                                ; variable is clear, branch to SET_LO_OC:
     lda     T2SC1              ; Load accumulator with value in
                                ; TIM2 CH1 Status and Control Register
                                ;(Arm CHxF flag clear)
     bclr    CHxF,T2SC1         ; Clear CHxF bit of TIM2 CH1 Status and
                                ; Control Register (clear interrupt flag)
     bclr    CHxIE,T2SC1        ; Disable interrupt requests TIM2 channel 1
                                ; Status and Control Register
     bset    spark,PORTA        ; Set "spark" bit of Port A
                                ;(SPOUT falling edge)(Idle LED on)
     bclr    sparking,engine2   ; Clear "sparking" bit of "engine2" variable
     bra     IGN_SEQ_DONE       ; Branch to IGN_SEQ_DONE:

SET_LO_OC:

;****************************************************************************
; - Get 1.0uS Timestamp for SPOUT "0ff" O/C calculations
;   T2CNTH:T2CNTL = SPOUT_tsH:SPOUT_tsL
;****************************************************************************

     lda     T2CNTL      ; Load accumulator with value in TIM2
                         ; Counter Register Lo byte
                         ;(Unlatch any previous reads)
     ldx     T2CNTH      ; Load index register Lo byte with value in
                         ; TIM2 Counter Register Hi byte
     stx     SPOUT_tsH   ; Copy to SPOUT Timestamp Hi byte variable
     lda     T2CNTL      ; Load accumulator with value in TIM2
                         ; Counter Register Lo byte
     sta     SPOUT_tsL   ; Copy to SPOUT Timestamp Lo byte variable

;****************************************************************************
; - Calculate the output compare value for delay from SPOUT rising edge to
;   SPOUT falling edge.
;   SPOUT_tsH:SPOUT_tsL + SPOUTon_pH:SPOUTon_pL = CH1_offH:CH1_offL
;****************************************************************************

CALC_OFF_OC:
     lda     SPOUT_tsL      ; Load accumulator with value in "SPOUT_tsL"
     add     SPOUTon_pL     ; Add without Carry A<-(A)+(M)
                            ;(SPOUT_tsL + SPOUTon_pL)
     tax                    ; Transfer result in accumulator to index
                            ; register Lo byte
     lda     SPOUT_tsH      ; Load accumulator with value in "PIP_tsH"
     adc     SPOUTon_pH     ; Add with Carry A<-(A)+(M)
     sta     CH1_offH       ; Copy to CH1 off O/C value Hi byte
     stx     CH1_offL       ; Copy to CH1 off O/C value Lo byte

;****************************************************************************
; - Set and arm TIM2 Chan 1 output compare value to drive SPOUT Lo
;   CH1_offH:CH1_offL = T2CH1H:T2CH1L
;****************************************************************************

OFF_DELAY:
     lda     T2SC1          ; Load accumulator with value in
                            ; TIM2 CH1 Status and Control Register
                            ;(Arm CHxF flag clear)
     bclr    CHxF,T2SC1     ; Clear CHxF bit of TIM2 CH1 Status and
                            ; Control Register (clear interrupt flag)
     clrh                   ; Clear index register Hi byte
     ldhx    CH1_offH       ; Load index register with value in CH1 off
                            ; O/C value H:L
     sthx    T2CH1H         ; Copy to TIM2 CH1 register H:L
                            ;(output compare value for SPOUT rising edge)

;****************************************************************************
; - Drive SPOUT Hi to command TFI module to fire coil.
;****************************************************************************

     bclr    spark,PORTA        ; Clear "spark" bit of Port A
                                ;(SPOUT rising edge)(idle LED off)
     bset    sparking,engine2   ; Set "sparking" bit of "engine2" variable


IGN_SEQ_DONE:
     pulh                  ; Pull value from stack to index register Hi byte
     rti                   ; Return from interrupt routine



;***********************************************************************************************
;
; -------------------- Serial Communications Interface ----------------------
;
; Communications is established when the PC communications program sends
; a command character - the particular character sets the mode:
;
; "A" = send all of the realtime variables via txport.(Output Channel Get Command)
; "V" = send the VE table and constants via txport (128 bytes)(Page Read Command)
; "W"+<offset>+<newbyte> = receive new VE or constant byte value and store in offset location
;                          (Page value(byte)write command)
; "X"+<offset>+<count>+<newbyte>+<newbyte>... = receive series of new data bytes(chunk write)
; "B" = jump to flash burner routine and burn VE/constant values in RAM into flash
; "C" = Test communications - echo back SECL
; "Q" = Send over Embedded Code Revision Number (irrelevant in Extra, send zero)
; "S" = Signature - update every time there is a change in data format
; "T" = full code revision in text. 32 bytes
; "P"+<page> = Load page of data from Flash to RAM(page select(activate))
;
; txmode:
;              01 = Getting realtime data
;              02 = ?
;              03 = Sending VE
;              04 = Sending Signature
;              05 = Getting offset VE
;              06 = Getting data VE
;              07 = Getting offset chunk write
;              08 = Getting count  chunk write
;              09 = Getting data   chunk write
;              0C = getting table number
;              0E = format string
;
; NOTE!!! txgoal = number of bytes to send + 1
;
;***************************************************************************

SCIRCV_ISR:
     pshh                 ; Push value in index register Hi byte to Stack
     lda     SCS1         ; Load accumulator with value in "SCS1"
                          ;(Clear the SCRF bit by reading this register)
     lda     txmode       ; Load accumulator with value in "txmode" variable
                          ;(Check if we are in the middle of a receive
                          ; new VE/constant)
     cmp     #$05         ; Compare with decimal 5
     beq     TXMODE_5     ; If the Z bit of CCR is set, branch to TXMODE_5:
     cmp     #$06         ; Compare with decimal 6
     beq     TXMODE_6     ; If the Z bit of CCR is set, branch to TXMODE_6:
     cmp     #$07         ; Compare with decimal 7
     beq     TXMODE_7     ; If the Z bit of CCR is set, branch to TXMODE_7:
     cmp     #$08         ; Compare with decimal 8
     beq     TXMODE_8     ; If the Z bit of CCR is set, branch to TXMODE_8:
     cmp     #$09         ; Compare with decimal 8
     beq     TXMODE_9     ; If the Z bit of CCR is set, branch to TXMODE_8:
     cmp     #$0C         ; Compare with decimal 12
     beq     TXMODE_C1    ; If the Z bit of CCR is set, branch to TXMODE_C1:
     bra     CHECK_TXCMD  ; Branch to CHECK_TXCMD:

TXMODE_C1:
     jmp     TXMODE_C

TXMODE_5:
     mov     SCDR,rxoffset   ; Move value in "SCDR" to "rxoffset"
     inc     txmode          ; (continue to next mode)
     jmp     DONE_RCV        ; Jump to DONE_RCV:

TXMODE_6:
     clrh                 ; Clear index register Hi byte
     lda     SCDR         ; Load accumulator with value in "SCDR"
     ldx     rxoffset     ; Load index register Lo byte with value in "rxoffset"
     sta     VE_r,x         ; Copy to VE table, offset in index register Lo byte
     clr     txmode       ; Clear "txmode" variable
     jmp     DONE_RCV     ; Jump to DONE_RCV:

TXMODE_7:
     mov     SCDR,rxoffset   ; Move value in "SCDR" to "rxoffset"
     inc     txmode          ; (continue to next mode)
     jmp     DONE_RCV        ; Jump to DONE_RCV:

TXMODE_8:
     mov     SCDR,txgoal   ; Move value in "SCDR" to "txgoal"
     inc     txmode          ; (continue to next mode)
     jmp     DONE_RCV        ; Jump to DONE_RCV:

TXMODE_9:
     clrh
     lda     SCDR
     ldx     rxoffset
     sta     VE_r,x
     inc     rxoffset
     dec     txgoal
     bne     TXMODE_9_CONT
     clr     txmode

TXMODE_9_CONT:
     jmp     DONE_RCV

CHECK_TXCMD:
     lda     SCDR       ; Load accumulator with value in "SCDR" (Get the command byte)
     cmp     #$41       ; Compare it with decimal 65 = ASCII "A"
     beq     MODE_A     ; If the Z bit of CCR is set, branch to Mode_A:
     cmp     #$42       ; Compare it with decimal 66 = ASCII "B"
     beq     MODE_B     ; If the Z bit of CCR is set, branch to Mode_B:
     cmp     #$43       ; Compare it with decimal 67 = ASCII "C"
     beq     MODE_C     ; If the Z bit of CCR is set, branch to Mode_C:
     cmp     #$56       ; Compare it with decimal 86 = ASCII "V"
     beq     MODE_V     ; If the Z bit of CCR is set, branch to Mode_V:
     cmp     #$57       ; Compare it with decimal 87 = ASCII "W"
     beq     MODE_W     ; If the Z bit of CCR is set, branch to Mode_W:
     cmp     #$51       ; Compare it with decimal 81 = ASCII "Q"
     beq     MODE_Q     ; If the Z bit of CCR is set, branch to Mode_Q:
     cmp     #$50       ; Compare it with decimal 80 = ASCII "P"
     beq     MODE_P     ; If the Z bit of CCR is set, branch to Mode_P:
     cmp     #$53       ; Compare it with decimal 83 = ASCII "S"
     beq     MODE_S     ; If the Z bit of CCR is set, branch to Mode_S:
     cmp     #$52       ; Compare it with decimal 82 = ASCII "R"
     beq     MODE_R     ; If the Z bit of CCR is set, branch to Mode_R:
     cmp     #$58       ; Compare it with decimal 88 = ASCII "X"
     beq     MODE_X     ; If the Z bit of CCR is set, branch to Mode_X:
     cmp     #$54       ; Compare it with decimal 84 = ASCII "T"
     beq     MODE_T     ; If the Z bit of CCR is set, branch to Mode_T:
     jmp     DONE_RCV

MODE_R:
MODE_A:
     lda     #$2A           ; Load accumulator with decimal 42(41 real time variables for
                            ; MV and TS)(Set this for the number of bytes to send +1)
     sta     txgoal         ; Copy to "txgoal" variable
     clr     txcnt          ; Clear "txcnt"
     lda     #$01           ; Load accumulator with decimal 1
     bra     EN_XMIT

MODE_B:
     lda     page
     bclr    SCRIE,SCC2
     mov     #$CC,flocker
     jsr     burnConst     ; Jump to "burnConst" subroutine
     clr     flocker
     clr     txmode        ; Clear "txmode" variable
     lda     page
     bset    SCRIE,SCC2
     bra     DONE_RCV      ; Branch to DONE_RCV:

MODE_C:
     clr     txcnt          ; Clear "txcnt" (Just send back SECL variable to test comm port)
     lda     #$01           ; Load accumulator with decimal 1
     sta     txgoal         ; Copy to "txgoal" variable
     lda     #$01           ; Load accumulator with decimal 1
     bra     EN_XMIT

MODE_P:
     mov     #$0C,txmode
     bra     DONE_RCV

MODE_Q:
     clr     txcnt          ; Clear "txcnt"(Just send back SECL variable to test comm port)
     lda     #$01           ; Load accumulator with decimal 1
     sta     txgoal         ; Copy to "txgoal" variable
     lda     #$05           ; Load accumulator with decimal 5
     bra     EN_XMIT

MODE_V:
     clr     txcnt          ; Clear "txcnt"
     mov     #PAGESIZE,txgoal
     lda     page
     lda     #$03           ; Load accumulator with decimal 3
     bra     EN_XMIT

MODE_W:
     lda     #$05         ; Load accumulator with decimal 5
     sta     txmode       ; Copy to "txmode" variable
     bra     DONE_RCV     ; Branch to DONE_RCV:

MODE_X:
     mov     #$07,txmode
     bra     DONE_RCV

MODE_T:
     clr     txcnt
     mov     #$20,txgoal
     lda     #$0E
     bra     EN_XMIT

MODE_S:
     clr     txcnt
     mov     #$20,txgoal
     lda     #$04

EN_XMIT:
     sta     txmode         ; Copy to "txmode" variable
     bset    TE,SCC2        ; Set "TE" bit of SCC2(Enable Transmit)
     bset    SCTIE,SCC2     ; Set "SCTIE" bit of SCC2(Enable transmit interrupt)

DONE_RCV:
     pulh                 ; Pull value from Stack to index register Hi byte
     rti                  ; Return from interrupt

;****************************************************************************
;----------------- Transmit Character Interrupt Handler --------------------
;****************************************************************************

SCITX_ISR:
     pshh                  ; Push value in index register Hi byte to Stack
     lda     SCS1          ; Load accumulator with value in "SCS1"
                           ; (Clear the SCRF bit by reading this register)
     clrh                  ; Clear index register Hi byte
     lda     txcnt         ; Load accumulator with value in "txcnt" variable
     tax                   ; Transfer value in accumulator to index register
                           ; Lo byte
     lda     txmode        ; Load accumulator with value in "txmode" variable
     beq     TX_DONE
     cmp     #$05          ; Compare it with decimal 5
     beq     IN_Q_MODE     ; If the Z bit of CCR is set, branch to IN_Q_MODE:
     cmp     #$04
     beq     IN_S_MODE
     cmp     #$0E
     beq     IN_T_MODE
     cmp     #$01          ; Compare it with decimal 1
     beq     IN_A_OR_C_MODE
     cmp     #$03          ; Compare it with decimal 3
     beq     IN_V_MODE
     bra     TX_DONE


IN_A_OR_C_MODE:
     lda     secl,X      ; Load accumulator with value in address "secl",
                         ; offset in index register Lo byte
     bra     CONT_TX     ; Branch to CONT_TX:

IN_V_MODE
     lda     page
     lda     VE_r,x        ; Load accumulator with value in address "ve",
                         ; offset in index register Lo byte
     bra     CONT_TX     ; Branch to CONT_TX:

IN_T_MODE:
     lda     textversion_f,x
     bra     CONT_TX

IN_S_MODE:
     lda     Signature,x
     bra     CONT_TX     ; Branch to CONT_TX:

IN_Q_MODE
     lda     REVNUM,X   ; Load accumulator with value in address "REVNUM",
                        ; offset in index register Lo byte

CONT_TX:
     sta     SCDR           ; Copy to "SCDR" variable (Send char)
     lda     txcnt          ; Load accumulator with value in "txcnt" variable
     inca                   ; Increment value in accumulator
                            ;(Increase number of chars sent)
     sta     txcnt          ; Copy to "txcnt" variable
     cmp     txgoal         ; Compare it to value in "txgoal" (Check if done)
     bne     DONE_BYTE      ; If the Z bit of CCR is clear, branch to DONE_BYTE:
                            ;(Branch if NOT done to DONE_XFER !?!?!)

FIN_TX:
     clr     txcnt          ; Clear "txcnt"
     clr     txgoal         ; Clear "txgoal"
     clr     txmode         ; Clear "txmode"

DONE_BYTE:
     pulh                   ; Pull value from Stack to index register Hi byte
     rti                    ; Return from interrupt

TXMODE_C:
     lda     SCDR          ; expect 1 to 3
     cmp     page          ; check if already loaded
     beq     DONE_LOAD
     clrx
     sta     page
     add     #$DF
     psha
     pulh
     bclr     SCRIE,SCC2     ; Turn off receive interrrupt so we don't re-enter
     cli                     ; Re-enable interrupts

LOAD_TABLE:
     lda     0,x             ; h:x
     pshh
     clrh
     sta     VE_r,x          ; Dump into RAM
     pulh
     incx
     cpx     #PAGESIZE+1
     bne     LOAD_TABLE

DONE_LOAD:
     bset    SCRIE,SCC2
     clr     txmode
     pulh
     rti

TX_DONE:                  ; We get here after we've sent the last byte
     bclr     TE,SCC2
     bclr     SCTIE,SCC2
     pulh                 ; Pull value from Stack to index register Hi byte
     rti                  ; Return from interrupt


;***********************************************************************************************
;
; ---------------------------- Interrupt for ADC conversion complete ---------------------------
;
; ADC channel is set by "adsel" variable which starts at 0. This reads channel 0, which is
; "map". When the conversion complete interrupt is requested the current value in the ADC Data
; Register (ADR) is stored as current "map" value. The "adsel" variable is then incremented to
; the next channel and the process repeats until channel 7 is requested. Channel 7 is the
; multiplexer channel which is controlled by "adsel2" variable which starts at 0, This reads
; multiplexer channel 0 which is "ITrm_ADC". When the conversion complete interrupt is
; requested the current value in the ADC Data Register (ADR) is stored as current "ITrm_ADC"
; value. The "adsel2" variable is then incremented to the next channel and the process repeats
; until channel 4 is requested at which time, "adsel2" and "adsel" are cleared to start the
; sequence again.
;
; "ADR" values are averaged with values in memory to reduce "digit rattle" and minimize the
; effects of spikes
;
;***********************************************************************************************

ADC_ISR:
     pshh                    ; Push index register Hi byte on to stack
                             ; (Do this because processor does not stack H)
     clrh                    ; Clear index register Hi byte
     lda     adsel           ; Load accumulator with value in ADC Channel Selector
     cmp     #$07            ; Compare with decimal 7
     beq     MULTIPLEX       ; If equal, branch to MULTIPLEX:
     tax                     ; Transfer value in accumulator to index register Lo
     lda     mapcur,x        ; Load accumulator with value in "mapcur,x"
     lda     ADR             ; Load accumulator with value in ADC Data Register(this also
                             ; clears conversion complete and interupt enable bit)
     add     mapcur,x        ; Add without Carry A<-(A)+(M)
     rora                    ; Rotate right accumulator (divide by two)
     sta     mapcur,x        ; Copy result to address mapcur,x
     lda     adsel           ; Load accumulator with value in ADC Channel Selector
     sta     ADCcnt          ; Copy to ADC channel counter
     inca                    ; Increment ADC Channel Selector Variable
     sta     adsel           ; Copy result to ADC Channel Selector Variable
     cmp     #$07            ; Compare with decimal 7
     beq     MULPLX_SET      ; If equal, branch to MULPLX_SET:
     bra     MULPLX_SET_DONE ; Branch to MULPLX_SET_DONE:

MULTIPLEX:
     lda     adsel2          ; Load accumulator with value in ADC Channel Selector 2
     tax                     ; Transfer value in accumulator to index register Lo
     lda     ITrm_ADCcur,x   ; Load accumulator with value in "ITrm_ADCcur,x"
     lda     ADR             ; Load accumulator with value in ADC Data Register(this also
                             ; clears conversion complete and interupt enable bit)
     add     ITrm_ADCcur,x   ; Add without Carry A<-(A)+(M)
     rora                    ; Rotate right accumulator (divide by two)
     sta     ITrm_ADCcur,x   ; Copy result to address ITrm_ADCcur,x
     lda     adsel2          ; Load accumulator with value in ADC Channel Selector 2
     add     #$07            ; Add without Carry A<-(A)+(M)
     sta     ADCcnt          ; Copy to ADC channel counter
     lda     adsel2          ; Load accumulator with value in ADC Channel Selector 2
     inca                    ; Increment ADC Channel Selector Variable 2
     cmp      #$04           ; Compare with decimal 4
     beq     MULTIPLEX_DONE  ; If equal, branch to MULTIPLEX_DONE:
     sta     adsel2          ; Copy to ADC Channel Selector Variable 2
     bra     MULPLX_SET      ; Branch to MULPLX_SET:

MULTIPLEX_DONE:
     clr     adsel           ; Clear "adsel"
     clr     adsel2          ; Clear "adsel2"
     bra     MULPLX_SET_DONE ; Branch to MULPLX_SET_DONE:


;***********************************************************************************************
; - Check the ADC selector variable. If it is decimal 7 check the ADC selector variable 2
;   to select the multiplexer channels. If it is anything other than decimal 7, fall through.
;***********************************************************************************************

MULPLX_SET:
     lda     adsel2           ; Load accumulator with value in ADC selector 2
     cbeqa   #$0,CHAN_0       ; Compare and if equal, branch to CHAN_0:(ign trim, A=0, B=0)
     cbeqa   #$1,CHAN_1       ; Compare and if equal, branch to CHAN_1:(oil prs, A=1, B=0)
     cbeqa   #$2,CHAN_2       ; Compare and if equal, branch to CHAN_2:(fuel prs, A=0, B=1)
     cbeqa   #$3,CHAN_3       ; Compare and if equal, branch to CHAN_3:(EGT, A=1, B=1)
     bra     MULPLX_SET_DONE  ; Branch to MULPLX_SET_DONE:

CHAN_0:
     bclr    MplexA,PORTA     ; Clear "MplexA" bit of Port A (logic Lo)
     bclr    MplexB,PORTA     ; Clear "MplexB" bit of Port A (Logic Lo)
     bra     MULPLX_SET_DONE  ; Branch to MULPLX_SET_DONE:

CHAN_1:
     bset    MplexA,PORTA     ; Set "MplexA" bit of Port A (Logic Hi)
     bclr    MplexB,PORTA     ; Clear "MplexB" bit of Port A (Logic Lo)
     bra     MULPLX_SET_DONE  ; Branch to MULPLX_IN_DONE

CHAN_2:
     bclr    MplexA,PORTA     ; Clear "MplexA" bit of Port A (Logic Lo)
     bset    MplexB,PORTA     ; Set "MplexB" bit of Port A (Logic Hi)
     bra     MULPLX_SET_DONE  ; Branch to MULPLX_SET_DONE:

CHAN_3:
     bset    MplexA,PORTA     ; Set "MplexA" bit of Port A (Logic Hi)
     bset    MplexB,PORTA     ; Set "MplexB" bit of Port A (Logic Hi)

MULPLX_SET_DONE:
     bset    adcc,inputs     ; Set "adcc"bit of "inputs" variable
     pulh                    ; Pull value from stack to index register Hi byte
     rti                     ; Return from interrupt


;**********************************************************************************************
; ----------------- Keyboard interrupt (Ign Mon on PortA4) ------------------
;
; The Keyboard interrupts are set up as edge sensetive only. A falling edge on
; a Keyboard pin does not latch an interrupt request if another keyboard pin
; is already low. To prevent losing an interrupt request in this situation,
; The Keyboard interrupts are disabled, and acknowledged. The pin requesting
; the interrupt is identified and disabled, then the keyboard interrupts are
; re enabled. In the main loop, the Port is polled to see when the pin returns
; high and re enabled as a keyboard interrupt pin at that time.
;***********************************************************************************************

KYBD_ISR:
     bset    ACKK,INTKBSCR       ; Set the Keyboard Acknowledge bit of Keyboard Status and
                                 ; Control Register (clear interrupt)

     brset   MONen,inputs,CHK_MON_LO   ; If "MONen" bit of "inputs" variable is set, branch to
                                       ; CHK_MON_LO:
     bra     KEYBD_DONE                ; Branch to KEYBD_DONE:

CHK_MON_LO:
     bclr     Ign_Mon,DDRA             ; Clear "Ign_Mon" bit of Port A Data Direction Register
                                       ;(Make sure PTA4 is configured as an input so it can
                                       ; be read)
     brset   Ign_Mon,porta,KEYBD_DONE  ; If "Ign Mon" bit of Port A is set, branch to
                                       ; KEYBD_DONE:

;***********************************************************************************************
; -------------------------- Calculate actual Ignition Timing Period ---------------------------
;***********************************************************************************************

DO_MON:
     bclr    Ign_Mon,INTKBIER   ; Clear "Ign_Mon" bit Keyboard Interrupt Enable Register
                                ; (disable keyboard interrupt for ignition monitor while pin
                                ; is low)
     bclr    MONen,inputs       ; Clear "MONen" bit of "inputs" variable
     lda     T2CNTL             ; Load accumulator with value in TIM2 counter register Lo byte
                                ; (Unlatch any previous reads)
     ldx     T2CNTH             ; Load index register Lo byte with value in TIM2 counter
                                ; register Hi byte (also latches Lo byte)
     stx     MON_tsH            ; Copy to "MON_tsH" variable
     lda     T2CNTL             ; Load accumulator with value in TIM2 counter register Lo byte
     sta     MON_tsL            ; Copy to "MON_tsL: variable
     sub     PIP_tsL            ; Subtract A<-(A)-(M)(Calculate cycle time)
                                ;(MON_tsL - PIP_tsL = MON_pL)
     sta     MON_pL             ; Copy result to "MON_pL" variable
     txa                        ; Transfer value in index register Lo byte to accumulator
                                ; (MON_tsH)
     sbc     PIP_tsH            ; Subtract with carry A<-(A)-(M)-(C)
                                ;(MON_tsH - PIP_tsH = MON_pH)
     sta     MON_pH             ; Copy result to "MON_pH" variable

KEYBD_DONE:
     rti                        ; Return from interrupt

;***********************************************************************************************
; ----- Dummy ISR vector - there just to keep the assembler happy -----
;***********************************************************************************************

Dummy:
	rti     ; Return from interrupt

;***********************************************************************************************
;
; ---------------------------------------- SUBROUTINES -----------------------------------------
;
;  - Ordered Table Search
;  - Linear Interpolation
;  - 32 x 16 divide
;  - Round after division
;  - Computation of Normalized Variables
;  - Pulse Width Totals
;  - VE table lookup
;  - ST table lookup
;  - Divide by 16
;
;***********************************************************************************************


;***********************************************************************************************
;
; ----------------------------------------Ordered Table Search ---------------------------------
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
;***********************************************************************************************

ORD_TABLE_FIND:
     clr     tmp5     ; Clear tmp5 variable
     ldhx    tmp1     ; Load high part of index register with value in tmp1
     lda     ,x       ; Load accumulator with low part of index register???
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


;***********************************************************************************************
;
; ---------------------------------- Linear Interpolation - 2D ---------------------------------
;
; Graph Plot        X2
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
;***********************************************************************************************

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
     jsr     DIVROUND         ; Jump to "DIVROUND" subroutine,(round result)
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
; ---------------------- 32 x 16 Unsigned Divide ---------------------------
;
; This routine takes the 32-bit dividend stored in INTACC1.....INTACC1+3
; and divides it by the 16-bit divisor stored in INTACC2:INTACC2+1.
; The quotient replaces the dividend and the remainder replaces the divisor.
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

*****************************************************************************
**
** ROUND after div (unsigned)
** 1)  check for div overflow (carry set), rail result if detected
** 2)  if (remainder * 2) > divisor then     ; was remainder > (divisor / 2)
** 2a)    increment result, rail if over-flow
**
*****************************************************************************

DIVROUND:
     bcs     DIVROUND0     ; If C bit of CCR is set, branch to DIVROUND0:
                           ; (div overflow? yes, branch)
     stx     tmp22         ; Copy value in index register Lo byte to
                           ; tmp22 variable (divisor)
     pshh                  ; Push value in index register Hi byte onto
                           ; stack (retrieve remainder)
     pulx                  ; Pull value on stack to index register Lo byte
     lslx                  ; Logical shift left index register lo byte (* 2)
     bcs     DIVROUND2     ; If C bit of CCR is set, branch to DIVROUND2:
                           ;(over-flow on left-shift, (remainder * 2) > $FF)
     cpx     tmp22         ; Compare value in tmp22 variable with value
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
; ---------------- Computation of Normalized Variables ----------------------
;
;  The following is the form of the evaluation for the normalized variables:
;
;  (A rem A * B)
;  -------------  = C rem C
;      100
;
;  Where A = Whole part of the percentage,
;        rem A = Remainder of A from previous calculation (range 0 to 99)
;        B = Percentage multiplied (this always has a zero remainder)
;        C = Whole part of result
;        rem C = remainder of result
;
;
;  Calculation is performed by the following method:
;
;     |(A * B) + (rem A * B)|
;     |          -----------|
;     |              100    |
;     ----------------------- = C rem C
;             100
;
;
;   Inputs:  tmp10 = A
;            tmp11 = rem A
;            tmp12 = B
;            tmp13 = rem B
;
;
;   Outputs: tmp10 = C
;            tmp11 = rem C
;            tmp13 = high order part of (A rem A) * B
;            tmp14 = low order part of (A rem A) * B
;
;****************************************************************************

Supernorm:
     lda     tmp10        ; Load accumulator with value in tmp10 (A)
     tax                  ; Transfer value in accumulator to index register Lo
     lda     tmp12        ; Load accumulator with value in tmp12 (B)
     mul                  ; Multiply ( X:A<-(X)x(A) )
     stx     tmp13        ; Copy value in index register Lo byte to tmp13
                          ;(High order of A * B)
     sta     tmp14        ; Copy value in accumulator to tmp14
                          ;(Low order of A * B)
     lda     tmp11        ; Load accumulator with value in tmp11 (rem A)
     tax                  ; Transfer value in accumulator to index reg Lo
     lda     tmp12        ; Load accumulator with value in tmp12 (B)
     mul                  ; Multiply ( X:A<-(X)x(A) )
     pshx                 ; Push value in index register to stack
     pulh                 ; Pull value in stack to index register Hi byte
                          ; (X)->(H)
     ldx     #$64         ; Load accumulator with decimal 100
     div                  ; Divide ( A<-(H:A)/(X);H<-Remainder )
     adc     tmp14        ; Add with carry ( A<-(A)+(M)+(C) )
                          ;(Add to lower part)
     sta     tmp14        ; Copy to tmp14 (Store back)
     bcc     Roundrem     ;If the C bit of CCR is clear, branch to Roundrem:
                          ;(Branch if no carry occurred)
     inc     tmp13        ; Increment value in tmp13
                          ;(Increment high-order part because an overflow
                          ; occurred in adc)

Roundrem:
     pshh                  ; Push value in index register Hi byte to stack
     pula                  ; Pull value from stack to accumulator
     cmp     #$32          ; Compare it to decimal 50
                           ;(Round if division remainder is greater than 50)
     ble     FinalNorm     ; If lesws than or equal to, branch to FinalNorm:
     lda     tmp14         ; Load accumulator with value in tmp14
     adc     #$01          ; Add with carry decimal 1
     sta     tmp14         ; Copy result to tmp13
     bcc     FinalNorm     ; If C bit of CCR is clear, branch to FinalNorm:
     inc     tmp13         ; Increment value in tmp13

FinalNorm:
     lda     tmp13        ; Load accumulator with value in tmp13
     psha                 ; Push value in accumulator to stack
     pulh                 ; Pull value in stack to index register Hi byte
     lda     tmp14        ; Load accumulator with value in tmp14
     ldx     #$64         ; Load index register Hi byte with decimal 100
     div                  ; Divide ( A<-(H:A)/(X);H<-Remainder )
     bcs     RailCalc     ; If C bit of CCR is set, branch to RailCalc:
     sta     tmp10        ; Copy result to tmp10
     pshh                 ; Push value in index register Hi byte to stack
     pula                 ; Pull value in stack to accumulator
     sta     tmp11        ; copy to tmp11
     cmp     #$32         ; Compare with decimal 50
                          ;(Round if division remainder is greater than 50)
     ble     ExitSN       ; If lower than or equal to, branch to ExitSN:
     lda     tmp11        ; Load accumulator with value in tmp11
     adc     #$01         ; Add with carry decimal 1
     sta     tmp11        ; Copy result to tmp11
     bcc     ExitSN       ; If C bit of CCR is set, branch to ExitSN:
     lda     tmp10        ; Load accumulator with value in tmp10
     add     #$01         ; Add it with decimal 1
     sta     tmp10        ; Copy result to tmp10
     bne     ExitSN       ; If Z bit of CCR is clear, branch to ExitSN:

RailCalc:
     mov     #$FF,tmp10      ; Move decimal 255 to tmp10 variable
                             ;(Rail value if rollover)

ExitSN:
     rts                     ; Return from subroutine

;***********************************************************************************************

;***********************************************************************************************
; Paging subroutines. ("page" stores which table is paged into RAM, the value is sent by
; Tuner Studio depending on the mode, default is zero)
;***********************************************************************************************
;***********************************************************************************************
; VE Table and Ranges. If page value is 1, get value from RAM, otherwise, use Flash
;***********************************************************************************************

VE1X:
     lda     page       ; Load accumulator with value in "page"
     cmp     #01T       ; Compare with decimal 1
     bne     VE1XF      ; If not (A)not=(M) branch to VE1XF:
     lda     VE_r,x     ; Load accumulator with value in "VE_r", offset in index register Lo
     bra     VE1XC      ; Branch to VE1XC:

VE1XF:
     lda     VE_f,x     ; Load accumulator with value in "VE_f" offset in index register Lo

VE1XC:
     rts                ; Return from subroutine

;***********************************************************************************************
; ST Table and Ranges, If page value is 2, get value from RAM, otherwise, use Flash
;***********************************************************************************************

VE2X:
     lda     page       ; Load accumulator with value in "page"
     cmp     #02T       ; Compare with decimal 2
     bne     VE2XF      ; If not (A)not=(M) branch to VE2XF:
     lda     ST_r,x     ; Load accumulator with value in "ST_r", offset in index register Lo
     bra     VE2XC      ; Branch to VE2XC:

VE2XF:
     lda     ST_f,x     ; Load accumulator with value in "ST_f" offset in index register Lo

VE2XC:
     rts                ; Return from subroutine

;***********************************************************************************************
; --------------------------------------- Divide by 16 -----------------------------------------
;
; This subroutine takes a 16 bit value stored in tmp2:tmp1 and divides it by 16. The 8
; bit result is tmp1
;***********************************************************************************************

DIV_BY_16:
;***********************************************************************************************
; - First divide by 2
;***********************************************************************************************

     lsr     tmp2     ; Logical shift right "tmp2" (divide by 2 Hi Byte)
     ror     tmp1     ; Rotate right throught carry "tmp1" (divide by 2 Lo Byte)

;***********************************************************************************************
; - Second divide by 2 (2x2=4 divide)
;***********************************************************************************************

     lsr     tmp2     ; Logical shift right "tmp2" (divide by 2 Hi Byte)
     ror     tmp1     ; Rotate right throught carry "tmp1" (divide by 2 Lo Byte)

;***********************************************************************************************
; - Third divide by 2 (2x2x2=8 divide)
;***********************************************************************************************

     lsr     tmp2     ; Logical shift right "tmp2" (divide by 2 Hi Byte)
     ror     tmp1     ; Rotate right throught carry "tmp1" (divide by 2 Lo Byte)

;***********************************************************************************************
; - Fourth divide by 2 (2x2x2x2=16 divide)
;***********************************************************************************************

     lsr     tmp2     ; Logical shift right "tmp2" (divide by 2 Hi Byte)
     ror     tmp1     ; Rotate right throught carry "tmp1" (divide by 2 Lo Byte)
     rts              ; Return from subroutine


;***********************************************************************************************
; REVNUM is not used with this format, but is retained to keep MT and TS happy.
; textversion_f is the specific code that this works with. (32 char)
; Signature is the data format that the code uses to communicate with MT and TS. (32 char)
;***********************************************************************************************

REVNUM:          db     00T
textversion_f:   db     'MSnS351WM by Robert Hiebert ****'
Signature:       db     'MS1/Extra format 029y3 *********'


;****************************************************************************
;
;------------------ Boot Loader-defined jump table/vector -------------------
;
;****************************************************************************

     include "burnerHP.asm"

     org     $FAC3              ; start bootloader-defined jump table/vector
                                ;(64,195)
     db      $12                ; scbr regi init value
     db      %00000001          ; config1
     db      %00000001          ; config2
     dw      start              ; Megasquirt code start
     dw      $FB00              ; bootloader start(64,256)

;****************************************************************************
;
; ----------------- Vector table origin vec_timebase -----------------------
;
;****************************************************************************



        db      $CC
	dw	Dummy        ;Timebase
        db      $CC
	dw	ADC_ISR      ;ADC Conversion Complete
        db      $CC
	dw	KYBD_ISR     ;Keyboard pin
        db      $CC
	dw	SCITX_ISR    ;SCI transmission complete/transmitter empty
        db      $CC
	dw	SCIRCV_ISR   ;SCI input idle/receiver full
        db      $CC
	dw	Dummy        ;SCI parity/framing/noise/receiver_overrun error
        db      $CC
	dw	Dummy	     ;SPI Transmitter empty
        db      $CC
	dw	Dummy	     ;SPI mode/overflow/receiver full
        db      $CC
	dw      Dummy        ;TIM2 overflow
        db      $CC
	dw	TIM2CH1_ISR  ;TIM2 Ch1
        db      $CC
	dw	TIM2CH0_ISR  ;TIM2 Ch0
        db      $CC
	dw	Dummy        ;TIM1 overflow
        db      $CC
	dw	Dummy        ;TIM1 Ch1
        db      $CC
	dw	Dummy        ;TIM1 Ch0
        db      $CC
	dw	Dummy	     ;CGM
        db      $CC
	dw	IRQ_ISR      ;IRQ
        db      $CC
	dw	Dummy	     ;SWI
        db      $CC
	dw	Start        ;Reset


;***********************************************************************************************
;
; ------------------------------------- Lookup Tables ------------------------------------------
;
;***********************************************************************************************

     org     $F000     ; Origin at Memory Location $F000=61440

     include "barofactor4250rjh.inc"  ; Converts Barometer ADC Raw Reading to baro correction
                                      ; factor in %

     include "kpafactor4250rjh.inc"   ; Converts Manifold Absolute Pressure ADC and Barometric
                                      ; Pressure ADC Raw Reading to Pressure in KPA

     include "thermfactor.inc"        ; Converts Coolant Temperature ADC Reading and Manifold
                                      ; Air TemperatureRaw Reading to temp in degrees F +40

     include "airdenfactor.inc"       ; Converts Manifold Air Temp ADC to air density
                                      ; corrrection factor in %

;**********************************************************************************************
; VE table and Ranges (copied into RAM on demand for tuning. (page = 1)(all pages 168 bytes)
; VE values are percent, KPARANGEVE values are KPA, RPMRANGEVE values are RPM/20
;**********************************************************************************************

     org     $E000    ;(57,344)

flash_table1:

VE_f:
	db      53T,53T,53T,56T,58T,59T,60T,60T,61T,62T,63T,64T ; VE (0,0-11)(Bottom Row)
	db      53T,53T,53T,54T,54T,55T,56T,57T,58T,59T,60T,61T ; VE (1,0-11)
	db      54T,54T,54T,55T,55T,56T,57T,58T,59T,60T,61T,62T ; VE (2,0-11)
	db      54T,54T,55T,56T,56T,56T,57T,58T,59T,60T,61T,62T ; VE (3,0-11)
	db      54T,54T,55T,56T,57T,57T,58T,59T,60T,61T,62T,63T ; VE (4,0-11)
	db      55T,55T,56T,57T,58T,59T,60T,61T,62T,63T,64T,65T ; VE (5,0-11)
	db      58T,58T,59T,60T,61T,62T,63T,64T,65T,66T,67T,68T ; VE (6,0-11)
	db      60T,60T,61T,62T,63T,64T,65T,66T,67T,68T,69T,70T ; VE (7,0-11)
	db      61T,61T,62T,63T,64T,65T,66T,67T,68T,69T,70T,71T ; VE (8,0-11)
	db      70T,70T,71T,72T,73T,74T,75T,75T,76T,77T,78T,79T ; VE (9,0-11)
	db      85T,85T,86T,87T,88T,89T,90T,91T,91T,92T,93T,94T; VE (10,0-11)
	db      86T,86T,87T,88T,89T,90T,91T,92T,92T,93T,94T,95T; VE (11,0-11)(Top Row)

RPMRANGEVE_f:        ; RPMRANGEVE[0-11](Left to Right)
	db      25T,35T,50T,65T,80T,95T,110T,130T,150T,170T,190T,210T
             ; 500,700,1000,1300,1600,1900,2200,2600,3000,3400,3800,4200

KPARANGEVE_f:        ; KPARANGEVE[0-b](Bottom to Top)
	db      30T,40T,45T,50T,55T,60T,65T,70T,75T,80T,85T,100T

flash_table1_end:

;**********************************************************************************************
; ST table and Ranges (copied into RAM on demand for tuning. (page = 2)(all pages 168 bytes)
; ST values are delay from PIP scaled to 255. MT table displays degrees advance,including
; the initial timing which is always 10 degrees BTDC
; KPARANGEST values are KPA, RPMRANGEST values are RPM/20
;**********************************************************************************************

     org     $E100    ;(57600)

flash_table2:

ST_f:
	db    218T,221T,213T,196T,176T,156T,147T,147T,147T,147T,147T,147T ;(0,0-11)(Bottom Row)
	db    207T,207T,201T,184T,167T,153T,147T,147T,147T,147T,147T,147T ;(1,0-11)
	db    198T,198T,181T,167T,156T,147T,147T,147T,147T,147T,147T,147T ;(2,0-11)
	db    187T,187T,170T,156T,147T,147T,147T,147T,147T,147T,147T,147T ;(3,0-11)
	db    190T,190T,176T,159T,147T,147T,147T,147T,147T,147T,147T,147T ;(4,0-11)
	db    193T,193T,187T,173T,156T,147T,147T,147T,147T,147T,147T,147T ;(5,0-11)
	db    198T,198T,190T,181T,170T,156T,153T,153T,153T,153T,153T,153T ;(6,0-11)
	db    201T,201T,196T,187T,179T,170T,156T,156T,156T,156T,156T,156T ;(7,0-11)
	db    207T,207T,201T,193T,187T,181T,170T,156T,156T,156T,156T,156T ;(8,0-11)
	db    213T,213T,207T,198T,193T,190T,181T,170T,170T,170T,170T,170T ;(9,0-11)
	db    221T,218T,213T,204T,196T,193T,190T,184T,181T,181T,181T,181T ;(10,0-11)
	db    224T,224T,221T,213T,204T,201T,198T,196T,190T,187T,184T,184T ;(11,0-11)(Top Row)

RPMRANGEST_f:        ; RPMRANGEST[0-11](Left to Right)
	db      25T,35T,50T,65T,80T,95T,110T,130T,150T,170T,190T,210T
             ; 500,700,1000,1300,1600,1900,2200,2600,3000,3400,3800,4200

KPARANGEST_f:        ; KPARANGEST[0-b](Bottom to Top)
	db      30T,40T,45T,50T,55T,60T,65T,70T,75T,80T,85T,100T

flash_table2_end:

;**********************************************************************************************
; AFR1 table and Ranges (copied into RAM on demand for tuning. (page = 3)(all pages 168 bytes)
; AFR1 values are AFRx10(Used for MLV VE Analyze only)
; KPARANGEAFR1 values are KPA, RPMRANGEAFR1 values are RPM/20
;**********************************************************************************************

     org     $E200    ;(57856)

flash_table3:

AFR1_f:
	db    130T,130T,130T,130T,130T,130T,130T,130T,130T,130T,130T,130T ;(0,0-11)(Bottom Row)
	db    130T,130T,130T,130T,130T,130T,130T,130T,130T,130T,130T,130T ;(1,0-11)
	db    130T,130T,130T,130T,130T,130T,130T,130T,130T,130T,130T,130T ;(2,0-11)
	db    130T,130T,130T,130T,130T,130T,130T,130T,130T,130T,130T,130T ;(3,0-11)
	db    130T,130T,130T,130T,130T,130T,130T,130T,130T,130T,130T,130T ;(4,0-11)
	db    130T,130T,130T,130T,130T,130T,130T,130T,130T,130T,130T,130T ;(5,0-11)
	db    130T,130T,130T,130T,130T,130T,130T,130T,130T,130T,130T,130T ;(6,0-11)
	db    130T,130T,130T,130T,130T,130T,130T,130T,130T,130T,130T,130T ;(7,0-11)
	db    130T,130T,130T,130T,130T,130T,130T,130T,130T,130T,130T,130T ;(8,0-11)
	db    130T,130T,130T,130T,130T,130T,130T,130T,130T,130T,130T,130T ;(9,0-11)
	db    130T,130T,130T,130T,130T,130T,130T,130T,130T,130T,130T,130T ;(10,0-11)
	db    130T,130T,130T,130T,130T,130T,130T,130T,130T,130T,130T,130T ;(11,0-11)(Top Row)

RPMRANGEAFR1_f:        ; RPMRANGEAFR1[0-11](Left to Right)
	db      25T,35T,50T,65T,80T,95T,110T,130T,150T,170T,190T,210T
             ; 500,700,1000,1300,1600,1900,2200,2600,3000,3400,3800,4200

KPARANGEAFR1_f:        ; KPARANGEAFR1[0-b](Bottom to Top)
	db      30T,40T,45T,50T,55T,60T,65T,70T,75T,80T,85T,100T

flash_table3_end:

;**********************************************************************************************
; AFR2 table and Ranges (copied into RAM on demand for tuning. (page = 4)(all pages 168 bytes)
; AFR2 values are AFRx10(Used for MLV VE Analyze only)
; KPARANGEAFR2 values are KPA, RPMRANGEAFR2 values are RPM/20
;**********************************************************************************************

     org     $E300    ;(58112)

flash_table4:

AFR2_f:
	db    117T,117T,117T,130T,147T,147T,147T,147T,147T,147T,147T,147T ;(0,0-11)(Bottom Row)
	db    117T,117T,117T,147T,165T,165T,165T,165T,165T,165T,165T,165T ;(1,0-11)
	db    130T,130T,147T,165T,165T,165T,165T,165T,165T,165T,165T,165T ;(2,0-11)
	db    130T,130T,147T,165T,165T,165T,165T,165T,165T,165T,165T,165T ;(3,0-11)
	db    130T,130T,147T,165T,165T,165T,165T,165T,165T,165T,165T,165T ;(4,0-11)
	db    130T,130T,147T,165T,165T,165T,165T,165T,165T,165T,165T,165T ;(5,0-11)
	db    130T,130T,147T,165T,165T,165T,165T,165T,165T,165T,165T,165T ;(6,0-11)
	db    130T,130T,147T,165T,165T,165T,165T,165T,165T,165T,165T,165T ;(7,0-11)
	db    130T,130T,147T,165T,165T,165T,165T,165T,165T,165T,165T,165T ;(8,0-11)
	db    130T,130T,130T,130T,130T,130T,130T,130T,130T,130T,130T,130T ;(9,0-11)
	db    128T,127T,126T,125T,124T,123T,122T,121T,120T,119T,118T,117T ;(10,0-11)
	db    126T,125T,124T,123T,122T,121T,120T,119T,118T,117T,116T,115T ;(11,0-11)(Top Row)

RPMRANGEAFR2_f:        ; RPMRANGEAFR2[0-11](Left to Right)
	db      25T,35T,50T,65T,80T,95T,110T,130T,150T,170T,190T,210T
             ; 500,700,1000,1300,1600,1900,2200,2600,3000,3400,3800,4200

KPARANGEAFR2_f:        ; KPARANGEAFR2[0-b](Bottom to Top)
	db      30T,40T,45T,50T,55T,60T,65T,70T,75T,80T,85T,100T

flash_table4_end:

;**********************************************************************************************
; Configuration constants and tables (copied into RAM on demand for tuning. page = 5)
; (all pages 168 bytes)
;**********************************************************************************************

     org     $E400    ;(58368)

flash_table5:

WWURANGE:   db      0T       ; WWURANGE[0] -40 F
            db      20T      ; WWURANGE[1] -20 F
            db      40T      ; WWURANGE[2]   0 F
            db      60T      ; WWURANGE[3] +20 F
            db      80T      ; WWURANGE[4] +40 F
            db      100T     ; WWURANGE[5] +60 F
            db      120T     ; WWURANGE[6] +80 F
            db      140T     ; WWURANGE[7] +100 F
            db      170T     ; WWURANGE[8] +130 F
            db      200T     ; WWURANGE[9] +160 F


WWU:        db      145T    ; WWU   Warmup bin @ -40 F
            db      140T    ; WWU   Warmup bin @ -20 F
            db      135T    ; WWU   Warmup bin @   0 F
            db      130T    ; WWU   Warmup bin @ +20 F
            db      125T    ; WWU   Warmup bin @ +40 F
            db      120T    ; WWU   Warmup bin @ +60 F
            db      115T    ; WWU   Warmup bin @ +80 F
            db      110T    ; WWU   Warmup bin @ +100 F
            db      105T    ; WWU   Warmup bin @ +130 F
            db      100T    ; WWU   Warmup bin @ +160 F


TPSDOTRATE: db      05T     ; TPSDOTRATE[0] (TPS ADC counts 0.1 volts)
            db      20T     ; TPSDOTRATE[1] (TPS ADC counts 0.4 volts)
            db      40T     ; TPSDOTRATE[2] (TPS ADC counts 0.8 volts)
            db      77T     ; TPSDOTRATE[3] (TPS ADC counts 1.5 volts)

TPSAQ:      db      20T     ; TPSAQ   Accel amount in 1ms units (tpsdot=0.1mS)
            db      30T     ; TPSAQ   Accel amount in 1ms units (tpsdot=0.4mS)
            db      40T     ; TPSAQ   Accel amount in 1ms units (tpsdot=0.8mS)
            db      50T     ; TPSAQ   Accel amount in 1ms units (tpsdot=1.5mS)
cwu:        db      110T    ; CWU          Crank Enrichment at - 40 F
cwh:        db      40T     ; CWH          Crank Enrichment at +165 F
awev:       db      70T     ; AWEV         Afterstart Warmup % enrich add on value
awc:        db      100T    ; AWC          Afterstart number of ignition cycles
tpsacold:   db      90T     ; TPSACOLD     Accel amount at - 40 F in .1 mS units
tpsthresh:  db      5T      ; TPSTHRESH    Accel difference over time threshold
tpsasync:   db      2T      ; TPSASYNC     Accel enrich time in .1 sec increments
req_fuel:   db      242T    ; REQFUEL      Required fuel constant
divider:    db      4T      ; DIVIDER      IRQ divide factor for fuel pulse
Alternate:  db      1T      ; ALTERNATE    Alternate injector firing (1 = alt, 0 = sim)
InjOpen:    db      9T      ; INJOPEN      Injector open time .1ms units
battfac:    db      12T     ; BATTFAC      Injector voltage compensation mms/V
floodClear: db      200T    ; floodClear   tpsADC value for flood clear
acmult:     db      100T    ; ACCELMULT    Accel cold mult factor (%/100)
pru:        db      20T     ; PRU          Primer PW @ -40 F)(0.1mS res)
prh:        db      10T     ; PRH          Primer PW @ 165 F)(0.1mS res)
PIP_Angle:  db      90T     ; PIP_Angle    PIP angle in crank degrees (6cyl=120, 8cyl=90)
CT_cnt:     db      48T     ; CT_cnt       Closed throttle position ADC count
WOT_cnt:    db      231T    ; WOT_cnt      Wide Open throttle position ADC count
awevh:      db      5T      ; awevh        After-start hot start % enrichment add-on
awch:       db      100T    ; awch         After-start hot number of cycles or time
tpsdq:      db      100T    ; TPSDQ        Deceleration fuel cut %
ASEHtype:   db      1T      ; ASEHtype     Configuration variable for ASE Hot counter type
                            ;              0 = ignition cycles, 1 = 100 ms counter,
                            ;              2 = 250 ms counter, 3 = 500 ms counter
                            ;              4 = 1 second counter
coolanton:  db      200T    ; coolanton    Set point for coolant temp ASE Hot Degrees F+40
heton:      db      240T    ; heton        High engine temperature alarm on set point (degF+40)
hetoff:     db      235T    ; hetoff       High engine temperature alarm off set point (degF+40)
lopon:      db      92T     ; lopon        Low oil pressure alarm on set point (OP_adc 0-255=0-100 PSI)
lopoff:     db      102T    ; lopoff       Low oil pressure alarm off set point (OP_adc 0-255=0-100 PSI)
hegton:     db      191T    ; hegton       High exhaust temp alarm on set point (EGT_adc 0-255=32-1328F)
hegtoff:    db      181T    ; hegtoff      High exhaust temp alarm off set point (EGT_adc 0-255=32-1328F)
lfpon:      db      102T    ; lfpon        Low fuel pressure alarm on set point (FP_adc 0-255=0-100 PSI)
lfpoff:     db      112T    ; lfpoff       Low fuel pressure alarm off set point (FP_adc 0-255=0-100 PSI)
hfpon:      db      153T    ; hfpon        High fuel pressure alarm on set point (FP_adc 0-255=0-100 PSI)
hfpoff:     db      143T    ; hfpoff       High fuel pressure alarm off set point (FP_adc 0-255=0-100 PSI)
revlon:     db      225T    ; revlon       Rev limiter alarm on set point (RPM20)
revloff:    db      175T    ; revloff      Rev limiter alarm off set point (RPM20)
runon:      db      17T     ; runon        Run on set point for ignition (RPM20)
startedon:  db      17T     ; startedon    Started on set point for fuel (RPM20)
airtempon:  db      190T    ; airtempon    Set point for Manifold Air Temp ASE Hot degrees F+40
BnkflowH:   db      2T      ; BnkflowH     Injector bank flow rate L/hr x 10 Hi byte
                            ;              configuration variable
BnkflowL:   db      31T     ; BnkflowL     Injector bank flow rate L/hr x 10 Lo byte
                            ;              configuration variable
ASEtype:    db      1T      ; ASEtype      Configuration variable for ASE counter type
                            ;              0 = ignition cycles, 1 = 100 ms counter,
                            ;              2 = 250 ms counter, 3 = 500 ms counter
                            ;              4 = 1 second counter


flash_table5_end:


     include "boot_r12.asm"

flash_1_size        equ {flash_table1_end-flash_table1}
flash_2_size        equ {flash_table2_end-flash_table2}
flash_3_size        equ {flash_table3_end-flash_table3}
flash_4_size        equ {flash_table4_end-flash_table4}
flash_5_size        equ {flash_table5_end-flash_table5}

     end

;***********************************************************************************************
; This marks the end of the program
;***********************************************************************************************
;****************************************************************************
; ------------------ Engine Configuration Bit Definitions ------------------
;****************************************************************************
;
; EngConfig:
;  Bit 0 = Number of Cylinders
;          1 = 8 cylinders
;          0 = 6 cylinders
;
;  Bit 1 = O2 Sensor type
;          1 = Wide band
;          0 = Narrow band
;
;  Bit 2 = Control Strategy
;          1 = Speed Density
;          0 = Alpha-N
;
;  %00000110 = 6T (SD, WB, 6cyl)
;  %00000111 = 7T (SD, WB, 8cyl)

