;****************************************************************************
;
;                          MS_IAC.asm 10/12/12
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
;  PTB5/AD5  - Idle Air Control Input Port                 - Pin 6
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


.header 'MS_IAC'    	          ; Listing file title
.pagewidth 130          	    ; Listing file width
.pagelength 90          	    ; Listing file height

.nolist                           ; Turn off listing file
     include "gp32.equ"           ; Include HC 908 equates
.list                             ; Turn on listing file
     org      ram_start           ; Origin  Memory location $0040=64
     include "MS_IAC.inc"         ; Include definitions for MS_IAC.asm

;***************************************************************************
;
; Main Routine Here - Initialization and main loop
;
; Note: Org down 256 bytes below the "rom_start" point
;       because of erase bug in bootloader routine
;
; Note: Items commented out after the Start entry point are
;       taken care of in the Boot_R12.asm code
;
;***************************************************************************


     org   {rom_start + 256}     ; Origin at memory location
                                 ; $8000+256 = 32,768+256 = 33,024=$8100

Start:
     ldhx   #init_stack+1     ; Load index register with value in
                              ; init_stack+1(Set the stack Pointer)
     txs                      ; Transfer value in index register Lo byte
                              ; to stack
                              ;(Move before burner to avoid conflict)

;* Note - uncomment this code if you do not use the Bootloader to initilize *
;       clra
;	sta	copctl
;	mov	#%00000001,config2
;	mov	#%00001001,config1
;	mov	#%00000001,config1
;	ldhx	#ram_last+1		; Set the stack Pointer
;	txs				;  to the bottom of RAM

;****************************************************************************
; - Set the phase lock loop for a bus frequency of 8.003584mhz
;  (Boot loader initially sets it at 7.3728mhz)
;****************************************************************************

;PllSet:
	bclr	BCS,pctl          ; Select external Clock Reference
	bclr	PLLON,pctl        ; Turn Of PLL
	mov	#$02,pctl         ; Set P and E Bits
	mov	#$D0,pmrs         ; Set L ($C0 for 7.37 MHz)
	mov	#$03,pmsh         ; Set N (MSB)
	mov	#$D1,pmsl         ; Set N (LSB) ($84 for 7.37 MHz)
	bset	AUTO,pbwc         ; Enable automatic bandwidth control
	bset	PLLON,pctl        ; Turn back on PLL
PLL_wait:
     brclr   LOCK,pbwc,PLL_wait   ; Wait for PLL to lock
     bset    BCS,pctl             ; Select VCO as base clock



;****************************************************************************
; ------------- Set up the port data-direction registers --------------------
;               Set directions,
;               Preset state of pins to become outputs
;               Set all unused pins to outputs initialized Lo
;****************************************************************************

;****************************************************************************
; - Port A set for MS_TECA hardware configuration
;****************************************************************************

; Port A
     mov     #$FF,PTAPUE     ; Move %11111111 into Port A pullup register
                             ;(Set all pullups)
     mov     #$FF,PORTA      ; Move %11111111 into Port A Data Register
                             ;(preinit all pins Hi, no input signals)
     clr     DDRA            ; Clear Port A Data Direction Register
                             ;(Inputs on PTA7,6,5,4,3,2,1,0)
                             ;(= Shiftup,Shiftdn,TCCapp,TCCrel,DFCen,
                             ;(DFCdis,ExhPS,VehSpd,

;****************************************************************************
; - Set up ADC inputs
;****************************************************************************

; Port B
     clr     PORTB           ; Clear Port B Data Register
                             ;(Preinit all pins low)
     clr     DDRB            ; Clear Port B Data Direction Register
                             ;(Set as ADC inputs, "ADSEL" selects channel)

;****************************************************************************
; - Port C set for MS_TECA hardware configuration
;****************************************************************************

; Port C
     mov     #$1F,PORTC      ; Move %00011111 into Port C Data Register
                             ;(preinit output pins Hi, no outputs)
     lda     #$1F            ; Load accumulator with %00011111
                             ; (set up port directions, 1 = out)
     sta     DDRC            ; Copy to Port C Data Direction Register
                             ; Inputs on PTC7,6,5 = NA,NA,NA
                             ; Outputs on PTC4,3,2,1,0
                             ; = ExhBrk,CCS,TCC,SS1,SS2

;****************************************************************************
; Port D set for MS_TECA hardware configuration
;****************************************************************************

; Port D
     mov     #$FF,PTDPUE     ; Move %11111111 into Port E pullup register
                             ;(Set all pullups)
     mov     #$3C,PORTD      ; Move %00111100 into Port D Data Register
                             ;(preinit output pins Hi, no outputs)
     lda     #$FC            ; Load accumulator with %11111100
                             ; (init port directions 1 = out)
     sta     DDRD            ; Copy to Port D Data Direction Register
                             ; Inputs on PTD1,0 = H2,H1
                             ; Outputs on PTD7,6,5,4,3,2
                             ; = NA,NA,IACpwmS,IACpwmP,LoopFrq,DFCper

;****************************************************************************
;- Port E not used set all pins as outputs
;****************************************************************************

; Port E
     clr     PORTE           ; Clear Port E Data Register (to avoid glitches)
     lda     #$01            ; Load accumulator with %00000001
                             ; (set up port directions, 1 = out)
                             ; (Serial Comm Port)
     sta     DDRE            ; Copy to Port E Data Direction Register


;****************************************************************************
; Set up TIM2 as a free running ~1us counter. Set Channel 0 output compare
; to generate the ~100us(0.1ms) clock tick interupt vector "TIM2CH0_ISR:"
;****************************************************************************

     mov     #$33,T2SC       ; Move %00110011 into Timer2
                             ; Status and Control Register
                             ;(Disable interupts, stop timer)
                             ;(Prescale and counter cleared))
                             ;(Prescale for bus frequency / 8)
     mov     #$FF,T2MODH     ; Move decimal 255 into T2 modulo reg Hi
     mov     #$FF,T2MODL     ; Move decimal 255 into T2 modulo reg Lo
                             ;(free running timer)
     mov     #$00,T2CH0H     ; Move decimal 0 into T1CH0 O/C register Hi
     mov     #$64,T2CH0L     ; Move decimal 100 into T1CH0 O/C register Lo
                             ;(~100uS)=(~0.1ms)
     mov     #$54,T2SC0      ; Move %01010100 into Timer2
                             ; channel 0 status and control register
                             ; (Output compare, interrupt enabled)
     mov     #$03,T2SC       ; Move %00000011 into Timer2
                             ; Status and Control Register
                             ; Disable interupts, counter Active
                             ; Prescale for bus frequency / 8
                             ; 8,003584hz/8=1000448hz
                             ; = .0000009995sec

;****************************************************************************
; - Set up Serial Communications Interface Module
;****************************************************************************

     lda      #$30           ; Load accumulator with %110000
     sta      SCBR           ; Copy to SCI Baud Rate Register
                             ; 8003584/(64*13*1)=9619.7 baud
     bset     ensci,SCC1     ; Set enable SCI bit of SCI Control Register 1
                             ; (Enable SCI)
     bset     RE,SCC2        ; Set receiver enable bit of SCI Control Reg. 2
                             ; (Enable receiver)
     bset     SCRIE,SCC2     ; Set SCI receive interrupt enable bit of
                             ; SCI Control Register 2 (Enable Rcv. Interrupt)
     lda      SCS1           ; Load accumulator with SCI Status Register 1
                             ; (Clear SCI transmitter Empty Bit)
     clr      txcnt          ; Clear SCI transmitter count
                             ; (incremented)(characters transmitted)
     clr      txgoal         ; Clear SCI number of bytes to transmit
                             ; (characters to be transmitted)

;****************************************************************************
; - Initialize the variables to 0 or to some acceptable starting value
;****************************************************************************

     clr     secH       ; Seconds counter, Hi byte
     clr     secL       ; Seconds counter, Lo byte
     clr     BAT        ; Battery Voltage 8 bit ADC reading
     clr     IACP       ; Port Engine Idle Air Control Sensor 8 bit ADC reading
     clr     IACS       ; Stbd Engine Idle Air Control Sensor 8 bit ADC reading
     clr     Volts      ; Battery voltage to 0.1V resolution
     clr     IACpwP     ; Port IAC pulse width variable(0-60, 100uS resolution)
     clr     IACpwS     ; Stbd IAC pulse width variable(0-60, 100uS resolution)
     clr     inputs     ; Input status bit field variable(1 of 2)
     clr     uSx100     ; 100 Microseconds counter
     clr     mS         ; Milliseconds counter
     clr     mSx5       ; 5 Milliseconds counter
     clr     mSx20      ; 20 Milliseconds counter
     clr     mSx100     ; 100 Milliseconds counter
     clr     adsel      ; ADC Selector Variable
     clr     txcnt      ; SCI transmitter count (incremented)
     clr     txgoal     ; SCI number of bytes to transmit
     clr     txmode     ; Transmit mode flag
     clr     rxoffset   ; Offset placeholder when receiving VE/constants vis. SCI
     clr     LoopCntr   ; Loop counter for main loop frequency check
     clr     IACcntP    ; IAC counter for IACpwP (100uS resolution)
     clr     IACcntS    ; IAC counter for IACpwS (100uS resolution)
     clr     AIACcntP   ; Port Auto IAC duration counter value(100mS res)
     clr     AIACcntS   ; Stbd Auto IAC duration counter value(100mS res)
     clr     blank1     ; Placeholder
     clr     blank2     ; Placeholder
     clr     blank3     ; Placeholder
     clr     blank4     ; Placeholder
     clr     blank5     ; Placeholder
     clr     blank6     ; Placeholder
     clr     blank7     ; Placeholder
     clr     blank8     ; Placeholder

;****************************************************************************
; - Fire up the ADC, and perform one conversion, Set up clock source for ADC
;   Do an initial conversion just to stabilize the ADC
;****************************************************************************

Stb_ADC:
     lda     #$70      ; Load accumulator with %01110000
     sta     ADCLK     ; Copy to ADC Clock Register
                       ;( bus clock/8 = ~1mhz )
     lda     #$04      ; Load accumulator with %00000100
                       ;(one conversion, no interrupt on channel AD4)
     sta     ADSCR     ; Copy to ADC Status and Control Register

ADCWait:
     brclr   coco,ADSCR,ADCWait   ; If "conversions complete flag" bit of
                                  ; ADC Status and Control Register is clear
                                  ; branch to ADCWait lable
                                  ;(keep looping while COnversion
                                  ; COmplete flag = 0)
     lda    ADR                   ; Load accumulator with value in ADC Result
                                  ; Variable (read value from ADc Result)
     sta    BAT                   ; Copy to Battery Voltage 8 bit ADC Reading
     lda    #$04                  ; Load accumulator with decimal 4
     sta    adsel                 ; Copy to "adsel"

TURN_ON_INTS:
     cli                          ; Clear intrupt mask
                                  ;( Turn on all interrupts now )


;****************************************************************************
;****************************************************************************
;********************    M A I N  E V E N T  L O O P     ********************
;****************************************************************************
;****************************************************************************

;****************************************************************************
; - Toggle pin 3 on Port D each program loop so frequency can be checked
;   with a frequency meter or scope. (for program developement)
;****************************************************************************

LOOPER:
     com     LoopCntr         ; Ones compliment "LoopCntr"
                              ;(flip state of "LoopCntr")
     bne     SET_LOOPCHK      ; If the Z bit of CCR is clear, branch
                              ; to SET_LOOPCHK
     bclr    LoopFrq,PORTD    ; Clear bit 3 of Port D (Program Loop LED)
     bra     LOOPCHK_DONE     ; Branch to LOOPCHK_DONE:

SET_LOOPCHK:
     bset    LoopFrq,PORTD    ; Set bit 3 of Port D (Program Loop LED)

LOOPCHK_DONE:

;****************************************************************************
; - Check to see if it's time to turn the Port IAC PWM off.
;****************************************************************************

     brclr   iaconP,inputs,IACP_CHK_DONE  ; If "iaconP" bit of "inputs"
                             ; variable is clear, branch to IACP_CHK_DONE:
     lda     IACcntP         ; Load accumulator with value in Port IAC counter
     bne     IACP_CHK_DONE   ; If Z bit of CCR is clear, branch to
                             ; IACP_CHK_DONE:
     bset    iacpwmP,portd   ; Set "iacpwmP" bit of Port D (IAC PW "off")
     bclr    iaconP,inputs   ; Clear "iaconP" bit of "inputs" variable

IACP_CHK_DONE:

;****************************************************************************
; - Check to see if it's time to turn the Stbd IAC PWM off.
;****************************************************************************

     brclr   iaconS,inputs,IACS_CHK_DONE  ; If "iaconS" bit of "inputs"
                             ; variable is clear, branch to IACP_CHK_DONE:
     lda     IACcntS         ; Load accumulator with value in Stbd IAC counter
     bne     IACS_CHK_DONE   ; If Z bit of CCR is clear, branch to
                             ; IACS_CHK_DONE:
     bset    iacpwmS,portd   ; Set "iacpwmS" bit of Port D (IAC PW "off")
     bclr    iaconS,inputs   ; Clear "iaconS" bit of "inputs" variable

IACS_CHK_DONE:

;****************************************************************************
; - Update the ADC readings and conversions, This is done only once per ADC
;   conversion completion, in the first pass through the main loop after the
;   ADC_ISR Interrupt routine has been completed.
;****************************************************************************

     brset   adcc,inputs,ADC_LOOKUPS  ; If "adcc" bit of "inputs" variable
                                      ; is set, branch to ADC_LOOKUPS:
     jmp     NO_ADC_PASS              ; Jump to NO_ADC_PASS:

ADC_LOOKUPS:
     clrh                    ; Clear index register Hi byte
     clrx                    ; Clear index register Lo byte

;VOLTS_CALC:
     lda     BAT             ; Load accumulator with value in Battery
                             ; Voltage 8 bit ADC reading
     tax                     ; Copy to index register Lo byte
     lda     BatVolt,x       ; Load accumulator with value in "BatVolt"
                             ; table, offset in index register Lo byte
     sta     Volts           ; Copy to Battery Voltage to 0.1V resolution

;IACPWP_CALC:
     lda     IACP            ; Load accumulator with value in Port Idle
                             ; Position Sensor 8 bit ADC reading
     tax                     ; Copy to index register Lo byte
     lda     IACcntrl,x      ; Load accumulator with value in IAC control
                             ; table, offset in index register Lo byte
     sta     IACpwP          ; Copy to Port Idle AIr Control PW variable

;IACPWS_CALC:
     lda     IACS            ; Load accumulator with value in Stbd Idle
                             ; Position Sensor 8 bit ADC reading
     tax                     ; Copy to index register Lo byte
     lda     IACcntrl,x      ; Load accumulator with value in IAC control
                             ; table, offset in index register Lo byte
     sta     IACpwS          ; Copy to Stbd Idle AIr Control PW variable
     bclr    adcc,inputs     ; Clear "adcc" bit of "inputs" variable

NO_ADC_PASS:

     bra     LOOPER          ; Branch to LOOPER: (End of Main Loop!!!)


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
; TIM2CH0_ISR: - TIM2 CH0 output compare ($0064 * 1uS) (100us Timer Tick)
;
; SCIRCV_ISR:  - SCI receive (not used)
;
; SCITX_ISR:   - SCI transmit
;
; ADC_ISR:     - ADC Conversion Complete
;
;****************************************************************************

;****************************************************************************
;============================================================================
; - TIM2 CH0 Interrupt (100 uS clock tick)
; - Generate time rates:
;   100 Microseconds,(for IAC PWM counters)
;   Milliseconds,(for ADC conversions)
;   5 Milliseconds,(for 200hz clock tick for IAC control PWM frequency
;   20 Milliseconds, (because we can)
;   100 Milliseconds, (because we can)
;   Seconds,(because we can)
;============================================================================
;****************************************************************************

TIM2CH0_ISR:
     pshh                  ; Push value in index register Hi byte to stack
     lda     T2SC0         ; Load accumulator with value in TIM2 CH0 Status
                           ; and Control Register (Arm CHxF flag clear)
     bclr    CHxF,T2SC0    ; Clear CHxF bit of TIM2 CH0 Status and
                           ; Control Register
     ldhx    T2CH0H        ; Load index register with value in TIM2 CH0
                           ; register H:L (output compare value)
     aix     #$64          ; Add decimal 100 (100 uS)
     sthx    T2CH0H        ; Copy result to TIM2 CH0 register
                           ;(new output compare value)

;============================================================================
;********************** 100 Microsecond section *****************************
;============================================================================

;****************************************************************************
; - Decrement the IAC PWM counters
;****************************************************************************

DEC_IACP_CNTR:
     lda     IACcntP         ; Load accumulator with value in Port IAC counter
     beq     DEC_IACP_DONE   ; If Z bit of CCR is set, branch to DEC_IACP_DONE:
     dec     IACcntP         ; Decrement Port IAC PWM "off" counter

DEC_IACP_DONE:

;DEC_IACS_CNTR:
     lda     IACcntS         ; Load accumulator with value in Srbd IAC counter
     beq     DEC_IACS_DONE   ; If Z bit of CCR is set, branch to DEC_IACS_DONE:
     dec     IACcntS         ; Decrement Stbd IAC PWM "off" counter

DEC_IACS_DONE:

;****************************************************************************
; - Increment 100 Microsecond counter
;****************************************************************************

INC_cuS:
     inc     uSx100       ; Increment 100 Microsecond counter
     lda     uSx100       ; Load accumulator with 100 Microsecond counter
     cmp     #$0A         ; Compare it with decimal 10
     bne     NOT_MS       ; If not equal, branch to NOT_MS:
     bra     FIRE_ADC     ; Branch to FIRE_ADC:

NOT_MS:     jmp     TIM2CH0_ISR_DONE

;============================================================================
;************************* millisecond section ******************************
;============================================================================

;****************************************************************************
; - Fire off another ADC conversion, channel is pointed to by "adsel"
;   adsel = 0 = Channel 4 = "BAT"
;   adsel = 1 = Channel 5 = "IACP"
;   adsel = 2 = Channel 6 = "IACS"
;****************************************************************************

FIRE_ADC:
     lda     adsel          ; Load accumulator with value in ADC Channel Selector
     cmp     #$03           ; Compare value in accumulator with decimal 3
     bhs     ROLL_ADSEL     ; If "adsel >= decimal 3, branch to ROLL_ADSEL:
     bra     ADSEL_OK       ; Branch to ADSEL_OK:

ROLL_ADSEL:
     clr     adsel          ; Clear "adsel"

ADSEL_OK:
     lda     adsel          ; Load accumulator with ADC Selector Variable
     add     #$04           ; Add A<-A+M (first ADC is channel 4)
     ora     #%01000000     ; Inclusive "or" with %01000000 and ADC Selector
                            ; Variable ( result in accumulator )
                            ;(Enables interupt with channel selected)
     sta     ADSCR          ; Copy result to ADC Status and Control Register

;****************************************************************************
; - Increment millisecond counter
;****************************************************************************

INC_mS:
     clr     uSx100              ; Clear 100 Microsecond counter
     inc     mS                  ; Increment Millisecond counter
     lda     mS                  ; Load accumulator with value in
                                 ; Millisecond counter
     cmp     #$05                ; Compare it with decimal 5
     beq     DO_IACP             ; If Z bit of CCR is set, branch to DO_IACP:
                                 ;(mS=5)
     jmp     TIM2CH0_ISR_DONE    ; Jump to TIM2CH0_ISR_DONE:


;============================================================================
;************************** 5 Millisecond section ***************************
;============================================================================

;****************************************************************************
; - Set Port AIC solonoid PWM "on" PTD4 (200 HZ)
;****************************************************************************

DO_IACP:
     lda     IACpwP         ; Load accumulator with value in Port IAC PW
     beq     NO_IACP        ; If Z bit of CCR is set, branch to NO_IACP:
                            ;(No IAC commanded so no PWM)
     sta     IACcntP        ; Copy to Port IAC PWM "off" counter
     bclr    iacpwmP,portd  ; Clear "iacpwmP" bit of Port D (IAC PW "on")
     bset    iaconP,inputs  ; Set "iaconP" bit of "inputs" variable
     bra     IACP_DONE      ; Branch to IACP_DONE:

NO_IACP:
     bset    iacpwmP,portd  ; Set "iacpwmP" bit of Port D (IAC PW "off")
     bclr    iaconP,inputs  ; Clear "iaconP" bit of "inputs" variable

IACP_DONE:

;****************************************************************************
; - Set Stbd AIC solonoid PWM "on" PTD5 (200 HZ)
;****************************************************************************

DO_IACS:
     lda     IACpwS         ; Load accumulator with value in Stbd IAC PW
     beq     NO_IACS        ; If Z bit of CCR is set, branch to NO_IACS:
                            ;(No IAC commanded so no PWM)
     sta     IACcntS        ; Copy to Stbd IAC PWM "off" counter
     bclr    iacpwmS,portd  ; Clear "iacpwmS" bit of Port D (IAC PW "on")
     bset    iaconS,inputs  ; Set "iaconS" bit of "inputs" variable
     bra     IACS_DONE      ; Branch to IACS_DONE:

NO_IACS:
     bset    iacpwmS,portd  ; Set "iacpwmS" bit of Port D (IAC PW "off")
     bclr    iaconS,inputs  ; Clear "iaconS" bit of "inputs" variable

IACS_DONE:

;****************************************************************************
; - Increment 5 millisecond counter
;****************************************************************************

INC_mSx5:
     clr     mS                  ; Clear "mS"
     inc     mSx5                ; Increment 5 Millisecond counter
     lda     mSx5                ; Load accumulator with value in
                                 ; 5 Millesecond counter
     cmp     #$04                ; Compare it with decimal 4
     bne     TIM2CH0_ISR_DONE    ; If the Z bit of CCR is clear,
                                 ; branch to TIM2CH0_ISR_DONE:

;============================================================================
;************************* 20 Millisecond section ***************************
;============================================================================

;****************************************************************************
; - Increment 20 Millisecond counter
;****************************************************************************

INC_mSx20:
     clr     mSx5                ; Clear 5 Millisecond counter
     inc     mSx20               ; Increment 20 Millisecond counter
     lda     mSx20               ; Load accumulator with value in
                                 ; 20 Millesecond counter
     cmp     #$05                ; Compare it with decimal 5
     bne     TIM2CH0_ISR_DONE    ; If the Z bit of CCR is clear,
                                 ; branch to TIM2CH0_ISR_DONE:


;============================================================================
;************************* 100 Millisecond section **************************
;============================================================================

     bset    clk100k,inputs     ; Set "clk100k" bit of "inputs" variable

 ;****************************************************************************
; - Increment 100 Millisecond counter
;****************************************************************************

INC_cmS:
     clr     mSx20               ; Clear 20 Millisecond counter
     inc     mSx100              ; Increment 100 Millisecond counter
     lda     mSx100              ; Load accumulator with value in
                                 ; 100 Millisecond counter
     cmp     #$0A                ; Compare with decimal 10
     beq     INC_S               ; If Z bit of CCR is set, branch to INC_S:
     bra     TIM2CH0_ISR_DONE    ; Branch to TIM2CH0_ISR_DONE:

;============================================================================
;**************************** Seconds section *******************************
;============================================================================

;****************************************************************************
; - Increment Seconds counter
;****************************************************************************

INC_S:
     clr     mSx100              ; Clear 0.1 Second variable
     inc     secl                ; Increment "Seconds" Lo byte variable
     bne     TIM2CH0_ISR_DONE    ; If the Z bit of CCR is clear, branch
                                 ; to TIM2CH0_ISR_DONE:
     inc     sech                ; Increment "Seconds" Hi byte variable

TIM2CH0_ISR_DONE:
     pulh                  ; Pull value from stack to index register Hi byte
     rti                   ; Return from interrupt

;****************************************************************************
;
; -------------------- Serial Communications Interface ----------------------
;
; Communications is established when the PC communications program sends
; a command character - the particular character sets the mode:
;
; "A" = send all of the realtime variables via txport.
; "V" = send the Constants group 1 via txport (128 bytes)
;       (This has to be in, don't know why)
; "C" = Test communications - echo back SECL
; "Q" = Send over Embedded Code Revision Number (divide number by 10
;       - i.e. $21T is rev 2.1)
; "I" = send the Constants group 2 via txport (64 bytes)
;       (This has to be in, don't know why)
;
; txmode:
;              01 = Getting realtime data
;              02 = ?
;              03 = Sending group 1
;              04 = ?
;              05 = Getting offset group 1
;              07 = Getting offset group 2
;              09 = Sending group 2
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
     cmp     #$07         ; Compare with decimal 7
     beq     TXMODE_7     ; If the Z bit of CCR is set, branch to TXMODE_7:
     bra     CHECK_TXCMD  ; Branch to CHECK_TXCMD:

TXMODE_5:                 ; (Getting offset for either W or J command)

TXMODE_7:
     mov     SCDR,rxoffset   ; Move value in "SCDR" to "rxoffset"
     inc     txmode          ; (continue to next mode)
     jmp     DONE_RCV        ; Jump to DONE_RCV:

CHECK_TXCMD:
     lda     SCDR       ; Load accumulator with value in "SCDR"
                        ;(Get the command byte)
     cmp     #$41       ; Compare it with decimal 65 = ASCII "A"
                        ;(Is the recieve character a big "A" ->
                        ; Download real-time variables?)
     beq     MODE_A     ; If the Z bit of CCR is set, branch to Mode_A:
     cmp     #$43       ; Compare it with decimal 67 = ASCII "C"
     beq     MODE_C     ; If the Z bit of CCR is set, branch to Mode_C:
     cmp     #$56       ; Compare it with decimal 86 = ASCII "V"
     beq     MODE_V     ; If the Z bit of CCR is set, branch to Mode_V:
     cmp     #$51       ; Compare it with decimal 81 = ASCII "Q"
     beq     MODE_Q     ; If the Z bit of CCR is set, branch to Mode_Q:
     cmp     #'I'       ; Compare it with 'I' = ASCII decimal 73 $49
     beq     MODE_I     ; If the Z bit of CCR is set, branch to Mode_I:
     bra     DONE_RCV   ; Branch to DONE_RCV:

MODE_A
     clr     txcnt          ; Clear "txcnt"
     lda     #$01           ; Load accumulator with decimal 1
     sta     txmode         ; Copy to "txmode" variable
     lda     #$09           ; Load accumulator with decimal 9
                            ;(Set this for 1 more than the number of bytes
                            ; to send)
                            ;(8 Real time variables for MS_IAC)
     sta     txgoal         ; Copy to "txgoal" variable
     bset    TE,SCC2        ; Set "TE" bit of SCC2 (Enable Transmit)
     bset    SCTIE,SCC2     ; Set "SCTIE" bit of SCC2
                            ;(Enable transmit interrupt)
     bra     DONE_RCV       ; Branch to DONE_RCV:

MODE_C:
     clr     txcnt          ; Clear "txcnt"
                            ; (Just send back SECL variable to test comm port)
     lda     #$01           ; Load accumulator with decimal 1
     sta     txmode         ; Copy to "txmode" variable
     lda     #$2            ; Load accumulator with decimal 2
     sta     txgoal         ; Copy to "txgoal" variable
     bset    TE,SCC2        ; Set "TE" bit of SCC2 (Enable Transmit)
     bset    SCTIE,SCC2     ; Set "SCTIE" bit of SCC2
                            ;(Enable transmit interrupt)
     bra     DONE_RCV       ; Branch to DONE_RCV:

MODE_V:
     clr     txcnt          ; Clear "txcnt"
     lda     #$03           ; Load accumulator with decimal 3
     sta     txmode         ; Copy to "txmode" variable
     lda     #$81           ; Load accumulator with decimal 129
                            ;(Set this for 1 more than the number of bytes
                            ; to send)
                            ;(Send 128 bytes, TO table, ranges, EPC stall
                            ; and shift tables + spares)
     sta     txgoal         ; Copy to "txgoal" variable
     bset    TE,SCC2        ; Set "TE" bit of SCC2 (Enable Transmit)
     bset    SCTIE,SCC2     ; Set "SCTIE" bit of SCC2
                            ;(Enable transmit interrupt)
     bra     DONE_RCV       ; Branch to DONE_RCV:

MODE_Q:
     clr     txcnt          ; Clear "txcnt"
                            ; (Just send back SECL variable to test comm port)
     lda     #$05           ; Load accumulator with decimal 5
     sta     txmode         ; Copy to "txmode" variable
     lda     #$2            ; Load accumulator with decimal 2
     sta     txgoal         ; Copy to "txgoal" variable
     bset    TE,SCC2        ; Set "TE" bit of SCC2 (Enable Transmit)
     bset    SCTIE,SCC2     ; Set "SCTIE" bit of SCC2
                            ;(Enable transmit interrupt)
     bra     DONE_RCV       ; Branch to DONE_RCV:

MODE_I:
     clr     txcnt          ; Clear "txcnt"
     lda     #$09           ; Load accumulator with decimal 9
                            ; (txmode = sending Cons Group 2)
     sta     txmode         ; Copy to "txmode" variable
     lda     #$41           ; Load accumulator with decimal 65
                            ;(Set this for 1 more than the number of bytes
                            ; to send)
                            ;(Send 64 bytes, constants + spares)
     sta     txgoal         ; Copy to "txgoal" variable
     bset    TE,SCC2        ; Set "TE" bit of SCC2 (Enable Transmit)
     bset    SCTIE,SCC2     ; Set "SCTIE" bit of SCC2
                            ;(Enable transmit interrupt)
     bra     DONE_RCV       ; Branch to DONE_RCV:

DONE_RCV
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
     cmp     #$05          ; Compare it with decimal 5
     beq     IN_Q_MODE     ; If the Z bit of CCR is set, branch to IN_Q_MODE:

IN_A_OR_C_MODE:
     lda     secH,X      ; Load accumulator with value in address "secH",
                         ; offset in index register Lo byte
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
     bne     DONE_XFER      ; If the Z bit of CCR is clear, branch to DONE_XFER:
                            ;(Branch if NOT done to DONE_XFER !?!?!)
     clr     txcnt          ; Clear "txcnt"
     clr     txgoal         ; Clear "txgoal"
     clr     txmode         ; Clear "txmode"
     bclr    TE,SCC2        ; Clear "TE" bit of SCC2 (Disable Transmit)
     bclr    SCTIE,SCC2     ; Clear "SCTIE" bit of SCC2
                            ;(Disable transmit interrupt)

DONE_XFER
     pulh                   ; Pull value from Stack to index register Hi byte
     rti                    ; Return from interrupt


;****************************************************************************
; - ADC conversion complete Interrupt
;   ADC channel is set by "adsel" variable which starts at 0. This reads
;   channel 4, which is "BAT". When the conversion complete interrupt is
;   requested the current value in "BAT" is averaged with the result of
;   the ADC in the ADC Data Register (ADR) and stored as current "BAT"
;   This is to smooth out ADC "jitter". The "adsel" variable is then
;   incremented to the next channel and the process repeats until the 3
;   channels are read, at which time, "adsel" is set at 0 to start the
;   sequence again.
;****************************************************************************


ADC_ISR:
     bset    adcc,inputs  ; Set "adcc" bit of "inputs" variable
     pshh              ; Push index register Hi byte on to stack
                       ;(Do this because processor does not stack H)
     clrh              ; Clear index register Hi byte
     lda     adsel     ; Load accumulator with value in ADC Channel Selector
     tax               ; Transfer value in accumulator to index register Lo
     lda     ADR       ; Load accumulator with value in ADC Data Register
                       ;(this also clears conversion complete and
                       ; interrupt enable bit)
     add     BAT,x     ; Add ADR and BAT,x (Add the two values)
     rora              ; Rotate right through carry (Divide by 2)
     sta     BAT,x     ; Copy result to address BAT,x
     lda     adsel     ; Load accumulator with value in ADC Channel Selector
     inca              ; Increment accumulator
     sta     adsel     ; Copy to ADC Channel Selector Variable
     pulh              ; Pull value from stack to index register Hi byte
     rti               ; Return from interrupt


;**************************************************************************
;==========================================================================
;- Dummy ISR vector - there just to keep the assembler happy
;==========================================================================
;**************************************************************************

Dummy:
     rti     ; Return from interrupt

;****************************************************************************
;-------------------Constants not possible to burn--------------------------
;****************************************************************************

        org     $E000      ; (57344)


REVNUM:
        db      20T     ; Revision 1.0

Signature db 32T,'** MS_IAC Embedded Code by RJH *'


;****************************************************************************
; - Flash Configuration Tables and Constants (copied into RAM at start up)
;****************************************************************************

;        org     $E100      ; SE100 to $E1C0 (57600 to 57792)

ms_rf_start_f:

;****************************************************************************
; - First group of 64 bytes (not used)
;****************************************************************************

ms_rf_end_f:

;***************************************************************************
; - Boot Loader routine goes here
;***************************************************************************

     include "boot_r12.asm"       ; Include Boot Loader routine

;****************************************************************************
; - Lookup Tables
;****************************************************************************

     org     $F000     ; $F000 to $F600 (61440 to 62976)

     include "BatVolt.inc"   ; table=BatVolt:,    offset=BAT,  result=Volts
     include "IdleCntrl.inc" ; table=IACcntrl:,   offset=IAC,  result=IACpw


;***************************************************************************
; - Start of bootloader-defined jump table/vector
;***************************************************************************

     org     $FAC3              ; start bootloader-defined jump table/vector
                                ;(64195)
     db      $12                ; scbr regi init value
     db      %00000001          ; config1
     db      %00000001          ; config2
     dw      {rom_start + 256}  ; megasquirt code start
     dw      $FB00              ; bootloader start(64256)

;****************************************************************************
; - Vector table (origin vec_timebase)
;****************************************************************************

        db      $CC
	dw	Dummy          ;Time Base Vector
        db      $CC
	dw	ADC_ISR        ;ADC Conversion Complete
        db      $CC
	dw	Dummy          ;Keyboard Vector
        db      $CC
	dw	SCITX_ISR      ;SCI Transmit Vector
        db      $CC
	dw	SCIRCV_ISR     ;SCI Receive Vector
        db      $CC
	dw	Dummy          ;SCI Error Vecotr
        db      $CC
	dw	Dummy          ;SPI Transmit Vector
        db      $CC
	dw	Dummy          ;SPI Receive Vector
        db      $CC
	dw      Dummy          ;TIM2 Overflow Vector
        db      $CC
	dw	Dummy          ;TIM2 Ch1 Vector
        db      $CC
	dw	TIM2CH0_ISR    ;TIM2 Ch0 Vector
        db      $CC
	dw	Dummy          ;TIM1 Overflow Vector
        db      $CC
	dw	Dummy          ;TIM1 Ch1 Vector
        db      $CC
	dw	Dummy          ;TIM1 Ch0 Vector
        db      $CC
	dw	Dummy          ;PLL Vector
        db      $CC
	dw	Dummy          ;IRQ Vector
        db      $CC
	dw	Dummy          ;SWI Vector
        db      $CC
	dw	Start          ;Reset Vector

	end

