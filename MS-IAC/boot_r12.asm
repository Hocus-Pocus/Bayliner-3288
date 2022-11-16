;=====================================
; MEGASQUIRT BOOTLOADER VERSION - Dec 2001
;
; Mods by Bruce Bowling
;
; Corrected bug in BootReset 5
;
;=====================================

.header 'MC68HC908GP32 User Bootloader'
.base 10t
.pagewidth 130
.pagelength 90
;********************************************************************************************
;*                                                                                          *
;*  Bootloader - MC68HC908GP32                                                              *
;*                                                          Copyright (c) Motorola, 2001    *
;*                                                                                          *
;********************************************************************************************
;*                                                                                          *
;*  This file provides the low level assembly bootloader routine.                           *
;*  This program has been specially tailored towards the MC68HC908GP32.                     *
;*                                                                                          *
;********************************************************************************************
;*                                                                                          *
;*  File name:          boot.asm                        Current Release Level:      1.1     *
;*  Last Edit Date:     07-Jun-01                       Classification:             ES      *
;*                                                                                          *
;*  Include Files:      gp32.equ            : MC68HC908GP32 MCU definitions                 *
;*                                                                                          *
;*  Assembler:          P&E's CASM08Z                   Version:    3.16                    *
;*                                                                                          *
;*  Target:             MC68HC908GP32                                                       *
;*                                                                                          *
;*  Documentation:      MC68HC908GP32/H  Rev 3                                              *
;*                      Motorola Microcontroller Technical Data                             *
;*                                                                                          *
;********************************************************************************************
;*                                                                                          *
;*  Author:             DHJ Klotz                                                           *
;*  First Release:      26-Feb-00                                                           *
;*                                                                                          *
;*  Update History:                                                                         *
;*                                                                                          *
;*  Rev     Date       Author  Description of Change                                        *
;*  ------  ---------  ------  -----------------------------------------------------------  *
;*  ES 1.0  26-Feb-00  DHJK    Initial release for HC908 Seminar 2000.                      *
;*  ES 1.1  07-Jun-01  DHJK    Improved functionality for Application Note.                 *
;*                                                                                          *
;********************************************************************************************
;*                                                                                          *
;*  Notes:                                                                                  *
;*    - In order to minimize overall program size, subroutines are position within the      *
;*      core bootloader routine.  Although this can make the program somewhat difficult     *
;*      to read and follow, it permits the use of relative branch opcodes.  Most of         *
;*      these subroutines can be called from an external application program.               *
;*                                                                                          *
;********************************************************************************************
;*                                                                                          *
;*    Motorola reserves the right to make changes without further notice to any product     *
;*    herein to improve reliability, function, or design.  Motorola does not assume any     *
;*    liability arising out of the application or use of any product, circuit, or software  *
;*    described herein; neither does it convey any license under its patent rights nor the  *
;*    rights of others.  Motorola products are not designed, intended, or authorized for    *
;*    use as components in systems intended for surgical implant into the body, or other    *
;*    applications intended to support life, or for any other application in which the      *
;*    failure of the Motorola product could create a situation where personal injury or     *
;*    death may occur.  Should Buyer purchase or use Motorola products for any such         *
;*    intended or unauthorized application, Buyer shall indemnify and hold Motorola and     *
;*    its officers, employees, subsidiaries, affiliates, and distributors harmless against  *
;*    all claims, costs, damages, and expenses, and reasonable attorney fees arising out    *
;*    of, directly or indirectly, any claim of personal injury or death associated with     *
;*    such unintended or unauthorized use, even if such claim alleges that Motorola was     *
;*    negligent regarding the design or manufacture of the part.                            *
;*                                                                                          *
;*    Motorola and the Motorola logo are registered trademarks of Motorola Ltd.             *
;*                                                                                          *
;********************************************************************************************

;*  Microcontroller Peripheral Equates  *****************************************************
;*

; uncomment out if standalone
        nolist

;        include "./Gp32.equ"
        list


;*  Flash Memory Specifics  =================================================================
;*
boot_start:     equ     $FB00               ; starting address of protected Bootloader
flash_protect:  equ     {boot_start>7&$FF}  ; Flash Block Protect Register value
flash_page:     equ     128                 ; Flash Erase Page size
flash_row:      equ     64                  ; Flash Program Row size
flash_erased:   equ     $FF                 ; Flash erased state


;*  RAM Utilization  ========================================================================
;*
        org     ram_start                   ; begining of RAM

count:          ds      1                   ; 0040:     => data counter
temp_sp:        ds      2                   ; 0041:0042 => temporary Stack Pointer storage
flash_first:    ds      2                   ; 0043:0044 => first Flash reprogram address
flash_last:     ds      2                   ; 0045:0046 => last Flash reprogram address + 1

ram_exec:       equ     $01ED               ; start of executable RAM space


;*  Bootloader Customization Parameters  ====================================================
;*
user_scbr:      equ     boot_start-61       ; FAC3      => SCBR register
init_scbr:      equ     $12                 ;   default set SCI for 9600 kbaud

user_config1:   equ     boot_start-60       ; FAC4      => CONFIG1 register
init_config1:   equ     %00000001           ;   default CONFIG1

user_config2:   equ     boot_start-59       ; FAC5      => CONFIG2 register
init_config2:   equ     %00000001           ;   default CONFIG2

user_first:     equ     boot_start-58       ; FAC6:FAC7 => 1st application address
init_first:     equ     rom_start           ;   default first Flash address

user_last:      equ     boot_start-56       ; FAC8:FAC9 => last application address
init_last:      equ     boot_start          ;   default last Flash address


;*  Application Program Jump Vector Table  ==================================================
;*
                                            ; FACA      => "JMP ext" instruction (opcode $CC)
user_timebase:  equ     boot_start-54       ; FACB:FACC => user Timebase jump vector

                                            ; FACD      => "JMP ext" instruction (opcode $CC)
user_ADC:       equ     boot_start-51       ; FACE:FACF => user ADC jump vector

                                            ; FAD0      => "JMP ext" instruction (opcode $CC)
user_keyboard:  equ     boot_start-48       ; FAD1:FAD2 => user Keyboard jump vector

                                            ; FAD3      => "JMP ext" instruction (opcode $CC)
user_SCItx:     equ     boot_start-45       ; FAD4:FAD5 => user SCI transmit jump vector

                                            ; FAD6      => "JMP ext" instruction (opcode $CC)
user_SCIrx:     equ     boot_start-42       ; FAD7:FAD8 => user SCI receive jump vector

                                            ; FAD9      => "JMP ext" instruction (opcode $CC)
user_SCIerr:    equ     boot_start-39       ; FADA:FADB => user SCI error jump vector

                                            ; FADC      => "JMP ext" instruction (opcode $CC)
user_SPItx:     equ     boot_start-36       ; FADD:FADE => user SPI transmit jump vector

                                            ; FADF      => "JMP ext" instruction (opcode $CC)
user_SPIrx:     equ     boot_start-33       ; FAE0:FAE1 => user SPI receive jump vector

                                            ; FAE2      => "JMP ext" instruction (opcode $CC)
user_Tim2Ov:    equ     boot_start-30       ; FAE3:FAE4 => user Timer 2 overflow jump vector

                                            ; FAE5      => "JMP ext" instruction (opcode $CC)
user_Tim2Ch1:   equ     boot_start-27       ; FAE6:FAE7 => user Timer 2 channel 1 jump vector

                                            ; FAE8      => "JMP ext" instruction (opcode $CC)
user_Tim2Ch0:   equ     boot_start-24       ; FAE9:FAEA => user Timer 2 channel 0 jump vector

                                            ; FAEB      => "JMP ext" instruction (opcode $CC)
user_Tim1Ov:    equ     boot_start-21       ; FAEC:FAED => user Timer 1 oveflow jump vector

                                            ; FAEE      => "JMP ext" instruction (opcode $CC)
user_Tim1Ch1:   equ     boot_start-18       ; FAEF:FAF0 => user Timer 1 channel 1 jump vector

                                            ; FAF1      => "JMP ext" instruction (opcode $CC)
user_Tim1Ch0:   equ     boot_start-15       ; FAF2:FAF3 => user Timer 1 channel 0 jump vector

                                            ; FAF4      => "JMP ext" instruction (opcode $CC)
user_PLL:       equ     boot_start-12       ; FAF5:FAF6 => user PLL jump vector

                                            ; FAF7      => "JMP ext" instruction (opcode $CC)
user_IRQ:       equ     boot_start-9        ; FAF8:FAF9 => user IRQ jump vector

                                            ; FAFA      => "JMP ext" instruction (opcode $CC)
user_SWI:       equ     boot_start-6        ; FAFB:FAFC => user SWI jump vector

                                            ; FAFD      => "JMP ext" instruction (opcode $CC)
user_reset:     equ     boot_start-3        ; FAFE:FAFF => user Reset interrupt jump vector


;*  Bootloader Program  *********************************************************************
;*

init_stack:     equ     ram_exec-1          ; initialize stack pointer to before RAM routine
;
init_scc1:      equ     %01000000           ; enable SCI, 8-bits, no parity, 1 stop
init_scc2:      equ     %00001100           ; no interupts, receiver and transmitter enabled

        org     boot_start                  ; beginning of code


;*  CGM Parameter Tables  ===================================================================
;*
;*  The following CGM parameter tables are placed here so that they are easy to access via
;*  external application programs.
;*
;*  7.3728 MHz bus frequency parameters (located at address "boot_start").
;*
bus7372800:
        db      $02                         ; P & E
        db      $C0                         ; L
        db      $03                         ; N msb
        db      $84                         ; N lsb

;*  8.003584 MHz bus frequency parameters (located at address "boot_start+4").
;*
bus8003584:
        db      $02                         ; P & E
        db      $D0                         ; L
        db      $03                         ; N msb
        db      $D1                         ; N lsb


;*  Power-on Reset  =========================================================================
;* MODIFIED FOR MEGASQUIRT - Initialization code here
;*

BootReset:
        clra
        sta     copctl
        mov     #%00000001,config2
        mov     #%00000001,config1
        ldhx    #ram_last+1
        txs

        ldhx    #bus7372800                 ; point to 7.3728 MHz parameters
        bsr     PLLset                      ; change bus speed

        lda      #%00000000
        sta      ddrb                       ; ADC Channels - inputs

        lda     #%01110000                  ; Set up ADC for divide by 8 and internal clock
        sta     adclk
        lda     #%00000100                  ; No interrupt, channel AD4 selected
        sta     adscr
        brclr   coco,adscr,*                ; wait until conversion complete

        lda     adr
        cmp     #$05                        ; Check for low voltage on divider
        blo     BootReset1                  ; enter bootloader if low voltage

;
;   Test application reset vector.
;
        lda     user_reset+1                ; get the MSB of the user reset vector
        cmp     #flash_erased               ; check if it's erased
        beq     BootReset1                  ; enter bootloader if erased
        bra     user_reset                  ; else, jump to user reset jump vector


;*  External CGM PLL Bus Frequency Change Subroutine  =======================================
;*
;*  This subroutine will program the CGM PLL to change the bus frequency in accordance with
;*  the data being pointed to by X:A (which is a common implementation for pointer parameter
;*  passing used by HC08 C compilers).
;*
;*  C function prototype:
;*
;*      void CGMChange (char parameters*);
;*
;*  Calling convention:
;*
;*      ldx     #{parameters>8}             ; get CGM parameter table address msb
;*      lda     #{parameters&$FF}           ; get CGM parameter table address lsb
;*      jsr     CGMChange                   ; go change the bus speed
;*
;*  Returns:    nothing
;*
;*  Changes:    H:X
;*
CGMChange:
        psha                                ; save pointer lsb on stack
        pshx                                ; save pointer msb on stack
        pulh                                ; initialize
        pulx                                ;  H:X points to data array


;*  Internal CGM PLL Bus Frequency Change Subroutine  =======================================
;*
;*  This subroutine will program the CGM PLL to change the bus frequency in accordance with
;*  the data being pointed to by H:X.
;*
;*  Calling convention:
;*
;*      ldhx    #parameters                 ; point to CGM parameter table
;*      jsr     PLLset                      ; go change the bus speed
;*
;*  Returns:    nothing
;*
;*  Changes:    H:X
;*
PLLset:
        bclr    BCS,pctl                    ; select external reference as base clock
        bclr    PLLON,pctl                  ; turn off PLL
        mov     x+,pctl                     ; program P & E
        mov     x+,pmrs                     ; program L
        mov     x+,pmsh                     ; program N msb
        mov     x+,pmsl                     ; program N lsb
        bset    AUTO,pbwc                   ; enable automatic bandwidth control
        bset    PLLON,pctl                  ; turn on PLL
PLLwait:
        brclr   LOCK,pbwc,PLLwait           ; wait for PLL to lock (Note: won't simulate)
        bset    BCS,pctl                    ; select VCO as base clock
        rts                                 ; return


;*  PutChar Subroutine  =====================================================================
;*
;*  This subroutine will output the character passed in ACC to the SCI.
;*
;*  C function prototype:
;*
;*      void PutChar (char data);
;*
;*  Calling convention:
;*
;*      lda     data                        ; get character
;*      jsr     PutChar                     ; go output it
;*
;*  Returns:    nothing
;*
;*  Changes:    nothing
;*
PutChar:
        brclr   SCTE,scs1,PutChar           ; wait until SCI transmitter is empty
        sta     scdr                        ; output character to the SCI
        rts                                 ; return


;*  Power-on Reset Bootloader Entry  ========================================================
;*
;*  This is where the Bootloader starts from power-on reset.
;*
BootReset1:
;
;   Initialize the PLL CGM for 7.3728 MHz bus speed from 32.768 kHz crystal.
;
;        ldhx    #bus7372800                 ; point to 7.3728 MHz parameters
;        bsr     PLLset                      ; change bus speed
;
;   Copy user Flash parameters into RAM.
;
        ldhx    #user_scbr                  ; point to first parameter
        mov     x+,count                    ; copy user SCI baud rate
        mov     x+,temp_sp                  ; copy user Configuration Register 1
        mov     x+,temp_sp+1                ; copy user Configuration Register 2
        mov     x+,flash_first              ; copy first user Flash address MSB
        mov     x+,flash_first+1            ; copy first user Flash address LSB
        mov     x+,flash_last               ; copy last user Flash address MSB
        mov     x+,flash_last+1             ; copy last user Flash address LSB
        ldhx    #count                      ; point to first parameter, now saved in RAM
        txs                                 ; use SP to point to parameter list in RAM
;
;   Test the user SCI baud rate.  The user can override the default baud rate.
;
        pula                                ; get user SCBR initial data
        cmp     #flash_erased               ; check if it's erased
        bne     BootReset2                  ; skip if not
        lda     #init_scbr                  ; else, force default value
BootReset2:
        sta     count                       ; save initial SCI baud rate
;
;   Program the write-once configuration registers.  The user can override the defaults.
;
        pula                                ; get user Configuration Register 1 initial data
        cmp     #flash_erased               ; check if it's erased
        bne     BootReset3                  ; skip if not
        lda     #init_config1               ; else, force default value
BootReset3:
        sta     config1                     ; initialize Configuration Register 1
;
        pula                                ; get user Configuration Register 2 initial data
        cmp     #flash_erased               ; check if it's erased
        bne     BootReset4                  ; skip if not
        lda     #init_config2               ; else, force default value
BootReset4:
        sta     config2                     ; initialize Configuration Register 2
;
;   Program the first and last user Flash addresses.  The user can override the defaults.
;
        pulx                                ; get first user Flash address LSB
        pulh                                ; get first user Flash address MSB
        cphx    #$FFFF                      ; check if it's erased
        bne     BootReset5                  ; skip if not
        lda     #{init_first&$FF}           ; else, get default first user address LSB
        psha                                ;  save it
        lda     #{init_first>8}             ;  and get default first user address MSB
        psha                                ;  save it
        ais     #2                          ; move stack pointer back
;
BootReset5:
        pulx                                ; get last user Flash address LSB
        pulh                                ; get last user Flash address MSB
        cphx    #$FFFF                      ; check if it's erased
        bne     BootReset6                  ; skip if not
;        ldx     #{init_last&$FF}            ; else, get default last user address LSB
        lda     #{init_last&$FF}            ; else, get default last user address LSB
        psha                                ;  save it
        lda     #{init_last>8}              ;  and get default last user address MSB
        psha                                ;  save it
BootReset6:


;*  User Bootloader Entry  ==================================================================
;*
;*  The user can launch the bootloader from here.
;*
BootResetUser:
        sei                                 ; disable all interrupts
        sta     copctl                      ; clear the COP counter
        ldhx    #init_stack+1               ; initialize
        txs                                 ;  the stack pointer
;
;   Initialize the PLL CGM for 7.3728 MHz bus speed from 32.768 kHz crystal.
;
        ldhx    #bus7372800                 ; point to 7.3728 MHz parameters
        bsr     PLLset                      ; change bus speed
;
;   Take over and initialize the SCI.  The user can override the default baud rate.
;
        mov     count,scbr                  ; initialize SCI baud rate
        mov     #init_scc1,scc1             ; initialize SCI Control Register 1
        mov     #init_scc2,scc2             ; initialize SCI Control Register 2


;*  Main Bootloader Control Loop  ==========================================================
;*
;*  Bootloader program supports the following commands:
;*
;*      'X'  = Exit and execute user program via user reset vector
;*      'P'  = Program Flash via S-Records
;*      'W'  = Erase Flash (Wipe)
;*      'U'  = Upgrade Flash by erasing all user space, then programming via S-Records
;*      'H'  = Help
;*      '?'  = Help
;*
;*  Note: avoid using 'A' - 'F', as these are valid S-Record characters that could get
;*        misinterpreted.
;*
cmd_exit:       equ     'X'                 ; Exit command
cmd_program:    equ     'P'                 ; Program Flash command
cmd_erase:      equ     'W'                 ; Erase Flash command (Wipe)
cmd_upgrade:    equ     'U'                 ; Upgrade Flash command
cmd_help:       equ     'H'                 ; Help command
cmd_help1:      equ     $1F                 ; '?' = alternate Help command
;
Boot:
        ldhx    #msg_hello                  ; point to hello message
        bsr     PrintString                 ; output it
        jsr     GetChar                     ; get a character from the SCI
        cmp     #ascii_CR                   ; check for ASCII carriage return
        beq     Boot                        ; just loop back if so
        bsr     PutChar                     ; else, echo character back
        and     #$DF                        ; convert to uppercase


;*  Execute User Program Command Check  =====================================================
;*
        cmp     #cmd_exit                   ; check for Exit command
        bne     Boot2                       ; skip if not
        lda     user_reset+1                ; else, get the MSB of the user reset vector
        cmp     #flash_erased               ; check if it's erased
        beq     Boot1                       ; skip if not
        jmp     user_reset                  ; else, jump to user reset jump vector
;
;   Remain in the Bootloader if the MSB of the User Reset Jump Vector is erased.
;
Boot1:
        ldhx    #msg_noreset                ; point to error message
        bsr     PrintString                 ; output it
        bra     Boot                        ; jump back to top


;*  Erase Flash Command Check  ==============================================================
;*
Boot2:
        cmp     #cmd_erase                  ; check for Erase Flash command
        bne     Boot3                       ; skip if not
        bsr     EraseFlash                  ; else, go erase Flash
;
;   Common Bootloader command completion points.
;
BootDone:
        ldhx    #msg_complete               ; point to operation complete message
BootDone1:
        bsr     PrintString                 ; output it
BootDone2:
        bra     Boot                        ; jump back to top


;*  External PutString Subroutine  ==========================================================
;*
;*  This subroutine will output the null terminated string pointed to by X:A (which is a
;*  common implementation for pointer parameter passing used by HC08 C compilers) to the SCI.
;*
;*  C function prototype:
;*
;*      void PutString (char string*);
;*
;*  Calling convention:
;*
;*      ldx     #{string>8}                 ; get CGM parameter table address msb
;*      lda     #{string&$FF}               ; get CGM parameter table address lsb
;*      jsr     PutString                   ; go change the bus speed
;*
;*  Returns:    nothing
;*
;*  Changes:    H:X
;*
PutString:
        psha                                ; save pointer lsb on stack
        pshx                                ; save pointer msb on stack
        pulh                                ; initialize
        pulx                                ;  H:X points to data array
        bra     PrintString                 ; go output string


;*  PrintString Subroutine  =================================================================
;*
;*  This subroutine will output the null teminated string pointed to by H:X to the SCI.
;*
;*  Calling convention:
;*
;*      ldhx    #string                     ; point to start of string
;*      jsr     PrintString                 ; go output it
;*
;*  Returns:    nothing
;*
;*  Changes:    H:X
;*
PrintString1:
        brclr   SCTE,scs1,PrintString1      ; wait until SCI transmitter is empty
        mov     x+,scdr                     ; output character to the SCI and advance pointer
PrintString:
        tst     ,x                          ; test string character
        bne     PrintString1                ; loop back if not null
        rts                                 ; else, return


;*  Program Flash Command Check  ============================================================
;*
Boot3:
        cmp     #cmd_program                ; check for Program Flash command
        bne     Boot4                       ; skip if not
;
        bsr     BootProg                    ; go accept S19 records and program the Flash
        bra     Boot                        ; return to top of control loop


;*  Program Flash Subroutine  ===============================================================
;*
;*  This subroutine will copy the Flash Program algorithm into RAM and execute it in
;*  conjunction with the S19 record retrieval to program the required Flash pages between
;*  address pointers "flash_first" and "flash_last".
;*
;*  Calling convention:
;*
;*      jsr     BootProg                    ; retrieve S19 records and program Flash
;*
;*  Returns:    nothing
;*
;*  Changes:    everything
;*
BootProg:
        ldhx    #ProgramRamSize             ; initialize pointer
BootProg1:
        lda     Delay-1,x                   ; get program from Flash
        sta     ram_exec-1,x                ; copy into RAM
        dbnzx   BootProg1                   ; decrement pointer and loop back until done
        ldhx    #msg_waiting                ; point to waiting message
        bsr     PrintString                 ; output it
;
;   Get S-Record from host.
;
BootProg2:
        tsx                                 ; get the Stack Pointer
        sthx    temp_sp                     ; save it temporarily
        ais     #-36                        ; allocate stack space for data
        bsr     GetSRec                     ; get an S-Record
        bne     BootProg5                   ; indicate error if S-Record is invalid
        pula                                ; get S-Record type
        cmp     #'0'                        ; check for text header record type
        beq     BootProg3                   ; ignore and get next record
        cmp     #'9'                        ; check for end record type
        beq     BootProg4                   ; indicate operation complete
        cmp     #'1'                        ; check for data record type
        bne     BootProg5                   ; indicate error if S-Record is invalid
;
;   Program Flash.
;
        jsr     {ram_exec+ProgramRam}       ; execute Program Flash algorithm from RAM
BootProg3:
        ais     #35                         ; deallocate stack space
        bra     BootProg2                   ; loop back for next S-Record
;
BootProg4:
        ais     #35                         ; deallocate stack space
        brclr   SCRF,scs1,BootDone          ; skip if SCI receiver is empty
        bsr     GetChar                     ; else, clear last ASCII carriage return from SCI
        brclr   SCRF,scs1,BootDone          ; skip if SCI receiver is empty
        bsr     GetChar                     ; else, clear last ASCII line feed from the SCI
        ldhx    #msg_complete               ; point to operation complete message
        bra     BootProg6                   ; go output it
;
BootProg5:
        ais     #36                         ; deallocate stack space
        ldhx    #msg_error                  ; point to error message
BootProg6:
        bsr     PrintString                 ; output it
        rts                                 ; return


;*  Upgrade Flash Command Check  ============================================================
;*
Boot4:
        cmp     #cmd_upgrade                ; check for Upgrade Flash command
        bne     Boot5                       ; skip if not
;
        ldhx    #init_first                 ; force
        sthx    flash_first                 ;  first Flash address
        ldhx    #init_last                  ; force
        sthx    flash_last                  ;  last Flash address
        bsr     EraseFlash                  ; go erase Flash
        bra     BootProg                    ; go program Flash


;*  Multiple Flash Page Erase Subroutine  ===================================================
;*
;*  This subroutine will copy the Flash Erase algorithm into RAM and execute it to erase
;*  all pages between address pointers "flash_first" and "flash_last".
;*
;*  Calling convention:
;*
;*      ldhx    #init_first                 ; initialize
;*      sthx    flash_first                 ;  first Flash address
;*      ldhx    #init_last                  ; initialize
;*      sthx    flash_last                  ;  last Flash address
;*      jsr     EraseFlash                  ; go erase flash
;*
;*  Returns:    nothing
;*
;*  Changes:    everything
;*
EraseFlash:
        ldhx    #EraseRamSize               ; initialize pointer
EraseFlash1:
        lda     MassErase-1,x               ; get program from Flash
        sta     ram_exec-1,x                ; copy into RAM
        dbnzx   EraseFlash1                 ; decrement pointer and loop back until done
        jmp     ram_exec                    ; execute Flash Mass Erase algorithm from RAM


;*  GetChar Subroutine  =====================================================================
;*
;*  This subroutine will wait forever for a character to be received by the SCI and then
;*  returns with that character in ACC.  No error checking is performed.  Note that this
;*  is the primary loop where the COP counter is cleared.
;*
;*  C function prototype:
;*
;*      char GetChar (void);
;*
;*  Calling convention:
;*
;*      jsr     GetChar                     ; get a character from the SCI
;*
;*  Returns:
;*      ACC= data
;*
GetChar:
        sta     copctl                      ; clear the COP counter
        brclr   SCRF,scs1,GetChar           ; wait forever until SCI receiver is full
        lda     scdr                        ; get data
        rts                                 ; return


;*  GetSRec Subroutine  =====================================================================
;*
;*  This subroutine will retrieve data in S19 record format via the SCI.
;*
;*  Calling convention:
;*
;*      ais     #-buffer_length             ; allocate stack space for data
;*      jsr     GetSRec                     ; go get S-record data
;*
;*  Returns:    CCRZ= 1 if valid S-Record retrieved.  Otherwise, CCRZ= 0.
;*              S-Record Type at SP+1     (1 byte)
;*              S-Record Size at SP+2     (1 byte)
;*              S-Record Address at SP+3  (2 bytes)
;*              S-Record Data at SP+5     (up to 32 bytes, typically)
;*
;*              |                |    <-sp (after local space allocation)
;*      H:X->   | SRecCount      |
;*              | SRecChkSum     |    <-sp (when called)
;*              | ReturnAddr msb |
;*              | ReturnAddr lsb |    <-sp (upon return)
;*              | SRecType       |
;*              | SRecSize       |
;*      H:X->   | SRecAddr msb   |
;*              | SRecAddr lsb   |
;*              | SRecData 00    |
;*              | SRecData 01    |  etc..
;*
;*  Changes:    everything
;*
SRecCount:      equ     1                   ; stack pointer offset for S-Record Counter
SRecChkSum:     equ     2                   ; stack pointer offset for S-Record Check Sum
SRecType:       equ     5                   ; stack pointer offset for S-Record Type
SRecSize:       equ     6                   ; stack pointer offset for S-Record Size
SRecAddr:       equ     7                   ; stack pointer offset for S-Record Address
SRedData:       equ     8                   ; stack pointer offset for S-Record Data
;
GetSRec:
        ais     #-2                         ; allocate local variable space
        clr     SRecSize,sp                 ; initialize S-Record size
GetSRec1:
        bsr     GetChar                     ; get a character from the SCI
        cmp     #ascii_CR                   ; check for ASCII carriage return
        bne     GetSRec1a                   ; just loop back if so
        lda     #ascii_LF                   ; get ASCII line feed
GetSRec1a:
        cmp     #'S'                        ; check for start of record character
        bne     GetSRec1                    ; loop back if not
        bsr     GetChar                     ; else, get next character from the SCI
        cmp     #'0'                        ; check for header record type
        beq     GetSRec1                    ; loop back if so
        cmp     #'9'                        ; else, check for end record type
        beq     GetSRec2                    ; continue if so
        cmp     #'1'                        ; else, check for data record type
        bne     GetSRec1                    ; loop back if not
GetSRec2:
        sta     SRecType,sp                 ; save S-Record type
        bsr     GetHexByte                  ; get the S-Record length
        bne     GetSRec4                    ; exit if not a valid hex byte
        sta     SRecCount,sp                ; initialize S-Record counter
        sta     SRecChkSum,sp               ; initialize S-Record check sum
        sub     #3                          ; adjust for address and checksum
        sta     SRecSize,sp                 ; save S-Record size
        tsx                                 ; use H:X as data stack frame pointer
        aix     #{SRecAddr-1}               ; adjust so pointer starts at S-Record Address
GetSRec3:
        bsr     GetHexByte                  ; get next S-Record hex byte
        bne     GetSRec4                    ; exit if not a valid hex byte
        sta     ,x                          ; save data in stack frame
        add     SRecChkSum,sp               ; add data to check sum
        sta     SRecChkSum,sp               ; save new check sum
        aix     #1                          ; move data stack frame pointer
        dbnz    SRecCount,sp,GetSRec3       ; loop back until all data has been received
        inc     SRecChkSum,sp               ; final calculation zeros check sum if it's okay
GetSRec4:
        ais     #2                          ; deallocate local variables
        rts                                 ; return


;*  Help Command Response  ==================================================================
;*
Boot5:
        cmp     #cmd_help                   ; check for Help command
        beq     Boot6                       ; continue if so
        cmp     #cmd_help1                  ; check for alternate Help command
        bne     Boot7                       ; skip if not
boot6:
        ldhx    #msg_help                   ; point to Help command message
        jmp     BootDone1                   ; go output it


;*  Unknown Command Response  ===============================================================
;*
Boot7:
        ldhx    #msg_what                   ; point to unknown command message
        jmp     BootDone1                   ; go output it


;*  GetHexByte Subroutine  ==================================================================
;*
;*  This subroutine retrieves two ASCII bytes via the SCI and converts (packs) them into one
;*  hex byte, which is returned in ACC.
;*
;*  Calling convention:
;*
;*      jsr     GetHexByte
;*
;*  Returns:    CCRZ= 1 if valid hex byte retrieved.  Otherwise, CCRZ= 0.
;*              ACC= data
;*
;*  Changes:    ACC
;*
GetHexByte:
        bsr     GetChar                     ; get msb character from the SCI
        bsr     IsHex                       ; check if valid ASCII hex character
        bne     GetHexByte2                 ; exit if not
        bsr     ToHex                       ; convert ASCII hex character to hex value
        nsa                                 ; swap lower nibble up
        psha                                ; save temporarily
        jsr     GetChar                     ; get lsb character from the SCI
        bsr     IsHex                       ; check if valid ASCII hex character
        bne     GetHexByte1                 ; exit if not
        bsr     ToHex                       ; convert ASCII hex character to hex value
        add     1,sp                        ; combine msb and lsb nibbles
        bit     #0                          ; CCRZ= 1
GetHexByte1:
        ais     #1                          ; deallocate local variable
GetHexByte2:
        rts                                 ; return


;*  ToHex Subroutine  =======================================================================
;*
;*  This subroutine converts the ASCII hex value passed in ACC to a binary hex value.
;*
;*  Calling convention:
;*
;*      lda     data
;*      jsr     ToHex
;*
;*  Returns:    ACC= data.
;*
;*  Changes:    ACC
;*
ToHex:
        sub     #'0'                        ; adjust first by subtracting '0'
        cmp     #9                          ; check if value was between '0' to '9'
        bls     ToHex1                      ; exit if so
        sub     #7                          ; else, adjust for value between 'A' to 'F'
ToHex1:
        rts                                 ; return


;*  IsHex Subroutine  =======================================================================
;*
;*  This subroutine checks if the value passed in ACC is a valid ASCII hex character within
;*  within the ranges of '0' to '9' or 'A' to 'F'.  Note that the range 'a' to 'f' is not
;*  checked.
;*
;*  Calling convention:
;*
;*      lda     data
;*      jsr     IsHex
;*
;*  Returns:    CCRZ= 1 if data is a valid hex character.  Otherwise, CCRZ= 0.
;*
;*  Changes:    nothing
;*
IsHex:
        cmp     #'0'                        ; check value against '0'
        blo     IsntHex                     ; not hex if lower
        cmp     #'9'                        ; check value against '9'
        bls     IsHex1                      ; is hex if lower
        cmp     #'A'                        ; check value against 'A'
        blo     IsntHex                     ; not hex if lower
        cmp     #'F'                        ; check value against 'F'
        bhi     IsntHex                     ; not hex if higher
IsHex1:
        bit     #0                          ; CCRZ= 1
IsntHex:
        rts                                 ; return


;*  Flash Mass Erase Subroutine  ============================================================
;*
;*  This subroutine performs multiple Page Erase operations in order to erase the application
;*  space Flash memory between "flash_first" and "flash_last".  This subroutine has been
;*  tuned for a bus speed of 7.3728 MHz.
;*  This subroutine is copied into and executed from RAM.
;*
MassErase:
        ldhx    flash_last                  ; initialize pointer to last Flash memory address
        bra     MassErase2                  ; go move pointer before erasing Flash
MassErase1:
;
;   Set ERASE, read the Flash Block Protect Register and write any data into Flash page.
;
        lda     #{ERASE}                    ; set ERASE control bit
        sta     flcr                        ;  in Flash Control Register
        lda     flbpr                       ; read from Flash Block Protect Register
        sta     ,x                          ; write any data to address within page
;
;   Wait for >10us, then set HVEN.
;
        lda     #1                          ; wait
        bsr     delay                       ;  for 11.7us
        lda     #{ERASE | HVEN}             ; set HVEN control bit
        sta     flcr                        ;  in Flash Control Register
;
;   Wait for >1ms, then clear ERASE.
;
        lda     #100                        ; wait
        bsr     delay                       ;  for 1.005ms
        lda     #{HVEN}                     ; clear ERASE control bit
        sta     flcr                        ;  in Flash Control Register
;
;   Wait for >5us, then clear HVEN.
;
        lda     #1                          ; wait
        bsr     delay                       ;  for 11.7us
        clra                                ; clear HVEN control bit
        sta     flcr                        ;  in Flash Control Register
;
;   Advance pointer and repeat until finished.
;
MassErase2:
        aix     #-64                        ; move pointer back
        aix     #-64                        ;  by one complete erase page
        cphx    flash_first                 ; check if finished
        bhi     MassErase1                  ; loop back if not
;
        rts                                 ; return


;*  Delay Subroutine  =======================================================================
;*
;*  This subroutine performs a simple software delay loop based upon the value passed in ACC.
;*  The following timing calculation applies:
;*
;*              delay = ((ACC * 74) + 12) (tcyc)
;*
;*  Calling convention:
;*
;*      lda     data
;*      jsr     delay
;*
;*  Returns:    nothing
;*
;*  Changes:    ACC
;*
Delay:
        psha                                ; [2] save outer delay loop parameter
Delay1:
        lda     #22                         ; [2] initialize inner delay loop counter
Delay2:
        dbnza   Delay2                      ; [3] decrement inner delay loop counter
        dbnz    1,sp,Delay1                 ; [6] decrement outer delay loop counter
        pula                                ; [2] deallocate local variable
        rts                                 ; [4] return

EraseRamSize:   equ     {*-MassErase}
ProgramRam:     equ     {*-Delay}


;*  Flash Program Subroutine  ===============================================================
;*
;*  This subroutine controls the Flash programming sequence.  A stack frame data block is
;*  passed to it in the format shown below.  This subroutine has been tuned for a bus speed
;*  of 7.3728 MHz.
;*  This subroutine is copied into and executed from RAM.
;*
;*              |                |    <-sp (when called)
;*              | ReturnAddr msb |
;*              | ReturnAddr lsb |    <-sp (upon return)
;*              | SRecSize       |
;*              | SRecAddr msb   |
;*              | SRecAddr lsb   |
;*              | SRecData 00    |
;*              | SRecData 01    |  etc..
;*
FlashProgram:
        tsx                                 ; get the Stack Pointer
        sthx    temp_sp                     ; save it temporarily
;
;   Get S-Record size and use the Stack Pointer as the data source pointer.
;
        ais     #2                          ; SP points to SRecSize
        pula                                ; get SRecSize
        sta     count                       ; save it temporarily
;
;   Establish H:X as the destination pointer.
;
        pulh                                ; get destination address msb
        pulx                                ; get destination address lsb

FlashProgram1:
        cphx    flash_first                 ; check against minimum address
        blo     FlashProgram2               ; skip if lower
        cphx    flash_last                  ; check against maximum address
        bhs     FlashProgram2               ; skip if the same or higher
;
;   Set PGM, read the Flash Block Protect Register and write anywhere in desired Flash row.
;
        lda     #{PGM}                      ; set PGM control bit
        sta     flcr                        ;  in Flash Control Register
        lda     flbpr                       ; read from Flash Block Protect Register
        sta     ,x                          ; write any data to first Flash address
;
;   Wait for >10us, then set HVEN.
;
        lda     #1                          ; wait
        bsr     delay                       ;  for 11.7us
        lda     #{PGM | HVEN}               ; set HVEN control bit
        sta     flcr                        ;  in Flash Control Register
;
;   Wait for >5us.
;
        lda     #1                          ; wait
        bsr     delay                       ;  for 11.7us
;
;   Write data to Flash and wait for 30 - 40us.
;
        pula                                ; get S-Record data
        sta     ,x                          ; write data to Flash
        lda     #3                          ; wait
        bsr     delay                       ;  for 31.7us
;
;   Clear PGM.
;
        lda     #{HVEN}                     ; clear PGM
        sta     flcr                        ;  in Flash Control Register
;
;   Wait for >5us, then clear HVEN.
;
        lda     #1                          ; wait
        bsr     delay                       ;  for 11.7us
        clra                                ; clear HVEN control bit
        sta     flcr                        ;  in Flash Control Register
;
;   Advance destination pointer and data counter.
;
FlashProgram2:
        aix     #1                          ; advance destination pointer
        dbnz    count,FlashProgram1         ; decrement counter and loop back if not done.
;
        ldhx    temp_sp                     ; restore
        txs                                 ;  Stack Pointer
        rts                                 ; return

ProgramRamSize: equ     {*-Delay}


;*  Messages  ===================================================================================
;*
ascii_CR:       equ     $0D                 ; ASCII carriage return
ascii_LF:       equ     $0A                 ; ASCII line feed
;
msg_hello:      db      ascii_CR,ascii_LF,'Boot>',0
msg_help:       db      '  (P)rogram (W)ipe (U)pgrade e(X)it',0
;
msg_complete:   db      '  Complete',0
msg_waiting:    db      ' - waiting ...',0
msg_error:      db      ' - error',0
msg_what:       db      ' - what?',0
msg_noreset:    db      ' - Reset Vector Invalid',0

;
;   Last location not to exceed $FDFF
;
BootEnd:


;*  Vectors  ************************************************************************************
;*
        org     vec_timebase                ; Timebase vector
        dw      user_timebase
        org     vec_adc                     ; ADC vector
        dw      user_ADC
        org     vec_kbd                     ; Keyboard vector
        dw      user_keyboard
        org     vec_scitx                   ; SCI transmit vector
        dw      user_SCItx
        org     vec_scirx                   ; SCI receive vector
        dw      user_SCIrx
        org     vec_scierr                  ; SCI error vector
        dw      user_SCIerr
        org     vec_spitx                   ; SPI transmit vector
        dw      user_SPItx
        org     vec_spirx                   ; SPI receive vector
        dw      user_SPIrx
        org     vec_tim2ov                  ; Timer 2 overflow vector
        dw      user_Tim2Ov
        org     vec_tim2ch1                 ; Timer 2 channel 1 vector
        dw      user_Tim2Ch1
        org     vec_tim2ch0                 ; Timer 2 channel 0 vector
        dw      user_Tim2Ch0
        org     vec_tim1ov                  ; Timer 1 oveflow vector
        dw      user_Tim1Ov
        org     vec_tim1ch1                 ; Timer 1 channel 1 vector
        dw      user_Tim1Ch1
        org     vec_tim1ch0                 ; Timer 1 channel 0 vector
        dw      user_Tim1Ch0
        org     vec_pll                     ; PLL vector
        dw      user_PLL
        org     vec_irq                     ; IRQ vector
        dw      user_IRQ
        org     vec_swi                     ; SWI vector
        dw      user_SWI
        org     vec_reset                   ; Reset vector
        dw      BootReset


;*  Flash Block Protect Register  ***************************************************************
;*
        org     flbpr
        db      flash_protect

        end
