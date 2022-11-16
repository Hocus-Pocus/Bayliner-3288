;****************************************************************************************
; Megasquirt Flash page erase and programming routines. Heavily based on the routines
; from boot_r12.asm
;
; JSM - revised timing for 8MHz
; RJH - revised for MS_ECU_HP 10/28/10
;****************************************************************************************
;****************************************************************************************
; - Variables
;
; page      - Page address pointer when added to $E000(MS_ECU_HP.h)
; burnDst   - Burn destination address(MS_ECU_HP.h)
; burnSrc   - Burn source address(MS_ECU_HP.h)
; burncount - Number of bytes to burn(MS_ECU_HP.h)
; ram_exec  - Start of execurable RAM space($01ED)(493)(boot_r12)
; flcr      - HC908 Flash Control Register
; HVEN      = High Voltage Enable Bit of flcr
; ERASE     = Erase Control bit of flcr
; PGM       = Program Control bit of flcr
; flbpr     - HC908 Flash Block Protect Register
;
;****************************************************************************************
;****************************************************************************************
; - Erase 256 bytes in Flash starting at the selected page address
;****************************************************************************************

burnConst:
     lda     flocker
     cmp     #$CC
     beq     BURN_CONT
     rts

BURN_CONT:
     lda     page     ; Load accumulaotr with value in "page"
     add     #$DF
;*     add     #$E0     ; A<-(A)+(M) Add value in accumulator with with $E0
     psha             ; Push result to Stack
     pulh             ; Pull result from stack and copy 
     clrx             ; Clear index register Lo byte(now a 16 bit value)
     sthx    burnDst  ; Copy to "burnDst"

;****************************************************************************************
; - Erase the first block of 128 bytes
;****************************************************************************************

     jsr     ms_EraseFlash     ; Jump to subroutine at ms_EraseFlash:

;****************************************************************************************
; - Load the address of the second 128 bytes of the page to be erased
;****************************************************************************************

     ldhx    burnDst     ; Load index register with value in "burnDst"
     aix     #64T        ; Add decimal 64 to index register Lo byte
     aix     #64T        ; Add decimal 64 to index register Lo byte (total of 128)
     sthx    burnDst     ; Copy result to "burnDst"

;****************************************************************************************
; - Erase the second block of 128 bytes
;****************************************************************************************

     jsr     ms_EraseFlash     ; Jump to subroutine at ms_EraseFlash:

;****************************************************************************************
; - Load the address of the destination page to be burnt. (pages start at $E000)
;****************************************************************************************

     lda     page     ; Load accumulaotr with value in "page"
     add     #$DF
;*     add     #$E0     ; A<-(A)+(M) Add value in accumulator with with $E0
     psha             ; Push result to Stack
     pulh             ; Pull result from stack and copy to index register Hi byte
     clrx             ; Clear index register Lo byte(now a 16 bit value)
     sthx    burnDst  ; Copy to "burnDst"

;****************************************************************************************
; - Load the address of the source page to be burnt.
;****************************************************************************************

     ldhx     #VE_r     ; Load index register with address at "VE_r"
     sthx    burnSrc    ; Copy to "burnSrc"

;****************************************************************************************
; - Burn 189 bytes per page
;****************************************************************************************

     lda     #189T               ; Load accumulator with decimal 189
     sta     burncount           ; Copy to "burncount"
     clrh                        ; Clear index register Hi byte
     clrx                        ; Clear index register Lo byte
     jmp     ms_ProgramFlash     ; Jump to subroutine at ms_ProgramFlash:


;****************************************************************************************
;
;======================  Single Flash Page Erase Subroutine  ============================
;
; This subroutine will copy the Flash Erase algorithm into RAM and execute
; it to erase the page starting at address pointers "burnDst"
;
;****************************************************************************************
;****************************************************************************************
; - Initialize pointer
;****************************************************************************************

ms_EraseFlash:
     ldhx    #ms_EraseRamSize     ; Load index register with address of "ms_EraseRamSize"

;****************************************************************************************
; - Get program from Flash, copy to RAM, decrement pointer and loop back until done.
;****************************************************************************************

ms_EraseFlash1:
     lda     ms_MassErase-1,x     ; Load accumulator with value at "ms_MAssErase-1,
                                  ; offset in index register Lo byte
     sta     ram_exec-1,x         ; Copy to location "ram_exec-1", offset in index
                                  ; register Lo byte
     dbnzx   ms_EraseFlash1       ; Decrement index register Lo byte and branch to lable
                                  ; at "ms_EraseFlash1:" if not zero
     sei                          ; Set interrupt mask bit of CCR(prohibit interrrupts)

;****************************************************************************************
; - Execute Flash Mass Erase algorithm from RAM
;****************************************************************************************

     jmp     ram_exec     ; Jump to subroutine at lable "ram_exec:"(start of executable
                          ; RAM space from "boot_r12"

;****************************************************************************************
;
;==========================  Flash Program Subroutine  ==================================
;
; This subroutine will copy the Flash Program algorithm into RAM and execute it to
; program  "burncount" bytes from the address pointed to by 'burnSrc' to the address
; pointed to by "burnDst"
;
;****************************************************************************************
;****************************************************************************************
; - Initialize pointer
;****************************************************************************************

ms_ProgramFlash:
     ldhx    #ms_ProgramRamSize     ; Load index register with address at
                                    ; "ms_ProgramRamSize"

;****************************************************************************************
; - Get program from Flash, copy to RAM, decrement pointer and loop back until done.
;****************************************************************************************

ms_ProgramFlash1:
     lda     ms_Delay-1,x             ; Load accumulator withe value in address
                                      ;  "ms_Delay-1, offset in index register Lo byte
     sta     ram_exec-1,x             ; Copy to address at "ram_exec-1, offset in index
                                      ; register Lo byte
     dbnzx   ms_ProgramFlash1         ; Decrement index register Lo byte and branch to
                                      ; lable at "ms_EraseFlash1:" if not zero
     sei                              ; Set interrupt mask bit of CCR
                                      ; (prohibit ineterrupts)
     jmp     {ram_exec+ms_ProgramRam} ; Jump to location at "ram_exec+ms_ProgramRam"

;****************************************************************************************
;
;==============================  Flash Erase Subroutine  ================================
;
;  This subroutine performs a single Page Erase @ BurnDst
;  This subroutine has been tuned for a bus speed of 7.3728 MHz. Constants revised for
;  8MHz
;  This subroutine is copied into and executed from RAM.
;
;****************************************************************************************
;****************************************************************************************
; - Initialize pointer to Flash Memory Address
;****************************************************************************************

ms_MassErase:
     ldhx    burnDst     ;Load accumulator with value in "burnDst"

;****************************************************************************************
;   Set ERASE, read the Flash Block Protect Register and write any data into Flash page.
;****************************************************************************************

     lda     #{ERASE}     ; Load accumulaotr with value in "{ERASE}"
     sta     flcr         ; Copy to Flash Control Register(set erase bit)
     lda     flbpr        ; Load accumulator with value in Flash Block Protest Register
     sta     ,x           ; Copy to index register Lo byte

;****************************************************************************************
;   Wait for >10us(11.1uS), then set HVEN in Flash Control Register.
;****************************************************************************************

     lda     #1                 ; Load accumulator with decimal 1
     bsr     ms_delay           ; Branch to subroutine at lable "ms_delay:"
     lda     #{ERASE | HVEN}    ; Load accumulator with value in "ERASE" bit wise Or
                                ; "HVEN"
     sta     flcr               ; Copy to Flash Control Register

;****************************************************************************************
;   Wait for >1ms(1.012mS), then clear ERASE.
;****************************************************************************************

     lda     #105T        ; Load accumulator with decimal 105
     bsr     ms_delay     ; Branch to subroutine at lable "ms_delay:"
     lda     #{HVEN}      ; Load accumulaotr with value in "HVEN"(clears ERASE)
     sta     flcr         ; Copy to Flash Control Register

;****************************************************************************************
;   Wait for >5us(11.1uS), then clear HVEN.
;****************************************************************************************

     lda     #1           ; Load accumulator with decimal 1
     bsr     ms_delay     ; Branch to subroutine at lable "ms_delay:"
     clra                 ; Clear accumulator
     sta     flcr         ; Copy to Flash Control Register
     rts                  ; Return from subroutine


;****************************************************************************************
;
;==================================  Delay Subroutine  ==================================
;
;  This subroutine performs a simple software delay loop based upon the value passed in
;  ACC.
;  The following timing calculation applies:
;
;   was supposed to be  delay = ((ACC * 74) + 12) (tcyc)
;   actually            delay = ((ACC * 108) + 12) (tcyc) i.e. longer/safer? delays
;   now                 delay = ((ACC * 77) + 12) (tcyc)
;
;  Calling convention:
;
;      lda     data
;      jsr     delay
;
;  Returns:    nothing
;
;  Changes:    ACC
;
;****************************************************************************************
;****************************************************************************************
; - Save outer delay loop parameter
;****************************************************************************************

ms_Delay:
     psha     ; Push value in accumulator to Stack

;****************************************************************************************
; - Initialize inner delay loop counter
;****************************************************************************************

ms_Delay1:
     lda     #23T     ; Load accumulator with decimal 23

;****************************************************************************************
; - Decrement inner delay loop counter, decrement outer delay loop counter, dealocate
;   local variable, return from subroutine
;****************************************************************************************

ms_Delay2:
     dbnza   ms_Delay2          ; Decrement accumulator and branch to lable at
                                ; "ms_Delay2" if not zero
     dbnz    1,sp,ms_Delay1     ; Decrement Stack Pointer and return to lable at
                                ; "ms_Delay1  if not zero
     pula                       ; Pull value from stack and copy to accumulator
     rts                        ; Return from subroutine

;****************************************************************************************
;
;============================  Flash Program Subroutine  ================================
;
;  This subroutine controls the Flash programming sequence.
;
;****************************************************************************************
;****************************************************************************************
; - Equates
;****************************************************************************************

ms_EraseRamSize:   equ     {*-ms_MassErase}
ms_ProgramRam:     equ     {*-ms_Delay}

;****************************************************************************************

ms_FlashProgram:
ms_FlashProgram1:

;****************************************************************************************
; - Set PGM, read the Flash Block Protect Register and write anywhere in desired Flash
;   row.
;****************************************************************************************

     lda     #{PGM}      ; Load accumulator with value in "{PGM}"
     sta     flcr        ; Copy to Flash Control Register
     lda     flbpr       ; Load accumulaotr with value in  Flash Block Protect Register
     ldhx    burnDst     ; Load index register with value in "burnDst"
     sta     ,x          ; Copy to address in "burnDst, offset in index register Lo byte

;****************************************************************************************
; - Wait for >10us, then set HVEN.
;****************************************************************************************

     lda     #1                ; Load accumulator with decimal 1
     bsr     ms_delay          ; Branch to subroutine at lable "ms_delay:"
     lda     #{PGM | HVEN}     ; Load accumulator with value in "ERASE" bit wise Or
                               ; "HVEN"
     sta     flcr              ; Copy to Flash Control Register

;****************************************************************************************
; - Wait for >5us.
;****************************************************************************************

     lda     #1           ; Load accumulator with decimal 1
     bsr     ms_delay     ; Branch to subroutine at lable "ms_delay:"

;****************************************************************************************
; - Write data to Flash and wait for 30 - 40us.
;****************************************************************************************

     ldhx    burnsrc     ; Load index register with address at "burnsrc"
     lda     ,x          ; Load accumulator with value in "burnsrc, offset in index
                         ; register Lo byte
     ldhx    burndst     ; Load index register with address at "burndst"
     sta     ,x          ; Copy to address at "burndst", offset in index register Lo byte
     lda     #3          ; Load accumulator with decimal 3(30.3uS)
     bsr     ms_delay    ; Jump to subroutine at "ms_delay:)

;****************************************************************************************
; - Clear PGM.
;****************************************************************************************

     lda     #{HVEN}     ; Load accumulator with value in {HVEN}
     sta     flcr        ; Copy to Flash Control Register

;****************************************************************************************
; - Wait for >5us, then clear HVEN.
;****************************************************************************************

     lda     #1           ; Load accumulator with decimal 1
     bsr     ms_delay     ; Branch to subroutine at lable "ms_delay:"
     clra                 ; Clear accumulator
     sta     flcr         ; Copy to Flash Control Register

;****************************************************************************************
; - Advance source pointer, advance destination pointer, decrement data counter,
;   return from subroutine
;****************************************************************************************

ms_FlashProgram2:
     ldhx    burnsrc                     ; Load index register with address at "burnscr"
     aix     #1                          ; Add decimal 1 to index register Lo byte
     sthx    BurnSrc                     ; Copy result to "burnSrc"
     ldhx    burndst                     ; Load index register with address at "burnDst"
     aix     #1                          ; Add decimal 1 to index register Lo byte
     sthx    BurnDst                     ; Copy result to "burnDst"
     dbnz    burncount,ms_FlashProgram1  ; Decrement "burnount" and branch to
                                         ;  "ms_FlashProgram1" back if not zero.
     rts                                 ; Return from subroutine

;****************************************************************************************
; - Equates
;****************************************************************************************

ms_ProgramRamSize: equ     {*-ms_Delay}




