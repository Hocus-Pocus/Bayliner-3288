;////////////////////////////////////////////////////////////////////////////

;****************************************************************************
; - Generic code for contact de-bounce of a push button(or any other contact
;   style switch) with edge detection from both open to closed, and closed
;   to open.
;   Auto-repeat is prevented in both states.
;
; - Variables:
; - Sw1DB:       ds1     ; Switch #1 de-bounce timer counter variable
; - LPflags:     ds1     ; Switch last pass status bit field variable
; - Swflags:     ds1     ; Switch status bit field variable
;
; - Equates:
; - Sw1:        equ0     ; Switch #1 contacts on PTA0
; - Sw1LP:      equ0     ; 1 = last pass open         0 = last pass closed
; - Sw1cls:     equ0     ; 1 = contact set closed     0 = contact set open
;****************************************************************************

; ================= This code goes in the main loop =========================

     lda     Sw1DB                ; Load accumulator with value in Switch
                                  ; #1 de-bounce counter variable
     bne     SW1_CHK_DONE         ; If Z bit of CCR is clear, branch to
                                  ; SW1_CHK_DONE: ("Sw1DB" not = 0,
                                  ; de-bounce in progress, skip over)
     brset   Sw1,porta,SW1_OPN    ; If "Sw1" bit of Port A is set,
                                  ; branch to SW1_OPN: (contacts are open)
     brclr   Sw1LP,LPflags,SW1_CHK_DONE  ; If "Sw1LP" bit of "LPflags"
                              ; variable is clear, branch to SW1_CHK_DONE:
                              ;(contacts are closed, but bit is already
                              ;  clear, so skip over)
     mov     #$7D,Sw1DB       ; Move decimal 125 into Switch #1
                              ; de-bounce counter variable (125mS)
     bclr    Sw1LP,LPflags    ; Clear "Sw1LP" bit of "LPflags" variable
     bset    Sw1cls,Swflags   ; Set "Sw1cls" bit of "Swflags" variable
     bra     SW1_CHK_DONE     ; Branch to SW1_CHK_DONE:

SW1_OPN:
     brset   Sw1LP,LPflags,SW1_CHK_DONE  ; If "Sw1LP" bit of "LPflags"
                              ; variable is set, branch to SW1_CHK_DONE:
                              ; (contacts are open, but bit is already
                              ; set, so skip over)
     mov     #$7D,Sw 1DB      ; Move decimal 125 into Switch #1
                              ; de-bounce counter variable (125mS)
     bset    Sw1LP,LPflags    ; Set "Sw1LP" bit of "LPflags" variable
     bclr    Sw1cls,Swflags   ; Clear "Sw1cls" bit of "Swflags" variable

SW1_CHK_DONE:
     brset   Sw1cls,Swflags,SW1_CLOSED   ; If "Sw1cls" bit of "Swflags"
                              ; variable is set, branch to SW1_CLOSED:
                              ;(Contact set #1 closed is valid, do whatever
                              ; it is you want to do under this condition)
     jmp     PROCEED          ; Jump to PROCEED:(proceed with program)

SW1_CLOSED:
                              ;(first do what you want to do)
     bclr    Sw1cls,Swflags   ; Clear "Sw1cls" bit of "Swflags" variable

     PROCEED:
                              ;(proceed with main loop)

; === This code goes in the 1.0mS section of a timer interrupt routine ===

;****************************************************************************
; - Check the value of the contact de-bounce counter variable, if other
;   than zero, decrement it.
;****************************************************************************

     lda     Sw1DB              ; Load accumulator with value in "Sw1DB"
                                ; variable
     beq     SW1DB_CHK_DONE     ; If "Z" bit of "CCR is set, branch to
                                ; SW1DB_CHK_DONE:
     dec     Sw1DB              ; Decrement "Sw1DB" variable

SW1DB_CHK_DONE:

;////////////////////////////////////////////////////////////////////////////

;****************************************************************************
; - Generic code for contact de-bounce of a push button(or any other contact
;   style switch) with edge detection from both open to closed, and closed
;   to open.
;   Auto-repeat at 2HZ is commanded as long as the contacts remain closed
;   for a period of 1 second or more.
;   Auto-repeat is prevented in the open state.
;
; - Variables:
; - Sw1DB:        ds1     ; Switch #1 de-bounce timer counter variable
; - Sw1ARC:       ds1     ; Switch #1 auto-repeat command timer counter var
; - Sw1AR:        ds1     ; Switch #1 auto-repeat timer counter variable
; - LPflags:      ds1     ; Switch last pass status bit field variable
; - ARCflags:     ds1     ; Switch auto-repeat command status bit field
; - ARflags:      ds1     ; Switch auto-repeat status bit field variable
; - Swflags:      ds1     ; Switch status bit field variable
;
; - Equates:
; - Sw1:        equ0     ; Switch #1 contacts on PTA0
; - Sw1LP:      equ0     ; 1 = last pass open         0 = last pass closed
; - Sw1cls:     equ0     ; 1 = contact set closed     0 = contact set open
;****************************************************************************

; ================= This code goes in the main loop =========================

     lda     Sw1DB                ; Load accumulator with value in Switch
                                  ; #1 de-bounce counter variable
     bne     SW1_CHK_DONE         ; If Z bit of CCR is clear, branch to
                                  ; SW1_CHK_DONE: ("Sw1DB" not = 0,
                                  ; de-bounce in progress, skip over)
     brset   Sw1,porta,SW1_OPN    ; If "Sw1" bit of Port A is set,
                                  ; branch to CNT1_OPN: (contacts are open)
     brclr   Sw1LP,LPflags,SW1_ARC_CHK  ; If "Sw1LP" bit of "LPflags"
                             ; variable is clear, branch to SW1_ARC_CHK:
                             ; (contacts closed, bit is already clear,
                             ; check for auto-repeat command)
     mov     #$7D,Sw1DB      ; Move decimal 125 into Switch #1
                             ; de-bounce counter variable (125mS)
     bclr    Sw1LP,LPflags   ; Clear "Sw1LP" bit of "LPflags" variable
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
                                 ; #1 auto repeat command timer counter
     bne     SW1_CHK_DONE        ; If Z bit of CCR is clear, branch to
                                 ; SW1_CHK_DONE: ("Sw1ARC" not = 0,
                                 ; auto-repeat command check in progress,
                                 ; skip over)
     brset   Sw1LP,ARflags,SW1_AR_PROG   ; If "Sw1LP" bit of "ARflags"
                                 ; variable is set, branch to SW1_AR_PROG:
                                 ;(auto-repeat check in progress)
     bra     SW1_AR:             ; Branch to SW1_AR:
     
SW1_AR_PROG:
     lda     Sw1AR               ; Load accumulator with value in Contact
                                 ; Set #1 auto repeat timer counter var
     bne     SW1_CHK_DONE        ; If Z bit of CCR is clear, branch to
                                 ; SW1_CHK_DONE: ("Sw1DB" not = 0,
                                 ; auto-repeat check in progress,
                                 ; skip over)
     bset    Sw1LP,ARflags       ; Set "Sw1LP" bit of "ARflags" variable
     
SW1_AR:
     mov     #$64,Sw1AR          ; Move decimal 100 into Contact Set #1
                                 ; auto-repeat counter variable(500mS)
     
SW1_CLS:
     bset    Sw1cls,Swflags      ; Set "Sw1cls" bit of "Swflags" variable
     jmp     SW1_CHK_DONE        ; Jump to SW1_CHK_DONE:

SW1_OPN:
     brset   Sw1LP,LPflags,SW1_CHK_DONE  ; If "Sw1LP" bit of "LPflags"
                              ; variable is set, branch to SW1_CHK_DONE:
                              ; (contact set open, and bit is already
                              ; set, so skip over)
     mov     #$7D,Sw1DB       ; Move decimal 125 into Contact Set #1
                              ; de-bounce counter variable (125mS)
     bset    Sw1LP,LPflags    ; Set "Sw1LP" bit of "LPflags" variable
     clr     Sw1AR            ; Clear Sw1 auto-repeat timer counter
     bclr    Sw1LP,ARflags    ; Clear "Sw1LP" bit of "ARflags" variable
     clr     Sw1ARC           ; Clear Sw1 auto-repeat command timer counter
     bclr    Sw1LP,ARCflags   ; Clear "Sw1LP" bit of "ARCflags" variable
     bclr    Sw1cls,Swflags   ; Clear "Sw1cls" bit of "Swflags" variable

SW1_CHK_DONE:
     brset   Sw1cls,Swflags,SW1_CLOSED   ; If "Sw1cls" bit of "Swflags"
                              ; variable is set, branch to SW1_CLOSED:
                              ;(Contact set #1 closed is valid, do whatever
                              ; it is you want to do under this condition)
     jmp     PROCEED          ; Jump to PROCEED:(proceed with program

SW1_CLOSED:
                              ;(first do what you want to do)
     bclr    Sw1cls,Swflags   ; Clear "Sw1cls" bit of "Swflags" variable

     PROCEED:
                              ;(proceed with main loop)



; === This code goes in the 1.0mS section of a timer interrupt routine ===

;****************************************************************************
; - Check the value of the contact de-bounce counter variables, if other
;   than zero, decrement them.
;****************************************************************************

     lda     Sw1DB              ; Load accumulator with value in "Sw1DB"
                                ; variable
     beq     SW1DB_CHK_DONE     ; If "Z" bit of "CCR is set, branch to
                                ; SW1DB_CHK_DONE:
     dec     Sw1DB              ; Decrement "Sw1DB" variable

SW1DB_CHK_DONE:

; === This code goes in the 5.0mS section of a timer interrupt routine ===

;****************************************************************************
; - Check the value of the contact auto-repeat command counter variable,
;   if other than zero, decrement it.
;****************************************************************************

     lda     Sw1ARC             ; Load accumulator with value in "Sw1ARC"
                                ; variable
     beq     SW1ARC_CHK_DONE    ; If "Z" bit of "CCR is set, branch to
                                ; SW1ARC_CHK_DONE:
     dec     Sw1ARC             ; Decrement "Sw1ARC" variable

SW1ARC_CHK_DONE:

;****************************************************************************
; - Check the value of the contact auto-repeat counter variable, if other
;   than zero, decrement it.
;****************************************************************************

     lda     Sw1AR              ; Load accumulator with value in "Sw1AR"
                                ; variable
     beq     SW1AR_CHK_DONE     ; If "Z" bit of "CCR is set, branch to
                                ; SW1AR_CHK_DONE:
     dec     Sw1AR              ; Decrement "Sw1AR" variable

SW1AR_CHK_DONE:

;////////////////////////////////////////////////////////////////////////////

