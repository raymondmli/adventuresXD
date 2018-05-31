.include "m2560def.inc"

.def row = r16				; row and column of keypad press
.def col = r17
.def rmask = r18		    ; row mask for keypad detection
.def cmask = r19			; column mask for keypad detection
.def temp1 = r22
.def temp2 = r23
.def direction = r20
.def currPosition = r21		; current floor
.def switchCount = r24		; #times lookAhead routine has switched direction. if > 1, no floors to visit
.def input = r25			; keypad input
.def doorState = r30		; whether the door is opening, opened, closed, closing, to_open

.equ PORTLDIR = 0xF0		; initialising most significant bits of PORT L to output, and least to input. Keypad port
.equ INITCOLMASK = 0xEF		; to 1111 1110, since we are scanning from rightmost column
.equ INITROWMASK = 0x01		; to 0000 0001, since we are scanning from top
.equ ROWMASK = 0x0F			; to 0000 1111, all one's to 'and' with input from column wire 
.equ DOWN = 0x2				; down is signified by 2, while up is 1 and stall is 0
.equ UP = 0x1
.equ STALL = 0x0
.equ BOTTOMFLOOR = 0x0		; floors numbered 0-9 in program
.equ TOPFLOOR = 0x9
.equ TOPFLOOR_INC1 = 0xA	; actual top floor (10)
.equ YES = 0x1
.equ NO = 0x0
.equ LCD_RS = 7				; lcd stuff
.equ LCD_E = 6				; don't remember
.equ LCD_RW = 5
.equ LCD_BE = 4
.equ F_CPU = 16000000		; variables for one ms delay
.equ DELAY_1MS = F_CPU / 4 / 1000 - 4

.equ OPENING = 0x1			; elevator door variables
.equ OPENED = 0x0
.equ CLOSED = 0x2
.equ CLOSING =0x3
.equ TO_OPEN = 0x04

.dseg
floors:						; floors, one byte for each floor
.byte 10
tempCounter:            	; Counter for how many 128's have went by (interrupts)
.byte 2
targetNumInterrupts:    	; How many interrupts do we want - 7812 for 1 sec
.byte 2

.cseg
.org 0x0000
	jmp RESET
.org INT0addr
    jmp extINT0
.org INT1addr
	jmp extINT1
.org OVF0addr
    jmp Timer0OVF

.macro do_lcd_command	  	; forgotten how the lcd works
	ldi r16, @0            	; Load input into r16.
	rcall lcd_command      	; Call lcd_command
	rcall lcd_wait         	; Call lcd_wait
.endmacro
.macro do_lcd_data
	mov r16, @0         	; Load input into r16
	rcall lcd_data          ; Call lcd_data
	rcall lcd_wait          ; Call lcd_wait
.endmacro

.macro lcd_set
	sbi PORTA, @0      ; Set pin specified by input inPORTA
.endmacro
.macro lcd_clr
	cbi PORTA, @0      ; Clear the pin in PORTA specified by input
.endmacro


lcd_command:
	out PORTF, r16     ; Output command (r16) to PORTF
	rcall sleep_1ms
	lcd_set LCD_E      ; Set the 6th pin (LCD_E) in PORTA - Start signal for read/write
	rcall sleep_1ms
	lcd_clr LCD_E      ; Clear the 6th pin in PORTA
	rcall sleep_1ms
	ret

lcd_data:
	out PORTF, r16
	lcd_set LCD_RS     ; Set the 7th pin (LCD_RS) in PORTA - Select Data Register (write or read)
	rcall sleep_1ms
	lcd_set LCD_E      ; Set the 6th pin (LCD_E) in PORTA - Start signal for read/write
	rcall sleep_1ms
	lcd_clr LCD_E      ; Clear the 6th pin in PORTA
	rcall sleep_1ms
	lcd_clr LCD_RS     ; Clear the 7th pin in PORTA
	ret

; waits for LCD to be unbusied
lcd_wait:
	push r16
	clr r16
	out DDRF, r16      ; Set PORTF to input
	out PORTF, r16     ;
	lcd_set LCD_RW
lcd_wait_loop:
	rcall sleep_1ms
	lcd_set LCD_E
	rcall sleep_1ms
	in r16, PINF
	lcd_clr LCD_E
	sbrc r16, 7
	rjmp lcd_wait_loop
	lcd_clr LCD_RW
	ser r16
	out DDRF, r16
	pop r16
	ret

sleep_1ms:
	push r24
	push r25
	ldi r25, high(DELAY_1MS)
	ldi r24, low(DELAY_1MS)
delayloop_1ms:
	sbiw r25:r24, 1
	brne delayloop_1ms
	pop r25
	pop r24
	ret

sleep_5ms:
	rcall sleep_1ms
	rcall sleep_1ms
	rcall sleep_1ms
	rcall sleep_1ms
	rcall sleep_1ms
	ret

; **************************************************************************
; This macro should loop through from the start to the destination floor
; in our list, incrementing or decrementing from floor to floor til destination
; Used in conjunction with setBit macro
; Address of destination floor byte stored in yh:yl.
; temp1 and temp2 will also store the destination floor
; **************************************************************************
.macro loopThrough ; @0 - start @1 - destination. Numbers representing their floor
    ldi yl, low(floors)     ; Load address of start of list to y
    ldi yh, high(floors)
    add yl, @0              ; Move the y pointer n(start) places so we get to the starting position
    ldi temp1, 0
    adc yh, temp1			; in case of an overflow
    mov temp1, @0           ; load parameter 0 to temp1 (current floor counter)
    mov temp2, @1           ; Load the desired number (the floor we want to loop to) to temp2

    cp temp1, temp2			; if floor is below, loop down, else loop up
    brlt loopUp
loopDown:
    cp temp1, temp2         ; Compare temp1 and temp2
    breq loopThroughEnd     ; If we are 'there' end macro
    adiw yh:yl, -1          ; Add -1 to the index pointer.
    dec temp1               ; Add -1 to counter
    rjmp loop
loopUp:
    cp temp1, temp2         ; Compare temp1 and temp2
    breq loopThroughEnd     ; If we are 'there' end macro
    adiw yh:yl, 1           ; Add 1 to the index pointer
    inc temp1               ; Add 1 to counter
    rjmp loop
loopThroughEnd:
    nop                     ; Shinami
.endmacro
; macro to move yh:yl to a particular floor byte
.macro getBit
    ldi yl, low(floors)     ; Load start of list to y
    ldi yh, high(floors)
    add yl, @0              ; Move the y pointer n(start) places so we get to the starting position
    ldi temp1, 0
    adc yh, temp1
.endmacro

; **********************************************************************
; This macro should count the number floors we need to service
; return value - temp2.
; **********************************************************************
.macro countFloors
    clr temp1
    clr temp2
    ldi yl, low(floors)     ; Load start of list to y
    ldi yh, high(floors)
countLoop:
    cpi temp1, 10
    breq exit
    push temp1
    ld temp1, y+
    cpi temp1, 1
    brne empty
    inc temp2
empty:
    pop temp1
    inc temp1
    rjmp countLoop
exit:
.endmacro

; **********************************************************************
; This macro should check if the currentfloor need to be serviced
; return value - temp2 (YES or NO)
; **********************************************************************
.macro checkBit
    clr temp2
	getBit currPosition
    ld temp2, y
.endmacro

; *************************************************************************
; This macro should set the bit of the floor that the y pointer is pointing to
; y pointer is set to the floor byte of our destination floor
; EDIT: doesn't require loopThrough, since getBit sets y to the destination floor.
; *************************************************************************
.macro setBit ; We need to service this floor (set it to 1)
    ldi temp1, BOTTOMFLOOR      ; Start from the bottom floor
    getBit @0                   ; Move the y pointer until we get to the destination floor
    ldi temp1, 1
    st y, temp1                 ; Store a 1 in that floor
.endmacro


; **********************************************************************
; This macro should clear the bit of the floor that the y pointer is pointing
; Assumes currPosition is the floor we want to clear, i.e. we've already visited
; EDIT: doesn't require loopThrough, since getBit sets y to the visited floor.
; **********************************************************************
.macro clrBit ; Clear the floor (we dont wanna go there)
    ldi temp1, BOTTOMFLOOR      ; Start from the bottom floor
    getBit currPosition         ; Move the y pointer until we get to the destination floor
    ldi temp1, 0                ; Clear the destination floor bit by setting it to 0
    st y, temp1
.endmacro
; **********************************************************************
; This macro should display the floors we have passed whilst travelling (?)
; in the LEDs of course. This includes the destination floor
; Assumes DDRC and DDRG are set for write, and currPosition is the floor we have travelled to
; Run: starts at temp1, increments to currPosition, left shifts and adds one to floor display
; If curPosition is 8/9, then we have to use top 2 bits of LEDbar. So cases are hardcoded.
; **********************************************************************
.macro displayFloorLED      ; Light updates as we move up to a floor (ie. at floor 5 we have 6 leds switched on)
    clr temp1
    ldi temp2, 1
    out PORTG, temp1		; clears top 2 LED bar bits
    cpi currPosition, 8     ; If it is floor 8 we have turn on the top two bits of the led bar which is not in PORTC
    breq floor8
    cpi currPosition, 9     ; If it is floor 9 we have turn on the top two bits of the led bar which is not in PORTC
    breq floor9
checkLoop:                  ; This is for floors 0-7
    cp temp1, currPosition  ; Check if temp1 has reached the floor we are at now
    breq display            ; If it is then we can display it
    lsl temp2               ; Otherwise left shift temp2 ie. 0b00000001 becomes 0b00000010
    inc temp2               ; Add 1 so 0b00000010 becomes 0b00000011 this way we turn on the bottomn two lights
    inc temp1
    rjmp checkLoop
floor8:
    ldi temp1, 0b11111101
    rjmp topTwoBits
floor9:
    ldi temp1, 0b11111111
    rjmp topTwoBits
topTwoBits:
    out PORTG, temp1        ; PORTG has the top two bits in the ledBar
    ldi temp2, 0b11111111   ; Also have to turn on every other light under those two bits
display:
    out PORTC, temp2
.endmacro

; **********************************************************************
; This macro should take in the number of interrupts(@0) we want and set
; that into the targetNumInterrupts space in memory
; 7812 - 1 second
; 3906 - 0.5 seconds
; **********************************************************************
.macro setTargetTime
    push r25
    push r24

    clr r25
    clr r24
    ldi temp1, low(@0)                  ; Add the interrupt to targetNumInterrupts
    add r24, temp1
    ldi temp1, high(@0)
    adc r25, temp1
    sts targetNumInterrupts, r24
    sts targetNumInterrupts + 1, r25

    pop r24
    pop r25
.endmacro

; **********************************************************************
; This macro should clear a word in memory
; Generally takes a symbol as a parameter
; **********************************************************************
.macro clear
    ldi YL, low(@0)     ; load the memory address to Y
    ldi YH, high(@0)
    clr temp1            ; set temp to zero by clearing
    st Y+, temp1         ; set the two bytes in SRAM to 0
    st Y, temp1
.endmacro
; **********************************************************************
; RUN: mainly setup
; initialises stack, sets portL for half output, half input
; sets PORTC, PORTG for output
; sets timer 0A for normal mode, timer 0B is set to a 8-prescale value. timer0OVF enabled
; PORT E set to output, pull-up resistor is enabled(?) and 0's loaded into E
; sets OCR3BL and OCR3BH to 0, TMR3 to phase-correct wgm and for output compare
; sets ISC01 in EICRA, which causes INT0 to trigger on falling edge. EDITED HERE TO MAKE INT1 trigger on falling edge.
; fetches current EIMSK and ors with INT0 and  INT1 (EDITED IN) to enable INT1
; sets PORTF and PORTA to output
; displays first LED, initialises bottom floor to currPos, direction to STALL, door to closed.
; also sets targetTime to 2s, which means that doors will initially stay closed until keypad input
; routine checks for keypad input
; since direction is stall, lift won't 'move' until keypad input : CONFIRM
; **********************************************************************
RESET:
	ldi temp1, low(RAMEND)	   ; initialising stack
	out SPL, temp1
	ldi temp1, high(RAMEND)
	out SPH, temp1

	ldi temp1, PORTLDIR        ; Set port L direction to output for Pin 7:4 and input for Pin 3:0
	sts DDRL, temp1
	ser temp1
	out DDRC, temp1            ; Set port C to output (LEDs)
    out DDRG, temp1			   ; Set top two LED bar bits to output
    ldi temp1, 0x1
	out PORTC, temp1           ; Turn off the leds except the one representing first floor
    clr temp1
    out PORTG, temp1

    ldi temp1, 0b00000000       ; Set the mode of operation of the timer to Normal Mode (counting direction is up)
    out TCCR0A, temp1           ; Timer will roll over when it passes its maximum 8-bit value
    ldi temp1, 0b00000010       ; Sets the prescaling value to 8 by setting CS01 (clock select bit 1) to 1
    out TCCR0B, temp1
    ldi temp1, 1<<TOIE0         ; Set TOIE0 (defined to be 0) to enable the Timer 0 Overflow Interrupt
    sts TIMSK0, temp1

    ldi temp1, 0b00010000
    out DDRE, temp1
	ldi temp1, 0x0
	out PORTE, temp1

    ldi temp1, 0x00                     ; First set the comparison voltage to middle
    sts OCR3BL, temp1                   ; stores comparison voltage
    clr temp1
    sts OCR3BH, temp1

	ldi temp1, (1 << CS30) 		               ; set the Timer3 to Phase Correct PWM mode.
	sts TCCR3B, temp1
	ldi temp1, (1<< WGM30)|(1<<COM3B1)         ; sets to output compare bit and the waveform generation
	sts TCCR3A, temp1

    ldi temp1, (1 << ISC01)|(1<<ISC11)       ; Set INT0 and INT1 to accept falling edge signal. EDIT: originally only set INT0 to accept falling edge
    sts EICRA, temp1                         ; Store temp in EICRA (used for INT0 and INT1)

    in temp1, EIMSK                          ; Get the current EIMSK
    ori temp1, (1 << INT0)|(1 << INT1)       ; Logical OR to enable INT1 and INT0
    out EIMSK, temp1                         ; Output temp to EIMSK
    sei

    ser temp1                      ; Set Port F and A to output
	out DDRF, temp1
	out DDRA, temp1
	clr temp1                      ; Clear all bits at the start
	out PORTF, temp1
	out PORTA, temp1
    do_lcd_command 0b00111000      ; Sets the interface data length to 8 bits and selects 2-line display and 5 x 7-dot character font.
	rcall sleep_5ms
	do_lcd_command 0b00000001      ; clear display
	do_lcd_command 0b00000110      ; Sets mode to increment the address by one and to shift cursor to the right at the time of write to internal RAM.
	do_lcd_command 0b00001110      ; No blinking (LSB is 0 not 1), turn on cursor (second bit), turn on display (third bit)

    setTargetTime 15624             ; Set target time to 2s
    ldi currPosition, BOTTOMFLOOR
    displayFloorLED
    ldi direction, STALL
    ldi doorState, CLOSED

main:
	ldi cmask, INITCOLMASK			; 1111 1110
	clr col

; Keypad code.
; **********************************************************************
; RUN:
; Begins with cmask as 1111 1110 and col == 0
; Stores cmask in PORTL, which only keep 1110 in least significant bits
; Delay is implemented for debouncing reasons
; Loads keypad input from PINL into temp1 and 'ands' with ROWMASK (0000 1111) to figure out if button is pressed
; If not, branch to nextCol which lsl's 1111 1110, to become 1101 and goes back to colLoop to try again
; If yes, br rowLoop which does the same thing. When row is found branches to convert
; First checks if our input could be 0 or letters. If input is 0, set input to 0. If letters rjmp to main.
; else apply the formula and set the corresponding floor byte to 1
; debouncing at the end: so long as the row is low i.e. the button has not been released or switch is still bouncing
; loop until it is not
; **********************************************************************
columnLoop:
	cpi col, 4         ; Check if we have checked all columns
	breq main          ; If we have checked then we go back to main and repeat
	sts PORTL, cmask   ; Otherwise output the mask to PORTL

	ldi temp1, 0xFF    ; Implement a delay

delay:
	dec temp1
	brne delay

	lds temp1, PINL        ; Load PINL to temp1 (keypad input)
	andi temp1, ROWMASK    ; Use logical and with ROWMASK (0000 1111)
	cpi temp1, 0xF         ; If there are no buttons pressed then we go to next column
	breq nextCol

	ldi rmask, INITROWMASK ; Otherwise load initial mask to rmask
	clr row

rowLoop:
	cpi row, 4             ; Check if we have checked all rows
	breq nextCol           ; If we have then go to the next column
	mov temp2, temp1       ; Otherwise move temp1 to temp2
	and temp2, rmask       ; Use logical and with rmaks
	breq convert           ; If they are equal meaning we have got the row we want then go to convert
	inc row                ; Otherwise increment row
	lsl rmask              ; Left shift rmask
	jmp rowLoop

nextCol:
	lsl cmask              ; Left shift the cmask
	inc cmask              ; increment cmask
	inc col                ; Increase col
	jmp columnLoop

convert:
	cpi col, 3             ; Check if col is 3 which means it is letter (we don't deal with it)
	breq main

	cpi row, 3             ; Check if row is 3
	breq possiblyZero

	mov input, row         ; Convert row and col to the number of the button
	lsl input              ; number = 3 * row + col + 1
	add input, row
	add input, col
	inc input
	jmp takeInput

possiblyZero:
	cpi col, 1             ; Check if col is 1
	brne main              ; If not then we go back to main
	clr input

takeInput:
    setBit input

checkDebouncing:
    lds temp1, PINL         ; Read PORTL.
    andi temp1, ROWMASK     ; Get the keypad output value
    cpi temp1, 0xF          ; Check if the row is low
    brne checkDebouncing
	jmp main
; **********************************************************************
; what's connected to int0: PB0
; checks if door is opened, and closes doors if so.
; sets tempCounter 23435 (3s), which is the 3s needed for door opening, so door automatically closes.
; else reti
; **********************************************************************
extINT0:
    in temp1, SREG
    push temp1
	push temp2
    push YH
    push YL
    push r25
    push r24
	push switchCount
	push direction

    cpi doorState, OPENED
    brne return
    clr r24
    clr r25
    ldi temp1, low(23435)                  ; Add the interrupt to targetNumInterrupts
    add r24, temp1
    ldi temp1, high(23435)
    adc r25, temp1
    sts tempCounter, r24
    sts tempCounter + 1, r25

return:
	pop direction
	pop switchCount
    pop r24
    pop r25
    pop YL
    pop YH
	pop temp2
    pop temp1
    out SREG, temp1
    reti
; **********************************************************************
; NOT SET UP YET IN INTERRUPT VECTORS SO DOES NOTHING
; Task: If push button
; Remember it takes 128 cycles for door state to be checked again
; Lacks debouncing. If we hold whilst doorsClosing, door will be opening forever
; **********************************************************************
extINT1:
	in temp1, SREG
	push temp1
	push temp2
	push YH
	push YL
	push r25
	push r24
	push r22
	push r23
	push r21
	push switchCount
	push direction
extINT1Body:
	clr r24
	clr r25
	rcall isStopped
	cpi direction, STALL
	breq doorsStall
	cpi doorState, CLOSING
	breq doorsClosing
	cpi doorState, OPENED
	breq doorsOpened
	rjmp extINT1End			; ends if none of the conditions for the 3 states are fulfilled

doorsStall:					; does the whole doors opening and closing shtick but immediately by setting tempCounter == targetNumInterrupts
	ldi temp1, low(15624)               ; this routine will occur after doors are closed, so correct #interrupts is 2s worth
	add r24, temp1						; since doorState is CLOSED, doors should open from timer0OVF
	ldi temp1, high(15624)
	adc r25, temp1
	sts tempCounter, r24
	sts tempCounter + 1, r25
	rjmp extINT1End
; door should stop closing, and opening and closing shtick should proceed.
; We do this by triggering our own opening routine and changing the door state at the end.
doorsClosing:
	ldi r22, low(7812)
	ldi r23, high(7812)
	lds yl, low(tempCounter)
	lds yh, high(tempCounter)
	sub r22, low(tempCounter)		; subtracting 1s of interrupts from the amount of time the doors had been closing for
	sbc r23, high(tempCounter)		; ..since we want tempCounter to count this amount of time for door reopening
	ldi temp1, r22
	add r24, temp1
	ldi temp1, r23
	adc r25, temp1
	sts tempCounter, r24
	sts tempCounter + 1, r25

	ldi temp1, 0xFF                 ; Turn motor on
	sts OCR3BL, temp1
	ldi doorState, OPENING          ; Update the current door state
	ldi temp1, 0b10101010
	out PORTC, temp1
	setTargetTime 7812

	rjmp extINT1End					; ends
; If the Open button is held down while the door is open, the door should remain open until the button is released
; subtracts from tempCounter whilst door is opened
; does nothing to program if tempCounter < 7812. Else sub 10 from tempCounter
doorsOpened:
	lds r22, low(tempCounter)
	lds r23, high(tempCounter)
	cpi r22, low(7812)
	cpc r23, high(7812)
	brlt extINT1End
	sbiw r23:r22, 10
	sts tempCounter, r22
	sts tempCounter + 1, r23

extINT1End:
	pop direction
	pop switchCount
	pop r21
	pop r23
	pop r22
    pop r24
    pop r25
    pop YL
    pop YH
	pop temp2
    pop temp1
	out SREG, temp1
 	reti
; **********************************************************************
; Helper function for extINT1 to check if elevator had stopped
; Assumes lift only stops when there are no floors to visit
; Changes direction to STALL
; Only tested with extINT1 so direction will not change in program since it is pushed and popped
; **********************************************************************
isStopped:
	in temp1, SREG
	push temp1
	push temp2
	push YH
	push YL
	push r25
	push r24
	push switchCount

stoppedBody:
	ldi temp1, direction
	rcall lookAhead						; if direction has changed, then there are no floors in current direction
	cpi temp1, direction
	breq isStoppedEnd					; if direction hasn't changed, then there are floors to visit, so lift hasn't stalled

changed:
	cpi switchCount, 1					; if one switch has already occurred, then another switch would mean there are no floors to visit
	breq stopped
	inc switchCount
	rjmp stoppedBody
	ldi direction, STALL

isStoppedEnd:
	pop switchCount
    pop r24
 	pop r25
    pop YL
	pop YH
	pop temp2
    pop temp1
    out SREG, temp1
    ret


; **********************************************************************
; Timer0 OVF handler. Set to OVF every 128 microseconds
; Every OVF interrupt, checks if targetNumInterrupts/7812 seconds had been reached.
; if not, store the new time. If yes br timeNow, which means 2 seconds have passed.
; which also means a new floor had been reached. So we:
; - reset target time to 2s of interrupts:
; - clear lcd display
; - store currPosition temp2. adds '0' to make temp2 ascii equivalent and displays temp2 on lcd
; - display currPosition on LED bar
; - checks if door needs to be opened. If yes, DESCRIPTION CONTINUES AT 'START OPENING'.
; - if no, check for floors that do need to be opened using checkFloors, and increments toward target floor by 1
; **********************************************************************
Timer0OVF:
    in temp1, SREG
    push temp1
    push YH
    push YL
    push r25
    push r24
    push r27
    push r26

    lds r24, tempCounter            ; Load the first byte of tempCounter into r24
    lds r25, tempCounter + 1        ; Load the second byte of tempCounter into r25
    lds r26, targetNumInterrupts
    lds r27, targetNumInterrupts + 1
    adiw r25:r24, 1                 ; Add 1 to the word (r25:r24)
    cp r24, r26                     ; compare the low(targetNumInterrupts) with r24
    cpc r25, r27                    ; Compare with carry the high(targetNumInterrupts)
    brge timeNow

    sts tempCounter, r24            ; It has not been targetNumInterrupts/7812 seconds yet so we just store the new time
    sts tempCounter + 1, r25
	rjmp end

timeNow:
	; targetNumInterrupts have been reached
    setTargetTime 15624             ; Reset targetTime to 2
    do_lcd_command 0b00000001       ; clear display

    mov temp2, currPosition
    subi temp2, -'0'
    do_lcd_data temp2

    displayFloorLED                 ; Display the current floor on the LEDs

    checkBit                        ; Check if we need to open the door at this floor. Remember checkBit stores @ 'temp2'
    cpi temp2, YES
    brne checkIsland

    cpi doorState, CLOSED           ; If we need to open the door
    breq startOpening
    cpi doorState, OPENING          ; If the door is already opening
    breq stopOpening
    cpi doorState, OPENED           ; If the door have been opened and need to close
    breq startClosing
    cpi doorState, CLOSING          ; If the door have been closing
    breq stopClosingIsland

    ; LED stuff
    rjmp end
; motor at max. Updates doorState and outs 10101010 to LEDs. Adds 1 second to targetNumInterrupts
; sets door state and clears temp counter
; when targetNumInterrupts is reached, timeNow will be called again
; targetTime is set back to 2s which will be overriden by these door state routines
; other than case in which doors have been 'closing' i.e. stopClosingIsland
startOpening:
    ldi temp1, 0xFF                 ; Turn motor on
    sts OCR3BL, temp1
    ldi doorState, OPENING          ; Update the current door state
    ldi temp1, 0b10101010
    out PORTC, temp1
    setTargetTime 7812
    rjmp clearCounter
; starts after startOpening's 1 second routine
; similar to startOpening, other than different LEDs displayed and longer targetTime
stopOpening:
    ldi temp1, 0x0                  ; Turn motor off
    sts OCR3BL, temp1
    ldi doorState, OPENED           ; Update the current door state
    setTargetTime 23436             ; Set targetNumInterrupts to 3s so we keep door open for 3s
    ser temp1
    out PORTC, temp1
    rjmp clearCounter

; if doors have been closing stop, since motors had been running for 1 sec
stopClosingIsland:
    rjmp stopClosing

; when we don't need to open door on floor.
checkIsland:
    rjmp check

; same as other routines.
startClosing:
    ldi temp1, 0xFF                 ; Turn motor on
    sts OCR3BL, temp1
    ldi doorState, CLOSING          ; Update the current door state
    setTargetTime 7812
    ldi temp1, 0b01010101
    out PORTC, temp1
    rjmp clearCounter
; after startClosing's 1 second routine, begins:
; does what is contained in comments + displays current floor LEDs.
; final routine when using doors and sets targetTime to 2s, for either travel time, or do nothing timer
; clears the floor
stopClosing:
    ldi temp1, 0x0                  ; Turn motor off
    sts OCR3BL, temp1
    ldi doorState, CLOSED           ; Update the current door state
    setTargetTime 15624
    displayFloorLED
    clrBit
; when we don't need to open door on floor.
check:
    clr switchCount
    rcall checkFloors               ; Move the elevator if we need to

clearCounter:
    clear tempCounter
    rjmp end

end:
    pop r26
    pop r27
    pop r24
    pop r25
    pop YL
    pop YH
    pop temp1
    out SREG, temp1
    reti

; **********************************************************************
; Task: Moves elevator by 1, if there are floors to move to, in it's current direction. Else clears direction.
; Cases:
; a) if there are no more floors to visit in current direction then previous direction != current direction
; - if direction was previously upward/down, but lookAhead changed it to down/up (respectively)..
;   then br directionChange (which doesn't actually change direction).
; - directionChange just records a direction change in switchCount and clears the switch count and direction, if there are no floors left to visit.
; - This is signified by more than one switch caused by lookAhead.

; In the latter case, the elevator's timer routine would also end?

; In the former case, so long as there are floors to visit
; then direction and previous direction (in r0) would be the same, and operateElevator would be branched to
; operateElevator increases/decreases current position by 1 according to direction.

; b) if there are floors to visit in the current direction
; if direction was previously upward/downward and lookAhead didn't change it, then operateElevator is called.

; c) if direction == STALL, switch would occur twice which would end routine, leaving direction as up.
; **********************************************************************
checkFloors:
    mov r0, direction
    rcall lookAhead             ; Points 'direction' to direction with floors to visit. Prioritises 'UP'
    ldi temp1, DOWN
    cp r0, temp1                ; If direction was originally down, wasDown
    breq wasDown
    rjmp wasUp               	; Otherwise wasUp
wasUp:
    cpi direction, UP
    brne directionChange
    rjmp operateElevator
wasDown:						; compares current direction with DOWN. If not down, then br direction change.
    cpi direction, DOWN
    brne directionChange
    rjmp operateElevator
directionChange:
    cpi switchCount, 1          ; If we switched once already
    breq operateElevatorClear   ; That mean there is no other floor set
    inc switchCount             ; Record that we switched once
    rjmp checkFloors            ; Check floor again in the new direction
operateElevatorClear:           ; No more floor are set
    clr switchCount
    clr direction               ; direction should be zero if there is nothing above or below the elevator
    rjmp operateElevatorEnd
operateElevator:
    rcall moveElevator          ; Move the elevator in the current direction by 1
    rjmp operateElevatorEnd
operateElevatorEnd:
    ret
; **********************************************************************
; Increase or decrease the currPosition by 1 depending on direction.
; **********************************************************************
moveElevator:
    clr temp1
    clr temp2
    cpi direction, UP
    breq moveAhead
    cpi direction, 0
    breq moveEnd
    dec currPosition
    rjmp moveEnd
moveAhead:
    inc currPosition
moveEnd:
    ret
; ****************************************************************
; See if there are floors waiting in the direction we are facing
; If yes set temp1 to 1, otherwise invert direction variable
; if direction is UP, look up. If direction is down, look down
; checks each floor using getBit to get a floor bit from each floor byte
; If floor is found end. Nothing happens and 'direction' stays the same
; elif checked to top/bottom floor, change 'direction' to opposite
; this means that no floor was set in the current direction
; if direction == STALL, changes direction to up and would look ahead to see nothing, so direction would switch to down.
; ****************************************************************
lookAhead:
    clr temp1
    clr temp2
    cpi direction, UP               ; If the direction we are going is up
    breq loopAhead
    cpi direction, DOWN             ; If the direction we are going is down
    breq loopBehind
    ldi direction, UP				; If no direction, direction set to up
loopAhead:                          ; Search upward
    mov temp2, currPosition
checkEachFloorUpwards:
    clr temp1
    cpi temp2, TOPFLOOR_INC1        ; If we checked all the way to the topfloor
    breq switch
    inc temp2                       ; Check the next floor
    getBit temp2
    ld temp1, y                     ; Get value from the list
    cpi temp1, 1                    ; If we found a floor that is set
    breq lookAheadEnd
    rjmp checkEachFloorUpwards
loopBehind:                         ; Same as loopAhead but downward
    mov temp2, currPosition
    rjmp checkEachFloorBackwards
checkEachFloorBackwards:
    clr temp1
    cpi temp2, BOTTOMFLOOR ;may need to refactor just like topfloor_inc1 and topfloor
    breq switch
    dec temp2
    getBit temp2
    ld temp1, y
    cpi temp1, 1
    breq lookAheadEnd
    rjmp checkEachFloorBackwards
; Switch the direction
; meaning no floor was set in the current direction
switch:
    cpi direction, UP
    breq UPToDOWN
    ldi direction, UP
    rjmp lookAheadEnd
UPToDOWN:
    ldi direction, DOWN
lookAheadEnd:
    ret
