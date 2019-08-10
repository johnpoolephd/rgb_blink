;**********************************************************************
;   This file is a basic code template for assembly code generation   *
;   on the PIC16F690. This file contains the basic code               *
;   building blocks to build upon.                                    *  
;                                                                     *
;   Refer to the MPASM User's Guide for additional information on     *
;   features of the assembler (Document DS33014).                     *
;                                                                     *
;   Refer to the respective PIC data sheet for additional             *
;   information on the instruction set.                               *
;                                                                     *
;**********************************************************************
;                                                                     *
;    Filename:	    blink.asm                                         *
;    Date:	    09/08/2019                                        *
;    File Version:                                                    *
;                                                                     *
;    Author:	    John Poole                                        *
;    Company:                                                         *
;                                                                     * 
;                                                                     *
;**********************************************************************
;                                                                     *
;    Files Required: P16F690.INC                                      *
;                                                                     *
;**********************************************************************
;                                                                     *
;    Notes:                                                           *
;                                                                     *
;**********************************************************************


	list		p=16f690		; list directive to define processor
	#include	<p16f690.inc>		; processor specific variable definitions
	
	__CONFIG    _CP_OFF & _CPD_OFF & _BOR_OFF & _PWRTE_ON & _WDT_OFF & _INTRC_OSC_NOCLKOUT & _MCLRE_ON & _FCMEN_OFF & _IESO_OFF


; '__CONFIG' directive is used to embed configuration data within .asm file.
; The labels following the directive are located in the respective .inc file.
; See respective data sheet for additional information on configuration word.






;***** VARIABLE DEFINITIONS
w_temp		EQU	0x7D			; variable used for context saving
status_temp	EQU	0x7E			; variable used for context saving
pclath_temp	EQU	0x7F			; variable used for context saving




;**********************************************************************
	ORG		0x000			; processor reset vector
  	goto		main			; go to beginning of program


	ORG		0x004			; interrupt vector location
	movwf		w_temp			; save off current W register contents
	movf		STATUS,w		; move status register into W register
	movwf		status_temp		; save off contents of STATUS register
	movf		PCLATH,w		; move pclath register into W register
	movwf		pclath_temp		; save off contents of PCLATH register


; isr code can go here or be located as a call subroutine elsewhere

	movf		pclath_temp,w		; retrieve copy of PCLATH register
	movwf		PCLATH			; restore pre-isr PCLATH register contents	
	movf		status_temp,w		; retrieve copy of STATUS register
	movwf		STATUS			; restore pre-isr STATUS register contents
	swapf		w_temp,f
	swapf		w_temp,w		; restore pre-isr W register contents
	retfie					; return from interrupt

;**********************************************************************
; template modified below 
;**********************************************************************
; H/W configuration
;	RC0(16)		red of CA RGB LED, ie. on when LOW, off when HIGH
;	RC1(15)		green of CA RGB LED
;	RC2(14)		blue of CA RGB LED
	
;**********************************************************************
RED	    EQU		0			; red LED bit
GREEN	    EQU		1			; green LED bit
BLUE	    EQU		2			; blue LED bit
LED	    EQU		PORTC			; RGB LED on PORT C

						; delay loop counter
WAIT_LOW    EQU		0x20			; bits <0:7>	
WAIT_MID    EQU		0x21			; bits <8:15>
WAIT_HIGH   EQU		0x22			; bits <16:23>
   
wait_1_sec
;**********************************************************************
; delay for 1 second
;   assumtions: f_clk = 4MHz
;   input:	none
;   output:	none
;**********************************************************************
   
	    movlw	0x30			; load high bits of delay counter
	    movwf	WAIT_HIGH
	    
	    movlw	0xff			; load mid bits of delay counter
	    movwf	WAIT_MID
	    movlw	0xff			; load low bits of delay counter
	    movlw	WAIT_LOW
	    
	    decfsz	WAIT_LOW		; decr low counter
	    goto	$-1			; done?
	    decfsz	WAIT_MID		; decr mid counter
	    goto	$-5			; done?
	    decfsz	WAIT_HIGH		; decr high counter
	    goto	$-8			; done?    
   
	    return
	    
	    
main
;**********************************************************************   
; start of program execution 
;**********************************************************************

init
	bsf		STATUS, RP1		; select memory bank 2
	bcf		STATUS, RP0
	clrf		ANSEL			; make PORTC digital I/O
	clrf		ANSELH
	    
	bsf		STATUS, RP0		; select memory bank 1
	bcf		STATUS, RP1		;
	movlw		0b11111000	    	; set RC0, RC1 and RC2 as outputs		
	andwf		TRISC, F
	;
	
	bcf		STATUS, RP0		; select memory bank 0
	bcf		STATUS, RP1
	bsf		LED, RED		; turn off ALL LEDs
	bsf		LED, GREEN		;
	bsf		LED, BLUE		;

	colour	
	bcf		LED, RED		; turn on RED
	call		wait_1_sec

	bsf		LED, RED		; turn off RED, turn on GREEN 
	bcf		LED, GREEN
	call		wait_1_sec
	
	bsf		LED, GREEN		; turn off GREEN, turn on BLUE
	bcf		LED, BLUE
	call		wait_1_sec
	
	bsf		LED, BLUE		; turn off BLUE
	
	goto		colour
	
	

;	ORG	0x2100				; data EEPROM location
;	DE	1,2,3,4				; define first four EEPROM locations as 1, 2, 3, and 4




	END                       ; directive 'end of program'

