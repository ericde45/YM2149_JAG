; OK - changer de couleur avec le DSP
; jouer un sample a en gros 44 khz avec I2S

; CC (Carry Clear) = %00100
; CS (Carry Set)   = %01000
; EQ (Equal)       = %00010
; MI (Minus)       = %11000
; NE (Not Equal)   = %00001
; PL (Plus)        = %10100
; HI (Higher)      = %00101
; T (True)         = %00000


	include	"jaguar.inc"

DSP_LSP_frequence					.equ			44100				; 22300=>17 => 23082 / 
; 7	51935,54688 Hz
; 8	46164,93056 Hz
; 9	41548,4375  Hz
; 12	31960,33654
; 15	25967,77344
; 19	20774,21875
; 51	7990,084135


; timer 1
; 44100
; 26593900 / 44100 = 603 = 3 × 3 × 67 => 9 / 67


DSP_STACK_SIZE	equ	32	; long words
DSP_USP			equ		(D_ENDRAM-(4*DSP_STACK_SIZE))
DSP_ISP			equ		(DSP_USP-(4*DSP_STACK_SIZE))




.text
.68000
.noclear
                               
	move.l	#INITSTACK, sp	
	move.w	#$06C7, VMODE
	;move.w	#$07C0, BG				; change la couleur de fond

	move.w     #$100,JOYSTICK

	move.l	#0,D_CTRL

; copie du code DSP dans la RAM DSP

	lea		DSP_LSP_debut,A0
	lea		D_RAM,A1
	move.l	#DSP_LSP_taille_bloc_memoire,d0


	sub.l	#1,D0
boucle_copie_bloc_DSP:
	move.b	(A0)+,(A1)+
	dbf		D0,boucle_copie_bloc_DSP

; init DSP
	; set timers
	move.l	#DSP_LSP_frequence,d0
	; n = (830968,75/(2*freq))-1 = 25 for 16000hz
        ; f = 830968,75/(2*(n+1))
	move.l	#83096875,d1
	divu.w	d0,d1
	and.l	#$ffff,d1
	divu.w	#200,d1
	and.l	#$ffff,d1
	subq.l	#1,d1
	move.l	d1,DSP_parametre_de_frequence
	
 	addq.l	#1,d1
	mulu.w	#200,d1
 	move.l	#83096875,d0
 	divu.w	d1,d0
	and.l	#$ffff,d0
 	move.l	d0,DSP_frequence_de_replay_calculee

;calcul les increments
; increment = 44100 / DSP_frequence_de_replay_calculee

	move.l	DSP_frequence_de_replay_calculee,d1
	move.l	#44100,d0
	divu	d1,d0
	move.l	d0,d2
	and.l	#$ffff,d0
	
	moveq	#1,d0
	
	move.l	d0,DSP_increment_frequence_entier

	move.l	#44100,d0	
	and.l	#$ffff0000,d2				; d2=reste
	divu	d0,d2
	and.l	#$ffff,d2
	lsl.l	#8,d2
	lsl.l	#8,d2
	
	moveq	#0,d2
	
	move.l	d2,DSP_increment_frequence_virgule

; 	;; I2S
; 	move.l	#%010101,SMODE
; 	move.l	d1,SCLK
	;; set DSP for interrupts
	move.l	#REGPAGE,D_FLAGS
	;; launch the driver
	move.l	#DSP_routine_init_DSP,D_PC
	move.l	#DSPGO,D_CTRL





boucle:
	move.l	DSP_increment_frequence_virgule,d0
	move.l	DSP_increment_frequence_entier,d1
	
	bra		boucle


	stop	#$2700



	.phrase
DSP_LSP_debut:
	.dsp
	.org	D_RAM
DSP_base_memoire:

; CPU interrupt
	.rept	8
		nop
	.endr
; I2S interrupt
	movei	#DSP_LSP_routine_interruption,r28						; 6 octets
	movei	#D_FLAGS,r30											; 6 octets
	jump	(r28)													; 2 octets
	load	(r30),r29	; read flags								; 2 octets = 16 octets
; Timer 1 interrupt
	movei	#DSP_LSP_routine_interruption_Timer1,r28						; 6 octets
	movei	#D_FLAGS,r30											; 6 octets
	jump	(r28)													; 2 octets
	load	(r30),r29	; read flags								; 2 octets = 16 octets
; Timer 2 interrupt	
	movei	#DSP_LSP_routine_interruption_Timer2,r28						; 6 octets
	movei	#D_FLAGS,r30											; 6 octets
	jump	(r28)													; 2 octets
	load	(r30),r29	; read flags								; 2 octets = 16 octets
; External 0 interrupt
	.rept	8
		nop
	.endr
; External 1 interrupt
	.rept	8
		nop
	.endr

; -------------------------------
; DSP : routine en interruption
; -------------------------------

DSP_LSP_routine_interruption:
; return from interrupt I2S
	load	(r31),r28	; return address
	bset	#10,r29		; clear latch 1 = I2S
	
	bclr	#3,r29		; clear IMASK
	addq	#4,r31		; pop from stack
	addqt	#2,r28		; next instruction
	jump	t,(r28)		; return
	store	r29,(r30)	; restore flags

DSP_LSP_routine_interruption_Timer1:
DSP_LSP_routine_interruption_Timer2:
; change la couleur de fond
	movei	#DSP_couleur,r27
	movei	#$FFFF0,r25
	load	(r27),r26
	addq	#1,r26
	;and		R25,R26

	cmp 	r25,r26
	jr		mi,DSP_LSP_routine_interruption_OK
	nop
	
	movei	#0,r26
	nop
	
DSP_LSP_routine_interruption_OK:
	store	r26,(r27)
	movei	#BG,r27
	storew	r26,(r27)

; fais un son
	movei	#DSP_pointeur_debut_sample,r24
	load	(R24),R24
	movei	#DSP_offset_actuel_replay_sample_entier,r23
	load	(R23),R22
	add		R22,R24
	
	movei	#DSP_increment_frequence_virgule,R21
	movei	#DSP_increment_frequence_entier,R20
	load	(R21),R21
	movei	#DSP_offset_actuel_replay_sample_virgule,R19
	load	(R20),R20
	load	(R19),R18

	add		R21,R18
	addc	R20,R22

	store	R18,(R19)			; on sauvegarde la partie a virgule de l'offset
	
	
	loadb	(R24),R24
;	movei	#128,R21
;	add		R21,R24

	movei	#$80,R15
	sub		R15,R24			; signe le sample

	movei	#-8,R15
	sha		R15,R24


	movei	#L_I2S,r15	; I2S output
	;sat16s	R24

	store	R22,(R23)
	store	r24,(r15+1)				; write left channel
	store	r24,(r15)				; write right channel
	
; return from interrupt
	load	(r31),r28	; return address
	;bset	#10,r29		; clear latch 1 = I2S
	;bset	#11,r29		; clear latch 1 = timer 1
	bset	#12,r29		; clear latch 1 = timer 1
	
	
	bclr	#3,r29		; clear IMASK
	addq	#4,r31		; pop from stack
	addqt	#2,r28		; next instruction
	jump	t,(r28)		; return
	store	r29,(r30)	; restore flags


; routine d'init du DSP
DSP_routine_init_DSP:


; assume run from bank 1
	movei	#DSP_ISP+(DSP_STACK_SIZE*4),r31			; init isp
	moveq	#0,r1
	moveta	r31,r31									; ISP (bank 0)
	movei	#DSP_USP+(DSP_STACK_SIZE*4),r31			; init usp

	;movei	#SCLK,r10
	;movei	#SMODE,r11
	;movei	#DSP_parametre_de_frequence,r12
	;movei	#%001101,r13			; SMODE bascule sur RISING
	;load	(r12),r12	; SCLK
	;store	r12,(r10)
	;store	r13,(r11)
; valeurs timer 1

	;movei	#JPIT1,r10
	;movei	#JPIT2,r11

	movei	#JPIT3,r10
	movei	#JPIT4,r11


	movei	#9-1,r12				; Timer 1 Pre-scaler  9
	movei	#67-1,r13				; Timer 1 Divider  67



	storew	r12,(r10)				; JPIT1 = 9-1
	storew	r13,(r11)				; JPIT2 = 67-1


; enable interrupts
	movei	#D_FLAGS,r28
	;movei	#D_I2SENA|REGPAGE,r29
	movei	#D_TIM2ENA|REGPAGE,r29
	store	r29,(r28)

DSP_boucle_centrale:


	jr		DSP_boucle_centrale
	nop

	.phrase
DSP_couleur:							dc.l		0
DSP_parametre_de_frequence:				dc.l		0
DSP_frequence_de_replay_calculee:		dc.l		1
DSP_pointeur_debut_sample:	dc.l		sample
DSP_pointeur_fin_sample:	dc.l		fin_sample
DSP_taille_sample:			dc.l		fin_sample-sample
DSP_offset_actuel_replay_sample_entier:		dc.l		0
DSP_offset_actuel_replay_sample_virgule:	dc.l		0
; 44100 // 46164
DSP_increment_frequence_entier:				dc.l		0
DSP_increment_frequence_virgule:			dc.l		$1111							; 2051469302<<1 				; (44100 / 46164)<<32

	.phrase	
	.68000
DSP_LSP_fin:

DSP_LSP_taille_bloc_memoire			.equ			DSP_LSP_fin - DSP_LSP_debut

sample:
	.incbin		"ulm8u.raw"					; 44100 hz / signed
fin_sample:
	.bss
	

