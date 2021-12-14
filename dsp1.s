; - une routine qui tourne en I2S à 50000 hz = replay du sample - buffer ou pas ? sinon 2 registres dédiés
;		- bloque N registres
;		- peut interrompre les 2 autres
; - une routine de remplissage / emulation de Paula, qui remplit un buffer
;	- mixte une voie par une voie
;	- remplit d'abord un buffer de N longs mots dans la ram DSP et les décompte
; - une routine de replay de LSP , à BPM / hertz ( 50 )


; OK - changer de couleur avec le DSP
; OK - jouer un sample a en gros 44 khz avec I2S

; CC (Carry Clear) = %00100
; CS (Carry Set)   = %01000
; EQ (Equal)       = %00010
; MI (Minus)       = %11000
; NE (Not Equal)   = %00001
; PL (Plus)        = %10100
; HI (Higher)      = %00101
; T (True)         = %00000


; one tick = 1.396825ms (clock interval is 279.365 nanoseconds)
; .715909 Mhz NTSC; .709379 Mhz PAL
; PAL 7093,79 ~ 7094 = 10ms   | NTSC 7159,09 ~ 7159 = 10ms

; (.715909 Mhz NTSC; .709379 Mhz PAL)
; 

; sur Amiga : 1773447 ( PAL ) / 1789773 ( NTSC ) / BPM = N
;				709379 ( PAL) / 715909 ( NTSC) / N = frequence hz

; sur Jaguar:
; Timer 1 du DSP
; 26.593900 / prediviser +1 / seconde diviseur +1 .
; pour 50 hz : 26593900 hz => 531 878 => 2 × 73 × 3643 => prediviseur = 146 / 3643
; 

;
; 50 hz = 20 ms = bpm 125


	include	"jaguar.inc"

DSP_LSP_frequence					.equ			20000				; 22300=>17 => 23082 / 
; 7	51935,54688 Hz
; 8	46164,93056 Hz
; 9	41548,4375  Hz
; 12	31960,33654
; 15	25967,77344
; 19	20774,21875
; 51	7990,084135

; emulation paula
;  4 voies
;	- 4 registre = increment virgule
;	- 4 registre = valeur actuelle virgule
;	- 4 reg = increment entier
;	- 4 reg = valeur actuelle entier
;	- 4 reg = volume
;	- 1 reg = octet du sample
;	- 4 reg = fin du sample ?
;	- 1 reg = buffer destination
;	- 1 reg = octet en cours somme des samples

; 4+4+4+4+4+1 = 21 +4 = 25 + 2 = 27




DSP_STACK_SIZE	equ	32	; long words
DSP_USP			equ		(D_ENDRAM-(4*DSP_STACK_SIZE))
DSP_ISP			equ		(DSP_USP-(4*DSP_STACK_SIZE))




.text
.68000
.noclear
                               
	move.l	#INITSTACK, sp	
	move.w	#$06C7, VMODE
    move.w	#%0000011011000111,VMODE	; 320x256
	
	move.w     #$100,JOYSTICK
 
	;move.w	#$07C0, BG				; change la couleur de fond

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


; 	;; I2S
; 	move.l	#%010101,SMODE
; 	move.l	d1,SCLK
	;; set DSP for interrupts
	move.l	#REGPAGE,D_FLAGS
	;; launch the driver
	move.l	#DSP_routine_init_DSP,D_PC
	move.l	#DSPGO,D_CTRL


	lea		LSP_music_data,a0
	lea		LSP_bank_data,a1
	bsr		LSP_PlayerInit

	move.l  #interruption_VBL_68000,LEVEL0     	; Install 68K LEVEL0 handler
    moveq	 #10,d0                	; Must be ODD
;    sub.w   #16,d0
    ori.w   #1,d0
    move.w  d0,VI

    move.w  #1,INT1                 	; Enable video interrupts
    and.w   #$f8ff,sr


boucle:
	
	bra		boucle
	nop

	stop	#$2700


; a0: music data (any mem)
; a1: sound bank data (chip mem)
; // a2: 16bit DMACON word address
LSP_PlayerInit:
			cmpi.l	#'LSP1',(a0)+
			bne		.dataError
			move.l	(a0)+,d0		; unique id
			cmp.l	(a1),d0			; check that sample bank is this one
			bne.s	.dataError

			lea		LSPVars,a3
			cmpi.w	#$0104,(a0)+			; minimal major & minor version of latest compatible LSPConvert.exe
			blt.s	.dataError
			
			move.w	(a0)+,m_currentBpm-LSPVars(a3)				; default BPM / = 125
			move.w	(a0)+,m_escCodeRewind-LSPVars(a3)
			move.w	(a0)+,m_escCodeSetBpm-LSPVars(a3)
			
			move.l	(a0)+,-(a7)				; music len in frame ticks
			
			; move.l	a2,m_dmaconPatch(a3)
			; move.w	#$8000,-1(a2)			; Be sure DMACon word is $8000 (note: a2 should be ODD address)
			
			move.w	(a0)+,d0				; instrument count
			lea		-12(a0),a2				; LSP data has -12 offset on instrument tab ( to win 2 cycles in fast player :) )
			move.l	a2,m_lspInstruments-LSPVars(a3)	; instrument tab addr ( minus 4 )
			subq.w	#1,d0
			move.l	a1,d1

.relocLoop:	;bset.b	#0,3(a0)				; bit0 is relocation done flag
			;bne.s	.relocated
			add.l	d1,(a0)					; 
			add.l	d1,6(a0)
.relocated:	
			lea		12(a0),a0				; un instrument = 12 octets : offset instrument.L, longeur instrument.W, offset debut repetition.L
			dbf		d0,.relocLoop
			
			move.w	(a0)+,d0				; codes count (+2)
			move.l	a0,m_codeTableAddr-LSPVars(a3)	; code table
			add.w	d0,d0
			add.w	d0,a0
			move.l	(a0)+,d0				; word stream size
			move.l	(a0)+,d1				; byte stream loop point
			move.l	(a0)+,d2				; word stream loop point

			move.l	a0,m_wordStream-LSPVars(a3)
			lea		0(a0,d0.l),a1			; byte stream
			move.l	a1,m_byteStream-LSPVars(a3)
			add.l	d2,a0
			add.l	d1,a1
			move.l	a0,m_wordStreamLoop-LSPVars(a3)
			move.l	a1,m_byteStreamLoop-LSPVars(a3)
			;bset.b	#1,$bfe001				; disabling this fucking Low pass filter!!
			lea		m_currentBpm-LSPVars(a3),a0
			move.l	(a7)+,d0				; music len in frame ticks
			rts

.dataError:	illegal

LSP_replay:						; player tick handle ( call this at music player rate )
		lea		LSP_PAULA,a6

		lea		LSPVars,a1
		move.l		(a1),a0					; byte stream
.process:	moveq		#0,d0
.cloop:		move.b		(a0)+,d0
		bne.s		.swCode
		addi.w		#$0100,d0
		bra.s		.cloop
.swCode:	add.w		d0,d0
		move.l		m_codeTableAddr-LSPVars(a1),a2	; code table
		move.w		0(a2,d0.w),d0			; code
		beq		.noInst
		cmp.w		m_escCodeRewind-LSPVars(a1),d0
		beq		.r_rewind
		cmp.w		m_escCodeSetBpm-LSPVars(a1),d0
		beq		.r_chgbpm

		add.b	d0,d0
		bcc.s	.noVd
		move.b	(a0)+,AUD3VOL-LSP_PAULA+3(a6)			; AUD3VOL
.noVd:		add.b	d0,d0
		bcc.s	.noVc
		move.b	(a0)+,AUD2VOL-LSP_PAULA+3(a6)			; AUD2VOL
.noVc:		add.b	d0,d0
		bcc.s	.noVb
		move.b	(a0)+,AUD1VOL-LSP_PAULA+3(a6)			; AUD1VOL
.noVb:		add.b	d0,d0
		bcc.s	.noVa
		move.b	(a0)+,AUD0VOL-LSP_PAULA+3(a6)			; AUD0VOL
.noVa:		
		move.l	a0,(a1)+	; store byte stream ptr
		move.l	(a1),a0		; word stream

		tst.b	d0
		beq.s	.noPa

		add.b	d0,d0
		bcc.s	.noPd
		move.w	(a0)+,AUD3PER-LSP_PAULA+2(a6)
.noPd:		add.b	d0,d0
		bcc.s	.noPc
		move.w	(a0)+,AUD2PER-LSP_PAULA+2(a6)
.noPc:		add.b	d0,d0
		bcc.s	.noPb
		move.w	(a0)+,AUD1PER-LSP_PAULA+2(a6)
.noPb:		add.b	d0,d0
		bcc.s	.noPa
		move.w	(a0)+,AUD0PER-LSP_PAULA+2(a6)
.noPa:		
		tst.w	d0
		beq.s	.noInst

		moveq	#0,d1
		move.l	m_lspInstruments-4-LSPVars(a1),a2	; instrument table
		lea		resetv+12,a4
		lea		3*20(a6),a5							; 3*16
		moveq	#4-1,d2

.vloop:		
		add.w	d0,d0
		bcs.s	.setIns
		add.w	d0,d0
		bcc.s	.skip
		move.l	(a4),a3
		move.l	(a3)+,(a5)				; audio pointeur
		move.w	(a3)+,4+2(a5)			; longeur en words
		bra.s	.skip
.setIns:	
		add.w	(a0)+,a2
		add.w	d0,d0
		bcc.s	.noReset
		bset	d2,d1
		; move.w	d1,$96-$a0(a6)		; dmacon
.noReset:
		move.l	(a2)+,(a5)				; audio pointeur
		move.w	(a2)+,4+2(a5)			; audio length words
		move.l	a2,(a4)					; adresse repeat
.skip:
		subq.w	#4,a4
		lea		-20(a5),a5					; -16
		dbf		d2,.vloop

		;move.l	m_dmaconPatch-4(a1),a3		; dmacon patch
		;move.b	d1,(a3)						; dmacon			

.noInst:	move.l	a0,(a1)			; store word stream (or byte stream if coming from early out)
		rts

.r_rewind:	
		move.l		m_byteStreamLoop-LSPVars(a1),a0
		move.l		m_wordStreamLoop-LSPVars(a1),m_wordStream-LSPVars(a1)
		bra			.process

.r_chgbpm:	
		move.b	(a0)+,m_currentBpm-LSPVars+1(a1)	; BPM
		bra		.process


interruption_VBL_68000:
	movem.l d0-d7/a0-a6,-(a7)

	move.w	#$07C0, BG
	
	bsr		LSP_replay

	move.w  #$101,INT1              	; Signal we're done
    move.w  #$0,INT2	

interruption_VBL_68000_exit:
    movem.l (a7)+,d0-d7/a0-a6
    rte

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
	movei	#DSP_LSP_routine_interruption_I2S,r28					; 6 octets
	movei	#DSP_LSP_pointeur_buffer_dma1,r30						; 6 octets
	;movei	#D_FLAGS,r30											; (6 octets)
	jump	(r28)													; 2 octets
	load	(r30),r29												; 2 octets R29= adresse buffer dma 1
	;load	(r30),r29	; read flags								; (2 octets = 16 octets)
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


DSP_LSP_routine_interruption_Timer1:
DSP_LSP_routine_interruption_Timer2:

; change la couleur de fond
	movei	#DSP_couleur,r27
	load	(r27),r26
	not		r26

	movei	#BG,r22
	storew	r26,(r22)

	movei	#5000,r1
boucle_DSP:
	nop
	subq	#1,r1
	jr		nz,boucle_DSP
	
	not		r26
	storew	r26,(r22)

	store	r26,(r27)

;	movei	#L_I2S,r15	; I2S output
;	sat16s	R24

;	store	r24,(r15+1)				; write left channel
;	store	r24,(r15)				; write right channel
	
	
DSP_LSP_routine_interruption_I2S:

; R28=adresse de saut
; R29=pointeur sur buffer dma 1
; R30=flags/buffer dma 1
; R31=pile
; return from interrupt
	load	(r29),R28					; long word son gauche
	movei	#L_I2S,r30					; R30=destination son DAC
	addq	#4,R29
	store	r28,(R30)
	load	(r29),r28					; long word droit
	addqt	#4,R30
	addq	#4,R29
	store	r28,(R30)
	movei	#DSP_LSP_pointeur_buffer_dma1,r28
	movei	#D_FLAGS,r30											; 6 octets
	store	R29,(R28)
	load	(r30),r29	; read flags
	load	(r31),r28	; return address
	bset	#10,r29		; clear latch 1 = I2S
	;bset	#11,r29		; clear latch 2 = timer 1
	;bset	#12,r29		; clear latch 3 = timer 2
	addq	#4,r31		; pop from stack
	addqt	#2,r28		; next instruction
	bclr	#3,r29		; clear IMASK
	jump	(r28)		; return
	store	r29,(r30)	; restore flags


; routine d'init du DSP
DSP_routine_init_DSP:


; assume run from bank 1
	movei	#DSP_ISP+(DSP_STACK_SIZE*4),r31			; init isp
	moveq	#0,r1
	moveta	r31,r31									; ISP (bank 0)
	movei	#DSP_USP+(DSP_STACK_SIZE*4),r31			; init usp

	movei	#SCLK,r10
	movei	#SMODE,r11
	movei	#DSP_parametre_de_frequence,r12
	movei	#%001101,r13			; SMODE bascule sur RISING
	load	(r12),r12				; SCLK
	store	r12,(r10)
	store	r13,(r11)
; valeurs timer 1

	movei	#146-1,r12				; Timer 1 Pre-scaler
	movei	#3643-1,r13				; Timer 1 Divider

	movei	#JPIT1,r10
	movei	#JPIT2,r11

	storew		r12,(r10)				; JPIT1 = 146-1
	storew		r13,(r11)				; JPIT2 = 3643-1
	
	
	
; enable interrupts
	movei	#D_FLAGS,r28
	;movei	#D_I2SENA|REGPAGE,r29
	movei	#D_TIM1ENA|REGPAGE,r29
	
	store	r29,(r28)

DSP_boucle_centrale:


	jr		DSP_boucle_centrale
	nop

	.phrase

DSP_couleur:	dc.l			0


DSP_frequence_de_replay_calculee:	dc.l		0
DSP_parametre_de_frequence:			dc.l		0

LSPVars:
m_byteStream:		dc.l	0	;  0 byte stream
m_wordStream:		dc.l	0	;  4 word stream
m_dmaconPatch:		dc.l	0	;  8 m_lfmDmaConPatch
m_codeTableAddr:	dc.l	1	; 12 code table addr
m_escCodeRewind:	dc.w	1	; 16 rewind special escape code
m_escCodeSetBpm:	dc.w	1	; 18 set BPM escape code
m_lspInstruments:	dc.l	1	; 20 LSP instruments table addr
m_relocDone:		dc.w	1	; 24 reloc done flag
m_currentBpm:		dc.w	1	; 26 current BPM
m_byteStreamLoop:	dc.l	1	; 28 byte stream loop point
m_wordStreamLoop:	dc.l	1	; 32 word stream loop point
LSPVars_fin:
sizeof_LSPVars		.equ 	LSPVars_fin-LSPVars

			
resetv:	dc.l	0,0,0,0


LSP_PAULA:
; voie 0
AUD0L:		dc.l		1
AUD0LEN:	dc.l		1
AUD0PER:	dc.l		1
AUD0VOL:	dc.l		1
AUD0DAT:	dc.l		1

; voie 1
AUD1L:		dc.l		1
AUD1LEN:	dc.l		1
AUD1PER:	dc.l		1
AUD1VOL:	dc.l		1
AUD1DAT:	dc.l		1

; voie 2
AUD2L:		dc.l		1
AUD2LEN:	dc.l		1
AUD2PER:	dc.l		1
AUD2VOL:	dc.l		1
AUD2DAT:	dc.l		1

; voie 3
AUD3L:		dc.l		1
AUD3LEN:	dc.l		1
AUD3PER:	dc.l		1
AUD3VOL:	dc.l		1
AUD3DAT:	dc.l		1

DSP_LSP_pointeur_buffer_dma1:			dc.l			DSP_LSP_buffer_dma1

DSP_LSP_buffer_dma1:
	.rept		1000
	dc.l		0,0						; droite et gauche
	.endr

	.phrase	
	.68000
DSP_LSP_fin:

DSP_LSP_taille_bloc_memoire			.equ			DSP_LSP_fin - DSP_LSP_debut

LSP_music_data:
	.incbin			"knulla-kuk.lsmusic"
	.even
LSP_bank_data:
	.incbin			"knulla-kuk.lsbank"
	.even

	.bss
	

