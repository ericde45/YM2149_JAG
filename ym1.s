; YM7 sur Jaguar
;
; TODO :
;			- placer la copie des 2 samples en tout debut de routine interruption I2S ( vue la variabilité liée à la génération du Noise )

; timer I2S = generation des samples
; Timer 1 = routine de replay

	include	"jaguar.inc"

; algo de la routine qui genere les samples
; 3 canaux : increment onde carrée * 3 , increment noise, volume voie * 3 , increment enveloppe

	
	
DSP_Audio_frequence					.equ			20774				; 22300=>17 => 23082 / 
YM_frequence_YM2149					.equ			2000000				; 2 000 000 = Atari ST , 1 000 000 Hz = Amstrad CPC, 1 773 400 Hz = ZX spectrum 
;YM_frequence_predivise				.equ			394339				; ((YM_frequence_YM2149/16)*65536)/DSP_Audio_frequence


; 7	51935,54688 Hz
; 8	46164,93056 Hz
; 9	41548,4375  Hz
; 12	31960,33654
; 15	25967,77344
; 19	20774,21875
; 51	7990,084135


; Timer 1 du DSP
; 26.593900 / prediviser +1 / seconde diviseur +1 .
; pour 50 hz : 26593900 hz => 531 878 => 2 × 73 × 3643 => prediviseur = 146 / 3643

DSP_STACK_SIZE	equ	32	; long words
DSP_USP			equ		(D_ENDRAM-(4*DSP_STACK_SIZE))
DSP_ISP			equ		(DSP_USP-(4*DSP_STACK_SIZE))

.opt "~Oall"

.text
.68000
;.noclear

	
                               
	move.l		#INITSTACK, sp	
	move.w		#$06C7, VMODE
    move.w		#%0000011011000111,VMODE	; 320x256
	
	move.w		#$100,JOYSTICK

	bsr			YM_init_ym7

; ------------------------
; debut DSP
	move.l	#0,D_CTRL

; copie du code DSP dans la RAM DSP

	lea		YM_DSP_debut,A0
	lea		D_RAM,A1
	move.l	#YM_DSP_fin-DSP_base_memoire,d0


	sub.l	#1,D0
boucle_copie_bloc_DSP:
	move.b	(A0)+,(A1)+
	dbf		D0,boucle_copie_bloc_DSP


; init DSP
	; set timers
	move.l	#DSP_Audio_frequence,d0
	; n = (830968,75/(2*freq))-1 = 25 for 16000hz
        ; f = 830968,75/(2*(n+1))
	move.l	#83096875,d1
	divu.w	d0,d1
	and.l	#$ffff,d1
	divu.w	#200,d1
	and.l	#$ffff,d1
	subq.l	#1,d1
	move.l	d1,DSP_parametre_de_frequence_I2S
	
 	addq.l	#1,d1
	mulu.w	#200,d1
 	move.l	#83096875,d0
 	divu.w	d1,d0
	and.l	#$ffff,d0
 	move.l	d0,DSP_frequence_de_replay_reelle_I2S



; launch DSP

	move.l	#REGPAGE,D_FLAGS
	move.l	#DSP_routine_init_DSP,D_PC
	move.l	#DSPGO,D_CTRL


toto:

	move.l		YM_DSP_increment_canal_A,d0
	move.l		tmpregistre0,d1
	move.l		tmpregistre1,d3
	move.l		YM_frequence_predivise,d2
	move.l		YM_pointeur_actuel_ymdata,a0
	bra			toto

	stop		#$2700



;----------------------------------------------------
YM_init_ym7:
; tout le long de l'init D6=YM_nb_registres_par_frame

	lea			fichier_ym7,A0
	
	lea			12(a0),a0					; on saute YM7! et Leonard! = +12
	lea			4(a0),a0					; +4 : 0202

	move.l		(a0)+,YM_nombre_de_frames_totales					; .L=nombre de frames => c05756
	move.w		(a0)+,YM_frequence_replay							; .w=frequence du replay ( 50 hz ?)
	move.w		(a0)+,d0
	move.w		d0,YM_flag_effets_sur_les_voies					; .w= effet sur la voie. A=bit 0, B=bit 1, C=bit 2

	moveq.l		#14,d6										; nb d'octet par frame = 14 de base = 14 registres
	moveq		#1,d1

	btst		#0,d0						; effet sur voie A ?
	beq.s		YM_init_ym7_pas_effet_sur_voie_A
	addq.l		#2,d6						; + 2 registres à lire lors du replay
	move.w		d1,YM_flag_effets_voie_A
YM_init_ym7_pas_effet_sur_voie_A:

	btst		#1,d0						; effet sur voie B ?
	beq.s		YM_init_ym7_pas_effet_sur_voie_B
	addq.l		#2,d6						; + 2 registres à lire lors du replay
	move.w		d1,YM_flag_effets_voie_B
YM_init_ym7_pas_effet_sur_voie_B:

	btst		#2,d0						; effet sur voie C ?
	beq.s		YM_init_ym7_pas_effet_sur_voie_C
	addq.l		#2,d6						; + 2 registres à lire lors du replay
	move.w		d1,YM_flag_effets_voie_C
YM_init_ym7_pas_effet_sur_voie_C:

	btst		#4,d0						; effet D Sinus Sid ?
	beq.s		YM_init_ym7_pas_effet_Sinus_Sid
	addq.l		#1,d6						; + 2 registres à lire lors du replay
	move.w		d1,YM_flag_effets_Sinus_Sid
YM_init_ym7_pas_effet_Sinus_Sid:

	move.w		d6,YM_nb_registres_par_frame

; debut init DG
	move.w		(a0)+,d7								; .w=nombre de digidrums
	beq.s		YM_pas_de_digidrums						; => pas de digidrums

; TODO : gérer les digidrums
	addq.l		#1,d6
	illegal
	
YM_pas_de_digidrums:
; debut init SID 
	move.w		(a0)+,d7								; .w= nombre de SID à fabriquer
	beq.s		YM_pas_de_SID						; => pas de SID

; TODO : gérer les SID
	illegal

YM_pas_de_SID:
; debut init 	Buzzer
	move.w		(a0)+,d7									; valeur dans le fichier YM7 = nombre de buzzer ( apres entete, digidrums et table SID )
	beq.s		YM_pas_de_Buzzer

; TODO : gérer les buzzer
	illegal

YM_pas_de_Buzzer:
	move.w		(a0)+,d7									; D7=nb samples sinus sid
	beq.s		YM_pas_de_Sinus_Sid

; TODO : gérer les Sinus Sid
	illegal

YM_pas_de_Sinus_Sid:

	lea			2(a0),a0						; saute la valeur 64 ( LZ4 )

	moveq		#0,d0
	
	move.w	 	(a0)+,d0
	move.w		d0,YM_ecart_entre_frames_blocs				; ecart entre les frames pour les N-1 premiers blocs / format YM7 = .w= ?   par exemple 0C10   ( stocké en C0576E / ecart entre les frames ?
	move.l		d0,PSG_ecart_entre_les_registres_ymdata
	move.w		(a0)+,YM_ecart_entre_frames_dernier_bloc		; ecart entre les frames pour le dernier blocs / format YM7 = .w= ?   par exemple 0C0E   ( stocké en C05784
	move.w		(a0)+,d0										; nombre de bloc compressés / format YM7 = .w= ? de 1 à 70 / par exemple 0004	( stocké en C0576A ) = nombre de blocs de donn�es compress�es ?
	move.w		d0,YM_nombre_de_blocs_lZ4

; A0=pointeur sur les tailles des blocs
	move.l		a0,YM_LZ4_pointeur_tailles_des_blocs

	lea			YM_tableau_des_blocs_decompresses,a2
	move.w		YM_ecart_entre_frames_blocs,d1

YM_init_ym7_boucle_creation_liste_des_blocs_decompresses_copie_ecarts:
	addq.l		#4,a2												; saute par dessus le pointeur sur le bloc
	subq		#1,d0
	beq.s		YM_init_ym7_creation_liste_des_blocs_decompresses_copie_ecarts_dernier_bloc
	
	move.w		d1,(a2)+
	bra.s		YM_init_ym7_boucle_creation_liste_des_blocs_decompresses_copie_ecarts

YM_init_ym7_creation_liste_des_blocs_decompresses_copie_ecarts_dernier_bloc:
	move.w		YM_ecart_entre_frames_dernier_bloc,d1
	move.w		d1,(a2)+


; allouer la ram pour compteur de frames * nb d'octets par frame

	move.l		YM_nombre_de_frames_totales,d0
	moveq		#0,d1
	move.w		YM_nb_registres_par_frame,d1
	mulu		d1,d0
; d0=taille à allouer
	bsr.s		YM_malloc
; d0=pointeur sur la zone memoire vide

	move.l		d0,YM_pointeur_origine_ymdata
	move.l		d0,YM_LZ4_pointeur_destination_bloc_actuel
	
; decompression des blocs
; - lire taille du bloc compressés
; - 

	move.l		YM_LZ4_pointeur_destination_bloc_actuel,a1

	move.l		YM_LZ4_pointeur_tailles_des_blocs,a4
	moveq		#0,d5
	move.w		YM_nombre_de_blocs_lZ4,d5
	move.l		d5,d4
	add.w		d5,d5						; nb bloc * 2
	lea			(a4,d5.w),a5				; A5 = pointeur sur les donnees du bloc 1

	lea			YM_tableau_des_blocs_decompresses,a2

YM_init_ym7_boucle_decomp_blocs:
	move.l		a1,(a2)+					; pointeur sur bloc decompresse
	addq.l		#2,a2						; saute le nb de frame du bloc
	
	moveq		#0,d0
	move.w		(a4)+,d0					; taille du bloc compresse

	move.l		a5,a0						; A0=pointeur sur packed bloc
	add.l		d0,a5						; A5 = pointe sur bloc LZ4 suivant

; input :
; / a0.l = packed buffer 
; / a1.l = output buffer 
; /  d0.l = LZ4 packed block size (in bytes)
	bsr.s		lz4_depack

; / a1=fin du bloc decompresse


	subq		#1,d4
	bgt			YM_init_ym7_boucle_decomp_blocs

	lea			YM_tableau_des_blocs_decompresses,a2
	move.l		(a2),a1						; A1 = pointeur sur premier bloc decompresse
	move.l		a1,YM_pointeur_actuel_ymdata
	
	moveq		#0,d0
	move.w		d0,YM_numero_bloc_en_cours
	

	rts

; ---------------------------------------
; allocation de mémoire
; malloc de d0, retour avec  un pointeur dans d0
; ---------------------------------------

YM_malloc:

	movem.l		d1-d3/a0,-(sp)

	move.l		debut_ram_libre,d1
	move.l		d1,a0
	move.l		d1,d3
	add.l		d0,d1
; arrondit multiple de 2
    btst		#0,d1
	beq.s		YM_malloc_pas_d_arrondi
	addq.l		#1,d1
YM_malloc_pas_d_arrondi:
	move.l		d1,debut_ram_libre
	
	move.l		d0,d2
	subq.l		#1,d2
	moveq.l		#0,d0

YM_malloc_boucle_clean_ram:
	move.b		d0,(a0)+
	dbf			d2,YM_malloc_boucle_clean_ram
	
	move.l		d3,d0

	movem.l		(sp)+,d1-d3/a0
	rts



depack_chunk_lz4:
; decompression LZ4		 https://github.com/arnaud-carre/lz4-68k/blob/master/lz4_normal.asm
; / a0.l = packed buffer 
; / a1.l = output buffer 
; /  d0.l = LZ4 packed block size (in bytes)

;---------------------------------------------------------
;
;	LZ4 block 68k depacker
;	Written by Arnaud Carré ( @leonard_coder )
;	https://github.com/arnaud-carre/lz4-68k
;
;	LZ4 technology by Yann Collet ( https://lz4.github.io/lz4/ )
;
;---------------------------------------------------------

; Normal version: 180 bytes ( 1.53 times faster than lz4_smallest.asm )
;
; input: a0.l : packed buffer
;		 a1.l : output buffer
;		 d0.l : LZ4 packed block size (in bytes)
;
; output: none
; a1=pointe sur la fin du bloc decompresse
;

lz4_depack:
			movem.l	d1-d4/a3-a4,-(sp)

			lea		0(a0,d0.l),a4	; packed buffer end

			moveq	#0,d0
			moveq	#0,d2
			moveq	#0,d3
			moveq	#15,d4
			bra.s	.tokenLoop
			
.lenOffset:	move.b	(a0)+,d1	; read 16bits offset, little endian, unaligned
			move.b	(a0)+,-(a7)
			move.w	(a7)+,d3
			move.b	d1,d3
			movea.l	a1,a3
			sub.l	d3,a3
			moveq	#$f,d1
			and.w	d0,d1
			cmp.b	d4,d1
			bne.s	.small

.readLen0:	move.b	(a0)+,d2
			add.l	d2,d1
			not.b	d2
			beq		.readLen0

			addq.l	#4,d1
.copy:		move.b	(a3)+,(a1)+
			subq.l	#1,d1
			bne.s	.copy
			bra.s	.tokenLoop
			
.small:		add.w	d1,d1
			neg.w	d1
			jmp		.copys(pc,d1.w)
			.rept	15
			move.b	(a3)+,(a1)+
			.endr
.copys:
			move.b	(a3)+,(a1)+
			move.b	(a3)+,(a1)+
			move.b	(a3)+,(a1)+
			move.b	(a3)+,(a1)+
			
.tokenLoop:	move.b	(a0)+,d0
			move.l	d0,d1
			lsr.b	#4,d1
			beq		.lenOffset
			cmp.b	d4,d1
			beq.s	.readLen1

.litcopys:	add.w	d1,d1
			neg.w	d1
			jmp		.copys2(pc,d1.w)
			.rept	15
				move.b	(a0)+,(a1)+
			.endr
.copys2:
			cmpa.l	a0,a4
			bne		.lenOffset
			movem.l	(sp)+,d1-d4/a3-a4
			rts
						
.readLen1:	move.b	(a0)+,d2
			add.l	d2,d1
			not.b	d2
			beq		.readLen1

.litcopy:	move.b	(a0)+,(a1)+
			subq.l	#1,d1
			bne		.litcopy

			; end test is always done just after literals
			cmpa.l	a0,a4
			bne		.lenOffset
			
.over:
			movem.l		(sp)+,d1-d4/a3-a4
			rts						; end


;-------------------------------------
;
;     DSP
;
;-------------------------------------

	.phrase
YM_DSP_debut:

	.dsp
	.org	D_RAM
DSP_base_memoire:

; CPU interrupt
	.rept	8
		nop
	.endr
; I2S interrupt
	movei	#DSP_LSP_routine_interruption_I2S,r28						; 6 octets
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
; DSP : routines en interruption
; -------------------------------
DSP_LSP_routine_interruption_I2S:
;-------------------------------------------------------------------------------------------------
;
; routine de replay, fabrication des samples
;
;-------------------------------------------------------------------------------------------------
; R28/R29/R30/R31 : utilisé par l'interruption

; - calculer le prochain noise : 0 ou $FFFF
; - calculer le prochain volume enveloppe
; - un canal = ( mixer

;		bt = ((((yms32)posA)>>31) | mixerTA) & (bn | mixerNA);
; (onde carrée normale OU mixerTA ) ET ( noise OU mixerNA ) 

;		vol  = (*pVolA)&bt;
;		volume ( suivant le pointeur, enveloppe ou fixe) ET mask du dessus
; - increment des positions apres : position A B C, position noise, position enveloppe

; mask = (mixerTA OR Tone calculé par frequence) AND ( mixerNA OR
; avec Tone calculé = FFFFFFFF bit 31=1 : bit 31 >> 31 = 1 : NEG 1 = -1

;--------------------------
; gerer l'enveloppe
; - incrementer l'offset enveloppe
; partie entiere 16 bits : virgule 16 bits
; partie entiere and %1111 = position dans la sous partie d'enveloppe
; ( ( partie entiere >> 4 ) and %1 ) << 2 = pointeur sur la sous partie d'enveloppe

	movei	#YM_DSP_increment_enveloppe,R27
	movei	#YM_DSP_offset_enveloppe,R26
	load	(R27),R27
	load	(R26),R25				; R25 = offset en cours enveloppe
	add		R27,R25
	store	R25,(R26)
	
	sharq	#16,R25					; on vire la virgule, on garde le signe
	moveq	#%1111,R26
	move	R25,R27
	and		R26,R27					; R27=partie entiere de l'offset AND 1111 = position dans la sous partie d'enveloppe
	
	movei	#YM_DSP_pointeur_enveloppe_en_cours,R24
	sharq	#4,R25					; offset / 16, on garde le signe
	jr		mi, YM_DSP_replay_sample_offset_env_negatif
	moveq	#%1,R26
	and		R26,R25					; R25 = pointeur sur la sous partie d'enveloppe
	
YM_DSP_replay_sample_offset_env_negatif:
	load	(R24),R24
	add		R25,R25					; R25*2
	add		R27,R27					; R27*2
	add		R25,R25					; R25*4
	add		R27,R27					; R27*4
	add		R25,R24					; R24 = pointeur sur la partie d'enveloppe actuelle
	load	(R24),R24				; R24 = pointeur sur la partie d'enveloppe actuelle
	movei	#YM_DSP_volE,R26
	add		R27,R24					
	load	(R24),R24				; R24 = volume actuel enveloppe
	store	R24,(R26)				; volume de l'enveloppe => YM_DSP_volE

; edz : si R25 est natif, le and ne marche plus

;--------------------------
; gérer le noise
; on avance le step de noise
; 	si on a 16 bits du haut>0 => on genere un nouveau noise
; 	et on masque le bas avec $FFFF
; l'increment de frequence du Noise est en 16:16

	movei	#YM_DSP_increment_Noise,R27
	movei	#YM_DSP_position_offset_Noise,R26
	movei	#YM_DSP_current_Noise_mask,R22
	load	(R27),R27
	load	(R26),R24
	load	(R22),R18			; R18 = current mask Noise
	add		R27,R24
	move	R24,R23
	shrq	#16,R23				; R23 = partie entiere, à zéro ?
	movei	#YM_DSP_replay_sample_pas_de_generation_nouveau_Noise,R20
	jump	eq,(R20)
	nop
; il faut generer un nouveau noise
; il faut masquer R24 avec $FFFF
	movei	#$FFFF,R23
	and		R23,R24				; YM_DSP_position_offset_Noise, juste virgule

; genere un nouveau Noise pseudo random
	movei	#YM_DSP_current_Noise,R23
	moveq	#1,R20
	load	(R23),R21
	move	R21,R27
	shrq	#3,R27
	xor		R21,R27
	and		R20,R27
	add		R27,R27
	rorq	#16,R27
	xor		R27,R21
	store	R21,(R23)

; calcul masque 
	and		R20,R21			; on gatde juste le bit 0
	sub		R20,R21			; 0-1= -1 / 1-1=0 => mask sur 32 bits
	store	R21,(R22)		; R21=>YM_DSP_current_Noise_mask
	move	R21,R18

YM_DSP_replay_sample_pas_de_generation_nouveau_Noise:
; en entrée : R24 = offset noise, R18 = current mask Noise

	store	R24,(R26)			; R24=>YM_DSP_position_offset_Noise
;---- R18 = mask current Noise ----

; ---------------
; canal A
	movei	#YM_DSP_Mixer_NA,R26
	move	R18,R24				; R24 = on garde la masque du current Noise
	
	
	load	(R26),R26
	or		R26,R18

; R18 = Noise OR mask du registre 7 de mixage du Noise A


	movei	#YM_DSP_increment_canal_A,R27
	movei	#YM_DSP_position_offset_A,R26
	load	(R27),R27
	load	(R26),R25
	add		R27,R25
	store	R25,(R26)							; YM_DSP_position_offset_A
	shrq	#31,R25
	neg		R25									; 0 devient 0, 1 devient -1 ($FFFFFFFF)
; R25 = onde carrée A

	movei	#YM_DSP_Mixer_TA,R26
	load	(R26),R26
	or		R26,R25
; R25 = onde carrée A OR mask du registre 7 de mixage Tone A


; Noise AND Tone

	movei	#YM_DSP_pointeur_sur_source_du_volume_A,R20
	and		R18,R25					; R25 = Noise and Tone
	load	(R20),R20				; R20 = pointeur sur la source de volume pour le canal A
	load	(r20),R20				; R20=volume pour le canal A 0 à 32767
	and		R25,R20					; R20=volume pour le canal A
; R20 = sample canal A

; ---------------
; canal B
	movei	#YM_DSP_Mixer_NB,R26
	move	R24,R18				; R24 = masque du current Noise
	
	load	(R26),R26
	or		R26,R18

; R18 = Noise OR mask du registre 7 de mixage du Noise B

	movei	#YM_DSP_increment_canal_B,R27
	movei	#YM_DSP_position_offset_B,R26
	load	(R27),R27
	load	(R26),R25
	add		R27,R25
	store	R25,(R26)							; YM_DSP_position_offset_B
	shrq	#31,R25
	neg		R25									; 0 devient 0, 1 devient -1 ($FFFFFFFF)
; R25 = onde carrée B

	movei	#YM_DSP_Mixer_TB,R26
	load	(R26),R26
	or		R26,R25
; R25 = onde carrée B OR mask du registre 7 de mixage Tone B

; Noise AND Tone

	movei	#YM_DSP_pointeur_sur_source_du_volume_B,R23
	and		R18,R25					; R25 = Noise and Tone
	load	(R23),R23				; R23 = pointeur sur la source de volume pour le canal B
	load	(r23),R23				; R23=volume pour le canal B 0 à 32767
	and		R25,R23					; R23=volume pour le canal B
; R23 = sample canal B

; ---------------
; canal C
	movei	#YM_DSP_Mixer_NC,R26
	move	R24,R18				; R24 = masque du current Noise
	
	load	(R26),R26
	or		R26,R18

; R18 = Noise OR mask du registre 7 de mixage du Noise C

	movei	#YM_DSP_increment_canal_C,R27
	movei	#YM_DSP_position_offset_C,R26
	load	(R27),R27
	load	(R26),R25
	add		R27,R25
	store	R25,(R26)							; YM_DSP_position_offset_C
	shrq	#31,R25
	neg		R25									; 0 devient 0, 1 devient -1 ($FFFFFFFF)
; R25 = onde carrée C

	movei	#YM_DSP_Mixer_TC,R26
	load	(R26),R26
	or		R26,R25
; R25 = onde carrée C OR mask du registre 7 de mixage Tone C

; Noise AND Tone

	movei	#YM_DSP_pointeur_sur_source_du_volume_C,R22
	and		R18,R25					; R25 = Noise and Tone
	load	(R22),R22				; R22 = pointeur sur la source de volume pour le canal C
	load	(r22),R22				; R22=volume pour le canal C 0 à 32767
	and		R25,R22					; R22=volume pour le canal C
; R22 = sample canal C

	movei	#32768,R27
	add		R23,R20					; R20 = canal A + canal B
	movei	#L_I2S,r26
	sub		R27,R20					; resultat signé sur 16 bits
	
	sub		R27,R22					; canal C / resultat signé sur 16 bits

; envoi du sample
	store	r22,(r26)				; write right channel
	addq	#4,R26
	store	r20,(r26)				; write left channel
	
; return from interrupt
	load	(r31),r28	; return address
	bset	#10,r29		; clear latch 1 = I2S
	;bset	#11,r29		; clear latch 1 = timer 1
	;bset	#12,r29		; clear latch 1 = timer 1	
	bclr	#3,r29		; clear IMASK
	addq	#4,r31		; pop from stack
	addqt	#2,r28		; next instruction
	jump	t,(r28)		; return
	store	r29,(r30)	; restore flags



DSP_LSP_routine_interruption_Timer1:



DSP_LSP_routine_interruption_Timer2:


; routine d'init du DSP
; registres bloqués par les interruptions : R29/R30/R31 ?
DSP_routine_init_DSP:

; -------------------------------------------------------------------------------
; calcul de la frequence prédivisee pour le YM
; ((YM_frequence_YM2149/16)*65536)/DSP_Audio_frequence

	movei	#YM_frequence_YM2149,r0
	shlq	#16-4-2,r0					; /16 puis * 65536
	
	movei	#DSP_frequence_de_replay_reelle_I2S,r1
	load	(r1),r1
	
	div		r1,r0
	shlq	#2,r0					; ramene a *65536

	
	movei	#YM_frequence_predivise,r1
	store	r0,(r1)


	




; -------------------------------------------------------------------------------
; routine de lecture des registres YM
	movei		#YM_pointeur_actuel_ymdata,R0
	movei		#PSG_ecart_entre_les_registres_ymdata,R3
	load		(R0),R1
	load		(R3),R8



; round(  ((freq_YM / 16) / frequence_replay) * 65536) /x;	
; 
; registres 0+1 = frequence voie A
	loadb		(R1),R2						; registre 0
	add			R8,R1
	loadb		(R1),R3						; registre 1
	movei		#%1111,R7
	add			R8,R1

	and			R7,R3
	movei		#YM_frequence_predivise,R5
	shlq		#8,R3
	load		(R5),R5
	add			R2,R3						; R3 = frequence YM canal A
	
	move		R5,R6
	
	div			r3,R5
	shlq		#15,R5
	
	movei		#YM_DSP_increment_canal_A,R2
	store		R5,(R2)

; registres 2+3 = frequence voie B
	loadb		(R1),R2						; registre 2
	add			R8,R1
	loadb		(R1),R3						; registre 3
	add			R8,R1

	and			R7,R3
	shlq		#8,R3
	move		R6,R5						; R5=YM_frequence_predivise
	add			R2,R3						; R3 = frequence YM canal B
	
	div			r3,R5
	shlq		#15,R5
	
	movei		#YM_DSP_increment_canal_B,R2
	store		R5,(R2)
	
; registres 4+5 = frequence voie C
	loadb		(R1),R2						; registre 4
	add			R8,R1
	loadb		(R1),R3						; registre 5
	add			R8,R1

	and			R7,R3
	shlq		#8,R3
	move		R6,R5						; R5=YM_frequence_predivise
	add			R2,R3						; R3 = frequence YM canal C
	
	div			r3,R5
	shlq		#15,R5
	
	movei		#YM_DSP_increment_canal_C,R2
	store		R5,(R2)
	
; registre 6
; 5 bit noise frequency
	loadb		(R1),R2						; registre 6
	movei		#%11111,R7
	add			R8,R1
	
	and			R7,R2						; on ne garde que 5 bits
	jr			ne,DSP_lecture_registre6_pas_zero
	move		R6,R5						; R5=YM_frequence_predivise

	moveq		#1,R2
DSP_lecture_registre6_pas_zero:
	
	movei		#YM_DSP_increment_Noise,R3
	div			R2,R5
	; shlq		#15,R5						; on laisse l'increment frequence Noise sur 16(entier):16(virgule)
	store		R5,(R3)

; registre 7 
; 6 bits interessants
;	Noise	 Tone
;	C B A    C B A
	loadb		(R1),R2						; registre 7
	add			R8,R1

; bit 0 = Tone A
	move		R2,R4
	moveq		#%1,R3
	and			R3,R4					; 0 ou 1
	movei		#YM_DSP_Mixer_TA,R5
	subq		#1,R4					; 0=>-1 / 1=>0 
	shlq		#1,R3					; bit suivant
	store		R4,(R5)

; bit 1 = Tone B
	move		R2,R4
	movei		#YM_DSP_Mixer_TB,R5
	and			R3,R4					; 0 ou 1
	shrq		#1,R4
	subq		#1,R4					; 0=>-1 / 1=>0 
	shlq		#1,R3					; bit suivant
	store		R4,(R5)

; bit 2 = Tone C
	move		R2,R4
	movei		#YM_DSP_Mixer_TC,R5
	and			R3,R4					; 0 ou 1
	shrq		#2,R4
	subq		#1,R4					; 0=>-1 / 1=>0 
	shlq		#1,R3					; bit suivant
	store		R4,(R5)
	
; bit 3 = Noise A
	move		R2,R4
	movei		#YM_DSP_Mixer_NA,R5
	and			R3,R4					; 0 ou 1
	shrq		#3,R4
	subq		#1,R4					; 0=>-1 / 1=>0 
	shlq		#1,R3					; bit suivant
	store		R4,(R5)
	
; bit 4 = Noise B
	move		R2,R4
	movei		#YM_DSP_Mixer_NB,R5
	and			R3,R4					; 0 ou 1
	shrq		#4,R4
	subq		#1,R4					; 0=>-1 / 1=>0 
	shlq		#1,R3					; bit suivant
	store		R4,(R5)
	
; bit 5 = Noise C
	move		R2,R4
	movei		#YM_DSP_Mixer_NC,R5
	and			R3,R4					; 0 ou 1
	shrq		#5,R4
	subq		#1,R4					; 0=>-1 / 1=>0 
	shlq		#1,R3					; bit suivant
	store		R4,(R5)
	

	movei		#YM_DSP_table_de_volumes,R14

; registre 8 = volume canal A
; B4=1 bit =M / M=0=>volume fixe / M=1=>volume enveloppe
; B3/B2/B1/B0 = volume fixe pour le canal A
;	Noise	 Tone
;	C B A    C B A
	loadb		(R1),R2						; registre 8
	add			R8,R1	

	moveq		#%1111,R3
	move		R2,R4
	movei		#YM_DSP_volA,R5
	and			R3,R4
	
	shlq		#2,R4					; volume sur 16 *4 
	load		(R14+R4),R4

	store		R4,(R5)

	movei		#YM_DSP_pointeur_sur_source_du_volume_A,R3
	btst		#4,R2
	jr			eq,DSP_lecture_registre8_pas_env
	nop
	

	movei		#YM_DSP_volE,R5
	
DSP_lecture_registre8_pas_env:
	store		R5,(R3)


; registre 9 = volume canal B
; B4=1 bit =M / M=0=>volume fixe / M=1=>volume enveloppe
; B3/B2/B1/B0 = volume fixe pour le canal B
;	Noise	 Tone
;	C B A    C B A
	loadb		(R1),R2						; registre 9
	add			R8,R1	

	moveq		#%1111,R3
	move		R2,R4
	movei		#YM_DSP_volB,R5
	and			R3,R4

	shlq		#2,R4					; volume sur 16 *4 
	load		(R14+R4),R4

	store		R4,(R5)

	movei		#YM_DSP_pointeur_sur_source_du_volume_B,R3

	btst		#4,R2
	jr			eq,DSP_lecture_registre9_pas_env
	nop
	
	movei		#YM_DSP_volE,R5
	
DSP_lecture_registre9_pas_env:
	store		R5,(R3)

; registre 10 = volume canal C
; B4=1 bit =M / M=0=>volume fixe / M=1=>volume enveloppe
; B3/B2/B1/B0 = volume fixe pour le canal C
;	Noise	 Tone
;	C B A    C B A
	loadb		(R1),R2						; registre 10
	add			R8,R1	

	moveq		#%1111,R3
	move		R2,R4
	movei		#YM_DSP_volC,R5
	and			R3,R4
	
	shlq		#2,R4					; volume sur 16 *4 
	load		(R14+R4),R4
	
	store		R4,(R5)

	movei		#YM_DSP_pointeur_sur_source_du_volume_C,R3

	btst		#4,R2
	jr			eq,DSP_lecture_registre10_pas_env
	nop

	movei		#YM_DSP_volE,R5
	
DSP_lecture_registre10_pas_env:
	store		R5,(R3)



; registre 11 & 12 = frequence de l'enveloppe sur 16 bits
	loadb		(R1),R2						; registre 11 = 8 bits du bas
	add			R8,R1
	loadb		(R1),R3						; registre 12 = 8 bits du haut

	movei		#YM_frequence_predivise,R5
	add			R8,R1
	shlq		#8,R3
	load		(R5),R5						; R5=YM_frequence_predivise
	add			R2,R3						; R3 = frequence YM canal B

	div			r3,R5
	
	movei		#YM_DSP_increment_enveloppe,R2
	store		R5,(R2)


; registre 13 = envelop shape
	loadb		(R1),R2						; registre 13 = Envelope shape control
	add			R8,R1

; - choix de la bonne enveloppe
	moveq		#%1111,R5
	movei		#$FFF00000,R3						; 16 bits du haut = -16, virgule = 0
	and			R5,R2
	movei		#YM_DSP_offset_enveloppe,R5
	movei		#YM_DSP_pointeur_enveloppe_en_cours,R0
	store		R3,(R5)
	movei		#YM_DSP_liste_des_enveloppes,R4
	shlq		#2,R2								; numero d'env dans registre 13 * 4
	add			R2,R4
	load		(R4),R4
	store		R4,(R0)								; pointe sur enveloppe
	


;---> precalculer les valeurs qui ne bougent pas pendant 1 VBL entiere	




	



DSP_boucle_centrale:


	jr		DSP_boucle_centrale




	nop

	.phrase
; datas DSP
DSP_frequence_de_replay_reelle_I2S:			dc.l			0
DSP_parametre_de_frequence_I2S:				dc.l			0

YM_DSP_increment_canal_A:			dc.l			0
YM_DSP_increment_canal_B:			dc.l			0
YM_DSP_increment_canal_C:			dc.l			0
YM_DSP_increment_Noise:				dc.l			0
YM_DSP_increment_enveloppe:			dc.l			0

YM_DSP_Mixer_TA:					dc.l			0
YM_DSP_Mixer_TB:					dc.l			0
YM_DSP_Mixer_TC:					dc.l			0
YM_DSP_Mixer_NA:					dc.l			0
YM_DSP_Mixer_NB:					dc.l			0
YM_DSP_Mixer_NC:					dc.l			0

YM_DSP_volA:					dc.l			0
YM_DSP_volB:					dc.l			0
YM_DSP_volC:					dc.l			0

YM_DSP_volE:					dc.l			0
YM_DSP_offset_enveloppe:		dc.l			0
YM_DSP_pointeur_enveloppe_en_cours:	dc.l		0

YM_DSP_pointeur_sur_source_du_volume_A:				dc.l		YM_DSP_volA
YM_DSP_pointeur_sur_source_du_volume_B:				dc.l		YM_DSP_volB
YM_DSP_pointeur_sur_source_du_volume_C:				dc.l		YM_DSP_volC

YM_DSP_position_offset_A:		dc.l			0
YM_DSP_position_offset_B:		dc.l			0
YM_DSP_position_offset_C:		dc.l			0

YM_DSP_position_offset_Noise:	dc.l			0
YM_DSP_current_Noise:			dc.l			0
YM_DSP_current_Noise_mask:		dc.l			0

YM_DSP_table_de_volumes:
	dc.l				0,161,265,377,580,774,1155,1575,2260,3088,4570,6233,9330,13187,21220,32767
					; 62,161,265,377,580,774,1155,1575,2260,3088,4570,6233,9330,13187,21220,32767


; - le registre 13 definit la forme de l'enveloppe
; - on initialise une valeur à -16
; partie entiere 16 bits : virgule 16 bits
; partie entiere and %1111 = position dans la sous partie d'enveloppe
; ( ( partie entiere >> 4 ) and %1 ) << 2 = pointeur sur la sous partie d'enveloppe


YM_DSP_forme_enveloppe_1:
; enveloppe montante
	dc.l				0,161,265,377,580,774,1155,1575,2260,3088,4570,6233,9330,13187,21220,32767
YM_DSP_forme_enveloppe_2:
; enveloppe descendante
	dc.l				32767,21220,13187,9330,6233,4570,3088,2260,1575,1155,774,580,377,265,161,0
YM_DSP_forme_enveloppe_3:
; enveloppe zero
	dc.l				0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
YM_DSP_forme_enveloppe_4:
; enveloppe a 1
	dc.l				32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767

;-- formes des enveloppes
; forme enveloppe  0 0 x x
	dc.l		YM_DSP_forme_enveloppe_2	
YM_DSP_enveloppe00xx:
YM_DSP_enveloppe1001:
	dc.l		YM_DSP_forme_enveloppe_3,YM_DSP_forme_enveloppe_3
; forme enveloppe  0 1 x x
	dc.l		YM_DSP_forme_enveloppe_1	
YM_DSP_enveloppe01xx:
	dc.l		YM_DSP_forme_enveloppe_3,YM_DSP_forme_enveloppe_3
; forme enveloppe  1 0 0 0
	dc.l		YM_DSP_forme_enveloppe_2	
YM_DSP_enveloppe1000:
	dc.l		YM_DSP_forme_enveloppe_2,YM_DSP_forme_enveloppe_2
; forme enveloppe  1 0 0 1 = forme enveloppe  0 0 x x
; forme enveloppe  1 0 1 0
	dc.l		YM_DSP_forme_enveloppe_2	
YM_DSP_enveloppe1010:
	dc.l		YM_DSP_forme_enveloppe_1,YM_DSP_forme_enveloppe_2
; forme enveloppe  1 0 1 1
	dc.l		YM_DSP_forme_enveloppe_2
YM_DSP_enveloppe1011:
	dc.l		YM_DSP_forme_enveloppe_4,YM_DSP_forme_enveloppe_4
; forme enveloppe  1 1 0 0
	dc.l		YM_DSP_forme_enveloppe_1
YM_DSP_enveloppe1100:
	dc.l		YM_DSP_forme_enveloppe_1,YM_DSP_forme_enveloppe_1
; forme enveloppe  1 1 0 1
	dc.l		YM_DSP_forme_enveloppe_1
YM_DSP_enveloppe1101:
	dc.l		YM_DSP_forme_enveloppe_4,YM_DSP_forme_enveloppe_4
; forme enveloppe  1 1 1 0
	dc.l		YM_DSP_forme_enveloppe_1
YM_DSP_enveloppe1110:
	dc.l		YM_DSP_forme_enveloppe_2,YM_DSP_forme_enveloppe_1
; forme enveloppe  1 1 1 1
	dc.l		YM_DSP_forme_enveloppe_1
YM_DSP_enveloppe1111:
	dc.l		YM_DSP_forme_enveloppe_3,YM_DSP_forme_enveloppe_3

YM_DSP_liste_des_enveloppes:
	dc.l		YM_DSP_enveloppe00xx, YM_DSP_enveloppe00xx, YM_DSP_enveloppe00xx , YM_DSP_enveloppe00xx
	dc.l		YM_DSP_enveloppe01xx,YM_DSP_enveloppe01xx,YM_DSP_enveloppe01xx,YM_DSP_enveloppe01xx
	dc.l		YM_DSP_enveloppe1000,YM_DSP_enveloppe1001,YM_DSP_enveloppe1010,YM_DSP_enveloppe1011
	dc.l		YM_DSP_enveloppe1100,YM_DSP_enveloppe1101,YM_DSP_enveloppe1110,YM_DSP_enveloppe1111


YM_DSP_fin:

	
	

	.phrase	
SOUND_DRIVER_SIZE			.equ			YM_DSP_fin-DSP_base_memoire
	.print	"--- Sound driver code size (DSP): ", /u SOUND_DRIVER_SIZE, " bytes / 8192 ---"


	
	.68000


	.even
fichier_ym7:
	.incbin		"YM/ancool_atari_baby.ym7"
	.even

debut_ram_libre:			dc.l			FIN_RAM

	.bss
	.phrase
tmpregistre0:			ds.l			1	
tmpregistre1:			ds.l			1	


YM_nombre_de_frames_totales:			ds.l				1
YM_frequence_replay:					ds.w				1
YM_flag_effets_sur_les_voies:			ds.w				1
YM_ecart_entre_frames_blocs:			ds.w				1
YM_ecart_entre_frames_dernier_bloc:		ds.w				1
YM_nombre_de_blocs_lZ4:					ds.w				1
YM_nb_registres_par_frame:				ds.w				1

YM_numero_bloc_en_cours:		ds.w		1
	.phrase
YM_pointeur_actuel_ymdata:		ds.l		1
YM_pointeur_origine_ymdata:		ds.l		1
YM_frequence_predivise:			ds.l		1
	.phrase

PSG_ecart_entre_les_registres_ymdata:		ds.l			1


YM_flag_effets_voie_A:	ds.w		1
YM_flag_effets_voie_B:	ds.w		1
YM_flag_effets_voie_C:	ds.w		1
YM_flag_effets_Sinus_Sid:	ds.w		1

YM_LZ4_pointeur_tailles_des_blocs:		ds.l		1
YM_LZ4_pointeur_destination_bloc_actuel:		ds.l		1

YM_tableau_des_blocs_decompresses:
; pointeur adresse bloc mémoire decompressé, écart entre les registres pour ce bloc
	.rept 10
		ds.l		1
		ds.w		1
	.endr


FIN_RAM:


; algo random DSP
; R0/R1/R2/R3/R4 
;			MOVEI		#DSP_RNG_number, R0		; 00f1c24c (00001b5c)	 R0 = #$00f1b4a0								= U235SE_rng
;			LOAD		(R0), R1			; 00f1c252 (00001b62)	 RD = long at addess in R1
;			MOVEQ		#$01, R4			; 00f1c254 (00001b64)	 R4 = #$01
;			MOVE		R1, R2				; 00f1c256 (00001b66)	 R2 = R1
;			MOVE		R1, R3				; 00f1c258 (00001b68)	 R3 = R1
;			SHRQ		#$02, R3			; 00f1c25a (00001b6a)	 R3 shift right by #$02
;			AND			R4, R2				; 00f1c25c (00001b6c)	 R2 = R4 and R2
;			AND			R4, R3				; 00f1c25e (00001b6e)	 R3 = R4 and R3
;			XOR			R2, R3				; 00f1c260 (00001b70)	 R3 = R2 xor R3
;			MOVE		R1, R2				; 00f1c262 (00001b72)	 R2 = R1
;			MOVE		R3, R4				; 00f1c264 (00001b74)	 R4 = R3
;			SHRQ		#$01, R2			; 00f1c266 (00001b76)	 R2 shift right by #$01
;			SHLQ		#$10, R4			; 00f1c268 (00001b78)	 R16 shift left by #$ebde350b
;			//MOVEI		#$ffff, R6			; 00f1c26a (00001b7a)	 R6 = #$0000ffff
;			OR			R2, R4				; 00f1c270 (00001b80)	 R4 = R2 or R4
;			//AND			R6, R3				; 00f1c272 (00001b82)	 R3 = R6 and R3
;			//MOVEI		#$f1b4a4, R2		; 00f1c274 (00001b84)	 R2 = #$00f1b4a4
;			//BTST		#$00, R3			; 00f1c27a (00001b8a)	 Test bit 0 in R3
;			//JR			EQ, LBL000e			; 00f1c27c (00001b8c)	 Conditional Relative Jump to LBL000e
;			//NOP								; 00f1c27e (00001b8e)	 No OPeration
;			//MOVEI		#$7f, R3			; 00f1c280 (00001b90)	 R3 = #$0000007f
;			//JR			LBL000f				; 00f1c286 (00001b96)	 Conditional Relative Jump to LBL000f
;			//NOP								; 00f1c288 (00001b98)	 No OPeration
;LBL000e:	
;			//MOVEI		#$ff80, R3			; 00f1c28a (00001b9a)	 R3 = #$0000ff80
;LBL000f:	
;			STORE		R4, (R0)			; 00f1c292 (00001ba2)	 addr in R0 <- long in R4 
;			//STORE		R3, (R2)			; 00f1c296 (00001ba6)	 addr in R2 <- long in R3 


; sur amiga:

;                          .data:00c05968 70 01                            moveq #1,%d0
;                           .data:00c0596a 74 01                            moveq #1,%d2
;                           .data:00c0596c 43 f9 00 c1 c6 50                lea table_noise_0x00c1c650,%a1
;boucle:						   
;                           .data:00c05972 32 00                            movew %d0,%d1
;                           .data:00c05974 e6 49                            lsrw #3,%d1
;                           .data:00c05976 b1 41                            eorw %d0,%d1
;                           .data:00c05978 c2 42                            andw %d2,%d1
;                           .data:00c0597a d2 41                            addw %d1,%d1
;                           .data:00c0597c 48 41                            swap %d1
;                           .data:00c0597e b3 80                            eorl %d1,%d0
;                           .data:00c05980 e2 88                            lsrl #1,%d0
;                           .data:00c05982 55 c3                            scs %d3
;                           .data:00c05984 12 c3                            moveb %d3,%a1@+
;                           .data:00c05986 51 cf ff ea                      dbf %d7,0x00c05972
	


; ----------------------------------
;static	const ymint		Env00xx[8]={ 1,0,0,0,0,0,0,0 };
;static	const ymint		Env01xx[8]={ 0,1,0,0,0,0,0,0 };
;static	const ymint		Env1000[8]={ 1,0,1,0,1,0,1,0 };
;static	const ymint		Env1001[8]={ 1,0,0,0,0,0,0,0 };
;static	const ymint		Env1010[8]={ 1,0,0,1,1,0,0,1 };
;static	const ymint		Env1011[8]={ 1,0,1,1,1,1,1,1 };
;static	const ymint		Env1100[8]={ 0,1,0,1,0,1,0,1 };
;static	const ymint		Env1101[8]={ 0,1,1,1,1,1,1,1 };
;static	const ymint		Env1110[8]={ 0,1,1,0,0,1,1,0 };
;static	const ymint		Env1111[8]={ 0,1,0,0,0,0,0,0 };
;static	const ymint *	EnvWave[16] = {	Env00xx,Env00xx,Env00xx,Env00xx,
;										Env01xx,Env01xx,Env01xx,Env01xx,
;										Env1000,Env1001,Env1010,Env1011,
;										Env1100,Env1101,Env1110,Env1111};

;static	ymint ymVolumeTable[16] =
;{	62,161,265,377,580,774,1155,1575,2260,3088,4570,6233,9330,13187,21220,32767};

;- faire en 3 étapes, -1,0,1
; - boucler sur 0,1
; phase de 3 étapes
; 3 * 16 * 10 *4 = 1920 octets

; sinon : 4 phases différentes,  de 16*4 = 64 * 4 = 256 octets



;-----------------------------------