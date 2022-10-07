; YM coso replay for Jaguar
; ne garder que YM de base + digidrums

; remis a zero avant chaque replay ?
; comparer entre ce qui est récupéré et buffer_trace_coso

; COSO : nb de registres utilisés = 14 : 0,1,2,3,4,5,6,7,8,9,10,11,12,13

;
; TODO :
;	OK - debugger les enveloppes : $8A/$0A : bit 7 = ne pas modifier l'env
;	OK - gerer le changement de bloc + bouclage
;	OK - init digidrums
;	OK - gérer digidrums 

;	OK - init SID :  nb sid *  somme des tailles de chaque sid // pas besoin de la repetition. allouer de la ram. créer une table : adresse du sid dans la ram DSP.L, adresse de la fin du SID dans la RAM DSP // convertir le SID en volume Jaguar
;			- $62 SID de 4.L = 1568 octets
;	OK - gérer SID : lecture d'un sample court ( 2 ou 4 octets ) avec test de bouclage permanent

;	OK - init Buzzer
;	OK - gérer Buzzer

;	OK - init Sinus Sid
;	OK - gérer Sinus Sid = 1 voie séparée

;	OK - replay à fréquence variable
;	OK - autoriser I2S pendant le timer 1

;	OK - décompresser LZ4 au fur et à mesure. en utilisant le 68000.
;	OK - décompresser LZ4 au fur et à mesure. en utilisant le DSP.

;	OK - simplifier la lecture des effets : BTST sur 1 seul registre
;	OK - with values decreasing from 8000 to zero. This will avoid a loud click on start up
;	OK - forcer pointeur sur volume pour digidrums dans Timer 1
;	OK - stéréo !!!! : placer 1 voie 100% a gauche, 1 voie 100% a droite, et 1 voie 60% a gauche/40% a droite : utiliser des % pour droite et gauche pour chaque canal A B C D, et multiplier
;	BOF ?- placer la copie des 2 samples en tout debut de routine interruption I2S ( vue la variabilité liée à la génération du Noise )
;	- utiliser l'assembleur pour Object Processor

; timer I2S = generation des samples
; Timer 1 = routine de replay

;CC (Carry Clear) = %00100
;CS (Carry Set)   = %01000
;EQ (Equal)       = %00010
;MI (Minus)       = %11000
;NE (Not Equal)   = %00001
;PL (Plus)        = %10100
;HI (Higher)      = %00101
;T (True)         = %00000



NUMERO_DE_MUSIQUE=5




	include	"jaguar.inc"

; STEREO
STEREO									.equ			0			; 0=mono / 1=stereo
STEREO_shit_bits						.equ			4
; stereo weights : 0 to 16
YM_DSP_Voie_A_pourcentage_Gauche		.equ			14
YM_DSP_Voie_A_pourcentage_Droite		.equ			2
YM_DSP_Voie_B_pourcentage_Gauche		.equ			10
YM_DSP_Voie_B_pourcentage_Droite		.equ			6
YM_DSP_Voie_C_pourcentage_Gauche		.equ			6
YM_DSP_Voie_C_pourcentage_Droite		.equ			10
YM_DSP_Voie_D_pourcentage_Gauche		.equ			2
YM_DSP_Voie_D_pourcentage_Droite		.equ			14


; algo de la routine qui genere les samples
; 3 canaux : increment onde carrée * 3 , increment noise, volume voie * 3 , increment enveloppe

CLEAR_BSS			.equ			1									; 1=efface toute la BSS jusqu'a la fin de la ram centrale
DSP_DEBUG			.equ			0
DSP_DEBUG_T1		.equ			0
DSP_DEBUG_BUZZER	.equ			0									; 0=Buzzer ON / 1=pas de gestion du buzzer
I2S_during_Timer1	.equ			0									; 0= I2S waits while timer 1 / 1=IMASK cleared while Timer 1
YM_avancer			.equ			1									; 0=on avance pas / 1=on avance
YM_position_debut_dans_musique		.equ		0
YM_Samples_SID_en_RAM_DSP			.equ		1						; 0 = samples SID en RAM 68000 / 1 = samples SID en RAM DSP.
DSP_random_Noise_generator_method	.equ		4						; algo to generate noise random number : 1 & 4 (LFSR) OK uniquement // 2 & 3 : KO
VBLCOUNTER_ON_DSP_TIMER1			.equ		0						; 0=vbl counter in VI interrupt CPU / 1=vbl counter in Timer 1

display_infos_during_replay			.equ		1
display_infos_debug					.equ		0

	
DSP_Audio_frequence					.equ			36000				; real hardware needs lower sample frequencies than emulators !
YM_frequence_YM2149					.equ			2000000				; 2 000 000 = Atari ST , 1 000 000 Hz = Amstrad CPC, 1 773 400 Hz = ZX spectrum 
YM_DSP_frequence_MFP				.equ			2457600
YM_DSP_precision_virgule_digidrums	.equ			11
YM_DSP_precision_virgule_SID		.equ			16
YM_DSP_precision_virgule_envbuzzer	.equ			16

;YM_frequence_predivise				.equ			394339				; ((YM_frequence_YM2149/16)*65536)/DSP_Audio_frequence

; 21500 => 21867
; 23000 => 23082
; 25000 => 23082
; 27000 => 27698
; 32000 => 34623
; 35000 => 37771

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

; ----------------------------
; parametres affichage
;ob_liste_originale			equ		(ENDRAM-$4000)							; address of list (shadow)
ob_list_courante			equ		((ENDRAM-$4000)+$2000)				; address of read list
nb_octets_par_ligne			equ		320
nb_lignes					equ		256


DSP_STACK_SIZE	equ	32	; long words
DSP_USP			equ		(D_ENDRAM-(4*DSP_STACK_SIZE))
DSP_ISP			equ		(DSP_USP-(4*DSP_STACK_SIZE))

.opt "~Oall"

.text
.68000
;.noclear
; clear BSS

	.if			CLEAR_BSS=1
	lea			DEBUT_BSS,a0
	lea			FIN_RAM,a1
	moveq		#0,d0
	
boucle_clean_BSS:
	move.b		d0,(a0)+
	cmp.l		a0,a1
	bne.s		boucle_clean_BSS
	.endif
	
	move.w		#$000,JOYSTICK

	.if			0=1
; évite le click du début:
; pas évident
	move.l		#$8000,d0
.slowsoundon:
	move.l		d0,L_I2S
	move.l		d0,L_I2S+4
	move.l		#20,d1
.ok:
	nop
	dbf			d1,.ok
	dbf			d0,.slowsoundon
	.endif

;check ntsc ou pal:

	moveq		#0,d0
	move.w		JOYBUTS ,d0

	move.l		#26593900,frequence_Video_Clock			; PAL
	move.l		#415530,frequence_Video_Clock_divisee

	
	btst		#4,d0
	beq.s		jesuisenpal
jesuisenntsc:
	move.l		#26590906,frequence_Video_Clock			; NTSC
	move.l		#415483,frequence_Video_Clock_divisee
jesuisenpal:



	;lea			buffer_de_debug,a0
	;move.l		a0,pointeur_buffer_de_debug

; remplit de merdouille:
	;lea			fin_buffer_de_debug,a1
	;move.l		#$55,d0
;.remplitdem:
	;move.b		d0,(a0)+
	;cmp.l		a0,a1
	;bne.s		.remplitdem

	


	move.l		#INITSTACK, sp	
	move.w		#%0000011011000111, VMODE			; 320x256
	move.w		#$100,JOYSTICK
    bsr     InitVideo               	; Setup our video registers.


	;bsr		creer_Object_list
	jsr     copy_olist              	; use Blitter to update active list from shadow

	move.l	#ob_list_courante,d0					; set the object list pointer
	swap	d0
	move.l	d0,OLP

	lea		CLUT,a2
	move.l	#255-2,d7
	moveq	#0,d0
	
copie_couleurs:
	move.w	d0,(a2)+
	addq.l	#5,d0
	dbf		d7,copie_couleurs

	lea		CLUT+2,a2
	move.w	#$F00F,(a2)+
	

	move.l	#ob_list_courante,d0					; set the object list pointer
	swap	d0
	move.l	d0,OLP





; ------------------------
; debut DSP
	move.l	#0,D_CTRL

; copie du code DSP dans la RAM DSP

	lea		YM_DSP_debut,A0
	lea		D_RAM,A1
	move.l	#YM_DSP_fin-DSP_base_memoire,d0
	lsr.l	#2,d0
	


	sub.l	#1,D0
boucle_copie_bloc_DSP:
	move.l	(A0)+,(A1)+
	dbf		D0,boucle_copie_bloc_DSP

; init coso

; ------------- numero de musique
	MOVEQ	#NUMERO_DE_MUSIQUE,D0
	lea		fichier_coso_depacked,a0
	bsr		INITMUSIC




; CLS
	moveq	#0,d0
	bsr		print_caractere
	
; $40e0                               
; apres copie on init le YM7
	bsr			YM_init_coso


	move.l  #VBL,LEVEL0     	; Install 68K LEVEL0 handler
	move.w  a_vde,d0                	; Must be ODD
	sub.w   #16,d0
	ori.w   #1,d0
	move.w  d0,VI

	move.w  #%01,INT1                 	; Enable video interrupts 11101


	and.w   #%1111100011111111,sr				; 1111100011111111 => bits 8/9/10 = 0
	and.w   #$f8ff,sr

; init DSP
; $40FC
	; set timers
	move.l		#DSP_Audio_frequence,d0
	move.l		frequence_Video_Clock_divisee,d1
	lsl.l		#8,d1
	divu		d0,d1
	and.l		#$ffff,d1
	add.l		#128,d1			; +0.5 pour arrondir
	lsr.l		#8,d1
	subq.l		#1,d1
	move.l		d1,DSP_parametre_de_frequence_I2S

;calcul inverse
 	addq.l	#1,d1
	add.l	d1,d1		; * 2 
	add.l	d1,d1		; * 2 
	lsl.l	#4,d1		; * 16
	move.l	frequence_Video_Clock,d0
	divu	d1,d0			; 26593900 / ( (16*2*2*(+1))
	and.l		#$ffff,d0
	move.l	d0,DSP_frequence_de_replay_reelle_I2S


; launch DSP

	move.l	#REGPAGE,D_FLAGS
	move.l	#DSP_routine_init_DSP,D_PC
	move.l	#DSPGO,D_CTRL
	move.l	#0,vbl_counter_replay_DSP
	move.l	#0,vbl_counter

; calcul RAM DSP
	move.l		#D_ENDRAM,d0
	sub.l		debut_ram_libre_DSP,d0
	
	move.l		a0,-(sp)
	lea			chaine_RAM_DSP,a0
	bsr			print_string
	move.l		(sp)+,a0
	
	bsr			print_nombre_4_chiffres
; ligne suivante
	moveq		#10,d0
	bsr			print_caractere

	move.b		#85,couleur_char

; replay frequency
	move.l		a0,-(sp)
	lea			chaine_replay_frequency,a0
	bsr			print_string
	move.l		(sp)+,a0

	move.l		DSP_frequence_de_replay_reelle_I2S,d0
	bsr			print_nombre_5_chiffres

	move.l		a0,-(sp)
	lea			chaine_HZ_init_YM7,a0
	bsr			print_string
	move.l		(sp)+,a0

	

	move.b		#145,couleur_char
	
	move.l		a0,-(sp)
	lea			chaine_playing_YM7,a0
	bsr			print_string
	move.l		(sp)+,a0

	move.l		#STEREO,d1
	cmp.l		#1,d1
	beq.s		printstereo

	lea			chaine_playing_YM7_MONO,a0
	bsr			print_string

	bra.s		okprintms
printstereo:
	lea			chaine_playing_YM7_STEREO,a0
	bsr			print_string
okprintms:

	move.b		#245,couleur_char

	
main:
;vsync
;	move.l		vbl_counter,d0
;vsync_toto:
;	move.l		vbl_counter,d1
;	cmp.l		d0,d1
;	beq.s		vsync_toto
	

	move.l		DSP_flag_registres_YM_lus,d0
	cmp.l		#0,d0
	beq.s		main
	move.l		#0,DSP_flag_registres_YM_lus
	
	bsr		PLAYMUSIC
	;bsr			copie_registres_musique

	lea		YM_registres_Coso,a0


	;move.l		PSG_compteur_frames_restantes,d0
	;move.l		PSG_ecart_entre_les_registres_ymdata,d1
	;move.l		YM_pointeur_actuel_ymdata,A0
	

;$40D0


	;move.l		YM_DSP_increment_sample_SID_voie_A,d0
	;cmp.l		#0,d0
	;beq.s		ok_toto
	
; $40B8
	
	;move.l		YM_DSP_pointeur_sample_SID_voie_A,d1 
	;move.l		YM_DSP_offset_sample_SID_voie_A,d2
	;move.l		YM_DSP_taille_sample_SID_voie_A,d3
	;move.l		YM_DSP_registre8,d4
	;move.l		YM_DSP_pointeur_sur_table_infos_samples_SID,d5
	;move.l		YM_DSP_volA,d6
	;move.l		YM_DSP_pointeur_sur_source_du_volume_A,d7

	;lea			buffer_de_debug,a1
	;move.l		pointeur_buffer_de_debug,a2

; $4102
	nop
	
	
ok_toto:

	.if		1=0


; gestion affichage ligne indicateurs
;envA
	move.b		#" ",d0
	move.l		#YM_DSP_volE,d1
	cmp.l		YM_DSP_pointeur_sur_source_du_volume_A,d1
	bne.s		.envA
	move.b		#"E",d0
.envA:
	move.b		d0,chaine_replay_envA
;envB
	move.b		#" ",d0
	move.l		#YM_DSP_volE,d1
	cmp.l		YM_DSP_pointeur_sur_source_du_volume_B,d1
	bne.s		.envB
	move.b		#"E",d0
.envB:
	move.b		d0,chaine_replay_envB
;envC
	move.b		#" ",d0
	move.l		#YM_DSP_volE,d1
	cmp.l		YM_DSP_pointeur_sur_source_du_volume_C,d1
	bne.s		.envC
	move.b		#"E",d0
.envC:
	move.b		d0,chaine_replay_envC
;TA
	move.b		#"T",d0
	cmp.l		#0,	YM_DSP_Mixer_TA
	beq.s		.TA
	move.b		#" ",d0
.TA:
	move.b		d0,chaine_replay_TA
;NA
	move.b		#"N",d0
	cmp.l		#0,	YM_DSP_Mixer_NA
	beq.s		.NA
	move.b		#" ",d0
.NA:
	move.b		d0,chaine_replay_NA
; DG A
	move.b		#" ",d0
	cmp.l		#0,	YM_DSP_pointeur_sample_digidrum_voie_A
	beq.s		.DGA
	move.b		#"D",d0
.DGA:
	move.b		d0,chaine_replay_DG_A

;TB
	move.b		#"T",d0
	cmp.l		#0,	YM_DSP_Mixer_TB
	beq.s		.TB
	move.b		#" ",d0
.TB:
	move.b		d0,chaine_replay_TB
;NB
	move.b		#"N",d0
	cmp.l		#0,	YM_DSP_Mixer_NB
	beq.s		.NB
	move.b		#" ",d0
.NB:
	move.b		d0,chaine_replay_NB
; DG B
	move.b		#" ",d0
	cmp.l		#0,	YM_DSP_pointeur_sample_digidrum_voie_B
	beq.s		.DGB
	move.b		#"D",d0
.DGB:
	move.b		d0,chaine_replay_DG_B

;TC
	move.b		#"T",d0
	cmp.l		#0,	YM_DSP_Mixer_TC
	beq.s		.TC
	move.b		#" ",d0
.TC:
	move.b		d0,chaine_replay_TC
;NC
	move.b		#"N",d0
	cmp.l		#0,	YM_DSP_Mixer_NC
	beq.s		.NC
	move.b		#" ",d0
.NC:
	move.b		d0,chaine_replay_NC
; DG C
	move.b		#" ",d0
	cmp.l		#0,	YM_DSP_pointeur_sample_digidrum_voie_C
	beq.s		.DGC
	move.b		#"D",d0
.DGC:
	move.b		d0,chaine_replay_DG_C




	lea			chaine_replay_YM7,a0
	bsr			print_string
	


; compteur de temps

	.if			VBLCOUNTER_ON_DSP_TIMER1=1
	move.l		vbl_counter_replay_DSP,d0
	.endif
	.if			VBLCOUNTER_ON_DSP_TIMER1=0
	move.l		vbl_counter,d0
	.endif
	move.l		d0,d1
	move.l		_50ou60hertz,d2
	divu		d2,d1
	and.l		#$FFFF,d1			; d1=secondes

	move.l		d1,d0
	divu		#60,d0
	and.l		#$FFFF,d0
	move.l		d0,d2
	bsr			print_nombre_2_chiffres

; ":"
	moveq	#0,d0
	move.b	#":",d0
	bsr		print_caractere

	mulu		#60,d2
	sub.l		d2,d1
	move.l		d1,d0
	bsr			print_nombre_2_chiffres_force

	moveq	#0,d0
	move.b	#" ",d0
	bsr		print_caractere

	
; retour a la ligne	
	moveq	#9,d0
	bsr		print_caractere

	.endif

	;.if		display_infos_debug=1

	moveq	#0,d0
	lea		YM_registres_Coso,a0
	move.b	(a0),d0
	bsr		print_nombre_2_chiffres_force
	
	move.l	#' ',d0
	bsr		print_caractere

	move.l	YM_DSP_volA,d0
	bsr		print_nombre_hexa_4_chiffres

	move.l	#' ',d0
	bsr		print_caractere

	move.l	YM_DSP_volB,d0
	bsr		print_nombre_hexa_4_chiffres

	move.l	#' ',d0
	bsr		print_caractere

	move.l	YM_DSP_volC,d0
	bsr		print_nombre_hexa_4_chiffres


	

; retour a la ligne	
	moveq	#9,d0
	bsr		print_caractere
	;.endif

	
	bra			main

	stop		#$2700

;--------------------------
; VBL

VBL:
                movem.l d0-d7/a0-a6,-(a7)
				
				;.if		display_infos_debug=1
				;add.w		#1,BG					; debug pour voir si vivant
				;.endif

                jsr     copy_olist              	; use Blitter to update active list from shadow


                addq.l	#1,vbl_counter

	
                ;move.w  #$101,INT1              	; Signal we're done
				move.w	#$101,INT1
                move.w  #$0,INT2
.exit:
                movem.l (a7)+,d0-d7/a0-a6
                rte


copie_registres_musique:
		move.l		pointeur_buffer_trace_coso,a0
		lea			YM_registres_Coso,a1
		moveq		#14-1,d0
boucle_copie_registres_musique:
		move.b		(a0)+,(a1)+
		dbf			d0,boucle_copie_registres_musique

		lea			fin_buffer_trace_coso,a1
		cmp.l		a1,a0
		bne.s		ok_pas_fin_fin_buffer_trace_coso
		lea			buffer_trace_coso,a0
ok_pas_fin_fin_buffer_trace_coso:
		move.l		a0,pointeur_buffer_trace_coso
		rts

; ---------------------------------------
; imprime une chaine terminée par un zéro
; a0=pointeur sur chaine
print_string:
	movem.l d0-d7/a0-a6,-(a7)	

print_string_boucle:
	moveq	#0,d0
	move.b	(a0)+,d0
	cmp.w	#0,d0
	bne.s	print_string_pas_fin_de_chaine
	movem.l (a7)+,d0-d7/a0-a6
	rts
print_string_pas_fin_de_chaine:
	bsr		print_caractere
	bra.s	print_string_boucle

; ---------------------------------------
; imprime un nombre HEXA de 2 chiffres
print_nombre_hexa_2_chiffres:
	movem.l d0-d7/a0-a6,-(a7)
	lea		convert_hexa,a0
	move.l		d0,d1
	divu		#16,d0
	and.l		#$F,d0			; limite a 0-15
	move.l		d0,d2
	mulu		#16,d2
	sub.l		d2,d1
	move.b		(a0,d0.w),d0
	bsr			print_caractere
	move.l		d1,d0
	and.l		#$F,d0			; limite a 0-15
	move.b		(a0,d0.w),d0
	bsr			print_caractere
	movem.l (a7)+,d0-d7/a0-a6
	rts
	
convert_hexa:
	dc.b		48,49,50,51,52,53,54,55,56,57
	dc.b		65,66,67,68,69,70
	
; ---------------------------------------
; imprime un nombre de 2 chiffres
print_nombre_2_chiffres:
	movem.l d0-d7/a0-a6,-(a7)
	move.l		d0,d1
	divu		#10,d0
	and.l		#$FF,d0
	move.l		d0,d2
	mulu		#10,d2
	sub.l		d2,d1
	cmp.l		#0,d0
	beq.s		.zap
	add.l		#48,d0
	bsr			print_caractere
.zap:
	move.l		d1,d0
	add.l		#48,d0
	bsr			print_caractere
	movem.l (a7)+,d0-d7/a0-a6
	rts

; ---------------------------------------
; imprime un nombre de 3 chiffres
print_nombre_3_chiffres:
	movem.l d0-d7/a0-a6,-(a7)
	move.l		d0,d1

	divu		#100,d0
	and.l		#$FF,d0
	move.l		d0,d2
	mulu		#100,d2
	sub.l		d2,d1
	cmp.l		#0,d0
	beq.s		.zap
	add.l		#48,d0
	bsr			print_caractere
.zap:
	move.l		d1,d0	
	divu		#10,d0
	and.l		#$FF,d0
	move.l		d0,d2
	mulu		#10,d2
	sub.l		d2,d1
	add.l		#48,d0
	bsr			print_caractere
	
	move.l		d1,d0
	add.l		#48,d0
	bsr			print_caractere
	movem.l (a7)+,d0-d7/a0-a6
	rts


; ---------------------------------------
; imprime un nombre de 2 chiffres , 00
print_nombre_2_chiffres_force:
	movem.l d0-d7/a0-a6,-(a7)
	move.l		d0,d1
	divu		#10,d0
	and.l		#$FF,d0
	move.l		d0,d2
	mulu		#10,d2
	sub.l		d2,d1
	add.l		#48,d0
	bsr			print_caractere
	move.l		d1,d0
	add.l		#48,d0
	bsr			print_caractere
	movem.l (a7)+,d0-d7/a0-a6
	rts

; ---------------------------------------
; imprime un nombre de 4 chiffres HEXA
print_nombre_hexa_4_chiffres:
	movem.l d0-d7/a0-a6,-(a7)
	move.l		d0,d1
	lea		convert_hexa,a0

	divu		#4096,d0
	and.l		#$FF,d0
	move.l		d0,d2
	mulu		#4096,d2
	sub.l		d2,d1
	move.b		(a0,d0.w),d0
	bsr			print_caractere

	move.l		d1,d0
	divu		#256,d0
	and.l		#$FF,d0
	move.l		d0,d2
	mulu		#256,d2
	sub.l		d2,d1
	move.b		(a0,d0.w),d0
	bsr			print_caractere


	move.l		d1,d0
	divu		#16,d0
	and.l		#$FF,d0
	move.l		d0,d2
	mulu		#16,d2
	sub.l		d2,d1
	move.b		(a0,d0.w),d0
	bsr			print_caractere
	move.l		d1,d0
	move.b		(a0,d0.w),d0
	bsr			print_caractere
	movem.l (a7)+,d0-d7/a0-a6
	rts

; ---------------------------------------
; imprime un nombre de 6 chiffres HEXA ( pour les adresses memoire)
print_nombre_hexa_6_chiffres:
	movem.l d0-d7/a0-a6,-(a7)
	
	move.l		d0,d1
	lea		convert_hexa,a0

	swap		d0
	and.l		#$F0,d0
	lsr.l		#4,d0
	and.l		#$F,d0
	and.l		#$FFFFF,d1
	move.b		(a0,d0.w),d0
	bsr			print_caractere

	move.l		d1,d0
	swap		d0
	and.l		#$F,d0
	and.l		#$FFFF,d1
	move.b		(a0,d0.w),d0
	bsr			print_caractere


	move.l		d1,d0
	divu		#4096,d0
	and.l		#$FF,d0
	move.l		d0,d2
	mulu		#4096,d2
	sub.l		d2,d1
	move.b		(a0,d0.w),d0
	bsr			print_caractere

	move.l		d1,d0
	divu		#256,d0
	and.l		#$FF,d0
	move.l		d0,d2
	mulu		#256,d2
	sub.l		d2,d1
	move.b		(a0,d0.w),d0
	bsr			print_caractere


	move.l		d1,d0
	divu		#16,d0
	and.l		#$FF,d0
	move.l		d0,d2
	mulu		#16,d2
	sub.l		d2,d1
	move.b		(a0,d0.w),d0
	bsr			print_caractere
	move.l		d1,d0
	move.b		(a0,d0.w),d0
	bsr			print_caractere
	movem.l (a7)+,d0-d7/a0-a6
	rts


; ---------------------------------------
; imprime un nombre de 4 chiffres
print_nombre_4_chiffres:
	movem.l d0-d7/a0-a6,-(a7)
	move.l		d0,d1

	divu		#1000,d0
	and.l		#$FF,d0
	move.l		d0,d2
	mulu		#1000,d2
	sub.l		d2,d1
	add.l		#48,d0
	bsr			print_caractere

	move.l		d1,d0
	divu		#100,d0
	and.l		#$FF,d0
	move.l		d0,d2
	mulu		#100,d2
	sub.l		d2,d1
	add.l		#48,d0
	bsr			print_caractere


	move.l		d1,d0
	divu		#10,d0
	and.l		#$FF,d0
	move.l		d0,d2
	mulu		#10,d2
	sub.l		d2,d1
	add.l		#48,d0
	bsr			print_caractere
	move.l		d1,d0
	add.l		#48,d0
	bsr			print_caractere
	movem.l (a7)+,d0-d7/a0-a6
	rts

; ---------------------------------------
; imprime un nombre de 5 chiffres
print_nombre_5_chiffres:
	movem.l d0-d7/a0-a6,-(a7)
	move.l		d0,d1

	divu		#10000,d0
	and.l		#$FF,d0
	move.l		d0,d2
	mulu		#10000,d2
	sub.l		d2,d1
	add.l		#48,d0
	bsr			print_caractere

	move.l		d1,d0
	divu		#1000,d0
	and.l		#$FF,d0
	move.l		d0,d2
	mulu		#1000,d2
	sub.l		d2,d1
	add.l		#48,d0
	bsr			print_caractere

	move.l		d1,d0
	divu		#100,d0
	and.l		#$FF,d0
	move.l		d0,d2
	mulu		#100,d2
	sub.l		d2,d1
	add.l		#48,d0
	bsr			print_caractere


	move.l		d1,d0
	divu		#10,d0
	and.l		#$FF,d0
	move.l		d0,d2
	mulu		#10,d2
	sub.l		d2,d1
	add.l		#48,d0
	bsr			print_caractere
	move.l		d1,d0
	add.l		#48,d0
	bsr			print_caractere
	movem.l (a7)+,d0-d7/a0-a6
	rts


; -----------------------------
; copie un caractere a l ecran
; d0.w=caractere

print_caractere:
	movem.l d0-d7/a0-a6,-(a7)



	cmp.b	#00,d0
	bne.s	print_caractere_pas_CLS
	move.l	#ecran1,A1_BASE			; = DEST
	move.l	#$0,A1_PIXEL
	move.l	#PIXEL16|XADDPHR|PITCH1,A1_FLAGS
	move.l	#ecran1+320*100,A2_BASE			; = source
	move.l	#$0,A2_PIXEL
	move.l	#PIXEL16|XADDPHR|PITCH1,A2_FLAGS
	
	move.w	#$00,B_PATD
	

	moveq	#0,d0
	move.w	#nb_octets_par_ligne,d0
	lsr.w	#1,d0
	move.w	#nb_lignes,d1
	mulu	d1,d0
	swap	d0
	move.w	#1,d0
	swap	d0
	;move.w	#65535,d0
	move.l	d0,B_COUNT
	move.l	#LFU_REPLACE|SRCEN|PATDSEL,B_CMD


	movem.l (a7)+,d0-d7/a0-a6
	rts
	
print_caractere_pas_CLS:

	cmp.b	#10,d0
	bne.s	print_caractere_pas_retourchariot
	move.w	#0,curseur_x
	add.w	#8,curseur_y
	movem.l (a7)+,d0-d7/a0-a6
	rts

print_caractere_pas_retourchariot:
	cmp.b	#09,d0
	bne.s	print_caractere_pas_retourdebutligne
	move.w	#0,curseur_x
	movem.l (a7)+,d0-d7/a0-a6
	rts

print_caractere_pas_retourdebutligne:

	lea		ecran1,a1
	moveq	#0,d1
	move.w	curseur_x,d1
	add.l	d1,a1
	moveq	#0,d1
	move.w	curseur_y,d1
	mulu	#nb_octets_par_ligne,d1
	add.l	d1,a1

	lsl.l	#3,d0		; * 8
	lea		fonte,a0
	add.l	d0,a0
	
	
; copie 1 lettre
	move.l	#8-1,d0
copieC_ligne:
	moveq	#8-1,d1
	move.b	(a0)+,d2
copieC_colonne:
	moveq	#0,d4
	btst	d1,d2
	beq.s	pixel_a_zero
	move.b	couleur_char,d4
pixel_a_zero:
	move.b	d4,(a1)+
	dbf		d1,copieC_colonne
	lea		nb_octets_par_ligne-8(a1),a1
	dbf		d0,copieC_ligne

	move.w	curseur_x,d0
	add.w	#8,d0
	cmp.w	#320,d0
	blt		curseur_pas_fin_de_ligne
	moveq	#0,d0
	add.w	#8,curseur_y
curseur_pas_fin_de_ligne:
	move.w	d0,curseur_x

	movem.l (a7)+,d0-d7/a0-a6

	rts


;----------------------------------
; recopie l'object list dans la courante

copy_olist:
				move.l	#ob_list_courante,A1_BASE			; = DEST
				move.l	#$0,A1_PIXEL
				move.l	#PIXEL16|XADDPHR|PITCH1,A1_FLAGS
				move.l	#ob_liste_originale,A2_BASE			; = source
				move.l	#$0,A2_PIXEL
				move.l	#PIXEL16|XADDPHR|PITCH1,A2_FLAGS
				move.w	#1,d0
				swap	d0
				move.l	#fin_ob_liste_originale-ob_liste_originale,d1
				move.w	d1,d0
				move.l	d0,B_COUNT
				move.l	#LFU_REPLACE|SRCEN,B_CMD
				rts


		if		1=0
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;
;; Simple Object List Routines for a simple life.
;;
;; call with *d0=object type
;;           *d1=height
;;           *d2=data width
;;           *d3=colour depth
;;           *d4=transparent
;;           *d5=image width
;;           *a0=address of object (phrase alligned)
;;           *a1=address for GFX   (phrase alligned)
;;
;; exit with object built
;;           link=object address+32
;;           scaled objects will *NOT* be 1:1 in x/y
;;           x/y positions will be -500,2 (off screen)
;;
bitmapobject    equ 0            ; object types
scaledobject    equ 1
gpuobject       equ 2
braobject       equ 3             
stopobject      equ 4

y_less          equ 0            ; branch object types
y_more          equ 1
always          equ 2

CreateObject:   lsl.w   #2,d0                   ; object type
                move.l  jmp_tab(pc,d0),a6       ; get routine address
                jmp (a6)                        ; call it!

jmp_tab:        .dc.l bitmp                     ; jump table
                .dc.l scaled
                .dc.l gpuob
                .dc.l braob
                .dc.l stopob

bitmp:          clr.l   (a0)                    ; template
                clr.l   4(a0)                   ;
                clr.l   8(a0)                   ;
                move.l  #$00008000,12(a0)       ;

                move.l  a0,d0                   ; link address
	sub.l	#ob_liste_originale,d0
	add.l	#ob_list_courante,d0

                ;sub.l   #ob_list1,d0            ;
                ;add.l   #ob_list,d0             ;
                add.l   #32,d0                  ;
                and.b   #%11111000,d0           ;
                lsl.l   #5,d0                   ;
                or.l    d0,2(a0)                ;

                move.l  a1,d0                   ; gfx address
                lsl.l   #8,d0                   ;
				

					
                or.l    d0,(a0)                 ;
				
				
				
                move.l  d1,d0                   ; height
                swap    d0                      ;
                lsr.l   #2,d0                   ;
                or.l    d0,4(a0)                ;
				
				
                ror.w   #1,d4                   ; transparency
                or.w    d4,10(a0)               ;
                lsl.w   #8,d3                   ; depth (colour depth)
                lsl.w   #4,d3                   ;
                or.w    d3,14(a0)               ;
                lsr.w   #3,d2                   ; data width
                swap    d2                      ;
                lsl.l   #2,d2                   ;
                or.l    d2,12(a0)               ;
                lsr.w   #3,d5                   ; image width
                swap    d5                      ;
                lsr.l   #4,d5                   ;
                or.l    d5,10(a0)               ;

				
                move.w  #-500,d0                ; x-pos
				
				move.w #16,d0
				
                and.w   #$fff,d0
                or.w    d0,14(a0)

                or.w    #2*8,6(a0)              ; y-pos
				
;move.w	#-32,d0	; y_pos
;and.w	#$ffff,d0
;or.w	d0,6(a0)
				
				
                lea     32(a0),a0               ; BITMAP object!
                rts

scaled:         bsr     bitmp                   ; same as bitmap
                move.l  #$0,-16(a0)             ; clear it out
                move.l  #%00000000000000000010000000100000,-12(a0)
                or.l    #$1,-28(a0)             ; SCALED object!
                rts

gpuob:          move.l  #0,(a0)+
                move.l  #$3ffa,(a0)+            ; GPU object!
                rts

				
; D1=branch type
; D2=Ypos
; A1=address if branch TAKEN
				
braob:          add     d2,d2                   ; mult y-pos
                bsr     branchobject            ; make the object
make8into32:    move.l  #braobject,d0           ;
                move.l  #always,d1              ; even it out for 32-byte
                move.l  #$7ff,d2                ; positions by following with
                lea     24(a0),a1               ; a BRA+16 object
                bsr     branchobject            ;
                lea     16(a0),a0               ;
                rts

branchobject:   clr.l   (a0)
                move.l  #3,4(a0)
                add.w   d1,d1
                move.w  branchtypes(pc,d1.w),d1
                or.w    d1,6(a0)                ; branch TYPE!
                move.l  a1,d0
	sub.l	#ob_liste_originale,d0
	add.l	#ob_list_courante,d0
                ;sub.l   #ob_list1,d0
                ;add.l   #ob_list,d0
                and.l   #$fffffff8,d0
                lsl.l   #5,d0                   ; Link if branch **TAKEN!**
                move.l  d0,2(a0)                ; (< & > swapped!)
                lsl.w   #3,d2                   ; scanline to branch on
                or.w    d2,6(a0)                ; is VC/2
                lea     8(a0),a0                ; next object
                rts

branchtypes:    .dc.w    $4000,$8000,$0000       ; 

stopob:         move.l  #0,(a0)+
                move.l  #4,(a0)+                ; STOP object!
                rts

		.endif
		
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;
;; Procedure: InitVideo (same as in vidinit.s)
;;            Build values for hdb, hde, vdb, and vde and store them.
;;

InitVideo:
                movem.l d0-d6,-(sp)

				
				move.w	#-1,ntsc_flag
				move.l	#50,_50ou60hertz
	
				move.w  CONFIG,d0                ; Also is joystick register
                andi.w  #VIDTYPE,d0              ; 0 = PAL, 1 = NTSC
                beq     .palvals
				move.w	#1,ntsc_flag
				move.l	#60,_50ou60hertz
	

.ntscvals:		move.w  #NTSC_HMID,d2
                move.w  #NTSC_WIDTH,d0

                move.w  #NTSC_VMID,d6
                move.w  #NTSC_HEIGHT,d4
				
                bra     calc_vals
.palvals:
				move.w #PAL_HMID,d2
				move.w #PAL_WIDTH,d0

				move.w #PAL_VMID,d6				
				move.w #PAL_HEIGHT,d4

				
calc_vals:		
                move.w  d0,width
                move.w  d4,height
                move.w  d0,d1
                asr     #1,d1                   ; Width/2
                sub.w   d1,d2                   ; Mid - Width/2
                add.w   #4,d2                   ; (Mid - Width/2)+4
                sub.w   #1,d1                   ; Width/2 - 1
                ori.w   #$400,d1                ; (Width/2 - 1)|$400
                move.w  d1,a_hde
                move.w  d1,HDE
                move.w  d2,a_hdb
                move.w  d2,HDB1
                move.w  d2,HDB2
                move.w  d6,d5
                sub.w   d4,d5
                add.w   #16,d5
                move.w  d5,a_vdb
                add.w   d4,d6
                move.w  d6,a_vde
			
			    move.w  a_vdb,VDB
				move.w  a_vde,VDE    
				
				
				move.l  #0,BORD1                ; Black border
                move.w  #0,BG                   ; Init line buffer to black
                movem.l (sp)+,d0-d6
                rts




	if		1=0
; -------------------------------------------
creer_Object_list:
; il faut créer une liste avec :
;	- un bra si y>0, sinon stop
; 	- un bra si y< max Y, sinon stop
; 	- un object bitmap
;	- un stop

; on stop tout
	move.l	#stoplist,d0
	swap.w	d0
	move.l	d0,OLP

; la creer dans ob_list_courante
; puis la copier dans ob_liste_originale


	lea		ob_liste_originale,a0

; bra pour debut ecran, y< 0
	
	move.l  a0,d0               ; address if bra not taken
	sub.l	#ob_liste_originale,d0
	add.l	#ob_list_courante,d0
    add.l   #32,d0              ; next BRA object
	lsr.l   #3,d0
	lsl.l   #8,d0
	clr.l   (a0)
	move.l  #$00008003,4(a0)    ; bra if yp<0
	or.l    d0,2(a0)            ; link to next BRA
	lea     8(a0),a0

; stop OP
	move.l  #0,(a0)+
	move.l  #4,(a0)+            ; stop object processor !
	lea     16(a0),a0			; aligner sur 32

; bra pour fin ecran Y> X
	move.l  a0,d0               ; address if bra not taken
	sub.l	#ob_liste_originale,d0
	add.l	#ob_list_courante,d0
	add.l   #32,d0
	lsr.l   #3,d0
	lsl.l   #8,d0
	clr.l   (a0)
	move.l  #$00004003,4(a0)    ; bra is yp>value
	or.l    d0,2(a0)            ; link to first object!

; test NTSC ou PAL pour ligne de fin ( en demi lignes)
	move.w	CONFIG,d6
	andi.w	#VIDTYPE,d6
	beq		.pal
	move.l	#492,d0				; NTSC = 246 lignes
	move.w	#1,ntsc_flag
	bra.s	.ntsc

.pal:
	move.l	#560,d0				; 280 lignes
	move.w	#-1,ntsc_flag

.ntsc:
	lsl.l   #3,d0
	or.l    d0,4(a0)			; la valeur Y max

	lea     8(a0),a0            ; next object
; stop OP
	move.l  #0,(a0)+
	move.l  #4,(a0)+            ; stop object processor !
	lea     16(a0),a0			; aligner sur 32



	lea		ecran1,a1
	move.l  #bitmapobject,d0        			; type
	move.l  #256,d1								; 74,d1 ; height
	move.l  #nb_octets_par_ligne,d2      						; bytes to next line
	moveq	#3,d3				; 3=8bpp, 4=16bit
	moveq	#0,d4                   			; Transparent
	;move.l  #640,d5          					; bytes/pixels*width / line;
	move.l	d2,d5
	bsr     CreateObject

	
	moveq.l #stopobject,d0          			; STOP Object
	bsr     CreateObject	
	
	

	move.l	a0,d0
	move.l	#ob_liste_originale,d1
	sub.l	d1,d0				; d0=taille de l'object list
	move.l	d0,taille_liste_OP

	rts
	endif

;----------------------------------------------------
;     routines YM
;----------------------------------------------------

;----------------------------------------------------
	
	
;----------------------------------------------------
YM_init_coso:
; tout le long de l'init D6=YM_nb_registres_par_frame

	move.b		#225,couleur_char

	lea			chaine_debut_init_YM7,a0
	bsr			print_string

	move.b		#5,couleur_char

	moveq		#50,d0
	move.l		d0,YM_frequence_replay							; .w=frequence du replay ( 50 hz )


	move.l		d0,-(sp)
	bsr		print_nombre_3_chiffres
	move.l		(sp)+,d0

	move.l		a0,-(sp)
	lea			chaine_HZ_init_YM7,a0
	bsr			print_string
	move.l		(sp)+,a0

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
; arrondit multiple de 2
    btst		#0,d0
	beq.s		YM_malloc_pas_d_arrondi
	addq.l		#1,d0
YM_malloc_pas_d_arrondi:
	add.l		d0,d1
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

; ---------------------------------------
; allocation de mémoire version RAM DSP
; malloc de d0, retour avec  un pointeur dans d0
; d0 => forcément un multiple de 4
; ---------------------------------------

YM_malloc_DSP:

	movem.l		d1-d3/a0,-(sp)

	move.l		debut_ram_libre_DSP,d1
	move.l		d1,a0
	move.l		d1,d3
	add.l		d0,d1
	move.l		d1,debut_ram_libre_DSP
	
	move.l		d0,d2
	moveq.l		#0,d0
	lsr.l		#2,d2		; 4 octets par 4 octets
	subq.l		#1,d2

YM_malloc_boucle_clean_ram_DSP:
	move.l		d0,(a0)+
	dbf			d2,YM_malloc_boucle_clean_ram_DSP
	
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
; 			a1.l = pointe sur la fin du bloc decompresse
;

lz4_depack_normal:
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



;---------------------------------------------------------
;
;	LZ4 block 68k small depacker
;	Written by Arnaud Carré ( @leonard_coder )
;	https://github.com/arnaud-carre/lz4-68k
;
;	LZ4 technology by Yann Collet ( https://lz4.github.io/lz4/ )
;
;---------------------------------------------------------

; Smallest version: depacker is only 72 bytes
;
; input: a0.l : packed buffer
;		 a1.l : output buffer
;		 d0.l : LZ4 packed block size (in bytes)
;
; output: none
;

lz4_depack_smallest:
			lea		0(a0,d0.l),a4	; packed buffer end
			moveq	#0,d0
			moveq	#0,d2
			moveq	#15,d4

.tokenLoop_smallest:	
			move.b	(a0)+,d0
			move.l	d0,d1
			lsr.b	#4,d1
			beq.s	.lenOffset_smallest

.readLen_smallest1:	
			cmp.b	d1,d4
			bne.s	.readEnd_smallest1
.readLoop_smallest1:	
			move.b	(a0)+,d2
			add.l	d2,d1				; final len could be > 64KiB
			not.b	d2
			beq.s	.readLoop_smallest1
		
.readEnd_smallest1:	

.litcopy_smallest:
			move.b	(a0)+,(a1)+
			subq.l	#1,d1			; block could be > 64KiB
			bne.s	.litcopy_smallest

			; end test is always done just after literals
			cmpa.l	a0,a4
			beq.s	.readEnd_smallest
			
.lenOffset_smallest:	
			move.b	(a0)+,d2	; read 16bits offset, little endian, unaligned
			move.b	(a0)+,-(a7)
			move.w	(a7)+,d1
			move.b	d2,d1
			movea.l	a1,a3
			sub.l	d1,a3		; d1 bits 31..16 are always 0 here
			moveq	#$f,d1
			and.w	d0,d1

.readLen_smallest2:	
			cmp.b	d1,d4
			bne.s	.readEnd_smallest2
.readLoop_smallest2:	
			move.b	(a0)+,d2
			add.l	d2,d1				; final len could be > 64KiB
			not.b	d2
			beq.s	.readLoop_smallest2
		
.readEnd_smallest2:

			addq.l	#4,d1
.copy_smallest:
			move.b	(a3)+,(a1)+
			subq.l	#1,d1
			bne.s	.copy_smallest
			bra.s	.tokenLoop_smallest

.readLen_smallest:	
			cmp.b	d1,d4
			bne.s	.readEnd_smallest
.readLoop_smallest:	
			move.b	(a0)+,d2
			add.l	d2,d1				; final len could be > 64KiB
			not.b	d2
			beq.s	.readLoop_smallest
		
.readEnd_smallest:	
			rts




;-------------------------------------
;
;     COSO
;
;-------------------------------------



TIMER=0		;0=TIMER A,1=TIMER B,2=TIMER C,3=TIMER D
EQUALISEUR=1	;0=EQUALISEUR

TYPE=1			;1=MUSIQUE NORMALE,2=MUSIQUE DIGIT
PRG=0			;0=PRG,1=REPLAY BINAIRE
MONOCHROM=1		;0=REPLAY MONOCHROME,1=REPLAY COULEUR
PCRELATIF=1		;0=DIGIT PRES DU REPLAY,1=DIGIT LOIN DU REPLAY
AEI=0			;0=REPLAY MODE AEI,1=MODE SEI

CUTMUS=0		;0=INCLUT FIN MUSIQUE,1=ON NE PEUT COUPER LA MUSIQUE
DIGIT=1			;0=INCLUT REPLAY DIGIT,1=SANS
MMME=1			;0=INCLUT REPLAY MMME,1=SANS

TURRICAN=0		;0=REPLAY TURRICAN
OLD=1			;0=ANCIENNE VERSION,1=NOUVELLE



off22	equ		0					; rs.l	1	;ptr courant dans pattern								4
off0	equ		4					; rs.l	1	;ptr base patterns										4
off34	equ		8					; rs.w	1	;ptr fin musique										2

off4	equ		10					; rs.w	1	;ptr patterns (.W au lieu de .L)						2
offa	equ		12					; rs.l	1	;ptr base modulation volume								4
offe	equ		16					; rs.w	1	;ptr modulation volume (.W au lieu de .L)				2
off12	equ		18					; rs.l	1	;ptr base modulation fr‚quence							4
off30	equ		22					; rs.w	1	;ptr modulation fr‚quence (.W au lieu de .L)			2

off38	equ		24					; rs.l	1	;incr‚ment pour crescendo					4

off8	equ		28					; rs.b	1	;											1
off9	equ		29					; rs.b	1	;											1

off16	equ		30					; rs.b	1	;											1
off17	equ		31					; rs.b	1	;											1
off18	equ		32					; rs.b	1	;											1
off19	equ		33					; rs.b	1	;											1
off1a	equ		34					; rs.b	1	;											1
off1b	equ		35					; rs.b	1	;											1
off1c	equ		36					; rs.b	1	;											1
off1d	equ		37					; rs.b	1	;											1
off1e	equ		38					; rs.b	1	;											1
off1f	equ		39					; rs.b	1	;											1
off21	equ		40					; rs.b	1	;											1

off26	equ		41					; rs.b	1	;											1
off27	equ		42					; rs.b	1	;											1
off28	equ		43					; rs.b	1	;15-volume sonore de la voix				1
off2a	equ		44					; rs.b	1	;0,1 ou 2=type de son						1
off2b	equ		45					; rs.b	1	;											1
off2c	equ		46					; rs.b	1	;											1
off2d	equ		47					; rs.b	1	;volume sonore calculé						1
off2e	equ		48					; rs.b	1	;											1
;off3c	equ		47
off3c	equ		50

coso_envoi_registres:
	MOVEM.L			A0-A1,-(A7)
	LEA.L			PSGREG+2,A0											; = c177be
	lea		 		YM_registres_Coso,A1
	MOVE.B			(A0),(A1)+					; 0
	MOVE.B			4(A0),(A1)+					; 1
	MOVE.B			8(A0),(A1)+					; 2 
	MOVE.B			12(A0),(A1)+				; 3
	MOVE.B			16(A0),(A1)+				; 4
	MOVE.B			20(A0),(A1)+				; 5
	MOVE.B			24(A0),(A1)+				; 6
	MOVE.B			28(A0),(A1)+				; 7
	MOVE.B			32(A0),(A1)+				; 8
	MOVE.B			36(A0),(A1)+				; 9
	MOVE.B			40(A0),(A1)+				; A
	MOVEM.L 		(A7)+,A0-A1
	RTS


PLAYMUSIC:
	LEA	PSGREG(PC),A6
	TST.B	BLOQUEMUS-PSGREG(A6)
	BNE.S	L25A

	move.b	#$C0,$1E(A6)		;pour que ‡a tienne...

	SUBQ.B	#1,L80E-PSGREG(A6)
	BNE.S	L180
	MOVE.B	L810-PSGREG(A6),L80E-PSGREG(A6)
	MOVEQ	#0,D5
	LEA	voice0(PC),A0
	BSR.W	L25C
	LEA	voice1(PC),A0
	BSR.W	L25C
	LEA	voice2(PC),A0
	BSR.W	L25C
L180:
	LEA	voice0(PC),A0
	BSR	L39A
	move	d0,6(A6)
	MOVE.B	D0,2(A6)
	MOVE.B	D1,$22(A6)
	LEA	voice1(PC),A0
	BSR	L39A
	move	d0,$E(A6)
	MOVE.B	D0,$A(A6)
	MOVE.B	D1,$26(A6)
	LEA	voice2(PC),A0
	BSR	L39A
	move	D0,$16(A6)
	MOVE.B	D0,$12(A6)
	MOVE.B	D1,$2A(A6)

	;MOVEM.L	(A6),D0-D7/A0-A2
	;MOVEM.L	D0-D7/A0-A2,$FFFF8800.W
	bsr			coso_envoi_registres
L25A:	RTS

;
; calcule nouvelle note
;
L25C:	SUBQ.B	#1,off26(A0)
	BPL.S	L25A
	MOVE.B	off27(A0),off26(A0)
	MOVE.L	off22(A0),A1
L26C:	MOVE.B	(A1)+,D0
	CMP.B	#$FD,D0
	BLO.W	L308
	EXT	D0
	ADD	D0,D0
	JMP		COSO_CODEFD+(3*2)(PC,D0.W)
COSO_CODEFD:
	BRA.S	L2F4		;$FD
	BRA.S	L2E2		;$FE
				;$FF

; NOUVELLE VERSION
	move	off4(a0),d1
	cmp	off34(a0),d1
	blS.S	L288
	tst.b	off21(a0)		;nouveau replay !!!!
	bne.s	L288			;pour bien boucler !!!!
	clr	d1
	move	d5,off4+off3c(a0)
	move	d5,off4+(off3c*2)(a0)
L288:
	MOVE.L	off0(a0),a1
	add	d1,a1
	add	#$C,d1

	move	d1,off4(a0)

	MOVEQ	#0,D1
	move.b	(a1)+,D1
	move.b	(a1)+,off2c(A0)
	move.b	(a1)+,off16(A0)
	moveq	#$10,d0
	add.b	(a1)+,D0
	bcc.s	L2B4
	move.b	d0,off28(A0)		;F0-FF=volume … soustraire
	BRA.S	L2C4
L2B4:	add.b	#$10,d0
	bcc.S	L2C4
	move.B	d0,L810-PSGREG(A6)	;E0-EF=vitesse
L2C4:	ADD	D1,D1
	MOVE.L	L934(PC),A1
	ADD	$C+2(A1),D1
	ADD	(A1,D1.W),A1

	MOVE.L	A1,off22(A0)
	BRA.s	L26C

L2E2:
	MOVE.B	(A1)+,d0
	move.b	d0,off27(A0)
	MOVE.B	d0,off26(A0)
	BRA.s	L26C
L2F4:
	MOVE.B	(A1)+,d0
	move.b	d0,off27(A0)
	MOVE.B	d0,off26(A0)
	MOVE.L	A1,off22(A0)
	RTS

L308:	MOVE.B	D0,off8(a0)
	MOVE.B	(A1)+,D1
	MOVE.B	D1,off9(a0)
	AND	#$E0,D1			;d1=off9&$E0
	BEQ.S	.L31C
	MOVE.B	(A1)+,off1f(A0)
.L31C:	MOVE.L	A1,off22(A0)
	MOVE.L	D5,off38(A0)
	TST.B	D0
	BMI	L398
	MOVE.B	off9(a0),D0
	eor.b	d0,d1			;d1=off9&$1F
	ADD.B	off16(A0),D1

	MOVE.L	L934(PC),A1

	CMP	$26(A1),D1
	BLS.S	NOBUG2
;	CLR	D1
	move	$26(a1),d1
	move	#$700,$ffff8240.w
NOBUG2:
	ADD	D1,D1
	ADD	8+2(A1),D1
	ADD	(A1,D1.W),A1

	move	d5,offe(A0)
	MOVE.B	(a1)+,d1
	move.b	d1,off17(A0)
	MOVE.B	d1,off18(A0)
	MOVEQ	#0,D1
	MOVE.B	(a1)+,D1
	MOVE.B	(a1)+,off1b(A0)
;	MOVE.B	#$40,off2e(A0)
	clr.b	off2e(a0)
	MOVE.B	(a1)+,D2
	MOVE.B	D2,off1c(A0)
	MOVE.B	D2,off1d(A0)
	MOVE.B	(a1)+,off1e(A0)
	MOVE.L	a1,offa(A0)
	add.b	d0,d0			;test bit 6
	bpl.s	L37A
	MOVE.B	off1f(A0),D1
L37A:
	MOVE.L	L934(PC),A1
	CMP	$24(A1),D1
	BLS.S	NOBUG3
	move	$24(a1),d1
	move	#$070,$ffff8240.w
;	CLR	D1
NOBUG3:
	ADD	D1,D1

	ADD	4+2(A1),D1
	ADD	(A1,D1.W),A1

	MOVE.L	a1,off12(A0)
	move	d5,off30(A0)
	MOVE.B	D5,off1a(A0)
	MOVE.B	D5,off19(A0)
L398:	RTS

;
; calcul de la note … jouer
;
L39A:	MOVEQ	#0,D7
	MOVE	off30(a0),d6
L3A0:	TST.B	off1a(A0)
	BEQ.S	L3AE
	SUBQ.B	#1,off1a(A0)
	BRA	L4C01
L3AE:	MOVE.L	off12(A0),A1
	add	d6,a1
L3B6:	move.b	(a1)+,d0
	CMP.B	#$E0,D0
	BLO	L4B0
;	CMP.B	#$EA,D0		;inutile ???
;	BHS	L4B0

	EXT	D0
	ADD	#32,D0
	MOVE.B	COSO_CODES(PC,D0.W),D0
	JMP		BRANCH_COSO(PC,D0.W)

COSO_CODES:
	DC.B	E0-BRANCH_COSO
	DC.B	E1-BRANCH_COSO
	DC.B	E2-BRANCH_COSO
	DC.B	E3-BRANCH_COSO
	DC.B	E4-BRANCH_COSO
	DC.B	E5-BRANCH_COSO
	DC.B	E6-BRANCH_COSO
	DC.B	E7-BRANCH_COSO
	DC.B	E8-BRANCH_COSO
	DC.B	E9-BRANCH_COSO
	DC.B	EA-BRANCH_COSO
	EVEN
BRANCH_COSO:

BUG:	DCB.L	2,$4A780001
;	DCB.L	$100-$EA,$4A780001

E1:	BRA	L4C01
E0:
	moveq	#$3f,d6		;$E0
;clr d6 … pr‚sent !!!!
	and.B	(A1),D6
	BRA.S	L3AE
E2:
	clr	offe(a0)
	MOVE.B	#1,off17(A0)
	addq	#1,d6
	bra.s	L3B6

E9:
	;MOVE.B	#$B,$FFFF8800.W
	;move.b	(A1)+,$FFFF8802.W
	;move.l	#$0C0C0000,$FFFF8800.W
	;move.l	#$0D0D0A0A,$FFFF8800.W
	
	PEA			(A0)										; 00C0364E 4850                     PEA.L (A0)
	lea		 	YM_registres_Coso,A0			; 00C03650 207a 18fa                MOVEA.L (PC,$18fa) == $00c04f4c [00c0663e],A0
	MOVE.B 		(A1)+,$0B(A0)						; B=11				; 00C03654 1159 000b                MOVE.B (A1)+ [fd],(A0,$000b) == $00c051c9 [30]
	MOVE.B 		#$00,$0C(A0)					; C=12			; 00C03658 117c 0000 000c           MOVE.B #$00,(A0,$000c) == $00c051ca [3c]
	MOVE.B 		#$0a,$0D(A0)					; D=13			; 00C0365E 117c 000a 000d           MOVE.B #$0a,(A0,$000d) == $00c051cb [ac]
	MOVE.L 		(A7)+,A0									; 00C03664 205f                     MOVEA.L (A7)+ [00c0013e],A0
	
	addq	#2,d6
	bra.S	L3B6
E7:
	moveq	#0,d0
	move.b	(A1),D0
	ADD	D0,D0

	MOVE.L	L934(PC),A1
	ADD	4+2(A1),D0
	ADD	(A1,D0.W),A1

	MOVE.L	A1,off12(A0)
	clr	d6
	BRA	L3B6
EA:	move.b	#$20,off9(a0)
	move.b	(a1)+,off1f(a0)
	addq	#2,d6
	bra	L3B6
E8:	move.b	(A1)+,off1a(A0)
	addq	#2,d6
	BRA	L3A0

E4:	clr.b	off2a(A0)
	MOVE.B	(A1)+,d7
	addq	#2,d6
	BRA	L3B6		;4AE
E5:	MOVE.B	#1,off2a(A0)
	addq	#1,d6
	BRA	L3B6
E6:	MOVE.B	#2,off2a(A0)
	addq	#1,d6
	BRA	L3B6		;4AE

E3:	addq	#3,d6
	move.b	(A1)+,off1b(A0)
	move.b	(A1)+,off1c(A0)
	bra	L3B6		;nouveau

;L4AE:	move.b	(a1)+,d0
L4B0:
	MOVE.B	d0,off2b(A0)
	addq	#1,d6
L4C01:	move	d6,off30(a0)
;
; modulation volume
;
	move	offe(a0),d6
L4C0:	TST.B	off19(A0)
	BEQ.S	L4CC
	SUBQ.B	#1,off19(A0)
	BRA.S	L51A
L4CC:	SUBQ.B	#1,off17(A0)
	BNE.S	L51A
	MOVE.B	off18(A0),off17(A0)

	MOVE.L	offa(A0),A1
	add	d6,a1
	move.b	(A1)+,D0
	CMP.B	#$E0,D0
	BNE.S	L512
	moveq	#$3f,d6
; clr d6 … pr‚sent
	and.b	(A1),D6
	subq	#5,D6
	move.l	offa(a0),a1
	add	d6,a1
	move.b	(a1)+,d0
L512:
	CMP.B	#$E8,D0
	BNE.S	L4F4
	addq	#2,d6
	move.b	(A1)+,off19(A0)
	BRA.S	L4C0
L4F4:	CMP.B	#$E1,D0
	BEQ.S	L51A
	MOVE.B	d0,off2d(A0)
	addq	#1,d6
L51A:	move	d6,offe(a0)

	clr	d5
	MOVE.B	off2b(A0),D5
	BMI.S	L528
	ADD.B	off8(a0),D5
	ADD.B	off2c(A0),D5
L528:
	add.b	D5,D5
;	LEA	L94E(PC),A1
;	MOVE	(A1,d5.w),D0
	MOVE	L94E-PSGREG(A6,D5.W),D0

	move.b	off2a(A0),D1	;0,1 ou 2
	beq.S	L57E

	MOVE.B	off21(A0),D2
	ADDQ	#3,D2

	subq.b	#1,D1
	BNE.S	L578
	subq	#3,d2
	MOVE.B	off2b(A0),D7
	bclr	#7,d7
	bne.s	L578		;BMI impossible !!!
	add.b	off8(a0),d7
L578:

	BSET	D2,$1E(A6)
L57E:
	tst.b	d7
	BEQ.S	L594
	not.b	d7
	and.b	#$1F,D7
	MOVE.B	D7,$1A(A6)
L594:

	TST.B	off1e(A0)
	BEQ.S	L5A4
	SUBQ.B	#1,off1e(A0)
	BRA.S	L5FA
L5A4:
	clr	d2
	MOVE.B	off1c(A0),D2

;	bclr	#7,d2		;nouveau replay
;	beq.s	.ok		;BUG ????
;	add.b	d2,d2
;.ok

	clr	d1
	MOVE.B	off1d(A0),D1
	tst.b	off2e(a0)
	bmi.S	L5CE
	SUB.B	off1b(A0),D1
	BCC.S	L5DC
	tas	off2e(a0)	;ou bchg
	MOVEQ	#0,D1
	BRA.S	L5DC
L5CE:	ADD.B	off1b(A0),D1
	ADD.B	d2,d2
	CMP.B	d2,D1
	BCS.S	L5DA
	and.b	#$7f,off2e(a0)	;ou bchg
	MOVE.B	d2,D1
L5DA:	lsr.b	#1,d2
L5DC:	MOVE.B	D1,off1d(A0)
L5E0:
	sub	d2,D1

	ADD.B	#$A0,D5
	BCS.S	L5F8
	moveq	#$18,d2

	add	d1,d1
	add.b	d2,d5
	bcs.s	L5F8
	add	d1,d1
	add.b	d2,d5
	bcs.s	L5F8
	add	d1,d1
	add.b	d2,d5
	bcs.s	L5F8
	add	d1,d1
L5F8:	ADD	D1,D0
;;	EOR.B	#1,d6		;inutilis‚ !!!
;	MOVE.B	d6,off2e(A0)
L5FA:
	BTST	#5,off9(a0)
	BEQ.s	L628
	moveq	#0,D1
	MOVE.B	off1f(A0),D1
	EXT	D1
	swap	d1
	asr.l	#4,d1		;lsr.l #4,d1 corrige bug ???
	add.l	d1,off38(a0)
	SUB	off38(a0),D0
L628:
	MOVE.B	off2d(A0),D1

	;IFEQ	TURRICAN
	;SUB.B	off28(A0),D1
	;BPL.S	.NOVOL
	;CLR	D1
;.NOVOL:
	;RTS
	;ELSEIF
	MOVEQ	#-16,D2		;DEBUGGAGE VOLUME
	AND.B	D1,D2
	SUB.B	D2,D1
	SUB.B	off28(A0),D1
	BMI.S	.NOVOL
	OR.B	D2,D1
	RTS
.NOVOL:
	MOVE	D2,D1
	RTS
	;ENDC


LCA:


ZEROSND:
	clr.B	$22(A6)
	clr.B	$26(A6)
	clr.B	$2A(A6)
	MOVEM.L	$1C(A6),D0-D3
	MOVEM.L	D0-D3,$FFFF8800.W
	RTS

INITMUSIC:
;
; init musique
;
; entr‚e :
;	A0=pointe sur le texte 'COSO'
;	D0=num‚ro de la musique … jouer
;
	LEA	PSGREG(PC),A6
	ST	BLOQUEMUS-PSGREG(A6)

	subq	#1,d0
	BLT.S	LCA		;musique=0 -> cut mus



	;LEA		L51(PC),A1
	;MOVE.L	A1,MODIF1+2-PSGREG(A6)
	;LEA	flagdigit(PC),A1
	;MOVE.L	A1,MODIF2+2-PSGREG(A6)

	MOVE.L	A0,L934-PSGREG(A6)
	MOVE.L	$10(A0),A3
	ADD.L	A0,A3
	MOVE.L	$14(A0),A1
	ADD.L	A0,A1
;	ADD	D0,D0
;	ADD	D0,A1
;	ADD	D0,D0
	MULU	#6,D0
	ADD	D0,A1
	MOVEQ	#$C,D0
	MULU	(A1)+,D0	;PREMIER PATTERN
	MOVEQ	#$C,D2
	MULU	(A1)+,D2	;DERNIER PATTERN
	SUB	D0,D2

	ADD.L	D0,A3

	MOVE.B	1(A1),L810-PSGREG(A6)

	MOVEQ	#0,D0
	LEA	voice0(PC),A2
;
; REGISTRES UTILISES :
;
; D0=COMPTEUR VOIX 0-2
; D1=SCRATCH
; D2=PATTERN FIN
; A0={L934}
; A1=SCRATCH
; A2=VOICEX
; A3=PATTERN DEPART
; A6=BASE VARIABLES
;
L658:
	LEA	L7C6(PC),A1
	MOVE.L	A1,offa(A2)
	MOVE.L	A1,off12(A2)
	MOVEQ	#1,D1
	MOVE.B	D1,off17(A2)	;1
	MOVE.B	D1,off18(A2)	;1

	MOVE.B	d0,off21(A2)
	move.l	A3,off0(A2)
	move	D2,off34(A2)
	MOVE.B	#2,off2a(A2)

	moveq	#0,D1
	;IFEQ	OLD
	;MOVE	D1,off4(a2)
	;ELSEIF
	move	#$c,off4(A2)
	;ENDC

	MOVE	D1,offe(A2)
	MOVE.B	D1,off2d(A2)
	MOVE.B	D1,off8(A2)
	MOVE.B	D1,off9(A2)
	MOVE	D1,off30(A2)
	MOVE.B	D1,off19(A2)
	MOVE.B	D1,off1a(A2)
	MOVE.B	D1,off1b(A2)
	MOVE.B	D1,off1c(A2)
	MOVE.B	D1,off1d(A2)
	MOVE.B	D1,off1e(A2)
	MOVE.B	D1,off1f(A2)
	MOVE.L	D1,off38(A2)
	MOVE.B	D1,off26(A2)
	MOVE.B	D1,off27(A2)
	MOVE.B	D1,off2b(A2)

	move.b	(A3)+,D1
	ADD	D1,D1

	MOVE.L	A0,A1
	ADD	$C+2(A1),D1
	ADD	(A1,D1.W),A1

	MOVE.L	A1,off22(A2)
	move.b	(A3)+,off2c(A2)
	move.b	(A3)+,off16(A2)
	moveq	#$10,D1
	add.B	(A3)+,D1
	bcs.s	L712
	moveq	#0,D1
L712:
	MOVE.B	D1,off28(A2)
	lea	off3c(A2),A2
	ADDQ	#4,D2
	addq	#1,d0
	cmp	#3,d0
	blo	L658

	MOVE.B	#1,L80E-PSGREG(A6)
	;IFEQ	CUTMUS
;	CLR	BLOQUEMUS-PSGREG(A6)
	CLR.B	BLOQUEMUS-PSGREG(A6)
;	CLR.B	L813-PSGREG(A6)
	;ENDC
	RTS			;ou BRA ZEROSND

L7C6:	DC.B	1,0,0,0,0,0,0,$E1

PSGREG:	
	DC.W	$0000,$0000,$101,$0000
	DC.W	$0202,$0000,$303,$0000
	DC.W	$0404,$0000,$505,$0000
	DC.W	$0606,$0000,$707,$FFFF
	DC.W	$0808
	DC.W	$0000,$909,$0000
	DC.W	$0A0A,$0000

L94E:	DC.W	$EEE,$E17,$D4D,$C8E
	DC.W	$BD9,$B2F,$A8E,$9F7
	DC.W	$967,$8E0,$861,$7E8
	DC.W	$777,$70B,$6A6,$647
	DC.W	$5EC,$597,$547,$4FB
	DC.W	$4B3,$470,$430,$3F4
	DC.W	$3BB,$385,$353,$323
	DC.W	$2F6,$2CB,$2A3,$27D
	DC.W	$259,$238,$218,$1FA
	DC.W	$1DD,$1C2,$1A9,$191
	DC.W	$17B,$165,$151,$13E
	DC.W	$12C,$11C,$10C,$FD
	DC.W	$EE,$E1,$D4,$C8
	DC.W	$BD,$B2,$A8,$9F
	DC.W	$96,$8E,$86,$7E
	DC.W	$77,$70,$6A,$64
	DC.W	$5E,$59,$54,$4F
	DC.W	$4B,$47,$43,$3F
	DC.W	$3B,$38,$35,$32
	DC.W	$2F,$2C,$2A,$27
	DC.W	$25,$23,$21,$1F
	DC.W	$1D,$1C,$1A,$19
	DC.W	$17,$16,$15,$13
	DC.W	$12,$11,$10,$F
; amiga=C178a8
L80E:	DC.B	4
L810:	DC.B	4
	;IFEQ	CUTMUS
BLOQUEMUS:DC.B	-1
	;ENDC



	EVEN
voice0:	ds.B	off3c
voice1:	ds.B	off3c
voice2:	ds.B	off3c
L934:	DC.L	0


MUSIC:
	.INCBIN		"C:/Jaguar/COSO/fichiers mus/COSO/SEVGATES.MUS"
	even
	

;-------------------------------------
;
;    FIN COSO
;
;-------------------------------------






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
	movei	#DSP_LSP_routine_interruption_Timer1,r12						; 6 octets
	movei	#D_FLAGS,r16											; 6 octets
	jump	(r12)													; 2 octets
	load	(r16),r13	; read flags								; 2 octets = 16 octets
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
; bank 0 : 
; R28/R29/R30/R31
; +
; R18/R19/R20/R21/R22/R23/R24/R25/R26/R27
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

	.if		DSP_DEBUG
; change la couleur du fond
	movei	#$777,R26
	movei	#BG,r27
	storew	r26,(r27)
	.endif

	

;--------------------------
; gerer l'enveloppe
; - incrementer l'offset enveloppe
; partie entiere 16 bits : virgule 16 bits
; partie entiere and %1111 = position dans la sous partie d'enveloppe
; ( ( partie entiere >> 4 ) and %1 ) << 2 = pointeur sur la sous partie d'enveloppe


; si positif, limiter, masquer, à 11111 ( 5 bits:16 )

	movei	#YM_DSP_pointeur_enveloppe_en_cours,R24
	load	(R24),R24						; R24=pointeur sur la liste de 3 pointeur de sequence d'enveloppe : -1,0,1 : [ R24+(R25 * 4) ] + (R27*4)

YM_DSP_replay_sample_gere_env:
	movei	#YM_DSP_increment_enveloppe,R27
	movei	#YM_DSP_offset_enveloppe,R26
	load	(R27),R27
	load	(R26),R25				; R25 = offset en cours enveloppe
	add		R27,R25					; offset+increment 16:16
	
	move	R25,R23
	sharq	#16,R23					; on vire la virgule, on garde le signe
	moveq	#%1111,R21
	move	R23,R27
	and		R21,R27					; R27=partie entiere de l'offset AND 1111 = position dans la sous partie d'enveloppe
	

	sharq	#4,R23					; offset / 16, on garde le signe
	jr		mi, YM_DSP_replay_sample_offset_env_negatif
	moveq	#%1,R21
	movei	#$0FFFFFFF,R22
	and		R22,R25					; valeur positive : on limite la valeur pour ne pas qu'elle redevienne négative
	and		R21,R23					; R25 = pointeur sur la sous partie d'enveloppe
	
YM_DSP_replay_sample_offset_env_negatif:
	store	R25,(R26)				; sauvegarde YM_DSP_offset_enveloppe

	add		R23,R23					; R23*2 = partie entiere %1
	add		R27,R27					; R27*2
	add		R23,R23					; R23*4
	add		R27,R27					; R27*4
	
	add		R23,R24					; R24 = pointeur sur la partie d'enveloppe actuelle : R24+(R25 * 4) 
	load	(R24),R24				; R24 = pointeur sur la partie d'enveloppe actuelle :  [ R24+(R25 * 4) ]
	movei	#YM_DSP_volE,R26
	add		R27,R24					; [ R24+(R25 * 4) ] + (R27*4)
	load	(R24),R24				; R24 = volume actuel enveloppe
	or		R24,R24
	store	R24,(R26)				; volume de l'enveloppe => YM_DSP_volE


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
	cmpq	#0,R23
	jump	eq,(R20)
	nop
; il faut generer un nouveau noise
; il faut masquer R24 avec $FFFF
	movei	#$FFFF,R23
	and		R23,R24				; YM_DSP_position_offset_Noise, juste virgule

	.if		DSP_random_Noise_generator_method=1
; generer un nouveau pseudo random methode 1
	MOVEI	#YM_DSP_current_Noise, R23		
	LOAD	(R23), R21			
	MOVEQ	#$01, R20			
	MOVE	R21, R27			
	MOVE	R21, R25			
	SHRQ	#$02, R25			
	AND		R20, R27			
	AND		R20, R25			
	XOR		R27, R25			
	MOVE	R21, R27			
	MOVE	R25, R20			
	SHRQ	#$01, R27			
	SHLQ	#$10, R20			
	OR		R27, R20			
	STORE	R20, (R23)	
	.endif

	.if		DSP_random_Noise_generator_method=2
; does not work !
; generer un nouveau pseudo random methode 2 : seed = seed * 1103515245 + 12345;
	MOVEI	#YM_DSP_Noise_seed, R23		
	LOAD	(R23), R21			
	movei	#1103515245,R20
	mult	R20,R21
	or		R21,R21
	movei	#12345,R27
	add		R27,R21
	STORE	R21, (R23)	
	.endif

	.if		DSP_random_Noise_generator_method=3
; wyhash16 : https://lemire.me/blog/2019/07/03/a-fast-16-bit-random-number-generator/
	MOVEI	#YM_DSP_Noise_seed, R23	
	movei	#$fc15,R20
	LOAD	(R23), R21
	add		R20,R21
	movei	#$2ab,R20
	mult	R20,R21
	move	R21,R25
	rorq	#16,R21
	xor		R25,R21
	store	R21,(R23)
	.endif

	.if		DSP_random_Noise_generator_method=4
; generer un nouveau pseudo random LFSR YM : https://www.smspower.org/Development/YM2413ReverseEngineeringNotes2018-05-13
	MOVEI	#YM_DSP_current_Noise, R23		
	LOAD	(R23), R21
	
	moveq	#1,R27
	move	R21,R20
	and		R27,R20				; 	bool output = state & 1;

	shrq	#1,R21				; 	state >>= 1;
	
	cmpq	#0,R20
	jr		eq,YM_DSP_replay_sample_LFSR_bit_0_egal_0
	
	nop
	movei	#$400181,R20
	xor		R20,R21
	
YM_DSP_replay_sample_LFSR_bit_0_egal_0:
	store	R21,(R23)
	.endif

; calcul masque 
	MOVEQ	#$01,R20
	and		R20,R21			; on garde juste le bit 0
	sub		R20,R21			; 0-1= -1 / 1-1=0 => mask sur 32 bits
	or		R21,R21
	store	R21,(R22)		; R21=>YM_DSP_current_Noise_mask
	move	R21,R18

YM_DSP_replay_sample_pas_de_generation_nouveau_Noise:
; en entrée : R24 = offset noise, R18 = current mask Noise

	store	R24,(R26)			; R24=>YM_DSP_position_offset_Noise


;---- ====> R18 = mask current Noise ----


;--------------------------
; ----- gerer digidrum A
	movei	#YM_DSP_pointeur_sample_digidrum_voie_A,R27					; pointeur << 21 + 11 bits de virgule 21:11
	load	(R27),R26
	movei	#YM_DSP_replay_sample_pas_de_digidrums_voie_A,R24
	cmpq	#0,R26
	jump	eq,(R24)
	nop

	move	R26,R24
	shrq	#YM_DSP_precision_virgule_digidrums,R24				; partie entiere du pointeur sample DG

	loadb	(R24),R23			; R23=sample DG sur 4 bits : de 0 a 15
	movei	#YM_DSP_table_de_volumes,R25
	shlq	#2,R23				; * 4 
	add		R23,R25
	movei	#YM_DSP_volA,R22
	movei	#YM_DSP_pointeur_sur_source_du_volume_A,R24
	load	(R25),R23
	store	R22,(R24)
	store	R23,(R22)			; volume du sample DG
	
	movei	#YM_DSP_increment_sample_digidrum_voie_A,R25				; increment << 21 + 11 bits de virgule 21:11
	movei	#YM_DSP_pointeur_fin_sample_digidrum_voie_A,R24
	load	(R25),R25
	load	(R24),R24					; pointeur de fin 21:11
	add		R25,R26						; pointeur + increment 21:11
	cmp		R24,R26
	jr		mi,YM_DSP_replay_DG_pas_fin_de_sample_voie_A
	nop
	moveq	#0,R26
YM_DSP_replay_DG_pas_fin_de_sample_voie_A:
	store	R26,(R27)			; YM_DSP_pointeur_sample_digidrum_voie_A

YM_DSP_replay_sample_pas_de_digidrums_voie_A:


; ----- gerer digidrum B
	movei	#YM_DSP_pointeur_sample_digidrum_voie_B,R27					; pointeur << 21 + 11 bits de virgule 21:11
	load	(R27),R26
	movei	#YM_DSP_replay_sample_pas_de_digidrums_voie_B,R24
	cmpq	#0,R26
	jump	eq,(R24)
	nop

	move	R26,R24
	shrq	#YM_DSP_precision_virgule_digidrums,R24				; partie entiere du pointeur sample DG

	loadb	(R24),R23			; R23=sample DG sur 4 bits : de 0 a 15
	movei	#YM_DSP_table_de_volumes,R25
	shlq	#2,R23				; * 4 
	add		R23,R25
	movei	#YM_DSP_volB,R22
	movei	#YM_DSP_pointeur_sur_source_du_volume_B,R24
	load	(R25),R23
	store	R22,(R24)
	store	R23,(R22)			; volume du sample DG
	
	movei	#YM_DSP_increment_sample_digidrum_voie_B,R25				; increment << 21 + 11 bits de virgule 21:11
	movei	#YM_DSP_pointeur_fin_sample_digidrum_voie_B,R24
	load	(R25),R25
	load	(R24),R24					; pointeur de fin 21:11
	add		R25,R26						; pointeur + increment 21:11
	cmp		R24,R26
	jr		mi,YM_DSP_replay_DG_pas_fin_de_sample_voie_B
	nop
	moveq	#0,R26
YM_DSP_replay_DG_pas_fin_de_sample_voie_B:
	store	R26,(R27)			; YM_DSP_pointeur_sample_digidrum_voie_B

YM_DSP_replay_sample_pas_de_digidrums_voie_B:


; ----- gerer digidrum C
	movei	#YM_DSP_pointeur_sample_digidrum_voie_C,R27					; pointeur << 21 + 11 bits de virgule 21:11
	load	(R27),R26
	movei	#YM_DSP_replay_sample_pas_de_digidrums_voie_C,R24
	cmpq	#0,R26
	jump	eq,(R24)
	nop

	move	R26,R24
	shrq	#YM_DSP_precision_virgule_digidrums,R24				; partie entiere du pointeur sample DG

	loadb	(R24),R23			; R23=sample DG sur 4 bits : de 0 a 15
	movei	#YM_DSP_table_de_volumes,R25
	shlq	#2,R23				; * 4 
	add		R23,R25
	movei	#YM_DSP_volC,R22
	movei	#YM_DSP_pointeur_sur_source_du_volume_C,R24
	load	(R25),R23
	store	R22,(R24)
	store	R23,(R22)			; volume du sample DG
	
	movei	#YM_DSP_increment_sample_digidrum_voie_C,R25				; increment << 21 + 11 bits de virgule 21:11
	movei	#YM_DSP_pointeur_fin_sample_digidrum_voie_C,R24
	load	(R25),R25
	load	(R24),R24					; pointeur de fin 21:11
	add		R25,R26						; pointeur + increment 21:11
	cmp		R24,R26
	jr		mi,YM_DSP_replay_DG_pas_fin_de_sample_voie_C
	nop
	moveq	#0,R26
YM_DSP_replay_DG_pas_fin_de_sample_voie_C:
	store	R26,(R27)			; YM_DSP_pointeur_sample_digidrum_voie_C

YM_DSP_replay_sample_pas_de_digidrums_voie_C:



;---- ====> R18 = mask current Noise ----
;--------------------------
; gérer les voies A B C 
; ---------------


; canal A

	movei	#YM_DSP_Mixer_NA,R26

	move	R18,R24				; R24 = on garde la masque du current Noise

	load	(R26),R26			; YM_DSP_Mixer_NA
	or		R26,R18				; YM_DSP_Mixer_NA OR Noise
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

	movei	#YM_DSP_pointeur_sur_source_du_volume_A,R26
	and		R18,R25					; R25 = Noise and Tone

	load	(R26),R27				; R20 = pointeur sur la source de volume pour le canal A
	load	(r27),R20				; R20=volume pour le canal A 0 à 32767
	
	;movei	#pointeur_buffer_de_debug,R26
	;load	(R26),R18
	;store	R20,(R18)
	;addq	#4,R18
	;store	R18,(R26)
	;nop
	
	
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
	or		R25,R25
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
	or		R25,R25
	store	R25,(R26)							; YM_DSP_position_offset_B
	shrq	#31,R25
	neg		R25									; 0 devient 0, 1 devient -1 ($FFFFFFFF)
; R25 = onde carrée C

	movei	#YM_DSP_Mixer_TC,R26
	load	(R26),R26
	or		R26,R25
; R25 = onde carrée B OR mask du registre 7 de mixage Tone C

; Noise AND Tone

	movei	#YM_DSP_pointeur_sur_source_du_volume_C,R22
	and		R18,R25					; R25 = Noise and Tone
	load	(R22),R22				; R23 = pointeur sur la source de volume pour le canal B
	load	(r22),R22				; R23=volume pour le canal B 0 à 32767
	and		R25,R22					; R23=volume pour le canal B
; R22 = sample canal C

; sans stereo : R20=A / R23=B / R22=C / R21=//

; mono desactivé
	.if		STEREO=0
	shrq	#1,R20					; quand volume maxi = 32767
	;shrq	#1,R21					; quand volume maxi = 32767
	shrq	#1,R23
	shrq	#1,R22
	add		R23,R20					; R20 = R20=canal A + R23=canal B
	;add		R21,R20					; R20 = R20=canal A + R23=canal B + R21=canal D
	movei	#32768,R27
	add		R22,R20					; + canal C
	movei	#L_I2S,r26
	sub		R27,R20					; resultat signé sur 16 bits
	movei	#L_I2S+4,r24
	store	r20,(r26)				; write right channel
	store	r20,(r24)				; write left channel
	.endif

	
	.if		STEREO=1

	movei	#YM_DSP_Voie_A_pourcentage_Droite,R24
	move	R20,R26					; R26=A
	mult	R24,R26
	shrq	#STEREO_shit_bits,R26
	
	movei	#YM_DSP_Voie_B_pourcentage_Droite,R24
	move	R23,R25					; R27=B
	mult	R24,R25
	shrq	#STEREO_shit_bits,R25
	
	movei	#YM_DSP_Voie_C_pourcentage_Droite,R24
	move	R22,R18					; R18=C
	mult	R24,R18
	shrq	#STEREO_shit_bits,R18

	add		R26,R25					; R27=A+B

	movei	#YM_DSP_Voie_D_pourcentage_Droite,R24
	move	R21,R26					; R26=D
	mult	R24,R26
	shrq	#STEREO_shit_bits,R26
	
	add		R18,R25
	add		R26,R25					; R25=droite


	movei	#YM_DSP_Voie_A_pourcentage_Gauche,R24
	mult	R24,R20
	shrq	#STEREO_shit_bits,R20
	
	movei	#YM_DSP_Voie_B_pourcentage_Gauche,R24
	mult	R24,R23
	shrq	#STEREO_shit_bits,R23
	
	movei	#YM_DSP_Voie_C_pourcentage_Gauche,R24
	mult	R24,R22
	shrq	#STEREO_shit_bits,R22

	add		R20,R23					; R23=A+B

	movei	#YM_DSP_Voie_D_pourcentage_Gauche,R24
	mult	R24,R21
	shrq	#STEREO_shit_bits,R21

	movei	#32768,R27
	
	add		R22,R23
	add		R21,R23					; R23=gauche

	sub		R27,R25
	movei	#L_I2S,r26
	sub		R27,R23
	movei	#L_I2S+4,r24

	store	r25,(r26)				; write right channel
	store	r23,(r24)				; write left channel

	.endif

	.if		DSP_DEBUG
; change la couleur du fond
	movei	#$000,R26
	movei	#BG,r27
	storew	r26,(r27)
	.endif

;------------------------------------	
; return from interrupt I2S
	load	(r31),r28	; return address
	bset	#10,r29		; clear latch 1 = I2S
	;bset	#11,r29		; clear latch 1 = timer 1
	;bset	#12,r29		; clear latch 1 = timer 2
	bclr	#3,r29		; clear IMASK
	addq	#4,r31		; pop from stack
	addqt	#2,r28		; next instruction
	jump	t,(r28)		; return
	store	r29,(r30)	; restore flags




















;--------------------------------------------
; ---------------- Timer 1 ------------------
;--------------------------------------------
; autorise interruptions, pour timer I2S
	.if		I2S_during_Timer1=1
	bclr	#3,r13		; clear IMASK
	store	r13,(r16)	; restore flags
	.endif

DSP_LSP_routine_interruption_Timer1:
	.if		DSP_DEBUG_T1
; change la couleur du fond
	movei	#$077,R1
	movei	#BG,r0
	loadw	(r0),r1
	addq	#$1,r1
	storew	r1,(r0)
	.endif


;-------------------------------------------------------------------------------------------------
; -------------------------------------------------------------------------------
; routine de lecture des registres YM
; bank 0 : 
 ; gestion timer deplacé sur :
; R12(R28)/R13(R29)/R16(R30)
; +
; R0/R1/R2/R3/R4/R5/R6/R7/R8/R9/R10/R11 + R14
; -------------------------------------------------------------------------------
	;-------------------------------------------------------------------------------------------------
; COSO = 11+3 registres
	movei		#YM_registres_Coso,R1
	moveq		#1,R8



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
	or			R5,R5
	shlq		#16,R5
	
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
	or			R5,R5
	shlq		#16,R5
	
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
	or			R5,R5
	shlq		#16,R5
	
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
	or			R5,R5
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
	;subq		#1,R4					; 0=>-1 / 1=>0 
	neg			R4						; 0=>0 / 1=>-1
	shlq		#1,R3					; bit suivant
	store		R4,(R5)

; bit 1 = Tone B
	move		R2,R4
	movei		#YM_DSP_Mixer_TB,R5
	and			R3,R4					; 0 ou 1
	shrq		#1,R4
	;subq		#1,R4					; 0=>-1 / 1=>0 
	neg			R4						; 0=>0 / 1=>-1
	shlq		#1,R3					; bit suivant
	store		R4,(R5)

; bit 2 = Tone C
	move		R2,R4
	movei		#YM_DSP_Mixer_TC,R5
	and			R3,R4					; 0 ou 1
	shrq		#2,R4
	;subq		#1,R4					; 0=>-1 / 1=>0 
	neg			R4						; 0=>0 / 1=>-1
	shlq		#1,R3					; bit suivant
	store		R4,(R5)
	
; bit 3 = Noise A
	move		R2,R4
	movei		#YM_DSP_Mixer_NA,R5
	and			R3,R4					; 0 ou 1
	shrq		#3,R4
	;subq		#1,R4					; 0=>-1 / 1=>0 
	neg			R4						; 0=>0 / 1=>-1
	shlq		#1,R3					; bit suivant
	store		R4,(R5)
	
; bit 4 = Noise B
	move		R2,R4
	movei		#YM_DSP_Mixer_NB,R5
	and			R3,R4					; 0 ou 1
	shrq		#4,R4
	neg			R4						; 0=>0 / 1=>-1
	;subq		#1,R4					; 0=>-1 / 1=>0 
	shlq		#1,R3					; bit suivant
	store		R4,(R5)
	
; bit 5 = Noise C
	move		R2,R4
	movei		#YM_DSP_Mixer_NC,R5
	and			R3,R4					; 0 ou 1
	shrq		#5,R4
	neg			R4						; 0=>0 / 1=>-1
;	subq		#1,R4					; 0=>-1 / 1=>0 
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

	move		R2,R4
	movei		#YM_DSP_registre8,R6
	moveq		#%1111,R3
	store		R4,(R6)					; sauvegarde la valeur de volume sur 16, pour DG
	movei		#YM_DSP_volE,R5
	and			R3,R4
	
	shlq		#2,R4					; volume sur 16 *4 
	load		(R14+R4),R4

	movei		#YM_DSP_volA,R6
	store		R4,(R6)

	movei		#YM_DSP_pointeur_sur_source_du_volume_A,R3
	btst		#4,R2					; test bit M : M=0 => volume contenu dans registre 8 / M=1 => volume d'env
	jr			ne,DSP_lecture_registre8_pas_volume_A
	nop
	
	move		R6,R5
	
DSP_lecture_registre8_pas_volume_A:
	store		R5,(R3)


; registre 9 = volume canal B
; B4=1 bit =M / M=0=>volume fixe / M=1=>volume enveloppe
; B3/B2/B1/B0 = volume fixe pour le canal B
;	Noise	 Tone
;	C B A    C B A
	loadb		(R1),R2						; registre 9
	add			R8,R1	

	move		R2,R4
	movei		#YM_DSP_registre9,R6
	moveq		#%1111,R3
	store		R4,(R6)					; sauvegarde la valeur de volume sur 16, pour DG
	movei		#YM_DSP_volE,R5
	and			R3,R4

	shlq		#2,R4					; volume sur 16 *4 
	load		(R14+R4),R4

	movei		#YM_DSP_volB,R6
	store		R4,(R6)

	movei		#YM_DSP_pointeur_sur_source_du_volume_B,R3

	btst		#4,R2
	jr			ne,DSP_lecture_registre9_pas_env
	nop
	
	move		R6,R5
	
DSP_lecture_registre9_pas_env:
	store		R5,(R3)

; registre 10 = volume canal C
; B4=1 bit =M / M=0=>volume fixe / M=1=>volume enveloppe
; B3/B2/B1/B0 = volume fixe pour le canal C
;	Noise	 Tone
;	C B A    C B A
	loadb		(R1),R2						; registre 10
	add			R8,R1	

	move		R2,R4
	movei		#YM_DSP_registre10,R6
	moveq		#%1111,R3
	store		R4,(R6)					; sauvegarde la valeur de volume sur 16, pour DG
	movei		#YM_DSP_volE,R5
	and			R3,R4
	
	shlq		#2,R4					; volume sur 16 *4 
	load		(R14+R4),R4
	
	movei		#YM_DSP_volC,R6
	store		R4,(R6)

	movei		#YM_DSP_pointeur_sur_source_du_volume_C,R3

	btst		#4,R2
	jr			ne,DSP_lecture_registre10_pas_env
	nop

	move		R6,R5
	
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

	jr			ne,DSP_lecture_registre11_12_pas_zero
	nop
	moveq		#0,R5
	jr			DSP_lecture_registre11_12_zero
	nop
	
DSP_lecture_registre11_12_pas_zero:	
	div			r3,R5

DSP_lecture_registre11_12_zero:	
	movei		#YM_DSP_increment_enveloppe,R2
	or			R5,R5
	store		R5,(R2)


; registre 13 = envelop shape
	loadb		(R1),R2						; registre 13 = Envelope shape control

	movei		#YM_DSP_registre13,R6

	add			R8,R1

	store		R2,(R6)					; sauvegarde la valeur env shape registre 13

; tester si bit 7 = 1 => ne pas modifier l'env en cours

	movei		#DSP_lecture_registre13_pas_env,R3
	btst		#7,R2
	jump		ne,(R3)
	nop

; - choix de la bonne enveloppe
	sub			R8,R1
	bset		#7,R2
	storeb		R2,(R1)
	add			R8,R1
	
	
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

DSP_lecture_registre13_pas_env:


	.if		1=0
; ----------------
; registre R11 = flag effets sur les voies : A=bit 0, B=bit 1, C=bit 2, bit 3=buzzer , bit 4=Sinus Sid
	movei	#YM_flag_effets_sur_les_voies,R11
	load	(R11),R11

;--------------------------------
; gestion des effets par voie
; ------- effet sur voie A ?
	;movei		#YM_flag_effets_voie_A,R3
	;load		(R3),R3
	movei		#DSP_lecture_registre_effet_voie_A_pas_d_effet,R4
	;cmpq		#0,R3
	btst		#0,R11
	jump		eq,(R4)
	
	loadb		(R1),R2						; octet 1 effet sur la voie : 8 bits du haut = index prediv ( sur 3 bits 0-7 )
	add			R8,R1
	loadb		(R1),R3						; octet 2 effet sur la voie : 8 bits du bas = diviseur
	add			R8,R1

	movei		#DSP_lecture_registre_effet_voie_A_pas_de_DG,R4
	btst		#7,R2
	jump		eq,(R4)

;--------------------------------
; digidrums sur la voie A
;--------------------------------
	moveq		#%111,R5
	movei		#YM_DSP_table_prediviseur,R6
	and			R5,R2						; 3 bits de R2 = prediviseur
	shlq		#2,R2						; * 4 
	add			R2,R6
	load		(R6),R6						; R6=prediviseur
	
	mult		R6,R3						; R3=prediviseur * diviseur
	movei		#YM_DSP_frequence_MFP,R5
	div			R3,R5						; frequence du MFP / ( prediviseur * diviseur )
	movei		#DSP_frequence_de_replay_reelle_I2S,R4
	load		(R4),R4
	or			R5,R5
	shlq		#YM_DSP_precision_virgule_digidrums,R5
	div			R4,R5						; R5=increment digidrum=(frequence du MFP / ( prediviseur * diviseur ) ) / frequence_de_replay_reelle_I2S en 16:16
	movei		#YM_DSP_table_digidrums,R3
	movei		#YM_DSP_registre8,R6
	load		(R6),R6
	shlq		#3,R6						; numero sample * 8
	add			R6,R3						; pointe sur pointeur sample + pointeur fin de sample
	load		(R3),R2						; R2=pointeur debut sample DG en 21:11
	movei		#YM_DSP_pointeur_sample_digidrum_voie_A,R6
	addq		#4,R3
	load		(R3),R4						; R4=pointeur fin sample DG en 21:11
	store		R2,(R6)						; stocke debut sample DG en 21:11
	addq		#4,R6						; passe au pointeur de fin du sample
	store		R4,(R6)						; stocke fin sample DG en 21:11
	addq		#4,R6						; passe au pointeur de fin du sample
	store		R5,(R6)						; stocke increment sample DG en 21:11

; force volume sur volA, mixerTA et mixerNA = $FFFFFFFF
	movei		#YM_DSP_pointeur_sur_source_du_volume_A,R3
	movei		#-1,R2
	movei		#YM_DSP_volA,R5
	movei		#YM_DSP_Mixer_NA,R4
	store		R5,(R3)
	movei		#YM_DSP_Mixer_TA,R7
	store		R2,(R4)
	movei		#DSP_lecture_registre_effet_voie_A_pas_d_effet,R3
	store		R2,(R7)
	
	jump		(R3)		; saute par dessus la routine SID
	nop
	
; numero sample DG = registre 8
; R2 and 11 bits = frequence de replay : table de frequence mfp -$400 : 
; stop, no function executed		: 256 valeurs = 0
; subdivider divides by 4
; subdivider divides by 10
; subdivider divides by 16
; subdivider divides by 16
; subdivider divides by 50
; subdivider divides by 64
; subdivider divides by 100
; subdivider divides by 200
;
; ( 2457600 / DSP_frequence_de_replay_reelle_I2S ) / prediv (4/10/16/16/50/64/100/200) / valeur sur 8 bits
; => ( 2457600 / DSP_frequence_de_replay_reelle_I2S ) (précalcumé) / ( prediv * valeur )
; mfpPrediv[8] = {0,4,10,16,50,64,100,200};
; premiere valeur = index prediv ( sur 3 bits 0-7 )
; deuxieme valeur = diviseur

DSP_lecture_registre_effet_voie_A_pas_de_DG:

DSP_lecture_registre_effet_voie_A_pas_d_effet:

; -----------------------------
; ------- effet sur voie B ?

;	movei		#YM_flag_effets_voie_B,R3
;	load		(R3),R3
	movei		#DSP_lecture_registre_effet_voie_B_pas_d_effet,R4
	;cmpq		#0,R3
	btst		#1,R11
	jump		eq,(R4)
	
	loadb		(R1),R2						; octet 1 effet sur la voie : 8 bits du haut = index prediv ( sur 3 bits 0-7 )
	add			R8,R1
	loadb		(R1),R3						; octet 2 effet sur la voie : 8 bits du bas = diviseur
	add			R8,R1

	movei		#DSP_lecture_registre_effet_voie_B_pas_de_DG,R4
	btst		#7,R2
	jump		eq,(R4)
; digidrums sur la voie B
	moveq		#%111,R5
	movei		#YM_DSP_table_prediviseur,R6
	and			R5,R2						; 3 bits de R2 = prediviseur
	shlq		#2,R2						; * 4 
	add			R2,R6
	load		(R6),R6						; R6=prediviseur
	
	mult		R6,R3						; R3=prediviseur * diviseur
	movei		#YM_DSP_frequence_MFP,R5
	div			R3,R5						; frequence du MFP / ( prediviseur * diviseur )
	movei		#DSP_frequence_de_replay_reelle_I2S,R4
	load		(R4),R4
	or			R5,R5
	shlq		#YM_DSP_precision_virgule_digidrums,R5
	div			R4,R5						; R5=increment digidrum=(frequence du MFP / ( prediviseur * diviseur ) ) / frequence_de_replay_reelle_I2S en 16:16
	movei		#YM_DSP_table_digidrums,R3
	movei		#YM_DSP_registre9,R6
	load		(R6),R6
	shlq		#3,R6						; numero sample * 8
	add			R6,R3						; pointe sur pointeur sample + pointeur fin de sample
	load		(R3),R2						; R2=pointeur debut sample DG en 21:11
	movei		#YM_DSP_pointeur_sample_digidrum_voie_B,R6
	addq		#4,R3
	load		(R3),R4						; R4=pointeur fin sample DG en 21:11
	store		R2,(R6)						; stocke debut sample DG en 21:11
	addq		#4,R6						; passe au pointeur de fin du sample
	store		R4,(R6)						; stocke fin sample DG en 21:11
	addq		#4,R6						; passe au pointeur de fin du sample
	store		R5,(R6)						; stocke increment sample DG e: 21:11

; force volume sur volB, mixerTB et mixerNB = $FFFFFFFF
	movei		#YM_DSP_pointeur_sur_source_du_volume_B,R3
	movei		#-1,R2
	movei		#YM_DSP_volB,R5
	movei		#YM_DSP_Mixer_NB,R4
	store		R5,(R3)
	movei		#YM_DSP_Mixer_TB,R7
	store		R2,(R4)
	movei		#DSP_lecture_registre_effet_voie_B_pas_d_effet,R3
	store		R2,(R7)
	
	jump		(R3)		; saute par dessus la routine SID
	nop

DSP_lecture_registre_effet_voie_B_pas_de_DG:
DSP_lecture_registre_effet_voie_B_pas_d_effet:



; -----------------------------
; ------- effet sur voie C ?
	;movei		#YM_flag_effets_voie_C,R3
	;load		(R3),R3
	movei		#DSP_lecture_registre_effet_voie_C_pas_d_effet,R4
	;cmpq		#0,R3
	btst		#2,R11
	jump		eq,(R4)
	
	loadb		(R1),R2						; octet 1 effet sur la voie : 8 bits du haut = index prediv ( sur 3 bits 0-7 )
	add			R8,R1
	loadb		(R1),R3						; octet 2 effet sur la voie : 8 bits du bas = diviseur
	add			R8,R1

	movei		#DSP_lecture_registre_effet_voie_C_pas_de_DG,R4
	btst		#7,R2
	jump		eq,(R4)
; digidrums sur la voie C
	moveq		#%111,R5
	
	
	movei		#YM_DSP_table_prediviseur,R6
	and			R5,R2						; 3 bits de R2 = prediviseur
	shlq		#2,R2						; * 4 
	add			R2,R6
	load		(R6),R6						; R6=prediviseur
	
	mult		R6,R3						; R3=prediviseur * diviseur
	movei		#YM_DSP_frequence_MFP,R5
	div			R3,R5						; frequence du MFP / ( prediviseur * diviseur )
	movei		#DSP_frequence_de_replay_reelle_I2S,R4
	load		(R4),R4
	or			R5,R5
	shlq		#YM_DSP_precision_virgule_digidrums,R5
	div			R4,R5						; R5=increment digidrum=(frequence du MFP / ( prediviseur * diviseur ) ) / frequence_de_replay_reelle_I2S en 16:16
	movei		#YM_DSP_table_digidrums,R3
	movei		#YM_DSP_registre10,R6
	load		(R6),R6
	shlq		#3,R6						; numero sample * 8
	add			R6,R3						; pointe sur pointeur sample + pointeur fin de sample
	load		(R3),R2						; R2=pointeur debut sample DG en 21:11
	movei		#YM_DSP_pointeur_sample_digidrum_voie_C,R6
	addq		#4,R3
	load		(R3),R4						; R4=pointeur fin sample DG en 21:11
	store		R2,(R6)						; stocke debut sample DG en 21:11
	addq		#4,R6						; passe au pointeur de fin du sample
	store		R4,(R6)						; stocke fin sample DG en 21:11
	addq		#4,R6						; passe au pointeur de fin du sample
	store		R5,(R6)						; stocke increment sample DG en 21:11

; force volume sur volC, mixerTC et mixerNC = $FFFFFFFF
	movei		#YM_DSP_pointeur_sur_source_du_volume_C,R3
	movei		#-1,R2
	movei		#YM_DSP_volC,R5
	movei		#YM_DSP_Mixer_NC,R4
	store		R5,(R3)
	movei		#YM_DSP_Mixer_TC,R7
	store		R2,(R4)
	movei		#DSP_lecture_registre_effet_voie_C_pas_d_effet,R3
	store		R2,(R7)
	
	jump		(R3)		; saute par dessus la routine SID
	nop	

DSP_lecture_registre_effet_voie_C_pas_de_DG:
DSP_lecture_registre_effet_voie_C_pas_d_effet:


	.endif

;---> precalculer les valeurs qui ne bougent pas pendant 1 VBL entiere	

; debug raz pointeur buffer debug
	;movei		#pointeur_buffer_de_debug,R0
	;movei		#buffer_de_debug,R1
	;store		R1,(R0)	
	;nop

; reading coso registers is done
	movei	#DSP_flag_registres_YM_lus,R2
	moveq	#1,R0
	store	R0,(R2)


	movei	#vbl_counter_replay_DSP,R0
	load	(R0),R1
	addq	#1,R1
	store	R1,(R0)
	
	.if		DSP_DEBUG_T1
; change la couleur du fond
	movei	#$000,R0
	movei	#BG,R1
	;storew	R0,(R1)
	.endif

;------------------------------------	
; return from interrupt Timer 1
	load	(r31),r12	; return address
	;bset	#10,r29		; clear latch 1 = I2S
	bset	#11,r13		; clear latch 1 = timer 1
	;bset	#12,r29		; clear latch 1 = timer 2
	bclr	#3,r13		; clear IMASK
	addq	#4,r31		; pop from stack
	addqt	#2,r12		; next instruction
	jump	t,(r12)		; return
	store	r13,(r16)	; restore flags


; ------------------- N/A ------------------
DSP_LSP_routine_interruption_Timer2:
; ------------------- N/A ------------------













; ----------------------------------------------
; routine d'init du DSP
; registres bloqués par les interruptions : R29/R30/R31 ?
DSP_routine_init_DSP:
; assume run from bank 1
	movei	#DSP_ISP+(DSP_STACK_SIZE*4),r31			; init isp
	moveq	#0,r1
	moveta	r31,r31									; ISP (bank 0)
	movei	#DSP_USP+(DSP_STACK_SIZE*4),r31			; init usp
	
; -------------------------------------------------------------------------------
; calcul de la frequence prédivisee pour le YM
; ((YM_frequence_YM2149/16)*65536)/DSP_Audio_frequence

	movei	#YM_frequence_YM2149,r0
	shlq	#16-4-2,r0					; /16 puis * 65536
	
	movei	#DSP_frequence_de_replay_reelle_I2S,r2
	load	(r2),r2
	
	div		r2,r0
	or		r0,r0					; attente fin de division
	shlq	#2,r0					; ramene a *65536

	
	movei	#YM_frequence_predivise,r1
	store	r0,(r1)



;calcul de ( 1<<31) / frequence de replay réelle )

	moveq	#1,R0
	shlq	#31,R0
	div		r2,r0
	or		R0,R0
	
	movei	#DSP_UN_sur_frequence_de_replay_reelle_I2S,r1
	store	R0,(R1)



; init I2S
	movei	#SCLK,r10
	movei	#SMODE,r11
	movei	#DSP_parametre_de_frequence_I2S,r12
	movei	#%001101,r13			; SMODE bascule sur RISING
	load	(r12),r12				; SCLK
	store	r12,(r10)
	store	r13,(r11)

; init Timer 1

	movei	#182150,R10				; 26593900 / 146 = 182150
	movei	#YM_frequence_replay,R11
	load	(R11),R11
	or		R11,R11
	div		R11,R10
	or		R10,R10
	move	R10,R13
	
	subq	#1,R13					; -1 pour parametrage du timer 1
	
	

; 26593900 / 50 = 531 878 => 2 × 73 × 3643 => 146*3643
	movei	#JPIT1,r10				; F10000
	;movei	#JPIT2,r11				; F10002
	movei	#146-1,r12				; Timer 1 Pre-scaler
	;movei	#3643-1,r13				; Timer 1 Divider  
	
	shlq	#16,r12
	or		R13,R12
	
	store	r12,(r10)				; JPIT1 & JPIT2


; init timer 2

;	movei	#JPIT3,r10				; F10004
;	movei	#JPIT4,r11				; F10006



; enable interrupts
	movei	#D_FLAGS,r28
	
	movei	#D_I2SENA|D_TIM1ENA|REGPAGE,r29			; I2S+Timer 1
	
	;movei	#D_TIM1ENA|REGPAGE,r29					; Timer 1 only
	;movei	#D_I2SENA|REGPAGE,r29					; I2S only
	;movei	#D_TIM2ENA|REGPAGE,r29					; Timer 2 only
	
	store	r29,(r28)



DSP_boucle_centrale:
	movei	#DSP_boucle_centrale,R20
	jump	(R20)
	nop

	


	.phrase


; datas DSP
DSP_flag_registres_YM_lus:			dc.l			0

vbl_counter_replay_DSP:				dc.l			0
YM_DSP_pointeur_sur_table_des_pointeurs_env_Buzzer:		dc.l		0

YM_DSP_registre8:			dc.l			0
YM_DSP_registre9:			dc.l			0
YM_DSP_registre10:			dc.l			0
YM_DSP_registre13:			dc.l			0

DSP_frequence_de_replay_reelle_I2S:					dc.l			0
DSP_UN_sur_frequence_de_replay_reelle_I2S:			dc.l			0
DSP_parametre_de_frequence_I2S:						dc.l			0

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

YM_DSP_volA:					dc.l			$1234
YM_DSP_volB:					dc.l			$1234
YM_DSP_volC:					dc.l			$1234

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
YM_DSP_current_Noise:			dc.l			$12071971
YM_DSP_current_Noise_mask:		dc.l			0
YM_DSP_Noise_seed:				dc.l			$12071971


; variables DG
YM_DSP_pointeur_sample_digidrum_voie_A:				dc.l		0
YM_DSP_pointeur_fin_sample_digidrum_voie_A:			dc.l		0
YM_DSP_increment_sample_digidrum_voie_A:			dc.l		0

YM_DSP_pointeur_sample_digidrum_voie_B:				dc.l		0
YM_DSP_pointeur_fin_sample_digidrum_voie_B:			dc.l		0
YM_DSP_increment_sample_digidrum_voie_B:			dc.l		0

YM_DSP_pointeur_sample_digidrum_voie_C:				dc.l		0
YM_DSP_pointeur_fin_sample_digidrum_voie_C:			dc.l		0
YM_DSP_increment_sample_digidrum_voie_C:			dc.l		0


YM_DSP_table_de_volumes:
	dc.l				0,161,265,377,580,774,1155,1575,2260,3088,4570,6233,9330,13187,21220,32767
; table volumes Amiga:
	;dc.l				$00*$c0, $00*$c0, $00*$c0, $00*$c0, $01*$c0, $02*$c0, $02*$c0, $04*$c0, $05*$c0, $08*$c0, $0B*$c0, $10*$c0, $18*$c0, $22*$c0, $37*$c0, $55*$c0
	
; volume 4 bits en 8 bits
; $00 $00 $00 $00 $01 $02 $02 $04 $05 $08 $0B $10 $18 $22 $37 $55
; ramené à 16383 ( 65535 / 4)
; *$c0

	;dc.l				0,161/2,265/2,377/2,580/2,774/2,1155/2,1575/2,2260/2,3088/2,4570/2,6233/2,9330/2,13187/2,21220/2,32767/2

					; 62,161,265,377,580,774,1155,1575,2260,3088,4570,6233,9330,13187,21220,32767



YM_DSP_table_prediviseur:
	dc.l		0,4,10,16,50,64,100,200	

; flags pour nb octets à lire
YM_flag_effets_sur_les_voies:			dc.l				0
YM_flag_effets_voie_A:		dc.l		0
YM_flag_effets_voie_B:		dc.l		0
YM_flag_effets_voie_C:		dc.l		0


PSG_compteur_frames_restantes:			dc.l		0
YM_pointeur_actuel_ymdata:				dc.l		0

; - le registre 13 definit la forme de l'enveloppe
; - on initialise une valeur à -16
; partie entiere 16 bits : virgule 16 bits
; partie entiere and %1111 = position dans la sous partie d'enveloppe
; ( ( partie entiere >> 4 ) and %1 ) << 2 = pointeur sur la sous partie d'enveloppe


YM_DSP_forme_enveloppe_1:
; enveloppe montante
	dc.l				62,161,265,377,580,774,1155,1575,2260,3088,4570,6233,9330,13187,21220,32767
; table volumes Amiga:
	;dc.l				$00*$c0, $00*$c0, $00*$c0, $00*$c0, $01*$c0, $02*$c0, $02*$c0, $04*$c0, $05*$c0, $08*$c0, $0B*$c0, $10*$c0, $18*$c0, $22*$c0, $37*$c0, $55*$c0

YM_DSP_forme_enveloppe_2:
; enveloppe descendante
	dc.l				32767,21220,13187,9330,6233,4570,3088,2260,1575,1155,774,580,377,265,161,62
; table volumes Amiga:
	;dc.l				$55*$c0, $37*$c0, $22*$c0, $18*$c0,$10*$c0,$0B*$c0,$08*$c0, $05*$c0,$04*$c0,$02*$c0,$02*$c0,$01*$c0,$00*$c0,$00*$c0,$00*$c0,$00*$c0

YM_DSP_forme_enveloppe_3:
; enveloppe zero
	dc.l				0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
YM_DSP_forme_enveloppe_4:
; enveloppe a 1
; table volumes Amiga:
	;dc.l				$55*$c0, $55*$c0,$55*$c0,$55*$c0,$55*$c0,$55*$c0,$55*$c0,$55*$c0,$55*$c0,$55*$c0,$55*$c0,$55*$c0,$55*$c0,$55*$c0,$55*$c0,$55*$c0
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


; digidrums
; en memoire DSP
YM_DSP_table_digidrums:
	.rept		16			; maxi 16 digidrums
		dc.l		0			; pointeur adresse du sample
		dc.l		0			; pointeur fin du sample 
	.endr

	.phrase	


;---------------------
; FIN DE LA RAM DSP
YM_DSP_fin:
;---------------------


SOUND_DRIVER_SIZE			.equ			YM_DSP_fin-DSP_base_memoire
	.print	"--- Sound driver code size (DSP): ", /u SOUND_DRIVER_SIZE, " bytes / 8192 ---"


	
	.68000


	.even
	.phrase
curseur_Y_min		.equ		8
curseur_x:	dc.w		0
curseur_y:	dc.w		curseur_Y_min

chaine_debut_init_YM7:			dc.b	"Init Coso",10,0
chaine_HZ_init_YM7:				dc.b	" Hz.",10,0
chaine_DG1_init_YM7:				dc.b	"There are ",0
chaine_DG2_init_YM7:			dc.b	" digidrums.",10,0
chaine_RAM_DSP:					dc.b	"DSP RAM available while running : ",0
chaine_replay_frequency:		dc.b	"Replay frequency : ",0
chaine_playing_YM7:				dc.b	"Now playing in ",0
chaine_playing_YM7_MONO:		dc.b	"mono.",10,10,0
chaine_playing_YM7_STEREO:		dc.b	"stereo.",10,10,0

chaine_replay_YM7:				
	dc.b	"A: "
chaine_replay_TA:
	dc.b	"T"
chaine_replay_NA:
	dc.b	"N"
chaine_replay_envA:
	dc.b	"E"
chaine_replay_SID_A:
	dc.b	"S"
chaine_replay_DG_A:
	dc.b	"D  "
	dc.b	"B: "
chaine_replay_TB:
	dc.b	"T"
chaine_replay_NB:
	dc.b	"N"
chaine_replay_envB:
	dc.b	"E"
chaine_replay_SID_B:
	dc.b	"S"
chaine_replay_DG_B:
	dc.b	"D  "
	dc.b	"C: "
chaine_replay_TC:
	dc.b	"T"
chaine_replay_NC:
	dc.b	"N"
chaine_replay_envC:
	dc.b	"E"
chaine_replay_SID_C:
	dc.b	"S"
chaine_replay_DG_C:
	dc.b	"D  "
chaine_replay_Sinus:
	dc.b	"s"
chaine_replay_Buzzer:
	dc.b	"Z ",0



couleur_char:				dc.b		25

	even
fonte:	
	.include	"fonte1plan.s"
	even
	
            .dphrase
stoplist:		dc.l	0,4




YM_table_frequences_Sinus_Sid_Amiga:		dc.w	566, 283, 141, 141


fichier_coso_depacked:
		;INCBIN		"airball.MUS"			; OK
		;.incbin		"C:\\Jaguar\\COSO\\fichiers mus\\COSO\\REPLI1.MUS"
		;.incbin		"C:\\Jaguar\\COSO\\fichiers mus\\NEW\\ENS6.MUS"
		 .incbin		"C:\\Jaguar\\COSO\\fichiers mus\\COSO\\SEVGATES.MUS"
		;.incbin		"C:\\Jaguar\\COSO\\fichiers mus\\COSO\\SEVGATE2.MUS"
		;.incbin		"C:\\Jaguar\\COSO\\seven5.mus"
		even
	
pointeur_buffer_trace_coso:		dc.l		buffer_trace_coso
buffer_trace_coso:
		incbin		"c:\jaguar\coso\dump_ST_registres_coso_airball.BIN"
fin_buffer_trace_coso:
		even


	.even

debut_ram_libre_DSP:		dc.l			YM_DSP_fin
debut_ram_libre:			dc.l			FIN_RAM

        .68000
		.dphrase
ob_liste_originale:           				 ; This is the label you will use to address this in 68K code
        .objproc 							   ; Engage the OP assembler
		.dphrase

        .org    ob_list_courante			 ; Tell the OP assembler where the list will execute
;
        branch      VC < 0, .stahp    			 ; Branch to the STOP object if VC < 0
        branch      VC > 265, .stahp   			 ; Branch to the STOP object if VC > 241
			; bitmap data addr, xloc, yloc, dwidth, iwidth, iheight, bpp, pallete idx, flags, firstpix, pitch
        bitmap      ecran1, 16, 26, nb_octets_par_ligne/8, nb_octets_par_ligne/8, 246-26,3
		;bitmap		ecran1,16,24,40,40,255,3
        jump        .haha
.stahp:
        stop
.haha:
        jump        .stahp
		
		.68000
		.dphrase
fin_ob_liste_originale:




	.bss
	.phrase
DEBUT_BSS:

YM_registres_Coso:			ds.b		14

	.phrase
frequence_Video_Clock:					ds.l				1
frequence_Video_Clock_divisee :			.ds.l				1

YM_nombre_de_frames_totales:			ds.l				1
YM_frequence_replay:					ds.l				1
	.phrase

YM_pointeur_origine_ymdata:		ds.l		1
YM_frequence_predivise:			ds.l		1
	.phrase

_50ou60hertz:			ds.l	1
ntsc_flag:				ds.w	1
a_hdb:          		ds.w   1
a_hde:          		ds.w   1
a_vdb:          		ds.w   1
a_vde:          		ds.w   1
width:          		ds.w   1
height:         		ds.w   1
taille_liste_OP:		ds.l	1
vbl_counter:			ds.l	1

            .dphrase
ecran1:				ds.b		320*256				; 8 bitplanes

FIN_RAM:


