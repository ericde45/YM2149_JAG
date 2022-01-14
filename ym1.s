; fantasia main, sid sur le B, rien actuellement
; pourri des le debut, SID B !


; YM7 sur Jaguar
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

; 	- passer le noise à la formule : seed = seed * 1103515245 + 12345;

;	- init Sinus Sid
;	- gérer Sinus Sid = 1 voie séparée

;	OK - replay à fréquence variable
;	- autoriser I2S pendant le timer 1


;	- simplifier la lecture des effets : BTST sur 1 seul registre
;	- with values decreasing from 8000 to zero. This will avoid a loud click on start up
;	- dans routine I2S : utiliser 3 registres pour les 3 volumes + registre sur table des volumes
;	- forcer pointeur sur volume pour digidrums dans Timer 1
;	- stéréo !!!! : placer 1 voie 100% a gauche, 1 voie 100% a droite, et 1 voie 60% a gauche/40% a droite
;	- placer la copie des 2 samples en tout debut de routine interruption I2S ( vue la variabilité liée à la génération du Noise )

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


	include	"jaguar.inc"

; algo de la routine qui genere les samples
; 3 canaux : increment onde carrée * 3 , increment noise, volume voie * 3 , increment enveloppe


DSP_DEBUG			.equ			0
DSP_DEBUG_T1		.equ			0
DSP_DEBUG_BUZZER	.equ			0
YM_avancer			.equ			1									; 0=on avance pas / 1=on avance
YM_position_debut_dans_musique		.equ		0
YM_Samples_SID_en_RAM_DSP			.equ		1						; 0 = samples SID en RAM 68000 / 1 = samples SID en RAM DSP.
DSP_random_Noise_generator_method	.equ		3						; algo to generate noise random number : 1 & 3 are OK

	
DSP_Audio_frequence					.equ			32000				; 22300=>17 => 23082 / 
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

DSP_STACK_SIZE	equ	32	; long words
DSP_USP			equ		(D_ENDRAM-(4*DSP_STACK_SIZE))
DSP_ISP			equ		(DSP_USP-(4*DSP_STACK_SIZE))

.opt "~Oall"

.text
.68000
;.noclear

	;move.l		#$1234,EDZ_compteur_reset_offset_entier_voie_A

	lea			buffer_de_debug,a0
	move.l		a0,pointeur_buffer_de_debug

	move.l		#INITSTACK, sp	
	move.w		#$06C7, VMODE			; 320x256
	move.w		#$100,JOYSTICK

; clear BSS

	lea			DEBUT_BSS,a0
	lea			$200000,a1
	moveq		#0,d0
	
boucle_clean_BSS:
	move.b		d0,(a0)+
	cmp.l		a0,a1
	bne.s		boucle_clean_BSS


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

	
                               
; apres copie on init le YM7
	bsr			YM_init_ym7



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
	lea			buffer_de_debug,a0
	move.l		pointeur_buffer_de_debug,a1


	move.l		YM_DSP_pointeur_sample_SID_voie_C,d0
;	cmp.l		#0,d0
;	beq.s		ok_toto
	
; $40B8
	move.l		EDZTMP1,d1
	move.l		YM_frequence_replay,d2
	;move.l		YM_DSP_increment_sample_SID_voie_C,d1 
	;move.l		YM_DSP_offset_sample_SID_voie_C,d2
	move.l		YM_DSP_taille_sample_SID_voie_C,d3
	move.l		YM_DSP_registre10,d4
	move.l		YM_DSP_pointeur_sur_table_infos_samples_SID,d5
	move.l		YM_DSP_volC,d6
	move.l		YM_DSP_pointeur_sur_source_du_volume_C,d7
; $40DA
	nop
	
;ok_toto:

	
	bra			toto

	stop		#$2700



;----------------------------------------------------
YM_init_ym7:
; tout le long de l'init D6=YM_nb_registres_par_frame

	lea			fichier_ym7,A0
	
	lea			12(a0),a0					; on saute YM7! et Leonard! = +12
	lea			4(a0),a0					; +4 : 0202

	move.l		(a0)+,YM_nombre_de_frames_totales					; .L=nombre de frames => c05756
	moveq		#0,d0
	move.w		(a0)+,d0
	move.l		d0,YM_frequence_replay							; .w=frequence du replay ( 50 hz ?)
	move.w		(a0)+,d0
	move.w		d0,YM_flag_effets_sur_les_voies					; .w= effet sur la voie. A=bit 0, B=bit 1, C=bit 2

	moveq.l		#14,d6										; nb d'octet par frame = 14 de base = 14 registres
	moveq		#1,d1

	btst		#0,d0						; effet sur voie A ?
	beq.s		YM_init_ym7_pas_effet_sur_voie_A
	addq.l		#2,d6						; + 2 registres à lire lors du replay
	move.l		d1,YM_flag_effets_voie_A
YM_init_ym7_pas_effet_sur_voie_A:

	btst		#1,d0						; effet sur voie B ?
	beq.s		YM_init_ym7_pas_effet_sur_voie_B
	addq.l		#2,d6						; + 2 registres à lire lors du replay
	move.l		d1,YM_flag_effets_voie_B
YM_init_ym7_pas_effet_sur_voie_B:

	btst		#2,d0						; effet sur voie C ?
	beq.s		YM_init_ym7_pas_effet_sur_voie_C
	addq.l		#2,d6						; + 2 registres à lire lors du replay
	move.l		d1,YM_flag_effets_voie_C
YM_init_ym7_pas_effet_sur_voie_C:

	btst		#4,d0						; effet D Sinus Sid ?
	beq.s		YM_init_ym7_pas_effet_Sinus_Sid
	addq.l		#1,d6						; + 2 registres à lire lors du replay
	move.l		d1,YM_flag_effets_Sinus_Sid
YM_init_ym7_pas_effet_Sinus_Sid:

	move.w		d6,YM_nb_registres_par_frame

	moveq		#0,d7
; debut init DG
	move.w		(a0)+,d7								; .w=nombre de digidrums
	beq.s		YM_pas_de_digidrums						; => pas de digidrums

; ----------------------- Init Digidrums -------------------------
; D7 = NB de digidrums
; structure entete digidrums :
;	- nb octets du sample
;	- multiplicateur / nb de fois = 1 pour digidrums

	lea			YM_DSP_table_digidrums,a2
	move.l		d7,d6					; D6=nb samples DG
	lsl.l		#2,d6					; * 4 octets 
	lea			(a0,d6.w),a1			; A1 = pointe sur debut samples DG

	subq.l		#1,d7

YM_init_ym7_boucle_table_digidrums:
	move.l		a1,d2
	lsl.l		#8,d2						; pré shiftés de 11 bits
	lsl.l		#3,d2						; pré shiftés de 11 bits
	move.l		d2,(a2)+					; stocke le pointeur sur le sample DG dans la table, pré shiftés de 11 bits
	moveq		#0,d0
	move.w		(a0)+,d0					; taille du sample
	add.l		d0,a1						; pointe sur le sample suivant
	move.l		a1,d2
	lsl.l		#8,d2						; pré shiftés de 11 bits
	lsl.l		#3,d2						; pré shiftés de 11 bits
	move.l		d2,(a2)+					; stocke l'adresse de la fin du sample DG dans la table , pré shiftés de 11 bits

	addq.l		#2,a0						; saute le nb multiplicateurs

	dbf			d7,YM_init_ym7_boucle_table_digidrums

	move.l		a1,a0						; on est après les samples DG

; arrondit A0:
	move.l		a0,d0
	btst		#0,d0
    beq.s		YM_init_ym7_OK_arrondit_DG
    addq.l 		#1,a0
YM_init_ym7_OK_arrondit_DG:	
; ----------------------- Fin Init DG -------------------------


	
YM_pas_de_digidrums:
; debut init SID 
	move.w		(a0)+,d7								; .w= nombre de SID à fabriquer
	beq.s		YM_pas_de_SID						; => pas de SID

; TODO : gérer les SID
; - une table qui pointe sur le début du sample SID, fin du sample SID, en RAM normale ( utilisée une fois par replay)
; - un sample par SID : .L * taille du SID, volume converti : RAM DSP ( lue en I2S )

; d7=nb de samples SID

	moveq		#0,d1
	move.l		d7,d6
	subq.l		#1,d6
	move.l		a0,a3

YM_init_boucle_calcul_taille_samples_SID:
	move.w		(a0)+,d0			; taille du sample SID
	addq.l		#2,a0				; on saute par dessus la repetition
	add.l		d0,d1
	dbf			d6,YM_init_boucle_calcul_taille_samples_SID
	
; d1 = taille totales des samples
; * 4 * 2 = *8 pour la RAM normale
; * 4 pour la RAM DSP

	move.l		d7,d0				; d7=nb samples
	;addq.l		#1,d0				; passage en pointeur+taille // +1 pour le pointeur de fin du dernier sample SID
	lsl.l		#3,d0				; * 8 = *4 *2 = pointeur adresse sample, taille
	.if			YM_Samples_SID_en_RAM_DSP=1
		bsr			YM_malloc_DSP
	.endif
	.if			YM_Samples_SID_en_RAM_DSP=0
		bsr			YM_malloc
	.endif	
	move.l		d0,a1				; A1=pointeur table 
	move.l		d0,YM_DSP_pointeur_sur_table_infos_samples_SID

	move.l		d1,d0
	lsl.l		#2,d0				; *4 pour la RAM DSP
	.if			YM_Samples_SID_en_RAM_DSP=1
		bsr			YM_malloc_DSP
	.endif
	.if			YM_Samples_SID_en_RAM_DSP=0
		bsr			YM_malloc
	.endif
	move.l		d0,a2				; A1=pointeur sur buffer samples SID en RAM DSP
	move.l		d0,YM_DSP_pointeur_sur_samples_SID_ram_DSP

	lea			YM_DSP_table_de_volumes,a4
	
	move.l		d7,d6				; d6=nb de SIDs
	subq.l		#1,d6

YM_init_boucle_copie_un_sample_SID_entier:
	moveq		#0,d1
	move.w		(a3)+,d1			; d1 = taille du sample
	addq.l		#2,a3				; passe par dessus la repetition

	move.l		a2,d4
	move.l		d4,(a1)+			; pointeur debut de sample SID
	swap		d1
	move.l		d1,(a1)+			; taille du sample SID en 16:16
	swap		d1
	
	subq.l		#1,d1				; pour le DBF


YM_init_boucle_copie_octets_un_sample_SID:	
	moveq		#0,d0
	move.b		(a0)+,d0			; octet de volume du sample SID
	
	lsl.l		#2,d0				; * 4
	move.l		(a4,d0.w),d0		; d0=volume en 16 bits	
	move.l		d0,(a2)+			; dans la RAM DSP
	dbf			d1,YM_init_boucle_copie_octets_un_sample_SID

	dbf			d6,YM_init_boucle_copie_un_sample_SID_entier

; saute les vmax qui sont à la suite  = nb de SID

	add.l		d7,a0

; arrondit A0:
	move.l		a0,d0
	btst		#0,d0
    beq.s		YM_init_ym7_OK_arrondit_SID
    addq.l 		#1,a0
YM_init_ym7_OK_arrondit_SID:		



YM_pas_de_SID:
; debut init 	Buzzer
	moveq		#0,d7
	move.w		(a0)+,d7									; valeur dans le fichier YM7 = nombre de buzzer ( apres entete, digidrums et table SID )
	beq.s		YM_pas_de_Buzzer

	move.l		#1,YM_flag_effets_Buzzer

; TODO : gérer les buzzer
; - une table de .L : ( pointeur sur enveloppes buzzer, taille de l'enveloppe buzzer ) * nb env buzzer
; - une table de .L : pour chaque env-buzzer : pointeurs sur enveloppe * taille de l'env-buzzer
; A0 pointe sur : .B / N*taille de l'enveloppe buzzer, suivi par .B / N*nb repetitions de chaque env buzzer
;				suivi par .B / X* numéro d'enveloppe entre 0 et 15

; - une table de .L : ( pointeur sur enveloppes buzzer, taille de l'enveloppe buzzer ) * nb env buzzer : 2*.L
	move.l		d7,d0
	lsl.l		#3,d0			; * 8 
	.if			YM_Samples_SID_en_RAM_DSP=1
		bsr			YM_malloc_DSP
	.endif
	.if			YM_Samples_SID_en_RAM_DSP=0
		bsr			YM_malloc
	.endif
	move.l		d0,a4
	move.l		d0,YM_DSP_pointeur_sur_table_des_pointeurs_env_Buzzer

; - une table de .L : pour chaque env-buzzer : pointeurs sur enveloppes * taille de l'env-buzzer
	move.l		d7,d0			; D0=nb env-buzzer
	moveq		#0,d1			;D1=taille totale des env-buzzer
	
	subq.l		#1,d0

	move.l		a0,a3

YM_init_ym7_calcul_taille_table_pointeurs_env_buzzer:
	moveq		#0,d2
	move.b		(a0)+,d2		; taille d'une env-buzzer
	add.l		d2,d1
	dbf			d0,YM_init_ym7_calcul_taille_table_pointeurs_env_buzzer

	move.l		d1,d0			;D0=taille totale des env-buzzer
	lsl.l		#2,d0			; *4
	.if			YM_Samples_SID_en_RAM_DSP=1
		bsr			YM_malloc_DSP
	.endif
	.if			YM_Samples_SID_en_RAM_DSP=0
		bsr			YM_malloc
	.endif	

	move.l		a3,a0		; A0=au début des tailles
	add.l		d7,a3		; saute les N tailles
	add.l		d7,a3		; saute les N répétitions

	move.l		d7,d6		;D6=nb env-buzzer
	subq.l		#1,d6		; -1 pour dbf

	move.l		d0,a1		; A1=pointeur sur liste des env des env-buzzer

	lea			YM_DSP_liste_des_enveloppes,a2

YM_init_ym7_boucle_remplit_table_pointeurs_sur_env_buzzer:
	moveq		#0,d2
	move.b		(a0)+,d2	; D2=taille de l'env-buzzer
	
	move.l		a1,(a4)+	; stocke pointeur sur env-buzzer
	swap		d2			; passage en 16:16
	move.l		d2,(a4)+	; taille de l'env-buzzer
	swap		d2		; retour valeur normale

	
	subq.l		#1,d2
YM_init_ym7_boucle_copie_adresse_env_pour_buzzer:	
	moveq		#0,d1
	move.b		(a3)+,d1	; numero de l'env
	lsl.l		#2,d1		; *4
	move.l		(a2,d1.w),d4		; D4=pointeur sur l'env
	move.l		d4,(a1)+			; stocke le pointeur sur l'env
	dbf			d2,YM_init_ym7_boucle_copie_adresse_env_pour_buzzer
	
	dbf			d6,YM_init_ym7_boucle_remplit_table_pointeurs_sur_env_buzzer

	move.l		a3,a0

; arrondit A0:
	move.l		a0,d0
	btst		#0,d0
    beq.s		YM_init_ym7_OK_arrondit_Buzzer
    addq.l 		#1,a0
YM_init_ym7_OK_arrondit_Buzzer:		



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
	sub.l		#YM_position_debut_dans_musique,d0
	move.l		d0,PSG_compteur_frames_restantes
	
	move.w		(a0)+,d0
	move.l		d0,YM_ecart_entre_frames_dernier_bloc		; ecart entre les frames pour le dernier blocs / format YM7 = .w= ?   par exemple 0C0E   ( stocké en C05784
	moveq		#0,d0
	move.w		(a0)+,d0										; nombre de bloc compressés / format YM7 = .w= ? de 1 à 70 / par exemple 0004	( stocké en C0576A ) = nombre de blocs de donn�es compress�es ?
	move.l		d0,YM_nombre_de_blocs_lZ4

; A0=pointeur sur les tailles des blocs
	move.l		a0,YM_LZ4_pointeur_tailles_des_blocs

	lea			YM_tableau_des_blocs_decompresses,a2
	moveq		#0,d1
	move.w		YM_ecart_entre_frames_blocs,d1

YM_init_ym7_boucle_creation_liste_des_blocs_decompresses_copie_ecarts:
	addq.l		#4,a2												; saute par dessus le pointeur sur le bloc
	subq		#1,d0
	beq.s		YM_init_ym7_creation_liste_des_blocs_decompresses_copie_ecarts_dernier_bloc
	
	move.l		d1,(a2)+
	bra.s		YM_init_ym7_boucle_creation_liste_des_blocs_decompresses_copie_ecarts

YM_init_ym7_creation_liste_des_blocs_decompresses_copie_ecarts_dernier_bloc:
	move.l		YM_ecart_entre_frames_dernier_bloc,d1
	move.l		d1,(a2)+


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
	move.l		YM_nombre_de_blocs_lZ4,d5
	move.l		d5,d4
	add.w		d5,d5						; nb bloc * 2
	lea			(a4,d5.w),a5				; A5 = pointeur sur les donnees du bloc 1

	lea			YM_tableau_des_blocs_decompresses,a2

YM_init_ym7_boucle_decomp_blocs:
	move.l		a1,(a2)+					; pointeur sur bloc decompresse
	addq.l		#4,a2						; saute le nb de frame du bloc .l
	
	moveq		#0,d0
	move.w		(a4)+,d0					; taille du bloc compresse

	move.l		a5,a0						; A0=pointeur sur packed bloc
	add.l		d0,a5						; A5 = pointe sur bloc LZ4 suivant

; input :
; / a0.l = packed buffer 
; / a1.l = output buffer 
; /  d0.l = LZ4 packed block size (in bytes)
	bsr		lz4_depack

; / a1=fin du bloc decompresse


	subq		#1,d4
	bgt			YM_init_ym7_boucle_decomp_blocs

	lea			YM_tableau_des_blocs_decompresses,a2
	move.l		(a2),a1						; A1 = pointeur sur premier bloc decompresse
	move.l		a1,YM_pointeur_actuel_ymdata
	
	moveq		#0,d0
	move.l		d0,YM_numero_bloc_en_cours
	
	.if	 YM_position_debut_dans_musique<>0
	move.l	#YM_position_debut_dans_musique,d0
	add.l	d0,a1
	move.l		a1,YM_pointeur_actuel_ymdata
	.endif
	
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
; bank 0 : 
; R28/R29/R30/R31
; +
; R18/R19/R20/R21/R22/R23/R24/R25/R26/R27 - R19 dispo
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
; gerer le buzzer
	.if		DSP_DEBUG_BUZZER=0
	movei	#YM_DSP_increment_envbuzzer_en_cours,R27
	movei	#YM_DSP_replay_sample_pas_de_Buzzer,R26
	load	(R27),R25
	cmpq	#0,R25
	jump	eq,(R26)
	nop
; on a un Buzzer

	movei	#YM_DSP_pointeur_sur_envbuzzer_en_cours,R24
	load	(R24),R23				; R23=pointe sur la premiere env du Buzzer
	movei	#YM_DSP_offset_envbuzzer_en_cours,R22
	load	(R22),R21				; offset en 16:16
	movei	#YM_DSP_taille_envbuzzer_en_cours,R20
	add		R25,R21					; offset=offset + increment en 16:16
	load	(R20),R20				; R20 = taille en 16:16
	cmp		R20,R21
	jr		mi,YM_DSP_replay_Buzzer_pas_de_bouclage_envbuzzer
	nop
	shlq	#16,R21
	shrq	#16,R21					; 16 bits du haut de R21 = 0
	
	movei	#YM_DSP_offset_enveloppe,R24
	movei	#$FFF00000,R25
	store	R25,(R24)

YM_DSP_replay_Buzzer_pas_de_bouclage_envbuzzer:
	move	R21,R19					; R19=offset en 16:16
	shrq	#16,R19					; 16 bits du haut de R19 = partie entiere de l'offset
	store	R21,(R22)				; stocke nouvel offset envbuzzer en 16:16
	shlq	#2,R19					; *4 pour pointeur sur env
	add		R19,R23
	load	(R23),R24				; R21 = pointe sur l'enveloppe en cours

	
	jr		YM_DSP_replay_sample_gere_env
	nop
	.endif
	

;--------------------------
; gerer l'enveloppe
; - incrementer l'offset enveloppe
; partie entiere 16 bits : virgule 16 bits
; partie entiere and %1111 = position dans la sous partie d'enveloppe
; ( ( partie entiere >> 4 ) and %1 ) << 2 = pointeur sur la sous partie d'enveloppe


; si positif, limiter, masquer, à 11111 ( 5 bits:16 )

YM_DSP_replay_sample_pas_de_Buzzer:
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


;--------------------------
; gerer les SID
;--------------------------


; ----- gerer SID canal A
	movei	#YM_DSP_pointeur_sample_SID_voie_A,R27					
	movei	#YM_DSP_replay_sample_pas_de_SID_voie_A,R24
	load	(R27),R26				; R26 = pointeur sur sample SID voie A
	cmpq	#0,R26
	jump	eq,(R24)
; on a un SID voie A
	addq	#4,R27					; R27 pointe sur YM_DSP_increment_sample_SID_voie_A
	load	(R27),R21				; R1=increment en 16:16
	addq	#4,R27					; R27 pointe sur YM_DSP_offset_sample_SID_voie_A
	load	(R27),R23				; R23 = YM_DSP_offset_sample_SID_voie_A : offset : 16:16 / entier:virgule
	move	R23,R22					; R22 = offset 16:16
	addq	#4,R27					; R27 pointe sur YM_DSP_taille_sample_SID_voie_A
	shrq	#16,R23					; partie entiere de l'offset 16:16
	shlq	#2,R23					; *4 / .L par valeur de volume de sample SID
	add		R23,R26					; pointeur sample sid + ( partie entiere de l'offset * 4))
	load	(R26),R25				; R25=volume du SID
	movei	#YM_DSP_volA,R24
	movei	#YM_DSP_pointeur_sur_source_du_volume_A,R23
	store	R25,(R24)				; met le volume du SID dans volume du canal
	store	R24,(R23)
	
	
; R22=offset 16:16
; R21=increment 16:16

	add		R21,R22					; offset = offset + increment // 16:16
	load	(R27),R23				; R23=taille sample en 16:16 // YM_DSP_taille_sample_SID_voie_A
	cmp		R23,R22					; comparaison taille et offset actuel, en 16:16
	jr		mi,YM_DSP_replay_sample_SID_pas_de_bouclage_canal_A
	nop
; ici on boucle
; mise a zero de l'offset
	movei	#$0000FFFF,R21
	and		R21,R22					; 16 bits du haut de R22 = 0
YM_DSP_replay_sample_SID_pas_de_bouclage_canal_A:
	subq	#4,R27					; on ramene R27 sur offset
	or		R27,R27
	store	R22,(R27)

YM_DSP_replay_sample_pas_de_SID_voie_A:

; ----- gerer SID canal B
	movei	#YM_DSP_pointeur_sample_SID_voie_B,R27					
	movei	#YM_DSP_replay_sample_pas_de_SID_voie_B,R24
	load	(R27),R26				; R26 = pointeur sur sample SID voie A
	cmpq	#0,R26
	jump	eq,(R24)
; on a un SID voie B
	addq	#4,R27					; R27 pointe sur YM_DSP_increment_sample_SID_voie_A
	load	(R27),R21				; R1=increment en 16:16
	addq	#4,R27					; R27 pointe sur YM_DSP_offset_sample_SID_voie_A
	load	(R27),R23				; R23 = YM_DSP_offset_sample_SID_voie_A : offset : 16:16 / entier:virgule
	move	R23,R22					; R22 = offset 16:16
	addq	#4,R27					; R27 pointe sur YM_DSP_taille_sample_SID_voie_A
	shrq	#16,R23					; partie entiere de l'offset 16:16
	shlq	#2,R23					; *4 / .L par valeur de volume de sample SID
	add		R23,R26					; pointeur sample sid + ( partie entiere de l'offset * 4))
	load	(R26),R25				; R25=volume du SID
	movei	#YM_DSP_volB,R24
	movei	#YM_DSP_pointeur_sur_source_du_volume_B,R23
	store	R25,(R24)				; met le volume du SID dans volume du canal
	store	R24,(R23)
	
	
; R22=offset 16:16
; R21=increment 16:16

	add		R21,R22					; offset = offset + increment // 16:16
	load	(R27),R23				; R23=taille sample en 16:16 // YM_DSP_taille_sample_SID_voie_A
	cmp		R23,R22					; comparaison taille et offset actuel, en 16:16
	jr		mi,YM_DSP_replay_sample_SID_pas_de_bouclage_canal_B
	nop
; ici on boucle
; mise a zero de l'offset
	movei	#$0000FFFF,R21
	and		R21,R22					; 16 bits du haut de R22 = 0
YM_DSP_replay_sample_SID_pas_de_bouclage_canal_B:
	subq	#4,R27					; on ramene R27 sur offset
	or		R27,R27
	store	R22,(R27)

YM_DSP_replay_sample_pas_de_SID_voie_B:

; ----- gerer SID canal C
	movei	#YM_DSP_pointeur_sample_SID_voie_C,R27					
	movei	#YM_DSP_replay_sample_pas_de_SID_voie_C,R24
	load	(R27),R26				; R26 = pointeur sur sample SID voie A
	cmpq	#0,R26
	jump	eq,(R24)
; on a un SID voie C
	addq	#4,R27					; R27 pointe sur YM_DSP_increment_sample_SID_voie_A
	load	(R27),R21				; R1=increment en 16:16
	addq	#4,R27					; R27 pointe sur YM_DSP_offset_sample_SID_voie_A
	load	(R27),R23				; R23 = YM_DSP_offset_sample_SID_voie_A : offset : 16:16 / entier:virgule
	move	R23,R22					; R22 = offset 16:16
	addq	#4,R27					; R27 pointe sur YM_DSP_taille_sample_SID_voie_A
	shrq	#16,R23					; partie entiere de l'offset 16:16
	shlq	#2,R23					; *4 / .L par valeur de volume de sample SID
	add		R23,R26					; pointeur sample sid + ( partie entiere de l'offset * 4))
	load	(R26),R25				; R25=volume du SID
	movei	#YM_DSP_volC,R24
	movei	#YM_DSP_pointeur_sur_source_du_volume_C,R23
	store	R25,(R24)				; met le volume du SID dans volume du canal
	store	R24,(R23)
	
	
; R22=offset 16:16
; R21=increment 16:16

	add		R21,R22					; offset = offset + increment // 16:16
	load	(R27),R23				; R23=taille sample en 16:16 // YM_DSP_taille_sample_SID_voie_A
	cmp		R23,R22					; comparaison taille et offset actuel, en 16:16
	jr		mi,YM_DSP_replay_sample_SID_pas_de_bouclage_canal_C
	nop
; ici on boucle
; mise a zero de l'offset
	movei	#$0000FFFF,R21
	and		R21,R22					; 16 bits du haut de R22 = 0
YM_DSP_replay_sample_SID_pas_de_bouclage_canal_C:
	subq	#4,R27					; on ramene R27 sur offset
	store	R22,(R27)

YM_DSP_replay_sample_pas_de_SID_voie_C:


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
	or		R25,R25
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



; sans stereo : R20=A / R23=B / R22=C
	shrq	#1,R20					; quand volume maxi = 32767
	shrq	#1,R23
	shrq	#1,R22
	add		R23,R20					; R20 = R20=canal A + R23=canal B
	movei	#32768,R27
	add		R22,R20					; + canal C
	movei	#L_I2S,r26
	sub		R27,R20					; resultat signé sur 16 bits
	store	r20,(r26)				; write right channel
	addq	#4,R26
	store	r20,(r26)				; write left channel


; R22 = sample canal C
;	movei	#32768,R27
;	add		R23,R20					; R20 = R20=canal A + R23=canal B
;	movei	#L_I2S,r26
;	sub		R27,R22					; canal C / resultat signé sur 16 bits
;	sub		R27,R20					; resultat signé sur 16 bits
; envoi du sample

;	store	r22,(r26)				; write right channel
;	addq	#4,R26
;	store	r20,(r26)				; write left channel

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
DSP_LSP_routine_interruption_Timer1:
	.if		DSP_DEBUG_T1
; change la couleur du fond
	movei	#$077,R26
	movei	#BG,r27
	storew	r26,(r27)
	.endif

;-------------------------------------------------------------------------------------------------
; -------------------------------------------------------------------------------
; routine de lecture des registres YM
; bank 0 : 
; R28/R29/R30/R31
; +
; R0/R1/R2/R3/R4/R5/R6/R7/R8/R9/R10 + R14
; -------------------------------------------------------------------------------
	;-------------------------------------------------------------------------------------------------
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

	moveq		#%1111,R3
	move		R2,R4
	movei		#YM_DSP_registre8,R6
	movei		#YM_DSP_volE,R5
	store		R4,(R6)					; sauvegarde la valeur de volume sur 16, pour DG
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

	moveq		#%1111,R3
	move		R2,R4
	movei		#YM_DSP_registre9,R6
	movei		#YM_DSP_volE,R5
	store		R4,(R6)					; sauvegarde la valeur de volume sur 16, pour DG
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

	moveq		#%1111,R3
	move		R2,R4
	movei		#YM_DSP_registre10,R6
	movei		#YM_DSP_volE,R5
	store		R4,(R6)					; sauvegarde la valeur de volume sur 16, pour DG
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

DSP_lecture_registre13_pas_env:


;--------------------------------
; gestion des effets par voie
; ------- effet sur voie A ?
	movei		#YM_flag_effets_voie_A,R3
	load		(R3),R3
	movei		#DSP_lecture_registre_effet_voie_A_pas_d_effet,R4
	cmpq		#0,R3
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
; on teste si on a un SID
	movei		#DSP_lecture_registre_effet_voie_A_pas_de_SID,R4
	btst		#6,R2					; bit6 = bit 14 sur 16 bits = flag SID
	jump		eq,(R4)
	nop
; routine SID
; il faut remplir :
;	- YM_DSP_pointeur_sample_SID_voie_A:					dc.l		0			; partie entiere
; vient de registre 8
;	- YM_DSP_pointeur_sample_SID_virgule_voie_A:			dc.l		0			; partie virgule sur 32 bits
; = 0
;	- YM_DSP_increment_sample_SID_voie_A:					dc.l		0
; doit etre calculé sur 32 bits
;	- YM_DSP_pointeur_fin_sample_SID_voie_A:				dc.l		0
; lu à partir du registre 8
;	- YM_DSP_pointeur_debut_sample_SID_voie_A:				dc.l		0			; adresse du debut du sample canal A
; = égal à YM_DSP_pointeur_sample_SID_voie_A

; calculer la fréquence => increment de replay : sur la base du MFP : ymu32 tmp = timerFreq * ((1<<31) / replayFrequency);   // timerFreq = tmpFreq = 2457600L / (prediv*diviseur);
; 16 bits : ( 2457600L / (prediv*diviseur) ) * 16 bits : ((1<<31) / replayFrequency)
; (1<<31) / replayFrequency) = DSP_UN_sur_frequence_de_replay_reelle_I2S
; 

; on arrive avec R2=index prediv  et  R3=diviseur // R8=increment à conserver // R1=source des données à conserver

; R7=raz de virgule ou pas
	movei		#YM_DSP_offset_sample_SID_voie_A,R10
	load		(R10),R7					; offset 16:16
	btst		#5,R2					; bit5 = bit 13 sur 16 bits = flag reset virgule SID // si égal à 1 => virgule=0
	jr			eq,DSP_lecture_registre_effet_voie_A_pas_de_reset_virgule_SID
	nop
	movei		#$FFFF0000,R5
	and			R5,R7					; virgule remise a zero
DSP_lecture_registre_effet_voie_A_pas_de_reset_virgule_SID:


	moveq		#%111,R5					; mask
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
	shlq		#YM_DSP_precision_virgule_SID,R5						; 16 bits de virgule
	div			R4,R5						; R5=increment digidrum=(frequence du MFP / ( prediviseur * diviseur ) ) / frequence_de_replay_reelle_I2S en 16:16
	movei		#YM_DSP_taille_sample_SID_voie_A,R0
; si nouvelle taille = taille actuelle, on ne remet pas l'offset à zéro
	load		(R0),R9						; R9 = ancienne taille

; R5 = increment / R7=offset / RO=pointeur sur YM_DSP_taille_sample_SID_voie_A
	or			R5,R5

; recuperer le pointeur sur la table des samples SID ( adresse dynamique crée lors du malloc )
	movei		#YM_DSP_pointeur_sur_table_infos_samples_SID,R2
	movei		#YM_DSP_registre8,R6
	load		(R2),R2							; pointeur sur la liste des samples SID au format 16 bits sur un .L
	load		(R6),R6						; R6 = numero du sample SID
	shlq		#3,R6						; *8 = 4+4 = .L+.L
	add			R6,R2
	load		(R2),R3						; R3 = pointeur adresse debut sample SID
	addq		#4,R2
	load		(R2),R4						; R4 = taille du SID
	
	cmp			R4,R9
	jr			eq,DSP_lecture_registre_effet_voie_A_tailles_egales_pas_de_raz_offset
; si changement de taille, il faut mettre l'offset à zero
	nop
	;movei		#EDZ_compteur_reset_offset_entier_voie_A,R2
	;load		(R2),R6
	;addq		#1,R6
	;or			R6,R6
	;store		R6,(R2)
	
	shlq		#16,R7
	shrq		#16,R7						; 16 bits du haut = 0 / offset entier = 0
	store		R4,(R0)						; stocke la nouvelle taille en 16:16


DSP_lecture_registre_effet_voie_A_tailles_egales_pas_de_raz_offset:

	store		R7,(R10)						; stocke YM_DSP_offset_sample_SID_voie_A
	movei		#YM_DSP_pointeur_sample_SID_voie_A,R0
	store		R3,(R0)						; stocke YM_DSP_pointeur_sample_SID_voie_A
	addq		#4,R0
	store		R5,(R0)						; stocke YM_DSP_increment_sample_SID_voie_A

; saute par dessus la routine SIDstop
	movei		#DSP_lecture_registre_effet_voie_A_pas_d_effet,R3
	jump		(R3)

DSP_lecture_registre_effet_voie_A_pas_de_SID:
; il faut faire un SIDstop voie A
	moveq		#0,R4
	movei		#YM_DSP_pointeur_sample_SID_voie_A,R2
	store		R4,(R2)				; stop le SID canal A


DSP_lecture_registre_effet_voie_A_pas_d_effet:

; -----------------------------
; ------- effet sur voie B ?

	movei		#YM_flag_effets_voie_B,R3
	load		(R3),R3
	movei		#DSP_lecture_registre_effet_voie_B_pas_d_effet,R4
	cmpq		#0,R3
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
; on teste si on a un SID
	movei		#DSP_lecture_registre_effet_voie_B_pas_de_SID,R4
	btst		#6,R2					; bit6 = bit 14 sur 16 bits = flag SID
	jump		eq,(R4)
	nop

; R7=raz de virgule ou pas
	movei		#YM_DSP_offset_sample_SID_voie_B,R10
	load		(R10),R7				; offset 16:16
	btst		#5,R2					; bit5 = bit 13 sur 16 bits = flag reset virgule SID // si égal à 1 => virgule=0
	jr			eq,DSP_lecture_registre_effet_voie_B_pas_de_reset_virgule_SID
	nop
	movei		#$FFFF0000,R5
	and			R5,R7					; virgule remise a zero
DSP_lecture_registre_effet_voie_B_pas_de_reset_virgule_SID:

	moveq		#%111,R5					; mask
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
	shlq		#YM_DSP_precision_virgule_SID,R5						; 16 bits de virgule
	div			R4,R5						; R5=increment digidrum=(frequence du MFP / ( prediviseur * diviseur ) ) / frequence_de_replay_reelle_I2S en 16:16
	movei		#YM_DSP_taille_sample_SID_voie_B,R0
; si nouvelle taille = taille actuelle, on ne remet pas l'offset à zéro
	load		(R0),R9						; R9 = ancienne taille

; R5 = increment / R7=offset / RO=pointeur sur YM_DSP_taille_sample_SID_voie_B
	or			R5,R5

; recuperer le pointeur sur la table des samples SID ( adresse dynamique crée lors du malloc )
	movei		#YM_DSP_pointeur_sur_table_infos_samples_SID,R2
	movei		#YM_DSP_registre9,R6
	load		(R2),R2							; pointeur sur la liste des samples SID au format 16 bits sur un .L
	load		(R6),R6						; R6 = numero du sample SID
	shlq		#3,R6						; *8 = 4+4 = .L+.L
	add			R6,R2
	load		(R2),R3						; R3 = pointeur adresse debut sample SID
	addq		#4,R2
	load		(R2),R4						; R4 = taille du SID

	cmp			R4,R9
	jr			eq,DSP_lecture_registre_effet_voie_B_tailles_egales_pas_de_raz_offset
; si changement de taille, il faut mettre l'offset à zero
	nop
		
	shlq		#16,R7
	shrq		#16,R7						; 16 bits du haut = 0 / offset entier = 0
	store		R4,(R0)						; stocke la nouvelle taille en 16:16


DSP_lecture_registre_effet_voie_B_tailles_egales_pas_de_raz_offset:

	store		R7,(R10)					; stocke YM_DSP_offset_sample_SID_voie_B
	movei		#YM_DSP_pointeur_sample_SID_voie_B,R0
	store		R3,(R0)						; stocke YM_DSP_pointeur_sample_SID_voie_B
	addq		#4,R0
	store		R5,(R0)						; stocke YM_DSP_increment_sample_SID_voie_B

; saute par dessus la routine SIDstop
	movei		#DSP_lecture_registre_effet_voie_B_pas_d_effet,R3
	jump		(R3)

DSP_lecture_registre_effet_voie_B_pas_de_SID:
; il faut faire un SIDstop voie B
	moveq		#0,R4
	movei		#YM_DSP_pointeur_sample_SID_voie_B,R2
	store		R4,(R2)				; stop le SID canal B


DSP_lecture_registre_effet_voie_B_pas_d_effet:



; -----------------------------
; ------- effet sur voie C ?
	movei		#YM_flag_effets_voie_C,R3
	load		(R3),R3
	movei		#DSP_lecture_registre_effet_voie_C_pas_d_effet,R4
	cmpq		#0,R3
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
; on teste si on a un SID
	movei		#DSP_lecture_registre_effet_voie_C_pas_de_SID,R4
	btst		#6,R2					; bit6 = bit 14 sur 16 bits = flag SID
	jump		eq,(R4)
	nop

; R7=raz de virgule ou pas
	movei		#YM_DSP_offset_sample_SID_voie_C,R10
	load		(R10),R7				; offset 16:16
	btst		#5,R2					; bit5 = bit 13 sur 16 bits = flag reset virgule SID // si égal à 1 => virgule=0
	jr			eq,DSP_lecture_registre_effet_voie_C_pas_de_reset_virgule_SID
	nop
	movei		#$FFFF0000,R5
	and			R5,R7					; virgule remise a zero
DSP_lecture_registre_effet_voie_C_pas_de_reset_virgule_SID:

	moveq		#%111,R5					; mask
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
	shlq		#YM_DSP_precision_virgule_SID,R5						; 16 bits de virgule
	div			R4,R5						; R5=increment digidrum=(frequence du MFP / ( prediviseur * diviseur ) ) / frequence_de_replay_reelle_I2S en 16:16
	movei		#YM_DSP_taille_sample_SID_voie_C,R0
; si nouvelle taille = taille actuelle, on ne remet pas l'offset à zéro
	load		(R0),R9						; R9 = ancienne taille

; R5 = increment / R7=offset / RO=pointeur sur YM_DSP_taille_sample_SID_voie_A
	or			R5,R5

; recuperer le pointeur sur la table des samples SID ( adresse dynamique crée lors du malloc )
	movei		#YM_DSP_pointeur_sur_table_infos_samples_SID,R2
	movei		#YM_DSP_registre10,R6
	load		(R2),R2							; pointeur sur la liste des samples SID au format 16 bits sur un .L
	load		(R6),R6						; R6 = numero du sample SID
	shlq		#3,R6						; *8 = 4+4 = .L+.L
	add			R6,R2
	load		(R2),R3						; R3 = pointeur adresse debut sample SID
	addq		#4,R2
	load		(R2),R4						; R4 = taille du SID

	cmp			R4,R9
	jr			eq,DSP_lecture_registre_effet_voie_C_tailles_egales_pas_de_raz_offset
; si changement de taille, il faut mettre l'offset à zero
	nop
	movei		#$0000FFFF,R9
	and			R9,R7					; partie entiere remise a zero

	;shlq		#16,R7
	;shrq		#16,R7						; 16 bits du haut = 0 / offset entier = 0
	store		R4,(R0)						; stocke la nouvelle taille en 16:16


DSP_lecture_registre_effet_voie_C_tailles_egales_pas_de_raz_offset:

	store		R7,(R10)					; stocke YM_DSP_offset_sample_SID_voie_B
	movei		#YM_DSP_pointeur_sample_SID_voie_C,R0
	store		R3,(R0)						; stocke YM_DSP_pointeur_sample_SID_voie_B
	addq		#4,R0
	store		R5,(R0)						; stocke YM_DSP_increment_sample_SID_voie_B

; saute par dessus la routine SIDstop
	movei		#DSP_lecture_registre_effet_voie_C_pas_d_effet,R3
	jump		(R3)

DSP_lecture_registre_effet_voie_C_pas_de_SID:
; il faut faire un SIDstop voie C
	moveq		#0,R4
	movei		#YM_DSP_pointeur_sample_SID_voie_C,R2
	store		R4,(R2)				; stop le SID canal C


DSP_lecture_registre_effet_voie_C_pas_d_effet:



; -----------------------------
; gestion effet Buzzer
	.if			DSP_DEBUG_BUZZER=0
	movei		#YM_flag_effets_Buzzer,R3
	load		(R3),R3
	movei		#DSP_lecture_registre_effet_Buzzer_pas_d_effet,R4
	cmpq		#0,R3
	jump		eq,(R4)
	;nop

; Buzzer stop
	moveq		#0,R0
	movei		#YM_DSP_increment_envbuzzer_en_cours,R3

	store		R0,(R3)

	loadb		(R1),R2						; octet 1 effet sur la voie : 8 bits du haut = R2=prediviseur
	add			R8,R1
	loadb		(R1),R9						; octet 2 effet sur la voie : 8 bits du bas = R9=diviseur
	add			R8,R1


	btst		#7,R2						; bit 15-8 : test bit 15 de la valeur lue : si =0 => on saute toute la partie mise en place du buzzer
	jump		eq,(R4)
	nop

; utilisés : R1/R2/R8/R9
; registre13=numero envbuzzer
	movei		#YM_DSP_registre13,R3
	movei		#YM_DSP_pointeur_sur_table_des_pointeurs_env_Buzzer,R4
	load		(R3),R3
	load		(R4),R4
	bclr		#7,R3						; clear le bit 7 $80
	shlq		#3,R3						; *8
	add			R3,R4
	load		(R4),R5						; R5=pointeur sur envbuzzer = liste d'env
	addq		#4,R4

	movei		#YM_DSP_offset_envbuzzer_en_cours,R3

	load		(R4),R6						; R6=taille envbuzzer

	load		(R3),R7						; R7 = offset envbuzzer 16:16

; utilisés : R1/R2/R3/R5/R6/R7/R8/R9
	btst		#6,R2						; bit 13-7
	jr			ne,DSP_lecture_registre_effet_Buzzer_clear_offset_envbuzzer_partie_entiere
	nop

	movei		#YM_DSP_taille_envbuzzer_en_cours,R4
	load		(R4),R0					; R0=taille Buzzer actuelle
	cmp			R0,R6					; ancienne taille = nouvelle taille ?
	jr			eq,DSP_lecture_registre_effet_Buzzer_PAS_de_clear_offset_envbuzzer_partie_entiere
	nop
DSP_lecture_registre_effet_Buzzer_clear_offset_envbuzzer_partie_entiere:
; remet a zero l'offset partie entiere de lecture de env buzzer
	movei		#$FFFF,R0
	store		R6,(R4)					; stocke la nouvelle taille
	and			R0,R7					; mets à zero la partie haute de l'offset envbuzzer 16:16
	
DSP_lecture_registre_effet_Buzzer_PAS_de_clear_offset_envbuzzer_partie_entiere:

	movei		#YM_DSP_pointeur_sur_envbuzzer_en_cours,R0
	store		R7,(R3)					; YM_DSP_offset_envbuzzer_en_cours
	store		R5,(R0)					; R5=pointeur sur envbuzzer = liste d'env / YM_DSP_pointeur_sur_envbuzzer_en_cours
	
; calculer increment : MFP en 16:16
; a partir de R2
; utilisés : R0/R1/R2/R8/R9	
	movei		#%111,R3			; on masque 11 bits
	addq		#4,R0					; on pointe sur YM_DSP_increment_envbuzzer_en_cours
	and			R3,R2					; R2=frequence sur 3 bits = prediviseur
	movei		#YM_DSP_table_prediviseur,R6
	shlq		#2,R2						; * 4 
	add			R2,R6						
	load		(R6),R6						; R6=prediviseur
	mult		R6,R9						; R9=prediviseur * diviseur
	movei		#YM_DSP_frequence_MFP,R5
	div			R9,R5						; frequence du MFP / ( prediviseur * diviseur )
	movei		#DSP_frequence_de_replay_reelle_I2S,R4
	load		(R4),R4
	or			R5,R5
	shlq		#YM_DSP_precision_virgule_envbuzzer,R5						; 16 bits de virgule
	div			R4,R5						; R5=increment digidrum=(frequence du MFP / ( prediviseur * diviseur ) ) / frequence_de_replay_reelle_I2S en 16:16
	store		R5,(R0)						; YM_DSP_increment_envbuzzer_en_cours



DSP_lecture_registre_effet_Buzzer_pas_d_effet:
	.endif
	
;---> precalculer les valeurs qui ne bougent pas pendant 1 VBL entiere	

; debug avec buffer pour voir les valeurs
	movei		#pointeur_buffer_de_debug,R0
	movei		#buffer_de_debug,R1
	store		R1,(R0)
	



	.if			YM_avancer=1
; avancer dans la lecture du YM7 a gérer !!	

	movei		#YM_pointeur_actuel_ymdata,R0
	load		(R0),R1
	addq		#1,R1
	movei		#PSG_compteur_frames_restantes,R2
	load		(R2),R3
	movei		#DSP_lecture_registres_player_VBL_YM7_pas_fin_du_bloc,R5
	subq		#1,R3
	
	;cmp			#0,R3
	jump		ne,(R5)
	nop
; fin du bloc en cours
	movei		#YM_numero_bloc_en_cours,R4
	load		(R4),R5
	addq		#1,R5					; numero du bloc en cours + 1
	movei		#YM_nombre_de_blocs_lZ4,R6
	load		(R6),R6
	cmp			R6,R5
	jr			ne,DSP_lecture_registres_player_VBL_YM7_pas_fin_de_tous_les_blocs
	nop
	moveq		#0,R5
	
DSP_lecture_registres_player_VBL_YM7_pas_fin_de_tous_les_blocs:
	store		R5,(R4)			; YM_numero_bloc_en_cours

; lire les infos du nouveau bloc
	movei		#YM_tableau_des_blocs_decompresses,R6
	shlq		#3,R5			; *8
	add			R5,R6
	load		(R6),R1			; pointeur sur le nouveau bloc
	addq		#4,R6
	load		(R6),R3			; nb frames du nouveau bloc
	movei		#PSG_ecart_entre_les_registres_ymdata,R5
	store		R3,(R5)
		
	
DSP_lecture_registres_player_VBL_YM7_pas_fin_du_bloc:
	store		R1,(R0)			; YM_pointeur_actuel_ymdata
	store		R3,(R2)			; PSG_compteur_frames_restantes


	.endif

	
	
	.if		DSP_DEBUG_T1
; change la couleur du fond
	movei	#$000,R26
	movei	#BG,r27
	storew	r26,(r27)
	.endif

;------------------------------------	
; return from interrupt Timer 1
	load	(r31),r28	; return address
	;bset	#10,r29		; clear latch 1 = I2S
	bset	#11,r29		; clear latch 1 = timer 1
	;bset	#12,r29		; clear latch 1 = timer 2
	bclr	#3,r29		; clear IMASK
	addq	#4,r31		; pop from stack
	addqt	#2,r28		; next instruction
	jump	t,(r28)		; return
	store	r29,(r30)	; restore flags


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
	movei	#EDZTMP1,R12
	store	R13,(R12)
	
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

; variables SID
YM_DSP_pointeur_sur_table_infos_samples_SID:	ds.l		1

YM_DSP_pointeur_sample_SID_voie_A:					dc.l		0		; pointeur fixe
YM_DSP_increment_sample_SID_voie_A:					dc.l		0		; increment en 16:16
YM_DSP_offset_sample_SID_voie_A:					dc.l		0		; offset : 16:16 / entier:virgule
YM_DSP_taille_sample_SID_voie_A:					dc.l		0		; taille en 16:16 ?

YM_DSP_pointeur_sample_SID_voie_B:					dc.l		0		; pointeur fixe
YM_DSP_increment_sample_SID_voie_B:					dc.l		0		; increment en 16:16
YM_DSP_offset_sample_SID_voie_B:					dc.l		0		; offset : 16:16 / entier:virgule
YM_DSP_taille_sample_SID_voie_B:					dc.l		0		; taille en 16:16 ?

YM_DSP_pointeur_sample_SID_voie_C:					dc.l		0		; pointeur fixe
YM_DSP_increment_sample_SID_voie_C:					dc.l		0		; increment en 16:16
YM_DSP_offset_sample_SID_voie_C:					dc.l		0		; offset : 16:16 / entier:virgule
YM_DSP_taille_sample_SID_voie_C:					dc.l		0		; taille en 16:16 ?

; variables Buzzer
YM_DSP_pointeur_sur_envbuzzer_en_cours:				dc.l		0		; pointeur sur la liste d'enveloppes qui constituent le Buzzer, adresse/pointeur sur enveloppes
YM_DSP_increment_envbuzzer_en_cours:				dc.l		0		; increment avancée envbuzzer en 16:16	
YM_DSP_offset_envbuzzer_en_cours:					dc.l		0		; offset actuel sur l'envbuzzer en 16:16
YM_DSP_taille_envbuzzer_en_cours:					dc.l		0		; taille de l'envbuzzer - 16:16 ??



YM_DSP_table_de_volumes:
	dc.l				62,161,265,377,580,774,1155,1575,2260,3088,4570,6233,9330,13187,21220,32767
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
YM_flag_effets_voie_A:		dc.l		0
YM_flag_effets_voie_B:		dc.l		0
YM_flag_effets_voie_C:		dc.l		0
YM_flag_effets_Sinus_Sid:	dc.l		0
YM_flag_effets_Buzzer:		dc.l		0


PSG_compteur_frames_restantes:			dc.l		0
YM_pointeur_actuel_ymdata:				dc.l		0
YM_nombre_de_blocs_lZ4:					dc.l		0
YM_numero_bloc_en_cours:				dc.l		0

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

YM_DSP_pointeur_sur_samples_SID_ram_DSP:		dc.l		0
	.phrase	
YM_DSP_fin:

	
	


SOUND_DRIVER_SIZE			.equ			YM_DSP_fin-DSP_base_memoire
	.print	"--- Sound driver code size (DSP): ", /u SOUND_DRIVER_SIZE, " bytes / 8192 ---"


	
	.68000


	.even
	.phrase



fichier_ym7:
	;.incbin				"YM/Gwem_Gwem_Camp.ym7"					; 57 HZ/SID/D
	;.incbin			"YM/Mad_Max_Ajh_99.ym7"					; SID
	;.incbin			"YM/505_Pulse.ym7"						; SID/
	;.incbin			"YM/505_Oxygene.ym7"							; 51 hz
	;.incbin			"YM/UltraSyd_Thunderdome.ym7"					; 69 HZ
	;.incbin			"YM/Tao_Ultimate_Medley_Part_1.ym7"		; 200 HZ = fait plus de 2 mo decompressé
	;.incbin			"Gwem_Stardust_Memory.ym7"				; D/Sinus Sid + Sid
	;.incbin			"Dma_Sc_Galaxy_Trip.ym7"				; D/Sinus Sid + Sid 
	;.incbin			"505_Robost.ym7"						; D/sinus Sid
	;.incbin			"YM/Cube_Bullet_Sequence.ym7"				; SID + Buzzer
	;.incbin			"YM/Gwem_flash_of_the_rom.ym7"			; SID + Buzzer + 59 Hz
	
	;.incbin		"YM/Mad_Max_Buzzer.ym7"					; YM7 buzzer + SID
	;.incbin			"YM/Tomchi_Sidified.ym7"					; SID a partir de la 2eme frame. (+1)
	.incbin		"YM/DMA_Sc_Fantasia_main.ym7"					; YM7 SID
	;.incbin			"YM/Furax_Virtualescape_main.ym7"				; YM7 SID
	;.incbin		"YM/MadMax_Virtual_Esc_End.ym7"			; YM7 SID voie A
	;.incbin			"YM/LotekStyle_Alpha_Proxima.ym7"		; SID voie C au debut
	;.incbin		"YM/PYM_main_menu.ym7"					; YM7 avec enveloppe et digidrums - OK
	;.incbin		"YM/buzztone.ym7"						; digidrums sur B & C	- OK
	;.incbin		"YM/ancool_atari_baby.ym7"				; ENV au début, pas d effet ensuite : OK
	;.incbin		"YM/Jess_For_Your_Loader.ym7"			; YM7 sans effets avec env
	;.incbin		"YM/Decade_boot.ym7"					; YM7 avec env
	;.incbin			"YM/CountZero_Decade_Demo_Boot.ym7.ym7"	; 
	;.incbin			"YM/Floopy_Cassiope.ym7"					; SID // destructuré

	.even

debut_ram_libre_DSP:		dc.l			YM_DSP_fin
debut_ram_libre:			dc.l			FIN_RAM

	.bss
	.phrase
DEBUT_BSS:
;EDZ_compteur_reset_offset_entier_voie_A:			ds.l	1


YM_nombre_de_frames_totales:			ds.l				1
YM_frequence_replay:					ds.l				1
YM_flag_effets_sur_les_voies:			ds.w				1
YM_ecart_entre_frames_blocs:			ds.w				1
YM_ecart_entre_frames_dernier_bloc:		ds.l				1
YM_nb_registres_par_frame:				ds.w				1

	.phrase

YM_pointeur_origine_ymdata:		ds.l		1
YM_frequence_predivise:			ds.l		1
	.phrase

PSG_ecart_entre_les_registres_ymdata:		ds.l			1



YM_LZ4_pointeur_tailles_des_blocs:		ds.l		1
YM_LZ4_pointeur_destination_bloc_actuel:		ds.l		1

YM_tableau_des_blocs_decompresses:
; pointeur adresse bloc mémoire decompressé, écart entre les registres pour ce bloc
	.rept 32
		ds.l		1
		ds.l		1
	.endr

EDZTMP1:							ds.l			1
pointeur_buffer_de_debug:		ds.l			1
buffer_de_debug:				ds.l			480
	

FIN_RAM:


