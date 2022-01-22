; affichage console 
;
; OK - mode video
; OK - pointeur video 
; OK - ecriture dans memoire video
; OK - police de caractère
; OK - CLS au blitter
; OK - print de chiffres : 2 et 4 chiffres / Hexa avec 


CLEAR_BSS			.equ			0									; 1=efface toute la BSS jusqu'a la fin de la ram centrale

; original values from object (just simple or needed)
ob_liste_originale			equ		$200000-$4000							; address of list (shadow)
ob_list_courante			equ		ob_liste_originale+$2000				; address of read list

;resolution video
nb_octets_par_ligne			equ		320
nb_lignes					equ		256

couleur_char				equ		25

                .include    "jaguar.inc"
                .text

				.68000

	.if			CLEAR_BSS=1
	lea			DEBUT_BSS,a0
	lea			$200000,a1
	moveq		#0,d0
	
boucle_clean_BSS:
	move.b		d0,(a0)+
	cmp.l		a0,a1
	bne.s		boucle_clean_BSS
	.endif

	move.l		#INITSTACK, sp	
	move.w		#$06C7, VMODE			; 320x256
	move.w		#$100,JOYSTICK


    move.w	#%0000011011000111,VMODE	; 320x256
    bsr     InitVideo               	; Setup our video registers.

; CLS
	moveq	#0,d0
	bsr		print_caractere


	.if		1=0
; clean ecran
	lea			ecran1,a0
	move.l		#256*320,d7
	lsr.l		#2,d7
	subq.l		#1,d7
	moveq		#0,d0
clear_ecran1:
	move.l		d0,(a0)+
	dbf			d7,clear_ecran1
	.endif


	lea		chaine1,a0
	bsr		print_string

	lea		chaine2,a0
	bsr		print_string
	
	move.l	#6712,d0
	bsr		print_nombre_4_chiffres

;CR
	moveq	#10,d0
	bsr		print_caractere

	
	move.l	#$1A,d0
	bsr		print_nombre_hexa_2_chiffres

;CR
	moveq	#10,d0
	bsr		print_caractere

	move.l	#$A1B2C3,d0
	bsr		print_nombre_hexa_6_chiffres


	bsr		creer_Object_list

	; jsr     copy_olist

	move.l	#ob_list_courante,d0					; set the object list pointer
	swap	d0
	move.l	d0,OLP

	lea		$F00400,a2
	move.l	#255-1,d7
	moveq	#0,d0
	
copie_couleurs:
	move.w	d0,(a2)+
	addq.l	#1,d0
	dbf		d7,copie_couleurs

	move.l  #VBL,LEVEL0     	; Install 68K LEVEL0 handler
	move.w  a_vde,d0                	; Must be ODD
	sub.w   #16,d0
	ori.w   #1,d0
	move.w  d0,VI

	move.w  #1,INT1                 	; Enable video interrupts
	and.w   #$f8ff,sr



toto:
	lea		ob_liste_originale,a0
	lea		ob_list_courante,a1

	bra		toto
	nop
	;stop		#$2700
	nop

;--------------------------
; VBL

VBL:
                movem.l d0-d7/a0-a6,-(a7)
				
				add.w		#1,BG

                jsr     copy_olist              	; use Blitter to update active list from shadow

                addq.l	#1,vbl_counter

                move.w  #$101,INT1              	; Signal we're done
                move.w  #$0,INT2
.exit:
                movem.l (a7)+,d0-d7/a0-a6
                rte

; ---------------------------------------
; imprime une chaine terminée par un zéro
; a0=pointeur sur chaine
print_string:

print_string_boucle:
	moveq	#0,d0
	move.b	(a0)+,d0
	cmp.w	#0,d0
	bne.s	print_string_pas_fin_de_chaine
	rts
print_string_pas_fin_de_chaine:
	bsr		print_caractere
	bra.s	print_string_boucle
	rts

; ---------------------------------------
; imprime un nombre HEXA de 2 chiffres
print_nombre_hexa_2_chiffres:
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
	rts
	
convert_hexa:
	dc.b		48,49,50,51,52,53,54,55,56,57
	dc.b		65,66,67,68,69,70
	
; ---------------------------------------
; imprime un nombre de 2 chiffres
print_nombre_2_chiffres:

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
	rts

; ---------------------------------------
; imprime un nombre de 4 chiffres HEXA
print_nombre_hexa_4_chiffres:

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
	rts

; ---------------------------------------
; imprime un nombre de 6 chiffres HEXA ( pour les adresses memoire)
print_nombre_hexa_6_chiffres:

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
	rts


; ---------------------------------------
; imprime un nombre de 4 chiffres
print_nombre_4_chiffres:

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
	move.b	#couleur_char,d4
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
				move.l	taille_liste_OP,d1
				move.w	d1,d0
				move.l	d0,B_COUNT
				move.l	#LFU_REPLACE|SRCEN,B_CMD
				rts

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
				
				move.w #0,d0
				
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


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;
;; Procedure: InitVideo (same as in vidinit.s)
;;            Build values for hdb, hde, vdb, and vde and store them.
;;

InitVideo:
                movem.l d0-d6,-(sp)

				
				move.w	#-1,ntsc_flag
	
				move.w  CONFIG,d0                ; Also is joystick register
                andi.w  #VIDTYPE,d0              ; 0 = PAL, 1 = NTSC
                beq     .palvals
				move.w	#1,ntsc_flag

.ntscvals:		move.w  #NTSC_HMID,d2
                move.w  #NTSC_WIDTH,d0

                move.w  #NTSC_VMID,d6
                move.w  #NTSC_HEIGHT,d4
				
                bra     calc_vals
.palvals:
				move.w #PAL_HMID+40,d2
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
	move.l	#560,d0
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

	.data
curseur_Y_min		.equ		8
curseur_x:	dc.w		0
curseur_y:	dc.w		curseur_Y_min

chaine1:	dc.b	"CHAINE 1",10,0
chaine2:	dc.b	"CHAINE 2",10,0


	even
fonte:	
dc.b  $00, $00, $00, $00, $00, $00, $00, $00 
dc.b  $18, $24, $42, $81, $E7, $24, $24, $3C 
dc.b  $3C, $24, $24, $E7, $81, $42, $24, $18 
dc.b  $18, $14, $F2, $81, $81, $F2, $14, $18 
dc.b  $18, $28, $4F, $81, $81, $4F, $28, $18 
dc.b  $FF, $81, $81, $81, $81, $81, $81, $FF 
dc.b  $F8, $88, $8F, $89, $F9, $41, $41, $7F 
dc.b  $FF, $89, $89, $89, $F9, $81, $81, $FF 
dc.b  $01, $03, $06, $8C, $D8, $70, $20, $00 
dc.b  $7E, $C3, $D3, $D3, $DB, $C3, $C3, $7E 
dc.b  $18, $3C, $2C, $2C, $7E, $18, $18, $00 
dc.b  $10, $1C, $12, $10, $10, $70, $F0, $60 
dc.b  $F0, $C0, $FE, $D8, $DE, $18, $18, $00 
dc.b  $70, $C8, $DE, $DB, $DB, $7E, $1B, $1B 
dc.b  $03, $00, $0F, $03, $03, $03, $03, $0F 
dc.b  $90, $20, $D0, $A0, $D0, $A8, $D0, $E0 
dc.b  $7C, $C6, $C6, $00, $C6, $C6, $7C, $00 
dc.b  $06, $06, $06, $00, $06, $06, $06, $00 
dc.b  $7C, $06, $06, $7C, $C0, $C0, $7C, $00 
dc.b  $7C, $06, $06, $7C, $06, $06, $7C, $00 
dc.b  $C6, $C6, $C6, $7C, $06, $06, $06, $00 
dc.b  $7C, $C0, $C0, $7C, $06, $06, $7C, $00 
dc.b  $7C, $C0, $C0, $7C, $C6, $C6, $7C, $00 
dc.b  $7C, $06, $06, $00, $06, $06, $06, $00 
dc.b  $7C, $C6, $C6, $7C, $C6, $C6, $7C, $00 
dc.b  $7C, $C6, $C6, $7C, $06, $06, $7C, $00 
dc.b  $00, $3C, $46, $06, $7E, $66, $3C, $00 
dc.b  $78, $66, $7D, $64, $7E, $03, $0B, $06 
dc.b  $07, $0F, $1F, $18, $18, $10, $1E, $17 
dc.b  $F0, $F8, $EC, $04, $04, $04, $3C, $54 
dc.b  $11, $0B, $0D, $06, $07, $2E, $39, $38 
dc.b  $FC, $FC, $FF, $E1, $E1, $21, $3F, $00 
dc.b  $00, $00, $00, $00, $00, $00, $00, $00 
dc.b  $18, $18, $18, $18, $18, $00, $18, $00 
dc.b  $66, $66, $44, $00, $00, $00, $00, $00 
dc.b  $00, $24, $7E, $24, $24, $7E, $24, $00 
dc.b  $14, $3E, $55, $3C, $1E, $55, $3E, $14 
dc.b  $62, $66, $0C, $18, $30, $66, $46, $00 
dc.b  $78, $CC, $61, $CE, $CC, $CC, $78, $00 
dc.b  $18, $18, $10, $00, $00, $00, $00, $00 
dc.b  $04, $08, $18, $18, $18, $18, $08, $04 
dc.b  $20, $10, $18, $18, $18, $18, $10, $20 
dc.b  $00, $54, $38, $FE, $38, $54, $00, $00 
dc.b  $00, $18, $18, $7E, $18, $18, $00, $00 
dc.b  $00, $00, $00, $00, $00, $30, $30, $20 
dc.b  $00, $00, $00, $3C, $00, $00, $00, $00 
dc.b  $00, $00, $00, $00, $00, $18, $18, $00 
dc.b  $03, $06, $0C, $18, $30, $60, $C0, $00 
dc.b  $3C, $66, $6E, $76, $66, $66, $3C, $00 
dc.b  $18, $38, $18, $18, $18, $18, $18, $00 
dc.b  $3C, $66, $0E, $1C, $38, $70, $7E, $00 
dc.b  $7E, $0C, $18, $3C, $06, $46, $3C, $00 
dc.b  $0C, $1C, $2C, $4C, $7E, $0C, $0C, $00 
dc.b  $7E, $60, $7C, $06, $06, $46, $3C, $00 
dc.b  $1C, $20, $60, $7C, $66, $66, $3C, $00 
dc.b  $7E, $06, $0E, $1C, $18, $18, $18, $00 
dc.b  $3C, $66, $66, $3C, $66, $66, $3C, $00 
dc.b  $3C, $66, $66, $3E, $06, $0C, $38, $00 
dc.b  $00, $18, $18, $00, $00, $18, $18, $00 
dc.b  $00, $18, $18, $00, $18, $18, $10, $00 
dc.b  $06, $0C, $18, $30, $18, $0C, $06, $00 
dc.b  $00, $00, $3C, $00, $00, $3C, $00, $00 
dc.b  $60, $30, $18, $0C, $18, $30, $60, $00 
dc.b  $3C, $46, $06, $0C, $18, $18, $00, $18 
dc.b  $3C, $66, $6E, $6A, $6E, $60, $3C, $00 
dc.b  $3C, $66, $66, $7E, $66, $66, $66, $00 
dc.b  $7C, $66, $66, $7C, $66, $66, $7C, $00 
dc.b  $3C, $62, $60, $60, $60, $62, $3C, $00 
dc.b  $7C, $66, $66, $66, $66, $66, $7C, $00 
dc.b  $7E, $60, $60, $7C, $60, $60, $7E, $00 
dc.b  $7E, $60, $60, $7C, $60, $60, $60, $00 
dc.b  $3C, $62, $60, $6E, $66, $66, $3E, $00 
dc.b  $66, $66, $66, $7E, $66, $66, $66, $00 
dc.b  $18, $18, $18, $18, $18, $18, $18, $00 
dc.b  $06, $06, $06, $06, $06, $46, $3C, $00 
dc.b  $66, $6C, $78, $70, $78, $6C, $66, $00 
dc.b  $60, $60, $60, $60, $60, $60, $7C, $00 
dc.b  $FC, $D6, $D6, $D6, $D6, $C6, $C6, $00 
dc.b  $62, $72, $7A, $5E, $4E, $46, $42, $00 
dc.b  $3C, $66, $66, $66, $66, $66, $3C, $00 
dc.b  $7C, $66, $66, $7C, $60, $60, $60, $00 
dc.b  $3C, $66, $66, $66, $66, $66, $3C, $06 
dc.b  $7C, $66, $66, $7C, $66, $66, $66, $00 
dc.b  $3C, $62, $70, $3C, $0E, $46, $3C, $00 
dc.b  $7E, $18, $18, $18, $18, $18, $18, $00 
dc.b  $66, $66, $66, $66, $66, $66, $3C, $00 
dc.b  $66, $66, $66, $66, $66, $64, $78, $00 
dc.b  $C6, $C6, $C6, $D6, $D6, $D6, $FC, $00 
dc.b  $66, $66, $66, $3C, $66, $66, $66, $00 
dc.b  $66, $66, $66, $3C, $18, $18, $18, $00 
dc.b  $7E, $0E, $1C, $38, $70, $60, $7E, $00 
dc.b  $1E, $18, $18, $18, $18, $18, $1E, $00 
dc.b  $40, $60, $30, $18, $0C, $06, $02, $00 
dc.b  $78, $18, $18, $18, $18, $18, $78, $00 
dc.b  $10, $38, $6C, $00, $00, $00, $00, $00 
dc.b  $00, $00, $00, $00, $00, $00, $7E, $00 
dc.b  $00, $C0, $C0, $60, $00, $00, $00, $00 
dc.b  $00, $3C, $06, $3E, $66, $66, $3E, $00 
dc.b  $60, $7C, $66, $66, $66, $66, $7C, $00 
dc.b  $00, $3C, $62, $60, $60, $62, $3C, $00 
dc.b  $06, $3E, $66, $66, $66, $66, $3E, $00 
dc.b  $00, $3C, $66, $7E, $60, $62, $3C, $00 
dc.b  $1E, $30, $7C, $30, $30, $30, $30, $00 
dc.b  $00, $3E, $66, $66, $66, $3E, $46, $3C 
dc.b  $60, $7C, $66, $66, $66, $66, $66, $00 
dc.b  $18, $00, $18, $18, $18, $18, $18, $00 
dc.b  $00, $08, $18, $18, $18, $18, $58, $30 
dc.b  $60, $64, $68, $70, $78, $6C, $66, $00 
dc.b  $18, $18, $18, $18, $18, $18, $0C, $00 
dc.b  $00, $FC, $D6, $D6, $D6, $D6, $C6, $00 
dc.b  $00, $7C, $66, $66, $66, $66, $66, $00 
dc.b  $00, $3C, $66, $66, $66, $66, $3C, $00 
dc.b  $00, $7C, $66, $66, $66, $7C, $60, $60 
dc.b  $00, $3E, $66, $66, $66, $66, $3E, $06 
dc.b  $00, $6C, $70, $60, $60, $60, $60, $00 
dc.b  $00, $3C, $72, $38, $1C, $4E, $3C, $00 
dc.b  $18, $3C, $18, $18, $18, $18, $0C, $00 
dc.b  $00, $66, $66, $66, $66, $66, $3E, $00 
dc.b  $00, $66, $66, $66, $66, $64, $78, $00 
dc.b  $00, $C6, $C6, $D6, $D6, $D6, $FC, $00 
dc.b  $00, $66, $66, $3C, $66, $66, $66, $00 
dc.b  $00, $66, $66, $66, $26, $1E, $46, $3C 
dc.b  $00, $7E, $0E, $1C, $38, $70, $7E, $00 
dc.b  $0E, $18, $18, $30, $18, $18, $0E, $00 
dc.b  $18, $18, $18, $18, $18, $18, $18, $18 
dc.b  $70, $18, $18, $0C, $18, $18, $70, $00 
dc.b  $00, $60, $F2, $9E, $0C, $00, $00, $00 
dc.b  $10, $10, $28, $28, $44, $44, $82, $FE 
dc.b  $FF, $FF, $C0, $C0, $C0, $C0, $C0, $C0 
dc.b  $24, $00, $66, $66, $66, $66, $3E, $00 
dc.b  $FF, $FF, $03, $03, $03, $03, $03, $03 
dc.b  $C0, $C0, $C0, $C0, $C0, $C0, $FF, $FF 
dc.b  $24, $00, $3C, $06, $3E, $66, $3E, $00 
dc.b  $03, $03, $03, $03, $03, $03, $FF, $FF 
dc.b  $FF, $FF, $00, $00, $00, $00, $00, $00 
dc.b  $00, $00, $00, $00, $00, $00, $FF, $FF 
dc.b  $C0, $C0, $C0, $C0, $C0, $C0, $C0, $C0 
dc.b  $03, $03, $03, $03, $03, $03, $03, $03 
dc.b  $00, $00, $00, $FF, $FF, $00, $00, $00 
dc.b  $18, $18, $18, $18, $18, $18, $18, $18 
dc.b  $C0, $C0, $C0, $FF, $FF, $C0, $C0, $C0 
dc.b  $03, $03, $03, $FF, $FF, $03, $03, $03 
dc.b  $24, $00, $3C, $66, $7E, $66, $66, $00 
dc.b  $FF, $FF, $18, $18, $18, $18, $18, $18 
dc.b  $18, $18, $18, $18, $18, $18, $FF, $FF 
dc.b  $00, $00, $7E, $1B, $7F, $D8, $7E, $00 
dc.b  $3F, $78, $D8, $DE, $F8, $D8, $DF, $00 
dc.b  $18, $34, $00, $3C, $66, $66, $3C, $00 
dc.b  $24, $00, $3C, $66, $66, $66, $3C, $00 
dc.b  $30, $18, $00, $3C, $66, $66, $3C, $00 
dc.b  $18, $24, $00, $66, $66, $66, $3C, $00 
dc.b  $30, $18, $00, $66, $66, $66, $3C, $00 
dc.b  $66, $00, $66, $66, $66, $3E, $46, $3C 
dc.b  $66, $00, $3C, $66, $66, $66, $3C, $00 
dc.b  $66, $00, $66, $66, $66, $66, $3C, $00 
dc.b  $18, $3C, $62, $60, $60, $62, $3C, $18 
dc.b  $1C, $3A, $30, $7C, $30, $30, $7E, $00 
dc.b  $66, $66, $3C, $18, $3C, $18, $18, $00 
dc.b  $3C, $66, $66, $6C, $66, $66, $EC, $00 
dc.b  $18, $18, $18, $18, $18, $18, $18, $18 
dc.b  $0C, $18, $00, $3C, $06, $7E, $3E, $00 
dc.b  $0C, $18, $00, $18, $18, $18, $18, $00 
dc.b  $0C, $18, $00, $3C, $66, $66, $3C, $00 
dc.b  $0C, $18, $00, $66, $66, $66, $3E, $00 
dc.b  $34, $58, $00, $7C, $66, $66, $66, $00 
dc.b  $1A, $2C, $62, $72, $5A, $4E, $46, $00 
dc.b  $00, $3C, $46, $3E, $66, $3E, $00, $7E 
dc.b  $00, $3C, $66, $66, $66, $3C, $00, $7E 
dc.b  $00, $18, $00, $18, $30, $60, $66, $3C 
dc.b  $00, $00, $00, $3E, $30, $30, $30, $00 
dc.b  $00, $00, $00, $7C, $0C, $0C, $0C, $00 
dc.b  $62, $E4, $68, $76, $2B, $43, $86, $0F 
dc.b  $62, $E4, $68, $76, $2E, $56, $9F, $06 
dc.b  $00, $18, $00, $18, $18, $18, $18, $18 
dc.b  $1B, $36, $6C, $D8, $6C, $36, $1B, $00 
dc.b  $D8, $6C, $36, $1B, $36, $6C, $D8, $00 
dc.b  $34, $58, $00, $3C, $06, $7E, $3E, $00 
dc.b  $34, $58, $00, $3C, $66, $66, $3C, $00 
dc.b  $02, $3C, $66, $6E, $76, $66, $3C, $40 
dc.b  $00, $02, $3C, $6E, $76, $66, $3C, $40 
dc.b  $00, $00, $7E, $DB, $DE, $D8, $7F, $00 
dc.b  $00, $7E, $D8, $D8, $FC, $D8, $D8, $DE 
dc.b  $20, $10, $3C, $66, $66, $7E, $66, $66 
dc.b  $34, $58, $3C, $66, $66, $7E, $66, $66 
dc.b  $34, $58, $3C, $66, $66, $66, $66, $3C 
dc.b  $66, $00, $00, $00, $00, $00, $00, $00 
dc.b  $0C, $18, $30, $00, $00, $00, $00, $00 
dc.b  $00, $10, $38, $10, $10, $10, $00, $00 
dc.b  $7A, $CA, $CA, $CA, $7A, $0A, $0A, $0A 
dc.b  $3C, $42, $99, $B5, $B1, $9D, $42, $3C 
dc.b  $3C, $42, $B9, $B5, $B9, $B5, $42, $3C 
dc.b  $F1, $5B, $55, $51, $51, $00, $00, $00 
dc.b  $66, $00, $E6, $66, $66, $F6, $06, $1C 
dc.b  $F6, $66, $66, $66, $66, $F6, $06, $1C 
dc.b  $00, $66, $76, $3C, $6E, $66, $00, $00 
dc.b  $00, $7C, $0C, $0C, $0C, $7E, $00, $00 
dc.b  $00, $1E, $06, $0E, $1E, $36, $00, $00 
dc.b  $00, $7E, $0C, $0C, $0C, $0C, $00, $00 
dc.b  $00, $7C, $06, $66, $66, $66, $00, $00 
dc.b  $00, $1C, $0C, $0C, $0C, $0C, $00, $00 
dc.b  $00, $1E, $0C, $06, $06, $06, $00, $00 
dc.b  $00, $7E, $36, $36, $36, $36, $00, $00 
dc.b  $60, $6E, $66, $66, $66, $7E, $00, $00 
dc.b  $00, $3C, $0C, $0C, $00, $00, $00, $00 
dc.b  $00, $3E, $06, $06, $06, $3E, $00, $00 
dc.b  $60, $7E, $06, $06, $06, $0E, $00, $00 
dc.b  $00, $6C, $3E, $66, $66, $6E, $00, $00 
dc.b  $00, $1C, $0C, $0C, $0C, $3C, $00, $00 
dc.b  $00, $3E, $36, $36, $36, $1C, $00, $00 
dc.b  $00, $36, $36, $36, $36, $7E, $00, $00 
dc.b  $00, $7E, $66, $76, $06, $7E, $00, $00 
dc.b  $00, $66, $66, $3C, $0E, $7E, $00, $00 
dc.b  $00, $3E, $06, $36, $36, $34, $30, $00 
dc.b  $00, $78, $0C, $0C, $0C, $0C, $00, $00 
dc.b  $00, $D6, $D6, $D6, $D6, $FE, $00, $00 
dc.b  $00, $7C, $6C, $6C, $6C, $EC, $00, $00 
dc.b  $00, $1C, $0C, $0C, $0C, $0C, $0C, $00 
dc.b  $00, $3E, $06, $06, $06, $06, $06, $00 
dc.b  $00, $FE, $66, $66, $66, $7E, $00, $00 
dc.b  $00, $7E, $66, $76, $06, $06, $06, $00 
dc.b  $00, $36, $36, $1C, $0C, $0C, $0C, $00 
dc.b  $1C, $32, $3C, $66, $66, $3C, $4C, $38 
dc.b  $00, $10, $38, $6C, $C6, $82, $00, $00 
dc.b  $66, $F7, $99, $99, $EF, $66, $00, $00 
dc.b  $00, $00, $76, $DC, $C8, $DC, $76, $00 
dc.b  $1C, $36, $66, $7C, $66, $66, $7C, $60 
dc.b  $00, $FE, $66, $62, $60, $60, $60, $F8 
dc.b  $00, $00, $FE, $6C, $6C, $6C, $6C, $48 
dc.b  $FE, $66, $30, $18, $30, $66, $FE, $00 
dc.b  $00, $1E, $38, $6C, $6C, $6C, $38, $00 
dc.b  $00, $00, $6C, $6C, $6C, $6C, $7F, $C0 
dc.b  $00, $00, $7E, $18, $18, $18, $18, $10 
dc.b  $3C, $18, $3C, $66, $66, $3C, $18, $3C 
dc.b  $00, $3C, $66, $7E, $66, $66, $3C, $00 
dc.b  $00, $3C, $66, $66, $66, $24, $66, $00 
dc.b  $1C, $36, $78, $DC, $CC, $EC, $78, $00 
dc.b  $0C, $18, $38, $54, $54, $38, $30, $60 
dc.b  $00, $10, $7C, $D6, $D6, $D6, $7C, $10 
dc.b  $3E, $70, $60, $7E, $60, $70, $3E, $00 
dc.b  $3C, $66, $66, $66, $66, $66, $66, $00 
dc.b  $00, $7E, $00, $7E, $00, $7E, $00, $00 
dc.b  $18, $18, $7E, $18, $18, $00, $7E, $00 
dc.b  $30, $18, $0C, $18, $30, $00, $7E, $00 
dc.b  $0C, $18, $30, $18, $0C, $00, $7E, $00 
dc.b  $00, $0E, $1B, $1B, $18, $18, $18, $18 
dc.b  $18, $18, $18, $18, $D8, $D8, $70, $00 
dc.b  $18, $18, $00, $7E, $00, $18, $18, $00 
dc.b  $00, $32, $4C, $00, $32, $4C, $00, $00 
dc.b  $38, $6C, $38, $00, $00, $00, $00, $00 
dc.b  $38, $7C, $38, $00, $00, $00, $00, $00 
dc.b  $00, $00, $00, $00, $18, $18, $00, $00 
dc.b  $00, $00, $0F, $18, $D8, $70, $30, $00 
dc.b  $38, $6C, $6C, $6C, $6C, $00, $00, $00 
dc.b  $38, $6C, $18, $30, $7C, $00, $00, $00 
dc.b  $78, $0C, $38, $0C, $78, $00, $00, $00 
dc.b  $00, $FE, $00, $00, $00, $00, $00, $00
	


            .dphrase
stoplist:		dc.l	0,4


            .bss
            .dphrase
DEBUT_BSS:
                        .phrase

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
ecran1:				ds.b		40*256				; 1 bitplane