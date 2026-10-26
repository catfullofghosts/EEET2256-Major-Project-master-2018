;RD Jordan S3136906
;Major Project EEET2256
;Pan and Tilt gesture controled camera
;Excuse the mess, very tight turn around for a project this size + first time writing an asembly project this size.
; ******************************************************

; Define some variables

;
.def  temp  = r16;
.def  data = r17;
.def data_out = r28;


; Define here Reset and interrupt vectors, if any
;
reset:
   rjmp start
   reti      ; Addr $01
   reti      ; Addr $02
   reti      ; Addr $03
   reti      ; Addr $04
   reti      ; Addr $05
   reti      ; Addr $06        Use 'rjmp myVector'
   reti      ; Addr $07        to define a interrupt vector
   reti      ; Addr $08
   reti      ; Addr $09
   reti      ; Addr $0A
   reti      ; Addr $0B        This is just an example
   reti      ; Addr $0C        Not all MCUs have the same
   reti      ; Addr $0D        number of interrupt vectors
   reti      ; Addr $0E
   reti      ; Addr $0F
   reti      ; Addr $10
   
;***********************************************************
;
;
;*******************************
;
; Program starts here after Reset


 start:

 ;*******Set up the stack pointer to use PUSH and POP
  CLR R1		;Clear Register 
  OUT 0x3F,R1	;	
  LDI R28,0x5F	;	
  LDI R29,0x08	;	
  OUT 0x3E,R29	;
  OUT 0x3D,R28	;	

     	;DDRB = 0xff;
  SER R25		;Set Register 
  OUT 0x17,R25	;	Out to I/O location 

  ;*******set servo defult position

call servo_init; set up the servo regiters

LDI temp, 0x0B; // set servo2 to 100' Y servo was 0x0C
PUSH temp;
call deg_to_tick;
call mov_servo_2;


LDI temp, 0x08; set servo1 to 80' x servo
PUSH temp;
call deg_to_tick;
call mov_servo_1;

  ;*******


 CALL I2cinit ;
    ;set slave address
	CLR R25		;Set Register 
	OUT 0x18,R25	;	turn all leds off
	LDI data, 0x92;
CALL read_reg_prox;

CPI data, 0xAB; compare id with receved data
BRNE  start ; if id not receved restart

OUT 0x18,data	;	display returned id value

;***************************SET UP SENSOR REGISTERS

;softI2C_write_reg_prox( WTIME,  0x00);
LDI data, 0x83;
LDI data_out, 0x00; 
CALL write_reg_prox;
;softI2C_write_reg_prox( PPULSE,  0x23); // set up proximity pulse lenth and pulse umber
LDI data, 0x8E;
LDI data_out, 0x23; 
CALL write_reg_prox;

;softI2C_write_reg_prox( GPULSE,  0xAA); //set number of gesture pulses and pulse time to 10 pulses and 32us per pulse
LDI data, 0xA6;
LDI data_out, 0xAA; 
CALL write_reg_prox;
// power on and turn on prox
;softI2C_write_reg_prox( ENABLE,  0x65); // on / procimity/ gesture, prox interupts <<use thisa one
LDI data, 0x80;
LDI data_out, 0x65; 
CALL write_reg_prox;
;softI2C_write_reg_prox( GPENTH,  0x10); // set gesture proximity entry threshold
LDI data, 0xA0;
LDI data_out, 0x10; 
CALL write_reg_prox;

;softI2C_write_reg_prox( GEXTH,  0x3A); // set gesture proximity exit threshold
LDI data, 0xA1;
LDI data_out, 0x3A; 
CALL write_reg_prox;



;*********************************
loop: 

;check for gesture detect flag using polling, Interupt would have been better, not enough time sadly
;softI2C_read_reg_prox(STATUS) & 0x04
LDI data, 0x93;
CALL read_reg_prox;
ANDI data, 0x04;
;OUT 0x18, data; display statis register
CPI data, 0x00; 
BREQ  loop ; if id not receved restart
rjmp decode_gest;
nop
servo_move_valid: ; The gesture interupt has been triggered, the sensors state machine has detected a gesture

;IN data, 0x18 ; read the port b to see which direction was trigered, <<used for debug
MOV data, R16;
CPI data, 0x01;see if up
BREQ move_up; 

CPI data, 0x02;see if down
BREQ move_down; 

CPI data, 0x04;see if left
BREQ move_left; 

CPI data, 0x08;see if right
BREQ move_right; 


jmp loop;

move_up:

;call delay_man1;
;load curent position from xyPOS
POP temp; x pos
POP data; y pos

MOV data_out, data;
CPI data_out, 0x0F;
BRSH stack_mod;

INC data;

stack_mod:
MOV data_out, data;

PUSH data; put y back on the stack
PUSH temp; put x back on the stack
MOV temp, data_out; place new Y pos in temp to be sent to servo 
;OUT 0x18, temp;


rjmp moveUD;


move_down:

;call delay_man1;
;load curent position from xyPOS
POP temp; x pos
POP data; y pos

MOV data_out, data;
CPI data_out, 0x0A;
BREQ stack_mod2;

DEC data;

stack_mod2:
MOV data_out, data;

PUSH data; put y back on the stack
PUSH temp; put x back on the stack
MOV temp, data_out; place new Y pos in temp to be sent to servo 
;OUT 0x18, temp;


rjmp moveUD;


move_left:

;call delay_man1;
;load curent position from xyPOS
POP temp; x pos
POP data; y pos
OUT 0x18, temp;
MOV data_out, temp;
CPI data_out, 0x00;
BREQ stack_mod3;

DEC temp;

stack_mod3:
MOV data_out, temp;

PUSH data; put y back on the stack
PUSH temp; put x back on the stack
MOV temp, data_out; place new X pos in temp to be sent to servo 
;OUT 0x18, temp;

rjmp moveLR;

move_right:

;call delay_man1;
;load curent position from xyPOS
POP temp; x pos
POP data; y pos
OUT 0x18, temp;
MOV data_out, temp;
CPI data_out, 0x0C;
BREQ stack_mod4;

INC temp;

stack_mod4:
MOV data_out, temp;

PUSH data; put y back on the stack
PUSH temp; put x back on the stack
MOV temp, data_out; place new X pos in temp to be sent to servo 
;OUT 0x18, temp;

rjmp moveLR;

moveUD:
call deg_to_tick;
call mov_servo_2;

call delay_1s; Used to avoid double triggering
call delay_1s;
call delay_1s;
;call delay_1s;
;call delay_1s;


jmp loop ;

moveLR:
call deg_to_tick;
call mov_servo_1;

call delay_1s; Used to avoid double triggering
call delay_1s;
call delay_1s;
;call delay_1s;
;call delay_1s;

jmp loop ;



;**************************************************START OF SUB ROUTINES (some parts based on dissasembly of the C code)
I2cinit:
 IN R24,0x12		;In from I/O location 
  ANDI R24,0x01		;Logical AND with immediate 
  OUT 0x12,R24		;Out to I/O location 
;	SCLPORT&=(1<<SCL);
  IN R24,0x12		;In from I/O location 
  ANDI R24,0x02		;Logical AND with immediate 
  OUT 0x12,R24		;Out to I/O location 
	;SOFT_I2C_SDA_HIGH;
  CBI 0x11,0		;Clear bit in I/O register 
;	SOFT_I2C_SCL_HIGH;
  CBI 0x11,1		;Clear bit in I/O register 
  ret;

  ;start transmission
  I2cstart:
  ;SOFT_I2C_SCL_HIGH;
  CBI 0x11,1	;	Clear bit in I/O register 

   LDI R24,0x05		;Load immediate 
  LDI R25,0x00	;	Load immediate 
  MOVW R30,R24		;Copy register pair 
  SBIW R30,0x01		;Subtract immediate from word 
  BRNE PC-0x01	;	Branch if not equal 

  ;	SOFT_I2C_SDA_LOW;
  SBI 0x11,0		;Set bit in I/O register 

  SBIW R24,0x01	;	Subtract immediate from word 

  BRNE PC-0x01		;Branch if not equal 
 ret;

 ;write byte
 I2cwrite:
 MOV R24, temp; mode our data into the data register
   LDI R25,0x08		;Load immediate 
  LDI R20,0x03		;Load immediate 
  LDI R21,0x00		;Load immediate 
  LDI R18,0x05		;Load immediate 
  LDI R19,0x00		;Load immediate 

		;SOFT_I2C_SCL_LOW;
  SBI 0x11,1		;Set bit in I/O register 

  MOVW R30,R20		;Copy register pair 
  SBIW R30,0x01		;Subtract immediate from word 

  BRNE PC-0x01		;Branch if not equal 

  	;if(data & 0x80)
  TST R24		; Test for Zero or Minus 
  BRGE PC+0x03	;	Branch if greater or equal, signed 
	;	SOFT_I2C_SDA_HIGH;
  CBI 0x11,0		;Clear bit in I/O register 
  RJMP PC+0x0002	;	Relative jump 
		;SOFT_I2C_SDA_LOW;
  SBI 0x11,0		;Set bit in I/O register 

  MOVW R30,R18	;	Copy register pair 
  SBIW R30,0x01		;Subtract immediate from word 
  BRNE PC-0x01		;Branch if not equal 
		;SOFT_I2C_SCL_HIGH;
  CBI 0x11,1		;Clear bit in I/O register 

  MOVW R30,R18		;Copy register pair 
  SBIW R30,0x01		;Subtract immediate from word 
  BRNE PC-0x01		;Branch if not equal 

  		;while((SCLPIN & (1<<SCL))==0);
  SBIS 0x10,1		;Skip if bit in I/O register set 
  RJMP PC-0x0001		;Relative jump 
		;data=data<<1;
  LSL R24		;Logical Shift Left 
  SUBI R25,0x01		;Subtract immediate 
	;for(i=0;i<8;i++)
  BRNE PC-0x14		;Branch if not equal 
	;SOFT_I2C_SCL_LOW;
  SBI 0x11,1		;Set bit in I/O register 

  LDI R24,0x03	;	Load immediate 
  LDI R25,0x00		;Load immediate 
  SBIW R24,0x01	;	Subtract immediate from word 
  BRNE PC-0x01		;Branch if not equal 

	;SOFT_I2C_SDA_HIGH;
  CBI 0x11,0		;Clear bit in I/O register 

  LDI R18,0x05		;Load immediate 
  LDI R19,0x00		;Load immediate 

  MOVW R24,R18		;Copy register pair 
  SBIW R24,0x01		;Subtract immediate from word 
  BRNE PC-0x01		;Branch if not equal 
	;SOFT_I2C_SCL_HIGH;
  CBI 0x11,1	;	Clear bit in I/O register 
  MOVW R24,R18		;Copy register pair 
  SBIW R24,0x01	;	Subtract immediate from word 
  BRNE PC-0x01		;Branch if not equal 

  ;	uint8_t ack=!(SDAPIN & (1<<SDA));
  IN R24,0x10	;	In from I/O location 
	;SOFT_I2C_SCL_LOW;
  SBI 0x11,1	;	Set bit in I/O register 
  MOVW R30,R18	;	Copy register pair 
  SBIW R30,0x01	;	Subtract immediate from word 
  BRNE PC-0x01	;	Branch if not equal 
	;return ack;
  COM R24	;	One's complement 

  ANDI R24,0x01		;Logical AND with immediate 
  ret 	;	Subroutine return 


  I2cread:
    MOV R25,R24		; Copy register 

  LDI R18,0x00		;;Load immediate 
  LDI R19,0x00		;Load immediate 
;	uint8_t data=0x00;
  LDI R24,0x00		;Load immediate 
  LDI R20,0x05		;Load immediate 
  LDI R21,0x00	;	Load immediate 
	;	data|=(0x80>>i);
  LDI R26,0x80	;	Load immediate 
  LDI R27,0x00	;	Load immediate 
	;	SOFT_I2C_SCL_LOW;
  SBI 0x11,1		;Set bit in I/O register 
  MOVW R30,R20		;Copy register pair 
  SBIW R30,0x01	;	;Subtract immediate from word 
  BRNE PC-0x01		;Branch if not equal 
;		SOFT_I2C_SCL_HIGH;
  CBI 0x11,1		;Clear bit in I/O register 
  MOVW R30,R20		;Copy register pair 
  SBIW R30,0x01	;	Subtract immediate from word 
  BRNE PC-0x01		;Branch if not equal 
	;	while((SCLPIN & (1<<SCL))==0);
  SBIS 0x10,1	;	Skip if bit in I/O register set 
  RJMP PC-0x0001		;Relative jump 
	;	if(SDAPIN &(1<<SDA))
  SBIS 0x10,0	;	Skip if bit in I/O register set 
  RJMP PC+0x0009	;	Relative jump 
	;	data|=(0x80>>i);
  MOVW R22,R26	;	Copy register pair 
  MOV R0,R18	;	Copy register 
  RJMP PC+0x0003	;	Relative jump 
  ASR R23	;	Arithmetic shift right 
  ROR R22	;	Rotate right through carry 
  DEC R0	;	Decrement 
  BRPL PC-0x03	;	Branch if plus 
  OR R24,R22		;Logical OR 
  SUBI R18,0xFF		;Subtract immediate 
  SBCI R19,0xFF		;Subtract immediate with carry 
	;for(i=0;i<8;i++)
  CPI R18,0x08		;Compare with immediate 
  CPC R19,R1	;	Compare with carry 
  BRNE PC-0x18	;	Branch if not equal 
	;SOFT_I2C_SCL_LOW;
  SBI 0x11,1	;	Set bit in I/O register 
  LDI R30,0x03	;	Load immediate 
  LDI R31,0x00		;Load immediate 
  SBIW R30,0x01	;	Subtract immediate from word 
  BRNE PC-0x01		;Branch if not equal 
;	if(ack)
  TST R25		;Test for Zero or Minus 
  BREQ PC+0x03		;Branch if equal 
	;	SOFT_I2C_SDA_LOW;
  SBI 0x11,0		;Set bit in I/O register 
  RJMP PC+0x0002	;	Relative jump 
	;	SOFT_I2C_SDA_HIGH;
  CBI 0x11,0		;Clear bit in I/O register 
  LDI R18,0x05		;Load immediate 
  LDI R19,0x00		;Load immediate 
  MOVW R30,R18		;Copy register pair 
  SBIW R30,0x01	;	Subtract immediate from word 
  BRNE PC-0x01		;Branch if not equal 
	;SOFT_I2C_SCL_HIGH;
  CBI 0x11,1		;Clear bit in I/O register 
  MOVW R30,R18		;Copy register pair 
  SBIW R30,0x01		;Subtract immediate from word 
  BRNE PC-0x01		;Branch if not equal 
;	SOFT_I2C_SCL_LOW;
  SBI 0x11,1		;Set bit in I/O register 
  MOVW R30,R18		;Copy register pair 
  SBIW R30,0x01		;Subtract immediate from word 
  BRNE PC-0x01		;Branch if not equal 
  RET 		;Subroutine return 

  I2cStop:
  ;SOFT_I2C_SDA_LOW;
  SBI 0x11,0	;	Set bit in I/O register 
  LDI R24,0x05	;	Load immediate 
  LDI R25,0x00	;	Load immediate 
  MOVW R30,R24	;	Copy register pair 
  SBIW R30,0x01		;Subtract immediate from word 
  BRNE PC-0x01		;Branch if not equal 
;	SOFT_I2C_SCL_HIGH;
  CBI 0x11,1		;Clear bit in I/O register 
  LDI R30,0x03		;Load immediate 
  LDI R31,0x00		;Load immediate 
  SBIW R30,0x01		;Subtract immediate from word 
  BRNE PC-0x01		;Branch if not equal 
;	SOFT_I2C_SDA_HIGH;
  CBI 0x11,0		;Clear bit in I/O register 
  SBIW R24,0x01		;Subtract immediate from word 
  BRNE PC-0x01	;	Branch if not equal 
  RET 	;	Subroutine return 


  read_reg_prox:
  CALL I2cstart ;
LDI temp ,0x72	;Load immediate <<< slave add
LDI R24,0x92	;	Load immediate 
 Call I2cwrite ;
 ;LDI temp ,0x92		;Load immediate <<< register add
 MOV temp, data;
 Call I2cwrite ;
  MOV R24,R28	;	Copy register  <<<<<< xx
 CALL I2cstart ;
 LDI temp ,0x73 	;Load immediate <<< salce add
 Call I2cwrite ;

 LDI R24,0x00	;	Load immediate  <<<<<xx
 Call I2cread ;

   MOV R28,R24		;Copy register <<<<xx
   MOV data, R24; put the returned data in data
   ;  OUT 0x18,R24	;	Out to I/O location 
   CALL I2cstop;
   ret;

    write_reg_prox:
  CALL I2cstart ;
LDI temp ,0x72	;Load immediate <<< slave add
LDI R24,0x92	;	Load immediate 
 Call I2cwrite ;
 ;LDI temp ,0x92		;Load immediate <<< register add
 MOV temp, data;
 Call I2cwrite ;
 MOV temp, data_out;
 Call I2cwrite ;
  CALL I2cstop;

 

   ret;

decode_gest: ;*****Function to read out and process the gesture data from the sensors FIFO buffer
;OUT 0x18,data	;	display returned id value

				;	FIFO[i] = softI2C_read_reg_prox(GFIFO_U);
LDI data, 0xFC; get up data
CALL read_reg_prox;
PUSH data;

				;	FIFO[i+1] = softI2C_read_reg_prox(GFIFO_D);

LDI data, 0xFD;
CALL read_reg_prox; get the down register val
PUSH data;

				;	FIFO[i+2] = softI2C_read_reg_prox(GFIFO_L);

LDI data, 0xFE; get left data
CALL read_reg_prox;
PUSH data;

				;	FIFO[i+3] = softI2C_read_reg_prox(GFIFO_L);

LDI data, 0xFF;
CALL read_reg_prox; get the right register val
PUSH data;

;**************
;clear the rest of the FIFO so that the Gesture interupt can be reset, I am only using the first set of Gesture Data, which acounts for some of the detection in acuracy
LDI data, 0xAE; get the G esture level which tells us the number of 
CALL read_reg_prox; get the right register val
INC data; increase data by 1 so once the counter hits the level + 1 it has read all of the FIFO data
PUSH data; save our gesture level to the stack
CLR data; set 0 for our counter
PUSH data; save counter to the stack

Clear_FIFO:
POP data; pull counter off stack
pop temp; pull gesture level off stack
CP temp, data ; check to see if counter has reached 
BREQ cleared
PUSH temp; put g level on stack
PUSH data; put counter on stack

LDI data, 0xFC; 
CALL read_reg_prox;
LDI data, 0xFD; 
CALL read_reg_prox;
LDI data, 0xFE; 
CALL read_reg_prox;
LDI data, 0xFF; 
CALL read_reg_prox;

POP data; pop counter off stack
INC data; add 1
PUSH data; put back on the stack


rjmp Clear_FIFO;
;**************

;retrever up/down/left/right
cleared:
POP R19; POP the first set of UP DOWN LEFT RIGHT data off the stack
POP R18;
POP R17;
POP R16;

CP R16, R17;
BRGE downcheck;
SUB R16,R17;
CPI R16, 0x00; see if diference is 0
BREQ LR_check; if difference is 0 gesture was not and up or down gesture
CPI R16, 0x10; see if diference is greater then 16
BRSH U_trigger;

rjmp LR_check;

downcheck:
SUB R16,R17;
CPI R16, 0x00; see if diference is 0
BREQ LR_check; if difference is 0 gesture was not and up or down gesture
CPI R16, 0x10; see if diference is greater then 16
BRSH D_trigger;

LR_check: ;*****LR check is really strugaling at the moment, needs further investigation
CP R18, R19;
BRGE right;
SUB R18,R19;
CPI R18, 0x00; see if diference is 0
BREQ loop2; if difference is 0 gesture was not a left right gesture
CPI R18, 0x02; see if diference is greater then 2
BRSH L_trigger;

jmp loop;

right:
SUB R18,R19;
CPI R18, 0x00; see if diference is 0
BREQ loop2; if difference is 0 gesture was not a left right gesture
CPI R18, 0x02; see if diference is greater then 2
BRSH R_trigger;

jmp loop;


U_trigger:
LDI R16, 0x01;
;OUT 0x18, R16;
jmp loop2;

D_trigger:
LDI R16, 0x02;
;OUT 0x18, R16;
jmp loop2;

L_trigger:
LDI R16, 0x04;
;OUT 0x18, R16;
jmp loop2;

R_trigger:
LDI R16, 0x08;
;OUT 0x18, R16;
jmp loop2;

loop2:

jmp servo_move_valid;

ret;

;*******************end soft I2C sub routines/start of servo sub routines

;SETUP PMW FOR SERVOS
servo_init:

LDI temp, 0xFF; 
OUT 0x2A, temp ; OCR1AL set from temp
LDI data, 0x00; 
OUT 0x2B, data ; OCR1AH set from tempb

OUT 0x28, temp ; OCR1BL set from temp
OUT 0x29, data ; OCR1BH set from tempb

LDI temp, 0xA2;
OUT 0x2F, temp; TCCR1A|=(1<<COM1A1)|(1<<COM1B1)|(1<<WGM11);        //NON Inverted PWM
LDI temp, 0x1B; 
OUT 0x2E, temp;TCCR1B|=(1<<WGM13)|(1<<WGM12)|(1<<CS11)|(1<<CS10); //PRESCALER=/64 MODE 14(FAST PWM) includes progrAMIBLE TOP value

LDI temp, 0x0E; 
OUT 0x27, temp ;  ICR1=3749; ICR1H =   //fPWM=50Hz (Period = 20ms Standard). 
LDI temp, 0xA5; 
OUT 0x26, temp ;  ICR1=3749; ICR1L = A5  //fPWM=50Hz (Period = 20ms Standard). 


LDI temp, 0x30; 
OUT 0x11, temp  ;   DDRD|=(1<<PD4)|(1<<PD5);   //PWM Pins as Out from PD4 and PD4 (A and B)*/

RET; return
;**************************************************
;Deg to tick, remember to move xpos or ypos to temp before calling this
deg_to_tick:
LDI ZH, high(DEG2TICK<<1)	; Initialize Z-pointer to address of our conversion table
LDI ZL, low(DEG2TICK<<1)	; assumes the table wont be above 255 in program memory, risky?
ADD ZL, temp; temp is our address in memory of the conversion table
ADD ZL, temp; but the values are doubled up so we need to step 2x along the array
LPM temp, Z; grab the lowest bit of the position
INC ZL;
LPM data, Z;grab the highest bit of the position

RET;
;*************************************************
;**************************************************
;move servo1
mov_servo_1:

LDI data_out, 0x00;
OUT 0x2F, data_out; TCCR1A|=(1<<COM1A1)|(1<<COM1B1)|(1<<WGM11);        //NON Inverted PWM 
OUT 0x2E, data_out;TCCR1B|=(1<<WGM13)|(1<<WGM12)|(1<<CS11)|(1<<CS10); //PRESCALER=/64 MODE 14(FAST PWM) includes progrAMIBLE TOP value

OUT 0x2B, data ; OCR1AH set from tempb
OUT 0x2A, temp ; OCR1AL set from temp

LDI data_out, 0xA2;
OUT 0x2F, data_out; TCCR1A|=(1<<COM1A1)|(1<<COM1B1)|(1<<WGM11);        //NON Inverted PWM
LDI data_out, 0x1B; 
OUT 0x2E, data_out;TCCR1B|=(1<<WGM13)|(1<<WGM12)|(1<<CS11)|(1<<CS10); //PRESCALER=/64 MODE 14(FAST PWM) includes progrAMIBLE TOP value


RET;
;*************************************************
;**************************************************
;move servo2
mov_servo_2:


LDI data_out, 0x00;
OUT 0x2F, data_out; TCCR1A|=(1<<COM1A1)|(1<<COM1B1)|(1<<WGM11);        //NON Inverted PWM
OUT 0x2E, data_out;TCCR1B|=(1<<WGM13)|(1<<WGM12)|(1<<CS11)|(1<<CS10); //PRESCALER=/64 MODE 14(FAST PWM) includes progrAMIBLE TOP value


OUT 0x29, data ; OCR1BH set from tempb
OUT 0x28, temp ; OCR1BL set from temp

LDI data_out, 0xA2;
OUT 0x2F, data_out; TCCR1A|=(1<<COM1A1)|(1<<COM1B1)|(1<<WGM11);        //NON Inverted PWM
LDI data_out, 0x1B; 
OUT 0x2E, data_out;TCCR1B|=(1<<WGM13)|(1<<WGM12)|(1<<CS11)|(1<<CS10); //PRESCALER=/64 MODE 14(FAST PWM) includes progrAMIBLE TOP value

RET;
;*************************************************
delay_1s:
	ldi R18, 50
delay2:
	ldi R17, 200
delay1:
	ldi R16, 199
delay0:
	nop
	dec R16
	brne delay0
	nop
	dec R17
	brne delay1
	dec R18
	brne delay2
ret
;***************************************************
;This array contains the conversion from degrees to clock ticks for servo 1 and 2, values listed low then high byte// goes from 0deg to 180deg in steps of 10 degrees
DEG2TICK:
  .DB   188, 0, 205, 0, 222, 0, 238, 0, 255, 0, 16 , 1, 33, 1, 50, 1, 66, 1, 83, 1, 100, 1, 133, 1, 150, 1, 167, 1, 184, 1, 201, 1, 217, 1, 234, 1
 
 ;This is where the defalt servo position data is stored
 xyPOS: 
 .DB	3, 12

 .exit 		