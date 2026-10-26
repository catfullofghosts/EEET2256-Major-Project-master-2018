//*********************************************************************
// EEET2256 Major Project, Robert D Jordan s3136906, 2018
// Implementation in C of gesture controlled pan tilt camera system
//
//*********************************************************************
#include <asf.h>
#include <math.h>
#define F_CPU 12000000UL
#include<avr/io.h>

//** I2C SETUP STUFF
#define SCLPORT	PORTD	//PORTD as SCL OUTPUT WRITE
#define SCLDDR	DDRD	// DDRB as SCL 
#define SDAPORT	PORTD	// PORTD as SDA OUTPUT WRITE
#define SDADDR	DDRD	// PORTD as SDA 
#define SDAPIN	PIND	// PORTD pin def
#define SCLPIN	PIND	// PORTD pin def
#define SCL	PD1		//PORTD 1 PIN AS SOFT SCL PIN
#define SDA	PD0		//PORTD 0 PIN AS SOFT SDA PIN


#define softI2C_sda_defLOW	SDADDR|=((1<<SDA)) // define high/low states for SDA
#define softI2C_sda_defHIGH	SDADDR&=(~(1<<SDA)) 

#define softI2C_scl_defLOW	SCLDDR|=((1<<SCL)) // define high/low states for SCL
#define softI2C_scl_defHIGH	SCLDDR&=(~(1<<SCL))

#define Q_DEL _delay_loop_2(3) // soft delay
#define H_DEL _delay_loop_2(5) // soft delay number 2

//***********************************************************


//******************************************Includes and libs
#include <asf.h>
#include <avr/io.h> //IO
#include <util/delay.h> //Timerless delay used by soft I2C
#include <avr/wdt.h> // Watchdog timer used for soft reset
//**********************************************************

//************************************** PROXCIMITY SENDOR ADDRESS DEFINITION

#define prox_sensor_add (0x39)

//************************************** REGISTER DEFINITIONS FOR PROX SENSOR
enum
{
	RAM = 0x00,
	ENABLE = 0x80,
	ATIME = 0x81,
	WTIME = 0x83,
	AILTIL = 0x84,
	AILTH = 0x85,
	AIHTL = 0x86,
	AIHTH = 0x87,
	PILT = 0x89,
	PIHT = 0x8B,
	PERS = 0x8C,
	CONFIG1 = 0x8D,
	PPULSE = 0x8E,
	CONTROL = 0x8F,
	CONFIG2 = 0x90,
	ID = 0x92,
	STATUS = 0x93,
	CDATAL = 0x94,
	CDATAH = 0x95,
	RDATAL = 0x96,
	RDATAH = 0x97,
	GDATAL = 0x98,
	GDATAH = 0x99,
	BDATAL = 0x9A,
	BDATAH = 0x9B,
	PDATA = 0x9C,
	POFFSET_UR = 0x9D,
	POFFSET_DL = 0x9E,
	CONFIG3 = 0x9F,
	GPENTH = 0xA0,
	GEXTH = 0xA1,
	GCONF1 = 0xA2,
	GCONF2 = 0xA3,
	GOFFSET_U = 0xA4,
	GOFFSET_D = 0xA5,
	GOFFSET_L = 0xA7,
	GOFFSET_R = 0xA9,
	GPULSE = 0xA6,
	GCONF3 = 0xAA,
	GCONF4 = 0xAB,
	GFLVL = 0xAE,
	GSTATUS = 0xAF,
	IFORCE = 0xE4,
	PICLEAR = 0xE5,
	CICLEAR = 0xE6,
	AICLEAR = 0xE7,
	GFIFO_U = 0xFC,
	GFIFO_D = 0xFD,
	GFIFO_L = 0xFE,
	GFIFO_R = 0xFF,
};
//**************************************

//**


//****************************************************************************************************************
// SERVO SETUP
//****************************************************************************************************************
int GAMMA = 1; // this is the gamma scaler for our easing function
int EASEOFFSET = 0; // this is the off set for the easing function in ticks

int x_pos = 60;
int y_pos = 120;

void PWM_init(void); //pwm initializer function
void PWM_on_off(bool state); // a function for turning the pwm control signal off
void PWMOutput(int dutyA, int dutyB) ; //PWM update function using raw clock ticks as argument
void DEGOutput(int degreesA, int degreesB); //PWM update function using degrees as argument
uint8_t Servo_trans(int servonum, int pos1, int pos2); // transion between 2 positions with ease in
void Servo_trans_FIXED(int servonum, int pos1, int pos2, int delay);// transition between 2 positions with a fixed speed
void SERVO_DELAY_T1(int dly); // a delay function to create easing for the servomovement for slower more natural movement, delay entered in ms
int con_deg2ticks(float degs); // a function to convert from degrees to clock ticks
float con_tick2degs(float ticks);

//****************************************************************************************************************

//**** GESTURE function prototypes
void SI2CInit();
void SI2CStart();
void SI2CStop();
uint8_t SI2CWriteByte(uint8_t data);
uint8_t SI2CReadByte(uint8_t ack);
uint8_t softI2C_read_reg_prox(uint8_t reg_add);
void softI2C_write_reg_prox(uint8_t reg_add, uint8_t data);
uint8_t UD_prox_detect();
uint8_t Gesture_DIR_DETECT();
void SENSOR_SETUP();
//****

int main()
{
PWM_init();//set up PWM output on TIMER1 A and B

DEGOutput( x_pos, y_pos); //set servo default positions

SENSOR_SETUP();
//setup gesture sensor registers
// set delay time
softI2C_write_reg_prox( WTIME,  0x00);
//
softI2C_write_reg_prox( PPULSE,  0x23); // set up proximity pulse length and pulse umber

softI2C_write_reg_prox( GPULSE,  0xAA); //set number of gesture pulses and pulse time to 10 pulses and 32us per pulse
// power on and turn on prox
softI2C_write_reg_prox( ENABLE,  0x65); // on / proximity/ gesture, prox interrupts <<use this one
softI2C_write_reg_prox( GPENTH,  0x10); // set gesture proximity entry threshold
softI2C_write_reg_prox( GEXTH,  0x3A); // set gesture proximity exit threshold

uint8_t VALID_MOVE = 1; // used to prevent reading a new gesture before the servo has finished moving

while(1)
{
	uint8_t dir_trig =	Gesture_DIR_DETECT(); // see if a gesture was detected
	 SERVO_DELAY_T1(1); // helps overcome the issue of a single gesture being read as multiple gestures
	 SERVO_DELAY_T1(1);
	 SERVO_DELAY_T1(1);
	if(dir_trig == 2)
	{
		VALID_MOVE = Servo_trans(1, x_pos, x_pos + 2);
		if(VALID_MOVE == 1)
		{
			x_pos += 2;
		}
		
	}
	if(dir_trig == 3)
	{
		VALID_MOVE = Servo_trans(1, x_pos, x_pos - 2);
		if(VALID_MOVE == 1)
		{
			x_pos -= 2;
		}
	}
	if(dir_trig == 1)
	{
		 VALID_MOVE = Servo_trans(2, y_pos, y_pos + 2);
		if(VALID_MOVE == 1)
		{
			y_pos -= 2;
		}
	
	}
	if(dir_trig == 0)
	{
		 VALID_MOVE = Servo_trans(2, y_pos, y_pos - 2);
		 if(VALID_MOVE == 1)
		 {
			y_pos += 2;
		 }
		  
	}

	
}

////Servo_trans_FIXED(2, 90, 130, 2);

 //Servo_trans(1, 50, 60);


 
}

//****************************************************************************************************************
//servo functions
//
void PWM_init()          //// reinitialization of timer 1 pwm
{
	
	//B7: FOC0   Force Output Compare					
	//B6 : WGM00  Wave form generation mode				
	//B5 : COM01  Compare Output Mode						
	//B4 : COM00  Compare Output Mode 						
	//B3 : WGM01  Wave form generation mode					
	//B2 : CS02   Clock Select								
	//B1 : CS01   Clock Select								
	//B0 : CS00   Clock Select								
	
   TCCR1A|=(1<<COM1A1)|(1<<COM1B1)|(1<<WGM11);        //NON Inverted PWM
   TCCR1B|=(1<<WGM13)|(1<<WGM12)|(1<<CS11)|(1<<CS10); //PRESCALER=/64 MODE 14(FAST PWM) includes progrAMIBLE TOP value

   ICR1=3749;  //fPWM=50Hz (Period = 20ms Standard).
   
   DDRD|=(1<<PD4)|(1<<PD5);   //PWM Pins as Out from PD4 and PD4 (A and B)
	
	//TCCR2 |= (1<<WGM00)|(3<<COM00)|(1<<CS22);//|(1<<CS10); //Setup timer 2
	//
	//DDRD|=(1<<PD7);          // TCCR1A as output pin 6 on portC???

}

void PWMOutput(int dutyA, int dutyB) //PWM update function with raw ticks as argument
{
	
	int pwm = dutyA;
	OCR1A=pwm;
	
	pwm = dutyB;
	OCR1B = pwm;
}

void DEGOutput(int degreesA, int degreesB) //PWM update function with degrees as argument
//HI-tek HS 422 servos have a 180' travel angle
{
	//convert degrees to PWM
	int PWMoutput = con_deg2ticks(degreesA);// convert degrees to ticks
	OCR1A = PWMoutput;
	
	PWMoutput = con_deg2ticks(degreesB);// convert degrees to ticks
	OCR1B = PWMoutput;
}

uint8_t Servo_trans(int servonum, int pos1, int pos2)/// position we want to go from and to with easing
{
	uint8_t VALID_MOVE = 1;
	if(servonum == 1)
	{
			if(pos2 < 10)
			{
				pos2 = 10;
				VALID_MOVE = 0;
			} else if (pos2 > 170)
			{
				pos2 = 170;
				VALID_MOVE = 0;
			}	
	}
	
		if(servonum == 2)
		{
			if(pos2 < 90)
			{
				pos2 = 90;
				VALID_MOVE = 0;
			} else if (pos2 > 160)
			{
				pos2 = 160;
				VALID_MOVE = 0;
			}
		}

	
 int pwm1 = con_deg2ticks(pos1);
 int pwm2 = con_deg2ticks(pos2);


if (pwm1 < pwm2)
{
		for(int i = pwm1; i <= pwm2; i++)
		{
				if (servonum == 1)
				{
					
						OCR1A = i;
						
						
					} else if (servonum == 2)
					{
					
						OCR1B = i;	
					}
					
					///easy ease
					float pos_now = con_tick2degs(i);
					float easeIN = pow((pos_now - pos2), 3)/(pos1 - pos2) / 100;				
					int ezy = abs(easeIN);
				   SERVO_DELAY_T1(ezy);
				 
					///
		}
}

if (pwm1 >  pwm2)
{
	for(int i = pwm1; i >= pwm2; i--)
	{
		if (servonum == 1)
		{
		
			OCR1A = i;
			
		} else if (servonum == 2)
		{
		
			OCR1B = i;
		}
		
		/////easy ease
					///easy ease
					float pos_now = con_tick2degs(i);
					float easeIN = pow((pos_now - pos2), 3)/(pos1 - pos2) / 130;
					int ezy = abs(easeIN);
					SERVO_DELAY_T1(ezy);
				
					/////
	}
}

return VALID_MOVE;	
}

void Servo_trans_FIXED(int servonum, int pos1, int pos2, int delay)/// position we want to go from and to with easing
{
		if(servonum == 1)
		{
			if(pos2 < 1)
			{
				pos2 = 1;
			} else if (pos2 > 179)
			{
				pos2 = 179;
			}
		}
		
		if(servonum == 2)
		{
			if(pos2 < 79)
			{
				pos2 = 79;
			} else if (pos2 > 179)
			{
				pos2 = 179;
			}
		}
	
	int pwm1 = con_deg2ticks(pos1);
	int pwm2 = con_deg2ticks(pos2);


	if (pwm1 < pwm2)
	{
		for(int i = pwm1; i <= pwm2; i++)
		{
			if (servonum == 1)
			{
				
				OCR1A = i;
				
				
			} else if (servonum == 2)
			{
				
				OCR1B = i;
			}
			
			///easy ease
			SERVO_DELAY_T1(delay);
			///
		}
	}

	if (pwm1 >  pwm2)
	{
		for(int i = pwm1; i >= pwm2; i--)
		{
			if (servonum == 1)
			{
				
				OCR1A = i;
				
			} else if (servonum == 2)
			{
				
				OCR1B = i;
			}
			


			SERVO_DELAY_T1(delay);
			/////
		}
	}

	
}

void SERVO_DELAY_T1(int dly)// a delay function using the timer to allow smoother transitions between positions entered in ms avoid high numers that create overflow
{
	TCCR0 = (1<<CS02) | (1<<CS00); /* Timer0, normal mode, /1024 prescalar */
	//TCNT0 = dly*18;  		/* Load delay into TCNT0*/ //18 = aprox 1 ms delay
	TCNT0 = dly;  		
	

	while((TIFR&0x01)==0);  /* Wait for TOV0 to roll over */
	TCCR0 = 0;
	TIFR = 0x1;  		/* Clear TOV0 flag*/
	
	
	
	return;
}

int con_deg2ticks(float degs) // convert servo position from degrees to timer clock ticks
{
	float ticks = (degs/180)*302 + 188;// convert degrees to ticks (input in degrees/range on input)*range of output + output range min
	int summed_ticks = ticks;
	return (summed_ticks);
}

float con_tick2degs(float ticks) // convert clock ticks back to degrees
{
	float degs = (ticks/302)*180 - 112.4;// convert degrees to ticks (input in degrees/range on input)*range of output + output range min
	float summed_degs = degs;
	return (summed_degs);
}


void PWM_on_off(bool state) // an unused function to turn the PWM off and on
{
	if (state == 0)
	{
		  
		   TCCR1B =0; //PRESCALER=/64 MODE 14(FAST PWM) includes progrAMIBLE TOP value
		  // PORTD =0;  ///bad practice FIX
		 
	

		   
	} else {
		   TCCR1A|=(1<<COM1A1)|(1<<COM1B1)|(1<<WGM11);        //NON Inverted PWM
		   TCCR1B|=(1<<WGM13)|(1<<WGM12)|(1<<CS11)|(1<<CS10); //PRESCALER=/64 MODE 14(FAST PWM) includes progrAMIBLE TOP value

		   ICR1=3749;  //fPWM=50Hz (Period = 20ms Standard).
	}
}

//****************************************************************************************************************

//****************************************************************************************************************
//Gesture functions/ soft I2C functions
//****************************************************************************************************************

void SI2CInit()
{
	SDAPORT&=(1<<SDA);
	SCLPORT&=(1<<SCL);
	
	softI2C_sda_defHIGH;
	softI2C_scl_defHIGH;
	
}
void SI2CStart()
{
	softI2C_scl_defHIGH;
	H_DEL;
	
	softI2C_sda_defLOW;
	H_DEL;
}

void SI2CStop()
{
	softI2C_sda_defLOW;
	H_DEL;
	softI2C_scl_defHIGH;
	Q_DEL;
	softI2C_sda_defHIGH;
	H_DEL;
}

uint8_t SI2CWriteByte(uint8_t data)
{
	
	uint8_t i;
	
	for(i=0;i<8;i++)
	{
		softI2C_scl_defLOW;
		Q_DEL;
		
		if(data & 0x80) // look at MSB and shift it out
		softI2C_sda_defHIGH;
		else
		softI2C_sda_defLOW;
		
		H_DEL;
		
		softI2C_scl_defHIGH;
		H_DEL;
		
		while((SCLPIN & (1<<SCL))==0);// wait for the clock
		
		data=data<<1; // shift the data along as we are shifting out the MSB we need to LSL till all 8 bits are passed out from the MSB pos
	}
	
	//9th clock = ACK
	softI2C_scl_defLOW;
	Q_DEL;
	
	softI2C_sda_defHIGH;
	H_DEL;
	
	softI2C_scl_defHIGH;
	H_DEL;
	
	uint8_t ack=!(SDAPIN & (1<<SDA));
	
	softI2C_scl_defLOW;
	H_DEL;
	
	return ack;
	
}


uint8_t SI2CReadByte(uint8_t ack)
{
	uint8_t data=0x00; // clear the read in buffer
	uint8_t i;
	
	for(i=0;i<8;i++) // same as the write out but shifted in the oposite direction
	{
		
		softI2C_scl_defLOW;
		H_DEL;
		softI2C_scl_defHIGH;
		H_DEL;
		
		while((SCLPIN & (1<<SCL))==0);
		
		if(SDAPIN &(1<<SDA))
		data|=(0x80>>i); // MSB is 1 shifted to the apropriate position.
		
	}
	
	softI2C_scl_defLOW;
	Q_DEL;						//Soft_I2C_Put_Ack
	
	if(ack)
	{
		softI2C_sda_defLOW;
	}
	else
	{
		softI2C_sda_defHIGH;
	}
	H_DEL;
	
	softI2C_scl_defHIGH;
	H_DEL;
	
	softI2C_scl_defLOW;
	H_DEL;
	
	return data;
	
}



uint8_t softI2C_read_reg_prox(uint8_t reg_add) //all wrapped up into one function
{
	uint8_t data =  0;

	board_init();
	SI2CInit();
	SI2CStart();
	SI2CWriteByte(0x39 << 1);// slave address
	SI2CWriteByte(reg_add);// reg address///
	SI2CStart();
	SI2CWriteByte((0x39 << 1)+1);// slave address///
	data =  SI2CReadByte(0);
	SI2CStop();
	return data;
}

void softI2C_write_reg_prox(uint8_t reg_add, uint8_t data) //all wrapped up into one function
{

	board_init();
	SI2CInit();
	SI2CStart();
	SI2CWriteByte(0x39 << 1);// slave address
	SI2CWriteByte(reg_add);// reg address///
	SI2CWriteByte(data);// reg address///
	SI2CStop();
	
}

uint8_t UD_prox_detect() // function for detecting gesture via polling the prox register (yeah, i know....... was running out of time, better done then done well)
{
	uint8_t DIRECTION = 4; // 0 = up, 1 = down, 2 - left, 3 = right // 4 = non valid direction
	uint8_t UPCOUNT = 0;
	uint8_t DOWNCOUNT = 0;

	
	for (uint8_t i = 0; i < 25; i++)
	{
		uint8_t ROLLING_UP = 50;
		uint8_t ROLLING_DOWN = 0;
		
		for (uint8_t i = 0; i < 10; i++)
		
		{
			softI2C_write_reg_prox( CONFIG3,  0x08);
			uint8_t up_read = softI2C_read_reg_prox(PDATA);
			ROLLING_UP = (ROLLING_UP + up_read) / 2;
			H_DEL;
			H_DEL;
			H_DEL;
			H_DEL;
			H_DEL;
			H_DEL;
			
			softI2C_write_reg_prox( CONFIG3,  0x04);
			uint8_t down_read = softI2C_read_reg_prox(PDATA);
			ROLLING_DOWN = (ROLLING_DOWN + down_read) / 2;
			H_DEL;
			H_DEL;
			H_DEL;
			H_DEL;
			H_DEL;
			H_DEL;
			
		}
		
		if((ROLLING_UP > ROLLING_DOWN))
		{
			UPCOUNT++;
			continue;
			} else if((ROLLING_UP < ROLLING_DOWN)&& (ROLLING_DOWN > 5)) {
			DOWNCOUNT++;
			continue;
		}
		
	}
	
	if((UPCOUNT > DOWNCOUNT) && (UPCOUNT > 3))
	{
		DIRECTION = 0;
	}
	else
	if((DOWNCOUNT > UPCOUNT) && (DOWNCOUNT > 5))
	{
		DIRECTION = 1;
	}

	softI2C_write_reg_prox( CONFIG3,  0x0F); // turn all IR sensors back on
	
	return	DIRECTION;
	//return DIRECTION;
}

uint8_t Gesture_DIR_DETECT() // function for detecting gesture via polling the prox register (yeah, i know....... was running out of time, better done then done well)
{
		uint8_t triggered = 0;
		int DIR_OUT = 4;
		while(triggered == 0)
		{
			// FIFO must be fully read to reset the gesture interrupt!!!!!!
			uint8_t interupt1 = (softI2C_read_reg_prox(STATUS) & 0x04); // bitwise & the statis register with the gesture interupt bit
			if((interupt1 > 0) && (triggered == 0))
			{
				
				uint8_t GES_FIFO_BYTES = softI2C_read_reg_prox(GFLVL); //number of 4 byte data sets in FIFO
				uint8_t FIFO[GES_FIFO_BYTES*4];
				for(uint8_t i = 0; i < (GES_FIFO_BYTES*4) ; i = i + 4) // read out all the data into an array
				{
					FIFO[i] = softI2C_read_reg_prox(GFIFO_U);
					FIFO[i+1] = softI2C_read_reg_prox(GFIFO_D);
					FIFO[i+2] = softI2C_read_reg_prox(GFIFO_L);
					FIFO[i+3] = softI2C_read_reg_prox(GFIFO_R);
				}
				
				int  UCount = 0;
				int  DCount = 0;
				int  LCount = 0;
				int  RCount = 0;
				int up_down_diff = 0;
				int left_right_diff = 0;
				
				
				
				
				if(abs(((int)FIFO[0] - (int)FIFO[1])) > 15)
				{
					up_down_diff = (int)FIFO[0] - (int)FIFO[1];
				}
				
				
				if(abs(((int)FIFO[2] - (int)FIFO[3])) > 60)
				{
					left_right_diff = (int)FIFO[2] - (int)FIFO[3];
				}
				
				
				if(up_down_diff != 0){
					if(up_down_diff < 0)
					{
						PORTB = 0x01;
						DIR_OUT = 0;
						triggered = 1;
						
					}
					
					if(up_down_diff > 0)
					{
						PORTB = 0x02;
						DIR_OUT = 1;
						triggered = 1;
						
					}
					
				}
				
	
				
				
				if(left_right_diff != 0){
					if(left_right_diff < 0)
					{
						PORTB = 0x04;
						DIR_OUT = 2;
						triggered = 1;
						
					}
					
					if(left_right_diff > 0)
					{
						PORTB = 0x08;
						DIR_OUT = 3;
						triggered = 1;
						
					}
				}
				

				
				
				
			}
			
		}
	return DIR_OUT;
}


void SENSOR_SETUP() // a function to check the sensor is connected and soft reset via watchdog if not
{
	uint8_t data =  softI2C_read_reg_prox(ID);
	if (data != 0xAB) // check correct device id is returned.
	{
		//error
		DDRB = 0xff;
		PORTB = 0xFF;//
		H_DEL;

		for (unsigned int i = 0; i < 65534 ; i++)
		{
			H_DEL;
			H_DEL;
			H_DEL;
		}
		
		//sort reset
		wdt_enable(WDTO_15MS);
		for(;;)
		{
		}
		while(0){}
	}
	//softI2C_write_reg_prox();
	
	DDRB = 0xff;
	PORTB = data;/// return should
}



//****************************************************************************************************************