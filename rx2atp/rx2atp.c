#define	AT2SO	1
/*
 * what: rx2at - RC receiver servo signal to ASCII conversion
 * who:	 miru
 * when: April 2011...
 */
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
const char version[] PROGMEM = "0.21 20130723";

#define	RX_CON	0	/* 0 for standard mount, 1 for reversed mount */
#define	IN_AUX	1	/* 1 for AUX1 channel inversion (legacy...) */
#define	NL_AIR	0	/* 1 to turn off LED when airborne */
#define	EN_BMP	0	/* 1 for BMP085 support */
#define	EN_HMC	0	/* 1 for HMC5883L support */

/* switch setup, S_GEAR-(2 pos), S_AUX1-(2 or 3 pos) *
 * S_LAND S_FMOD
 * S_AUX1 S_AUX1 one 3 position switch (e.g. DX7)
 * S_GEAR S_GEAR one 2 position switch
 * S_GEAR S_AUX1 two switches (e.g. DX6)
 * S_AUX1 S_GEAR two switches */
#define	S_LAND	S_AUX1
#define	S_FMOD	S_AUX1

/* transmitter mode, channel assignments of the sticks
 * T_MODE 1 -> left: V-ELEV H-RUDD right: V-THRO H-AILE
 * T_MODE 4 -> left: V-THRO H-AILE right: V-ELEV H-RUDD
 * T_MODE 2 -> left: V-THRO H-RUDD right: V-ELEV H-AILE, US common mode
 * T_MODE 3 -> left: V-ELEV H-AILE right: V-THRO H-RUDD, US reversed */
#define	T_MODE	2

/* drone configuration choices
 * outdoor:     FALSE or TRUE, use (0 or 1)
 * no_shell:    FALSE or TRUE, use (0 or 1)
 * max_euler:   0.0 ... 0.52 max pitch/roll angle [rad]
 * max_vz:      200 ... 2000 max climb speed [mm/s]
 * max_yaw      0.7 ... 6.11 max yaw speed [rad/s]
 * max_alt:     altitude limit [m] (0 is off)
 * animation:   0 - disable, enabled otherwise
 * videorecord: 0 - disable, 1 - enable (NOOP on Drone 1 or Drone 2 without USB stick)
 *            outdoor,no_shell,max_euler,max_vz,max_yaw,max_alt,anim,rvid */
const char cfg1[] PROGMEM = "1,1,0.35,1500,3.5,0,1,1";	/* S standard */
const char cfg2[] PROGMEM = "1,1,0.52,2000,6.1,0,1,1";	/* W wild (max) */
const char cfg3[] PROGMEM = "0,0,0.21,700,1.75,2,0,0";	/* N normal */
const char cfg4[] PROGMEM = "0,0,0.10,700,1.50,2,0,0";	/* E easy */

/* Visible Low Battery Alert
 * the program can watch the battery capacity (percentage) on the drone, if it
 * goes below VLBA_THR the VLBA mechanism triggers. The drone's LEDs flash in RED
 * and VLBA output gets activated (blinking or steady), set to 0 to turn VLBA off */
#define	VLBA_THR	15	/* 0 or > 60 turns it off */
#define	VLBA_POL	1	/* 1-active high, 0-active low */
#define	VLBA_BLINK	1	/* 1-blink, 0-no blink */

/* You can route the GEAR channel status to an output to switch something ON/OFF. */
#define	GSWO_ENA	1	/* -1 disable, 0 active low, 1 active high */

/* Receiver error signaling delay (seconds)
 * max ~8.3 seconds, depends on LOOPHZ, set to 0 for immediate action (old way) */
#define	MERX		S2LTIC(0.5)

#define	LOOPHZ	30	/* loop frequency */
#define	S2LTIC(s)	(u08_t)((double)LOOPHZ*(double)(s)+0.5)


#if defined(__AVR_ATmega328P__)
#define	U4	0
/* Arduino Nano and ProMini */
/* LABEL PORT  DIR description
 *   RX0 PD.0  IN  UART receive
 *   TX1 PD.1  OUT UART transmit
 *   D2  PD.2  IN  RX AUX1 channel
 *   D3  PD.3  IN  RX GEAR channel
 *   D4  PD.4  IN  RX RUDD channel
 *   D5  PD.5  IN  RX ELEV channel
 *   D6  PD.6  IN  RX AILE channel
 *   D7  PD.7  IN  RX THRO channel
 * LABEL PORT  DIR description
 *   D8  PB.0
 *   D9  PB.1  OUT VLBA
 *   D10 PB.2  OUT GSWO (if GSWO_ENA >= 0)
 *   D11 PB.3  IN  SETUP input
 *   D12 PB.4
 *   D13 PB.5  OUT (LED,   Arduino comes this way)
 *   -   PB.6  -   (quarz, Arduino comes this way)
 *   -   PB.7  -   (quarz, Arduino comes this way)
 * LABEL PORT  DIR description
 *   A0  PC.0  IN  GPS RX
 *   A1  PC.1  OUT GPS TX
 *   A2  PC.2
 *   A3  PC.3  
 *   A4  PC.4  I/O (SDA two wire interface)
 *   A5  PC.5  I/O (SCL two wire interface)
 *   A6  -         (analog input 6, Nano and newer ProMini)
 *   A7  -         (analog input 7, Nano and newer ProMini)
 *
 * TIMER0: 'soft' serial port for GPS
 * TIMER1: system clock
 * TIMER2: not used
 */
#define	SIO_UDR		UDR0
#define	SIO_RX_VECT	USART_RX_vect
#define	SIO_TX_VECT	USART_UDRE_vect
#define	SIO_UCSRA	UCSR0A
#define	SIO_UCSRB	UCSR0B
#define	SIO_UCSRC	UCSR0C
#define	SIO_UBRR	UBRR0
#define	SIO_U2X		U2X0
#define	SIO_UDRIE	UDRIE0
#define	SIO_RXCIE	RXCIE0
#define	SIO_RXEN	RXEN0
#define	SIO_TXEN	TXEN0
#define	PIN_CHG_IRQEN()	PCICR |= _BV(PCIE2)|_BV(PCIE1)

#elif defined(__AVR_ATmega32U4__)
#define	U4	1
/* MiruPCB board layout
 * LABEL PORT  notes         DIR description
 *       PB.0
 * B1    PB.1  (SCK)         IN  RX THRO
 * B2    PB.2  (MOSI)        IN  RX AILE
 * B3    PB.3  (MISO)        IN  RX ELEV
 * B4    PB.4                IN  RX RUDD
 * B5    PB.5                IN  RX GEAR
 * B6    PB.6                IN  RX AUX1
 * RST   PB.7                (O) RESET*
 * LABEL PORT  notes         DIR description
 *       PC.6
 * LED   PC.7  LED green     OUT activity LED
 * LABEL PORT  notes         DIR description
 * SCL   PD.0  (SCL)         I/O (two wire interface)
 * SDA   PD.1  (SDA)         I/O (two wire interface)
 *       PD.2  (UART RX)     IN  UART receive
 *       PD.3  (UART TX)     OUT UART transmit
 * GTX   PD.4                OUT GPS  transmit
 *       PD.5
 * VLBA  PD.6                OUT VLBA
 * GSWO  PD.7                OUT GSWO
 * LABEL PORT  notes         DIR description
 *       PE.2  (HWB)
 * GRX   PE.6  (INT6)        IN  GPS receive
 * LABEL PORT  notes         DIR description
 *       PF.0
 *       PF.1
 *       PF.4
 *       PF.5
 *       PF.6               (OUT debug signal 1)
 *       PF.7               (OUT debug signal 0)
 *
 * TIMER0: 'soft' serial port for GPS
 * TIMER1: system clock
 * TIMER2: not used
 */
#define	SIO_UDR		UDR1
#define	SIO_RX_VECT	USART1_RX_vect
#define	SIO_TX_VECT	USART1_UDRE_vect
#define	SIO_UCSRA	UCSR1A
#define	SIO_UCSRB	UCSR1B
#define	SIO_UCSRC	UCSR1C
#define	SIO_UBRR	UBRR1
#define	SIO_U2X		U2X1
#define	SIO_UDRIE	UDRIE1
#define	SIO_RXCIE	RXCIE1
#define	SIO_RXEN	RXEN1
#define	SIO_TXEN	TXEN1
#define	PIN_CHG_IRQEN()	PCICR |= _BV(PCIE0), EIFR |= _BV(6)

#else
#error "sorry, processor not supported"
#endif

/* NOOPs, get redefined if needed */
#define	usb_sq()
#define	i2c_setup()
#define	bmp_sq()
#define	bmp_rp(upd)	0
#define	hmc_sq()
#define	hmc_rp(upd)	0

void port_setup(void)
{
#if	U4 == 0		/* is Arduino Nano or ProMini */
	PORTB = 0b00000000;
	DDRB  = 0b00000000;

/* (label 13 on Arduino) HW comes with a LED on this pin */
#define	LED	_BV(5)
#define	LEDON()	PORTB |=  LED
#define	LEDOF()	PORTB &= ~LED
#define	LEDTG()	PORTB ^=  LED
	DDRB |= LED;	/* output */

/* (label 11 on Arduino) ground for SETUP */
	PORTB |= _BV(3);	/* enable pull-up */
#define	SETUP() ((PINB & _BV(3)) == 0)

/* (label 10 on Arduino) reflect gear channel status if enabled */
#define	GSWO	_BV(2)
#if	GSWO_ENA >= 0
	#if	GSWO_ENA > 0
	#define	GSWON()	PORTB |=  GSWO
	#define	GSWOF()	PORTB &= ~GSWO
	#else
	#define	GSWON()	PORTB &= ~GSWO
	#define	GSWOF()	PORTB |=  GSWO
	#endif
	DDRB |= GSWO;	/* output */
	GSWOF();
#else
	#define	GSWON()
	#define	GSWOF()
#endif

/* (label  9 on Arduino) VLBA output pin */
#define	VLBA	_BV(1)
#if	VLBA_POL == 1
#define	VLBON()	PORTB |=  VLBA
#define	VLBOF()	PORTB &= ~VLBA
#else
#define	VLBOF()	PORTB |=  VLBA
#define	VLBON()	PORTB &= ~VLBA
#endif
#define	VLBTG()	PORTB ^=  VLBA
	VLBOF();
	DDRB |= VLBA;	/* output */

/* I2C interface */
	PORTC = 0b00000000;
	PORTC |= _BV(5)|_BV(4);	/* enable pullups for SDA/SCL */
	DDRC  = 0b00000000;

/* (label A0 on Arduino) GPS serial port in */
#define	GPS_RXI_VECT	PCINT1_vect
#define	GPS_RXI	_BV(0)
#define	GPS_RXI_INPUT()	(PINC & GPS_RXI)
#define	GPS_RXI_IRQON()	PCMSK1 |=  GPS_RXI
#define	GPS_RXI_IRQOF()	PCMSK1 &= ~GPS_RXI
	PORTC |=  GPS_RXI;	/* enable pullup on RXI */

/* (label A1 on Arduino) GPS serial port out */
#define	GPS_TXO	_BV(1)
#define	GPS_TXO_SET()	PORTC |=  GPS_TXO
#define	GPS_TXO_CLR()	PORTC &= ~GPS_TXO
#define	GPS_TXO_VAL()	(PORTC & GPS_TXO)
	DDRC  |=  GPS_TXO;	/* TXO is output */
	PORTC |=  GPS_TXO;	/* set TXO idle */

#define	RXSIG_VECT	PCINT2_vect
#define	RXSIG		PIND
#define	RXMSK		(_BV(7)|_BV(6)|_BV(5)|_BV(4)|_BV(3)|_BV(2))
	PCMSK2 = RXMSK;
	PORTD =  RXMSK|03;   	/* enable pull-ups for receiver inputs, and UART signals */
	DDRD  = 0b00000000;


#else	/* is MiruPCB */
#define	RXSIG_VECT	PCINT0_vect
#define	RXSIG		PINB
#define	RXMSK		(_BV(6)|_BV(5)|_BV(4)|_BV(3)|_BV(2)|_BV(1))
	PCMSK0 = RXMSK;
	PORTB  = RXMSK;	/* enable pull-ups for receiver and GPS inputs */
	DDRB   = 0b00000000;

	PORTC = 0b00000000;
	DDRC  = 0b00000000;
/* HW comes with a LED on this pin */
#define LED	_BV(7)
#define	LEDON()	PORTC |=  LED
#define	LEDOF()	PORTC &= ~LED
#define	LEDTG()	PORTC ^=  LED
	DDRC  |= LED;	/* output */

/* I2C interface */
	PORTD = 0b00001111;   	/* enable pull-ups for UART and I2C signals */
	DDRD  = 0b00000000;
/* GSWO output pin */
#define	GSWO	_BV(7)
#if	GSWO_ENA >= 0
	#if	GSWO_ENA > 0
	#define	GSWON()	PORTD |=  GSWO
	#define	GSWOF()	PORTD &= ~GSWO
	#else
	#define	GSWON()	PORTD &= ~GSWO
	#define	GSWOF()	PORTD |=  GSWO
	#endif
	DDRD  |= GSWO;	/* output */
#else
	#define	GSWON()
	#define	GSWOF()
#endif
	GSWOF();
/* VLBA output pin */
#define	VLBA	_BV(6)
#if	VLBA_POL == 1
#define	VLBON()	PORTD |=  VLBA
#define	VLBOF()	PORTD &= ~VLBA
#else
#define	VLBOF()	PORTD |=  VLBA
#define	VLBON()	PORTD &= ~VLBA
#endif
#define	VLBTG()	PORTD ^=  VLBA
	VLBOF();
	DDRD  |= VLBA;		/* output */
/* GPS serial port out */
#define	GPS_TXO	_BV(4)
#define	GPS_TXO_SET()	PORTD |=  GPS_TXO
#define	GPS_TXO_CLR()	PORTD &= ~GPS_TXO
#define	GPS_TXO_VAL()	(PORTD & GPS_TXO)
	PORTD |= GPS_TXO;	/* set TXO idle */
	DDRD  |= GPS_TXO;	/* output */

	PORTE = 0b00000000;
/* GPS serial port in */
#define	GPS_RXI_VECT	INT6_vect
#define	GPS_RXI	_BV(6)
#define	GPS_RXI_INPUT()	(PINE & GPS_RXI)
#define	GPS_RXI_IRQON()	EIMSK |=  GPS_RXI
#define	GPS_RXI_IRQOF()	EIMSK &= ~GPS_RXI
	PORTE |=  GPS_RXI;	/* enable pullup on RXI */
	DDRE  = 0b00000000;
	EICRB = 0b00010000;	/* -,-,isc61,ISC60,-,-,-,- */

	PORTF = 0b00000000;
	DDRF  = 0b00000000;
#endif

	MCUCR = 0b00000000;	/* PUD=0 clear global pull-up disable */
}

void sio_setup(void)
{
	SIO_UCSRB = 0b00000000;	/* rxcie0,txcie0,udrie0,rxen0,txen0,ucsz02,rxb80,txb80 */
	SIO_UCSRC = 0b00000110;	/* umsel01,umsel00,upm01,upm00,usbs0,UCSZ01,UCSZ00,ucpol0 */
	SIO_UCSRA = 0b00000000;	/* rxc0,txc0,udre0,fe0,dor0,upe0,u2x0,mpcm0 */
#if	F_CPU == 16000000UL	/* (see NOTES USART) */
	SIO_UBRR  = 16;
	SIO_UCSRA |= _BV(SIO_U2X);
#else
	SIO_UBRR  = (F_CPU/(16UL*115200UL)-1);
#endif
}

void tmr1_setup(void)
{
#define	F_TM1		(F_CPU/8UL)	/* (see NOTES TIMER1) */
#define	MS2TIC(ms)	(u32_t)((double)F_TM1*(double)(ms)*1e-3)
#define	TIC2MS(tic)	(u32_t)((tic)/(F_TM1/1000L))
#define	TIC2MS10(tic)	(u32_t)((tic)/(F_TM1/10000L))
#define	TIC2US(tic)	(u32_t)((tic)/(F_TM1/1000000L))

	TCCR1A = 0b00000000;	/* com1a1,com1a0,com1b1,com1b0,-,-,wgm11,wgm10 */
	TCCR1B = 0b00000010;	/* icnc1,ices1,-,wgm13,wgm12,cs12,CS11,cs10 F_CPU/8 */
	TCCR1C = 0b00000000;	/* foc1a,foc1b,-,-,-,-,-,- */
	TIMSK1 = 0b00000001;	/* -,-,icie1,-,-,ocie1b,ocie1a,TOIE1 */
}

#define	NEL(x)		(sizeof(x)/sizeof(x[0]))
#define	ABS(x)		((x)>=0?(x):-(x))
#define	MIN(a,b)	((a)<=(b)?(a):(b))
/* circular buffers */
#define	CB_FILL(r,w,s)	(((w)-(r)+(s))%(s))	/* available elements for get */
#define	CB_FREE(r,w,s)	(((r)-(w)+(s)-1)%(s))	/* available elements for put */
#define	CB_HASD(r,w)	((r)!=(w))		/* has data */
#define	CB_NO_D(r,w)	((r)==(w))		/* has no data */

typedef char		s08_t;
typedef volatile s08_t	vs08_t;
typedef unsigned char	u08_t;
typedef volatile u08_t	v08_t;
typedef unsigned short	u16_t;
typedef volatile u16_t	v16_t;
typedef unsigned long	u32_t;

typedef struct {
	u32_t	dcst;	/* drone status */
	char	cbat;	/* battery percentage left */
	char	sgps;	/* GPS status */
}	__attribute__ ((packed)) m2a_t;

typedef struct {	/* radio signal sample */
	u08_t	smp;
	u32_t	tmr;
}	__attribute__ ((packed)) rsm_t;

struct {
	v16_t	t1ov;	/* timer 1 overflow count */
	v16_t	dcnt;	/* delay count for ms_dly() */
	u08_t	exit;
	u08_t	alin;

	struct { /* UART is connected to drone */
		/* circular receive buffer (r == w -> empty) */
		v08_t	rxw;
		v08_t	rxr;
		u08_t	rxb[32];
		/* circular transmit buffer (r == w -> empty) */
		v08_t	txw;
		v08_t	txr;
		u08_t	txb[200];
	}	sio;

#if	U4
	struct { /* FTDI simulation */
		u08_t	ep0a;	/* interface active */
		u08_t	ep0r;	/* EP0 received */
		union {	/* setup request packet */
			u08_t	b[8];
			struct {
				u08_t	bmRequestType;
				#define REQ_DIR	0x80
				#define  REQ_H2D	0x00
				#define  REQ_D2H	0x80
				#define REQ_TYP	0x60
				#define  REQ_STD	0x00
				#define  REQ_CLS	0x20
				#define  REQ_VEN	0x40
				#define REQ_DST	0x03
				#define  REQ_DEV	0x00
				#define  REQ_IFC	0x01
				#define  REQ_EPT	0x02
				#define  REQ_OTH	0x03
				u08_t	bRequest;
				#define GET_STATUS	0
				#define CLR_FEA		1
				#define SET_FEA		3
				#define SET_ADR		5
				#define GET_DSC		6
				#define SET_DSC		7
				#define GET_CFG		8
				#define SET_CFG		9
				#define GET_IFC		10
				#define SET_IFC		11
				u16_t	wValue;
				u16_t	wIndex;
				u16_t	wLength;
			};
		}	ep0b;
		/* EP1 management dev->host */
		u08_t	sofi;
		v08_t	ep1h;
		v08_t	ep1r;
		v08_t	ep1w;
		u08_t	ep1b[200];
		/* EP2 management host->dev */
		v08_t	ep2r;

		/* FTDI simulator */
		u08_t	mdst;	/* modem status */
		#define	MDST_RLSD	_BV(7)	/* receive line signal */
		#define	MDST_RI		_BV(6)	/* ring indicator */
		#define	MDST_DSR	_BV(5)	/* data set ready */
		#define	MDST_CTS	_BV(4)	/* clear to send */
		#define	RST_MDST	(MDST_DSR|MDST_CTS|_BV(0))
		u08_t	lnst;	/* line status */
		#define	LNST_FIFO	_BV(7)	/* fifo error */
		#define	LNST_TEMT	_BV(6)	/* transmitter empty */
		#define	LNST_THRE	_BV(5)	/* transmit holding register enpty */
		#define	LNST_BI		_BV(4)	/* break interrupt */
		#define	LNST_FE		_BV(3)	/* framing error */
		#define	LNST_PE		_BV(2)	/* parity error */
		#define	LNST_OE		_BV(1)	/* overrun */
		#define	LNST_DTR	_BV(0)	/* data terminal ready */
		#define	RST_LNST	(LNST_TEMT|LNST_THRE|LNST_DTR)
		u08_t	rbsq;	/* reboot sequencer */
		u16_t	tlat;	/* latency timer */
	}	usb;
#endif

	struct { /* messages from drone */
		v08_t	ack;
		v08_t	cst;
		v08_t	nda;
		union {
			u08_t	b[0];
			m2a_t	m2a;
		}	dat;
		m2a_t	m2a;
	}	drm;

	struct { /* 'soft' serial port for GPS */
		int	b100;
		u08_t	tcnt;
		u08_t	thbt;
		u16_t	lstp;	/* sample of timer 1 at last stop bit */
		/* receiver */
		v08_t	rxi;	/* last value of input GPS_RXI */
		u08_t	rxm;	/* data bit mask register */
		u08_t	rxd;	/* data shift register */
		v08_t	rxw;
		v08_t	rxr;
		/* transmitter */
		u08_t	txm;
		u08_t	txd;
		v08_t	txe;
		v08_t	txw;
		v08_t	txr;
		/* GPS data assembly */
		u08_t	cst;
		u08_t	cks;
		u08_t	seq;
		u08_t	b_r;
		u08_t	b_w;
		u08_t	b_s;
		u08_t	b_n;
		/* buffers */
		union {
			u32_t	rxt[95];	/* for baudrate detection */
			struct {
			u08_t	rxb[96];	/* buffers 24.7 ms on 38400 baud stream */
			u08_t	txb[32];
			char	buf[252];
			};
		};
	}	gps;

	struct {
		u08_t	stat;
		u08_t	tick;
	}	vlba;

	struct { /* RC receiver signals connected to port D */
		u08_t	fms;	/* flight mode (rx_read()) */
#define	FMS_LAND	0
#define	FMS_FM_1	1	/* use camera for stabilization on ROL/PTC sticks centered */
#define	FMS_FM_2	2	/* idle up, don't use camera on ROL/PTC stick centered */
#define	FMS_FM_3	3	/* auto pilot mode */
		u08_t	fsq;	/* flight mode sequencer */
		u08_t	fst;	/* flight mode sequencer tic */
		u08_t	fsa;	/* flight mode sequencer animation trigger */
		u08_t	eft;	/* signals with frame time error */
		u08_t	ept;	/* signals with pulse time error */
		u08_t	erx;	/* consecutive error count rx_read() */

		u08_t	stk;	/* stick status (rx_read()) */
		u08_t	chg;	/* stick status change (rx_read()) */
#define	TS_RT	0x01
#define	TS_LF	0x02
#define	TS_UP	0x04
#define	TS_DN	0x08
#define	ES_RT	0x10
#define	ES_LF	0x20
#define	ES_UP	0x40
#define	ES_DN	0x80

		u08_t	smp;	/* last signal sample */
		u08_t	rsr;	/* read index radio sampling buffer */
		u08_t	rsw;	/* write index radio sampling buffer */
		rsm_t	rsb[32];
#define	S_AILE	0
#define	S_ELEV	1
#define	S_THRO	2
#define	S_RUDD	3
#define	S_AUX1	4
#define	S_GEAR	5
#define	S_NCHN	6
/* Note: independent of TX mode 1-4, throttle and elevator channels are never on the
 * same physical stick and are always controlled by vertical movement of the stick */
#if   T_MODE == 1 || T_MODE == 4
	#define	SH_THRO	S_AILE
	#define	SH_ELEV	S_RUDD
#else
	#define	SH_THRO	S_RUDD
	#define	SH_ELEV	S_AILE
#endif
#define	STKTHR	500	/* throttle/elevator stick threshold for switching */
		struct chn {
			u08_t	msk; /* port D mask for signal */
			u32_t	tup; /* timer1 when it went up */
			u32_t	ftm; /* time between the last two ups */
			int	dbn; /* signal dead band */
			int	dur; /* how long signal was up */
			int	val; /* interpreted value (rx_read()) */
		}	chn[S_NCHN];
	}	rxs;
/* RX channel mapping to drone */
#define S_ROL	S_AILE
#define S_PTC	S_ELEV
#define S_GAZ	S_THRO
#define S_YAW	S_RUDD
/* RX channels used */
#if	GSWO_ENA >= 0
#define	S_SIG	(_BV(S_ROL)|_BV(S_PTC)|_BV(S_GAZ)|_BV(S_YAW)|_BV(S_AUX1)|_BV(S_GEAR))
#else
#define	S_SIG	(_BV(S_ROL)|_BV(S_PTC)|_BV(S_GAZ)|_BV(S_YAW)|_BV(S_LAND)|_BV(S_FMOD))
#endif

	/* drone program in flash */
	struct {
		u16_t	adr;
		u16_t	siz;
		u16_t	cks;
	}	arm;

	u08_t	dsnd;	/* snd_sc() destination, 0 sio, 1 usb */
	char	pad;	/* drone control <0 emergency, >0 fly, else land */
	char	cfw;	/* configuration wanted by telling from the sticks */
	u08_t	eewen;	/* EEROM */
	union {
		u08_t	eedat[6];
		struct {
			u16_t	eemag;
			u16_t	bid;	/* boot id */
			u08_t	cfg;
			u08_t	eecks;
		};
	};
#define	EE_MAG	0x2701
}	gl;

/*
 * How the 'soft' serial port for GPS works:
 * TIMER0 (8bit) is setup so it rolls on RS232 bit boundaries, this provides all that's
 * needed to do a transmitter with the timer rollover interrupt. The interrupt is turned
 * off when there is nothing to transmit.
 * To build a receiver at the same baudrate, the 1->0 transition of the RX input is used
 * to set the second compare register of the timer with the current timer value+some,
 * after that the compare interrupt becomes the sampling point for the RX input. Once a
 * byte is received, the RX transition interrupt is reenabled to wait for the next byte.
 */
void tmr0_setup(int b100)	/* argument is baudrate/100 */
{
	u16_t	cnt;
	u08_t	ccr;

	ccr = SREG; cli();
	TIMSK0 = 0b00000000;	/* -,-,-,-,-,ocie0b,ocie0a,toie0 */
	TCCR0A = 0b00000000;	/* com0a1,com0a0,com0b1,com0b0,-,-,wgm01,wgm00  */
	TCCR0B = 0b00000000;	/* foc0a,foc0b,-,-,wgm02,cs02,cs01,cs00  OFF */
	SREG = ccr;
	gl.gps.rxw = gl.gps.rxr = 0;
	gl.gps.txw = gl.gps.txr = 0;
	gl.gps.txe = 0;
	gl.gps.b100 = 0;
	gl.gps.tcnt = 0;
	gl.gps.thbt = 0;
	if (b100 <= 0) return;
	/* turn it on for baudrate b100 * 100 */
	gl.gps.txe = 1;
	cnt = (F_CPU/100UL)/(long)b100;
	for (ccr = 1; cnt >= 210L; cnt >>= 3) ccr++;
	if (ccr >= 5) return;
	gl.gps.b100 = b100;
	gl.gps.tcnt = cnt - 1;
	/* sampling offset relative to 1->0 transition of start bit */
	if (gl.gps.tcnt > 80)
		gl.gps.thbt = (cnt*50)/100;
	else	gl.gps.thbt = (cnt*15)/100;
	OCR0A  = gl.gps.tcnt;
	TCCR0B = ccr;
	TCCR0A = 0b00000010;	/* com0a1,com0a0,com0b1,com0b0,-,-,WGM01,wgm00  CTC mode */
}

ISR(TIMER1_OVF_vect)
{
	gl.t1ov++;
}

inline void itic(u16_t *t)	/* called with interrupts disbled */
{
	t[1] = gl.t1ov;
	t[0] = TCNT1;
	if ((TIFR1 & _BV(TOV1)) && t[0] < 0xffff) t[1]++;
}

u32_t tic(void)
{
	union { u16_t uw[2]; u32_t ul; } t;
	u08_t	srg;

	srg = SREG; cli();
	itic(t.uw);
	SREG = srg;
	return t.ul;
}

void ms_dly(int ndcnt)
{
	int	i;

	for (; ndcnt; ndcnt--)
		for (i = gl.dcnt; --i >= 0; )
			__asm__ __volatile__ (" nop");
}

void msdly_cali(void)
{
	u32_t	tmr;

	gl.dcnt = 2000;
	tmr = tic();
	ms_dly(100);
	tmr = tic() - tmr;
	gl.dcnt = (int)((200UL*F_TM1)/tmr);
}

void blip(unsigned char nb)
{
	while (nb--) {
		LEDON(); ms_dly( 20);
		LEDOF(); ms_dly(180);
	}
}

/*
 * EEPROM
 */
void eepr_save(void)
{
	u08_t	i;

	gl.eemag = EE_MAG;
	gl.eecks = 0;
	for (i = 0; i < (NEL(gl.eedat)-1); i++) gl.eecks += gl.eedat[i];
	gl.eewen = 0;
}

void eepr_updt(void)
{
	u08_t	t;

	if ((EECR & _BV(EEPE)) == 0) {
		EEAR = (int)gl.eewen;
		EEDR = gl.eedat[gl.eewen];
		t = SREG; cli();
		EECR = _BV(EEMPE);
		EECR |= _BV(EEPE);
		SREG = t;
		gl.eewen++;
	}
}

/*
 * uart receiver
 */
ISR(SIO_RX_VECT) /* receiver interrupt */
{
	u08_t	i;

	gl.sio.rxb[i = gl.sio.rxw] = SIO_UDR;
	if (++i >= NEL(gl.sio.rxb)) i = 0;
	if (i != gl.sio.rxr) gl.sio.rxw = i;
}

int sio_rc(void) /* read next character received */
{
	u08_t	i;
	int	b;

	if (CB_NO_D(gl.sio.rxr,gl.sio.rxw)) return -1;
	b = (int)gl.sio.rxb[i = gl.sio.rxr];
	gl.sio.rxr = ++i >= NEL(gl.sio.rxb) ? 0 : i;
	return b;
}

char sio_rf(void) /* flush sio receive buffer */
{
	if (gl.sio.rxr == gl.sio.rxw) return 0;
	gl.sio.rxr = gl.sio.rxw;
	return 1;
}

/*
 * uart transmitter
 */
ISR(SIO_TX_VECT) /* tx empty interrupt */
{
	u08_t	i;

	if (CB_HASD(gl.sio.txr,gl.sio.txw)) {
		SIO_UDR = gl.sio.txb[i = gl.sio.txr];
		gl.sio.txr = ++i >= NEL(gl.sio.txb) ? 0 : i;
	}
	if (CB_NO_D(gl.sio.txr,gl.sio.txw))
		SIO_UCSRB &= ~_BV(SIO_UDRIE); /* no more, disable TX IRQ */
}

void sio_sc(char c) /* send character */
{
	u08_t	i;

	gl.sio.txb[i = gl.sio.txw] = c;
	if (++i >= NEL(gl.sio.txb)) i = 0;
	if (i != gl.sio.txr) {
		gl.sio.txw = i;
		SIO_UCSRB |= _BV(SIO_UDRIE); /* enable TX IRQ */
	}
}

u08_t sio_tf(void) /* txb free space */
{
	return CB_FREE(gl.sio.txr,gl.sio.txw,NEL(gl.sio.txb));
}

void gps_rcv(void);

void sio_tw(void) /* wait for tx empty */
{
	do gps_rcv();
	while (CB_HASD(gl.sio.txr,gl.sio.txw));
}

void sio_ss(char *s) /* send string from ram */
{
	char	c;

	while ((c = *s++)) sio_sc(c);
}

void sio_sp(const char *p) /* send string from flash */
{
	char	c;

	while ((c = pgm_read_byte(p++))) sio_sc(c);
}

#define	SIO_SP(str)	sio_sp(PSTR(str))

void sio_rev(void)
{
	u08_t	n, k;

	for (n = 0; n < 5; n++) {
		k = pgm_read_byte(&version[n]);
		if (k == '.') continue;
		if (k == ' ') break;
		sio_sc(k);
	}
}

void snd_sc(char c)
{
#if	U4
	void usb_sc(char c);
	if (gl.dsnd) usb_sc(c); else
#endif
	sio_sc(c);
}

/*
 * convert to ASCII and transmit
 */
/* send hex */
void snd_snx(u08_t n) { n &= 0x0f; if (n <= 9) snd_sc('0'+n); else snd_sc('A'-10+n); }
void snd_sbx(u08_t b) { snd_snx(b>>4); snd_snx(b>>0); }
void snd_swx(u16_t w) { snd_sbx(w>>8); snd_sbx(w>>0); }

void snd_sid(char lc, int i) /* integer decimal */
{
	u08_t	fw, nd, sn, b[16];

	fw = sn = nd = 0;
	if (lc > 0) snd_sc(lc);		/* 'lc' is leading character */
	else if (lc < 0) fw = -lc;	/* 'lc' is field width */
	if (i < 0) i = -i, sn = '-';	/* sign */
	do b[nd++] = (i%10) + '0'; while ((i /= 10));
	if (sn) b[nd++] = sn;
	for (sn = nd; sn < fw; sn++) snd_sc(' ');
	do snd_sc(b[--nd]); while (nd);
}

void snd_spd(u08_t fw, u08_t pr, long i) /* pseudo decimal */
{
	u08_t	nd, sn, b[20];

	if (pr > 16) pr = 16;
	sn = nd = 0;
	if (i < 0) i = -i, sn = '-';
	do {
		b[nd++] = (i%10) + '0';
		if (nd == pr) b[nd++] = '.';
	}
	while ((i /= 10) || nd < pr);
	if (pr && (nd - pr) == 1) b[nd++] = '0';
	if (sn) b[nd++] = '-';
	while (nd < fw) fw--, snd_sc(' ');
	do snd_sc(b[--nd]); while (nd);
}

void snd_sp(const char *p) /* string from flash */
{
	char	c;

	while ((c = pgm_read_byte(p++))) snd_sc(c);
}

#define	SND_SP(str)	snd_sp(PSTR(str))

/*
 * 'soft' serial port for GPS
 */
ISR(TIMER0_COMPA_vect)
{
	u08_t	i;

	/* transmit bit interrupt */
	if (gl.gps.txm == 0) {
		if (GPS_TXO_VAL()) { /* was stop bit or pause */
			if (gl.gps.txr != gl.gps.txw) {
				gl.gps.txd = gl.gps.txb[i = gl.gps.txr];
				gl.gps.txr = ++i >= NEL(gl.gps.txb) ? 0 : i;
				GPS_TXO_CLR();	/* send start bit */
			}
			else {
				TIMSK0 &= ~_BV(OCIE0A);	/* compa interrupt OFF */
				gl.gps.txe = 1;
			}
		}
		else { /* was start bit */
			gl.gps.txm = 1;
			if (gl.gps.txd & 01) GPS_TXO_SET();
		}
	}
	else {
		gl.gps.txe = 0;
		gl.gps.txm <<= 1;
		if (gl.gps.txm == 0 || (gl.gps.txd & gl.gps.txm))
			GPS_TXO_SET();
		else	GPS_TXO_CLR();
	}
}

ISR(TIMER0_COMPB_vect)
{
	u08_t	i;

	/* receive bit interrupt */
	i = GPS_RXI_INPUT();
	if (gl.gps.rxm == 0xff) { /* start bit */
		if (i == 0) {
			gl.gps.rxd = 0;
			gl.gps.rxm = 1;
		}
	}
	else {
		if (i) gl.gps.rxd |= gl.gps.rxm;
		if (gl.gps.rxm) gl.gps.rxm <<= 1;	/* more bits to sample */
		else {					/* just sampled stop bit */
			TIMSK0 &= ~_BV(OCIE0B);		/* compb interrupt OFF */
			if (i) {			/* stop bit is valid */
				gl.gps.rxb[i = gl.gps.rxw] = gl.gps.rxd;
				if (++i >= NEL(gl.gps.rxb)) i = 0;
				if (i != gl.gps.rxr) gl.gps.rxw = i;
				gl.gps.rxi |= GPS_RXI;
			}
			GPS_RXI_IRQON();
			gl.gps.lstp = TCNT1;
		}
	}
}

ISR(GPS_RXI_VECT)
{
	u08_t	rxi, tmp;

	rxi = GPS_RXI_INPUT();		/* sample input */
	tmp = rxi ^ gl.gps.rxi;		/* compare to last sample */
	gl.gps.rxi = rxi;		/* save input */
	if (gl.gps.tcnt == 0) {		/* sample timer for figuring out baudrate */
		if (gl.gps.rxw < NEL(gl.gps.rxt)) {
			itic((u16_t *)&gl.gps.rxt[gl.gps.rxw]);
			gl.gps.rxw++;
		}
	}
	else if (tmp && rxi == 0) {	/* it changed and is 0 now -> received start bit */
		tmp = TCNT0;
		tmp += gl.gps.thbt;
		if (tmp == gl.gps.tcnt) tmp++;
		if (tmp > gl.gps.tcnt) tmp -= gl.gps.tcnt;
		if (tmp < 1) tmp++;
		OCR0B = tmp;		/* set sampling point */
		TIFR0  |= _BV(OCF0B);	/* clear in case it is set */
		TIMSK0 |= _BV(OCIE0B);	/* compB interrupt ON */
		GPS_RXI_IRQOF();
		gl.gps.rxm = 0xff;	/* tell timer irq COMPB, next sample is start bit */
	}
}

void gps_sc(char c) /* send character */
{
	u08_t	i;

	gl.gps.txb[i = gl.gps.txw] = c;
	if (++i >= NEL(gl.gps.txb)) i = 0;
	if (i == gl.gps.txr) return;
	gl.gps.txw = i;
	if (TIMSK0 & _BV(OCIE0A)) return;
	TIFR0  |= _BV(OCF0A);		/* clear compa flag */
	TIMSK0 |= _BV(OCIE0A);		/* compa interrupt ON */
}

void gps_snx(u08_t n) { if ((n &= 0x0f) <= 9) gps_sc('0'+n); else gps_sc('A'-10+n); }

void gps_snd(const char *p)
{
	u08_t	b, cks;

	gps_sc('$');
	for (cks = 0; (b = pgm_read_byte(p)); p++) cks ^= b, gps_sc(b);
	gps_sc('*');
	gps_snx(cks>>4); gps_snx(cks>>0);
	gps_sc('\r'); gps_sc('\n');
}

#define	GPS_SND(str)	gps_snd(PSTR(str))

void gps_bdt(void)
{
	int	b;
	u08_t	i;

	/* figure out baudrate from single bit timing, 4800...57600 is MAX! */
	if (gl.gps.rxw == NEL(gl.gps.rxt)) {
		b = 1000;
		for (i = NEL(gl.gps.rxt); --i > 5; ) { /* ignore first couple of samples */
			gl.gps.rxt[i] -= gl.gps.rxt[i-1];
			if (gl.gps.rxt[i] < b) b = gl.gps.rxt[i];
		}
		     if (b >= (int)MS2TIC(0.016) && b <= (int)MS2TIC(0.018)) b = 576;
		else if (b >= (int)MS2TIC(0.025) && b <= (int)MS2TIC(0.027)) b = 384;
		else if (b >= (int)MS2TIC(0.051) && b <= (int)MS2TIC(0.053)) b = 192;
		else if (b >= (int)MS2TIC(0.103) && b <= (int)MS2TIC(0.106)) b =  96;
		else if (b >= (int)MS2TIC(0.205) && b <= (int)MS2TIC(0.210)) b =  48;
		else b = 0;
		tmr0_setup(b);
		gl.gps.cst = 0;
		gl.gps.b_r = gl.gps.b_w = 0;
	}
}

void gps_rcv(void)
{
	u08_t	c, i;

	if (gl.gps.tcnt == 0) {
		gps_bdt();
		return;
	}
	while (1) {
		if (CB_NO_D(gl.gps.rxr,gl.gps.rxw)) break;
		c = (int)gl.gps.rxb[i = gl.gps.rxr];
		gl.gps.rxr = ++i >= NEL(gl.gps.rxb) ? 0 : i;
		/* NMEA: $..........*hh\r\n */
		switch (gl.gps.cst) {
		default:
GPSDROP0:		SND_SP("slog,GPSR,");
			if (i == gl.gps.b_r) gl.gps.cst |= 0x10; /* overflow */
			snd_sbx(gl.gps.cst);
			snd_sc(',');
			snd_sbx(c);
			snd_sc('\r');
GPSDROP1:		gl.gps.cst = 0;
			break;
		case 6: /* stays here until next '$' arrives */
		case 0: if (c != '$') break;
			gl.gps.cks = 0;
			i = gl.gps.b_s = gl.gps.b_w;
			gl.gps.buf[i] = c;
			if (++i >= NEL(gl.gps.buf)) i = 0;
			if (i == gl.gps.b_r) goto GPSDROP0;
			gl.gps.b_n = 1;
			gl.gps.b_s = i;
			gl.gps.cst = 1;
			break;
		case 1:
			gl.gps.buf[i = gl.gps.b_s] = c;
			if (++i >= NEL(gl.gps.buf)) i = 0;
			if (c < ' ' || c > '~' || i == gl.gps.b_r) goto GPSDROP0;
			if (gl.gps.b_n == 1 && c != 'G') goto GPSDROP1;
			if (gl.gps.b_n == 2 && c != 'P' && c != 'N') goto GPSDROP1;
			gl.gps.b_n++;
			gl.gps.b_s = i;
			if (c == '*') gl.gps.cst = 2;
			else gl.gps.cks ^= c;
			break;
		case 2: case 3:
			gl.gps.buf[i = gl.gps.b_s] = c;
			if (++i >= NEL(gl.gps.buf)) i = 0;
			     if (c >= '0' && c <= '9') c -= '0';
			else if (c >= 'A' && c <= 'F') c -= 'A'-10;
			if (c > 0x0f || i == gl.gps.b_r) goto GPSDROP0;
			gl.gps.b_n++;
			gl.gps.b_s = i;
			gl.gps.cst++;
			if (gl.gps.cst == 3) gl.gps.cks ^= c << 4;
			else if (gl.gps.cks != c) goto GPSDROP0;
			break;
		case 4:
			gl.gps.buf[i = gl.gps.b_s] = c;
			if (++i >= NEL(gl.gps.buf)) i = 0;
			if (c != '\r' || i == gl.gps.b_r) goto GPSDROP0;
			gl.gps.b_n++;
			gl.gps.b_s = i;
			gl.gps.cst = 5;
			break;
		case 5: if (c == '\n' && gl.gps.seq >= 10) gl.gps.b_w = gl.gps.b_s;
			gl.gps.cst = 6;
			break;
		}
	}
	if (gl.gps.cst !=  6) return;	/* message incomplete */
	if (gl.gps.seq >= 10) return;	/* good to go */
	gl.gps.b_r = gl.gps.b_w = 0;	/* kill received messages during setup */
	if (gl.gps.txr != gl.gps.txw || gl.gps.txe == 0) return; /* not done sending last cmd */
	if ((TCNT1 - gl.gps.lstp) < (u16_t)MS2TIC(10.0)) return; /* wait for gap in data stream */
	switch (gl.gps.seq) {
	case 0: /* check/set baudrate */
		if (gl.gps.b100 == 384)        gl.gps.seq = 2;
		else GPS_SND("PMTK251,38400"), gl.gps.seq = 1;
		break;
	case 1: tmr0_setup(0);
		gl.gps.seq = 2;
		break;
	/* get RMC,GGA sentences only */
	case 2: GPS_SND("PMTK314,0,5,0,1,0,0,0,0"); gl.gps.seq = 3; break;
	/* set rate to 200ms, 5Hz */
	case 3: GPS_SND("PMTK220,200");             gl.gps.seq = 4; break;
	/* done */
	case 4: gl.gps.seq = 10; break;
	}
}

#if	EN_BMP || EN_HMC
#undef	i2c_setup
/*
 * Two Wire Interface (I2C)
 * 400000 HZ, 9 bit/b -> 0.0225 ms/b
 * 100000 HZ, 9 bit/b -> 0.0900 ms/b
 * write: <adr>|0,<reg>,<dat>
 *  read: <adr>|0,<reg>,<adr>|1,<dat>[,<dat>[...]]
 */
struct {
	vs08_t	act;
	v08_t	srg;
	u08_t	adr;
	u08_t	reg;
	u08_t	dat;
	v08_t	irc;
	u08_t	nrc;
	v08_t	rcv[23];
}	gltwi;
#define	TWI_RD	_BV(0)

#define	TWICLK	400000	/* (Hz) */
void i2c_setup(void)
{
	TWSR = 0;				/* PRSV prescaler value = 1 */
	TWBR = (F_CPU/(2L*TWICLK)-8-1/2);	/* TWBR = F_CPU/(2*TWICLK) - 8 - PRSV/2 */
}

ISR(TWI_vect)
{
	u08_t	srg;

	srg = gltwi.srg;
	gltwi.srg = TWSR & 0xf8;
	switch (gltwi.act) {
	default:
		gltwi.srg = srg;
ERROR:
		if (gltwi.act > 0) gltwi.act *= -1;
		TWCR = _BV(TWINT)|_BV(TWSTO)|_BV(TWEN)|_BV(TWIE);
		break;
	case 1: /* START sent */
		if (gltwi.srg != 0x08) goto ERROR;
		TWDR = (gltwi.adr & ~TWI_RD);
		gltwi.act = 2;
		TWCR = _BV(TWINT)|_BV(TWEN)|_BV(TWIE);
		break;
	case 2: /* address-W sent */
		if (gltwi.srg != 0x18) goto ERROR;
		TWDR = gltwi.reg;
		gltwi.act = 3;
		TWCR = _BV(TWINT)|_BV(TWEN)|_BV(TWIE);
		break;
	case 3: /* register sent */
		if (gltwi.srg != 0x28) goto ERROR;
		if (gltwi.adr & TWI_RD) { /* read transaction */
			gltwi.act = 5;
			TWCR = _BV(TWINT)|_BV(TWSTA)|_BV(TWEN)|_BV(TWIE);
			break;
		}
		TWDR = gltwi.dat;
		gltwi.act = 4;
		TWCR = _BV(TWINT)|_BV(TWEN)|_BV(TWIE);
		break;

	case 4: /* data byte sent */
		if (gltwi.srg != 0x28) goto ERROR;
		gltwi.act = 8;
		TWCR = _BV(TWINT)|_BV(TWSTO)|_BV(TWEN)|_BV(TWIE);
		break;

	case 5: /* repeated START sent */
		if (gltwi.srg != 0x10) goto ERROR;
		TWDR = gltwi.adr;
		gltwi.act = 6;
		TWCR = _BV(TWINT)|_BV(TWEN)|_BV(TWIE);
		break;
	case 6: /* address-R sent */
		if (gltwi.srg != 0x40) goto ERROR;
		gltwi.act = 7;
		if (gltwi.nrc > 1)
			TWCR = _BV(TWINT)|_BV(TWEN)|_BV(TWIE)|_BV(TWEA);
		else	TWCR = _BV(TWINT)|_BV(TWEN)|_BV(TWIE);
		break;
	case 7: /* data byte received */
		if (gltwi.srg == 0x50) {
			gltwi.rcv[gltwi.irc++] = TWDR;
			if ((gltwi.nrc - gltwi.irc) > 1)
				TWCR = _BV(TWINT)|_BV(TWEN)|_BV(TWIE)|_BV(TWEA);
			else	TWCR = _BV(TWINT)|_BV(TWEN)|_BV(TWIE);
		}
		else if (gltwi.srg == 0x58) {
			gltwi.rcv[gltwi.irc++] = TWDR;
			gltwi.act = 8;
			TWCR = _BV(TWINT)|_BV(TWSTO)|_BV(TWEN)|_BV(TWIE);
		}
		else goto ERROR;
		break;
	}
}

void i2c_strt(void)
{
	gltwi.irc = 0;
	gltwi.srg = 0;
	gltwi.act = 1;
	TWCR = _BV(TWINT)|_BV(TWSTA)|_BV(TWEN)|_BV(TWIE);
}

void i2c_send(u08_t adr, u08_t reg, u08_t dat)
{
	gltwi.adr = (adr<<1);
	gltwi.reg = reg;
	gltwi.dat = dat;
	i2c_strt();
}

void i2c_recv(u08_t adr, u08_t reg, u08_t nda)
{
	gltwi.adr = (adr<<1)|TWI_RD;
	gltwi.reg = reg;
	gltwi.nrc = nda;
	i2c_strt();
}
#endif

#if	EN_BMP
#undef	bmp_sq
#undef	bmp_rp
	/* BMP085 pressure/temperature sensor on twi */
#define	A_BMP	0b1110111
#define	OSS	3
struct {
	vs08_t	seq;	
	u08_t	cid;	/* chip id: [0xd0] 0x55 */
	u08_t	idx;
	u08_t	cdn;
	u16_t	tmr;
	u32_t	res;	/* bmp085 result register */
	u08_t	upd;
	u16_t	ut;	/* raw temperature */
	long	up;	/* raw pressure */
	u16_t	cal[11];
}	glbmp;

u08_t bmp_rp(u08_t upd)
{
	u08_t	i;

	if (glbmp.upd & upd & 01) {
		SND_SP("BMPR");
		snd_sid(',',glbmp.seq);
		snd_sc(','); snd_spd(0,0,glbmp.ut);
		snd_sc(','); snd_spd(0,0,glbmp.up);
		glbmp.upd &= ~01;
	}
	if (glbmp.upd & upd & 02) {
		SND_SP("BMPC,"); snd_snx(OSS);
		for (i = 0; i < NEL(glbmp.cal); i++)
			snd_sc(','), snd_swx(glbmp.cal[i]);
		glbmp.upd &= ~02;
	}
	return glbmp.upd;
}

#define	BMPTIC		MS2TIC(0.25)
#define	MS2BMP(ms)	(MS2TIC(ms)/BMPTIC)

void bmp_sq(void)
{
	u16_t	dtm;
	u08_t	i;

	if (glbmp.cdn) {
		dtm = TCNT1 - glbmp.tmr;
		if (dtm < (u16_t)BMPTIC) return;
		glbmp.tmr = TCNT1;
		if ((glbmp.cdn -= 1)) return;
	}
	switch (glbmp.seq) {
	case  1: case  3: case  5: case  7:
		if (gltwi.act != 8) {
			glbmp.seq = -glbmp.seq;
			gltwi.act = 0;
			glbmp.upd |= 01;
			break;
		}
		if (glbmp.seq != 1) gltwi.act = 0;
		break;
	case  0: case  4: case  6:
		if (gltwi.act) return;
		break;
	}
	switch (glbmp.seq) {
	case 0: /* read chip id */
		i2c_recv(A_BMP,0xd0,1);
		glbmp.cdn = MS2BMP(0.25);
		glbmp.seq = 1;
		break;
	case 1: glbmp.cid = gltwi.rcv[0];
		if (glbmp.cid != 0x55) {
			gltwi.act = 0;
			glbmp.seq = -100;
			glbmp.upd |= 01;
			break;
		}
		/* read calibration registers */
		i2c_recv(A_BMP,0xaa,22);
		glbmp.cdn = MS2BMP(3.0);
		glbmp.seq = 3;
		break;
	case 3: for (i = 0; i < 11; i++) {
			glbmp.cal[i] =  gltwi.rcv[i*2+0]<<8;
			glbmp.cal[i] += gltwi.rcv[i*2+1];
		}
		glbmp.upd |= 02;
		glbmp.seq = 4;
		break;

	case 4: /* request data */
		i2c_send(A_BMP,0xf4,glbmp.idx?(0x34+(OSS<<6)):0x2e);
		glbmp.cdn = MS2BMP(0.5);
		glbmp.seq = 5;
		break;
	case 5: glbmp.cdn = glbmp.idx?MS2BMP(28.0):MS2BMP(6.0);
		glbmp.seq = 6;
		break;
	case 6: /* read result register */
		i2c_recv(A_BMP,0xf6,glbmp.idx?3:2);
		glbmp.cdn = MS2BMP(0.5);
		glbmp.seq = 7;
		break;
	case 7: glbmp.res = gltwi.rcv[0];
		glbmp.res <<= 8;
		glbmp.res |= gltwi.rcv[1];
		glbmp.idx++;
		if (glbmp.idx == 1) { /* temperature data */
			glbmp.ut = glbmp.res;
			glbmp.seq = 4;
			break;
		}
		/* pressure data */
		glbmp.res <<= 8;
		glbmp.res |= gltwi.rcv[2];
		glbmp.res >>= 8-OSS;
		glbmp.up = glbmp.res;
		glbmp.upd |= 01;
		if (glbmp.idx >= 10) glbmp.idx = 0;
		glbmp.cdn = MS2BMP(10.0);
		glbmp.seq = 8;
		break;
	case 8: if (glbmp.upd == 0) glbmp.seq = 4;
		break;
	}
	if (glbmp.cdn) glbmp.tmr = TCNT1;
}

#undef	BMPTIC
#undef	MS2BMP

#endif

#if	EN_HMC
#undef	hmc_sq
#undef	hmc_rp
#define	A_HMC	0b0011110
struct {
	vs08_t	seq;	
	u08_t	upd;
	u08_t	twi;
	u08_t	cdn;
	u16_t	tmr;
	int	bia[3];
	int	raw[3];
}	glhmc;

u08_t hmc_rp(u08_t upd)
{
	u08_t	i;

	if (glhmc.upd & upd & 01) {
		SND_SP("HMCR");
		snd_sid(',',glhmc.seq);
		for (i = 0; i < NEL(glhmc.raw); i++)
			snd_sc(','), snd_spd(0,0,glhmc.raw[i]);
		glhmc.upd &= ~01;
	}
	if (glhmc.upd & upd & 02) {
		SND_SP("HMCC");
		SND_SP(",1160,"), snd_spd(0,0,glhmc.bia[0]);
		SND_SP(",1160,"), snd_spd(0,0,glhmc.bia[1]);
		SND_SP(",1080,"), snd_spd(0,0,glhmc.bia[2]);
		glhmc.upd &= ~02;
	}
	return glhmc.upd;
}

#define	HMCTIC		MS2TIC(0.5)
#define	MS2HMC(ms)	(MS2TIC(ms)/HMCTIC)

void hmc_sq(void)
{
	union { int i; u16_t w; struct { u08_t lsb, msb; }; } t;

	if (glhmc.cdn) {
		t.w = TCNT1 - glhmc.tmr;
		if (t.w < (u16_t)HMCTIC) return;
		glhmc.tmr = TCNT1;
		if ((glhmc.cdn -= 1)) return;
	}
	if (glhmc.twi&01) { /* release i2c */
		if (gltwi.act != 8) {
			glhmc.seq = -glhmc.seq;
			glhmc.upd |= 01;
		}
		glhmc.twi = 0;
		gltwi.act = 0;
	}
	if (glhmc.seq == 0 || (glhmc.twi&02)) { /* acquire i2c */
		if (gltwi.act) return;
		glhmc.twi = 0;
	}
	switch (glhmc.seq) {
	case  0: /* read chip id */
		i2c_recv(A_HMC,10,3);
		glhmc.cdn = MS2HMC(0.5);
		glhmc.twi = 1;
		glhmc.seq = 1;
		break;
	case  1:
		if (gltwi.rcv[0] != 'H' || gltwi.rcv[1] != '4' || gltwi.rcv[2] != '3') {
			glhmc.seq = -100;
			glhmc.upd |= 01;
			break;
		}
		glhmc.twi = 2;
		glhmc.seq = 2;
		break;

	case  2: /* set CFG A */
		i2c_send(A_HMC,0x00,0b01110001); /* -,MA1,MA0,DO2,do1,do0,ms1,MS2  avg 8, upd 15 Hz, +bias */
		glhmc.cdn = MS2HMC(1.0);
		glhmc.twi = 1;
		glhmc.seq++;
		break;
	case  3: glhmc.seq++; glhmc.twi = 2; break;
	case  4: /* set CFG B */
		i2c_send(A_HMC,0x01,1<<5);	/* gain +-1.3Ga */
		glhmc.cdn = MS2HMC(1.0);
		glhmc.twi = 1;
		glhmc.seq++;
		break;
	case  5: glhmc.seq++; glhmc.twi = 2; break;

	case  6: case 26: /* single measurement */
		i2c_send(A_HMC,0x02,0x01);
		glhmc.cdn = MS2HMC(1.0);
		glhmc.twi = 1;
		glhmc.seq++;
		break;
	case  7: case 27: /* wait a while */
		glhmc.cdn = glhmc.seq == 7 ? MS2HMC(100.0) : MS2HMC(68);
		glhmc.twi = 2;
		glhmc.seq++;
		break;
	case  8: case 28: /* read result register */
		i2c_recv(A_HMC,0x03,6);
		glhmc.cdn = MS2HMC(1.0);
		glhmc.seq++;
		break;
	case  9: case 29: /* transfer raw values */
		t.msb = gltwi.rcv[0]; t.lsb = gltwi.rcv[1]; glhmc.raw[0] = t.i; /* X */
		t.msb = gltwi.rcv[2]; t.lsb = gltwi.rcv[3]; glhmc.raw[2] = t.i; /* Z */
		t.msb = gltwi.rcv[4]; t.lsb = gltwi.rcv[5]; glhmc.raw[1] = t.i; /* Y */
		glhmc.twi = 1;
		glhmc.seq++;
		break;
	case 10: /* transfer bias */
		glhmc.bia[0] = glhmc.raw[0];
		glhmc.bia[1] = glhmc.raw[1];
		glhmc.bia[2] = glhmc.raw[2];
		glhmc.upd |= 02;
		glhmc.twi = 2;
		glhmc.seq = 20;
		break;

	case 20: /* set CFG A */
		i2c_send(A_HMC,0x00,0b01110000); /* -,MA1,MA0,DO2,do1,do0,ms1,MS2  avg 8, upd 15 Hz, norm */
		glhmc.cdn = MS2HMC(1.0);
		glhmc.twi = 1;
		glhmc.seq++;
		break;
	case 21: glhmc.seq++; break;
	case 22: /* set CFG B */
		i2c_send(A_HMC,0x01,1<<5);	/* gain +-1.3Ga */
		glhmc.cdn = MS2HMC(1.0);
		glhmc.twi = 1;
		glhmc.seq++;
		break;
	case 23:
		glhmc.twi = 2;
		glhmc.seq = 26;
		break;

	case 30: /* measurements valid */
		glhmc.upd |= 01;
		glhmc.cdn = MS2HMC(1.0);
		glhmc.seq++;
		break;
	case 31: /* wait for measurement digest */
		if (glhmc.upd & 01) break;
		glhmc.seq = 26;
		glhmc.twi = 2;
		break;
	}
	if (glhmc.cdn) glhmc.tmr = TCNT1;
}

#undef	HMCTIC
#undef	MS2BMP

#endif

/*
 * RC receiver
 */
#define	PTMAX	MS2TIC(2.05)	/* pulse should be shorter than that */
#define	PTMIN	MS2TIC(0.95)	/* pulse should be longer than that */
#define	PTREF	MS2TIC(1.50)	/* nominal center position */
#define	PTRNG	MS2TIC(0.40)	/* +- nominal spread */

#define	FTMIN	MS2TIC( 8.0)	/* min frametime */
#define	FTMAX	MS2TIC(35.0)	/* max frametime */
#define	FTTOT	MS2TIC(250.0)	/* max frame time out */

ISR(RXSIG_VECT) /* RX signal change interrupt */
{
	union { rsm_t *r; u08_t *b; } p;
	u08_t	i;

	p.r = gl.rxs.rsb;
	i = gl.rxs.rsw;
	p.b += i * sizeof(*p.r);
	p.r->smp = RXSIG;
	itic((u16_t *)&p.r->tmr);
	if (++i >= NEL(gl.rxs.rsb)) i = 0;
	if (i != gl.rxs.rsr) gl.rxs.rsw = i;
}

void rx_proc(void)
{
	u32_t	tmr, dtm;
	char	k;
	u08_t	i, smp, chg, lsm;

	lsm = gl.rxs.smp;
	smp = SREG; cli(); i = gl.rxs.rsw; SREG = smp;
	k = CB_FILL(gl.rxs.rsr,i,NEL(gl.rxs.rsb));
	while (k--) {
		i = gl.rxs.rsr;
		smp = gl.rxs.rsb[i].smp;
		tmr = gl.rxs.rsb[i].tmr;
		gl.rxs.rsr = ++i >= NEL(gl.rxs.rsb) ? 0 : i;
		chg = (smp ^ lsm) & RXMSK;
		lsm = smp;
		for (i = 0; chg && i < NEL(gl.rxs.chn); i++)
			if (chg & gl.rxs.chn[i].msk) {
				dtm = tmr - gl.rxs.chn[i].tup;
				if (smp & gl.rxs.chn[i].msk)
					gl.rxs.chn[i].ftm = dtm, gl.rxs.chn[i].tup = tmr;
				else	gl.rxs.chn[i].dur = dtm;
				chg &= ~gl.rxs.chn[i].msk;
			}
	}
	gl.rxs.smp = lsm;
}

int rx_tk2v(int dur, int dbn)
{
	int	val;

	dur -= PTREF;
	if (dur >= 0) { if ((dur -= dbn) <= 0) return 0; }
	else          { if ((dur += dbn) >= 0) return 0; }
	val = (int)((long)(dur * 1000L + PTRNG/2)/(long)(PTRNG - dbn));
	val = val < -1000 ? -1000 : val > 1000 ? 1000 : val;
	return val;
}

u08_t rx_esel(int thr)
{
	int	v, h;
	u08_t	isp;

	isp = 0;
	if ((v = gl.rxs.chn[ S_ELEV].val) < 0) v *= -1; else isp |= 01;
	if ((h = gl.rxs.chn[SH_ELEV].val) < 0) h *= -1; else isp |= 02;
	if (v < thr && h < thr) return 0;
	return v >= h ? (isp & 01) ? 1 : 3 : (isp & 02) ? 4 : 2;
}

u08_t rx_read(void)
{
	u08_t	err, ept, mch;
	u32_t	tot;
	int	val;
	struct chn *c;

	rx_proc();
	ept = 0;
	err = 0;
	tot = tic();
	mch = _BV(NEL(gl.rxs.chn)-1);
	for (c = gl.rxs.chn + NEL(gl.rxs.chn); --c >= gl.rxs.chn; mch >>= 1)
		if (!(mch & S_SIG)) continue;	/* bit/channel is not used */
		else if ((tot - c->tup) > FTTOT) err |= mch;
		else if (c->ftm < FTMIN || c->ftm > FTMAX) err |= mch;
		else if (c->dur < PTMIN || c->dur > PTMAX) ept |= mch;
	gl.rxs.ept = ept;
	gl.rxs.eft = err;
	if ((err |= ept)) { /* there is a problem with one or more RX signals */
		if (gl.rxs.erx < MERX) {
			/* delay error reporting */
			gl.rxs.erx++;
			return 0;
		}
		GSWOF();
		for (c = gl.rxs.chn + NEL(gl.rxs.chn); --c >= gl.rxs.chn; ) c->val = 0;
		return err;
	}
	/* all RX signals ok */
	gl.rxs.erx = 0;
	mch = _BV(NEL(gl.rxs.chn)-1);
	for (c = gl.rxs.chn + NEL(gl.rxs.chn); --c >= gl.rxs.chn; mch >>= 1)
		c->val = rx_tk2v(c->dur,c->dbn);

	if (IN_AUX) gl.rxs.chn[S_AUX1].val *= -1;

	val = gl.rxs.chn[S_LAND].val;
	switch (S_LAND) {
	case S_GEAR: gl.rxs.fms =       val < 0 ? FMS_LAND : FMS_FM_1; break;
	case S_AUX1: gl.rxs.fms = val ? val < 0 ? FMS_LAND : FMS_FM_2 : FMS_FM_1; break;
	}
	if (S_FMOD != S_LAND && gl.rxs.fms != FMS_LAND) /* 2 switches and in the air */
		gl.rxs.fms = gl.rxs.chn[S_FMOD].val < 0 ? FMS_FM_1 : FMS_FM_2;

	/* extended flightmode sequencer */
	switch (gl.rxs.fsq) {
	default: gl.rxs.fsq = 0; break;
	case 0: if (gl.rxs.fms == FMS_FM_2) gl.rxs.fsq = 1; break;
	case 1: if (gl.rxs.fms == FMS_LAND) gl.rxs.fsq = 0;
		else if (gl.rxs.fms == FMS_FM_1) {
			/* switch to FM_1, arms trigger mechanism */
			gl.rxs.fst = S2LTIC(0.5);
			gl.rxs.fsq = 2;
			gl.rxs.fms = FMS_FM_2;
		}
		break;
	case 2: gl.rxs.fsq = 0;
		if ((gl.rxs.fst -= 1) == 0) break; /* timeout */
		if (gl.rxs.fms == FMS_LAND) break; /* land */
		if (gl.rxs.fms == FMS_FM_1) {
			/* keep FM_2 during timeout period */
			gl.rxs.fsq = 2;
			gl.rxs.fms = FMS_FM_2;
			break;
		}
		if (gl.rxs.fms != FMS_FM_2) break;
		/* user switched back to FM_2 before timeout */
		gl.rxs.fst = S2LTIC(0.40);	/* animation trigger window */
		gl.rxs.fsq = 3;
		break;
	case 3: if (gl.rxs.fms != FMS_FM_2) { /* switch changed */
			gl.rxs.fsq = 0;
			break;
		}
		if ((gl.rxs.fst -= 1) == 0) { /* enter FM_3 */
			gl.rxs.fms = FMS_FM_3;
			gl.rxs.fsq = 5;
			break;
		}
		if ((mch = rx_esel(100)) == 0) break;
		/* trigger animation */
		gl.rxs.fst = S2LTIC(0.5);
		gl.rxs.fsa = mch;
		gl.rxs.fsq = 4;
		break;
	case 4: if ((gl.rxs.fst -= 1) == 0 || gl.rxs.fsa == 0)
			gl.rxs.fsq = gl.rxs.fsa = 0;
		break;
	case 5: gl.rxs.fsq = 0;
		if (gl.rxs.fms != FMS_FM_2) break;	/* switch changed */
		if (rx_esel(150)) break;		/* stick moved */
		gl.rxs.fsq = 5;
		gl.rxs.fms = FMS_FM_3;
		break;
	}

	/* Note: no code if not enabled */
	if (GSWO_ENA >= 0 && gl.rxs.chn[S_GEAR].val < 0) GSWOF(); else GSWON();

	if (gl.rxs.fms == FMS_LAND) {
		mch = 0;
		val = gl.rxs.chn[ S_THRO].val; if (ABS(val) > STKTHR) mch |= val < 0 ? TS_DN : TS_UP;
		val = gl.rxs.chn[SH_THRO].val; if (ABS(val) > STKTHR) mch |= val < 0 ? TS_LF : TS_RT;
		val = gl.rxs.chn[ S_ELEV].val; if (ABS(val) > STKTHR) mch |= val < 0 ? ES_UP : ES_DN;
		val = gl.rxs.chn[SH_ELEV].val; if (ABS(val) > STKTHR) mch |= val < 0 ? ES_LF : ES_RT;
		gl.rxs.chg = gl.rxs.stk ^ mch;
		gl.rxs.stk = mch;
		/* alternate configuration selection */
		if (gl.rxs.stk & TS_UP)
			gl.cfw = (char)rx_esel(STKTHR) - 1;
		else {
			if (gl.cfw >= 0 && gl.cfg != gl.cfw && gl.eewen >= NEL(gl.eedat)) {
				gl.cfg = gl.cfw;
				eepr_save();
			}
			gl.cfw = -1;
		}
	}
	else gl.rxs.stk = 0, gl.cfw = -1;
	return 0;
}

/*
 * messages from drone
 */
#define	C_ACK	0x01
void sio_rcv(void)
{
	int	r;
	char	c;

	while ((r = sio_rc()) >= 0)
		if ((c = r) <= 0) gl.drm.cst = 0;
		else if (c == C_ACK) gl.drm.ack = 1, gl.drm.cst = 0;
		else switch (gl.drm.cst) {
		case 0: if (c == '$') gl.drm.cst = 1, gl.drm.nda = 0; break;
		case 1: if (c == '*') {
				if (gl.drm.nda == sizeof(gl.drm.dat.m2a))
					gl.drm.m2a = gl.drm.dat.m2a;
				gl.drm.cst = 0;
				break;
			}
			/* FALLTHROUGH */
		case 2: if (c >= '0' && c <= '9') c -= '0';
			else if (c >= 'A' && c <= 'F') c -= 'A'-10;
			else if (c >= 'a' && c <= 'f') c -= 'a'-10;
			else c = -1;
			if (c < 0 || gl.drm.nda >= sizeof(gl.drm.dat)) {
				gl.drm.cst = 0;
				break;
			}
			if (gl.drm.cst == 1)
				gl.drm.dat.b[gl.drm.nda] = (c<<4);
			else	gl.drm.dat.b[gl.drm.nda++] |= c;
			gl.drm.cst ^= 3;
			break;
		}
}

#if	U4
#undef	usb_sq
/*
 * MiruPCB USB interface
 */
typedef struct { /* Standard Device Descriptor */
	u08_t	bLength;
	u08_t	bDescriptorType;
	u16_t	bcdUSB;
	u08_t	bDeviceClass;
	u08_t	bDeviceSubClass;
	u08_t	bDeviceProtocol;
	u08_t	bMaxPacketSize0;
	u16_t	idVendor;
	u16_t	idProduct;
	u16_t	bcdDevice;
	u08_t	iManufacturer;
	u08_t	iProduct;
	u08_t	iSerialNumber;
	u08_t	bNumConfigurations;
}	usb_sdd_t; /* 18 bytes */

typedef struct { /* Standard Configuration Descriptor */
	u08_t	bLength;
	u08_t	bDescriptorType;
	u16_t	wTotalLength;
	u08_t	bNumInterfaces;
	u08_t	bConfigurationValue;
	u08_t	iConfiguration;
	u08_t	bmAttributes;
	u08_t	bMaxPower;	/* in 2mA units, 50 ^ 100mA */
}	usb_scd_t; /* 9 bytes */

typedef struct { /* Standard Interface Descriptor */
	u08_t	bLength;
	u08_t	bDescriptorType;
	u08_t	bInterfaceNumber;
	u08_t	bAlternateSetting;
	u08_t	bNumEndPoints;
	u08_t	bInterfaceClass;
	u08_t	bInterfaceSubClass;
	u08_t	bInterfaceProtocol;
	u08_t	iInterface;
}	usb_sid_t; /* 9 bytes */

typedef struct { /* Standard Endpoint Descriptor */
	u08_t	bLength;
	u08_t	bDescriptorType;
	u08_t	bEndPointAddress;
	u08_t	bmAttributes;
	u16_t	wMaxPacketSize;
	u08_t	bInterval;
}	usb_sep_t; /* 7 bytes */

#define DSC_DEV	1	// DEVICE
#define DSC_CFG	2	// CONFIGURATION
#define DSC_STR	3	// STRING
#define DSC_IFC	4	// INTERFACE
#define DSC_EPT	5	// ENDPOINT
#define DSC_DQU	6	// DEVICE_QUALIFIER
#define DSC_OSC	7	// OTHER_SPEED_CONFIGURATION
#define DSC_PWR	8	// INTERFACE_POWER

const usb_sdd_t usb_sdd PROGMEM = {
 .bLength = sizeof(usb_sdd_t), .bDescriptorType = DSC_DEV, .bcdUSB = 0x0200,
 .bDeviceClass       = 0,
 .bDeviceSubClass    = 0,
 .bDeviceProtocol    = 0,
 .bMaxPacketSize0    = 64,
 .idVendor           = 0x0403, /* FTDI */
 .idProduct          = 0x6001, /* FT232 USB-Serial (UART) IC */
 .bcdDevice          = 0x0600,
 .iManufacturer      = 0,
 .iProduct           = 0,
 .iSerialNumber      = 0,
 .bNumConfigurations = 1,
};

#define	EP1_LGS		3	/* endpoint size (0 == 8, 1 == 16, 2 == 32 ... 6 == 512) */
#define	EP2_LGS		3	/* endpoint size (0 == 8, 1 == 16, 2 == 32 ... 6 == 512) */
const struct {
	usb_scd_t	scd;	/* 9 bytes */
	usb_sid_t	sid;	/* 9 bytes */
	usb_sep_t	ept[2];	/* 7 bytes each */
}	usb_cfg PROGMEM = {
 .scd.bLength = sizeof(usb_scd_t), .scd.bDescriptorType = DSC_CFG,
 .scd.wTotalLength        = sizeof(usb_scd_t) + sizeof(usb_sid_t) + 2*sizeof(usb_sep_t),
 .scd.bNumInterfaces      = 1,
 .scd.bConfigurationValue = 1,
 .scd.iConfiguration      = 0,
 .scd.bmAttributes        = 0xa0,
 .scd.bMaxPower           = 100/2,

 .sid.bLength = sizeof(usb_sid_t), .sid.bDescriptorType = DSC_IFC,
 .sid.bInterfaceNumber    = 0,
 .sid.bAlternateSetting   = 0,
 .sid.bNumEndPoints       = 2,
 .sid.bInterfaceClass     = 0xff, /* vendor specific */
 .sid.bInterfaceSubClass  = 0xff,
 .sid.bInterfaceProtocol  = 0xff,
 .sid.iInterface          = 2,

 .ept[0].bLength = sizeof(usb_sep_t), .ept[0].bDescriptorType = DSC_EPT,
 .ept[0].bEndPointAddress = 0x81, /* EP 1 IN dev->host */
 .ept[0].bmAttributes     = 0x02, /* BULK */
 .ept[0].wMaxPacketSize   = (8<<EP1_LGS),
 .ept[0].bInterval        = 0,

 .ept[1].bLength = sizeof(usb_sep_t), .ept[1].bDescriptorType = DSC_EPT,
 .ept[1].bEndPointAddress = 0x02, /* EP 2 OUT dev<-host */
 .ept[1].bmAttributes     = 0x02, /* BULK */
 .ept[1].wMaxPacketSize   = (8<<EP2_LGS),
 .ept[1].bInterval        = 0,
};

u08_t usb_tf(void) /* txb free space */
{
	return CB_FREE(gl.usb.ep1r,gl.usb.ep1w,NEL(gl.usb.ep1b));
}

void usb_sc(char c)
{
	u08_t	i;

	gl.usb.ep1b[i = gl.usb.ep1w] = c;
	if (++i >= NEL(gl.usb.ep1b)) i = 0;
	if (i != gl.usb.ep1r) gl.usb.ep1w = i;
}

void usb_sp(const char *p) /* send string from flash */
{
	char	c;

	while ((c = pgm_read_byte(p++))) usb_sc(c);
}

#define	USB_SP(str)	usb_sp(PSTR(str))

int usb_rc(void) /* read next character received */
{
	int	b;

	if (gl.usb.ep2r == 0) return -1;
	UENUM = 2;
	if (!(UEINTX & _BV(FIFOCON))) return -1;
	b = UEDATX;
	if (--gl.usb.ep2r == 0) UEINTX &= ~_BV(FIFOCON);
	return b;
}

ISR(USB_GEN_vect)
{
	u08_t	irq, ept;

	irq = UDINT;
	ept = UENUM;
	UDINT = 0b00000000;	/* -,uprsmi,eorsmi,wakeup,eorsti,sofi,-,suspi */
	irq &= UDIEN;
	if (irq & _BV(EORSTI)) {
		gl.usb.ep0a = 0;
		gl.usb.ep0r = 0;
		gl.usb.ep1w = 0;
		gl.usb.ep1r = 0;
		gl.usb.sofi = 0;
		gl.usb.ep1h = 1;
		gl.usb.mdst = RST_MDST;
		gl.usb.lnst = RST_LNST;
		UENUM   = 0;
		UECONX  = 0b00000001;	/* -,-,stallrq,stallqc,rstdt,-,-,EPEN */
		UECFG0X = 0b00000000;	/* ept1,ept0,-,-,-,-,-,epdir */
		UECFG1X = 0b00110010;	/* -,eps2,EPS1,EPS0,epbk1,epbk0,ALLOC,- 64 bytes */
		if (UESTA0X & _BV(CFGOK)) {
			UEIENX = _BV(RXSTPE);	/* enable IRQs for EP 0 */
			gl.usb.ep0a = 1;	/* enable usb_sq */
		}
	}
	if (irq & _BV(SOFI)) gl.usb.sofi++;
	UENUM = ept;
}

ISR(USB_COM_vect)
{
	u08_t	ept = UENUM;

	/* UEINTX: fifocon,nakini,rwal,nakouti,rxstpi,rxouti,stalledi,txini */
	if (UEINT & 01) { /* control endpoint */
		UENUM = 0;
		if ((UEINTX & _BV(RXSTPI))) {
			if (gl.usb.ep0r == 0)
				while (gl.usb.ep0r < 8) gl.usb.ep0b.b[gl.usb.ep0r++] = UEDATX;
			UEINTX = 0;
		}
	}
	if (UEINT & 02) { /* IN endpoint dev->host, outgoing data */
		UENUM = 1;
		if ((UEINTX & _BV(TXINI))) {
			gl.usb.ep1h = 0;
			UEINTX &= ~_BV(TXINI);
		}
	}
	if (UEINT & 04) { /* OUT endpoint dev<-host, incoming data */
		UENUM = 2;
		if ((UEINTX & _BV(RXOUTI))) {
			gl.usb.ep2r += UEBCX;
			UEINTX &= ~_BV(RXOUTI);
		}
	}
	UENUM = ept;
}

void usbd_psnd(const void *buf, u08_t nbu)
{
	const u08_t	*d = buf;
	u08_t		n;

	for (n = 0; n < nbu; n++) {
		while (!(UEINTX & (_BV(TXINI)|_BV(RXOUTI))));
		if (UEINTX & _BV(RXOUTI)) break;
		UEDATX = pgm_read_byte(d++);
	}
}

void usb_sq(void)
{
	u08_t	i;

	if (gl.usb.ep0a == 0) return;
	/*
	 * EP1, IN endpoint dev->host
	 */
	if (gl.usb.ep1h == 0 && (CB_HASD(gl.usb.ep1r,gl.usb.ep1w) || gl.usb.sofi >= 40)) {
		UENUM = 1;
		UEDATX = gl.usb.mdst;
		UEDATX = gl.usb.lnst;
		while ((UEINTX & _BV(RWAL))) {
			if (CB_NO_D(gl.usb.ep1r,gl.usb.ep1w)) break;
			UEDATX = gl.usb.ep1b[i = gl.usb.ep1r];
			gl.usb.ep1r = ++i >= NEL(gl.usb.ep1b) ? 0 : i;
		}
		gl.usb.sofi = 0;
		gl.usb.ep1h = 1;
		UEINTX &= ~_BV(FIFOCON);
	}
	/*
	 * control endpoint
	 */
	if (gl.usb.ep0r == 0) return;
	i = SREG; cli();
	UENUM = 0;
	if (gl.usb.ep0b.bmRequestType & REQ_D2H)
		while (!(UEINTX & _BV(TXINI)));
	else	UEINTX = ~_BV(TXINI);
	switch ((gl.usb.ep0b.bmRequestType & REQ_TYP)) {
	case REQ_STD:
		switch (gl.usb.ep0b.bRequest) {
		case GET_STATUS:
			UEDATX = 0;
			UEDATX = 0;
			UEINTX &= ~_BV(TXINI);
			break;
		case SET_ADR:
			while (!(UEINTX & _BV(TXINI)));
			UDADDR = gl.usb.ep0b.wValue;
			UEINTX &= ~_BV(TXINI);
			UDADDR |= _BV(ADDEN);
			break;
		case GET_DSC:
			switch (gl.usb.ep0b.wValue>>8) {
			case DSC_DEV: /* 1 USB_DEVICE_DESCRIPTOR_TYPE */
				usbd_psnd(&usb_sdd,MIN(gl.usb.ep0b.wLength,sizeof(usb_sdd)));
				UEINTX &= ~_BV(TXINI);
				break;
			case DSC_CFG: /* 2 USB_CONFIGURATION_DESCRIPTOR_TYPE */
				usbd_psnd(&usb_cfg,MIN(gl.usb.ep0b.wLength,sizeof(usb_cfg)));
				UEINTX &= ~_BV(TXINI);
				break;
#if	0
			case DSC_STR: /* 3 USB_STRING_DESCRIPTOR_TYPE */
			case DSC_IFC: /* 4 USB_INTERFACE_DESCRIPTOR_TYPE */
			case DSC_EPT: /* 5 USB_ENDPOINT_DESCRIPTOR_TYPE */
			case DSC_DQU: /* 6 USB_DEVICE_QUALIFIER_TYPE */
			case DSC_OSC: /* 7 USB_OTHER_SPEED_CONFIGURATION_TYPE */
			case DSC_PWR: /* 8 USB_INTERFACE_POWER_TYPE */
#endif
			default: UECONX |= _BV(STALLRQ); break;
			}
			break;
		case GET_CFG:
			UEDATX = 1;
			UEINTX &= ~_BV(TXINI);
			break;
		case SET_CFG:
			switch ((gl.usb.ep0b.bmRequestType & REQ_DST)) {
			case REQ_DEV:
				UENUM   = 1;	/* IN DEV -> HOST */
				UECONX  = 1;
				UECFG0X = 0b10000001; /* ept[1:0],-,-,-,-,-,EPDIR  BULK IN */
				UECFG1X = (EP1_LGS<<4)|0b0110; /* -,epsize[2:0]=010,epbk[1:0]=01,ALLOC,- double */
				UEIENX  = 0b00000001; /* flerre,nakine,-,nakoute,rxstpe,rxoute,stallede,TXINE */
				UENUM   = 2;	/* OUT HOST -> DEV */
				UECONX  = 1;
				UECFG0X = 0b10000000; /* ept[1:0],-,-,-,-,-,epdir  BULK OUT */
				UECFG1X = (EP2_LGS<<4)|0b0110; /* -,epsize[2:0]=010,epbk[1:0]=01,ALLOC,- double */
				UEIENX  = 0b00000100; /* flerre,nakine,-,nakoute,rxstpe,RXOUTE,stallede,txine */
				break;
			case REQ_IFC:
			case REQ_EPT:
			default:
				break;
			}
			break;
		default: UECONX |= _BV(STALLRQ); break;
		}
		break;
	case REQ_VEN:
		switch (gl.usb.ep0b.bRequest) {
		case  0: /* FTDI_SIO_RESET		 0 reset port */
			gl.usb.mdst = RST_MDST;
			gl.usb.lnst = RST_LNST;
			gl.usb.tlat = 0;
			UEINTX &= ~_BV(TXINI);
			break;
		case  3: /* FTDI_SIO_SET_BAUD_RATE	 3 set baud rate */
#define	FTDI_B9600	0x4138
#define	FTDI_B1200	0x09c4
			switch (gl.usb.rbsq) {
			case 0: case 1:
				if (gl.usb.ep0b.wValue == FTDI_B9600) gl.usb.rbsq++;
				else gl.usb.rbsq = 0;
				break;
			case 2: if (gl.usb.ep0b.wValue == FTDI_B1200) gl.exit = 1;
				/* FALLTHROUGH */
			default:
				gl.usb.rbsq = 0;
				break;
			}
#undef	FTDI_B9600
#undef	FTDI_B1200
			/* FALLTHROUGH */
		case  1: /* FTDI_SIO_MODEM_CTRL		 1 set modem control register */
		case  2: /* FTDI_SIO_SET_FLOW_CTRL	 2 set flow control register */
		case  4: /* FTDI_SIO_SET_DATA		 4 set data format */
		case  6: /* FTDI_SIO_SET_EVENT_CHAR	 6 set event character */
		case  7: /* FTDI_SIO_SET_ERROR_CHAR	 7 set error character */
			UEINTX &= ~_BV(TXINI);
			break;
		case  5: /* FTDI_SIO_GET_MODEM_STATUS	 5 get modem status register */
			if (gl.usb.ep0b.wLength > 0) UEDATX = gl.usb.mdst;
			if (gl.usb.ep0b.wLength > 1) UEDATX = gl.usb.lnst;
			UEINTX &= ~_BV(TXINI);
			break;
		case  9: /* FTDI_SIO_SET_LATENCY_TIMER	 9 set latency timer */
			gl.usb.tlat = gl.usb.ep0b.wValue;
			UEINTX &= ~_BV(TXINI);
			break;
		case 10: /* FTDI_SIO_GET_LATENCY_TIMER	10 get latency timer */
			if (gl.usb.ep0b.wLength > 0) UEDATX = (gl.usb.tlat>>0);
			if (gl.usb.ep0b.wLength > 1) UEDATX = (gl.usb.tlat>>8);
			UEINTX &= ~_BV(TXINI);
			break;
		default: UECONX |= _BV(STALLRQ); break;
		}
		break;
	default:
		UECONX |= _BV(STALLRQ);
		break;
	}
	gl.usb.ep0r = 0;
	SREG = i;
}

void usb_tw(void) /* wait for tx empty */
{
	do usb_sq(); while (gl.usb.sofi < 100 && CB_HASD(gl.usb.ep1r,gl.usb.ep1w));
}

void usb_init(void)
{
	u08_t	sreg;

	sreg = SREG; cli();
	USBCON = 0b00000000;	/* usbe,-,frzclk,otgpade,-,-,-,vbuste */
	UHWCON = 0b00000001;	/* -,-,-,-,-,-,-,UVREGE */
	USBCON = 0b10100000;	/* USBE,-,FRZCLK,otgpade,-,-,-,vbuste */
	PLLCSR = 0b00010010;	/* -,-,-,PINDIV,-,-,PLLE,plock */
	while (!(PLLCSR & _BV(PLOCK)));
	UDINT  = 0b00000000;	/* -,uprsme,eorsme,wakeupe,eorste,sofe,-,suspe */
	USBCON = 0b10010000;	/* USBE,-,frzclk,OTGPADE,-,-,-,vbuste */
	UDIEN  = 0b00001100;	/* -,uprsme,eorsme,wakeupe,EORSTE,SOFE,-,suspe */
	UDCON  = 0b00000000;	/* -,-,-,-,rstcpu,lsm,rmwkup,detach */
	SREG = sreg;
}
#endif

#if	AT2SO == 1
/*
 * drone companion program piggy back in flash
 */
u16_t rdflash(u16_t adr); /* read flash word */
void _rdflash(void)	/* addr (r24:r25), return (r24:r25) */
{
asm("rdflash:     ");
asm(" push r31    ");
asm(" push r30    ");
asm(" movw r30,r24");
asm(" lpm  r24,Z+ ");
asm(" lpm  r25,Z  ");
asm(" pop  r30    ");
asm(" pop  r31    ");
}

void at2so_chck(u16_t adr)
{
	u16_t	mag, siz, cks;

	if (adr == 0) return;
	mag = rdflash(adr); adr += 2;
	siz = rdflash(adr); adr += 2;
	cks = rdflash(adr); adr += 2;
	if (mag != 0x55aa) return;
	if (siz < ( 1*1024)) return;
	if (siz > (20*1024)) return;
	gl.arm.adr = adr;
	gl.arm.siz = siz;
	gl.arm.cks = cks;
}

int at2so_load(void)
{
	union { u16_t w; u08_t b[2]; } v;
	u16_t	cks, adr;
	int	n;

	/* put tty we are talking to into raw mode, will be
	 * sending binary data -> NO interpretation please! */
	SIO_SP("stty -echo -iexten pass8 raw\n");
	ms_dly(50); sio_rf();
	/* 'dd' is the program of choice to receive the data, need to
	 * tell it how much is coming, so it quits after the transfer.
	 * 'dd' on drone is the 'amputated' version without bells and
	 * whistles. */
	SIO_SP("dd of=/tmp/$F.gz bs=1 count=");
	snd_sid(0,gl.arm.siz);
	sio_sc('\n');
	ms_dly(100); sio_rf();
	for (adr = gl.arm.adr, n = cks = 0;;) {
		v.w = rdflash(adr+n); /* reads 2 bytes from flash */
		sio_sc(v.b[0]); cks += v.b[0];
		if (++n >= gl.arm.siz) break;
		sio_sc(v.b[1]); cks += v.b[1];
		if (++n >= gl.arm.siz) break;
		if ((n & 0x1f) == 0) {
			LEDTG(); /* makes LED look like it is dimmed */
			while (CB_HASD(gl.sio.txr,gl.sio.txw));
			ms_dly(1);
		}
	}
	ms_dly(50); sio_rf();
	/* put tty back to 'normal' */
	SIO_SP("stty sane\n"); ms_dly(50); sio_rf();
	if (n != gl.arm.siz || cks != gl.arm.cks) return 0; /* upload bad */
	/* unzip it */
	SIO_SP("gunzip /tmp/$F.gz\n"); ms_dly(100); sio_rf();
	ms_dly(50); sio_rf();
	return 1;
}
#endif

#define	C_STRT	0x02
#define	C_STOP	0x04

void at2so_lstp(u08_t n)
{
	while (1) {
		blip(1); ms_dly(1000);
		blip(n); ms_dly(2000);
	}
}

char sio_w4lf(int nms)
{
	int	k;

	for ( ; nms; nms -= 1, ms_dly(1))
		while ((k = sio_rc()) >= 0) if (k == '\n') return 0;
	return -1;
}

char sio_w4ln(int nms, u08_t *lbu, u08_t nlb)
{
	int	k;
	u08_t	i;

	for (i = 0; nms; nms -= 1, ms_dly(1))
		while ((k = sio_rc()) >= 0) {
			if (k == '\n') return (char)i;
			if (i < nlb) lbu[i++] = k;
		}
	return -1;
}

int at2so_txfr(const char *pnm)
{
	u08_t	lbu[4];

	sio_rf(); SIO_SP("F="); sio_sp(pnm); sio_rev(); SIO_SP(".arm\n");
	if (sio_w4lf(99)) return -1; /* communication bad */
	sio_rf(); SIO_SP("if [ -f $D/$F ];then cp $D/$F /tmp;echo 1;else echo 0;fi\n");
	if (sio_w4lf(99)) return -1; /* communication bad */
	return sio_w4ln(99,lbu,NEL(lbu)) < 1 ? -1 : lbu[0] == '1' ? 1 : 0;
}

void at2so_exec(u08_t blkwifi)
{
	int	n;

#if	1
	/* change baudrate since Arduino at 16 Mhz has problems receiving back to
	 * back characters at 115200 with half speed receiver
	 * 76800 would be nice because it can do it, but stty on drone is amputated... */
	if ((SIO_UCSRA & _BV(SIO_U2X))) {
		/* is using double speed on serial transmitter, which is not good for
		 * receiver (half sampling rate) -> change baudrate to 38400 to fix this */
		SIO_SP("stty 38400\n");
		ms_dly(10); sio_rf();
		SIO_UBRR  = (F_CPU/(16UL*38400UL)-1);
		SIO_UCSRA = 0b00000000;	/* rxc0,txc0,udre0,fe0,dor0,upe0,U2X0,mpcm0 */
		ms_dly(50); sio_rf();
		SIO_UCSRB |= _BV(SIO_RXCIE)|_BV(SIO_RXEN)|_BV(SIO_TXEN);
		sio_sc('\r');		/* send a character at new baudrate */
		ms_dly(20); sio_rf();	/* wait a while and clear residuals if any */
	}
#endif
	sio_rf(); sio_sc('\n');
	if (sio_w4lf(99)) at2so_lstp(1);	/* communication bad */
	sio_rf(); SIO_SP("D=/data/video/usb\n");
	if (sio_w4lf(99)) at2so_lstp(2);	/* communication bad */
	if ((n = at2so_txfr(PSTR("pilot"))) < 0) at2so_lstp(3);
	else if (n == 0 && (n = at2so_txfr(PSTR("at2so"))) < 0) at2so_lstp(4);
	else if (n == 0) {
		sio_rf(); SIO_SP("D=/data/video\n");
		if (sio_w4lf(99)) at2so_lstp(5);/* communication bad */
		if ((n = at2so_txfr(PSTR("pilot"))) < 0) at2so_lstp(6);
		else if (n == 0 && (n = at2so_txfr(PSTR("at2so"))) < 0) at2so_lstp(7);
	}
#if	AT2SO == 1
	if (n == 0 && gl.arm.adr != 0) n = at2so_load();
#endif
	if (n == 0) at2so_lstp(8);		/* no companion program */

	LEDON();
	/* make companion program executable */
	SIO_SP("chmod +x /tmp/$F\n"); ms_dly(50); sio_rf();
	/* launch compagnion program */
	SIO_SP("/tmp/$F");
	if (blkwifi) SIO_SP(" -w");
	SIO_SP(" -l ");	gl.bid++; snd_sid(0,gl.bid); /* boot id */
	ms_dly(25); sio_rf();	/* wait out echo */
	sio_sc('\n');		/* and launch */

	/* wait for companion program to send start character */
	while (1) {
		if ((n = sio_rc()) < 0) ms_dly(1);
		else if (n == C_STRT) break;
		while (n == C_STOP) at2so_lstp(9); /* there is another controller active */
	}
	eepr_save();	/* store new 'bid' in EEPROM */
}

/*
 * RC receiver interface debug/setup loop
 */
void csisclr(void)  { SND_SP("\033[H\033[0m\033[2J"); }	/* home, clear attributes, clear screen */
void csirclr(void)  { SND_SP("\033[0J"); }
void csilclr(void)  { SND_SP("\033[K"); }
void csirow(int  r) { SND_SP("\033["); snd_sid(0,r); SND_SP(";1H"); }
void csilco(char c) { SND_SP("\033[0;"); snd_sbx(0x30+c); snd_sc('m'); }
void csihco(char c) { SND_SP("\033[1;"); snd_sbx(0x30+c); snd_sc('m'); }

#define	DLN0	1
#define	DLN1	2
#define	DLN2	3
#define	DLN3	4
#define	DLNRXI	6
#define	DLNRXC	7
#define	DLNGGA	14
#define	DLNRMC	15
#define	DLNOTH	16
#define	DLNBMPD	18  /* BMP data */
#define	DLNBMPC	19  /* BMP calibration */
#define	DLNHMCD	20  /* HMC data */
#define	DLNHMCC	21  /* HMC calibration */

#define	BLACK	0
#define	RED	1
#define	GREEN	2
#define	YELLOW	3
#define	BLUE	4
#define	MAGENTA	5
#define	CYAN	6
#define	WHITE	7

u08_t rdbcfg(u08_t spmcsr, u16_t adr);
void _rdbcfg(void) /* spmcsr (r24), addr (r23:r22), return (r24) */
{
asm("rdbcfg:");
asm(" push r31");
asm(" push r30");
asm(" movw r30,r22");
asm(" out  0x37,r24");
asm(" lpm  r24,Z   ");
asm(" pop  r30");
asm(" pop  r31");
}

void rdcfg(u08_t *buf)
{
	u08_t	srg;

	srg = SREG; cli();
	buf[0] = rdbcfg(0x09,3); /* high fuse */
	buf[1] = rdbcfg(0x09,0); /* low fuse */
	buf[2] = rdbcfg(0x09,2); /* ext fuse */
	buf[3] = rdbcfg(0x09,1); /* lockbits */
	buf[4] = rdbcfg(0x21,0); /* signature byte */
	buf[5] = rdbcfg(0x21,2); /* signature byte */
	buf[6] = rdbcfg(0x21,4); /* signature byte */
	SREG = srg;
}

void su_sgps(void)
{
	char	c;
	int	i;

	while (CB_HASD(gl.gps.b_r,gl.gps.b_w) && sio_tf() >= 100) {
		if (++gl.gps.b_r >= NEL(gl.gps.buf)) gl.gps.b_r = 0;
		if ((i = gl.gps.b_r + 2) >= NEL(gl.gps.buf)) i -= NEL(gl.gps.buf);
		c = gl.gps.buf[i];
		if (c == 'G') csirow(DLNGGA), csihco(GREEN);
		else if (c == 'R') csirow(DLNRMC), csihco(GREEN);
		else csirow(DLNOTH), csihco(RED);
		while (1) {
			c = gl.gps.buf[gl.gps.b_r];
			if (++gl.gps.b_r >= NEL(gl.gps.buf)) gl.gps.b_r = 0;
			if (c == '\r') break;
			snd_sc(c);
		}
		csilclr();
	}
}

#define	HZ2TIC(hz)	((double)F_TM1/(double)(hz))
#define	DRUPD		(u32_t)HZ2TIC(LOOPHZ)	/* dr_loop() cycle time in timer 1 ticks */

void su_loop(void)
{
	u08_t	i = 0, k, n, msk;
	char	j;
	int	rxs;
	long	tmp;
	u32_t	nxt;
	u08_t	cfg[7];

#if	U4 == 0
#define	GUI_TW	sio_tw
#define	GUI_RC	sio_rc
#define	GUI_TF	sio_tf
#else
#define	GUI_TW	usb_tw
#define	GUI_RC	usb_rc
#define	GUI_TF	usb_tf
#endif
	sio_rf();
	while (1) {
		if (gl.exit) return;
		if (++i < 21 && (i & 02)) LEDTG();
		else if (i >= 40) i = 0;
		nxt = tic() + DRUPD;
		do GUI_TW(); while ((tmp = nxt - tic()) > 0);
		if (GUI_RC() == C_STRT) break;
#if	U4
		if (CB_NO_D(gl.sio.rxr,gl.sio.rxw)) continue;
		/* USB <-> RS232 serial bridge */
		csisclr();
		while (gl.usb.sofi < 100) {
			while ((rxs = sio_rc()) > 0) usb_sc(rxs);
			usb_tw();
			while ((rxs = usb_rc()) > 0) sio_sc(rxs);
			if ((tmp = nxt - tic()) <= 0)
				LEDTG(), nxt = tic() + MS2TIC(250);
		}
		return;
#endif
	}
	LEDOF();
	gl.dsnd = U4 ? 1 : 0;
AGAIN:
	GUI_TW();
	csisclr();
	csirow(DLN0); csihco(CYAN);
	SND_SP("rx2at ");
	snd_sp(version);
	SND_SP(", at2so ");
	if (gl.arm.adr == 0) SND_SP("not ");
	SND_SP("attached");

	rdcfg(cfg);
	csirow(DLN1);
	csilco(WHITE); SND_SP("cpusg ");
	k = 0x1e; i = cfg[4]; msk = 0xff; csihco((i&msk)==k?GREEN:RED); snd_sbx(i);
	k = 0x95; i = cfg[5]; msk = 0xff; csihco((i&msk)==k?GREEN:RED); snd_sbx(i);
	k = U4 ? 0x87:0x0f; i = cfg[6]; msk = 0xff; csihco((i&msk)==k?GREEN:RED); snd_sbx(i);
	csilco(WHITE); SND_SP(" fuses ");
	snd_sbx(cfg[0]); snd_sc('-');	/* high */
	snd_sbx(cfg[1]); snd_sc('-');	/* low */
	snd_sbx(cfg[2]); snd_sc('-');	/* extended */
	snd_sbx(cfg[3]);		/* lockbits */

	GUI_TW();
	csirow(DLN2); csilco(WHITE);
	SND_SP(" loop ");
	snd_spd(0,1,TIC2MS10(DRUPD));
	SND_SP("ms, dcnt="); snd_sid(0,gl.dcnt);

	csirow(DLNRXI); csihco(CYAN);
	SND_SP("-RX-  f[ms]  p[ms] value");

	tmr0_setup(0);
GPSCHANGE:
	GUI_TW();
	csirow(DLN3); csilco(WHITE);
	SND_SP("  gps ");
	if (gl.gps.b100) {
		snd_sid(0,gl.gps.b100);
		SND_SP("00 bps");
	}
	csilclr();

	nxt = tic() + DRUPD;
	for (n = k = 0; !gl.exit; n ^= 01) {
		/* EEPROM update */
		if (gl.eewen < NEL(gl.eedat)) eepr_updt();
		i = gl.gps.tcnt;
		LEDOF();
		do {
			usb_sq();
			gps_rcv();
			bmp_sq();
			hmc_sq();
		}
		while ((tmp = nxt - tic()) > 0);
		LEDON();
		if (i != gl.gps.tcnt) goto GPSCHANGE;
		nxt = tic() + DRUPD;
		GUI_TW();

		rxs = rx_read() & S_SIG;
		if (n & 01) {
			for (i = 0; i < NEL(gl.rxs.chn); i++) {
				if (!((msk = _BV(i)) & S_SIG)) continue;
				if (GUI_TF() < 48) GUI_TW();
				csirow(DLNRXC+i);
				csihco(rxs&msk?RED:GREEN);
				switch (i) {
				case S_THRO: SND_SP("THRO"); break;
				case S_AILE: SND_SP("AILE"); break;
				case S_ELEV: SND_SP("ELEV"); break;
				case S_RUDD: SND_SP("RUDD"); break;
				case S_GEAR: SND_SP("GEAR"); break;
				case S_AUX1: SND_SP("AUX1"); break;
				}
				csihco(gl.rxs.eft&msk?RED:GREEN);
				snd_spd(7,1,TIC2MS10(gl.rxs.chn[i].ftm));
				if (gl.rxs.ept&msk) csihco(RED);
				snd_spd(7,3,TIC2US(gl.rxs.chn[i].dur));
				snd_sid(-6,gl.rxs.chn[i].val);
				if (rxs == 0) {
					if (i == S_LAND)
						switch (gl.rxs.fms) {
						case FMS_LAND: SND_SP(" LAND"); break;
						case FMS_FM_1: SND_SP(" FM_1"); break;
						case FMS_FM_2: SND_SP(" FM_2"); break;
						case FMS_FM_3: SND_SP(" FM_3"); break;
						default:       SND_SP(" ????"); break;
						}
					if (gl.rxs.fms == FMS_LAND) {
						if (i == SH_THRO) {
							if (gl.rxs.stk & TS_LF) SND_SP(" TRIM");
							if (gl.rxs.stk & TS_RT) SND_SP(" ESTP");
						}
						if (i == S_THRO) {
							j = gl.cfg;
							if (gl.rxs.stk & TS_UP) {
								if ((j = gl.cfw) < 0) j = gl.cfg;
								if (gl.rxs.stk & (ES_UP|ES_DN|ES_LF|ES_RT)) csihco(YELLOW);
							}
							SND_SP(" CFG"); snd_snx(j+1);
						}
						if (i == SH_ELEV && !(gl.rxs.stk & TS_UP)) {
							if (gl.rxs.stk & ES_RT) SND_SP(" VID+");
						}
					}
					else {
						if (i == S_LAND && gl.rxs.fsa) {
						     SND_SP(" ANI"); snd_snx(gl.rxs.fsa);
						}
					}
				}
				csilclr();
			}
			SND_SP("\r\n");
		}
		else {
			if (CB_HASD(gl.gps.b_r,gl.gps.b_w)) GUI_TW(), su_sgps();
			else {
				if ((msk = bmp_rp(0))) {
					if (msk & 01) {
						csirow(DLNBMPD); csihco(WHITE);
						msk = bmp_rp(01);
						csilclr();
					}
					if (msk & 02) {
						csirow(DLNBMPC); csihco(WHITE);
						msk = bmp_rp(02);
						csilclr();
					}
				}
				if ((msk = hmc_rp(0))) {
					if (msk & 01) {
						csirow(DLNHMCD); csihco(WHITE);
						msk = hmc_rp(01);
						csilclr();
					}
					if (msk & 02) {
						csirow(DLNHMCC); csihco(WHITE);
						msk = hmc_rp(02);
						csilclr();
					}
				}
			}
			if (GUI_RC() == C_STRT) goto AGAIN;
		}
	}
#undef	GUI_TW
#undef	GUI_RC
#undef	GUI_TF
}

/*
 * DRONE control loop
 */
void dr_scfg(void)
{
	const char	*p;

	SIO_SP("RX=C,");
	switch (gl.cfg) {
	default:
	case 0: p = cfg1; break;
	case 1: p = cfg2; break;
	case 2: p = cfg3; break;
	case 3: p = cfg4; break;
	}
	sio_sp(p);
	sio_sc('\r');
}

void dr_fred(u08_t dur)
{
	SIO_SP("RX=L,2,1084227584,"); /* 1084227584 ^ 5.0 */
	snd_snx(dur);
	sio_sc('\r');
}

void dr_vlba(void)
{
	if (++gl.vlba.tick < 6) {
		if (VLBA_BLINK && gl.vlba.stat && gl.vlba.tick == 2) VLBOF();
		return;
	}
	gl.vlba.tick = 0;
	switch (gl.vlba.stat) {
	case 0: if (VLBA_THR > 0 && VLBA_THR <= 60 && gl.drm.m2a.cbat <= VLBA_THR) {
			VLBON();
			dr_fred(0);
			gl.vlba.stat = 1;
			break;
		}
		break;
	case 1: if (gl.drm.m2a.cbat > VLBA_THR) {
			VLBOF();
			dr_fred(1);
			gl.vlba.stat = 0;
		}
		else VLBON();
		break;
	}
}

void dr_sgps(void) /* send GPS data to drone */
{
	u08_t	c, r, w;

	r = gl.gps.b_r;
	w = gl.gps.b_w;
	while (CB_HASD(r,w) && sio_tf() >= 80)
		do {
			c = gl.gps.buf[r];
			if (++r >= NEL(gl.gps.buf)) r = 0;
			sio_sc(c);
			if (c == '\r') break;
		}
		while (CB_HASD(r,w));
	gl.gps.b_r = r;
}

void dr_send(u08_t s)
{
	u08_t	rp;

	SIO_SP("RX=");
	if (gl.pad < 0) sio_sc('E');
	else snd_sid(0,s==10?gl.rxs.fms:0);
	snd_sid(',',gl.rxs.chn[S_ROL].val);
	snd_sid(',',gl.rxs.chn[S_PTC].val);
	snd_sid(',',gl.rxs.chn[S_GAZ].val);
	snd_sid(',',gl.rxs.chn[S_YAW].val);
	if (gl.rxs.fsa) {
		SIO_SP("\rRX=A");
		snd_sid(',',gl.rxs.fsa);
		gl.rxs.fsa = 0;
	}
	sio_sc('\r');
	if (CB_HASD(gl.gps.b_r,gl.gps.b_w)) dr_sgps();
	if ((rp = bmp_rp(0)) && sio_tf() >= 80) {
		     if (rp & 02) rp = bmp_rp(02), sio_sc('\r');
		else if (rp & 01) rp = bmp_rp(01), sio_sc('\r');
	}
	if ((rp = hmc_rp(0)) && sio_tf() >= 80) {
		     if (rp & 02) rp = hmc_rp(02), sio_sc('\r');
		else if (rp & 01) rp = hmc_rp(01), sio_sc('\r');
	}
	sio_sc('\n');
}

void dr_loop(void)
{
	u08_t	s, i, k, led;
	long	tmp;
	u32_t	nxt;

	gl.rxs.stk = 0;
	led = 0;
	s = i = k = 0;
	/* This is it, this program is talking to the program on the drone now.
	 * 0,1,2) send configuration commands
	 * 3+)    your actions on TX/RX are in charge */
	sio_rf();
	nxt = tic() + DRUPD;
	tmr0_setup(0);	/* get GPS setup going */
	while (1) {
		/* packet to drone */
		dr_send(s);
		/* EEPROM update */
		if (gl.eewen < NEL(gl.eedat)) eepr_updt();

		/* keep loop on track, don't want to be too pushy or late */
		gl.drm.ack = 0;
		do {
			gps_rcv();
			sio_rcv();
			bmp_sq();
			hmc_sq();
		}
		while ((tmp = nxt - tic()) > 0);
		nxt = tic() + DRUPD;

		if (s < 3) rx_read();
		else { /* configuration is done */
			/* Arduino LED blink */
#if	NL_AIR
			if (s == 10) LEDOF(); else
#endif
			if (led == 0) LEDTG();
			else if ((led >>= 1) == 0) {
				led = 1<<(5-1);
				if (i < k) LEDON();
				if (++i >= 6) i = 0;
			}
			else LEDOF();
			dr_vlba();
			if (rx_read() & S_SIG) { /* radio quit or got disconnected from TX */
				gl.rxs.stk = gl.rxs.chg = 0;
				gl.pad = led = 0;
				if (s != 9) s = 9, SIO_SP("slog,RX,0\r");
				continue;
			}
		}

		switch (s) {
		/*
		 * drone configuration
		 */
		case 0: /* send configuration string */
			dr_scfg();
			s = 1;
			k = S2LTIC(3.0);
			LEDOF();
			break;
		case 1: if (gl.drm.ack) s = 3, i = 0; /* received C_ACK from drone */
			else if (--k == 0) s = 0, LEDON();
			break;
		/*
		 * drone on ground, or getting there
		 */
		case 3:
			if (gl.rxs.fms == FMS_LAND) {
				if (gl.rxs.chg & TS_RT) {
					if (gl.rxs.stk & TS_RT) /* EMERGENCY */
						gl.pad = -1;
					else	gl.pad =  0;
				}
				else if (gl.rxs.stk & gl.rxs.chg & TS_LF) /* FTRIM */
					SIO_SP("RX=T\r");
				if (gl.rxs.stk & TS_UP) { /* show/change config */
					led = 1;
					k = gl.cfg + 1;
					i = 0;
					s = 5;
				}
				else if (gl.rxs.stk & TS_DN) { /* show GPS status */
					if (gl.drm.m2a.sgps >= 0) {
						if (gl.rxs.chg & TS_DN) led = 1;
						k = gl.drm.m2a.sgps;
					}
				}
				else if (gl.rxs.chg & TS_DN) led = 0;
				if (gl.rxs.stk & gl.rxs.chg & ES_RT) /* VZAP */
					SIO_SP("RX=V,Z\r");
				break;
			}
			if (gl.eewen < NEL(gl.eedat)) break;	/* wait for EEPROM write to complete */
			if (gl.rxs.stk != 0) break;		/* all sticks centered for launch */
			if (gl.rxs.chn[S_THRO].val) break;	/* throttle should be dead 0 */
			gl.pad = 1, s = 10;			/* launch */
			SIO_SP("RX=V,1\r");			/* video record on */
			break;
		case 5: /* display/set configuration */
			k = (gl.cfw < 0 ? gl.cfg : gl.cfw) + 1;
			if (gl.rxs.stk & TS_UP) break;
			if (gl.eewen < NEL(gl.eedat))	/* did get changed */
				s = i = 0;		/* send new configuration to drone */
			else	s = 3;
			led = 0;
			break;
		case 9: /* radio loss recovery */
			if (gl.rxs.fms != FMS_LAND) break; /* want FSW on LAND */
			SIO_SP("slog,RX,1\r");
			s = 3;
			break;
		/*
		 * drone airborne
		 */
		case 10:
			if (gl.rxs.fms != FMS_LAND) break;
			gl.rxs.stk = gl.rxs.chg = 0;
			gl.pad = 0;
			SIO_SP("RX=V,0\r"); /* video record off */
			s = 3;
			break;
		}
	}
}

void init(void)
{
extern	u08_t	__bss_start;
extern	u08_t	__bss_end;
	u08_t	i, k, *b = &__bss_start;
	int	n = &__bss_end - &__bss_start;

	cli();
#if	U4
	UDIEN  = 0;
	USBCON = 0;
	UHWCON = 0;
#endif
	while (--n >= 0) *b++ = 0;	/* clear BSS segment */

	for (EECR = 0; gl.eewen < NEL(gl.eedat); gl.eewen++) {
		EEAR = (int)gl.eewen;
		EECR |= _BV(EERE);
		gl.eedat[gl.eewen] = EEDR;
	}
	for (i = k = 0; i < (NEL(gl.eedat)-1); i++) k += gl.eedat[i];
	if (gl.eemag != EE_MAG || gl.eecks != k) gl.cfg = gl.bid = 0;
	gl.cfw = -1;
#if	AT2SO == 1
	extern	u16_t flash_pgb(void);
	at2so_chck(flash_pgb());
#endif
}

int main(void)
{
	u08_t	i, j, k;

AGAIN:
	init();

#if	U4 == 0
	/* Nano, ProMini - RC receiver hardware bits on port D[7:2] */
#if	RX_CON == 0
	gl.rxs.chn[S_THRO].msk = _BV(7);
	gl.rxs.chn[S_AILE].msk = _BV(6);
	gl.rxs.chn[S_ELEV].msk = _BV(5);
	gl.rxs.chn[S_RUDD].msk = _BV(4);
	gl.rxs.chn[S_GEAR].msk = _BV(3);
	gl.rxs.chn[S_AUX1].msk = _BV(2);
#elif	RX_CON == 1
	gl.rxs.chn[S_THRO].msk = _BV(2);
	gl.rxs.chn[S_AILE].msk = _BV(3);
	gl.rxs.chn[S_ELEV].msk = _BV(4);
	gl.rxs.chn[S_RUDD].msk = _BV(5);
	gl.rxs.chn[S_GEAR].msk = _BV(6);
	gl.rxs.chn[S_AUX1].msk = _BV(7);
#else
#error	"RX_CON bad value"
#endif
#else
	/* MiruPCB - RC receiver hardware bits on port B[6:1] */
#if	RX_CON == 0
	gl.rxs.chn[S_THRO].msk = _BV(1);
	gl.rxs.chn[S_AILE].msk = _BV(2);
	gl.rxs.chn[S_ELEV].msk = _BV(3);
	gl.rxs.chn[S_RUDD].msk = _BV(4);
	gl.rxs.chn[S_GEAR].msk = _BV(5);
	gl.rxs.chn[S_AUX1].msk = _BV(6);
#elif	RX_CON == 1
	gl.rxs.chn[S_THRO].msk = _BV(6);
	gl.rxs.chn[S_AILE].msk = _BV(5);
	gl.rxs.chn[S_ELEV].msk = _BV(4);
	gl.rxs.chn[S_RUDD].msk = _BV(3);
	gl.rxs.chn[S_GEAR].msk = _BV(2);
	gl.rxs.chn[S_AUX1].msk = _BV(1);
#else
#error	"RX_CON bad value"
#endif
#endif

	/* dead bands */
	gl.rxs.chn[S_THRO].dbn = MS2TIC(0.06);
	gl.rxs.chn[S_AILE].dbn = MS2TIC(0.04);
	gl.rxs.chn[S_ELEV].dbn = MS2TIC(0.04);
	gl.rxs.chn[S_RUDD].dbn = MS2TIC(0.04);
	gl.rxs.chn[S_GEAR].dbn = MS2TIC(0.20);
	gl.rxs.chn[S_AUX1].dbn = MS2TIC(0.20);

	port_setup();
	LEDON();
	tmr0_setup(0);
	tmr1_setup();
	sio_setup();
	i2c_setup();
	gl.rxs.smp = RXSIG;
	gl.gps.rxi = GPS_RXI_INPUT();
	sei();		/* turn interrupts on */
	msdly_cali();	/* calculates propper gl.dcnt */
	SIO_UCSRB |= _BV(SIO_RXCIE)|_BV(SIO_RXEN)|_BV(SIO_TXEN);	/* turn USART on */
	GPS_RXI_IRQON();	/* enable GPS softserial port */
	PIN_CHG_IRQEN();	/* turn pin change interrupts on */
	LEDOF();

#if	U4
	usb_init();
	ms_dly(500);
	if (gl.usb.ep0a) {
		su_loop();
		while (1) PORTB = 0, DDRB = 0x80; /* hardware reset (RST connected to this pin) */
		goto AGAIN;
	}
	UDIEN  = 0;
	USBCON = 0;
	UHWCON = 0;
#else
	if (SETUP()) {
		sio_rf();
		ms_dly(500);
		su_loop();
		goto AGAIN;
	}
#endif

	/* wait for drone to boot
	 * make sure radio works and has 'sane' state, aka is in LAND */
#define	NS_BOOT	18
#define	NSQUIET	5
	gl.rxs.erx = MERX;	/* mark radio down */
	for (i = j = k = 0; 1; ) {
		if (sio_rf() || j < NS_BOOT) i = 0;
		else if (i < NSQUIET) i++;
		if (rx_read() & S_SIG) k = 0;
		else if (gl.rxs.fms != FMS_LAND) k = 1;
		else if (j < NS_BOOT) k = 2;
		else if (i < NSQUIET) k = 3;
		else break; /* all conditions met, go on */
		switch (k) {
		case 0: blip(1); ms_dly(800-50); break; /* radio not ready */
		case 1: blip(2); ms_dly(600-50); break; /* flight mode not LAND */
		case 2: blip(3); ms_dly(400-50); break; /* drone still booting */
		case 3: blip(4); ms_dly(200-50); break; /* drone still talking */
		}
		/* Note: this loop goes on a 1 second pace, need to refresh RX,
		 *       otherwise 'dead signal' detector triggers on rx_read() */
		rx_read(); ms_dly(50);
		if (j < NS_BOOT) j++;
	}

	/* start shell on drone */
	sio_sc('\r'); ms_dly(100); sio_rf();
	sio_sc('\r'); ms_dly( 50); sio_rf(); /* do it twice... */

	LEDON();
	/* look at throttlestick, if it is not centered, user wants to lock out WiFi... */
	k = (gl.rxs.stk & (TS_UP|TS_DN)) ? 1 : 0;
	at2so_exec(k);	/* load/launch companion program on drone */
	LEDOF();

	dr_loop();
	/* NOTREACHED */
	return 0;
}

#if	AT2SO == 1

void _flash_pgb(void) {
asm("flash_pgb:");
asm(" call eod_flash_pgb");
asm(" .word 0x55aa"); /* magic number */
asm(" .word  11673"); /* size */
asm(" .word 0xbab0"); /* checksum */
asm(" .byte 31,139,8,0,207,199,238,81,2,3,237,124,123,124,84,69");
asm(" .byte 150,127,221,126,36,33,4,184,121,32,33,19,201,77,0,7");
asm(" .byte 33,36,183,67,2,24,65,154,36,188,70,30,157,7,202,142");
asm(" .byte 217,73,58,73,3,209,144,244,36,141,194,252,112,233,164,27");
asm(" .byte 68,135,140,12,201,56,172,3,211,205,142,59,139,14,63,55");
asm(" .byte 58,142,203,58,56,219,96,96,208,193,223,162,50,14,203,160");
asm(" .byte 211,143,212,199,24,152,93,156,245,17,241,209,191,239,185,183");
asm(" .byte 146,92,208,217,199,239,247,199,239,143,159,209,67,157,58,117");
asm(" .byte 234,212,169,170,83,167,78,213,189,125,119,46,91,189,92,146");
asm(" .byte 36,54,242,103,98,179,24,229,138,31,6,32,29,174,34,26");
asm(" .byte 179,22,51,133,37,161,236,107,44,139,202,221,249,203,25,203");
asm(" .byte 63,173,67,136,233,96,17,144,192,244,186,197,94,29,52,129");
asm(" .byte 0,171,40,55,83,25,242,197,157,58,164,51,29,44,99,172");
asm(" .byte 250,159,87,135,43,203,117,176,10,154,86,238,32,144,52,120");
asm(" .byte 17,201,177,57,122,187,84,14,125,89,138,131,64,210,96,68");
asm(" .byte 104,130,65,199,149,200,175,236,212,65,97,58,140,148,85,114");
asm(" .byte 79,19,251,146,191,145,250,133,45,205,13,133,45,77,115,91");
asm(" .byte 154,91,183,110,43,232,104,43,152,167,211,101,161,251,138,181");
asm(" .byte 235,197,88,234,117,136,158,10,152,14,200,5,36,138,246,178");
asm(" .byte 1,147,1,19,0,211,0,147,0,55,49,189,159,121,128,155");
asm(" .byte 1,153,128,175,1,178,0,201,128,28,131,62,18,251,243,127");
asm(" .byte 164,79,146,65,239,145,191,20,161,23,205,193,248,27,234,144");
asm(" .byte 142,25,134,124,26,96,34,96,156,200,79,21,41,205,213,20");
asm(" .byte 209,175,27,255,110,133,82,161,135,198,234,147,93,156,53,228");
asm(" .byte 127,2,184,96,200,23,128,127,192,144,191,13,112,213,144,175");
asm(" .byte 66,249,167,134,252,250,27,244,51,246,65,17,227,100,204,103");
asm(" .byte 163,126,198,158,49,254,123,145,87,12,249,103,1,249,134,124");
asm(" .byte 41,202,23,26,242,231,1,21,134,252,39,100,118,134,124,19");
asm(" .byte 248,107,13,249,165,200,111,54,228,201,206,60,134,124,55,153");
asm(" .byte 168,33,127,19,202,247,26,242,63,3,28,48,228,105,146,159");
asm(" .byte 48,228,95,5,244,25,242,189,128,227,134,124,34,248,207,24");
asm(" .byte 242,87,168,15,134,252,63,2,194,134,252,116,240,95,49,228");
asm(" .byte 223,163,53,111,200,159,38,91,122,120,44,63,1,252,178,33");
asm(" .byte 223,114,195,124,252,158,198,220,80,254,35,192,44,67,158,97");
asm(" .byte 237,108,161,53,51,159,213,213,109,218,210,214,90,215,225,113");
asm(" .byte 182,123,234,234,88,221,55,238,175,171,114,109,106,238,240,184");
asm(" .byte 218,203,91,156,29,29,174,14,98,110,247,16,183,141,53,182");
asm(" .byte 180,53,222,87,183,201,229,241,52,111,113,81,193,166,198,198");
asm(" .byte 186,14,189,172,174,206,233,114,54,52,215,109,109,125,160,185");
asm(" .byte 181,169,174,209,237,174,115,183,171,95,78,182,81,221,70,93");
asm(" .byte 131,14,136,116,121,88,135,167,189,209,189,157,117,184,219,155");
asm(" .byte 91,61,27,89,99,91,107,171,171,209,195,182,182,98,129,223");
asm(" .byte 7,118,104,212,202,58,92,45,68,116,54,180,181,123,24,212");
asm(" .byte 112,55,55,81,69,79,91,11,219,226,218,210,1,49,117,117");
asm(" .byte 174,246,246,214,182,58,40,234,244,52,183,181,178,6,180,202");
asm(" .byte 60,141,40,115,122,60,237,172,221,229,108,34,94,106,171,221");
asm(" .byte 217,220,225,2,165,241,254,141,237,109,91,88,115,91,163,167");
asm(" .byte 133,117,108,71,75,91,168,167,40,107,115,163,81,180,67,58");
asm(" .byte 182,58,209,229,14,87,107,147,167,141,57,27,27,93,110,15");
asm(" .byte 196,110,18,98,235,234,168,67,98,24,183,56,155,91,217,3");
asm(" .byte 237,205,30,23,232,141,219,156,117,78,143,107,91,179,135,173");
asm(" .byte 40,47,175,155,87,80,194,86,172,94,85,86,94,87,84,80");
asm(" .byte 172,251,39,253,63,51,192,194,140,148,255,202,127,22,237,95");
asm(" .byte 9,255,173,16,190,128,214,219,250,91,182,143,39,239,243,128");
asm(" .byte 52,226,163,36,86,96,40,207,104,110,158,64,222,103,167,161");
asm(" .byte 220,103,240,37,122,185,105,180,188,183,82,98,147,225,200,50");
asm(" .byte 145,210,90,201,166,20,14,76,161,20,78,108,6,165,80,126");
asm(" .byte 22,165,112,156,249,148,194,225,169,148,162,94,49,165,112,156");
asm(" .byte 11,41,133,102,139,40,133,19,180,83,138,150,42,40,133,147");
asm(" .byte 91,73,41,28,240,106,74,161,140,131,82,56,196,26,74,97");
asm(" .byte 184,27,40,133,227,171,165,20,78,178,158,82,56,239,38,74");
asm(" .byte 225,180,55,83,10,167,216,66,41,156,182,155,82,56,76,15");
asm(" .byte 165,112,74,219,40,133,35,223,65,41,28,191,151,82,56,120");
asm(" .byte 63,165,112,254,123,40,133,115,223,75,41,54,137,125,148,98");
asm(" .byte 19,72,178,207,125,247,219,140,93,78,242,190,56,104,9,207");
asm(" .byte 229,150,112,128,79,12,63,26,78,186,250,43,238,120,153,22");
asm(" .byte 232,163,209,148,87,246,70,29,31,31,231,35,248,74,3,110");
asm(" .byte 55,224,11,13,184,106,192,103,25,112,197,128,103,26,112,217");
asm(" .byte 128,39,25,112,102,192,135,135,199,240,171,6,124,208,128,135");
asm(" .byte 13,248,5,3,126,206,128,159,49,224,33,3,126,204,128,247");
asm(" .byte 25,240,35,6,60,104,192,15,24,240,125,6,124,143,1,247");
asm(" .byte 26,240,109,6,220,109,192,55,27,240,122,194,251,130,49,22");
asm(" .byte 14,198,44,242,161,129,9,74,48,98,81,48,47,108,46,151");
asm(" .byte 67,1,110,9,205,229,41,44,192,83,212,0,255,105,60,126");
asm(" .byte 249,175,1,43,255,9,118,135,141,198,255,43,216,55,232,25");
asm(" .byte 40,55,171,143,134,153,210,243,14,118,184,216,180,120,161,244");
asm(" .byte 227,120,124,104,239,175,97,7,180,22,192,195,148,55,56,149");
asm(" .byte 73,74,208,204,148,151,172,224,137,132,96,19,10,234,146,77");
asm(" .byte 48,117,31,202,171,99,176,139,36,141,95,224,243,192,71,182");
asm(" .byte 146,228,208,23,144,164,158,27,144,148,215,6,76,172,58,98");
asm(" .byte 214,182,199,234,216,199,241,120,150,196,130,212,110,132,137,20");
asm(" .byte 75,106,104,132,15,203,41,139,234,253,57,94,73,169,138,34");
asm(" .byte 93,170,201,86,79,140,242,81,153,76,186,177,78,206,66,193");
asm(" .byte 8,149,47,70,25,202,211,71,218,134,44,51,150,79,50,180");
asm(" .byte 27,202,71,254,125,212,99,106,48,38,169,103,209,54,198,52");
asm(" .byte 212,201,177,108,135,242,88,13,104,211,77,86,45,236,169,142");
asm(" .byte 81,253,124,141,182,218,202,78,116,98,155,243,154,36,251,37");
asm(" .byte 180,93,19,51,49,71,234,159,226,241,23,152,18,140,97,9");
asm(" .byte 15,89,212,239,15,176,208,27,60,143,213,198,164,144,207,100");
asm(" .byte 9,73,214,36,61,52,64,25,244,66,153,100,247,69,177,154");
asm(" .byte 99,215,168,125,59,241,214,196,88,232,37,171,164,116,129,94");
asm(" .byte 21,249,35,228,201,24,71,120,130,24,245,9,248,49,21,227");
asm(" .byte 28,202,11,70,100,240,152,114,125,97,75,104,63,207,145,89");
asm(" .byte 84,178,7,99,25,169,123,35,22,185,139,247,43,61,28,250");
asm(" .byte 96,12,189,193,126,230,251,91,146,209,95,73,115,225,160,177");
asm(" .byte 123,134,234,111,128,156,81,25,50,100,48,200,8,5,99,217");
asm(" .byte 236,68,164,95,237,129,45,117,113,210,85,130,174,212,247,89");
asm(" .byte 74,128,35,102,188,40,201,229,209,19,39,131,145,148,19,93");
asm(" .byte 225,138,208,65,212,147,162,181,12,253,97,114,236,223,227,241");
asm(" .byte 228,126,181,139,211,28,80,123,10,234,100,22,237,231,217,105");
asm(" .byte 251,185,132,57,37,27,147,84,95,52,135,230,67,13,190,148");
asm(" .byte 89,52,198,43,201,222,176,74,125,100,222,40,6,123,136,217");
asm(" .byte 207,113,201,113,142,91,43,252,17,203,202,96,164,34,169,106");
asm(" .byte 212,118,84,216,152,153,213,70,40,77,97,222,176,132,190,194");
asm(" .byte 221,206,54,133,206,113,51,234,89,250,247,130,183,54,134,241");
asm(" .byte 123,132,236,69,213,198,80,79,135,151,205,125,183,201,30,224");
asm(" .byte 21,152,167,236,170,94,206,130,193,200,126,107,66,88,242,6");
asm(" .byte 35,121,127,181,59,106,113,63,20,102,114,48,150,136,62,189");
asm(" .byte 251,121,252,114,50,210,68,57,24,201,1,191,19,235,104,220");
asm(" .byte 205,187,163,146,99,87,52,137,250,44,251,195,211,65,191,7");
asm(" .byte 244,28,182,62,150,93,4,121,142,224,75,102,182,62,146,93");
asm(" .byte 229,159,60,188,75,31,247,109,185,1,222,129,190,249,111,223");
asm(" .byte 207,11,22,251,162,73,24,3,147,234,11,175,163,126,202,175");
asm(" .byte 115,147,61,24,153,192,42,97,163,149,233,52,247,100,199,102");
asm(" .byte 172,233,165,208,67,146,79,14,164,203,157,81,73,126,137,155");
asm(" .byte 67,190,232,108,157,198,83,104,205,163,223,38,7,165,108,182");
asm(" .byte 36,99,29,128,119,49,228,16,237,35,146,33,135,193,219,24");
asm(" .byte 150,228,174,48,100,199,10,49,127,84,135,213,7,105,173,205");
asm(" .byte 150,194,231,6,242,217,183,180,117,147,128,242,155,104,124,229");
asm(" .byte 75,188,28,56,182,153,44,171,250,6,95,139,241,198,86,148");
asm(" .byte 245,28,236,229,124,78,15,39,57,41,180,118,28,101,81,239");
asm(" .byte 156,30,126,9,52,210,217,59,199,199,153,220,205,195,200,91");
asm(" .byte 144,90,49,62,126,57,192,47,44,233,225,231,48,142,204,177");
asm(" .byte 43,236,134,12,162,83,189,22,148,141,212,59,35,202,245,245");
asm(" .byte 255,230,0,83,78,113,51,250,144,192,106,176,126,119,209,120");
asm(" .byte 60,2,31,23,35,58,34,73,78,118,108,97,65,156,24,253");
asm(" .byte 105,76,57,167,249,40,172,187,212,159,161,31,153,10,249,181");
asm(" .byte 134,176,5,169,255,246,46,190,67,216,254,195,240,127,210,75");
asm(" .byte 216,91,255,146,214,65,14,223,248,121,124,232,103,160,213,64");
asm(" .byte 167,4,200,51,205,194,56,98,111,91,137,189,205,28,134,236");
asm(" .byte 133,24,163,12,216,68,126,48,82,254,135,53,81,115,127,48");
asm(" .byte 146,59,171,43,50,125,17,214,131,210,13,255,208,29,85,208");
asm(" .byte 135,137,152,135,252,133,190,200,15,209,7,162,57,144,150,255");
asm(" .byte 161,59,106,185,122,104,224,29,200,255,33,14,31,151,145,38");
asm(" .byte 98,254,201,135,77,128,45,221,1,30,7,244,99,242,33,110");
asm(" .byte 81,247,243,36,5,235,67,109,196,58,60,196,153,218,141,113");
asm(" .byte 105,8,107,126,87,45,127,22,249,95,204,82,3,191,176,160");
asm(" .byte 77,179,218,213,103,65,62,27,237,90,216,33,126,126,28,187");
asm(" .byte 204,212,67,252,90,97,48,182,223,172,132,39,142,172,243,87");
asm(" .byte 30,191,221,94,54,137,185,201,230,96,235,71,206,173,137,30");
asm(" .byte 175,132,109,98,222,173,234,190,8,22,209,55,46,194,102,91");
asm(" .byte 200,103,163,189,205,144,231,101,119,197,18,212,224,51,94,53");
asm(" .byte 248,19,140,101,132,250,210,64,125,146,187,163,148,167,121,155");
asm(" .byte 67,243,132,177,130,63,188,184,16,117,205,74,239,59,38,90");
asm(" .byte 127,86,7,124,163,255,29,216,225,11,25,104,239,184,212,203");
asm(" .byte 43,192,43,169,225,216,113,155,31,237,119,71,221,66,183,99");
asm(" .byte 219,37,86,138,49,185,250,117,196,71,228,167,201,111,65,63");
asm(" .byte 172,155,72,146,189,155,227,156,55,164,106,123,73,48,150,2");
asm(" .byte 221,146,84,180,143,20,107,50,66,122,192,111,68,102,211,186");
asm(" .byte 180,251,163,86,216,7,124,206,11,25,144,79,107,91,22,50");
asm(" .byte 203,223,94,19,181,64,230,173,224,235,151,105,31,11,70,202");
asm(" .byte 38,118,71,11,144,63,134,124,89,65,119,116,2,211,229,80");
asm(" .byte 26,251,44,126,185,9,237,209,152,84,96,31,32,90,178,250");
asm(" .byte 42,127,11,244,189,130,95,26,223,29,157,6,126,239,72,254");
asm(" .byte 118,93,198,84,33,227,117,240,18,15,249,142,23,128,51,187");
asm(" .byte 131,250,244,6,141,21,205,121,28,115,116,129,232,218,122,13");
asm(" .byte 70,62,21,252,156,82,199,86,226,157,71,251,8,213,65,4");
asm(" .byte 11,95,16,134,187,11,166,23,163,111,210,239,245,254,45,195");
asm(" .byte 184,173,3,60,10,248,17,128,230,87,73,13,112,5,254,69");
asm(" .byte 145,0,75,3,252,3,200,243,162,31,203,0,103,88,175,182");
asm(" .byte 86,176,174,46,50,204,197,25,213,207,223,70,121,166,164,211");
asm(" .byte 209,192,197,43,218,62,31,142,101,230,250,248,239,80,118,9");
asm(" .byte 243,151,47,93,95,47,31,115,120,14,101,22,67,189,231,68");
asm(" .byte 61,11,234,189,140,178,39,80,239,210,13,237,93,66,123,39");
asm(" .byte 81,118,149,141,213,219,33,234,93,85,124,252,151,40,107,113");
asm(" .byte 4,184,125,233,15,56,245,125,130,168,87,171,249,134,32,173");
asm(" .byte 241,136,221,182,139,119,127,166,249,225,88,59,250,100,209,252");
asm(" .byte 110,48,86,89,184,43,250,244,103,122,254,73,164,51,84,178");
asm(" .byte 187,30,173,29,216,218,5,106,227,56,116,115,219,95,28,60");
asm(" .byte 252,89,124,232,69,140,215,147,194,254,200,14,127,141,252,2");
asm(" .byte 45,134,193,58,71,91,125,202,137,136,228,32,191,24,140,153");
asm(" .byte 117,91,139,109,19,243,136,208,248,133,36,5,243,0,219,176");
asm(" .byte 168,121,220,3,253,60,159,233,107,1,123,125,150,44,116,181");
asm(" .byte 202,186,111,216,41,202,104,191,31,161,83,140,182,88,208,77");
asm(" .byte 130,158,164,249,12,54,180,139,232,20,75,9,219,192,26,215");
asm(" .byte 228,91,84,242,7,187,56,179,135,53,249,119,192,6,114,107");
asm(" .byte 176,118,150,206,125,119,195,252,0,255,22,236,60,9,118,158");
asm(" .byte 143,126,211,58,253,86,65,48,182,68,179,49,71,244,187,18");
asm(" .byte 187,56,5,118,244,48,210,36,245,45,62,78,121,139,155,102");
asm(" .byte 248,96,215,111,113,51,252,216,120,74,243,187,34,25,144,99");
asm(" .byte 70,172,129,179,78,22,245,209,170,118,133,19,148,106,110,149");
asm(" .byte 171,17,227,149,71,165,25,93,240,5,85,49,156,165,178,18");
asm(" .byte 137,134,50,162,145,191,148,146,218,99,210,172,46,19,218,54");
asm(" .byte 181,161,156,226,41,137,189,6,255,221,21,182,176,202,200,39");
asm(" .byte 241,120,169,153,85,99,63,168,230,44,163,19,99,139,186,42");
asm(" .byte 240,204,206,136,121,17,0,237,58,81,111,111,73,128,31,43");
asm(" .byte 236,225,253,133,62,205,151,211,184,77,144,223,194,30,247,150");
asm(" .byte 214,214,68,224,104,39,50,137,82,232,108,82,194,17,226,173");
asm(" .byte 217,21,224,27,172,1,173,239,52,7,147,209,247,254,182,3");
asm(" .byte 252,216,206,3,60,161,254,235,240,161,119,197,250,219,246,240");
asm(" .byte 239,96,191,194,58,124,74,157,28,224,127,87,8,127,141,58");
asm(" .byte 38,231,233,216,108,240,255,2,115,96,103,55,97,191,103,201");
asm(" .byte 118,54,37,150,49,89,243,23,102,216,95,186,5,124,246,250");
asm(" .byte 211,49,5,124,63,35,127,38,233,124,146,52,37,118,53,99");
asm(" .byte 140,239,146,5,246,10,121,55,129,239,48,241,37,11,190,228");
asm(" .byte 41,177,179,6,190,231,136,239,222,211,177,20,240,209,126,97");
asm(" .byte 74,212,249,76,137,83,98,71,12,124,123,193,103,218,116,58");
asm(" .byte 102,2,95,55,201,179,10,121,214,41,49,191,129,175,137,228");
asm(" .byte 185,78,199,134,63,141,95,246,129,79,22,253,144,209,143,122");
asm(" .byte 3,223,34,240,201,232,199,191,130,111,59,181,155,32,218,77");
asm(" .byte 152,18,91,100,224,203,160,118,55,158,142,13,128,175,141,218");
asm(" .byte 21,124,18,248,50,13,124,87,204,104,23,124,23,193,231,34");
asm(" .byte 121,227,132,188,113,83,98,239,167,143,241,245,131,207,212,124");
asm(" .byte 58,118,14,124,247,128,79,17,250,41,208,239,156,129,239,32");
asm(" .byte 248,20,232,119,26,124,149,212,46,248,40,230,238,75,39,63");
asm(" .byte 144,24,221,65,237,161,252,87,40,95,165,237,65,119,105,229");
asm(" .byte 189,40,79,192,186,74,196,26,222,0,158,231,81,190,84,236");
asm(" .byte 195,86,177,134,123,177,102,55,209,154,168,119,68,231,163,78");
asm(" .byte 146,250,38,214,194,155,218,90,200,152,7,253,96,199,103,16");
asm(" .byte 179,148,205,162,181,95,73,177,84,242,25,216,214,34,200,182");
asm(" .byte 64,230,97,200,204,135,204,201,168,147,129,186,83,24,234,22");
asm(" .byte 251,34,55,145,12,236,251,44,197,23,161,125,88,134,172,107");
asm(" .byte 57,136,81,176,190,31,92,130,61,148,249,194,87,18,217,229");
asm(" .byte 190,162,0,63,171,199,77,218,254,69,242,207,78,242,241,129");
asm(" .byte 180,0,127,194,20,224,15,67,126,6,228,103,66,246,84,161");
asm(" .byte 87,22,112,90,163,95,163,20,246,222,2,25,231,114,40,166");
asm(" .byte 175,140,96,127,76,62,7,253,142,160,126,19,234,111,71,125");
asm(" .byte 11,234,195,183,14,29,64,62,1,253,70,108,24,43,5,189");
asm(" .byte 184,33,192,15,44,121,76,59,151,221,68,125,23,227,50,21");
asm(" .byte 229,10,198,37,119,204,135,37,167,34,142,146,213,115,90,223");
asm(" .byte 210,128,83,223,210,41,93,228,139,28,93,178,155,79,6,158");
asm(" .byte 33,202,111,18,229,83,68,121,31,202,207,216,2,60,59,47");
asm(" .byte 192,159,85,122,34,180,134,239,131,204,145,53,124,33,185,103");
asm(" .byte 116,29,127,10,190,235,214,178,172,132,205,170,20,38,29,109");
asm(" .byte 168,147,18,162,253,62,24,179,134,207,195,111,156,231,19,225");
asm(" .byte 55,18,128,79,132,207,72,164,52,27,126,35,124,233,29,58");
asm(" .byte 71,72,114,107,56,119,0,99,42,247,115,107,184,31,49,81");
asm(" .byte 117,44,119,16,113,90,54,98,71,228,19,65,191,66,103,199");
asm(" .byte 220,147,49,187,114,18,49,98,87,36,47,131,198,231,85,62");
asm(" .byte 65,126,149,231,101,234,241,221,68,224,147,64,163,115,155,21");
asm(" .byte 241,165,69,57,207,205,242,165,119,24,116,78,160,120,19,250");
asm(" .byte 38,82,170,249,158,206,48,201,176,40,104,19,242,71,100,36");
asm(" .byte 0,79,4,13,254,45,138,115,199,236,99,20,47,151,246,104");
asm(" .byte 99,63,141,226,106,218,99,209,198,84,192,56,196,189,89,72");
asm(" .byte 191,134,24,35,27,112,51,96,26,32,7,64,62,216,44,191");
asm(" .byte 254,206,205,236,45,158,77,254,15,253,36,61,166,33,47,211");
asm(" .byte 185,9,186,228,0,55,234,146,13,89,55,27,250,51,13,120");
asm(" .byte 14,104,151,41,30,118,4,99,123,69,156,39,107,251,87,55");
asm(" .byte 218,212,125,38,237,27,86,61,166,138,253,236,147,248,229,239");
asm(" .byte 252,65,143,53,174,118,234,177,218,158,7,54,174,223,6,7");
asm(" .byte 253,50,246,74,218,55,95,67,250,47,128,8,224,93,192,123");
asm(" .byte 128,143,1,210,9,198,198,1,82,1,83,1,185,128,91,1");
asm(" .byte 11,1,41,159,199,227,118,164,43,1,123,33,235,52,14,119");
asm(" .byte 118,252,191,167,86,98,87,87,204,125,119,199,186,0,223,136");
asm(" .byte 125,172,88,246,107,62,156,98,12,55,116,41,130,109,166,48");
asm(" .byte 63,175,159,20,224,79,195,150,107,145,30,69,122,126,73,175");
asm(" .byte 54,166,55,195,86,86,131,182,58,71,95,67,118,224,23,114");
asm(" .byte 122,249,110,224,139,128,135,129,251,104,13,0,191,4,220,75");
asm(" .byte 235,24,248,0,240,7,129,31,16,114,176,215,36,103,130,158");
asm(" .byte 9,57,15,208,90,4,158,1,220,3,92,6,46,3,167,125");
asm(" .byte 237,211,137,24,51,224,173,180,222,64,183,0,191,15,248,241");
asm(" .byte 181,1,78,101,159,78,11,240,77,200,15,3,31,6,222,100");
asm(" .byte 104,3,251,125,242,32,232,131,160,127,11,244,141,211,70,98");
asm(" .byte 137,92,126,225,142,235,231,198,66,231,28,224,20,131,45,255");
asm(" .byte 68,139,23,99,199,80,239,24,234,175,188,171,155,55,128,150");
asm(" .byte 128,24,37,17,227,243,140,149,13,133,74,245,54,16,59,37");
asm(" .byte 115,200,214,242,56,75,60,56,73,235,91,169,31,117,119,160");
asm(" .byte 110,141,168,247,49,120,46,128,39,116,175,159,75,54,95,236");
asm(" .byte 66,169,159,15,47,70,223,242,122,40,14,136,153,222,11,252");
asm(" .byte 208,129,254,30,253,17,99,254,131,140,245,1,158,160,11,116");
asm(" .byte 232,149,148,135,253,185,168,135,159,89,19,24,105,51,139,226");
asm(" .byte 180,25,104,195,14,125,167,163,141,108,224,41,104,111,225,39");
asm(" .byte 90,108,22,123,11,237,165,168,189,188,9,118,108,41,242,115");
asm(" .byte 147,22,95,179,161,99,133,189,92,146,210,98,37,180,230,149");
asm(" .byte 94,208,27,195,215,116,157,31,167,181,114,100,182,159,159,191");
asm(" .byte 57,192,67,19,32,27,178,126,67,227,142,186,73,144,49,75");
asm(" .byte 59,39,6,99,71,111,214,219,181,162,156,240,94,240,102,209");
asm(" .byte 152,213,107,241,152,182,95,120,239,243,107,244,155,133,30,41");
asm(" .byte 13,126,126,231,18,93,23,73,200,203,32,125,224,95,175,230");
asm(" .byte 99,174,165,84,138,83,147,83,228,253,220,146,183,95,243,185");
asm(" .byte 56,239,150,82,255,83,208,127,138,215,88,26,98,149,252,128");
asm(" .byte 118,22,166,59,9,35,47,108,229,113,179,24,171,96,11,198");
asm(" .byte 162,232,49,156,121,171,99,102,140,235,195,116,247,115,136,177");
asm(" .byte 89,0,249,199,136,223,1,251,0,199,1,73,121,143,113,247");
asm(" .byte 162,128,22,255,236,132,76,138,135,236,44,77,187,123,131,188");
asm(" .byte 244,102,212,205,132,172,108,240,173,92,141,125,139,85,71,38");
asm(" .byte 67,79,186,199,216,111,54,135,159,203,14,240,188,2,95,116");
asm(" .byte 220,98,95,212,164,248,225,83,253,225,42,156,121,222,187,22");
asm(" .byte 191,92,117,71,119,52,81,241,69,123,193,227,78,9,240,43");
asm(" .byte 160,145,173,83,249,239,62,209,199,229,193,37,99,243,147,141");
asm(" .byte 182,50,138,116,187,162,253,154,236,177,22,245,50,80,254,27");
asm(" .byte 240,211,152,59,178,245,24,253,167,144,117,236,206,0,15,206");
asm(" .byte 238,213,116,119,144,173,11,187,95,131,126,92,89,226,231,131");
asm(" .byte 133,126,62,3,252,212,222,171,215,244,118,255,113,164,221,59");
asm(" .byte 244,118,173,104,35,5,169,157,230,162,237,49,222,116,39,197");
asm(" .byte 94,55,69,105,191,108,161,57,69,185,140,114,51,236,232,253");
asm(" .byte 120,124,136,108,198,82,164,219,13,181,85,132,24,93,18,50");
asm(" .byte 100,208,201,111,154,232,238,2,249,57,56,87,220,3,25,114");
asm(" .byte 131,110,59,7,199,7,248,95,127,162,219,83,74,209,110,158");
asm(" .byte 162,62,38,250,191,155,43,154,109,62,198,7,74,3,163,227");
asm(" .byte 159,172,217,67,15,230,184,103,244,108,14,189,158,154,44,120");
asm(" .byte 143,17,175,146,22,165,117,110,22,118,50,17,101,41,12,117");
asm(" .byte 96,43,76,118,106,247,47,227,72,71,156,93,82,210,124,92");
asm(" .byte 6,253,10,246,113,73,238,68,189,202,152,12,218,32,242,114");
asm(" .byte 158,239,84,112,182,239,84,19,100,166,40,61,252,137,44,180");
asm(" .byte 137,56,57,15,54,120,36,75,191,127,201,65,127,30,44,236");
asm(" .byte 138,62,49,219,167,197,205,103,150,244,240,141,107,187,163,21");
asm(" .byte 243,95,229,51,208,39,178,127,187,173,137,75,56,35,219,111");
asm(" .byte 233,230,203,54,98,223,89,220,29,165,51,68,130,242,198,59");
asm(" .byte 193,44,248,46,204,131,84,191,59,154,128,88,138,217,253,225");
asm(" .byte 15,177,87,192,247,68,102,107,247,160,39,7,146,181,180,159");
asm(" .byte 95,186,77,191,167,169,130,207,178,99,191,48,41,206,240,173");
asm(" .byte 215,40,110,88,31,91,133,254,30,111,14,240,35,165,122,140");
asm(" .byte 129,57,184,120,244,27,111,114,155,90,161,141,125,33,99,143");
asm(" .byte 28,248,231,199,120,113,41,217,117,109,76,69,187,234,108,221");
asm(" .byte 14,48,78,169,223,32,91,128,220,34,237,220,219,25,38,91");
asm(" .byte 155,7,92,139,253,132,76,58,43,61,39,112,172,173,100,155");
asm(" .byte 166,215,50,254,179,130,221,81,172,187,193,227,201,143,241,3");
asm(" .byte 176,151,75,239,82,27,119,197,206,77,13,240,137,114,48,213");
asm(" .byte 204,186,223,165,59,139,51,200,179,80,55,47,20,109,177,250");
asm(" .byte 93,97,230,14,70,54,143,11,240,217,160,253,226,19,61,158");
asm(" .byte 156,160,6,211,37,182,59,45,81,61,53,197,36,238,105,85");
asm(" .byte 123,48,70,186,94,88,24,224,254,91,3,220,6,221,118,204");
asm(" .byte 241,241,4,204,5,233,233,185,53,240,3,240,62,190,140,238");
asm(" .byte 132,81,103,51,120,168,188,105,42,165,61,60,77,216,250,231");
asm(" .byte 72,123,73,6,104,181,152,15,178,31,186,75,205,144,177,183");
asm(" .byte 203,240,253,50,221,227,119,115,21,245,206,192,102,200,214,127");
asm(" .byte 253,113,252,114,253,202,0,157,9,99,21,216,27,189,176,43");
asm(" .byte 255,61,126,254,77,200,251,61,202,146,48,47,171,231,119,107");
asm(" .byte 243,74,247,137,180,135,124,19,253,235,199,89,155,246,116,172");
asm(" .byte 167,72,24,124,86,230,136,212,38,116,195,183,179,167,46,127");
asm(" .byte 172,159,51,41,78,186,197,194,178,254,2,125,175,157,127,136");
asm(" .byte 15,223,173,251,115,218,111,27,113,94,180,75,63,208,206,209");
asm(" .byte 245,192,55,22,116,71,67,105,56,235,165,118,133,67,114,42");
asm(" .byte 226,10,22,173,204,163,251,222,180,216,6,9,103,3,181,60");
asm(" .byte 154,141,88,194,148,15,31,1,125,2,31,83,252,238,128,77");
asm(" .byte 5,35,14,212,103,69,63,24,241,35,89,180,38,143,148,238");
asm(" .byte 226,142,76,236,133,137,1,254,247,31,235,123,18,237,77,116");
asm(" .byte 247,119,164,84,231,45,147,216,11,77,11,96,119,246,138,40");
asm(" .byte 173,1,134,245,148,173,194,126,237,190,48,221,11,46,148,224");
asm(" .byte 71,217,250,136,181,62,72,119,194,179,41,149,212,223,14,40");
asm(" .byte 168,59,28,143,63,158,160,82,156,93,29,49,59,196,61,167");
asm(" .byte 242,198,64,14,198,156,238,17,173,238,141,97,51,187,59,118");
asm(" .byte 11,233,70,107,69,61,133,241,126,147,87,161,238,120,156,69");
asm(" .byte 161,79,132,236,68,193,120,95,25,233,11,236,5,135,219,139");
asm(" .byte 74,201,33,172,133,55,248,6,200,186,9,188,38,245,13,77");
asm(" .byte 95,24,78,214,12,148,177,250,55,120,57,108,105,45,236,20");
asm(" .byte 54,242,136,10,60,69,34,91,193,57,20,118,178,20,105,18");
asm(" .byte 120,159,68,95,87,3,47,165,51,19,248,150,1,55,129,254");
asm(" .byte 75,208,107,128,31,0,253,46,164,159,97,204,76,104,255,19");
asm(" .byte 228,103,137,182,201,118,62,210,226,201,55,184,10,252,125,216");
asm(" .byte 162,13,233,120,200,255,38,210,171,40,155,133,51,61,221,119");
asm(" .byte 29,132,111,216,54,69,95,191,102,197,132,188,143,111,65,159");
asm(" .byte 206,39,4,248,33,218,199,192,119,64,220,233,70,105,109,161");
asm(" .byte 62,221,103,4,11,123,180,118,222,2,237,232,45,136,187,196");
asm(" .byte 217,227,95,208,70,223,146,30,109,239,121,19,248,32,206,199");
asm(" .byte 251,110,10,112,242,7,123,33,147,1,223,3,185,231,64,63");
asm(" .byte 135,115,244,116,208,61,160,135,129,119,209,222,135,190,156,129");
asm(" .byte 221,171,37,250,61,39,233,184,1,117,102,205,60,196,243,65");
asm(" .byte 91,14,221,30,50,179,161,205,37,180,215,232,177,179,169,62");
asm(" .byte 104,166,241,124,129,244,197,124,109,128,141,153,212,229,209,68");
asm(" .byte 177,167,222,14,122,253,19,140,109,6,120,0,59,0,126,192");
asm(" .byte 94,192,32,192,242,183,140,13,151,4,180,103,25,239,35,165");
asm(" .byte 250,5,192,175,0,255,4,251,200,32,82,216,197,208,0,210");
asm(" .byte 143,144,210,253,221,165,18,109,111,139,44,178,30,210,248,77");
asm(" .byte 180,110,20,71,20,126,233,149,179,197,24,51,49,62,215,10");
asm(" .byte 131,233,201,116,134,133,188,107,147,170,200,7,188,114,109,73");
asm(" .byte 48,214,99,50,133,169,111,159,162,223,139,172,88,223,168,159");
asm(" .byte 170,221,245,195,47,91,15,105,178,255,56,140,216,4,178,158");
asm(" .byte 192,89,144,158,119,120,252,116,86,59,160,245,57,129,246,92");
asm(" .byte 240,110,128,30,21,130,159,244,8,163,78,112,210,30,190,33");
asm(" .byte 225,144,182,62,201,95,210,222,158,144,74,207,240,180,88,4");
asm(" .byte 123,245,107,154,12,122,85,231,165,143,117,31,159,200,238,138");
asm(" .byte 252,61,61,75,42,214,247,236,12,172,167,141,104,147,116,28");
asm(" .byte 198,185,254,85,200,125,31,105,13,206,255,175,12,235,113,91");
asm(" .byte 27,230,110,6,230,130,236,239,30,97,127,100,115,54,165,42");
asm(" .byte 122,55,99,223,160,88,113,61,250,123,28,231,222,227,253,164");
asm(" .byte 123,143,54,159,212,182,69,233,74,163,148,238,52,76,202,165");
asm(" .byte 119,48,38,230,73,240,175,255,11,178,15,160,141,50,172,175");
asm(" .byte 131,208,109,25,116,216,225,131,125,149,30,128,79,213,158,249");
asm(" .byte 37,239,200,208,207,244,103,112,246,62,9,254,53,208,227,232");
asm(" .byte 189,123,248,102,11,221,151,35,6,128,254,190,97,242,107,119");
asm(" .byte 83,236,120,177,234,175,186,233,153,77,148,246,60,215,52,218");
asm(" .byte 227,254,1,103,155,206,104,50,157,253,193,215,72,227,195,124");
asm(" .byte 239,108,152,79,54,167,175,215,175,211,253,27,198,69,198,26");
asm(" .byte 123,143,158,147,192,127,232,124,52,142,218,253,118,242,135,22");
asm(" .byte 118,121,56,61,192,119,89,217,229,11,237,1,158,232,222,29");
asm(" .byte 238,75,126,136,143,212,251,19,234,209,254,63,67,127,46,152");
asm(" .byte 92,38,246,167,149,226,57,33,157,195,233,254,224,251,195,116");
asm(" .byte 119,201,134,250,204,250,217,252,212,176,126,230,78,208,238,164");
asm(" .byte 238,166,56,226,226,37,156,219,233,46,195,170,236,142,238,66");
asm(" .byte 185,76,123,179,99,87,148,124,220,175,63,143,151,146,143,38");
asm(" .byte 189,239,129,15,90,93,69,123,132,238,159,239,254,79,252,243");
asm(" .byte 58,131,127,150,230,232,254,121,193,240,152,127,94,129,250,33");
asm(" .byte 225,115,233,121,107,13,244,144,161,199,170,97,221,47,255,43");
asm(" .byte 230,232,146,77,143,67,66,179,175,247,193,243,255,47,124,240");
asm(" .byte 136,223,181,170,228,139,171,99,121,208,227,160,184,187,120,80");
asm(" .byte 140,207,205,208,221,204,118,107,113,71,18,230,242,97,205,46");
asm(" .byte 29,177,100,237,89,172,35,54,153,252,49,214,198,230,52,156");
asm(" .byte 31,69,29,25,117,242,161,239,133,217,136,153,112,230,167,56");
asm(" .byte 86,113,29,226,43,211,232,25,79,48,82,43,248,50,201,158");
asm(" .byte 103,146,61,188,206,75,208,62,221,127,204,23,243,87,140,244");
asm(" .byte 115,140,197,0,206,200,23,178,40,6,79,141,77,38,159,152");
asm(" .byte 172,223,45,219,229,78,243,133,44,159,149,41,175,91,85,165");
asm(" .byte 204,132,88,205,250,39,180,67,247,225,212,46,221,51,92,192");
asm(" .byte 88,253,17,52,106,243,92,106,128,223,33,218,29,132,252,189");
asm(" .byte 245,224,67,60,99,247,166,147,63,73,126,46,149,238,205,17");
asm(" .byte 147,127,164,143,249,247,49,230,111,163,238,172,173,135,184,93");
asm(" .byte 165,243,217,110,62,32,193,254,48,54,243,133,28,28,199,146");
asm(" .byte 19,97,243,244,140,114,27,234,223,34,232,255,3,244,124,212");
asm(" .byte 91,40,211,179,218,96,68,17,244,75,143,156,158,252,41,160");
asm(" .byte 246,187,167,39,47,2,156,70,127,168,252,32,234,238,64,91");
asm(" .byte 116,214,255,38,206,231,173,116,70,7,252,24,240,19,192,63");
asm(" .byte 2,126,3,248,3,224,223,0,127,2,88,78,226,172,2,200");
asm(" .byte 2,228,0,242,0,37,128,5,128,13,40,95,131,244,195,151");
asm(" .byte 24,107,64,250,29,192,32,100,251,79,234,247,7,223,69,186");
asm(" .byte 31,208,11,248,107,192,97,192,79,1,79,3,66,128,1,212");
asm(" .byte 251,53,210,151,1,231,1,111,3,28,144,25,67,122,6,176");
asm(" .byte 30,248,31,79,210,27,5,53,169,204,4,144,106,82,63,61");
asm(" .byte 169,63,211,11,238,150,152,5,245,199,3,226,233,208,13,233");
asm(" .byte 116,192,2,192,50,192,122,64,59,224,17,192,97,192,47,0");
asm(" .byte 175,0,222,2,12,3,188,255,196,216,148,83,140,125,6,60");
asm(" .byte 21,155,181,167,150,158,19,126,167,98,42,240,23,66,144,135");
asm(" .byte 244,22,192,28,64,17,224,14,192,2,64,15,116,122,10,160");
asm(" .byte 64,135,16,210,215,1,33,212,93,131,178,26,64,61,100,214");
asm(" .byte 33,221,12,216,2,184,31,176,19,176,11,64,119,33,116,15");
asm(" .byte 178,15,248,143,0,12,50,150,33,253,59,192,135,128,79,0");
asm(" .byte 65,148,39,66,198,207,157,18,155,136,148,238,255,105,190,110");
asm(" .byte 62,165,143,169,95,206,209,230,250,111,63,210,231,58,223,122");
asm(" .byte 104,114,175,156,51,57,97,100,142,81,254,99,81,70,126,10");
asm(" .byte 229,220,13,26,61,207,198,254,49,68,60,199,145,255,129,224");
asm(" .byte 89,34,120,206,130,182,4,60,79,128,142,56,101,232,2,203");
asm(" .byte 209,108,112,132,143,238,27,6,65,75,144,31,138,238,51,208");
asm(" .byte 14,226,60,52,117,2,252,205,104,204,129,243,85,106,14,207");
asm(" .byte 199,62,184,22,124,51,232,28,43,233,178,186,12,122,5,197");
asm(" .byte 61,37,214,108,242,52,240,44,20,60,219,5,15,61,155,94");
asm(" .byte 41,104,30,65,163,119,56,146,234,31,138,214,74,57,154,239");
asm(" .byte 104,253,104,204,183,202,234,239,120,147,126,191,147,181,208,150");
asm(" .byte 163,237,123,231,177,46,169,207,123,4,127,195,71,163,207,99");
asm(" .byte 146,103,161,207,249,147,15,241,15,63,26,243,189,47,27,124");
asm(" .byte 175,12,63,237,205,69,63,42,247,115,58,251,164,32,127,10");
asm(" .byte 126,150,238,114,76,185,157,225,144,146,198,115,28,214,104,213");
asm(" .byte 122,248,65,199,100,109,109,147,239,37,159,43,57,244,117,77");
asm(" .byte 49,8,197,243,35,50,233,92,122,222,153,195,51,165,199,52");
asm(" .byte 223,130,125,233,63,244,231,57,6,153,116,95,84,131,212,133");
asm(" .byte 125,203,66,119,43,136,167,26,112,6,212,124,125,190,126,135");
asm(" .byte 196,236,221,252,242,135,104,215,237,136,146,254,59,210,48,126");
asm(" .byte 216,91,179,213,67,188,194,148,195,135,80,182,85,243,171,119");
asm(" .byte 107,227,68,116,210,43,152,151,99,125,86,221,45,153,181,119");
asm(" .byte 100,200,63,246,38,245,157,204,177,78,98,123,147,106,111,57");
asm(" .byte 164,197,62,175,66,222,190,162,28,110,17,207,220,94,38,221");
asm(" .byte 49,22,33,237,221,16,140,71,40,91,123,47,165,54,149,116");
asm(" .byte 79,141,157,32,89,74,89,148,206,63,87,16,63,157,45,195");
asm(" .byte 28,204,105,136,138,103,247,111,224,44,252,219,43,57,189,60");
asm(" .byte 222,234,143,226,28,252,91,73,222,29,78,152,211,245,250,119");
asm(" .byte 133,254,232,207,207,175,44,241,255,98,16,240,52,150,228,162");
asm(" .byte 244,28,200,233,229,116,247,47,155,115,248,63,128,175,76,243");
asm(" .byte 119,119,211,252,103,209,61,98,217,71,250,51,203,195,36,67");
asm(" .byte 13,199,44,54,63,15,80,236,183,164,151,155,221,15,133,175");
asm(" .byte 236,244,99,95,247,135,255,109,231,239,56,213,67,220,157,181");
asm(" .byte 15,114,7,33,183,9,50,255,6,245,230,66,230,149,71,123");
asm(" .byte 57,221,105,124,254,189,61,209,73,222,61,81,58,63,89,188");
asm(" .byte 123,232,1,45,226,124,196,107,246,160,68,103,198,58,242,249");
asm(" .byte 216,87,115,169,13,200,118,35,77,162,61,80,209,247,72,74");
asm(" .byte 37,153,246,193,74,218,35,31,49,169,244,156,29,229,193,47");
asm(" .byte 236,145,143,211,153,164,81,236,203,217,22,221,94,159,255,112");
asm(" .byte 236,185,34,189,27,161,63,71,164,119,116,126,55,128,179,250");
asm(" .byte 64,34,251,38,61,115,123,132,236,44,25,122,36,215,147,252");
asm(" .byte 187,34,9,238,160,13,251,177,237,237,120,188,180,165,56,135");
asm(" .byte 183,204,200,225,245,114,57,206,207,221,92,65,172,68,182,147");
asm(" .byte 1,156,238,7,20,216,210,158,140,28,174,189,227,0,144,1");
asm(" .byte 245,104,119,7,234,197,11,112,206,180,251,195,116,175,96,177");
asm(" .byte 55,132,175,172,243,241,5,255,213,62,238,251,178,62,158,231");
asm(" .byte 247,136,62,14,163,143,18,206,185,15,162,45,201,237,142,226");
asm(" .byte 204,53,143,230,132,158,119,88,208,135,177,190,94,68,95,79");
asm(" .byte 15,36,179,191,24,237,235,56,138,61,220,193,216,56,59,181");
asm(" .byte 85,19,177,4,131,54,11,250,251,123,244,151,246,224,241,246");
asm(" .byte 221,209,69,174,28,190,97,114,14,79,131,252,75,165,111,106");
asm(" .byte 118,78,107,116,243,204,28,173,223,45,40,83,212,174,232,76");
asm(" .byte 26,99,244,113,91,9,232,246,70,244,113,55,223,182,44,135");
asm(" .byte 143,60,195,78,166,103,230,133,189,163,103,248,248,52,127,148");
asm(" .byte 127,16,191,252,196,100,221,30,233,121,114,208,154,195,211,33");
asm(" .byte 135,252,238,115,37,52,150,56,75,22,234,247,59,136,23,248");
asm(" .byte 200,250,159,73,235,31,122,93,98,250,250,207,253,79,214,255");
asm(" .byte 84,163,79,17,235,159,98,233,73,152,51,23,234,57,144,63");
asm(" .byte 245,129,254,204,122,34,100,93,85,31,27,233,231,43,84,94");
asm(" .byte 113,19,250,4,191,172,241,8,191,71,125,255,151,15,244,243");
asm(" .byte 247,85,251,110,109,44,26,62,212,239,5,168,46,98,165,139");
asm(" .byte 219,150,143,245,63,227,63,209,49,133,125,49,230,124,252,131");
asm(" .byte 209,247,70,94,33,57,181,31,234,239,4,252,242,3,125,93");
asm(" .byte 94,85,253,252,18,218,96,245,189,218,29,206,170,107,241,228");
asm(" .byte 65,232,58,152,144,195,127,250,129,30,23,221,137,177,220,248");
asm(" .byte 118,119,244,234,67,47,14,210,57,227,73,200,40,249,44,62");
asm(" .byte 68,239,85,93,153,174,221,147,68,36,216,193,128,89,63,39");
asm(" .byte 124,7,245,46,225,108,226,85,124,218,59,25,22,249,39,218");
asm(" .byte 185,21,54,147,21,66,140,219,63,143,222,77,234,228,33,164");
asm(" .byte 151,144,210,57,38,140,244,24,210,1,164,244,174,142,5,107");
asm(" .byte 163,90,140,39,61,111,160,184,204,162,252,156,211,187,56,136");
asm(" .byte 255,248,92,216,64,38,61,255,7,189,69,216,211,3,72,235");
asm(" .byte 231,5,172,178,210,67,239,241,153,101,197,71,239,253,13,73");
asm(" .byte 226,125,21,122,14,173,233,35,61,246,14,157,235,238,131,124");
asm(" .byte 21,109,30,193,57,144,158,247,75,208,151,228,75,226,172,50");
asm(" .byte 72,247,114,182,221,225,145,250,111,211,59,134,232,99,200,118");
asm(" .byte 110,244,126,44,113,130,55,106,65,159,238,33,93,145,110,41");
asm(" .byte 12,198,62,69,60,157,137,252,249,162,0,127,31,241,239,251");
asm(" .byte 121,244,126,132,143,31,193,57,237,10,202,168,221,15,49,134");
asm(" .byte 3,38,125,188,38,136,126,218,200,38,193,63,9,54,155,255");
asm(" .byte 129,254,126,69,37,210,16,228,88,196,115,10,248,215,161,9");
asm(" .byte 232,123,50,171,74,103,218,187,161,39,172,22,229,144,118,207");
asm(" .byte 160,235,94,29,133,47,40,213,222,103,82,94,211,214,10,157");
asm(" .byte 151,188,144,187,13,122,180,160,205,59,33,243,77,180,191,25");
asm(" .byte 184,5,182,66,231,181,89,31,140,157,77,178,139,2,167,48");
asm(" .byte 62,47,157,47,244,157,58,0,24,15,219,73,214,124,0,197");
asm(" .byte 237,231,112,134,215,206,201,89,18,243,106,243,75,245,51,53");
asm(" .byte 91,113,196,194,182,192,169,243,235,124,167,46,44,197,249,211");
asm(" .byte 65,119,188,1,158,17,242,71,159,171,236,143,100,194,214,38");
asm(" .byte 177,131,131,51,194,254,232,36,246,189,193,98,145,94,193,185");
asm(" .byte 225,249,162,254,136,105,145,47,250,188,173,63,98,166,116,94");
asm(" .byte 127,228,2,98,111,122,102,122,234,125,221,134,237,146,159,215");
asm(" .byte 98,255,169,204,241,71,255,248,190,254,92,121,3,244,168,252");
asm(" .byte 103,146,115,124,48,225,143,148,62,61,72,184,49,77,68,155");
asm(" .byte 51,85,138,185,190,55,232,197,62,180,183,210,175,61,187,208");
asm(" .byte 159,215,238,141,126,34,228,227,76,156,69,244,78,244,165,30");
asm(" .byte 109,63,98,98,151,95,22,126,159,177,112,250,18,122,47,15");
asm(" .byte 115,146,14,191,179,118,36,230,131,78,240,195,47,16,125,5");
asm(" .byte 244,90,41,232,153,160,219,4,253,44,124,107,153,160,159,97");
asm(" .byte 218,157,248,11,100,227,101,224,95,36,232,22,240,207,20,244");
asm(" .byte 48,248,231,11,250,37,240,231,136,187,235,31,67,207,2,232");
asm(" .byte 35,171,189,88,35,233,154,15,57,142,243,203,126,208,233,60");
asm(" .byte 76,239,86,246,33,255,189,247,233,206,35,24,123,62,245,124");
asm(" .byte 228,87,249,231,35,51,62,28,141,17,179,122,113,86,210,202");
asm(" .byte 210,206,71,126,174,244,70,110,254,112,52,110,163,251,183,216");
asm(" .byte 243,233,231,35,207,22,156,143,76,25,237,55,203,250,119,236");
asm(" .byte 195,191,212,198,104,159,246,78,223,251,183,234,113,176,165,74");
asm(" .byte 98,51,16,11,207,5,204,7,44,1,172,0,44,133,98,254");
asm(" .byte 106,137,221,117,106,236,189,159,70,224,171,17,71,63,128,116");
asm(" .byte 39,99,210,228,103,227,241,157,192,23,126,31,113,56,210,167");
asm(" .byte 0,211,180,87,150,43,99,157,228,199,66,74,152,201,206,50");
asm(" .byte 73,169,140,186,181,187,149,190,8,83,235,203,232,93,214,54");
asm(" .byte 198,30,55,177,212,200,183,65,79,47,108,139,164,78,106,139");
asm(" .byte 152,20,123,56,71,105,143,118,41,93,50,189,191,102,122,239");
asm(" .byte 81,74,35,157,147,170,233,189,229,112,231,146,114,69,18,184");
asm(" .byte 4,188,115,162,160,223,1,186,192,37,224,157,19,4,125,49");
asm(" .byte 232,2,151,128,119,166,8,250,34,208,5,46,1,239,28,47");
asm(" .byte 232,183,131,46,112,9,120,103,178,160,151,130,46,112,9,120");
asm(" .byte 231,56,65,191,13,116,129,75,192,59,147,4,125,33,232,2");
asm(" .byte 151,128,119,38,10,250,2,208,5,46,1,239,76,16,244,249");
asm(" .byte 160,11,92,2,222,105,21,244,18,208,5,46,1,239,180,8");
asm(" .byte 122,49,232,2,151,128,119,154,5,125,30,232,2,151,128,119");
asm(" .byte 154,4,189,8,116,129,75,192,59,37,65,183,129,46,112,9");
asm(" .byte 120,39,19,116,21,116,129,99,47,82,232,217,3,78,98,101");
asm(" .byte 250,251,227,139,34,99,120,112,222,217,73,244,30,169,215,76");
asm(" .byte 249,84,204,99,142,210,160,189,147,61,207,20,52,240,97,175");
asm(" .byte 148,194,47,153,164,224,207,221,18,98,120,216,7,226,151,100");
asm(" .byte 51,246,158,173,113,248,28,251,139,131,251,153,57,108,150,151");
asm(" .byte 134,181,247,216,37,37,74,113,137,41,53,55,234,85,250,34");
asm(" .byte 157,106,95,170,153,21,165,150,157,189,154,90,14,88,4,155");
asm(" .byte 161,231,244,211,23,181,135,205,74,87,72,146,21,156,48,115");
asm(" .byte 245,84,109,124,118,42,189,51,16,47,236,52,73,114,204,100");
asm(" .byte 245,198,226,150,16,233,159,102,146,82,99,166,201,157,177,120");
asm(" .byte 198,137,24,236,50,13,50,35,211,69,92,93,105,222,23,86");
asm(" .byte 212,198,104,106,58,226,44,27,139,194,140,191,201,228,111,67");
asm(" .byte 143,112,212,100,117,208,253,112,169,100,161,247,102,89,105,144");
asm(" .byte 245,69,156,240,59,228,175,63,135,111,158,75,239,78,75,149");
asm(" .byte 177,174,241,24,47,22,50,155,153,151,222,213,142,116,202,125");
asm(" .byte 218,24,154,172,114,140,218,248,0,62,92,254,231,182,72,82");
asm(" .byte 104,117,52,69,169,10,203,41,56,195,141,247,6,153,210,240");
asm(" .byte 44,234,28,86,77,190,103,244,241,42,130,158,185,104,87,193");
asm(" .byte 80,116,153,48,7,105,191,141,199,135,58,17,27,97,12,34");
asm(" .byte 52,6,233,244,172,12,250,155,180,251,156,34,122,39,95,210");
asm(" .byte 222,235,103,42,181,153,14,60,53,110,73,211,246,55,47,244");
asm(" .byte 53,73,222,233,218,216,206,216,15,185,222,34,224,54,234,231");
asm(" .byte 78,139,79,27,35,42,43,203,191,138,250,193,212,242,69,87");
asm(" .byte 37,236,9,169,222,25,24,251,226,62,204,175,42,73,86,111");
asm(" .byte 186,166,27,61,63,162,185,177,201,81,154,95,22,234,211,222");
asm(" .byte 75,191,179,216,23,211,222,49,101,250,251,251,146,178,79,163");
asm(" .byte 107,239,162,24,242,38,91,170,62,182,12,49,170,28,137,74");
asm(" .byte 168,207,66,65,106,15,228,160,249,30,200,129,141,154,76,37");
asm(" .byte 136,129,96,79,73,74,85,180,235,246,242,48,124,219,11,169");
asm(" .byte 69,190,176,140,243,145,2,27,51,73,181,49,213,244,19,109");
asm(" .byte 188,73,47,5,115,145,42,202,246,178,125,17,27,202,142,78");
asm(" .byte 10,105,99,18,15,5,99,193,57,217,225,195,183,103,203,41");
asm(" .byte 176,163,20,54,47,245,14,237,89,99,87,152,124,148,215,130");
asm(" .byte 126,102,244,165,202,244,60,97,30,120,77,190,200,225,52,95");
asm(" .byte 100,159,173,51,108,146,82,48,94,240,103,114,208,246,147,156");
asm(" .byte 78,155,15,113,49,236,61,178,83,57,25,253,76,191,239,127");
asm(" .byte 196,36,85,143,206,189,62,127,106,204,116,34,37,218,153,17");
asm(" .byte 148,130,150,189,145,195,22,226,175,138,198,213,246,147,152,239");
asm(" .byte 19,224,57,65,99,174,141,37,232,37,216,155,167,178,118,122");
asm(" .byte 134,241,58,202,94,131,61,68,189,176,29,91,154,222,31,234");
asm(" .byte 179,147,105,182,20,221,171,246,68,70,250,165,217,201,9,22");
asm(" .byte 245,98,238,97,99,102,216,152,9,241,89,114,10,244,99,108");
asm(" .byte 30,202,164,40,244,70,89,170,25,54,164,151,201,157,145,16");
asm(" .byte 108,233,240,237,41,97,140,69,132,198,130,226,46,211,137,115");
asm(" .byte 224,127,45,253,93,237,183,34,10,250,205,162,163,125,145,212");
asm(" .byte 116,178,47,198,108,233,38,201,150,78,247,20,84,151,158,217");
asm(" .byte 145,141,140,163,103,248,108,158,118,198,128,189,104,54,164,253");
asm(" .byte 254,68,200,217,105,241,142,218,24,225,102,129,127,217,188,148");
asm(" .byte 106,243,82,166,205,11,230,36,226,181,32,150,161,53,90,20");
asm(" .byte 140,229,98,78,148,34,95,132,230,132,252,19,205,197,131,202");
asm(" .byte 223,68,77,167,200,38,171,35,228,171,208,247,220,195,192,15");
asm(" .byte 3,63,10,60,23,254,45,23,190,45,95,2,29,248,97,224");
asm(" .byte 71,129,155,139,250,34,249,39,251,82,175,162,191,52,143,135");
asm(" .byte 112,22,34,25,198,121,164,57,164,249,219,169,236,255,179,115");
asm(" .byte 247,40,61,243,248,63,156,135,126,195,60,88,197,88,62,15");
asm(" .byte 125,104,44,255,94,255,205,78,228,89,122,86,143,188,89,140");
asm(" .byte 237,15,81,151,232,71,65,31,153,51,196,180,89,166,220,115");
asm(" .byte 177,191,161,212,246,90,236,123,72,159,64,185,23,241,241,78");
asm(" .byte 139,246,62,104,233,143,161,111,94,126,53,61,35,126,156,21");
asm(" .byte 147,143,195,186,135,127,20,126,58,205,248,219,32,39,108,113");
asm(" .byte 100,110,105,78,193,135,245,22,54,95,247,91,35,248,238,36");
asm(" .byte 186,111,253,19,206,62,136,39,232,93,36,230,213,206,145,17");
asm(" .byte 19,206,17,79,4,3,220,142,178,160,28,224,201,193,71,49");
asm(" .byte 166,143,134,217,190,123,195,171,190,223,23,193,25,36,137,126");
asm(" .byte 111,195,212,199,121,146,248,125,8,61,179,144,28,171,162,244");
asm(" .byte 219,167,113,116,6,117,88,195,224,155,69,119,69,244,156,198");
asm(" .byte 18,186,158,215,132,250,139,4,47,241,89,108,143,191,115,189");
asm(" .byte 44,191,38,203,68,113,255,137,235,203,70,234,189,135,243,45");
asm(" .byte 157,135,54,32,126,33,160,126,101,34,182,210,127,3,241,104");
asm(" .byte 216,68,191,237,146,233,157,68,186,99,217,63,57,255,79,241");
asm(" .byte 161,62,240,237,101,250,111,250,232,55,90,146,254,219,75,41");
asm(" .byte 53,77,78,103,117,171,42,170,23,207,220,154,159,55,179,35");
asm(" .byte 111,236,159,9,75,107,102,151,175,91,187,124,213,138,197,172");
asm(" .byte 217,237,113,54,180,184,58,148,185,14,101,213,90,199,250,26");
asm(" .byte 101,105,121,249,50,71,141,114,187,50,86,180,252,186,156,71");
asm(" .byte 105,117,122,64,100,158,70,55,251,2,121,169,226,168,90,86");
asm(" .byte 181,110,125,205,170,181,43,148,185,110,101,102,135,146,171,204");
asm(" .byte 237,80,108,69,11,10,84,252,103,83,230,206,109,114,183,181");
asm(" .byte 123,148,153,77,202,220,123,149,138,181,75,107,64,242,180,41");
asm(" .byte 165,51,155,216,204,5,91,75,149,153,5,179,59,146,89,249");
asm(" .byte 157,37,37,37,243,235,64,28,107,99,169,208,17,114,183,54");
asm(" .byte 185,71,37,17,35,201,90,189,14,77,206,109,105,219,52,215");
asm(" .byte 221,238,218,216,188,77,65,111,149,60,86,232,217,226,46,156");
asm(" .byte 217,81,128,42,172,105,139,171,99,147,178,67,217,212,238,114");
asm(" .byte 235,197,200,120,156,205,45,202,92,155,114,7,180,101,203,54");
asm(" .byte 172,170,209,134,137,89,216,178,234,26,71,62,52,40,95,179");
asm(" .byte 244,78,74,239,174,88,183,130,210,53,235,106,214,81,186,116");
asm(" .byte 221,186,42,173,28,253,213,242,171,107,202,41,93,95,189,182");
asm(" .byte 66,227,95,165,167,101,171,215,221,77,105,117,77,149,206,183");
asm(" .byte 170,170,140,210,138,242,106,228,213,133,219,180,127,216,93,101");
asm(" .byte 75,181,226,149,142,234,85,66,156,150,214,173,89,186,130,229");
asm(" .byte 171,249,42,91,189,172,98,113,254,188,124,155,186,176,184,168");
asm(" .byte 104,65,201,194,226,124,27,115,172,89,87,177,108,113,126,209");
asm(" .byte 132,53,171,170,203,145,230,23,169,248,95,85,243,231,225,31");
asm(" .byte 214,184,181,195,211,182,165,180,195,213,209,209,220,214,90,215");
asm(" .byte 220,196,230,58,91,90,70,200,78,183,187,165,89,255,101,49");
asm(" .byte 21,9,170,187,189,109,99,115,139,203,64,25,169,222,228,234");
asm(" .byte 104,100,247,55,55,185,218,74,27,219,154,92,141,117,27,221");
asm(" .byte 29,172,168,68,144,180,127,235,180,2,102,43,186,77,80,183");
asm(" .byte 56,183,213,53,52,123,218,157,30,23,179,149,64,165,249,197");
asm(" .byte 172,201,229,113,53,122,74,245,164,206,179,221,237,98,243,232");
asm(" .byte 55,210,158,246,182,150,82,145,214,181,184,238,119,181,176,77");
asm(" .byte 174,86,87,187,179,165,180,213,121,127,147,211,227,132,10,91");
asm(" .byte 218,190,64,108,115,83,23,58,152,186,109,62,90,80,85,27");
asm(" .byte 91,189,110,157,3,131,179,170,98,217,93,24,55,19,43,175");
asm(" .byte 171,46,103,171,87,173,213,231,145,50,229,235,214,220,93,129");
asm(" .byte 37,32,150,194,216,250,192,180,44,95,1,75,144,88,121,77");
asm(" .byte 213,234,197,168,0,9,58,90,2,108,166,90,180,129,57,202");
asm(" .byte 215,84,204,236,88,140,44,21,143,252,143,74,85,203,150,163");
asm(" .byte 198,86,189,41,219,245,3,179,217,217,218,138,30,217,230,169");
asm(" .byte 108,107,135,171,189,161,109,91,169,72,235,26,183,52,233,147");
asm(" .byte 59,179,67,255,127,116,52,218,182,122,154,218,218,218,71,243");
asm(" .byte 27,91,154,55,109,246,212,61,208,236,217,140,162,186,142,205");
asm(" .byte 46,154,78,81,232,218,218,226,106,175,115,182,110,194,236,97");
asm(" .byte 220,191,48,164,247,127,231,75,201,219,157,15,140,210,156,45");
asm(" .byte 158,102,207,214,38,189,190,81,125,76,255,214,142,6,182,188");
asm(" .byte 166,106,213,154,197,19,52,101,109,215,91,34,149,220,168,167");
asm(" .byte 179,181,121,11,91,14,19,205,159,217,200,58,176,54,243,245");
asm(" .byte 57,177,49,242,66,98,18,180,209,101,13,91,59,182,43,185");
asm(" .byte 98,62,70,44,79,168,192,198,134,112,116,236,216,38,119,71");
asm(" .byte 233,168,41,176,17,91,96,244,91,249,22,182,182,170,114,149");
asm(" .byte 62,43,172,98,105,205,117,232,178,117,203,9,159,217,113,189");
asm(" .byte 249,43,139,177,254,147,103,118,140,153,255,8,101,108,245,220");
asm(" .byte 72,161,5,161,211,24,145,216,61,162,160,227,47,147,153,54");
asm(" .byte 143,201,122,147,186,67,160,86,53,195,84,153,170,255,217,216");
asm(" .byte 252,141,13,106,99,201,109,69,204,117,219,194,198,226,121,183");
asm(" .byte 169,108,75,115,251,214,45,109,77,186,223,114,122,138,58,218");
asm(" .byte 10,48,110,172,108,221,58,221,49,145,148,194,173,110,116,214");
asm(" .byte 85,120,191,171,157,154,43,240,108,243,176,138,229,107,238,38");
asm(" .byte 203,41,108,114,221,95,232,241,108,255,50,199,217,172,56,61");
asm(" .byte 155,85,205,241,86,173,115,124,185,107,133,95,55,186,214,5");
asm(" .byte 163,220,73,207,72,157,125,125,204,79,39,106,29,175,211,240");
asm(" .byte 37,166,49,122,162,129,103,239,179,132,63,169,157,192,63,214");
asm(" .byte 240,163,26,254,67,13,127,218,164,201,121,41,174,253,44,191");
asm(" .byte 30,105,95,95,159,215,131,244,0,142,241,7,69,222,210,79");
asm(" .byte 233,17,111,198,41,61,127,68,75,159,244,38,157,142,239,220");
asm(" .byte 1,62,15,210,167,204,207,120,63,253,181,94,206,206,196,119");
asm(" .byte 186,65,175,71,234,65,122,22,105,223,83,183,122,155,94,70");
asm(" .byte 250,164,197,219,135,116,51,232,189,175,16,255,83,222,179,191");
asm(" .byte 137,239,124,50,241,25,239,142,179,241,157,181,160,47,122,3");
asm(" .byte 117,145,110,187,172,203,237,255,55,93,191,190,143,136,255,105");
asm(" .byte 111,203,176,222,206,190,225,248,206,255,232,91,41,95,253,125");
asm(" .byte 245,247,213,223,87,127,95,253,125,245,247,213,223,87,127,255");
asm(" .byte 63,253,245,61,204,88,239,195,99,223,147,27,129,130,145,111");
asm(" .byte 217,137,212,39,190,29,23,124,72,255,94,157,231,159,196,247");
asm(" .byte 224,28,146,246,125,58,139,248,246,154,5,249,108,195,183,244");
asm(" .byte 54,119,234,223,179,115,251,245,239,208,217,187,244,239,216,61");
asm(" .byte 44,233,223,110,147,197,55,240,152,248,38,93,74,165,164,125");
asm(" .byte 159,110,88,124,139,46,85,124,107,238,200,110,29,15,238,214");
asm(" .byte 191,35,150,36,190,67,71,233,231,241,120,155,29,244,56,82");
asm(" .byte 146,113,21,233,181,93,255,253,177,72,113,140,69,137,71,31");
asm(" .byte 250,127,7,244,23,172,28,211,197,62,107,250,173,138,118,184");
asm(" .byte 80,212,130,34,155,82,164,226,88,186,160,104,30,91,58,27");
asm(" .byte 165,218,55,206,152,68,31,215,178,150,212,44,99,9,150,36");
asm(" .byte 105,156,148,102,201,144,38,75,83,204,153,210,84,41,203,148");
asm(" .byte 47,173,40,47,47,85,102,85,183,109,109,111,116,181,111,87");
asm(" .byte 86,204,153,163,172,110,246,184,72,150,90,160,222,54,183,68");
asm(" .byte 189,85,41,46,40,41,176,97,234,59,54,211,135,205,156,13");
asm(" .byte 172,160,185,213,227,106,119,179,130,214,54,143,171,96,105,217");
asm(" .byte 170,185,30,231,38,86,176,217,217,177,153,21,52,109,111,237");
asm(" .byte 216,190,69,79,61,237,172,96,83,235,214,2,113,196,185,46");
asm(" .byte 83,135,178,118,87,11,241,233,136,187,197,67,146,155,241,175");
asm(" .byte 199,133,179,80,193,70,100,80,212,70,71,66,86,176,180,106");
asm(" .byte 77,1,200,212,188,142,54,55,109,211,249,235,156,237,237,206");
asm(" .byte 237,58,255,8,126,111,99,187,166,130,115,75,115,35,154,109");
asm(" .byte 131,56,93,76,3,78,120,154,0,250,72,90,115,195,86,143");
asm(" .byte 11,249,198,182,45,91,92,173,158,255,134,77,140,23,246,79");
asm(" .byte 54,169,125,175,82,210,237,238,198,111,48,166,139,239,58,18");
asm(" .byte 159,246,93,73,73,255,222,153,241,123,140,76,124,255,209,42");
asm(" .byte 248,104,109,108,6,223,5,67,249,200,186,249,186,104,219,36");
asm(" .byte 214,140,29,136,106,210,235,74,108,236,123,147,133,98,221,152");
asm(" .byte 196,26,115,91,244,181,117,163,126,11,152,190,62,136,143,214");
asm(" .byte 198,53,8,90,110,104,215,36,160,66,172,39,173,93,172,41");
asm(" .byte 59,22,109,189,208,217,108,224,175,22,223,133,52,137,53,25");
asm(" .byte 76,208,215,162,229,134,239,79,214,26,248,104,13,31,73,208");
asm(" .byte 215,182,69,244,111,132,207,37,116,77,16,62,38,152,168,251");
asm(" .byte 155,27,199,175,222,192,167,173,23,240,237,145,174,231,35,184");
asm(" .byte 207,192,167,125,203,20,74,216,231,127,81,222,183,13,124,228");
asm(" .byte 211,60,75,198,190,155,105,228,123,192,96,7,59,192,183,3");
asm(" .byte 124,181,9,95,228,219,105,224,187,112,10,176,140,177,154,47");
asm(" .byte 145,247,136,206,231,238,98,226,59,170,203,245,111,101,166,220");
asm(" .byte 192,247,3,241,13,78,51,27,249,222,233,88,153,145,239,176");
asm(" .byte 248,126,168,89,248,96,203,159,225,251,159,66,63,226,163,111");
asm(" .byte 132,37,253,25,190,103,197,152,152,217,200,119,84,199,190,225");
asm(" .byte 106,156,223,95,26,228,145,239,78,169,100,108,31,251,226,124");
asm(" .byte 132,12,124,7,193,119,176,82,95,11,55,182,123,90,200,37");
asm(" .byte 62,250,222,217,139,224,27,190,245,139,124,191,209,121,220,35");
asm(" .byte 116,226,155,243,37,235,242,247,2,87,69,126,16,124,179,111");
asm(" .byte 224,147,12,186,141,126,203,178,138,177,129,47,249,214,234,255");
asm(" .byte 6,251,131,71,27,48,87,0,0,0");
asm("eod_flash_pgb:");
asm(" pop r25"); /* pull return address from stack */
asm(" pop r24");
asm(" clc"); /* multiply by 2 to get data address */
asm(" rol r24");
asm(" rol r25");
asm(" ret");
}

#endif

