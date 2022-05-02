/*
 *******************************************************************************************
 dcc_gen V3.0
 - new uC: ATMEGA32
 - Start-Target Feature
 - improved Console Functions
 dcc_gen V3.1
 - (2022-4-15) SETUP: fix programmer config (platformio.ini), runs now with USBProg3.0 again
               ATMEGA8: reduce RAM: less Routes, no selftest, Known Issue: still problems with UART
               DCC: changed sync length to 16 iterations, use utils/delay functions,
                    re-adjust DCC bit length, fix switch command (add active flag)
               GENERAL: cleanup unused functions, remove white spaces, formating, renaming
 - (2022-4-28) SETUP: fix programmer config for USBasp (FW 2011)
               ATMEGA8: refactor state machine, fixed memory issue (also fixes the UART)
 *******************************************************************************************
*/

#define	__AVR_ATmega8__	1
//#define	__AVR_ATmega32__	1

#define VERSION 3
#define SUBVERSION 1
#ifdef __AVR_ATmega32__
  #define MEGA_CPU 32
#endif
#ifdef __AVR_ATmega8__
  #define MEGA_CPU 8
#endif


#ifdef __AVR_ATmega32__
 #define PIN_OUT1 PD2
 #define PIN_OUT2 PD3
#endif

#include "avr/io.h"
#include <avr/interrupt.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "avr/eeprom.h"
#include <avr/pgmspace.h>
#include <util/delay.h>

#define false 0
#define true 1
  
//******************************************************************************************
// Berechnung des Wertes fuer das Baudratenregister
// aus Taktrate und gewuenschter Baudrate
//******************************************************************************************
#ifndef F_CPU
/* In neueren Version der WinAVR/Mfile Makefile-Vorlage kann
   F_CPU im Makefile definiert werden, eine nochmalige Definition
   hier wuerde zu einer Compilerwarnung fuehren. Daher "Schutz" durch
   #ifndef/#endif

   Dieser "Schutz" kann zu Debugsessions fuehren, wenn AVRStudio
   verwendet wird und dort eine andere, nicht zur Hardware passende
   Taktrate eingestellt ist: Dann wird die folgende Definition
   nicht verwendet, sondern stattdessen der Defaultwert (8 MHz?)
   von AVRStudio - daher Ausgabe einer Warnung falls F_CPU
   noch nicht definiert: */
#warning "F_CPU war noch nicht definiert, wird nun nachgeholt mit 8000000"
#define F_CPU 8000000UL  // Systemtakt in Hz - Definition als unsigned long beachten
                         // Ohne ergeben sich unten Fehler in der Berechnung
#endif
#define BAUD 9600UL      // Baudrate
// Berechnungen
#define UBRR_VAL ((F_CPU+BAUD*8)/(BAUD*16)-1)   // clever runden
#define BAUD_REAL (F_CPU/(16*(UBRR_VAL+1)))     // Reale Baudrate
#define BAUD_ERROR ((BAUD_REAL*1000)/BAUD)      // Fehler in Promille, 1000 = kein Fehler.
#if ((BAUD_ERROR<990) || (BAUD_ERROR>1010))
  #error Systematischer Fehler der Baudrate groesser 1% und damit zu hoch!
#endif
// Quelle: www.mikrocontroller.net AVR GCC Tutorial
//******************************************************************************************

//******************************************************************************************
// function prototypes
//******************************************************************************************
void InitPorts(void);
void InitTimer(void);
void HeartbeatSet(unsigned char state);
void Selftest(void);
int main(void);

//DCC functions
void DCCSet(void);
void DCCDelete(void);
void DCCBitOne(void);
void DCCBitZero(void);
void DCCBitZeroLong(void);
void DCCSync(void);
void DCCStartbit(void);
void DCCByte(char cByte);
void DCCStopbit(void);
char DCCSwitch
       ( unsigned char p_cDecAdresse
       , unsigned char p_cDecOutput
       , unsigned char p_cState
       );
void DCCToggle(void);

//delay functions
void delay_ms(unsigned int uiDuration);
void delay_us(unsigned int uiDuration);

//UART functions
void           UARTInit(void);
char           UARTReceive(void);
char           UARTGetC(void);
void           UARTTransmit(char data);
void           UARTSendString(const char * pStr);
void           UARTHello(void);
void           UARTCheckCommand(void);
void           UARTSendHelp(void); 
void           UARTComunicate(void);
void           UARTSendMemoryElements(void);
void           UARTSendMemoryRoutes(void);
char           UARTSendMemoryTrace(unsigned char route);
void           UARTSendMemoryOverview(void);

//define stream for printf
static FILE uart_stream = FDEV_SETUP_STREAM((void *)UARTTransmit,NULL, _FDEV_SETUP_RW);

//protocol functions
char GetTokenToChar
       ( char * pcBuffer
       , char cIndex
       );
unsigned char GetTokenToUChar
       ( char * pcBuffer
       , char cIndex
       );

void CopyNumericToken
       ( char * dest
       , char * src
       , char length
       );

//keyboard functions
void KeybRead(void);
char KeybIsKey(unsigned char Index);

//memory functions
char MemoryReadFromEEprom(void);
char MemorySaveToEEprom(void);
void MemoryInit(void);
char MemorySetElement
       ( unsigned char index
       , unsigned char assigned_id
       , unsigned char dec_adr
       , unsigned char dec_state
       );
char MemorySetElementWait
       ( unsigned char index
       , unsigned char assigned_id
       , unsigned char dec_adr
       , unsigned char dec_state
       , unsigned char wait
       , unsigned char forward
       );
char MemorySetRoute
       ( unsigned char index
       , unsigned char fromKey
       , unsigned char toKey
       , unsigned char bidirectional
       , unsigned char targetDefined
       );
void MemoryTestInit(char token);
void MemoryClear(void);
void MemorySwitchField
       ( unsigned char line
       , unsigned char field
       );
char MemoryCareTargetlessRoutes
       ( unsigned char FirstPressedKey
       );
void MemoryCareCompleteRoutes
       ( unsigned char FirstPressedKey
       , unsigned char SecondPressedKey
       );

//task functions
void LEDTask(void);
void MemoryTask(void);
void KeybTask(void);

//******************************************************************************************
// data and structures memory
//******************************************************************************************
#define MAX_DECODER_ADRESS                        128
#define MAX_BIDIRECTIONAL                         1
#define MAX_DECODER_STATE                         1
#define MAX_KEYS_COUNT_LIMIT                      10
#define MAX_DECODER_WAIT                          16
#define MAX_DECODER_FORWARD_ROUTE                 1


#ifdef __AVR_ATmega8__
  #define MAX_MEMORY_ELEMENTS                     0x8F
  #define MAX_MEMORY_ROUTES                       60
  #define MAX_KEY_GROUPS                          1
  #define MAX_REAL_KEYS                           10
#endif

#ifdef __AVR_ATmega32__
  #define MAX_MEMORY_ELEMENTS                     240
  #define MAX_MEMORY_ROUTES                       120
  #define MAX_KEY_GROUPS                          4
  #define MAX_REAL_KEYS                           20
#endif

#define MAX_VIRTUAL_KEYS                          (MAX_REAL_KEYS * MAX_KEY_GROUPS)

struct MemoryElement 
{ unsigned char assigned_id;//assigned route
  unsigned dec_adr:7;       //decoder adress
  unsigned dec_state:1;     //decoder state
  unsigned wait:4;          //wait time befor send command 0-15s
  unsigned forwardRoute:1;  //if set, dec_adr points to route
  unsigned spare:3;
} MemoryElement;

struct MemoryRoute
{ unsigned fromKey:7;       //key starting route
  unsigned bidirectional:1; //route works in both direktions
  unsigned toKey:7;         //key end route
  unsigned targetDefined:1; //tbc
} MemoryRoute;

struct EEProm
{
  struct MemoryElement arrMemoryElements[MAX_MEMORY_ELEMENTS];
  struct MemoryRoute arrMemoryRoutes[MAX_MEMORY_ROUTES];
} EEProm;

//settings
unsigned char InitRoute = 1;
unsigned char InitRouteWait = 5; //[sec]
unsigned char InitWaitBetweenSwitch = 5; //[10ms]

//EEprom
struct EEProm EEPPromShadow EEMEM;
unsigned char EEPInitRoute EEMEM;
unsigned char EEPInitRouteWait EEMEM; //[sec]
unsigned char EEPInitWaitBetweenSwitch EEMEM; //[10ms]
unsigned char EEPGuilty EEMEM;

//******************************************************************************************
// global variables
//****************************************************************************************** 
char  UARTReceiveBuffer[100];
char  UARTLastCommand[100];
char* pLastReceiveByte = 0;
char  bReceivedCommandComplete;
char  cStatusMode = 'I';        //I = idle
char  cStatusError = 0;         //0 = no error

char cTimerBase = 19; //1160us Timer
long lLedTaskTimer = 0;
long lKeybTaskTimer = 0;
long lMemoryTaskTimer = 0;

unsigned char cKey[MAX_VIRTUAL_KEYS];

char RuningMemory = false;
char LastPressedKey = 0;
char FirstPressedKey = 0;
char SecondPressedKey = 0;
char keyActive = false;
uint8_t indexFIFOBuffer = 0xFF;

char HeartbeatActive = false;
char HeartbeatNackCount = 0;

#define LED_STATE_IDLE    0
#define LED_STATE_WAITING 1
#define LED_STATE_COMMIT  2
#define LED_STATE_NACK    3

unsigned char ledActiveState = LED_STATE_IDLE;

#define MEMORY_STATE_IDLE               0
#define MEMORY_STATE_FIRST_KEY_AKTIVE   1
#define MEMORY_STATE_WAITING_SECOND_KEY 2
#define MEMORY_STATE_FINISH_ROUTE       3
#define MEMORY_STATE_WAIT_AFTER         4
#define MEMORY_STATE_SWITCH_IT          5


unsigned char memoryActiveState = MEMORY_STATE_IDLE;
unsigned char memoryNextActiveState = MEMORY_STATE_IDLE;
unsigned long memoryTimeout = 0;
unsigned char memoryActualIndex = 0;

#define KEYB_STATE_IDLE               0
#define KEYB_STATE_SET_NEXT_GROUP     1
#define KEYB_STATE_READ_GROUP         2

unsigned char keybActiveState = KEYB_STATE_IDLE;
unsigned long keybTimeout = 0;
unsigned char keybActiveGroup = 0;
unsigned char keybActiveKey = 0;

//******************************************************************************************
// Init Ports
//******************************************************************************************
void InitPorts(void)
{
#ifdef __AVR_ATmega8__
  //init ports
  PORTB = 0x0; // external pullup
  PORTD = 0x0; // external pullup
  PORTC = 0x0; // external pullup
  DDRB = 0x0;
  DDRD = 0x0;
  DDRC = 0x0;

  PORTC = 1<<PC5;	/* turn the LED off */
  DDRC = 1<<PC5;	/* PC5 as output - the LED is there */

  //care amplifier outputs
  PORTB |= 1<<PB0;	/* Output A LOW  */
  DDRB |= 1<<PB0;	/* PB0 as output */
  PORTB |= 1<<PB1;	/* Output B LOW  */
  DDRB |= 1<<PB1;	/* PB1 as output */
#endif

#ifdef __AVR_ATmega32__
  //init ports
  PORTA = 0x0; // external pullup
  PORTB = 0x0; // external pullup
  PORTC = 0x0; // external pullup
  PORTD = 0x0; // external pullup
  DDRA = 0x0;
  DDRB = 0x0;
  DDRC = 0x0;
  DDRD = 0x0;

  //care LED port
  PORTB |= 1<<PB4;
  DDRB |= 1<<PB4;

  //care amplifier outputs
  PORTD |= 1<<PIN_OUT1;	/* Output A LOW  */
  DDRD |= 1<<PIN_OUT1;	/* driver enable 0 */
  PORTD |= 1<<PIN_OUT2;	/* Output B LOW  */
  DDRD |= 1<<PIN_OUT2;	/* driver enable 1 */
#endif

  //clear keys array
  memset(&cKey, 0, sizeof(cKey));

  //activate global interrupts
  sei();
}

//******************************************************************************************
// Init Timer
// 8bit Timer is main timer and support 1ms base timer
//******************************************************************************************
void InitTimer(void) 
{
  // Timer 0 konfigurieren
  TCCR0 = (1<<CS01); // Prescaler 8
  // Overflow Interrupt erlauben
  TIMSK |= (1<<TOIE0);
}

//******************************************************************************************
// Overflow Interrupt Handler
// called if TCNT0 changes 255 to 0 (256 Steps),
// freq_nopreload = 8000000 / 8 / 256 = 3906.25 Hz, ^= 256us
// freq_preload60 = 5000.18 Hz ^= 200us
// freq_preload202 ^= 58 us
// freq_preload144 ^= 116 us
//******************************************************************************************
ISR (TIMER0_OVF_vect)
{ //preload
  TCNT0 = 60;
  //care base timer
  if (cTimerBase>0) {
    cTimerBase--;
  } else {
    cTimerBase = 4;
    //care other timers, decrement if set
    if (lLedTaskTimer > 0) lLedTaskTimer--;
    if (lKeybTaskTimer > 0) lKeybTaskTimer--;
    if (lMemoryTaskTimer > 0) lMemoryTaskTimer--;
  }
}

//******************************************************************************************
// Simple LED Support
//	state = 0 -> Led Off
//	state = 1 -> Led On
// 	state !=[0,1] -> Led Toggle
//******************************************************************************************
void HeartbeatSet(unsigned char state)
{
#ifdef __AVR_ATmega8__
  switch (state)
  {
  case 0:
    PORTC &= ~(1<<5);
    break;
  case 1:
    PORTC |= 1<<5;
    break;
  default:
    if (PORTC & 1<<5)
      PORTC &= ~(1<<5);
    else
      PORTC |= 1<<5;
  }
#endif

#ifdef __AVR_ATmega32__
  switch (state)
  {
  case 0:
    PORTB &= ~(1<<PB4);
    break;
  case 1:
    PORTB |= 1<<PB4;
    break;
  default:
    if (PORTB & 1<<PB4)
      PORTB &= ~(1<<PB4);
    else
      PORTB |= 1<<PB4;
  }
#endif
}

//******************************************************************************************
// DCC Burst Functions
//******************************************************************************************
void DCCSet(void)
{ //set amp port
#ifdef __AVR_ATmega8__
  PORTB &= ~(1<<0);
  PORTB |=  (1<<1);
#endif
#ifdef __AVR_ATmega32__
  PORTD &= ~(1<<PIN_OUT1);
  PORTD |=  1<<PIN_OUT2;
#endif
}

void DCCDelete(void)
{ //delete amp port
#ifdef __AVR_ATmega8__
  PORTB |=  (1<<0);
  PORTB &= ~(1<<1);
#endif
#ifdef __AVR_ATmega32__
  PORTD &= ~(1<<PIN_OUT2);
  PORTD |=  1<<PIN_OUT1;
#endif
}

void DCCBitOne(void)
{
  DCCSet();
  delay_us(58);
  DCCDelete();
  delay_us(55);
}

void DCCBitZero(void)
{
  DCCSet();
  delay_us(114);
  DCCDelete();
  delay_us(104);
}

void DCCBitZeroLong(void)
{
  DCCSet();
  delay_ms(2);
  DCCDelete();
  delay_ms(2);
}

void DCCSync(void)
{ char i;
  //more than 10 ones
  for (i=0;i<16;i++)
  {
    DCCBitOne();
  }
}

void DCCStartbit(void)
{ //start bit
  DCCBitZero();
}

void DCCByte(char cByte)
{ char i;
  for (i=7;i>=0;i--)
  { //shift and compare
    if (cByte & 1<<i) {
      DCCBitOne();
    } else {
      DCCBitZero();
    }
  }
}

void DCCStopbit(void)
{ //start bit
  DCCBitOne();
}

void DCCToggle(void)
{ static char dcctoggleflag = 0;
  //check flag
  if (dcctoggleflag) {
    //delete ports
    DCCDelete();
  } else {
    //set ports
    DCCSet();
  }
  //invert
  dcctoggleflag = !dcctoggleflag;
}

char DCCSwitch(unsigned char p_cDecAdresse, unsigned char p_cDecOutput, unsigned char p_cState)
{ 
  unsigned char adressbyte;
  unsigned char databyte;
  unsigned char checksumbyte;

  //Nr. 0 modules not used
  p_cDecAdresse++;

  //check ranges
  if ((p_cDecOutput > 3) || (p_cState > 1))
  { //leave
    return false;
  }

  // DCC basic accessory decoder format 
  // 10AAAAAA 0 1AAADAAC 0
  // the middle adress block needs to be inverted (results in the xor below)
  // D=1 means coil on, D=0 coil off
  // the last two adress bits are usually known as decoder output 1-4
  // C refers the coil to be activated (usually straight/round or green/red)
  adressbyte = 0x80 + (p_cDecAdresse & 0x3F);
  databyte  = 0x80 + ((p_cDecAdresse / 0x40) ^ 0x07) * 0x10;
  databyte  += ((p_cDecOutput & 0x03) << 1);
  databyte  += (p_cState & 0x01);
  databyte  += 0x08;
  checksumbyte  = adressbyte ^ databyte;

  // Overflow Interrupt erlauben
  TIMSK &= ~(1<<TOIE0);

  //send a signal
  DCCBitZeroLong();    //oszi
  DCCSync();           //synchronisation
  DCCStartbit();
  DCCByte(adressbyte); //adress
  DCCStartbit();
  DCCByte(databyte);  //data
  DCCStartbit();
  DCCByte(checksumbyte); //prüfsumme
  DCCStopbit();
  DCCBitZeroLong();    //oszi

  // Overflow Interrupt erlauben
  TIMSK |= (1<<TOIE0);
  
  //tell user
  printf_P
    ( PSTR("DW;%u;%u;%u(MA=%u)\n\r")
    , p_cDecAdresse
    , p_cDecOutput
    , p_cState
    , ((p_cDecAdresse-1) * 4) + p_cDecOutput +1
    );

  return true;
}

//******************************************************************************************
// FIFO functions (www.mikrocontroller.net/articles/FIFO)
//******************************************************************************************

#define BUFFER_SIZE 64 // muss 2^n betragen (8, 16, 32, 64 ...)
#define BUFFER_MASK (BUFFER_SIZE-1) // Klammern auf keinen Fall vergessen

struct FIFOBuffer 
{
  uint8_t data[BUFFER_SIZE];
  uint8_t read; // zeigt auf das Feld mit dem ältesten Inhalt
  uint8_t write; // zeigt immer auf leeres Feld
} FIFObuffer = {{}, 0, 0};

uint8_t FIFOBufferIn(uint8_t byte)
{
  uint8_t next = ((FIFObuffer.write + 1) & BUFFER_MASK);
  if (FIFObuffer.read == next)
    return false;
  //FIFObuffer.data[FIFObuffer.write] = byte; //schneller
  FIFObuffer.data[FIFObuffer.write & BUFFER_MASK] = byte; // absolut Sicher
  FIFObuffer.write = next;
  return true;
}

uint8_t FIFOBufferOut(uint8_t *pByte)
{
  if (FIFObuffer.read == FIFObuffer.write)
    return false;
  *pByte = FIFObuffer.data[FIFObuffer.read];
  FIFObuffer.read = (FIFObuffer.read+1) & BUFFER_MASK;
  return true;
}

//******************************************************************************************
// Selftest Support
//******************************************************************************************
void Selftest(void)
{ 
  #ifdef __AVR_ATmega32__
    //just tell i am alive by blinking three times
    HeartbeatSet(0);
    delay_ms(200);
    HeartbeatSet(1);
    delay_ms(200);
    HeartbeatSet(0);
    delay_ms(200);
    HeartbeatSet(1);
    delay_ms(200);
    HeartbeatSet(0);
    delay_ms(200);
    HeartbeatSet(1);
    delay_ms(200);
  #endif
}

//very simple delay
//based on: for (i=10000;i;i--); //ca. 181ms
void delay_ms(unsigned int uiDuration) 
{
  _delay_ms(uiDuration);
}
void delay_us(unsigned int uiDuration) 
{
  _delay_us(uiDuration);
}

//******************************************************************************************
// UART functions
//******************************************************************************************
// Init UART to 8 bit, no control (!), 1 stop
void UARTInit(void)
{ //set calculated baudrate (defines)
  UBRRH = UBRR_VAL >> 8;
  UBRRL = UBRR_VAL & 0xFF;
  //Enable Receiver and Transmitter
  UCSRB = (1<<RXEN)|(1<<TXEN);
  //Set frame format: 8data, 1stop bit
  UCSRC = (1<<URSEL)|(1<<UCSZ1)|(1<<UCSZ0);
  // target UARTs to Stdout
  stdout = &uart_stream;
  //clear flag
  bReceivedCommandComplete = false;
  //reset pointer
  pLastReceiveByte = UARTReceiveBuffer;
}
 
// output function for printf
int uart_putchar( char c, FILE *stream )
{
  // wait for empty buffer
  while ( !( UCSRA & (1<<UDRE)) );
  // send char
  UDR = c;
  return 0;
}

// Get a char from UART or return zero
char UARTReceive(void)
{
  if (UCSRA & (1<<RXC))
    return UDR;
  else    /* no data pending */
    return 0;
}

/* Zeichen empfangen */
char UARTGetC(void)
{
    while (!(UCSRA & (1<<RXC)))   // warten bis Zeichen verfuegbar
        ;
    return UDR;                   // Zeichen aus UDR an Aufrufer zurueckgeben
}

void UARTTransmit(char data)
{
  while (!( UCSRA & (1<<UDRE)) );
  /* Put data into buffer, sends the data */
  UDR = data;
}

void UARTSendString(const char * pStr)
{ //send till zero
  while (*pStr) {
    UARTTransmit(*pStr);
    pStr++;
    delay_ms(1);
  }

}

void UARTHello(void)
{
  //hello world
  printf_P(PSTR("\x1B[2J\x1B[Hdcc_gen v%u.%u ATMega%u\n\r" ), VERSION, SUBVERSION, MEGA_CPU);
  #ifdef __AVR_ATmega32__
    printf_P(PSTR("type '?' for help...\n\r"));
  #endif
}

void CopyNumericToken
  ( char * dest
  , char * src
  , char length
  )
{ //temps
  unsigned char cCount = 0;
  //check param
  if (dest && src) {
    //copy nums
    while ((*src >= 48) && (*src <= 57) && (cCount<length)) 
    { //copy
      *dest = *src;
      //next
      dest++;
      src++;
      cCount++;
      //close string
      *dest = 0;
    }
  }
}

char GetTokenToChar
  ( char * pcBuffer
  , char cIndex
  )
{ //temps
  char cRet = 0;
  char bFound = false;
  char cCount = 0;
  char cSeperator = ';';
  char * pcTemp = pcBuffer;
  char strNumBuffer[10];
  unsigned char countNumBuffer = 0;
  //check parameter
  if (!pcBuffer) return cRet;
  //loop buffer
  while (!bFound) {
    //check for separator
    if ((*pcTemp == cSeperator)||(*pcTemp == '\r')||(*pcTemp == '\n')||(*pcTemp == 0)) {
      //got one
      cCount++;
      //check index
      if (cCount == cIndex) {
        //calc length of token
        countNumBuffer = pcTemp-pcBuffer;
        //check length
        if (countNumBuffer < 10) {
          //copy string
          CopyNumericToken(strNumBuffer, pcBuffer, countNumBuffer);
          //convert to number
          cRet = atoi(strNumBuffer);
        }
        //leave
        bFound = true;
      } else { 
        //set next starting point
        pcBuffer = pcTemp+1;
      }
    }
    if ((*pcTemp == '\n')||(*pcTemp == '\r')||(*pcTemp == 0))
    { //leave anyway
      bFound = true;
    } else {
      //next sign
      pcTemp++;
    }
  }
  //fin
  return cRet;
}

unsigned char GetTokenToUChar
  ( char * pcBuffer
  , char cIndex
  )
{ //temps
  unsigned char cRet = 0;
  char bFound = false;
  char cCount = 0;
  char cSeperator = ';';
  char * pcTemp = pcBuffer;
  char strNumBuffer[10];
  unsigned char countNumBuffer = 0;
  //check parameter
  if (!pcBuffer) return cRet;
  //loop buffer
  while (!bFound) {
    //check for separator
    if ((*pcTemp == cSeperator)||(*pcTemp == '\r')||(*pcTemp == '\n')||(*pcTemp == 0)) {
      //got one
      cCount++;
      //check index
      if (cCount == cIndex) {
        //calc length of token
        countNumBuffer = pcTemp-pcBuffer;
        //check length
        if (countNumBuffer < 10) {
          //copy string
          CopyNumericToken(strNumBuffer, pcBuffer, countNumBuffer);
          //convert to number
          cRet = (unsigned char)atoi(strNumBuffer);
        }
        //leave
        bFound = true;
      } else { 
        //set next starting point
        pcBuffer = pcTemp+1;
      }
    }
    if ((*pcTemp == '\n')||(*pcTemp == '\r')||(*pcTemp == 0))

    { //leave anyway
      bFound = true;
    } else {
      //next sign
      pcTemp++;
    }
  }
  //fin
  return cRet;
}

void UARTSendMemoryElements(void) 
{ //temps
  unsigned char i = 0;
  //send Elements
  for (i = 0; i < MAX_MEMORY_ELEMENTS; i++) 
  {
    if (EEProm.arrMemoryElements[i].assigned_id != 0)
    {
      printf_P
        ( PSTR("El#%u;Rt=%u;MA=%u;St=%u\n\r")
        , i+1
        , EEProm.arrMemoryElements[i].assigned_id
        , EEProm.arrMemoryElements[i].dec_adr
        , EEProm.arrMemoryElements[i].dec_state
        );
    }
  }
}

void UARTSendMemoryRoutes(void) 
{ //temps
  unsigned char i = 0;
  //send Elements
  for (i = 0; i < MAX_MEMORY_ROUTES; i++) 
  {
    if (  ((unsigned char)EEProm.arrMemoryRoutes[i].fromKey != 0) 
       || ((unsigned char)EEProm.arrMemoryRoutes[i].toKey != 0)
       )
    {
      printf_P
        ( PSTR("Rt#%u;fK=%u;tK=%u;bi=%u;tD=%u\n\r")
        , i+1
        , (unsigned char)EEProm.arrMemoryRoutes[i].fromKey
        , (unsigned char)EEProm.arrMemoryRoutes[i].toKey
        , EEProm.arrMemoryRoutes[i].bidirectional
        , EEProm.arrMemoryRoutes[i].targetDefined
        );
    }
  }
}

char UARTSendMemoryTrace(unsigned char route)
{ //temps
  unsigned char i, j;

  //care parameter
  if ((route > MAX_MEMORY_ROUTES) || (route < 0)) 
  {
    return false;
  }
  //loop all Routes
  for (i = route; i < MAX_MEMORY_ROUTES; i++) 
  { //pick the ones with keys defined
    unsigned char fromKey = EEProm.arrMemoryRoutes[i-1].fromKey;
    unsigned char toKey   = EEProm.arrMemoryRoutes[i-1].toKey;
    if (  (fromKey != 0) || (toKey != 0)
       )
    { //tell user
      printf_P
        ( PSTR("Rt#%u;fK=%u;tK=%u;bi=%u;tD=%u\n\r")
        , i
        , EEProm.arrMemoryRoutes[i-1].fromKey
        , EEProm.arrMemoryRoutes[i-1].toKey
        , EEProm.arrMemoryRoutes[i-1].bidirectional
        , EEProm.arrMemoryRoutes[i-1].targetDefined
        );
      //find according elements
      for (j = 0; j < MAX_MEMORY_ELEMENTS; j++) {
        if (EEProm.arrMemoryElements[j].assigned_id == (i)) 
        { //care forwarded routes
          if (EEProm.arrMemoryElements[j].forwardRoute != 0) {
            UARTSendMemoryTrace(EEProm.arrMemoryElements[j].dec_adr);
          } 
          else
          {
            //tell user
            printf_P
              ( PSTR("|-#%u:MA=%u;St=%u\n\r")
              , j+1
              , EEProm.arrMemoryElements[j].dec_adr
              , EEProm.arrMemoryElements[j].dec_state
              );
          }
        }
      }
      if (route > 0) return true;
    }
  }
  return true;
}

void UARTSendMemoryOverview()
{
  unsigned char count1 = 0;
  unsigned char count2 = 0;
  unsigned char i;
  //summarize
  for (i = 1; i < MAX_MEMORY_ROUTES; i++) 
  { //pick the ones with keys defined
    unsigned char fromKey = EEProm.arrMemoryRoutes[i-1].fromKey;
    unsigned char toKey   = EEProm.arrMemoryRoutes[i-1].toKey;
    if (  (fromKey != 0) || (toKey  != 0)
       )
    {
      count1++; 
    }
  }
  for (i = 0; i < MAX_MEMORY_ELEMENTS; i++) 
  {
    if (EEProm.arrMemoryElements[i].assigned_id != 0) 
    {
      count2++; 
    } 
  }
  #ifdef __AVR_ATmega32__
    printf_P
      ( PSTR("Elements used: %u (%u proz.)\n\r")
      , count2
      , (unsigned char)(((float)count2*100.0)/(float)MAX_MEMORY_ELEMENTS)
      );
    printf_P
      ( PSTR("Routes used: %u (%u proz.)\n\r")
      , count1
      , (unsigned char)(((float)count1*100.0)/(float)MAX_MEMORY_ROUTES)
      );
  #endif
  #ifdef __AVR_ATmega8__
    printf_P
      ( PSTR("El:%u(%u%%) Rt:%u(%u%%)\n\r")
      , count2
      , (unsigned char)(((float)count2*100.0)/(float)MAX_MEMORY_ELEMENTS)
      , count1
      , (unsigned char)(((float)count1*100.0)/(float)MAX_MEMORY_ROUTES)
      );
  #endif
}


void UARTCheckCommand(void) 
{ //temps
  char accepted = false;
  //check origin
  if (UARTReceiveBuffer[0]=='?') 
  {  
    UARTSendHelp();
  }
  if (UARTReceiveBuffer[0]=='M') {
    //prepare answer header
    switch (UARTReceiveBuffer[1])
    {
    case 'A':
      //send switch countwise
      accepted = DCCSwitch
        ( (GetTokenToUChar(UARTReceiveBuffer, 2)-1)/4
        , (GetTokenToUChar(UARTReceiveBuffer, 2)-1)%4
        , GetTokenToUChar(UARTReceiveBuffer, 3)
        );
      break;
    case 'I':
      //answer
      UARTHello();
      accepted = true;
      break;
    case 'S':
      //answer
      printf_P
        ( PSTR("DS;%u;%u;%u\n\r")
        , cStatusMode           
        , ledActiveState
        , memoryActiveState
        );
      accepted = true;
      break;
    case 'W':
      //send switch with channel separation
      accepted = DCCSwitch
        ( GetTokenToUChar(UARTReceiveBuffer, 2)
        , GetTokenToUChar(UARTReceiveBuffer, 3)
        , GetTokenToUChar(UARTReceiveBuffer, 4)
        );
      break;
    case 'F':
      //write route
      accepted = MemorySetRoute
        ( GetTokenToUChar(UARTReceiveBuffer, 2)
        , GetTokenToUChar(UARTReceiveBuffer, 3)
        , GetTokenToUChar(UARTReceiveBuffer, 4)
        , GetTokenToUChar(UARTReceiveBuffer, 5)
        , GetTokenToUChar(UARTReceiveBuffer, 6)
        );
      break;
    case 'R':
      //answer
      if (UARTReceiveBuffer[3]=='E') 
      {
        UARTSendMemoryElements();
      } 
      else if (UARTReceiveBuffer[3]=='R')
      {
        UARTSendMemoryRoutes();
      }
      else
      {
        UARTSendMemoryElements();
        UARTSendMemoryRoutes();
      }
      accepted = true;
      break;
    case 'T':
      //answer
      accepted = UARTSendMemoryTrace(GetTokenToUChar(UARTReceiveBuffer, 2));
      break;
    case 'O':
      //answer
      UARTSendMemoryOverview();
      accepted = true;
      break;
    case 'P':
      //write element
      accepted = MemorySetElement
        ( GetTokenToUChar(UARTReceiveBuffer, 2)
        , GetTokenToUChar(UARTReceiveBuffer, 3)
        , GetTokenToUChar(UARTReceiveBuffer, 4)
        , GetTokenToUChar(UARTReceiveBuffer, 5)
        );
      break;
    case 'C':
      {
        char token = GetTokenToUChar(UARTReceiveBuffer, 2);
        //sure?
        if ( (token == 42) || (token == 21) || (token == 84)) 
        {
          //clear memory
          MemoryClear();
          //init normal
          MemoryTestInit(token);
          accepted = true;
        } else {
          #ifdef __AVR_ATmega32__
            printf_P(PSTR("not valid, should be 42\n\r"));
          #endif
          #ifdef __AVR_ATmega8__
            printf_P(PSTR("!42\n\r"));
          #endif
        }
      }
      break;
    case 'E':
      //sure?
      if (GetTokenToUChar(UARTReceiveBuffer, 2) == 42) 
      {
        //write to memory
        MemorySaveToEEprom();
        accepted = true;
      } else {
        //tell user
        #ifdef __AVR_ATmega32__
          printf_P(PSTR("not valid, should be 42\n\r"));
        #endif
        #ifdef __AVR_ATmega8__
          //printf_P(PSTR("!42\n\r"));
        #endif
      }
      break;
    default:
      break;
    }
  }
  //command accepted?
  if (!accepted)
  {
    //UARTSendString(PSTR("DN\n\r"));
    //user entertainment
    HeartbeatNackCount = 3;
    ledActiveState = LED_STATE_NACK;
  }
  //command finished
  bReceivedCommandComplete = false;
  return;
}

void UARTSendHelp(void) 
{ //temps
#ifdef __AVR_ATmega32__
  printf_P(PSTR("UART Protokoll\n\r"                                                                      ));
  printf_P(PSTR("==============\n\r"                                                                      ));
  printf_P(PSTR("Jedes Commando wird mit einem CR oder LF abgeschlossen. Die Parameter mit ; getrennt\n\r"));
  printf_P(PSTR("Baudrate: 9600, 8 data bit, 1 stop bit, no parity, no flow control\n\r"                  ));
  printf_P(PSTR("\n\r"                                                                                    ));
  printf_P(PSTR("MI      Info           -> Name, Version (numerisch, 3Stellen)\n\r"                       ));
  printf_P(PSTR("\n\r"                                                                                    ));
  printf_P(PSTR("MS      Status         -> Mode (I=Idle, B=Busy, P=Programm)\n\r"                         ));
  printf_P(PSTR("\n\r"                                                                                    ));
  printf_P(PSTR("MW;<MA-Modul,1-255>;<Ausgang,0-3>;<Stellung,0-1>\n\r"                                    ));
  printf_P(PSTR("        Schalte Weiche an Modul/Channel\n\r"                                             ));
  printf_P(PSTR("\n\r"                                                                                    ));
  printf_P(PSTR("MA;<MA-Adresse,1-255>;<Stellung,0-1>\n\r"                                                ));
  printf_P(PSTR("        Schalte Weiche mit laufender Nummer\n\r"                                         ));
  printf_P(PSTR("\n\r"                                                                                    ));
  printf_P(PSTR("MF;<Zeile,1-248>;<fromKey,1-80>;<toKey,1-80>,<bidirectional,0-1>;<targetDef,0-1>\n\r"    ));
  printf_P(PSTR("        setze Route, toKey ist nur relevant wenn targetDef gesetzt\n\r"                  ));
  printf_P(PSTR("\n\r"                                                                                    ));
  printf_P(PSTR("MP;<Zeile,1-240>;<Route,1-0xF8>;<MA-Adresse,0-255>;<Stellung,0-1>\n\r"                   ));
  printf_P(PSTR("        setze Element (Magnetartikel + Zustand) und Zuordnung zu einer Route\n\r"        ));
  printf_P(PSTR("\n\r"                                                                                    ));
  printf_P(PSTR("MR      auslesen der Routes/Elements (MR;E MR;R) \n\r"                                   ));
  printf_P(PSTR("ME;42   Daten ins EEProm schreiben\n\r"                                                  ));
  printf_P(PSTR("MC;42   Alle Daten loeschen (initialisieren subcmd: 84 = stapel, 42, 21)\n\r"            ));
  printf_P(PSTR("MT      trace routes and list 'em (add nr for single ond)\n\r"                           ));
  printf_P(PSTR("MO      show amount of used memory\n\r"                                                  ));
  printf_P(PSTR("\n\r"                                                                                    ));
  printf_P(PSTR("Beispiel:\n\r"                                                                           ));
  printf_P(PSTR("MA;1;1    -> Schaltet die erste Weiche auf rot\n\r"                                      ));
  printf_P(PSTR("(Zahlen numerisch im klartext angeben)\n\r"                                              ));
#endif
}

void UARTComunicate(void)
{ //temps
  char ch = 1;
  char sendback = true;
  //char available?
  //ch = UARTGetC();
  ch = UARTReceive();
  //if new char is available and not processing
  if ((ch != 0)&&(bReceivedCommandComplete!=true))
  {
    //care escape sequences
    if (ch == 0x1B) 
    { //escape sequence
      sendback = false;
      ch = UARTGetC();
      if (ch == 0x5B)
      {
        ch = UARTGetC();
        switch (ch)
        { 
        case 'A': 
          //up arrow send last command
          UARTSendString(UARTLastCommand); 
          break;
        case 'B': break; //down arrow not used 
        case 'C': break; //right arrow not used
        case 'D': 
          //left arrow work like delete
          pLastReceiveByte--; //delete one byte
          UARTTransmit('\b'); 
          break;
        }
      }
    }
    else
    {
      //check for end transfer
      if ((ch == 0x0A) || (ch == 0x0D))
      { //set flag
        bReceivedCommandComplete = true;
        if ((pLastReceiveByte >= UARTReceiveBuffer)&&(pLastReceiveByte <= (UARTReceiveBuffer+100)))
        {
          //append zero
          *pLastReceiveByte = 0;
        }
        //reset pointer
        pLastReceiveByte = UARTReceiveBuffer;
        //send line feed
        UARTSendString("\n\r");
      }
      else
      { 
        if ((ch == 0x08) || (ch == 0x7F))
        {
          pLastReceiveByte--; //delete one byte
          UARTSendString("\b\x1B[K");
          sendback = false;
        }
        else
        {
          if ((pLastReceiveByte >= UARTReceiveBuffer)&&(pLastReceiveByte <= (UARTReceiveBuffer+100)))
          { //store byte
            *pLastReceiveByte = ch;
            //point to next
            pLastReceiveByte++;
          }
          else
          {
            //clear flag
            bReceivedCommandComplete = false;
            //reset pointer
            pLastReceiveByte = UARTReceiveBuffer;
          }
        }
      }
    }
    if (sendback)
    {
      //send back
      UARTTransmit(ch);
    }
  }
  //check command
  if (bReceivedCommandComplete == true)
  {
    //care command
    UARTCheckCommand();
    //remember
    strcpy(UARTLastCommand, UARTReceiveBuffer);
    //send promt
    UARTSendString("> ");
  }
}

//******************************************************************************************
// Keyboard functions
//******************************************************************************************
#ifdef __AVR_ATmega8__
void KeybRead(void)
{ //temps
  char cLimit = 10;
  //check inputs
  if (PINC & (1<<PC0)) {if (cKey[0]>0) cKey[0]--; } else {if (cKey[0]<cLimit) cKey[0]++;}
  if (PINC & (1<<PC1)) {if (cKey[1]>0) cKey[1]--; } else {if (cKey[1]<cLimit) cKey[1]++;}
  if (PINC & (1<<PC2)) {if (cKey[2]>0) cKey[2]--; } else {if (cKey[2]<cLimit) cKey[2]++;}
  if (PINC & (1<<PC3)) {if (cKey[3]>0) cKey[3]--; } else {if (cKey[3]<cLimit) cKey[3]++;}
  if (PINC & (1<<PC4)) {if (cKey[4]>0) cKey[4]--; } else {if (cKey[4]<cLimit) cKey[4]++;}
  if (PIND & (1<<PD3)) {if (cKey[5]>0) cKey[5]--; } else {if (cKey[5]<cLimit) cKey[5]++;}
  if (PIND & (1<<PD4)) {if (cKey[6]>0) cKey[6]--; } else {if (cKey[6]<cLimit) cKey[6]++;}
  if (PIND & (1<<PD5)) {if (cKey[7]>0) cKey[7]--; } else {if (cKey[7]<cLimit) cKey[7]++;}
  if (PIND & (1<<PD6)) {if (cKey[8]>0) cKey[8]--; } else {if (cKey[8]<cLimit) cKey[8]++;}
  if (PIND & (1<<PD7)) {if (cKey[9]>0) cKey[9]--; } else {if (cKey[9]<cLimit) cKey[9]++;}
}

char KeybIsKey(unsigned char Index)
{ //temps
  char cRet = false;
  //check index
  if (Index > 9) return false;
  //check value  
  if (cKey[Index] > 8) cRet = true;
  //fin
  return cRet;
}
#endif

#ifdef __AVR_ATmega32__
char KeyByIndex(char index)
{ char ret = false;
  switch (index)
  { 
    case  0: ret = (PINA & (1<<PA0)); break;
    case  1: ret = (PINA & (1<<PA1)); break;
    case  2: ret = (PINA & (1<<PA2)); break;
    case  3: ret = (PINA & (1<<PA3)); break;
    case  4: ret = (PINA & (1<<PA4)); break;
    case  5: ret = (PINA & (1<<PA5)); break;
    case  6: ret = (PINA & (1<<PA6)); break;
    case  7: ret = (PINA & (1<<PA7)); break;
    case  8: ret = (PINC & (1<<PC7)); break;
    case  9: ret = (PINC & (1<<PC6)); break;
    case 10: ret = (PINC & (1<<PC5)); break;
    case 11: ret = (PINC & (1<<PC4)); break;
    case 12: ret = (PINC & (1<<PC3)); break;
    case 13: ret = (PINC & (1<<PC2)); break;
    case 14: ret = (PINC & (1<<PC1)); break;
    case 15: ret = (PINC & (1<<PC0)); break;
    case 16: ret = (PIND & (1<<PD7)); break;
    case 17: ret = (PIND & (1<<PD6)); break;
    case 18: ret = (PIND & (1<<PD5)); break;
    case 19: ret = (PIND & (1<<PD4)); break;
    default: ret = false;
  }
  return ret;
}

void KeybSetGroup(unsigned char keybActiveGroup)
{
  switch (keybActiveGroup) 
  {
  case 0:
    PORTB |=  (1<<PB0);
    PORTB &= ~(1<<PB1);
    PORTB &= ~(1<<PB2);
    PORTB &= ~(1<<PB3);
    break;
  case 1:
    PORTB &= ~(1<<PB0);
    PORTB |=  (1<<PB1);
    PORTB &= ~(1<<PB2);
    PORTB &= ~(1<<PB3);
    break;
  case 2:
    PORTB &= ~(1<<PB0);
    PORTB &= ~(1<<PB1);
    PORTB |=  (1<<PB2);
    PORTB &= ~(1<<PB3);
    break;
  case 3:
    PORTB &= ~(1<<PB0);
    PORTB &= ~(1<<PB1);
    PORTB &= ~(1<<PB2);
    PORTB |=  (1<<PB3);
    break;
  }
  return;
}


void KeybRead(void)
{ 
  switch(keybActiveState)
  {
  case KEYB_STATE_IDLE:
    if (keybTimeout > 5) 
    { //set next state
      keybActiveState = KEYB_STATE_READ_GROUP;
      keybTimeout = 0;
      //start with first key in first group
      keybActiveGroup = 0;
      keybActiveKey = 0;
    }
    keybTimeout++;
    break;
  
  case KEYB_STATE_SET_NEXT_GROUP:
    //set next group
    keybActiveGroup++;
    if (keybActiveGroup >= MAX_KEY_GROUPS) keybActiveGroup = 0;
    //switch group channel
    KeybSetGroup(keybActiveGroup);
    //set next state
    keybActiveState = KEYB_STATE_READ_GROUP;
    keybTimeout = 0;
    keybActiveKey = 0;    
    break;
  
  case KEYB_STATE_READ_GROUP:
    if (keybActiveKey < MAX_REAL_KEYS)
    { //temps
      unsigned char key = (keybActiveGroup * MAX_REAL_KEYS) + keybActiveKey;
      //check inputs
      if (KeyByIndex(keybActiveKey)) 
      {
        if (cKey[key]>0) cKey[key]--; 
      } else {
        if (cKey[key]<MAX_KEYS_COUNT_LIMIT) cKey[key]++;
      }
      keybActiveKey++;
    } else {
      //set next state
      keybActiveState = KEYB_STATE_SET_NEXT_GROUP;
      keybTimeout = 0;
      keybActiveKey = 0;
    }
    break;
  
  default:
    keybActiveState = KEYB_STATE_IDLE;
    break;
  }
}

char KeybIsKey(unsigned char Index)
{ //temps
  char cRet = false;
  //check index
  if (Index >= MAX_VIRTUAL_KEYS) return false;
  //check value  
  if (cKey[Index] > 8) cRet = true;
  //fin
  return cRet;
}
#endif


//******************************************************************************************
// memory functions
//******************************************************************************************
char MemoryReadFromEEprom(void) 
{ //temps
  char bRet = false;
  unsigned char ucGuilty = 0;
  //checkbyte
  ucGuilty = eeprom_read_byte(&EEPGuilty);
  if (ucGuilty == 42) 
  { //read block from memory
    eeprom_read_block
      ( &EEProm
      , &EEPPromShadow
      , sizeof(EEProm)
      );
    //some settings
    InitRoute = eeprom_read_byte(&EEPInitRoute);
    InitRouteWait = eeprom_read_byte(&EEPInitRouteWait);
    InitWaitBetweenSwitch = eeprom_read_byte(&EEPInitWaitBetweenSwitch);
    //read ok
    bRet = true;
  } 
  if (!bRet)
  { 
    //tell user
    #ifdef __AVR_ATmega32__
        printf_P(PSTR("Error reading EEProm! Checkbyte not guilty (V:%u)\n\r"),ucGuilty);
    #endif
    #ifdef __AVR_ATmega8__
        printf_P(PSTR("ErrEEProm!(V:%u)\n\r"),ucGuilty);
    #endif
  }
  //fin
  return bRet;
}

char MemorySaveToEEprom(void)
{ //temps
  char bRet = true;
  //write block to memory
  eeprom_write_block
    ( &EEProm
    , &EEPPromShadow
    , sizeof(EEProm)
    );
  //some settings
  eeprom_write_byte(&EEPInitRoute, InitRoute);
  eeprom_write_byte(&EEPInitRouteWait, InitRouteWait);
  eeprom_write_byte(&EEPInitWaitBetweenSwitch, InitWaitBetweenSwitch);
  //checkbyte
  eeprom_write_byte(&EEPGuilty, 42);
  //tell user
  printf_P(PSTR("EEW!\n\r"));
  //fin
  return bRet;
}

#ifdef __AVR_ATmega32__
void MemoryStapelInit1(void)
{ 
  MemorySetRoute  ( 1,  50,  51, false, false);
  MemorySetElement( 1,  1,  1, 0);
  MemorySetElement( 2,  1,  2, 0);
  MemorySetElementWait(3,  1, 33, 1, 2, 0);    
  MemorySetRoute  ( 2,  50,  51, false, false);
  MemorySetElement( 4,  2,  1, 0);
  MemorySetElement( 5,  2,  2, 1);
  MemorySetElement( 6,  2,  4, 0);
  MemorySetElementWait(7,  2, 34, 1, 2, 0);
  MemorySetRoute  ( 3,  50,  51, false, false);
  MemorySetElement( 8,  3,  1, 0);
  MemorySetElement( 9,  3,  2, 1);
  MemorySetElement(10,  3,  4, 1);
  MemorySetElementWait(11,  3, 35, 1, 2, 0);
  MemorySetRoute  ( 4,  50,  51, false, false);
  MemorySetElement(12,  4,  1, 1);
  MemorySetElement(13,  4,  3, 0);
  MemorySetElement(14,  4, 10, 1);
  MemorySetElementWait(15,  4, 36, 1, 2, 0);
  MemorySetRoute  ( 5,  50,  51, false, false);
  MemorySetElement(16,  5,  1, 1);
  MemorySetElement(17,  5,  3, 0);
  MemorySetElement(18,  5, 10, 0);
  MemorySetElementWait(19,  5, 23, 1, 2, 0);
  MemorySetRoute  ( 6,  50,  51, false, false);
  MemorySetElement(20,  6,  1, 1);
  MemorySetElement(21,  6,  3, 1);
  MemorySetElementWait(22,  6, 24, 1, 2, 0);
}

void MemoryStapelInit2(void)
{
  MemorySetRoute  ( 7,  9,  0, false, false);
  MemorySetElement(23,  7, 33, 0);
  MemorySetRoute  ( 8, 10,  0, false, false);
  MemorySetElement(24,  8, 34, 0);
  MemorySetRoute  ( 9, 11,  0, false, false);
  MemorySetElement(25,  9, 35, 0);
  MemorySetRoute  (10, 12,  0, false, false);
  MemorySetElement(26, 10, 36, 0);
  MemorySetRoute  (11, 13,  0, false, false);
  MemorySetElement(27, 11, 23, 0);
  MemorySetRoute  (12, 14,  0, false, false);
  MemorySetElement(28, 12, 24, 0);
}

void MemoryStapelInit3(void)
{
  MemorySetRoute  (13,  3, 15, false, true);
  MemorySetElement(29, 13, 17, 0);
  MemorySetElement(30, 13, 19, 0);
  MemorySetElementWait(31, 13, 33, 1, 2, 0);    
  MemorySetRoute  (14,  4, 15, false, true);
  MemorySetElement(32, 14, 17, 0);
  MemorySetElement(33, 14, 19, 1);
  MemorySetElement(34, 14, 20, 0);
  MemorySetElementWait(35, 14, 34, 1, 2, 0);
  MemorySetRoute  (15,  5, 15, false, true);
  MemorySetElement(36, 15, 17, 0);
  MemorySetElement(37, 15, 19, 1);
  MemorySetElement(38, 15, 20, 1);
  MemorySetElementWait(39, 15, 35, 1, 2, 0);
  MemorySetRoute  (16,  6, 15, false, true);
  MemorySetElement(40, 16, 17, 1);
  MemorySetElement(41, 16, 18, 0);
  MemorySetElement(42, 16, 21, 1);
  MemorySetElementWait(43, 16, 36, 1, 2, 0);
  MemorySetRoute  (17,  7, 15, false, true);
  MemorySetElement(44, 17, 17, 1);
  MemorySetElement(45, 17, 18, 0);
  MemorySetElement(46, 17, 21, 0);
  MemorySetElementWait(47, 17, 23, 1, 2, 0);
  MemorySetRoute  (18,  8, 15, false, true);
  MemorySetElement(48, 18, 17, 1);
  MemorySetElement(49, 18, 18, 1);
  MemorySetElementWait(50, 18, 24, 1, 2, 0);
}

void MemoryStapelInit4(void)
{ 
  MemorySetRoute  (19, 16, 24, false, true);
  MemorySetElement(51, 19, 13, 0);
  MemorySetElement(52, 19, 14, 0);
  MemorySetElementWait(53, 19, 25, 1, 2, 0);    
  MemorySetRoute  (20, 16, 23, false, true);
  MemorySetElement(54, 20, 13, 0);
  MemorySetElement(55, 20, 14, 1);
  MemorySetElement(56, 20, 16, 0);
  MemorySetElementWait(57, 20, 26, 1, 2, 0);
  MemorySetRoute  (21, 16, 22, false, true);
  MemorySetElement(58, 21, 13, 0);
  MemorySetElement(59, 21, 14, 1);
  MemorySetElement(60, 21, 16, 1);
  MemorySetElementWait(61, 21, 27, 1, 2 ,0);
  MemorySetRoute  (22, 16, 21, false, true);
  MemorySetElement(62, 22, 13, 1);
  MemorySetElement(63, 22, 15, 0);
  MemorySetElement(64, 22, 22, 1);
  MemorySetElementWait(65, 22, 28, 1, 2, 0);
  MemorySetRoute  (23, 16, 20, false, true);
  MemorySetElement(66, 23, 13, 1);
  MemorySetElement(67, 23, 15, 0);
  MemorySetElement(68, 23, 22, 0);
  MemorySetElementWait(69, 23, 29, 1, 2, 0);
  MemorySetRoute  (24, 16, 19, false, true);
  MemorySetElement(70, 24, 13, 1);
  MemorySetElement(71, 24, 15, 1);
  MemorySetElementWait(72, 24, 30, 1, 2, 0);
}

void MemoryStapelInit5(void)
{
  MemorySetRoute  (25, 30,  0, false, false);
  MemorySetElement(73, 25, 25, 0);
  MemorySetRoute  (26, 29,  0, false, false);
  MemorySetElement(74, 26, 26, 0);
  MemorySetRoute  (27, 28,  0, false, false);
  MemorySetElement(75, 27, 27, 0);
  MemorySetRoute  (28, 27,  0, false, false);
  MemorySetElement(76, 28, 28, 0);
  MemorySetRoute  (29, 26,  0, false, false);
  MemorySetElement(77, 29, 29, 0);
  MemorySetRoute  (30, 25,  0, false, false);
  MemorySetElement(78, 30, 30, 0);
}

void MemoryStapelInit6(void)
{
  MemorySetRoute  (31, 124, 82, true, true);
  MemorySetElement(79, 31,  5, 0);
  MemorySetElement(80, 31,  6, 0);
  MemorySetElementWait(81, 31, 25, 1, 2, 0);    
  MemorySetRoute  (32, 123, 82, true, true);
  MemorySetElement(82, 32,  5, 0);
  MemorySetElement(83, 32,  6, 1);
  MemorySetElement(84, 32,  8, 0);
  MemorySetElementWait(85, 32, 26, 1, 2, 0);
  MemorySetRoute  (33, 122, 82, true, true);
  MemorySetElement(86, 33,  5, 0);
  MemorySetElement(87, 33,  6, 1);
  MemorySetElement(88, 33,  8, 1);
  MemorySetElementWait(89, 33, 27, 1, 2, 0);
  MemorySetRoute  (34, 121, 82, true, true);
  MemorySetElement(90, 34,  5, 1);
  MemorySetElement(91, 34,  7, 0);
  MemorySetElement(92, 34,  9, 1);
  MemorySetElementWait(93, 34, 28, 1, 2, 0);
  MemorySetRoute  (35, 120, 82, true, true);
  MemorySetElement(94, 35,  5, 1);
  MemorySetElement(95, 35,  7, 0);
  MemorySetElement(96, 35,  9, 0);
  MemorySetElementWait(97, 35, 29, 1, 2, 0);
  MemorySetRoute  (36, 119, 82, true, true);
  MemorySetElement(98, 36,  5, 1);
  MemorySetElement(99, 36,  7, 1);
  MemorySetElementWait(100, 36, 30, 1, 2, 0);
}

void MemoryStapelInit7(void)
{
  MemorySetRoute  ( 37, 18,  0, false, false);
  MemorySetElement(101, 37, 33, 0);
  MemorySetElement(102, 37, 34, 0);
  MemorySetElement(103, 37, 35, 0);
  MemorySetElement(104, 37, 36, 0);
  MemorySetElement(105, 37, 23, 0);
  MemorySetElement(106, 37, 24, 0);

  MemorySetRoute  ( 38, 31,  0, false, false);
  MemorySetElement(107, 38, 25, 0);
  MemorySetElement(108, 38, 26, 0);
  MemorySetElement(109, 38, 27, 0);
  MemorySetElement(110, 38, 28, 0);
  MemorySetElement(111, 38, 29, 0);
  MemorySetElement(112, 38, 30, 0);

  MemorySetRoute  ( 39, 115,  0, false, false);
  MemorySetElementWait(113, 39, 37, 0, 0, true);

  MemorySetRoute  ( 40, 82,  0, false, false);
  MemorySetElementWait(114, 40, 38, 0, 0, true);
}

void MemoryStapelInit8(void)
{
  MemorySetRoute  ( 41, 33, 36, true, true);
  MemorySetElement(115, 41, 42, 1);
  MemorySetElement(116, 41, 43, 1);
  MemorySetElement(117, 41, 44, 0);
  MemorySetElement(118, 41, 41, 0);
  MemorySetElement(119, 41, 45, 0);
  MemorySetRoute  ( 42, 35, 34, true, true);
  MemorySetElement(120, 42, 44, 1);
  MemorySetElement(121, 42, 41, 1);
  MemorySetElement(122, 42, 42, 0);
  MemorySetElement(123, 42, 43, 0);
  MemorySetElement(124, 42, 45, 1);
  MemorySetRoute  ( 43, 33, 34, true, true);
  MemorySetElement(125, 43, 44, 1);
  MemorySetElement(126, 43, 41, 1);
  MemorySetElement(127, 43, 42, 1);
  MemorySetElement(128, 43, 43, 1);
  MemorySetElement(129, 43, 45, 1);
  MemorySetRoute  ( 44, 35, 36, true, true);
  MemorySetElementWait(130, 44, 43, 0, 0, true);
}

void MemoryStapelInit9(void)
{
  MemorySetRoute  (     45,  3, 32, false, true);
  MemorySetElement(    143, 45, 32, 0);
  MemorySetElementWait(144, 45, 41, 0, 0, true);
  MemorySetElementWait(145, 45,  1, 0, 0, true);
  MemorySetRoute  (     46,  4, 32, false, true);
  MemorySetElement(    146, 46, 32, 0);
  MemorySetElementWait(147, 46, 41, 0, 0, true);
  MemorySetElementWait(148, 46,  2, 0, 0, true);
  MemorySetRoute  (     47,  5, 32, false, true);
  MemorySetElement(    149, 47, 32, 0);
  MemorySetElementWait(150, 47, 41, 0, 0, true);
  MemorySetElementWait(151, 47,  3, 0, 0, true);
  MemorySetRoute  (     48,  6, 32, false, true);
  MemorySetElement(    152, 48, 32, 0);
  MemorySetElementWait(153, 48, 41, 0, 0, true);
  MemorySetElementWait(154, 48,  4, 0, 0, true);
  MemorySetRoute  (     49,  7, 32, false, true);
  MemorySetElement(    155, 49, 32, 0);
  MemorySetElementWait(156, 49, 41, 0, 0, true);
  MemorySetElementWait(157, 49,  5, 0, 0, true);
  MemorySetRoute  (     50,  8, 32, false, true);
  MemorySetElement(    158, 50, 32, 0);
  MemorySetElementWait(159, 50, 41, 0, 0, true);
  MemorySetElementWait(160, 50,  6, 0, 0, true);

  MemorySetRoute  (     51, 1,  24, false, true);
  MemorySetElement(    161, 51, 31, 0);
  MemorySetElementWait(162, 51, 42, 0, 0, true);
  MemorySetElementWait(163, 51, 31, 0, 0, true);
  MemorySetRoute  (     52, 1,  23, false, true);
  MemorySetElement(    164, 52, 31, 0);
  MemorySetElementWait(165, 52, 42, 0, 0, true);
  MemorySetElementWait(166, 52, 32, 0, 0, true);
  MemorySetRoute  (     53, 1,  22, false, true);
  MemorySetElement(    167, 53, 31, 0);
  MemorySetElementWait(168, 53, 42, 0, 0, true);
  MemorySetElementWait(169, 53, 33, 0, 0, true);
  MemorySetRoute      ( 54, 1,  21, false, true);
  MemorySetElement(    170, 54, 31, 0);
  MemorySetElementWait(171, 54, 42, 0, 0, true);
  MemorySetElementWait(172, 54, 34, 0, 0, true);
  MemorySetRoute  (     55, 1,  20, false, true);
  MemorySetElement(    173, 55, 31, 0);
  MemorySetElementWait(174, 55, 42, 0, 0, true);
  MemorySetElementWait(175, 55, 35, 0, 0, true);
  MemorySetRoute  (     56, 1,  19, false, true);
  MemorySetElement(    176, 56, 31, 0);
  MemorySetElementWait(177, 56, 42, 0, 0, true);
  MemorySetElementWait(178, 56, 36, 0, 0, true);
}


void MemoryStapelInit10(void)
{
  MemorySetRoute  ( 57, 1,  3, false, true);
  MemorySetElement(    179, 57, 32, 1);
  MemorySetElementWait(180, 57, 43, 0, 0, true);
  MemorySetElementWait(181, 57,  1, 0, 0, true);
  MemorySetRoute  ( 58, 1,  4, false, true);
  MemorySetElement(    182, 58, 32, 1);
  MemorySetElementWait(183, 58, 43, 0, 0, true);
  MemorySetElementWait(184, 58,  2, 0, 0, true);
  MemorySetRoute  ( 59, 1,  5, false, true);
  MemorySetElement(    185, 59, 32, 1);
  MemorySetElementWait(186, 59, 43, 0, 0, true);
  MemorySetElementWait(187, 59,  3, 0, 0, true);
  MemorySetRoute  ( 60, 1,  6, false, true);
  MemorySetElement(    188, 60, 32, 1);
  MemorySetElementWait(189, 60, 43, 0, 0, true);
  MemorySetElementWait(190, 60,  4, 0, 0, true);
  MemorySetRoute  ( 61, 1,  7, false, true);
  MemorySetElement(    191, 61, 32, 1);
  MemorySetElementWait(192, 61, 43, 0, 0, true);
  MemorySetElementWait(193, 61,  5, 0, 0, true);
  MemorySetRoute  ( 62, 1,  8, false, true);
  MemorySetElement(    194, 62, 32, 1);
  MemorySetElementWait(195, 62, 43, 0, 0, true);
  MemorySetElementWait(196, 62,  6, 0, 0, true);

  MemorySetRoute  ( 63, 24, 32, false, true);
  MemorySetElement(    197, 63, 31, 1);
  MemorySetElementWait(198, 63, 43, 0, 0, true);
  MemorySetElementWait(199, 63, 31, 0, 0, true);
  MemorySetRoute  ( 64, 23, 32, false, true);
  MemorySetElement(    200, 64, 31, 1);
  MemorySetElementWait(201, 64, 43, 0, 0, true);
  MemorySetElementWait(202, 64, 32, 0, 0, true);
  MemorySetRoute  ( 65, 22, 32, false, true);
  MemorySetElement(    203, 65, 31, 1);
  MemorySetElementWait(204, 65, 43, 0, 0, true);
  MemorySetElementWait(205, 65, 33, 0, 0, true);
  MemorySetRoute  ( 66, 21, 32, false, true);
  MemorySetElement(    206, 66, 31, 1);
  MemorySetElementWait(207, 66, 43, 0, 0, true);
  MemorySetElementWait(208, 66, 34, 0, 0, true);
  MemorySetRoute  ( 67, 20, 32, false, true);
  MemorySetElement(    209, 67, 31, 1);
  MemorySetElementWait(210, 67, 43, 0, 0, true);
  MemorySetElementWait(211, 67, 35, 0, 0, true);
  MemorySetRoute  ( 68, 19, 32, false, true);
  MemorySetElement(    212, 68, 31, 1);
  MemorySetElementWait(213, 68, 43, 0, 0, true);
  MemorySetElementWait(214, 68, 36, 0, 0, true);
}

void MemoryStapelInit11(void)
{
  MemorySetElement(215, 13, 32, 1);
  MemorySetElement(216, 14, 32, 1);
  MemorySetElement(217, 15, 32, 1);
  MemorySetElement(218, 16, 32, 1);
  MemorySetElement(219, 17, 32, 1);
  MemorySetElement(220, 18, 32, 1);

  MemorySetElement(221, 19, 31, 1);
  MemorySetElement(222, 20, 31, 1);
  MemorySetElement(223, 21, 31, 1);
  MemorySetElement(224, 22, 31, 1);
  MemorySetElement(225, 23, 31, 1);
  MemorySetElement(226, 24, 31, 1);
}
#endif

void MemoryTestInit(char token)
{ // init for stapelmueller
  if (token == 84) 
  { 
    #ifdef __AVR_ATmega32__
      MemoryStapelInit1(); //init routes upper incomming
      MemoryStapelInit2(); //init routes upper stopping
      MemoryStapelInit3(); //init routes upper outgoing
      MemoryStapelInit4(); //init routes lower incomming
      MemoryStapelInit5(); //init routes upper stopping
      MemoryStapelInit6(); //init routes lower outgoing
      MemoryStapelInit7(); //init routes stopping upper/lower
      MemoryStapelInit8(); //init Hosentreager and append to existing routes
      MemoryStapelInit9(); //init Hosentreager crossing routes
      MemoryStapelInit10(); //add Hosentraeger straight through routes
      MemoryStapelInit11(); //care directions
    #endif
  }
  if (token == 42)
  {
    //init routes
    MemorySetRoute(1, 1, 4, false, true);
    MemorySetRoute(2, 1, 3, false, true);
    MemorySetRoute(3, 4, 5, false, true);
    MemorySetRoute(4, 3, 5, false, true);
    MemorySetRoute(5, 2, 0, false, false);
    MemorySetRoute(6, 6, 0, false, false);
    //define elements
    MemorySetElement(1, 1, 1, 1);
    MemorySetElement(2, 1, 2, 1);
    MemorySetElement(3, 2, 2, 0);
    MemorySetElement(4, 3, 3, 1);
    MemorySetElement(5, 3, 4, 1);
    MemorySetElement(6, 4, 4, 0);
    MemorySetElement(7, 5, 2, 0);
    MemorySetElement(8, 6, 3, 0);
  }
  if (token == 21)
  {
    #ifdef __AVR_ATmega32__
      char i;
      for (i = 0; i < 80; i++)
      {
        MemorySetRoute(i+1, i+1, 0, false, false);      
        MemorySetElement(i+1, i+1, (i/2)+1, i%2);
      }
    #endif
  }
  //tell user
  #ifdef __AVR_ATmega32__
    printf_P(PSTR("Memory initial filled!\n\r"));
  #endif
  #ifdef __AVR_ATmega8__
    printf_P(PSTR("M_F!\n\r"));
  #endif
}

void MemoryInit(void) 
{ //init from EEprom
  if (!MemoryReadFromEEprom()) {
    //clear memory
    MemoryClear();
    //test init
    MemoryTestInit(42);
  }
}

char MemorySetElement
  ( unsigned char index
  , unsigned char assigned_id
  , unsigned char dec_adr
  , unsigned char dec_state
  )
{ //call base function
  return MemorySetElementWait(index, assigned_id, dec_adr, dec_state, 0, 0);
}

char MemorySetElementWait
  ( unsigned char index
  , unsigned char assigned_id
  , unsigned char dec_adr
  , unsigned char dec_state
  , unsigned char wait
  , unsigned char forwardRoute
 )
{ //check line
  if (  (index          > 0)
     && (index          <= MAX_MEMORY_ELEMENTS)
     && (assigned_id    > 0) 
     && (assigned_id    <= MAX_MEMORY_ROUTES)
     && (dec_adr        > 0)
     && (dec_adr        <= MAX_DECODER_ADRESS)
     && (dec_state      >= 0)
     && (dec_state      <= MAX_DECODER_STATE)
     && (wait           >= 0)
     && (wait           <= MAX_DECODER_WAIT)
     && (forwardRoute   >= 0)
     && (forwardRoute   <= MAX_DECODER_FORWARD_ROUTE)
     )
  { //set data to arrays
    EEProm.arrMemoryElements[index-1].assigned_id = assigned_id;
    EEProm.arrMemoryElements[index-1].dec_adr = dec_adr;
    EEProm.arrMemoryElements[index-1].dec_state = dec_state;
    EEProm.arrMemoryElements[index-1].wait = wait;
    EEProm.arrMemoryElements[index-1].forwardRoute = forwardRoute;
    return true;
  } 
  else
  {
    return false;
  } 
}

char MemorySetRoute
  ( unsigned char index
  , unsigned char fromKey
  , unsigned char toKey
  , unsigned char bidirectional
  , unsigned char targetDefined
  )
{ //check line
  if (  (index > 0)   
     && (index <= MAX_MEMORY_ELEMENTS)
     && (fromKey > 0) 
     && (toKey >= 0)   
     && (bidirectional >= 0) 
     && (bidirectional <= MAX_BIDIRECTIONAL)
     )
  { //set data to arrays
    EEProm.arrMemoryRoutes[index-1].fromKey = fromKey;
    EEProm.arrMemoryRoutes[index-1].toKey = toKey;
    EEProm.arrMemoryRoutes[index-1].bidirectional = bidirectional;
    EEProm.arrMemoryRoutes[index-1].targetDefined = targetDefined;
    return true;
  } 
  else 
  {
    return false;
  }
}

void MemoryClear(void)
{ //set all to zeros
  memset(&EEProm, 0, sizeof(EEProm));
}

char MemoryDoRoute(unsigned char index)
{ //temps
  char ret = false;
  unsigned char i;
  //find according elements
  for (i = 0; i < MAX_MEMORY_ELEMENTS; i++) {
    if (EEProm.arrMemoryElements[i].assigned_id == (index + 1)) {
      //care about forwarded routes
      if (EEProm.arrMemoryElements[i].forwardRoute > 0)
      { //tell user
        printf_P(PSTR(";>>Rt%u"),EEProm.arrMemoryElements[i].dec_adr);
        //call recursiv sub route!
        if (!MemoryDoRoute(EEProm.arrMemoryElements[i].dec_adr-1))
        {  //error
           return false;
        }
      } 
      else
      { 
        //add to fifo
        if (FIFOBufferIn(i)) 
        { //tell user
          printf_P(PSTR(";+#%u"),i+1);
          ret = true;
        }
        else
        { //tell user
          #ifdef __AVR_ATmega32__
            printf_P(PSTR("Error adding Element to Fifo!\r\n"));
          #endif
          return false;
        }
      }
    }
  }
  //fin
  return ret;
}

char MemoryCareTargetlessRoutes
  ( unsigned char FirstPressedKey
  )
{ //temps
  unsigned char i;
  char ret = false;  
  //tell user
#ifdef __AVR_ATmega32__
  printf_P(PSTR("Care Targetless Route for key %u"),FirstPressedKey);
#endif
#ifdef __AVR_ATmega8__
  printf_P(PSTR("Rt_wo:%u"),FirstPressedKey);
#endif
  //loop all Routes
  for (i = 0; i < MAX_MEMORY_ROUTES; i++) {
    //pick the ones without target
    if (  (EEProm.arrMemoryRoutes[i].fromKey == FirstPressedKey)
       && (EEProm.arrMemoryRoutes[i].fromKey > 0)
       )
    { if (EEProm.arrMemoryRoutes[i].targetDefined) {
        //tell caller there are targets
        ret = true;
      } else {
        //tell user
        printf_P(PSTR(";+##Rt%u"),i+1);
        //care route
        MemoryDoRoute(i);
      }
    }
  }
  //tell user
  printf_P(PSTR("\n\r"));
  //fin
  return ret;
}

void MemoryCareCompleteRoutes
  ( unsigned char FirstPressedKey
  , unsigned char SecondPressedKey
  )
{ //temps
  unsigned char i;
  //tell user
#ifdef __AVR_ATmega32__
  printf_P(PSTR("Care routes with target. Start key %u, end key %u"),FirstPressedKey, SecondPressedKey);
#endif
#ifdef __AVR_ATmega8__
  printf_P(PSTR("rt_w:%u>%u"),FirstPressedKey, SecondPressedKey);
#endif
  //loop all Routes
  for (i = 0; i < MAX_MEMORY_ROUTES; i++) {
    //pick the ones with target
    if (  (EEProm.arrMemoryRoutes[i].fromKey > 0) 
       && (EEProm.arrMemoryRoutes[i].toKey > 0)
       && (EEProm.arrMemoryRoutes[i].targetDefined)
       && (  (  (EEProm.arrMemoryRoutes[i].fromKey == FirstPressedKey) 
             && (EEProm.arrMemoryRoutes[i].toKey == SecondPressedKey)
             )
          || (  (EEProm.arrMemoryRoutes[i].fromKey == SecondPressedKey)
             && (EEProm.arrMemoryRoutes[i].toKey == FirstPressedKey)
             && (EEProm.arrMemoryRoutes[i].bidirectional > 0)
             )
          )
       )
    { //tell user
      printf_P(PSTR(";+##R%u"),i+1);
      //care route
      MemoryDoRoute(i);
    }
  }
  //tell user
  printf_P(PSTR("\n\r"));
}

//******************************************************************************************
// Tasks
//******************************************************************************************

void LEDTask(void)
{
  //LED task
  if (lLedTaskTimer == 0) {
    //steatemachine
    switch (ledActiveState) 
    {
    case LED_STATE_IDLE:
      if (HeartbeatActive)
      { 
        //reset timer
        lLedTaskTimer = 1980;
      } else {
        //reset timer
        lLedTaskTimer = 20;
      }
      //set port
      HeartbeatSet(HeartbeatActive);
      break;
    case LED_STATE_WAITING:
      if (HeartbeatActive)
      { 
        //reset timer
        lLedTaskTimer = 50;
      } else {
        //reset timer
        lLedTaskTimer = 100;
      }
      //set port
      HeartbeatSet(HeartbeatActive);
      break;
    case LED_STATE_COMMIT:
      //reset timer
      lLedTaskTimer = 200;
      //set port
      HeartbeatSet(false);
      break;
    case LED_STATE_NACK:
      //reset timer
      lLedTaskTimer = 300;
      //set port
      HeartbeatSet(HeartbeatActive);
      if (HeartbeatNackCount > 0)
      {
        HeartbeatNackCount--;
      }
      else
      {
        ledActiveState = LED_STATE_IDLE;
      }
      break;
    default:
      break;
    }
    //invert
    HeartbeatActive = !HeartbeatActive;      
  }
}

void KeybTask(void)
{ //temps
  char i = 0;
  //check keys 
  if (lKeybTaskTimer == 0)
  {
    //reset timer
    lKeybTaskTimer = 40;
    //check keypressed
    if (!keyActive && (!RuningMemory)) 
    { //loop through keys
      for (i = 0; i<MAX_VIRTUAL_KEYS; i++)
      {
        //check key
        if (KeybIsKey(i)) {
          //start memory
          RuningMemory = true;
          LastPressedKey = i+1;
          //set flag
          keyActive = true;
          //leave
          break;
        }
      }
    }
    else
    {
      //wait till all keys released
      char tempKeyActive = false;
      for (i = 0; i < MAX_VIRTUAL_KEYS; i++)
      {
        if (KeybIsKey(i)) 
        { //check keys
          tempKeyActive = true;
          //leave
          break;
        }
      }
      //set flag
      keyActive = tempKeyActive;
    }
  }
}


void setNextMemoryActiveState(unsigned char p_nextMemoryActiveState)
{ // set the state
  memoryNextActiveState = p_nextMemoryActiveState;
  // when going back to idle also set back led tast
  if (MEMORY_STATE_IDLE == memoryNextActiveState)
  {
      ledActiveState = LED_STATE_IDLE;
      memoryTimeout = 0;
      FirstPressedKey = 0;
      SecondPressedKey = 0;
  }
  //sync timer
  lLedTaskTimer = 0;
}


void MemoryTaskStatemachine_IDLE(void)
{
  if (MEMORY_STATE_IDLE == memoryActiveState)
  {
    // check if a key press is detected
    if (RuningMemory)
    {
      // set next state
      setNextMemoryActiveState(MEMORY_STATE_FIRST_KEY_AKTIVE);
      lMemoryTaskTimer = 10;
      // user entertainment
      ledActiveState = LED_STATE_COMMIT;
    }
    DCCToggle();
  }
}

void MemoryTaskStatemachine_FIRST_KEY_AKTIVE(void)
{
  if (MEMORY_STATE_FIRST_KEY_AKTIVE == memoryActiveState)
  {
    //care flags and key
    RuningMemory = false;
    FirstPressedKey = LastPressedKey;
    if (!MemoryCareTargetlessRoutes(FirstPressedKey))
    {
      setNextMemoryActiveState(MEMORY_STATE_WAIT_AFTER);
    }
    else
    {
      //set next state
      setNextMemoryActiveState(MEMORY_STATE_WAITING_SECOND_KEY);
      lMemoryTaskTimer = 10;
    }
  }
}

void MemoryTaskStatemachine_WAITING_SECOND_KEY(void)
{
  if (MEMORY_STATE_WAITING_SECOND_KEY == memoryActiveState)
  {
    //care timeout
    if (memoryTimeout > 50)
    {
      setNextMemoryActiveState(MEMORY_STATE_WAIT_AFTER);
    }
    else
    {
      memoryTimeout++;
    }
    if (RuningMemory)
    {
      //set next state
      setNextMemoryActiveState(MEMORY_STATE_FINISH_ROUTE);
      memoryTimeout = 0;
      lMemoryTaskTimer = 10;
    }
  }
}

void MemoryTaskStatemachine_FINISH_ROUTE(void)
{
  if (MEMORY_STATE_FINISH_ROUTE == memoryActiveState)
  {
    //care flags and key
    RuningMemory = false;
    SecondPressedKey = LastPressedKey;
    //back to idle if equal
    if (FirstPressedKey == SecondPressedKey)
    {
      setNextMemoryActiveState(MEMORY_STATE_WAIT_AFTER);
    }
    else
    {
      MemoryCareCompleteRoutes(FirstPressedKey, SecondPressedKey);
      //set next state
      setNextMemoryActiveState(MEMORY_STATE_WAIT_AFTER);
      memoryTimeout = 0;
      lMemoryTaskTimer = 10;
    }
    //user entertainment
    ledActiveState = LED_STATE_COMMIT;
  }
}

void MemoryTaskStatemachine_WAIT_AFTER(void)
{
  if (MEMORY_STATE_WAIT_AFTER == memoryActiveState)
  {
    // empty FIFO...
    if (FIFOBufferOut(&indexFIFOBuffer))
    {
      if ((indexFIFOBuffer >= 0) && (indexFIFOBuffer < MAX_MEMORY_ELEMENTS))
      {
        // remember index
        memoryActualIndex = indexFIFOBuffer;
        //... care wait time...
        lMemoryTaskTimer = (long)EEProm.arrMemoryElements[indexFIFOBuffer].wait * 1000;
        if (lMemoryTaskTimer < 200)
          lMemoryTaskTimer = 200;
        // switch it
        setNextMemoryActiveState(MEMORY_STATE_SWITCH_IT);
      }
      indexFIFOBuffer = 0xFF;
      //... and leave if empty.
    }
    else
    {
      setNextMemoryActiveState(MEMORY_STATE_IDLE);
    }
  }
}

void MemoryTaskStatemachine_SWITCH_IT(void)
{
  if (MEMORY_STATE_SWITCH_IT == memoryActiveState)
  {
    //... care contend...
    DCCSwitch((EEProm.arrMemoryElements[memoryActualIndex].dec_adr - 1) / 4, (EEProm.arrMemoryElements[memoryActualIndex].dec_adr - 1) % 4, EEProm.arrMemoryElements[memoryActualIndex].dec_state);
    // send twice to capture ESD
    DCCSwitch((EEProm.arrMemoryElements[memoryActualIndex].dec_adr - 1) / 4, (EEProm.arrMemoryElements[memoryActualIndex].dec_adr - 1) % 4, EEProm.arrMemoryElements[memoryActualIndex].dec_state);
    // back to previous state
    setNextMemoryActiveState(MEMORY_STATE_WAIT_AFTER);
    lMemoryTaskTimer = 10;
  }
}

void MemoryTask(void)
{ 
  //check keys 
  if (lMemoryTaskTimer == 0)
  {
    //reset timer
    lMemoryTaskTimer = 100;
    //statemachine
    MemoryTaskStatemachine_IDLE();
    MemoryTaskStatemachine_FIRST_KEY_AKTIVE();
    MemoryTaskStatemachine_WAITING_SECOND_KEY();
    MemoryTaskStatemachine_FINISH_ROUTE();
    MemoryTaskStatemachine_WAIT_AFTER();
    MemoryTaskStatemachine_SWITCH_IT();
    memoryActiveState = memoryNextActiveState;
  }
}


//******************************************************************************************
// main ruitine
//******************************************************************************************
int main(void)
{ //init all hardware stuff
  InitPorts();
  UARTInit();
  Selftest();
  UARTHello();
  InitTimer();
  MemoryInit();
  //send promt
  UARTSendString("> ");
  //wait forever for receive
  while (1)
  { //timer0
    LEDTask();
    //timer1
    KeybTask();
    //timer2
    MemoryTask();
    //care keyboard
    KeybRead();
    //care comunication
    UARTComunicate();
  }
  //fin
  return 0;
}