//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------

#include <SI_C8051F320_Register_Enums.h>
#include <stdint.h>
#include "USBXpress/USBXpress.h"
#include "descriptor.h"

//-----------------------------------------------------------------------------
// Global CONSTANTS
//-----------------------------------------------------------------------------

#define SYSCLK       12000000       // SYSCLK frequency in Hz
#define SPI_CLOCK    250000         // SPI clock 250 kHz


//-----------------------------------------------------------------------------
// Function PROTOTYPES
//-----------------------------------------------------------------------------

void SYSCLK_Init (void);
void PORT_Init (void);
void Timer2_Init (int counts);
void SPI0_Init (void);
void ResetSources_Init (void);
void External_Devices_Init (void);

INTERRUPT_PROTO(Timer2_ISR, TIMER2_IRQn);

void delay_100ms(void);

enum ERRORSTATES {
	NO_ERROR,
	ADC_START_PIN_CANNOT_BE_LOWERED,
	ADC_TEST_NOT_COMPLETE,
	ADC_TEST_READ_WRITE_REGISTER_FAIL
} stateOfError = NO_ERROR;

void error(enum ERRORSTATES st) {
	stateOfError = st;
}

//-----------------------------------------------------------------------------
// blinker functions
//-----------------------------------------------------------------------------
SBIT(LED, SFR_P2, 2);                  // LED='1' means ON
SBIT(LED2, SFR_P2, 3);
SBIT(btnCalibrate_inv, SFR_P2, 0);  //'0' when pressed

bit blinkState = 1;
uint8_t blinkDividerCount = 0;
#define blinkDivider 2
uint8_t blinkCurTick = 0;
uint8_t blinkTicks = 3;
#define blinkPause 5*2

void blinkSay (uint8_t counts)
{
	blinkTicks = counts;
}

void blinkDo ()
{
	uint8_t blinkStopper;
	if (blinkDividerCount++ >= blinkDivider) {
		blinkDividerCount = 0;
		blinkStopper = (blinkState ? blinkTicks : blinkPause);
		if (blinkCurTick >= blinkStopper) {
		   blinkState = !blinkState;
		   blinkCurTick = 0;
		} else {
			if (blinkState) {LED = !LED;};
			if (!LED) {blinkCurTick++;};
		}
	}
}

#define blink2Divider 5
int blink2DividerCount = 0;
void blink2Do()
{
	if (stateOfError)
	{
		LED2 = 1;
	} else {
		if (blink2DividerCount++ >= blink2Divider) {
			blink2DividerCount = 0;
			LED2 = !LED2;
		}
	}
}

int watchCounter;
enum ERRORSTATES watchState;
void setWatchDog(int cnt, enum ERRORSTATES err)
{
	watchCounter = cnt;
	watchState = err;
}
void checkWatchDog()
{
	if (watchCounter && (watchCounter-- == 1)) {error(watchState);}
}

//-----------------------------------------------------------------------------
// ADC functions
//-----------------------------------------------------------------------------

SBIT(CS1, SFR_P1, 2);
SBIT(ChipSelect1_inv, SFR_P1, 2);
SBIT(CS2, SFR_P1, 4);
SBIT(ChipSelect2_inv, SFR_P1, 4);
SBIT(Start, SFR_P1, 3);
SBIT(Reset_inv, SFR_P1, 5);
SBIT(Data_RDY1_inv, SFR_P1, 1);
SBIT(Data_RDY2_inv, SFR_P1, 6);
bit flagSPIDataReaded = 0;
uint8_t byteSPIDataReaded = 0;

void ads1246Reset()
{
	delay_100ms();
	Start=1;    		// по диаграмме стр 49 оригинального описания АЦП
	Reset_inv=0;			// аппаратный сброс на вход АЦП
	delay_100ms(); 		// АЦП сброшено в исходно состояние
	Reset_inv=1;
	delay_100ms(); 		// Ждем когда отресетится
	delay_100ms(); 		// Ждем когда отресетится
	delay_100ms(); 		// Ждем когда отресетится
}

uint8_t SPISendReceiveByte(uint8_t dataByte)
{
	flagSPIDataReaded = 0;
	SPI0DAT = dataByte;
	while (!flagSPIDataReaded) {};			// ждем, когда сработает прерывание
	return byteSPIDataReaded;
}

#define ADS1246_COMMAND_SWITCH_TO_RDATAC 0x14
#define ADS1246_COMMAND_SWITCH_TO_SDATAC 0x16
#define ADS1246_COMMAND_READ_REGISTER 0x20
#define ADS1246_COMMAND_WRITE_REGISTER 0x40
#define COMMAND_NOP 0xff

// ADS1246 registers:
// 00h BCS BCS1 BCS0 0 0 0 0 0 1
// 01h VBIAS 0 0 0 0 0 0 VBIAS1 VBIAS0
// 02h MUX1 CLKSTAT 0 0 0 0 MUXCAL2 MUXCAL1 MUXCAL0
// 03h SYS0 0 PGA2 PGA1 PGA0 DR3 DR2 DR1 DR0
// 04h OFC0 OFC7 OFC6 OFC5 OFC4 OFC3 OFC2 OFC1 OFC0
// 05h OFC1 OFC15 OFC14 OFC13 OFC12 OFC11 OFC10 OFC9 OFC8
// 06h OFC2 OFC23 OFC22 OFC21 OFC20 OFC19 OFC18 OFC17 OFC16
// 07h FSC0 FSC7 FSC6 FSC5 FSC4 FSC3 FSC2 FSC1 FSC0
// 08h FSC1 FSC15 FSC14 FSC13 FSC12 FSC11 FSC10 FSC9 FSC8
// 09h FSC2 FSC23 FSC22 FSC21 FSC20 FSC19 FSC18 FSC17 FSC16
// 0Ah ID ID3 ID2 ID1 ID0 DRDY 0 0 0 MODE

#define ADS1246_FIRST_REGISTER 0x00
#define ADS1246_REGISTER_BCS   0x00 //    BCS1  BCS0     0     0     0       0       0       1
#define ADS1246_REGISTER_VBIAS 0x01 //       0     0     0     0     0       0  VBIAS1  VBIAS0
#define ADS1246_REGISTER_MUX1  0x02 // CLKSTAT     0     0     0     0 MUXCAL2 MUXCAL1 MUXCAL0
#define ADS1246_REGISTER_SYS0  0x03 //       0  PGA2  PGA1  PGA0   DR3     DR2     DR1     DR0

#define ADS1246_REGISTER_OFC0  0x04 //    OFC7  OFC6  OFC5  OFC4  OFC3   OFC2   OFC1   OFC0
#define ADS1246_REGISTER_OFC1  0x05 //   OFC15 OFC14 OFC13 OFC12 OFC11  OFC10   OFC9   OFC8
#define ADS1246_REGISTER_OFC2  0x06 //   OFC23 OFC22 OFC21 OFC20 OFC19  OFC18  OFC17  OFC16

#define ADS1246_REGISTER_FSC0  0x07 //    FSC7  FSC6  FSC5  FSC4  FSC3   FSC2   FSC1   FSC0
#define ADS1246_REGISTER_FSC1  0x08 //   FSC15 FSC14 FSC13 FSC12 FSC11  FSC10   FSC9   FSC8
#define ADS1246_REGISTER_FSC2  0x09 //   FSC23 FSC22 FSC21 FSC20 FSC19  FSC18  FSC17  FSC16

#define ADS1246_REGISTER_ID    0x0A // ID3 ID2 ID1 ID0 DRDY 0 0 0 MODE
#define ADS1246_NUMBER_OF_REGISTERS 0x0A + 1

void ads1246WriteRegister(uint8_t reg, uint8_t dataByte)
{
	SPISendReceiveByte(ADS1246_COMMAND_WRITE_REGISTER | reg);// команда WR 1-ый байт  второй полубайт это номер регистра
	SPISendReceiveByte(1-1);// команда WR  2-ой байт число записываемых подряд регистров минус один
	SPISendReceiveByte(dataByte);// байт данных

}

uint8_t ads1246ReadRegister(uint8_t reg)
{
	SPISendReceiveByte(ADS1246_COMMAND_READ_REGISTER | reg);   	// команда Read 1-ый, байт читаем регистр MUX
	SPISendReceiveByte(1-1);   	// команда Read 2-ой байт, число читаемых подряд регистров минус один
	// Код NOP "нет операции"; получение данных идёт в момент передачи этой команды
	return SPISendReceiveByte(COMMAND_NOP);
}

void ads1246ReadAllRegisters(uint8_t regs[ADS1246_NUMBER_OF_REGISTERS])
{
	uint8_t i;
	SPISendReceiveByte(ADS1246_COMMAND_READ_REGISTER  // команда Read 1-ый байт,
                       | ADS1246_FIRST_REGISTER);     // читаем с первого регистра
	SPISendReceiveByte(ADS1246_NUMBER_OF_REGISTERS - 1);   	// команда Read 2-ой байт, число читаемых подряд регистров минус один
	for (i=0; i++ < ADS1246_NUMBER_OF_REGISTERS;) {
		regs[i] = SPISendReceiveByte(COMMAND_NOP);
	}
}

void ads1246Test()
{
	ChipSelect1_inv = 0;			// выбор микросхемы 1
	ads1246Reset();
	ChipSelect1_inv = 1;			// отключаем микросхему 1

	ChipSelect2_inv = 0;			// выбор микросхемы 1
	ads1246Reset();
	ChipSelect2_inv = 1;			// отключаем микросхему 1

	ChipSelect1_inv = 0;			// выбор микросхемы 1

	while (Data_RDY1_inv) {}; // ждем появления Data_RDY1 = 0

	// Посылаем в АЦП команду SDATAC - прекращение непрерывного считывания данных
	// для считывания значения регистра
	SPISendReceiveByte(ADS1246_COMMAND_SWITCH_TO_SDATAC);
	while (Data_RDY1_inv) {}; // ждем появления Data_RDY1 = 0

	ads1246WriteRegister(ADS1246_REGISTER_MUX1, 0x4f);  //пишем регистр MUX2

	// читаем тот же регистр. Записанные данные должны совпасть
	if ((ads1246ReadRegister(ADS1246_REGISTER_MUX1) & 0x7f) != 0x4f) {error(ADC_TEST_READ_WRITE_REGISTER_FAIL);}
}

void ads1246Calibrate() {

	SPISendReceiveByte(ADS1246_COMMAND_SWITCH_TO_SDATAC);
	while (Data_RDY1_inv) {};

	ads1246WriteRegister(ADS1246_REGISTER_MUX1, 0x02); // Пишем в регистр MUX 02 - код системной калибровки
	SPISendReceiveByte(0x61);// Подаем команду SPI  SYSGCAL
	while (Data_RDY1_inv) {}; // ждем появления Data_RDY1 = 0

/*
 * мы можем прочесть результаты калибровки, если есть желание
 *
	SPISendReceiveByte(ADS1246_COMMAND_READ_REGISTER | 0x07);   	// команда Read 1-ый байт, регистр FSC0
	SPISendReceiveByte(3-1);   	// команда Read 2-ой байт, число читаемых подряд регистров минус один
	Some = SPISendReceiveByte(COMMAND_NOP);
	Useful = SPISendReceiveByte(COMMAND_NOP);
	Place = SPISendReceiveByte(COMMAND_NOP);
    // считали результаты системной калибровки
*/

	ads1246WriteRegister(ADS1246_REGISTER_MUX1, 0x01); // Пишем в регистр MUX 01 - код калибровки по уровню
	SPISendReceiveByte(0x60);// Подаем команду SPI  SYSOCAL
	while (Data_RDY1_inv) {}; // ждем появления Data_RDY1 = 0
/*
 * мы можем прочесть результаты калибровки, если есть желание
 *
	SPISendReceiveByte(ADS1246_COMMAND_READ_REGISTER | 0x07);   	// команда Read 1-ый байт, регистр FSC0
	SPISendReceiveByte(3-1);   	// команда Read 2-ой байт, число читаемых подряд регистров минус один
	Some = SPISendReceiveByte(COMMAND_NOP);
	Useful = SPISendReceiveByte(COMMAND_NOP);
	Place = SPISendReceiveByte(COMMAND_NOP);
    // считали результаты системной калибровки
*/
	ads1246WriteRegister(ADS1246_REGISTER_MUX1, 0x00); // Пишем в регистр MUX 00 - просто измерение
	// Посылаем в АЦП команду RDATAC - непрерывное считывание данных
	SPISendReceiveByte(ADS1246_COMMAND_SWITCH_TO_RDATAC);
	while (Data_RDY1_inv) {};
}


//-----------------------------------------------------------------------------
// External Global Variables
//-----------------------------------------------------------------------------

#define USB_BLOCK_SIZE  0x40 //!< Size of USB packst

enum WORKSTATES {
	INIT_SYSTEM,
	DEFAULT_READ,
	SET_SPEED,
} workstate = INIT_SYSTEM;

enum DATAAGE {
   	DATA_NEW,
   	DATA_SENDING,
   	DATA_OLD,
   	DATA_LOST,
   	DATA_NO_DATA,
};

   bool deviceWasOpened = false;
   struct exchangeStruct {
	   uint8_t mySize;
	   uint8_t myVersion;
	   enum WORKSTATES workstate; // !!!DANGER 'enum' can be longer than byte / depends on compiler
	   char trash[4];
	   int32_t dataReaded;
	   enum DATAAGE dataAge;
   } dataSet = {sizeof(struct exchangeStruct), 1, INIT_SYSTEM, "---", 0, DATA_NO_DATA};

   struct exchangeStruct cmdSet;

   int DEBUG_sz = sizeof(struct exchangeStruct);
   uint16_t *usbWxLen;
   uint16_t *usbRxLen;

void writeDataToHost(void)
{
	dataSet.workstate = workstate;
	Block_Write((uint8_t *)&dataSet, sizeof(dataSet), usbWxLen);
	if (dataSet.dataAge == DATA_NEW || dataSet.dataAge == DATA_LOST) {dataSet.dataAge = DATA_SENDING;}
}

void readCommandsFromHost(void)
{
	Block_Read((uint8_t *)&cmdSet, sizeof(cmdSet), usbRxLen);
	workstate = cmdSet.workstate;
}

/**************************************************************************//**
 * @brief USBXpress callback
 *
 * This function is called by USBXpress.
 *
 *****************************************************************************/
USBXpress_API_CALLBACK(myAPICallback)
{
   uint32_t INTVAL = Get_Callback_Source();

   if (INTVAL & DEVICE_OPEN) {
	   deviceWasOpened = true;
	   writeDataToHost();
	   readCommandsFromHost();
   }
   if (INTVAL & TX_COMPLETE)                          // USB Write complete
   {
	   if (dataSet.dataAge == DATA_SENDING) {dataSet.dataAge = DATA_OLD;}
   }
/*   if (INTVAL & DEVICE_OPEN)
   {
     if(usbRxReady)
     {
       //Prime first read if we are not already waitig for a read
       resetState();
       Block_Read(usbRxBuf, USB_BLOCK_SIZE, usbRxLen);   // Start first USB Read
       usbRxReady = 0;
     }
     //If usbRx is not ready then we're already pending a read and don't need to re-setup.
   }

   if (INTVAL & RX_COMPLETE)                          // USB Read complete
   {
	   switchRxBuffer(); //switch to next buffer. Length is automatically updated.

	   if(!*usbRxLen)
	   {
		   //If next buffer empty start RX immediately
		   if(Block_Read(usbRxBuf, USB_BLOCK_SIZE, usbRxLen))
		   {
		     usbRxReady = 1; // If read failed then mark inactive
		   }
	   }
	   else
	   {
	     //If next buffer not empty mark that inactive
	     usbRxReady = 1;
	   }

	   //Evaluate if our receive should kick the TX side.
	   if(usbTxReady && *usbTxLen)
	   {
	     usbTxReady = 0;
	     Block_Write(usbTxBuf, *usbTxLen, usbTxLen);
	   }

   }

   if (INTVAL & TX_COMPLETE)                          // USB Write complete
   {
      *usbTxLen = 0;    //Mark current buffer empty
      switchTxBuffer(); //Switch buffer

      if(*usbTxLen)
      {
        if(Block_Write(usbTxBuf, *usbTxLen, usbTxLen))
        {
          // if write failed then mark inactive
          usbTxReady = 1;
        }
      }
      else
      {
        //if next buffer not ready mark inactive
        usbTxReady = 1;
      }


      //Evaluate if our TX just make an RX buffer available and kick RX.
      if(usbRxReady && !*usbRxLen)
      {
        usbRxReady = 0;
        Block_Read(usbRxBuf, USB_BLOCK_SIZE, usbRxLen);
      }


   }*/

}

int32_t tst34 = 0;

//-----------------------------------------------------------------------------
// MAIN Routine
//-----------------------------------------------------------------------------
void main(void)
{
   // Disable Watchdog timer --
	PCA0MD &= ~PCA0MD_WDTE__BMASK; // ProgrammableCounterArray 0 Mode, Watch Dog Timer Enable bitmask

   SYSCLK_Init();                     // Initialize system clock
   ResetSources_Init();
   PORT_Init();                       // Initialize crossbar and GPIO
   Timer2_Init(0);						// Init Timer2 to generate interrupts
   External_Devices_Init();
   SPI0_Init();

   IE_EA = 1;                          // Enable global interrupts
   blinkSay(1);

   // USBXpress Initialization
   USB_Init(&InitStruct);

   // Enable USBXpress API interrupts
   API_Callback_Enable(myAPICallback);

   blinkSay(2);

   setWatchDog(100, ADC_TEST_NOT_COMPLETE);
   ads1246Test();
   setWatchDog(0, NO_ERROR);

   blinkSay(3);

   while (1) {
	   switch (workstate) {
	   	   case INIT_SYSTEM:
				ChipSelect1_inv = 0;			// выбор микросхемы 1
				ads1246Reset();
				// работаем с Мсх 2
				// Посылаем в АЦП команду SDATAC - прекращение непрерывного считывания данных
				// для считывания значения регистра
				//	bit cycle

				while (Data_RDY1_inv) {}; // ждем появления Data_RDY1 = 0

				// Посылаем в АЦП команду SDATAC - прекращение непрерывного считывания данных
				// для считывания значения регистра

				SPISendReceiveByte(ADS1246_COMMAND_SWITCH_TO_SDATAC);
				while (Data_RDY1_inv) {};
				ads1246WriteRegister(ADS1246_REGISTER_MUX1, 0x00); // Пишем в регистр MUX 00 - просто измерение
				workstate = DEFAULT_READ;
				break;

	   	   case DEFAULT_READ:
	   		   // Посылаем в АЦП команду RDATAC - непрерывное считывание данных
	   		   SPISendReceiveByte(ADS1246_COMMAND_SWITCH_TO_RDATAC);
	   		   while (Data_RDY1_inv) {};

			   while ( workstate == DEFAULT_READ ) // continuously read data
			   {
				    int32_t dataReaded;
					while (Data_RDY1_inv) {};
					dataReaded = (int8_t)SPISendReceiveByte(COMMAND_NOP);
					dataReaded *= 256;
					dataReaded += SPISendReceiveByte(COMMAND_NOP);
					dataReaded *= 256;
					dataReaded += SPISendReceiveByte(COMMAND_NOP);
					dataSet.dataReaded = dataReaded;
					dataSet.dataAge = (dataSet.dataAge == DATA_OLD ? DATA_NEW : DATA_LOST);
					//			TMR2RL= dataSet.dataReaded[0] * 256 + dataSet.dataReaded[1];  ;
					writeDataToHost();
					if (!btnCalibrate_inv) {
						btnCalibrate_inv = 1;
						ads1246Calibrate();
					}
				};
			   break;
/*
	   	   case SET_SPEED:
//				SPISendReceiveByte(ADS1246_COMMAND_SWITCH_TO_SDATAC);
//				while (Data_RDY1_inv) {};
				ads1246WriteRegister(ADS1246_REGISTER_MUX1, 0x00); // Пишем в регистр MUX 00 - просто измерение

				if(cmdSet.dataReaded > 9) {cmdSet.dataReaded = 9;};
				writeDataToHost();
				workstate = DEFAULT_READ;
				blinkSay(5);
				break;*/
	   	   default: // something strange happened
	   		workstate = INIT_SYSTEM;
	   }

   };                          // Spin forever
}

//-----------------------------------------------------------------------------
// SYSCLK_Init
//-----------------------------------------------------------------------------
//
// Return Value: none
// Parameters:   none
//
// This routine initializes the system clock to use the internal 12MHz / 1
// oscillator as its clock source.  Also enables missing clock detector reset.
//
//-----------------------------------------------------------------------------
void SYSCLK_Init (void)
{
   OSCICN  |= OSCICN_IOSCEN__ENABLED
			  | OSCICN_IFCN__HFOSC_DIV_1;        // Select full speed HFOSC

     CLKMUL = 0x00;                                //clear multiplier
     CLKMUL |= CLKMUL_MULEN__ENABLED;              //enable multiplier
     delay_100ms(); //!!!TODO too long, 5 microseconds enough
     CLKMUL |= CLKMUL_MULINIT__SET;                //Initialize multiplier
     delay_100ms(); //!!!TODO too long, 5 microseconds enough

     while(!(CLKMUL & CLKMUL_MULRDY__BMASK));      // Wait for multiplier to lock

     CLKSEL = CLKSEL_USBCLK__CLKMUL | CLKSEL_CLKSL__CLKMUL_DIV_2;
}

//-----------------------------------------------------------------------------
// SPI0_Init
//-----------------------------------------------------------------------------
//
// Return Value: none
// Parameters:   none
//
// This routine initializes SPI to work as Master in 4 Wire single master mode, with NSS byte high
// and data centered on second edge of SCK. SPI clock divider set
//
//-----------------------------------------------------------------------------
// Peripheral specific initialization functions,
// Called from the Init_Device() function
void SPI0_Init()
{
    SPI0CFG   = SPI0CFG_CKPHA__DATA_CENTERED_SECOND | SPI0CFG_MSTEN__MASTER_ENABLED;

    SPI0CN    = SPI0CN_NSSMD__3_WIRE;
    SPI0CKR   = (SYSCLK/(2*SPI_CLOCK))-1;

    IE_ESPI0 = 1; // allow interrupts from SPI0
    SPI0CN_SPIEN = 1; // enable SPI
}

//-----------------------------------------------------------------------------
// PORT_Init
//-----------------------------------------------------------------------------
//
// Return Value: none
// Parameters:   none
//
// Configure the Crossbar and GPIO ports.
//
// P2.2 - LED (push-pull)
//
// All other port pins unused
//
//-----------------------------------------------------------------------------
void PORT_Init (void)
{
// Port init order:
	//Step 1. Select the input mode (analog or digital) for all Port pins, using the Port Input Mode
	//register (PnMDIN).
	//Step 2. Select the output mode (open-drain or push-pull) for all Port pins, using the Port Output
	//Mode register (PnMDOUT).
	//Step 3. Select any pins to be skipped by the I/O Crossbar using the Port Skip registers (PnSKIP).
	//Step 4. Assign Port pins to desired peripherals (XBR0, XBR1).
	//Step 5. Enable the Crossbar (XBARE = ‘1’).

	P0MDOUT |= 0x0D;                     // SCK, MOSI, NSS push-pull, MISO - ????
	P1MDOUT |= 0x34; //Enable CircuitSelect1 on P1.2, Start P1.3, CircuitSelect2 on P1.4, Reset_inv P1.5
	P2MDOUT |= 0x0C;                    // Enable LED on P2.2 as a push-pull output, and P2.3 for debug purposes

	XBR0 = XBR0_SPI0E__ENABLED;                        // SPI I/O routed to port pin
   // Enable crossbar and weak pull-ups
   XBR1 = XBR1_WEAKPUD__PULL_UPS_ENABLED | XBR1_XBARE__ENABLED;
}

void ResetSources_Init()
{
	// do not play with it for now
//    RSTSRC = 0x04;                      // Enable missing clock detector

	//  VDM0CN = VDM0CN_VDMEN__ENABLED;            // Enable VDD Monitor
	  //Delay ();                                  // Wait for VDD Monitor to stabilize
	  //RSTSRC = RSTSRC_PORSF__SET;                // Enable VDD Monitor as a reset source

}

//-----------------------------------------------------------------------------
// Timer2_Init
//-----------------------------------------------------------------------------
//
// Return Value: none
// Parameters:   none
//
// Configure Timer2 to 16-bit auto-reload and generate an interrupt at
// interval specified by <counts> using SYSCLK/12 as its time base.
//
//-----------------------------------------------------------------------------
void Timer2_Init (int counts)
{
   TMR2CN = 0x00;                      // Stop Timer2; Clear TF2;
                                       // use SYSCLK/12 as timebase
   CKCON  &= ~(CKCON_T2MH__BMASK |
               CKCON_T2ML__BMASK );       // Timer2 clocked based on TMR2CN_T2XCLK;

   TMR2RL = -counts;                   // Init reload values
   TMR2 = 0xffff;                      // Set to reload immediately
   IE_ET2 = 1;                         // Enable Timer2 interrupts
   TMR2CN_TR2 = 1;                     // Start Timer2
}


void External_Devices_Init ()
{
	Start = 0;
	delay_100ms();
	if (Start) {error(ADC_START_PIN_CANNOT_BE_LOWERED);}
}
//-----------------------------------------------------------------------------
// Interrupt Service Routines
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Timer2_ISR
//-----------------------------------------------------------------------------
//
// This routine changes the state of the LED whenever Timer2 overflows.
//
//-----------------------------------------------------------------------------

INTERRUPT(Timer2_ISR, TIMER2_IRQn)
{
	blinkDo();
	checkWatchDog();
	blink2Do();
	TMR2CN_TF2H = 0;                    // Clear Timer2 interrupt flag
}

INTERRUPT(SPI0_ISR, SPI0_IRQn)
{
	if (SPI0CN_SPIF) {
		byteSPIDataReaded = SPI0DAT;
		flagSPIDataReaded = 1;
		SPI0CN_SPIF = 0;
	}
}


uint8_t delayerOne;
uint8_t delayerTwo;
void delay_100ms()
{
	for(delayerOne = 0;delayerOne++ < 255;)
	{
		for(delayerTwo = 0;delayerTwo++ < 255;) {}
	}
}

//-----------------------------------------------------------------------------
// End Of File
//-----------------------------------------------------------------------------
