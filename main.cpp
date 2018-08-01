/*
 * ch4_trail2.cpp
 *
 * Created: 24-06-2018 19:22:06
 * Author : Surya Teja
 */ 

#include <avr/io.h>
#define F_CPU 16000000
#include <util/delay.h>
#include <stdio.h>

#define SoftReset 0x00	//Software Reset
#define SysStatus 0x01 //System Status
#define FuncEn 0x02 //Function Enable
#define SagTh 0x03 //Voltage Sag Threshold ----further can change according to usage
#define SmallPMod 0x04 //Small-Power Mode
#define LastData 0x06 //Last Read/Write SPI/UART Value
#define LSB 0x08 //RMS/Power 16-bit LSB
#define CalStart 0x20 //Calibration Start Command
#define PLconstH 0x21 //High Word of PL_Constant
#define PLconstL 0x22 //Low Word of PL_Constant
#define Lgain 0x23 //L Line Calibration Gain
#define Lphi 0x24 //L Line Calibration Angle
#define Ngain 0x25 //N Line Calibration Gain
#define Nphi 0x26 //N Line Calibration Angle
#define PStartTh 0x27 //Active Startup Power Threshold
#define PNolTh 0x28 //Active No-Load Power Threshold
#define QStartTh 0x29 //Reactive Startup Power Threshold
#define QNolTh 0x2A //Reactive No-Load Power Threshold
#define MMode 0x2B //Metering Mode Configuration
#define CSOne 0x2C //Checksum 1
#define AdjStart 0x30 //Measurement Calibration Start Command
#define Ugain 0x31 //Voltage rms Gain
#define IgainL 0x32 //L Line Current rms Gain
#define IgainN 0x33 //N Line Current rms Gain
#define Uoffset 0x34 //Voltage Offset
#define IoffsetL 0x35 //L Line Current Offset
#define IoffsetN 0x36 //N Line Current Offset
#define PoffsetL 0x37 //L Line Active Power Offset
#define QoffsetL 0x38 //L Line Reactive Power Offset
#define PoffsetN 0x39 //N Line Active Power Offset
#define QoffsetN 0x3A //N Line Reactive Power Offset
#define CSTwo 0x3B //Checksum 2
#define APenergy 0x40 //Forward Active Energy
#define ANenergy 0x41 //Reverse Active Energy
#define ATenergy 0x42 //Absolute Active Energy
#define RPenergy 0x43 //Forward (Inductive) Reactive Energy
#define Rnenerg 0x44 //Reverse (Capacitive) Reactive Energy
#define Rtenergy 0x45 //Absolute Reactive Energy
#define EnStatus 0x46 //Metering Status
#define Irms 0x48 //L Line Current rms
#define Urms 0x49 //Voltage rms
#define Pmean 0x4A //L Line Mean Active Power
#define Qmean 0x4B //L Line Mean Reactive Power
#define Freq 0x4C //Voltage Frequency
#define PowerF 0x4D //L Line Power Factor
#define Pangle 0x4E //Phase Angle between Voltage and L Line Current
#define Smean 0x4F //L Line Mean Apparent Power
#define IrmsTwo 0x68 //N Line Current rms
#define PmeanTwo 0x6A //N Line Mean Active Power
#define QmeanTwo 0x6B //N Line Mean Reactive Power
#define PowerFTwo 0x6D //N Line Power Factor
#define PangleTwo 0x6E //Phase Angle between Voltage and N Line Current
#define SmeanTwo 0x6F //N Line Mean Apparent Power
const int energy_CS = 4;
void spi_init()
{
	DDRB=(1<<PB5)|(1<<PB7); // Set MOSI and SCK pins as outputs
  SPCR = 0x5B; //SPE=1, MSTR=1, CPOL=1, CPHA=1, freq=f/128
  	   	 	   //spi enable, master mode, clock idle high, phase low
  SPSR = 0x00; //status register
} 
unsigned char spi_send(unsigned char data)
{
	// Start transmission
	SPDR = data;
	
	// Wait for transmission complete
	while(!(SPSR & (1<<SPIF)));
	return(SPDR);
}

class ATM90E26
{
	public:
	ATM90E26(int pin=energy_CS);
	double  GetLineVoltage();
	double GetLineCurrent();
	double GetActivePower();
	double GetReactivePower();
	double GetFrequency();
	double GetPowerFactor();
	double GetImportEnergy();
	double GetExportEnergy();
	void SetUGain(unsigned short);
	void SetLGain(unsigned short);
	void SetIGain(unsigned short);
	void SetCRC1(unsigned short);
	void SetCRC2(unsigned short);
	void InitEnergyIC();
	unsigned short GetSysStatus();
	unsigned short  GetMeterStatus();
	private:
	unsigned short CommEnergyIC(unsigned char RW,unsigned char address, unsigned short val);
	int _cs;
	unsigned short _lgain;
	unsigned short _ugain;
	unsigned short _igain;
	unsigned short _crc1;
	unsigned short _crc2;
};

ATM90E26::ATM90E26(int pin)
{
	_cs = pin;
	_lgain = 0x1D39;
	_ugain = 0xD464;
	_igain = 0x6E49;
	_crc1 = 0x4A34;
	_crc2 = 0xD294;
}
void ATM90E26::SetLGain(unsigned short lgain) { _lgain = lgain;}
void ATM90E26::SetUGain(unsigned short ugain) { _ugain = ugain ;}
void ATM90E26::SetIGain(unsigned short igain) { _igain = igain ;}
void ATM90E26::SetCRC1(unsigned short crc1) { _crc1 = crc1;}
void ATM90E26::SetCRC2(unsigned short crc2) { _crc2 = crc2;}


unsigned short ATM90E26::CommEnergyIC(unsigned char RW,unsigned char address, unsigned short val)
{
	unsigned char* data=(unsigned char*)&val;
	unsigned short output;
	output=(val>>8)|(val<<8);//switch MSB and LSB of value
	val=output;
	address|=RW<<7;//Set read write flag---from data sheet section:4.1.1 read and write sequence plots
	PORTB &= ~(1 << energy_CS);
	_delay_us(10);
	spi_init();
	spi_send(address);
	// Must wait 4 us for data to become valid 
	_delay_us(4);
	 int i;
	if(RW)
	{
		
		for (i=0; i<2; i++)
		{
			*data = spi_send(0x00);
			data++;
		}
	}
	else
	{
		for (i=0; i<2; i++)
		{
			spi_send(*data);             // write all the bytes
			data++;
		}
	}
	PORTB |= (1 << energy_CS);
	_delay_us(10);
	output=(val>>8)|(val<<8); //reverse MSB and LSB
	return output;
}
	double  ATM90E26::GetLineVoltage(){
		unsigned short voltage=CommEnergyIC(1,Urms,0xFFFF);
		return (double)voltage/100;
	}

	unsigned short ATM90E26::GetMeterStatus(){
		return CommEnergyIC(1,EnStatus,0xFFFF);
	}

	double ATM90E26::GetLineCurrent(){
		unsigned short current=CommEnergyIC(1,Irms,0xFFFF);
		return (double)current/1000;
	}

	double ATM90E26::GetActivePower(){
		short int apower= (short int)CommEnergyIC(1,Pmean,0xFFFF); //Complement, MSB is signed bit
		return (double)apower;
	}
	double ATM90E26::GetReactivePower(){
		short int rpower= (short int)CommEnergyIC(1,Qmean,0xFFFF); //Complement, MSB is signed bit
		return (double)rpower;
	}

	double ATM90E26::GetFrequency(){
		unsigned short freq=CommEnergyIC(1,Freq,0xFFFF);
		return (double)freq/100;
	}

	double ATM90E26::GetPowerFactor(){
		short int pf= (short int)CommEnergyIC(1,PowerF,0xFFFF); //MSB is signed bit
		//if negative
		if(pf&0x8000){
			pf=(pf&0x7FFF)*-1;
		}
		return (double)pf/1000;
	}

	double ATM90E26::GetImportEnergy(){
		//Register is cleared after reading
		unsigned short ienergy=CommEnergyIC(1,APenergy,0xFFFF);
		return (double)ienergy*0.0001; //returns kWh if PL constant set to 1000imp/kWh
	}

	double ATM90E26::GetExportEnergy(){
		//Register is cleared after reading
		unsigned short eenergy=CommEnergyIC(1,ANenergy,0xFFFF);
		return (double)eenergy*0.0001; //returns kWh if PL constant set to 1000imp/kWh
	}

	unsigned short ATM90E26::GetSysStatus(){
		return CommEnergyIC(1,SysStatus,0xFFFF);
	}


	void ATM90E26::InitEnergyIC(){
		unsigned short systemstatus;
		
		DDRB =(1<<_cs);
		DDRD =(1<<PD0);
		DDRD =(1<<PD1);
		

		
		CommEnergyIC(0,SoftReset,0x789A); //Perform soft reset
		CommEnergyIC(0,FuncEn,0x0030); //Voltage sag irq=1, report on warnout pin=1, energy dir change irq=0
		CommEnergyIC(0,SagTh,0x1F2F); //Voltage sag threshold
		

		//Set metering calibration values
		CommEnergyIC(0,CalStart,0x5678); //Metering calibration startup command. Register 21 to 2B need to be set
		CommEnergyIC(0,PLconstH,0x00B9); //PL Constant MSB
		CommEnergyIC(0,PLconstL,0xC1F3); //PL Constant LSB
		CommEnergyIC(0,Lgain,_lgain);   //Line calibration gain
		CommEnergyIC(0,Lphi,0x0000); //Line calibration angle
		CommEnergyIC(0,PStartTh,0x08BD); //Active Startup Power Threshold
		CommEnergyIC(0,PNolTh,0x0000); //Active No-Load Power Threshold
		CommEnergyIC(0,QStartTh,0x0AEC); //Reactive Startup Power Threshold
		CommEnergyIC(0,QNolTh,0x0000); //Reactive No-Load Power Threshold
		CommEnergyIC(0,MMode,0x9422); //Metering Mode Configuration. All defaults. refer from pg 31 of datasheet.
		CommEnergyIC(0,CSOne,_crc1); //Write CSOne, as self calculated
		

		//Set measurement calibration values
		CommEnergyIC(0,AdjStart,0x5678); //Measurement calibration startup command, registers 31-3A
		CommEnergyIC(0,Ugain,_ugain);    //Voltage rms gain
		CommEnergyIC(0,IgainL,_igain);   //L line current gain
		CommEnergyIC(0,Uoffset,0x0000);  //Voltage offset
		CommEnergyIC(0,IoffsetL,0x0000); //L line current offset
		CommEnergyIC(0,PoffsetL,0x0000); //L line active power offset
		CommEnergyIC(0,QoffsetL,0x0000); //L line reactive power offset
		CommEnergyIC(0,CSTwo,_crc2); //Write CSTwo, as self calculated
		
		CommEnergyIC(0,CalStart,0x8765); //Checks correctness of 21-2B registers and starts normal metering if ok
		CommEnergyIC(0,AdjStart,0x8765); //Checks correctness of 31-3A registers and starts normal measurement  if ok

		systemstatus = GetSysStatus();
		
		if (systemstatus&0xC000){
			//checksum 1 error
			PORTD =(1<<PD0);//PD0 light will be ON when checksum1 error occur
		}
		if (systemstatus&0x3000){
			//checksum 2 error
			PORTD=(1<<PD1);//PD1 light will be ON when checksum2 error occurs
		}
	}


int main(void)
{
	ATM90E26 m;
	DDRD =(0<<PD2);//input from ATM90E26(pin 17) warn out pin for voltage sag detection
	DDRD =(1<<PD4);//led output signal for voltage sag detection
	m.InitEnergyIC(); //calibration begins
	
    while (1)
    {
		if(!(PINC & (1<<PC3))) //checking whether voltage sag is detected
		{
			printf("Sys Status:\n");
			m.GetSysStatus();          //return system status
			printf("Meter Status:\n");
			m.GetMeterStatus();			//return meter status
			printf("Voltage:\n");
			m.GetLineVoltage();			// voltage
			printf("Current:\n");
			m.GetLineCurrent();			// current
			printf("Active power:\n");
			m.GetActivePower();         //active power
			printf("Reactive power:\n"); //reactive power
			m.GetReactivePower();
			printf("p.f.:\n");			//power factor
			m.GetPowerFactor();
			printf("Energy consumed by EV station:\n");
			m.GetImportEnergy();		//total energy consumed by Ev station
			printf("Energy consumed by EV vehicle:\n");
			m.GetExportEnergy();		//total energy consumed by EV vehicle by charging
			_delay_ms(500);
		}
	else
	{
		PORTD=(1<<PD4);       //warning light for voltage sag detection 
	}
	}
}


