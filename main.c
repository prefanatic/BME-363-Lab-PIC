
/*********************************************************************************************/
/* BME 361 Biomeasurement Lab - PIC18F4525 Demo                            		       		 */
/* Laboratories 1-8: A/D, D/A, LCD display, ECG simulation, filters, QRS detection		 	 */
/* Instructors: John DiCecco, Ying Sun         						                         */
/* Latest update: 6/06/2015																	 */
/*********************************************************************************************/

/**************************** Specify the chip that we are using *****************************/ 
//#include <p18cxxx.h> // We don't need this - MPLAB injects it depending on the type of PIC we have set up.
#include <timers.h>
#include <math.h>
#include <stdlib.h>
#include <delays.h>

/************************* Configure the Microcontroller PIC18f4525 **************************/
#pragma config OSC = XT
#pragma config WDT = OFF
#pragma config PWRT = OFF
#pragma config FCMEN = OFF
#pragma config IESO = OFF
#pragma config BOREN = ON
#pragma config BORV = 2
#pragma config WDTPS = 128
#pragma config PBADEN = OFF
#pragma config DEBUG = OFF
#pragma config LVP = OFF
#pragma config STVREN = OFF
#define _XTAL_FREQ 4000000

/******************************** Define Prototype Functions *********************************/
unsigned char ReadADC();
void rx_int (void);
void _highPriorityInt(void);
void Backlight(unsigned char state);
void SetPosition(unsigned char position);
void PrintLine(rom unsigned char *string, unsigned char numChars);
void PrintInt(int value, unsigned char position);
void PrintNum(unsigned char value1, unsigned char position1);
void SetupSerial();
void ClearScreen();
void SetupADC(unsigned char channel);
void Delay_ms(unsigned int x);
void Transmit(unsigned char value);
void TransmitGraphData(unsigned char tag, unsigned char value);
void TransmitFunction();

/************************************** Global variables *************************************/
unsigned char function, mode, update, debounce0, debounce1, LEDcount, output, counter, i, j;
unsigned char data0, data1, data2, array[9], rank[9], do_median, do_MOBD, refractory, display;
unsigned char temp, sampling[16], TMRcntH[16], TMRcntL[16], sampling_H, sampling_L;
unsigned char bluetoothEnabled, needToTransferOriginal, needToTransferModified; // BLUETOOTH
int dummy, d0, d1, d2, mobd, threshold, rri_count, hr;

unsigned char ReadADC() { /************* start A/D, read from an A/D channel *****************/
	unsigned char ADC_VALUE;
	ADCON0bits.GO = 1;				// Start the AD conversion
	while(!PIR1bits.ADIF) continue;	// Wait until AD conversion is complete
	ADC_VALUE = ADRESH;				// Return the highest 8 bits of the 10-bit AD conversion
	return ADC_VALUE;
}

#pragma code _highPriorityInt=0x8
void rx_int (void){ /****************** Set up highpriority interrup vector ******************/
		_asm goto _highPriorityInt _endasm
}

#pragma interrupt _highPriorityInt
void _highPriorityInt(void) { /********** high priority interrupt service routine ************/
checkflags:
	if (INTCONbits.TMR0IF == 1) {	// When there is a timer0 overflow, this loop runs
		INTCONbits.TMR0IE = 0;		// Disable TMR0 interrup
		INTCONbits.TMR0IF = 0;		// Reset timer 0 interrupt flag to 0
		if (debounce0 != 0) debounce0--;	// switch debounce delay counter for INT0
		if (debounce1 != 0) debounce1--;	// switch debounce delay counter for INT1
		TMR0H = 0xF0;			// Reload TMR0 for 4.167 ms count, Sampling rate = 240 Hz
		TMR0L = 0x7C;			// 0xFFFF-0xEFB8 = 0x1047 = 4167, adjust for delay by 68 us
		switch (function) {
		case 1:					// Function 1: ECG simulation
			TMR0H = 0xFC;		// Reload TMR0 for 1 ms count, sampling rate = 1KHz
			TMR0L = 0xFA;		// 0xFFFF-0xFC17 = 0x3E8 = 1000,adust for delay by 227 us
			PORTBbits.RB4 = !PORTBbits.RB4;		// Toggle RB4 (pin 37) for frequency check
			switch (mode) {
			case 0:													// P wave up
				counter++;		output++;		if (counter == 30) mode++;
				break;
			case 1:													// P wave flat
				counter--;		if (counter == 0) mode++;
				break;
			case 2:													// P wave down
				counter++;		output--;		if (counter == 30) mode++;
				break;
			case 3:													// PR segment
				counter++;		if (counter == 100) mode++;
				break;
			case 4:													// QRS complex - Q
				counter++;		output -= 3;
				if (counter == 105){
					counter = 0;
					mode++;
				}
				break;
			case 5:													// QRS complex - R up
				counter++;		output += 6;	if (counter == 30) mode++;
				break;
			case 6:													// QRS complex - R down
				counter++;		output -= 6;	if (counter == 62) mode++;
				break;		
			case 7:													//QRS complex - S
				counter++;		output += 3;
				if (counter == 71) {
					mode++;
					counter = 0;
				}
				break;
			case 8:													// ST segment
				counter++;
				if (counter == 89){
					counter = 0;
					mode++;
				}
				break;
			case 9:													// T wave up
				counter++;		output++;		if (counter == 55)mode++;
				break;
			case 10:												// T wave flat
				counter++;		if (counter == 110) mode++;
				break;
			case 11:												// T wave down
				counter++;		output--;		if (counter == 165) mode++;
				break;
			case 12:												// End ECG
				counter--;		if (counter == 0) mode++;
				break;
			case 13:												// Reset ECG
				counter++;
				if (counter == 202){
					counter = mode = 0;
					output = 50;
				}
				break;
			}
            needToTransferOriginal = 1;
			PORTD = output;
			break;
		case 2:						// Function 2: Echo
			data0 = ReadADC();		// Read A/D and save the present sample in data0
            output = data0;
            needToTransferOriginal = 1;
			PORTD = data0;			// Echo back 
			PORTBbits.RB4 = !PORTBbits.RB4;		// Toggle RB4 (pin 37) for frequency check
			break;
		case 3:						// Function 3: Echo (vary rate)
			TMR0H = sampling_H;		// Reload TMR0 high-order byte
			TMR0L = sampling_L;		// Reload TMR0 low-order byte
			data0 = ReadADC();		// Read A/D and save the present sample in data0
            output = data0;
            needToTransferOriginal = 1;
			PORTD = data0;			// Echo back 
			PORTBbits.RB4 = !PORTBbits.RB4;		// Toggle RB4 (pin 37) for frequency check
			break;
		case 4:						// Function 4: Derivative
			data1 = data0;			// Store previous data points
			data0 = ReadADC();		// Read A/D and save the present sample in data0
			dummy = (int)data0 - data1 + 128;	// Take derivative and shift to middle
            output = data0;
            needToTransferOriginal = 1;
            needToTransferModified = 1;
			PORTD = (unsigned char)dummy;		// Output to D/A
			break;
		case 5:						// Function 5: Low-pass filter
			data2 = data1;			// Store previous data points
			data1 = data0;
			data0 = ReadADC();		// Read A/D and save the present sample in data0
			dummy = ((int)data0 + data1 + data1 + data2) / 4;	// smoother
            output = data0;
            needToTransferOriginal = 1;
            needToTransferModified = 1;
			PORTD = (unsigned char)dummy;		// Output to D/A
			break;
		case 6:						// Function 6: High-frequency enhancement filter
			data2 = data1;			// Store previous data points
			data1 = data0;
			data0 = ReadADC();		// Read A/D and save the present sample in data0
			dummy = ((int)data0 + data1 + data1 + data2) / 4;	// smoother
			dummy = data0 + data0 - dummy;
            output = data0;
            needToTransferOriginal = 1;
            needToTransferModified = 1;
			PORTD = (unsigned char)dummy;		// Output to D/A
			break;
		case 7:						// Function 7: 60Hz notch filter
			data2 = data1;			// Store previous data points
			data1 = data0;
			data0 = ReadADC();		// Read A/D and save the present sample in data0
			dummy = ((int)data0 + data2) / 2;	// 60 Hz notch       
            output = data0;
            needToTransferOriginal = 1;
            needToTransferModified = 1;
			PORTD = (unsigned char)dummy;		// Output to D/A
			break;
		case 8:						// Function 8: Median filter
			data0 = ReadADC();		// Read A/D and save the present sample in data0
            output = data0;
            needToTransferOriginal = 1;
			do_median = 1;			// Flag main() to do median filter
			break;
		case 9:						// Function 9: Heart rate meter
			TMR0H = 0xED;			// Reload TMR0 for 5 ms count, sampling rate = 200 Hz
			TMR0L = 0x44;			// 0xFFFF-0xED44 = 0x12BB = 4795, (205 us delay)
			data1 = data0;			// Move old ECG sample to data1
			data0 =  ReadADC();		// Store new ECG sample from ADC to data0
            //TransmitGraphData(1, data0);
            output = data0;
            needToTransferOriginal = 1;
			PORTBbits.RB2 = !PORTBbits.RB2;	// Toggle RB2 for sampling rate check
			do_MOBD = 1;			// Flag main() to do MOBD for QRS detection
			break;
		}
		INTCONbits.TMR0IE = 1;		// Enable TMR0 interrupt
	}	
	if (INTCONbits.INT0IF == 1) {	// INT0 (pin 33) negative edge
		INTCONbits.INT0IE = 0;		// Disable interrupt
		INTCONbits.INT0IF = 0;		// Reset interrupt flag
		if (debounce0 == 0) {
			if (function <= 0) function = 9;	// Set function range 0-9
			else function--;
			if (function == 9) SetupADC(1);			// ECG comes from AN1 channel
			else SetupADC(0);					// Others come from AN0 channel
			update = 1;				// Signal main() to update LCD display
			debounce0 = 10;			// Set switch debounce delay counter decremented by TMR0
		}
		INTCONbits.INT0IE = 1;		// Enable interrupt
		goto checkflags;			// Check again in case there is a timer interrupt
	}
	if (INTCON3bits.INT1IF == 1) {	// INT1 (pin 34) negative edge
		INTCON3bits.INT1IE = 0;		// Disable interrupt
		INTCON3bits.INT1IF = 0;		// Reset interrupt flag
		if (debounce1 == 0) {
			if (function >= 9) function = 0;	// Set function range 0-9
			else function++;
			if (function == 9) SetupADC(1);			// ECG comes from AN1 channel
			else SetupADC(0);					// Others come from AN0 channel
			update = 1;				// Signal main() to update LCD display
			debounce1 = 10;			// Set switch debounce delay counter decremented by TMR0
		}
		INTCON3bits.INT1IE = 1;		// Enable interrupt
		goto checkflags;			// Check again in case there is a timer interrupt
	}
}

/**
 * TransmitGraphData
 * Sends a value over UART to with a tag to identify the type of graph.
 * @param tag Character value of the identifier.
 * @param value Character value of the raw data.
 */
void TransmitGraphData(unsigned char tag, unsigned char value) {
    if (bluetoothEnabled == 0) return;
    
    Transmit(tag);
    Transmit(value);
}

/**
 * TransmitFunction
 * Sends the current function over UART, with a specified tag of 0.
 */
void TransmitFunction() {
    if (bluetoothEnabled == 0) return;
    
    Transmit(0);
    Transmit(function);
}

/**
 * Transmit
 * Waits for UART to be ready, then pushes the specified character out.
 * This method will block until UART is ready again.
 * @param value Character value to be sent.
 */
void Transmit(unsigned char value) {  /********** Send an ASCII Character to USART ***********/
	while(!PIR1bits.TXIF) continue;		// Wait until USART is ready
	TXREG = value;						// Send the data
	while (!PIR1bits.TXIF) continue;	// Wait until USART is ready
    
    if (bluetoothEnabled == 0) {
        Delay_ms(5); // Give the LCD some time to listen to what we've got to say.
    }
}

/** ** 
    Start -- LCD Related Functions
 ** **/

/**
 * ClearScreen
 * Clears the connected LCD screen over UART.
 */
void ClearScreen(){   /************************** Clear LCD Screen ***************************/
    if (bluetoothEnabled == 1) return;
    
	Transmit(254);					// See datasheets for Serial LCD and HD44780
	Transmit(0x01);					// Available on our course webpage
}

/**
 * Backlight
 * Enables or disables the LCD backlight depending on the value of state(0,1)
 * @param state Either 1 or 0 to signal the LCD to turn on or off.
 */
void Backlight(unsigned char state){  /************* Turn LCD Backlight on/off ***************/
    if (bluetoothEnabled == 1) return;
    
	Transmit(124);
	if (state) Transmit(0x9D);		// If state == 1, backlight on
	else Transmit(0x81);			// otherwise, backlight off
}

/**
 * SetPosition
 * Sets the position of the character writer on the attached LCD.
 * @param position Character value of the position.
 */
void SetPosition(unsigned char position){ 	/********** Set LCD Cursor Position  *************/
    if (bluetoothEnabled == 1) return;
    
	Transmit(254);
	Transmit(128 + position);
}

/**
 * PrintLine
 * Prints the specified character string sent.
 * @param string
 * @param numChars
 */
void PrintLine(rom unsigned char *string, unsigned char numChars){ /**** Print characters ****/
    	unsigned char count;

        
    if (bluetoothEnabled == 1) return;
   
	for (count=0; count<numChars; count++) Transmit(string[count]);
}

void PrintInt(int value, unsigned char position){ /******** Print number at position *********/
    	int units, tens, hundreds, thousands;

    if (bluetoothEnabled == 1) return;
    
    
	SetPosition(position);				// Set at the present position
	if (value > 9999) {
		PrintLine((rom unsigned char*)"Over", 4);
		return;
	}
	if (value < -9999) {
		PrintLine((rom unsigned char*)"Under", 5);
		return;
	}
	if (value < 0) {
		value = -value;
		Transmit(45);
	}
	else Transmit(43);
	thousands = value / 1000;		// Get the thousands digit, convert to ASCII and send
	if (thousands != 0) Transmit(thousands + 48);
	value = value - thousands * 1000;
	hundreds = value / 100;			// Get the hundreds digit, convert to ASCII and send
	Transmit(hundreds + 48);
	value = value - hundreds * 100;
	tens = value / 10;				// Get the tens digit, convert to ASCII and send
	Transmit(tens + 48);
	units = value - tens * 10;
	Transmit(units + 48);			// Convert to ASCII and send
}

void PrintNum(unsigned char value1, unsigned char position1){ /** Print number at position ***/
    	int units, tens, hundreds, thousands;

    if (bluetoothEnabled == 1) return;
    
    
	SetPosition(position1);			// Set at the present position
	hundreds = value1 / 100;		// Get the hundreds digit, convert to ASCII and send
	if (hundreds != 0) Transmit(hundreds + 48);
	else Transmit(20);
	value1 = value1 - hundreds * 100;
	tens = value1 / 10;				// Get the tens digit, convert to ASCII and send
	Transmit(tens + 48);
	units = value1 - tens * 10;
	Transmit(units + 48);			// Convert to ASCII and send
}

/** ** 
    End -- LCD Related Functions
 ** **/

void SetupBluetooth() {
    if (bluetoothEnabled == 0) return;
    
    SPBRG = 1;  // Push up to 115200 BAUD to configure the BL module.
    
    Delay_ms(500);
    
    // Enter command mode.
    Transmit("$");
    Transmit("$");
    Transmit("$");
    
    // Wait for the BL module to respond.
    Delay_ms(100);
    
    // Tell it that we want 9600 BAUD.
    PrintLine((rom unsigned char*)"U,9600,N", 8);
    
    Delay_ms(100);
    
    SPBRG = 25;  // Swap back to 9600 BAUD for UART.
}

// For LCD - use SPBRG = 25;  9600 BAUD at 4MHz: 4,000,000/(16x9600) - 1 = 25.04
// For Bluetooth - use SPBRG = 1; 115200 BAUD at 4MHz: 4,000,000/(16x115200) - 1 = 1

void SetupSerial(){  /*********** Set up the USART Asynchronous Transmit (pin 25) ************/
	TRISC = 0x80;					// Transmit and receive, 0xC0 if transmit only
	SPBRG = 25;						
	TXSTAbits.TXEN = 1;				// Transmit enable
	TXSTAbits.SYNC = 0;				// Asynchronous mode
	RCSTAbits.CREN = 1;				// Continuous receive (receiver enabled)
	RCSTAbits.SPEN = 1;				// Serial Port Enable
	TXSTAbits.BRGH = 1;				// High speed baud rate
}

void SetupADC(unsigned char channel){ 	/******** Configure A/D and Set the Channel **********/
	TRISA = 0b11111111;					// Set all of Port A as input
	// ADCON2 Setup
	// bit 7: Left justify result of AD (Lowest 6bits of ADRESL are 0's) (0)
	// bit 6: Unimplemented
	// bit 5-3: AD aquisition time set to 2 TAD (001)
	// bit 2-0: Conversion clock set to Fosc/8 (001)
	ADCON2 = 0b00001001;
	// ADCON1 Setup
	// bit 7-6: Unimplemented
	// bit 5: Vref - (VSS) (0)
	// bit 4: Vref + (VDD) (0)
	// bit 3-0: Configuration of AD ports (Set A0-A4, others to digital, including the PORTB)
	ADCON1	= 0b00001010;
	// ADCON0 Setup
	// bit 7,6 = Unimplemented
	// bits 5-2 = Channel select
	// bit 1: GO Bit (Starts Conversion when = 1)
	// bit 0: AD Power On
	ADCON0 = (channel << 2) + 0b00000001;
	PIE1bits.ADIE = 0;		// Turn off the AD interrupt
	PIR1bits.ADIF = 0;		// Reset the AD interrupt flag
}

void Delay_ms(unsigned int x){ 	/****** Generate a delay for x ms, assuming 4 MHz clock ******/
unsigned char y;
	for(;x > 0; x--) for(y=0; y< 82;y++);
}

void main(){   /****************************** Main program **********************************/
    bluetoothEnabled = 1; // BLUETOOTH
	function = mode = LEDcount = counter = debounce0 = debounce1 = 0;	// Initialize
	display = do_median = do_MOBD = rri_count = 0;
	threshold = 50;				// Threshold for the MOBD QRS-detection algorithm
	update = 1;					// Flag to signal LCD update
	output = 50;				// Baseline for ECG simulation
    sampling[0] = 16;	sampling[1] = 17;	sampling[2] = 18;	sampling[3] = 19;
    sampling[4] = 20;	sampling[5] = 25;	sampling[6] = 30;	sampling[7] = 50;
    sampling[8] = 70;	sampling[9] = 88;	sampling[10] = 108;	sampling[11] = 126;
    sampling[12] = 145;	sampling[13] = 173;	sampling[14] = 192;	sampling[15] = 228;
	TMRcntH[0] = 11;	TMRcntH[1] = 26;	TMRcntH[2] = 38;	TMRcntH[3] = 50;
	TMRcntH[4] = 60;	TMRcntH[5] = 99;	TMRcntH[6] = 125;	TMRcntH[7] = 177;
	TMRcntH[8] = 200;	TMRcntH[9] = 212;	TMRcntH[10] = 220;	TMRcntH[11] = 225;
	TMRcntH[12] = 229;	TMRcntH[13] = 234;	TMRcntH[14] = 236;	TMRcntH[15] = 239;
	TMRcntL[0] = 219;	TMRcntL[1] = 55;	TMRcntL[2] = 251;	TMRcntL[3] = 103;
	TMRcntL[4] = 175;	TMRcntL[5] = 191;	TMRcntL[6] = 201;	TMRcntL[7] = 223;
	TMRcntL[8] = 49;	TMRcntL[9] = 151;	TMRcntL[10] = 124;	TMRcntL[11] = 242;
	TMRcntL[12] = 244;	TMRcntL[13] = 75;	TMRcntL[14] = 119;	TMRcntL[15] = 184;
	sampling_H = 0xF0;			// initialize for 4.167 ms count, Sampling rate = 240 Hz
	sampling_L = 0x7C;			// 0xFFFF-0xEFB8 = 0x1047 = 4167, adjust for delay by 68 us
	TRISB = 0b00000011;			// RB0-1 as inputs, others outputs, RB3 drives buzzer
	TRISD = 0b00000000;			// Set all port D pins as outputs
	PORTD = 0;					// Set port D to 0's
	SetupADC(0);				// Call SetupADC() to set up channel 0, AN0 (pin 2)
	SetupSerial();				// Set up USART Asynchronous Transmit for LCD display
    //SetupBluetooth();   
	T0CON = 0b10001000;			// Setup the timer control register for interrupt
	INTCON = 0b10110000;		// GIE(7) = TMR0IE = INT0IE = 1
	INTCONbits.TMR0IE = 1;		// Enable TMR0 interrupt
	INTCON2bits.INTEDG0 = 0;	// Set pin 33 (RB0/INT0) for negative edge trigger
	INTCON2bits.INTEDG1 = 0;	// Set pin 34 (RB1/INT1) for negative edge trigger
	INTCONbits.INT0IE = 1;		// Enable INT0 interrupt
	INTCON3bits.INT1IE = 1;		// Enable INT1 interrupt
	Delay_ms(3000);				// Wait until the LCD display is ready
	Backlight(1);				// turn LCD display backlight on
	ClearScreen();				// Clear screen and set cursor to first position	
	PrintLine((rom unsigned char*)"  BME 361 Demo", 14);
	SetPosition(64);			// Go to beginning of Line 2;
	PrintLine((rom unsigned char*)" Biomeasurement",15);	// Put your trademark here 
	Delay_ms(3000);
	ClearScreen();				// Clear screen and set cursor to first position
	PrintLine((rom unsigned char*)"Function", 8);
	while (1) {
        if (bluetoothEnabled) {
            if (needToTransferOriginal) {
                needToTransferOriginal = 0;
                TransmitGraphData(1, output);
            }
            if (needToTransferModified) {
                needToTransferModified = 0;
                TransmitGraphData(2, (unsigned char) dummy);
            }
        }
        
		if (update) {			// The update flag is set by INT0 or INT1
			update = 0;			// Reset update flag
            if (function != 0) LEDcount = PORTB = 0;	// Reset LED's
            
            if (bluetoothEnabled == 1) {
                TransmitFunction(); // Transmit our function across.
            } else {
                PrintNum(function, 8);	// Update the function number on LCD display
                SetPosition(64);	// Go to beginning of Line 2;
                switch (function) {
                    case 0: PrintLine((rom unsigned char*)"Binary counter  ",16); break; 
                    case 1: PrintLine((rom unsigned char*)"ECG simulation  ",16); break; 
                    case 2: PrintLine((rom unsigned char*)"Echo (A/D - D/A)",16); break; 
                    case 3: PrintLine((rom unsigned char*)"Echo @ fs     Hz",16); break; 
                    case 4: PrintLine((rom unsigned char*)"Derivative      ",16); break; 
                    case 5: PrintLine((rom unsigned char*)"Low-pass filter ",16); break; 
                    case 6: PrintLine((rom unsigned char*)"Hi-freq enhance ",16); break; 
                    case 7: PrintLine((rom unsigned char*)"60Hz notch filtr",16); break; 
                    case 8: PrintLine((rom unsigned char*)"Median filter   ",16); break; 
                    case 9: PrintLine((rom unsigned char*)"HR =       bpm  ",16); break; 
                }
            }
		}
		switch (function) {
		case 0:				// Function 0: Binary counter
			LEDcount++;						// Upcounter
			PORTB = LEDcount & 0b11110000;	// Mask out the lower 4 bits
			PORTD = LEDcount;				// Output ramp to verify linearity of the D/A
			Delay_ms(100);					// Delay to slow down the counting
			break;
		case 3:				// Function 3: Echo (vary rate)
			INTCONbits.TMR0IE = 0;			// Disable TMR0 interrupt
			SetupADC(2);					// Switch to A/D channel AN2
			counter = ReadADC();			// Read potentiometer setting from AN2
			SetupADC(0);					// Switch to A/D channel AN0
			counter = counter >> 4;			// Scale it to 0-15
			temp = sampling[counter];		// Display sampling rate
			PrintNum(temp,74);
			sampling_L = TMRcntL[counter];	// Load TMR0 low-order byte
			sampling_H = TMRcntH[counter];	// Load TMR0 high-order byte
			INTCONbits.TMR0IE = 1;			// Enable TMR0 interrupt
			Delay_ms(1000);					// Delay to slow down the counting
			break;
		case 8:				// Function 8: Median filter (9-point)
			if (do_median) {
				do_median = 0;				// Reset do_median flag
				INTCONbits.TMR0IE = 0;		// Disable TMR0 interrupt
				for (i=8; i>0; i--) array[i] = array[i-1];	// Store the previous 8 points
				array[0] = data0;			// Get new data point from A/D
				for (i=0; i<9; i++) rank[i] = array[i];	// Make a copy of data array
				for (i=0; i<5; i++) {		// Perform a bubble sort
					for (j=i+1; j<9; j++) {
						if (rank[i] < rank[j]) {
							temp = rank[i];		// Swap
							rank[i] = rank[j];
							rank[j] = temp;
						}
					}
				}
                
                TransmitGraphData(2, rank[4]);
				PORTD = rank[4];			// Median is at rank[4] of rank[0-8]
				INTCONbits.TMR0IE = 1;		// Enable TMR0 interrupt
			}
			break;
		case 9:				// Function 9: Multiplication of Backward Differences (MOBD)
			if (do_MOBD){					// MOBD = Multiplication of Backwards Differences
				INTCONbits.TMR0IE = 0;		// Disable TMR0 interrupt
				do_MOBD = 0;				// Reset new_data flag
				d2 = d1;					// Move oldest difference to d2
				d1 = d0;					// Move older difference to d1
				d0 = (int)data0 - data1;	// Store new difference in d0, (int) casting important
				rri_count++;				// Increment RR-interval
				mobd = 0;					// mobd = 0, unless sign consistency is met:
				if (d0 > 0 && d1 > 0 && d2 > 0){	// (1) If 3 consecutive positive differences
					mobd = d0 * d1;			// Multiply first two differences
					mobd = mobd >> 3;		// Scale down (divide by 8)
					mobd = mobd * d2;		// Multiply the oldest difference
				}
				if (d0 < 0 && d1 < 0 && d2 < 0){	// (2) If 3 consecutive negative differences
					d0 = -d0;				// Take absolute value of differences
					d1 = -d1;
					d2 = -d2;
					mobd = d0 * d1;			// Multiply first two differences
					mobd = mobd >> 3;		// Scale down (divide by 8)
					mobd = mobd * d2;		// Multiply the oldest difference
				}
				if (refractory){			// Avoid detecting extraneous peaks after QRS	
					refractory++;
					if (refractory == 40){	// Delay for 200 ms
						refractory = 0;		// Reset refractory flag to 0
						PORTBbits.RB3 = 0;	// Turn buzzer/LED off (Pin 36)
					}
				}
				else if (mobd > threshold){	// If a peak is detected,
					refractory = 1;			// Set refractory flag
					PORTBbits.RB3 = 1;		// Turn buzzer/LED on (Pin 36)
					display = 1;			// Set display flag										
				}
				PORTD =(unsigned char)mobd;	// Output mobd value to Port D
				INTCONbits.TMR0IE = 1;		// Enable TMR0 interrupt
			}
			if (display){					// Display Heart Rate in 3 digits
				hr = 12000/rri_count;		// 60/0.005 = 12000
                TransmitGraphData(2, hr);
				rri_count = 0;				// Reset RRI counter
				PrintNum(hr, 71);			// Isolates each digit and displays
				display = 0;				// Reset display flag
			}
			break;
		}
	}
}