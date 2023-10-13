/*
 * T5Registers.h
 *
 *  Created on: Oct 12, 2023
 *      Author: sctsim
 */

#ifndef T5REGISTERS_H_
#define T5REGISTERS_H_
#include <iostream>
using namespace std;

class T5Registers {
public:
	T5Registers();										// Default Constructor for the T5Registers class
	virtual ~T5Registers();								// Virtual Destructor for the T5Registers class

	T5Registers(const T5Registers &other);				// Copy constructor: used to initialize an object as a copy of another object of the same type. other is a reference to the object being copied
	T5Registers(T5Registers &&other);					// Move constructor: used to initialize an object by taking resources from another object. other is an rvalue reference of the original object(probably a temp object)

	T5Registers& operator=(const T5Registers &other);	// Copy assignment operator: allows one object to be assigned the values of another through a deep copy.
	T5Registers& operator=(T5Registers &&other);		// Move assignment operator: allows one object to take on the resources of another without making a copy. (by pointing to the same resources)

	void printRegisterData(unsigned int bits, string function, char RW, unsigned int defaultValue) {
        cout << "Bits: " << bits << "\nFunction: " << function << "\nR/W: " << RW << "\nDefault Value: " << defaultValue << "\n\n";
    }

    void printAll() {
        // Print Tack Statistics
        cout << "Tack Statistics:\n";
        printRegisterData(tackStatistics.bits31_24, tackStatistics.function31_24, tackStatistics.RW31_24, tackStatistics.defaultValue31_24);
        printRegisterData(tackStatistics.bits23_16, tackStatistics.function23_16, tackStatistics.RW23_16, tackStatistics.defaultValue23_16);
        printRegisterData(tackStatistics.bits15_8, tackStatistics.function15_8, tackStatistics.RW15_8, tackStatistics.defaultValue15_8);
        printRegisterData(tackStatistics.bits7_0, tackStatistics.function7_0, tackStatistics.RW7_0, tackStatistics.defaultValue7_0);

        // Print FIFO Statistics
        cout << "FIFO Statistics:\n";
        printRegisterData(fifoStatistics.bits31_16, fifoStatistics.function31_16, fifoStatistics.RW31_16, fifoStatistics.defaultValue31_16);
        printRegisterData(fifoStatistics.bits15_0, fifoStatistics.function15_0, fifoStatistics.RW15_0, fifoStatistics.defaultValue15_0);

        // Print Packet Statistics
        cout << "Packet Statistics:\n";
        printRegisterData(packetStatistics.bits31_16, packetStatistics.function31_16, packetStatistics.RW31_16, packetStatistics.defaultValue31_16);
        printRegisterData(packetStatistics.bits15_0, packetStatistics.function15_0, packetStatistics.RW15_0, packetStatistics.defaultValue15_0);

        // Print Ramp Count Statistics
        cout << "Ramp Count Statistics:\n";
        printRegisterData(rampCountStatistics.bits31_16, rampCountStatistics.function31_16, rampCountStatistics.RW31_16, rampCountStatistics.defaultValue31_16);
        printRegisterData(rampCountStatistics.bits15_0, rampCountStatistics.function15_0, rampCountStatistics.RW15_0, rampCountStatistics.defaultValue15_0);
    }
/*
    // You can also add public methods to read or write to this register
    // Consider adding a method to handle the specific write behavior for this register!

    // Example:
    void writeTriggerStatistics(unsigned int value) {
        // Assuming a hypothetical method resetCounterForRegister to reset counters for given register addresses
        resetCounterForRegister(0xF);
        resetCounterForRegister(0x10);
        resetCounterForRegister(0x11);
        resetCounterForRegister(0x12);
        // ... and so on for other affected registers
    }
*/
private:
	int moduleNumber;									// Number of Module should be 1-177 or 0-176

	uint32_t registers[84];  							// array to store all 84 registers (0-83)

	/*
	Master Board FPGA Memory Map Registers (32-Bit):
	*/
	/*
	0. FpgaVersion		R
		Assigned value of 0xFED000030, assigned in Firmware to highlight and track
		incremental changes in firmware. Incremented with every firmware revision
    // Example private member for FPGA Version*/
    struct FPGAVersion {
        unsigned int bits;       // Assuming 32-bit unsigned integer for 31-0
        string function;
        char RW;                 // 'R' or 'W' or 'RW'
        unsigned int defaultValue;
    } fpgaVersion;

    /*
	1. Detector ID		RW
		Bits
		31-16	Any value for control software, does not have effect on any FPGA logic
		15-8	Detector ID, fill Detector ID field of reported event
		7-0		CTA ID, fill CTA ID field of reported event
	// Example private member for Detector ID
	*/
    struct DetectorID {
        unsigned int bits31_16;  // Assuming 16-bit unsigned integer
        unsigned int bits15_8;   // Assuming 8-bit unsigned integer
        unsigned int bits7_0;    // Assuming 8-bit unsigned integer
        string function31_16;
        string function15_8;
        string function7_0;
        char RW31_16;
        char RW15_8;
        char RW7_0;
        unsigned int defaultValue31_16;
        unsigned int defaultValue15_8;
        unsigned int defaultValue7_0;
    } detectorID;

	/*
	2. Serial ID LSW	R
		Serial number the least significant word
	3. Serial ID MSW	R
		Serial number the most significant word*/
	 struct SerialIDNum {
			unsigned int bits;
			string function;
			char RW;
			unsigned int defaultValue;
		} serialIDLSW, serialIDMSW;  // LSW and MSW for Serial Number

	/*
	4. Status register	R
		Bits
		31-16	Unused, always 0
		15-12	Status of backplane lines from bp4 to bp7 (also, bp5 is reset and will not be available due to board reset)
		11		Unused, always 0
		10		mgt_AVCC_OK is OK, 1- OK, 0 – is not
		9		+1_8V is OK, 1- OK, 0 – is not
		8-2		Unused, always 0
		1		Underflow on summary FIFO of event data
		0		overflow on summary FIFO of event data
	5. Latched Status register*/
    struct StatusRegister {
        unsigned int bits31_16;
        unsigned int bits15_12;
        unsigned int bit11;
        unsigned int bit10;
        unsigned int bit9;
        unsigned int bits8_2;
        unsigned int bit1;
        unsigned int bit0;
        string function31_16;
        string function15_12;
        string function11;
        string function10;
        string function9;
        string function8_2;
        string function1;
        string function0;
        char RW31_16;
        char RW15_12;
        char RW11;
        char RW10;
        char RW9;
        char RW8_2;
        char RW1;
        char RW0;
        unsigned int defaultValue31_16;
        unsigned int defaultValue15_12;
        unsigned int defaultValue11;
        unsigned int defaultValue10;
        unsigned int defaultValue9;
        unsigned int defaultValue8_2;
        unsigned int defaultValue1;
        unsigned int defaultValue0;
    } status, latchedStatus;

	/*
	6. FIFO Status register ASIC0
	7. Latched FIFO Status register ASIC 0
	8. FIFO Status register ASIC1
	9. Latched FIFO Status register ASIC 1
	10. FIFO Status register ASIC2
	11. Latched FIFO Status register ASIC 2
	12. FIFO Status register ASIC3
	13. Latched FIFO Status register ASIC 3
	14. Trigger FIFO Status register
	*/
    struct FIFOStatusRegister {
        unsigned int bits31_0;
        string function31_0;
        char RW31_0;
        unsigned int defaultValue31_0;
    } fifoStatusASIC0, fifoStatusASIC1, fifoStatusASIC2, fifoStatusASIC3,
      latchedFIFOStatusASIC0, latchedFIFOStatusASIC1, latchedFIFOStatusASIC2, latchedFIFOStatusASIC3,
      triggerFIFOStatus;

    /*
	15. Trigger statistics
	*/
    struct TriggerStatisticsRegister {
        unsigned int bits31_16;
        string function31_16;
        char RW31_16;
        unsigned int defaultValue31_16;

        unsigned int bits15_0;
        string function15_0;
        char RW15_0;
        unsigned int defaultValue15_0;
    } triggerStatistics;

    /*
	16. TACK statistic
	*/
	struct TackStatisticsRegister {
        unsigned int bits31_24;
        string function31_24;
        char RW31_24;
        unsigned int defaultValue31_24;

        unsigned int bits23_16;
        string function23_16;
        char RW23_16;
        unsigned int defaultValue23_16;

        unsigned int bits15_8;
        string function15_8;
        char RW15_8;
        unsigned int defaultValue15_8;

        unsigned int bits7_0;
        string function7_0;
        char RW7_0;
        unsigned int defaultValue7_0;
    } tackStatistics;

    /*
	17. FIFO statistics
	*/
    struct FIFOStatisticsRegister {
        unsigned int bits31_16;
        string function31_16;
        char RW31_16;
        unsigned int defaultValue31_16;

        unsigned int bits15_0;
        string function15_0;
        char RW15_0;
        unsigned int defaultValue15_0;
    } fifoStatistics;

    /*
	18. Packet statistics
	*/
    struct PacketStatisticsRegister {
        unsigned int bits31_16;
        string function31_16;
        char RW31_16;
        unsigned int defaultValue31_16;

        unsigned int bits15_0;
        string function15_0;
        char RW15_0;
        unsigned int defaultValue15_0;
    } packetStatistics;

    /*
	19. Command/Ramp count statistics
	*/
    struct RampCountStatisticsRegister {
        unsigned int bits31_16;
        string function31_16;
        char RW31_16;
        unsigned int defaultValue31_16;

        unsigned int bits15_0;
        string function15_0;
        char RW15_0;
        unsigned int defaultValue15_0;
    } rampCountStatistics;

    /*
    20. ADC_control_reg
	*/


    /*
	21. ADC_config_reg



	22. Time register



	23. Control register 0



	24. Control register 1



	25. Trigger control register 0



	26. Trigger control register 1



	27. Row/Column control/status



	28. Number of samples to read



	29. Serial data idelay control register



	30. Configuration waveform register



	31. Mics Test Register



	32. SST FB PLL alignments register



	33. SST FB idelay alignments register Status



	34. SST FB idelay alignments register Control



	35. Spare



	36. ROVDD feedback control ASIC 0



	37. ROVDD feedback control ASIC 1



	38. ROVDD feedback control ASIC 2



	39. ROVDD feedback control ASIC 3



	40. VDEL feedback control ASIC 0



	41. VDEL feedback control ASIC 1



	42. VDEL feedback control ASIC 2



	43. VDEL feedback control ASIC 3



	44. VDEL/ROVDD calculated values ASIC 0



	45. VDEL/ROVDD calculated values ASIC 1



	46. VDEL/ROVDD calculated values ASIC 2



	47. VDEL/ROVDD calculated values ASIC 3



	48. VPED DAC control ASIC 0



	49. VPED DAC control ASIC 1



	50. VPED DAC control ASIC 2



	51. VPED DAC control ASIC 3



	52. VDELNP DAC control ASIC 0



	53. VDELNP DAC control ASIC 1



	54. VDELNP DAC control ASIC 2



	55. VDELNP DAC control ASIC 3



	56. DC_IN DAC control



	57. HV DAC control



	58. Zero suppression config



	59. ADC measurement reg 0



	60. ADC measurement reg 1



	61. ADC measurement reg 2



	62. ADC measurement reg 3



	63. ADC measurement reg 4



	64. ADC measurement reg 5



	65. ADC measurement reg 6



	66. ADC measurement reg 7



	67. ADC measurement reg 8



	68. ADC measurement reg 9



	69. ADC measurement reg 10



	70. ADC measurement reg 11



	71. ADC measurement reg 12



	72. Trigger Efficiency Cntl 0



	73. Trigger Efficiency Cntl 1



	74. Trigger Efficiency Input Cntr



	75. Trigger Efficiency Cntr



	76. Software reset register



	77. Channel enable register 0



	78. Channel enable register 1



	79. Spare/Test



	80. Write Target register



	81. Read 0 Target register



	82. Read 1 Target register



	83. Dead-time control



	 */
};

#endif /* T5REGISTERS_H_ */
