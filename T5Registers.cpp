/*
 * T5Registers.cpp
 *
 *  Created on: Oct 12, 2023
 *      Author: sctsim
 */

#include "T5Registers.h"

T5Registers::T5Registers() {
	// TODO Auto-generated constructor stub
    // Initializing FPGA Version
	T5Registers::fpgaVersion.bits = 0xFED00030;
	T5Registers::fpgaVersion.function = "Assigned value of 0xFED00030, assigned in Firmware to highlight and track incremental changes in firmware. Incremented with every firmware revision";
	T5Registers::fpgaVersion.RW = 'R';
	T5Registers::fpgaVersion.defaultValue = 0;

    // Initializing Detector ID
	T5Registers::detectorID.bits31_16 = 0;  // You can initialize it with a specific value if needed
	T5Registers::detectorID.function31_16 = "Any value for control software, does not have effect on any FPGA logic";
	T5Registers::detectorID.RW31_16 = 'RW';
	T5Registers::detectorID.defaultValue31_16 = 0;

	T5Registers::detectorID.bits15_8 = 0;
	T5Registers::detectorID.function15_8 = "Detector ID, fill Detector ID field of reported event";
	T5Registers::detectorID.RW15_8 = 'RW';
	T5Registers::detectorID.defaultValue15_8 = 0;

	T5Registers::detectorID.bits7_0 = 0;
	T5Registers::detectorID.function7_0 = "CTA ID, fill CTA ID field of reported event";
	T5Registers::detectorID.RW7_0 = 'RW';
	T5Registers::detectorID.defaultValue7_0 = 0;

    // Initializing Serial Number LSW
	T5Registers::serialIDLSW.bits = 0;  // You can initialize it with a specific value if needed
	T5Registers::serialIDLSW.function = "Serial number the least significant word";
	T5Registers::serialIDLSW.RW = 'R';
	T5Registers::serialIDLSW.defaultValue = 0;

    // Initializing Serial Number MSW
	T5Registers::serialIDMSW.bits = 0;  // You can initialize it with a specific value if needed
	T5Registers::serialIDMSW.function = "Serial number the most significant word";
	T5Registers::serialIDMSW.RW = 'R';
	T5Registers::serialIDMSW.defaultValue = 0;


    T5Registers::status.bits31_16 = 0;
    T5Registers::status.function31_16 = "Unused, always 0";
    T5Registers::status.RW31_16 = 'R';
    T5Registers::status.defaultValue31_16 = 0;

    T5Registers::status.bits15_12 = 0;
    T5Registers::status.function15_12 = "Status of backplane lines from bp4 to bp7";
    T5Registers::status.RW15_12 = 'R';
    T5Registers::status.defaultValue15_12 = 0;

    T5Registers::status.bit11 = 0;
    T5Registers::status.function11 = "Unused, always 0";
    T5Registers::status.RW11 = 'R';
    T5Registers::status.defaultValue11 = 0;

    T5Registers::status.bit10 = 0;
    T5Registers::status.function10 = "mgt_AVCC_OK is OK. 1- OK, 0 – is not";
    T5Registers::status.RW10 = 'R';
    T5Registers::status.defaultValue10 = 0;

    T5Registers::status.bit9 = 0;
    T5Registers::status.function9 = "+1_8V is OK. 1- OK, 0 – is not";
    T5Registers::status.RW9 = 'R';
    T5Registers::status.defaultValue9 = 0;

    T5Registers::status.bits8_2 = 0;
    T5Registers::status.function8_2 = "Unused, always 0";
    T5Registers::status.RW8_2 = 'R';
    T5Registers::status.defaultValue8_2 = 0;

    T5Registers::status.bit1 = 0;
    T5Registers::status.function1 = "Underflow on summary FIFO of event data";
    T5Registers::status.RW1 = 'R';
    T5Registers::status.defaultValue1 = 0;

    T5Registers::status.bit0 = 0;
    T5Registers::status.function0 = "Overflow on summary FIFO of event data";
    T5Registers::status.RW0 = 'R';
    T5Registers::status.defaultValue0 = 0;

    // Initializing Latched Status Register (similar to Status Register but with some 'R*' bits)
    T5Registers::latchedStatus = status;  // Starting with the same values
    T5Registers::latchedStatus.RW15_12 = 'R*';
    T5Registers::latchedStatus.RW10 = 'R*';
    T5Registers::latchedStatus.RW9 = 'R*';
    T5Registers::latchedStatus.RW1 = 'R*';
    T5Registers::latchedStatus.RW0 = 'R*';


    // Initializing FIFO Status register ASIC 0
    T5Registers::fifoStatusASIC0.bits31_0 = 0;
    T5Registers::fifoStatusASIC0.function31_0 = "Data Storage FIFO underflow(bits 1,3,5,7...) and overflow (0,2,4...). Two bits per acquisition channel. Channel 0 errors reported in bits 0(underflow) and 1(overflow), channel 1 in bits 2 and 3,...";
    T5Registers::fifoStatusASIC0.RW31_0 = 'R';
    T5Registers::fifoStatusASIC0.defaultValue31_0 = 0;

    // Initializing Latched FIFO Status register ASIC 0
    T5Registers::latchedFIFOStatusASIC0 = fifoStatusASIC0; // Starting with the same values
    T5Registers::latchedFIFOStatusASIC0.RW31_0 = 'R*';

    // Initializing FIFO Status register ASIC 1
    T5Registers::fifoStatusASIC1.bits31_0 = 0;
    T5Registers::fifoStatusASIC1.function31_0 = "Data Storage FIFO underflow(bits 1,3,5,7...) and overflow (0,2,4...). Two bits per acquisition channel. Channel 0 errors reported in bits 0(underflow) and 1(overflow), channel 1 in bits 2 and 3,...";
    T5Registers::fifoStatusASIC1.RW31_0 = 'R';
    T5Registers::fifoStatusASIC1.defaultValue31_0 = 0;

    // Initializing Latched FIFO Status register ASIC 1
    T5Registers::latchedFIFOStatusASIC1 = fifoStatusASIC1; // Starting with the same values
    T5Registers::latchedFIFOStatusASIC1.RW31_0 = 'R*';

    // Initializing FIFO Status register ASIC 2
    T5Registers::fifoStatusASIC2.bits31_0 = 0;
    T5Registers::fifoStatusASIC2.function31_0 = "Data Storage FIFO underflow(bits 1,3,5,7...) and overflow (0,2,4...). Two bits per acquisition channel. Channel 0 errors reported in bits 0(underflow) and 1(overflow), channel 1 in bits 2 and 3,...";
    T5Registers::fifoStatusASIC2.RW31_0 = 'R';
    T5Registers::fifoStatusASIC2.defaultValue31_0 = 0;

    // Initializing Latched FIFO Status register ASIC 2
    T5Registers::latchedFIFOStatusASIC2.bits31_0 = 0;
    T5Registers::latchedFIFOStatusASIC2.function31_0 = "Data Storage FIFO underflow(bits 1,3,5,7...) and overflow (0,2,4...). Two bits per acquisition channel. Channel 0 errors reported in bits 0(underflow) and 1(overflow), channel 1 in bits 2 and 3,...";
    T5Registers::latchedFIFOStatusASIC2.RW31_0 = 'R*';
    T5Registers::latchedFIFOStatusASIC2.defaultValue31_0 = 0;

    // Initializing FIFO Status register ASIC 3
    T5Registers::fifoStatusASIC3.bits31_0 = 0;
    T5Registers::fifoStatusASIC3.function31_0 = "Data Storage FIFO underflow(bits 1,3,5,7...) and overflow (0,2,4...). Two bits per acquisition channel. Channel 0 errors reported in bits 0(underflow) and 1(overflow), channel 1 in bits 2 and 3,...";
    T5Registers::fifoStatusASIC3.RW31_0 = 'R';
    T5Registers::fifoStatusASIC3.defaultValue31_0 = 0;

    // Initializing Latched FIFO Status register ASIC 3
    T5Registers::latchedFIFOStatusASIC3 = fifoStatusASIC3; // Starting with the same values
    T5Registers::latchedFIFOStatusASIC3.RW31_0 = 'R*';

    // Initializing Trigger FIFO Status register
    T5Registers::triggerFIFOStatus.bits31_0 = 0;
    T5Registers::triggerFIFOStatus.function31_0 = "Free running time counter value set by sync over TACK pass 32 MSB";
    T5Registers::triggerFIFOStatus.RW31_0 = 'R';
    T5Registers::triggerFIFOStatus.defaultValue31_0 = 0;


    // Initializing Tack statistics register
    T5Registers::tackStatistics.bits31_24 = 0;
    T5Registers::tackStatistics.function31_24 = "Count sync errors. Time sync verification failed.";
    T5Registers::tackStatistics.RW31_24 = 'R';
    T5Registers::tackStatistics.defaultValue31_24 = 0;

    T5Registers::tackStatistics.bits23_16 = 0;
    T5Registers::tackStatistics.function23_16 = "Count of range error over Tack.";
    T5Registers::tackStatistics.RW23_16 = 'R';
    T5Registers::tackStatistics.defaultValue23_16 = 0;

    T5Registers::tackStatistics.bits15_8 = 0;
    T5Registers::tackStatistics.function15_8 = "Count Tack Parity errors";
    T5Registers::tackStatistics.RW15_8 = 'R';
    T5Registers::tackStatistics.defaultValue15_8 = 0;

    T5Registers::tackStatistics.bits7_0 = 0;
    T5Registers::tackStatistics.function7_0 = "Count Tack Parity errors";
    T5Registers::tackStatistics.RW7_0 = 'R';
    T5Registers::tackStatistics.defaultValue7_0 = 0;

    // Initializing FIFO statistics register
    T5Registers::fifoStatistics.bits31_16 = 0;
    T5Registers::fifoStatistics.function31_16 = "Count all enabled for counting (bits 31 of register 0x17) built packets on all incoming channels.";
    T5Registers::fifoStatistics.RW31_16 = 'R';
    T5Registers::fifoStatistics.defaultValue31_16 = 0;

    T5Registers::fifoStatistics.bits15_0 = 0;
    T5Registers::fifoStatistics.function15_0 = "Count all built packets on all incoming channels.";
    T5Registers::fifoStatistics.RW15_0 = 'R';
    T5Registers::fifoStatistics.defaultValue15_0 = 0;

   // Initializing Packet statistics register
    T5Registers::packetStatistics.bits31_16 = 0;
    T5Registers::packetStatistics.function31_16 = "Internal Arbiter Counter, count all packet forwarded from internal storage to MAC.";
    T5Registers::packetStatistics.RW31_16 = 'R';
    T5Registers::packetStatistics.defaultValue31_16 = 0;

    T5Registers::packetStatistics.bits15_0 = 0;
    T5Registers::packetStatistics.function15_0 = "MAC Counter, count all transmitted packet by MAC.";
    T5Registers::packetStatistics.RW15_0 = 'R';
    T5Registers::packetStatistics.defaultValue15_0 = 0;

   // Initializing Ramp count statistics register
    T5Registers::rampCountStatistics.bits31_16 = 0;
    T5Registers::rampCountStatistics.function31_16 = "Count command issued to Camera module.";
    T5Registers::rampCountStatistics.RW31_16 = 'R';
    T5Registers::rampCountStatistics.defaultValue31_16 = 0;

    T5Registers::rampCountStatistics.bits15_0 = 0;
    T5Registers::rampCountStatistics.function15_0 = "Count number of event processed.";
    T5Registers::rampCountStatistics.RW15_0 = 'R';
    T5Registers::rampCountStatistics.defaultValue15_0 = 0;



}

T5Registers::~T5Registers() {
	// TODO Auto-generated destructor stub
}

T5Registers::T5Registers(const T5Registers &other) {
	// TODO Auto-generated constructor stub

}

T5Registers::T5Registers(T5Registers &&other) {
	// TODO Auto-generated constructor stub

}

T5Registers& T5Registers::operator=(const T5Registers &other) {
	// TODO Auto-generated method stub

}

T5Registers& T5Registers::operator=(T5Registers &&other) {
	// TODO Auto-generated method stub

}

