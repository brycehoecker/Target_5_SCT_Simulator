#include <iostream>
#include "T5Registers.cpp"
#include "config.h"
using namespace std;

int main(int argc, char **argv) {
	cout << "Hello World" << endl;

    const int NUM_REGISTERS = 177;
    T5Registers registersArray[NUM_REGISTERS];

    for (int i = 0; i < NUM_REGISTERS; ++i) {
        cout << "Register " << (i+1) << ":\n";
        registersArray[i].printAll();
        cout << "---------------------------------\n"; // separator for clarity
    }

	cout << "Version " << TargetSimulatorCMake_VERSION_MAJOR << "." << TargetSimulatorCMake_VERSION_MINOR << endl;
	return 0;
}
