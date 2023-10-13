#include <iostream>
#include "T5Registers.cpp"
#include "config.h"
using namespace std;

int main(int argc, char **argv) {
	cout << "Hello World" << endl;

	T5Registers registersArray[177];

	cout << "Version " << TargetSimulatorCMake_VERSION_MAJOR << "." << TargetSimulatorCMake_VERSION_MINOR << endl;
	return 0;
}
