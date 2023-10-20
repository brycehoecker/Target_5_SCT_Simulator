/*
 * RegisterSettings.h
 *
 *  Created on: Oct 19, 2023
 *      Author: sctsim
 */

#ifndef REGISTERSETTINGS_H_
#define REGISTERSETTINGS_H_

#include "UDPBase.h"
#include <stdlib.h>
#include <bitset>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <limits>
#include <vector>
#include <string>
#include <cstdint>
#include <iostream>
#include <map>
#include <sstream>
using namespace std;

/*
class RegisterSettings {
public:
	RegisterSettings();
	virtual ~RegisterSettings();
};
*/
/*!
 * @class RegisterSettings
 * @brief A class to hold all the register values for FPGA and ASIC on target
 *modules
 *
 */


class RegisterSettings {
public:
	/// Accessing mode for setting, eRW = Read and Write, eR = Read Only, eW = Write
	/// Only (do not check return value),eRW_NS = Read/Write, but non sticky write,
	/// so do not read back after a write
	enum Access { eRW, eR, eW, eRW_NS };
	struct RegSetting {
		uint32_t regAddr;
		// TODO(Akira): Change them to uint8_t, but be carefule when converting
		// stringstream to uint8_t values
		uint16_t nBits;
		uint16_t startBit;
		uint32_t value;
		//  bool isReadOnly;
		Access access;
		uint32_t lowerBound;
		uint32_t upperBound;
		float multiplier;
		float offset;
		string description;
	};
	struct SettingASIC {
		RegSetting settingASIC[4];
	};
	struct RegisterFPGA {
		uint32_t val;
		//  bool isReadOnly;
		Access access;
	};
	struct RegisterASIC {
		uint16_t val[4];
	};

	RegisterSettings();
	virtual ~RegisterSettings();
	/// Default constructor needs as arguments a FPGA and\n
	/// the TargetASIC definition file
	RegisterSettings(const string& targetFPGADefinitionFile,
				   const string& targetASICDefinitionFile);
	/// Constructor when we have a trigger and sampling ASIC
	RegisterSettings(const string& targetFPGADefinitionFile,
				   const string& targetASICDefinitionFile,
				   const string& targetTriggerASICDefinitionFile);
	/// Prints an FPGA Setting to the standard output
	void PrintSetting(RegSetting set) const;
	/// Prints an FPGA Setting to the standard output
	void PrintFPGASetting(const string& settingName) const;
	void PrintASICSetting(const string& settingName, bool isTriggerASIC = false) const;
	void PrintTriggerASICSetting(const string& settingName) const;
	void PrintAllSettings() const;
	void PrintHeaderFPGA() const;
	void PrintHeaderASIC(bool isTriggerASIC = false) const;
	void PrintHeaderTriggerASIC() const;
	void PrintAllRegisters() const;
	/// Generates a markdown file for documentation of ASIC registers
	void GenerateASICMarkdown(const string& fname, bool isTriggerASIC = false) const;
	/// Generates a markdown file for documentation of ASIC registers
	void GenerateTriggerASICMarkdown(const string& fname) const;
	/// Generates a markdown file for documentation of FPGA registers
	void GenerateFPGAMarkdown(const string& fname) const;
	int ReadUserFPGAConfigFile(const string& configFile);
	int ReadUserTriggerASICConfigFile(const string& configFile);
	int ReadUserASICConfigFile(const string& configFile,
							 bool isTriggerASIC = false);
	// Modify the value of a setting
	int ModifyFPGASetting(const string& name, uint32_t newVal);
	void GetRegisterPartially(uint32_t reg, RegSetting set,
							uint32_t& reg_par);  // NOLINT(runtime/references)

	int ModifyTriggerASICSetting(const string& name, uint8_t asic, uint16_t val);
	int ModifyASICSetting(const string& name, uint8_t asic, uint16_t val, bool isTriggerASIC = false);  /// HARM:DONE

	int GetFPGARegisterValue(uint32_t addr,
						   uint32_t& val) const;  // NOLINT(runtime/references)
	int GetTriggerASICRegisterValue(
	  const string& name, uint8_t asic,
	  uint16_t& val) const;  // NOLINT(runtime/references)

	int GetASICRegisterValue(
	  const string& name, uint8_t asic, uint16_t& val,
	  bool isTriggerASIC = false) const;  // NOLINT(runtime/references)
	int GetFPGASettingRegisterAddress(
	  const string& name,
	  uint32_t& addr) const;  // NOLINT(runtime/references)

	int GetTriggerASICSettingRegisterAddress(const string& name, uint8_t& addr) const;  // NOLINT(runtime/references)
	int GetASICSettingRegisterAddress( const string& name, uint8_t& addr, bool isTriggerASIC = false) const;  // NOLINT(runtime/references)

	string fType;
	uint32_t fFPGAFirmwareVersion;
	string fFPGADescription;
	string fFPGAAuthor;
	uint32_t fFPGANumberOfRegisters;
	string fASICDescription;
	string fASICAuthor;
	uint32_t fASICNumberOfRegisters;
	string fTriggerASICDescription;
	string fTriggerASICAuthor;
	uint32_t fTriggerASICNumberOfRegisters;

	int CheckFPGARegisterConsistency();
	int CheckTriggerASICRegisterConsistency();
	int CheckASICRegisterConsistency(bool isTriggerASIC = false);  /// HARM:DONE

	int ReadDefinitionFileFPGA();
	int ReadDefinitionFileTriggerASIC();
	int ReadDefinitionFileASIC(bool isTriggerASIC = false);

	// Helper code to modify parts of a 32 bit register
	void ModifyRegisterPartially(uint32_t& reg,  // NOLINT(runtime/references)
			RegSetting set);
	bool CheckRegisterPartially(uint32_t reg, RegSetting set);
	// Add settings and updates register map accordingly
	int AddFPGASetting(const string& name, RegSetting setting);
	int AddTriggerASICSetting(const string& name, SettingASIC setting);
	int AddASICSetting(const string& name, SettingASIC setting, bool isTriggerASIC = false);
	int UpdateFPGASettingMapFromRegisterMap();

	const string fDefintionFileFPGA;
	const string fDefintionFileASIC;
	const string fDefintionFileTriggerASIC;

	// TODO Fix map and Setting stuff below
	map<string, RegSetting> fSettingMapFPGA;
	typedef map<string, RegSetting>::const_iterator SF_cit;
	/// settings for the sampling asic (or sampling and triggering T5/T7)
	map<string, SettingASIC> fSettingMapASIC;
	typedef map<string, SettingASIC>::const_iterator SA_cit;
	/// Trigger ASIC settings map
	map<string, SettingASIC> fSettingMapTriggerASIC;

	map<uint32_t, RegisterFPGA> fRegisterMapFPGA;
	typedef map<uint32_t, RegisterFPGA>::const_iterator RF_cit;
	map<uint8_t, RegisterASIC> fRegisterMapASIC;
	typedef map<uint8_t, RegisterASIC>::const_iterator RA_cit;
	/// Trigger ASIC
	map<uint8_t, RegisterASIC> fRegisterMapTriggerASIC;
};

#endif /* REGISTERSETTINGS_H_ */
