/*
 * RegisterSettings.cpp
 *
 *  Created on: Oct 19, 2023
 *      Author: sctsim
 */

#include "RegisterSettings.h"

using namespace std;

/*
RegisterSettings::RegisterSettings() {
	// TODO Auto-generated constructor stub

}

RegisterSettings::~RegisterSettings() {
	// TODO Auto-generated destructor stub
}
*/

RegisterSettings::RegisterSettings(const string& pTargetFPGADefinitionFile, const string& pTargetASICDefinitionFile)
	: fDefintionFileFPGA(pTargetFPGADefinitionFile),fDefintionFileASIC(pTargetASICDefinitionFile), fDefintionFileTriggerASIC("") {
	if (fDefintionFileFPGA == "") {
		cerr << "WARNING: Initialization of RegisterSettings without FPGA def file" << endl;
	} //if
	else {
		ReadDefinitionFileFPGA();
		CheckFPGARegisterConsistency();
		ReadDefinitionFileASIC();
		CheckASICRegisterConsistency();
	}//else
}// RegisterSettings::RegisterSettings

RegisterSettings::RegisterSettings(const string& pTargetFPGADefinitionFile,
									const string& pTargetASICDefinitionFile,
									const string& pTargetTriggerASICDefinitionFile)
									: fDefintionFileFPGA(pTargetFPGADefinitionFile),
									  fDefintionFileASIC(pTargetASICDefinitionFile),
									  fDefintionFileTriggerASIC(pTargetTriggerASICDefinitionFile) {
	if (fDefintionFileFPGA == "") {
		cerr << "WARNING: Initialization of RegisterSettings without FPGA def file" << endl;
	}
	else {
		ReadDefinitionFileFPGA();
		CheckFPGARegisterConsistency();
		ReadDefinitionFileASIC();
		CheckASICRegisterConsistency();
		if (fDefintionFileTriggerASIC != "") {
			ReadDefinitionFileTriggerASIC();
			CheckTriggerASICRegisterConsistency();
		}//if
	}//else
}//RegisterSettings::RegisterSettings

int RegisterSettings::ReadDefinitionFileFPGA() {
	ifstream ifs_FPGA_Def(fDefintionFileFPGA.c_str());

	if (!ifs_FPGA_Def) {
		cerr << "**FATAL ERROR CANNOT OPEN " << fDefintionFileFPGA << " specify complete path to definition file " << endl;
		exit(-1);
	}//if (!ifs_FPGA_Def)

	string FPGA_Def_str;
	// HEADER
	while (getline(ifs_FPGA_Def, FPGA_Def_str)) {
		if (FPGA_Def_str[0] == '#') continue;
		if (FPGA_Def_str == "HEADER") break;
	}//while (getline(ifs_FPGA_Def, FPGA_Def_str))

	getline(ifs_FPGA_Def, FPGA_Def_str);
	stringstream ssType(FPGA_Def_str);
	string dum;
	ssType >> dum;
	if (dum != "TM_TYPE") {
		cerr << "EXPECTED TM_TYPE but read: " << dum << endl;
		exit(-1);
	}//if (dum != "TM_TYPE")
	ssType >> fType;

	getline(ifs_FPGA_Def, FPGA_Def_str);

	stringstream ssFW(FPGA_Def_str);

	ssFW >> dum;
	if (dum != "TM_FIRMWARE_VERSION") {
	cerr << "EXPECTED TM_FIRMWARE_VERSION but read: " << dum << endl;
	exit(-1);
	}
	ssFW >> hex >> fFPGAFirmwareVersion;

	getline(ifs_FPGA_Def, FPGA_Def_str);
	stringstream ssDE(FPGA_Def_str);

	ssDE >> dum;
	if (dum != "DESCRIPTION") {
		cerr << "EXPECTED DESCRIPTION but read: " << dum << endl;
		exit(-1);
	}
	fFPGADescription = "";
	while (ssDE >> dum) {
		fFPGADescription += dum + " ";
	}

	getline(ifs_FPGA_Def, FPGA_Def_str);
	stringstream ssAU(FPGA_Def_str);
	ssAU >> dum;
	if (dum != "RESPONSIBLE_AUTHOR") {
		cerr << "EXPECTED RESPONSIBLE_AUTHOR but read: " << dum << endl;
		exit(-1);
	}
	ssAU >> fFPGAAuthor;

	getline(ifs_FPGA_Def, FPGA_Def_str);
	stringstream ssN(FPGA_Def_str);
	ssN >> dum;
	if (dum != "NUM_REGISTERS") {
		cerr << "EXPECTED NUM_REGISTERS but read: " << dum << endl;
		exit(-1);
	}
	ssN >> hex >> fFPGANumberOfRegisters;

	// SETTINGS
	while (getline(ifs_FPGA_Def, FPGA_Def_str)) {
		if (FPGA_Def_str[0] == '#') continue;
		if (FPGA_Def_str == "SETTINGS") break;
	}

	while (getline(ifs_FPGA_Def, FPGA_Def_str)) {
		if (FPGA_Def_str[0] == '#') continue;
		RegSetting setting;
		stringstream ss(FPGA_Def_str);
		string name;

		ss >> name;
		ss >> hex >> setting.regAddr;
		ss >> dec >> setting.nBits;
		ss >> dec >> setting.startBit;
		ss >> hex >> setting.value;
		//    ss >> setting.isReadOnly;

		uint16_t accessMode;
		ss >> accessMode;
		if (accessMode == 0) {
			setting.access = eRW;
		}
		else if (accessMode == 1) {
			setting.access = eR;
		}
		else if (accessMode == 2) {
			setting.access = eW;
		}
		else if (accessMode == 3) {
			setting.access = eRW_NS;
		}
		else {
// TODO fix error
//			cerr << "Unknown AccesMode" << int(accessMode) << " in line: " << line << endl;
			exit(-1);
		}
		//    ss >> setting.access;
		ss >> hex >> setting.lowerBound;
		ss >> hex >> setting.upperBound;
		ss >> dec >> setting.multiplier;
		ss >> dec >> setting.offset;
		string word;
		while (ss >> word) setting.description += word + " ";

		if (setting.description.size() == 0) {
			cerr << "WARNING: description missing for Setting: " << name << endl;
		}// if size=0

		if (setting.description[0] != '#') {
// TODO fix error
//			cerr << "ERROR: comments in *.def file should start with a #, check this line for consistency: " << line << endl;
		}// if [0] is not #

		AddFPGASetting(name, setting);
	}
	return TC_OK;
}//RegisterSettings::ReadDefinitionFileFPGA()
int RegisterSettings::ReadDefinitionFileTriggerASIC() {
  return ReadDefinitionFileASIC(true);
}
int RegisterSettings::ReadDefinitionFileASIC(bool isTriggerASIC) {
	ifstream ReadASICDefFileifs;
	string fDefFileASIC;
	fDefFileASIC = fDefintionFileASIC;
// TODO fix the READING ASICDefFiles
	if (!isTriggerASIC) {
		//ReadASICDefFileifs.open(fDefintionFileASIC.c_str());
		//ReadASICDefFileifs.open(fDefFileASIC);
	}
	else
		//ReadASICDefFileifs.open(fDefintionFileTriggerASIC.c_str());

	if (!ReadASICDefFileifs) {
		//string filename = fDefintionFileASIC;
		string filename = this->fDefintionFileASIC;
		if (isTriggerASIC) filename = fDefintionFileTriggerASIC; cerr << "FATAL ERROR CANNOT OPEN " << filename << " specify complete path to definition file " << endl;
		exit(-1);
	}// if (!ifs)

	string line;
	// HEADER
	while (getline(ReadASICDefFileifs, line)) {
		if (line[0] == '#') continue;
		if (line == "HEADER") break;
	}//while (getline(ReadASICDefFileifs, line))

	string dum;
	getline(ReadASICDefFileifs, line);
	stringstream ssDE(line);
	ssDE >> dum;
	if (dum != "DESCRIPTION") {
		cerr << "EXPECTED DESCRIPTION but read: " << dum << endl;
		exit(-1);
	}
	string descr = "";
	while (ssDE >> dum) {
		descr += dum + " ";
	}
	if (!isTriggerASIC) {
		fASICDescription = descr;
	}
	else {
		fTriggerASICDescription = descr;
	}

	getline(ReadASICDefFileifs, line);
	stringstream ssAU(line);
	ssAU >> dum;
	if (dum != "RESPONSIBLE_AUTHOR") {
		cerr << "EXPECTED RESPONSIBLE_AUTHOR but read: " << dum << endl;
		exit(-1);
	}
	string author = "";
	while (ssAU >> dum) {
		author += dum + " ";
	}
	if (!isTriggerASIC) {
		fASICAuthor = author;
	}
	else {
		fTriggerASICAuthor = author;
	}

	getline(ReadASICDefFileifs, line);
	stringstream ssN(line);
	ssN >> dum;
	if (dum != "NUM_REGISTERS") {
		cerr << "EXPECTED NUM_REGISTERS but read: " << dum << endl;
		exit(-1);
	}
	if (!isTriggerASIC) {
		ssN >> hex >> fASICNumberOfRegisters;
	} else {
	ssN >> hex >> fTriggerASICNumberOfRegisters;
	}
	// SETTINGS
	while (getline(ReadASICDefFileifs, line)) {
		if (line[0] == '#') continue;
		if (line == "SETTINGS") break;
	}

	while (getline(ReadASICDefFileifs, line)) {
		if (line[0] == '#') continue;
		RegSetting setting;
		stringstream ss(line);
		string name;
		ss >> name;
		ss >> hex >> setting.regAddr;
		ss >> dec >> setting.nBits;
		ss >> dec >> setting.startBit;
		ss >> hex >> setting.value;
		//    ss >> setting.isReadOnly;
		//    ss >> setting.access;
		uint16_t accessMode;
		ss >> accessMode;
		if (accessMode == 0) {
		  setting.access = eRW;
		} else if (accessMode == 1) {
		  setting.access = eR;
		} else if (accessMode == 2) {
		  setting.access = eW;
		} else if (accessMode == 3) {
		  setting.access = eRW_NS;
		} else {
		  cerr << "Unknown AccesMode in " << line << endl;
		  exit(-1);
		}

		ss >> hex >> setting.lowerBound;
		ss >> hex >> setting.upperBound;
		ss >> dec >> setting.multiplier;
		ss >> dec >> setting.offset;

		if (ss.bad()) {
		  cerr << "ERROR: Could not parse a line" << endl;
		  cerr << line << endl;
		  exit(-1);
		}

		string word;
		while (ss >> word) setting.description += word + " ";

		if (setting.description.size() == 0) {
		  // cerr << "WARNING: description missing for Setting: " << name
		  //           << endl;
		}
		SettingASIC asicSettings;
		for (int i = 0; i < 4; ++i) asicSettings.settingASIC[i] = setting;
		AddASICSetting(name, asicSettings, isTriggerASIC);
	}
	return TC_OK;
}
int RegisterSettings::CheckFPGARegisterConsistency() {
	/// CHECK THE TOTAL NUMBER OF REGISTERS
	if (fFPGANumberOfRegisters != fRegisterMapFPGA.size()) {
	  cerr << "ERROR: number of FPGA registers (" << hex << fRegisterMapFPGA.size() << ") doesn't correspond to the value " "specified in the HEADER: " << fFPGANumberOfRegisters << endl;
	}

	/// CHECK FOR MISSING REGISTER
	RF_cit itReg = fRegisterMapFPGA.cbegin();
	vector<uint32_t> vMissingRegisters;
	uint32_t iMiss = 0;
	// TODO(Harm): Use normal for loop
	while (itReg != fRegisterMapFPGA.cend()) {
	//    cout << itReg->first << "\t" << itReg->second.val << endl;
	if (itReg->first != iMiss) {
	  vMissingRegisters.push_back(iMiss);
	} else {
	  ++iMiss;
	}

	++itReg;
	}

	if (vMissingRegisters.size() != 0) {
	cerr << "ERROR: missing registers FPGA in *.def file : ";
	cerr << "First Missing register is: 0x" << hex
			  << vMissingRegisters[0] << endl;
	// TODO(Harm): Don't use exit
	exit(-1);
	}

	/// CHECK THAT ALL BITS ARE IN SETTINGS MAP
	itReg = fRegisterMapFPGA.cbegin();
	// TODO(Harm): Use normal for loop
	while (itReg != fRegisterMapFPGA.cend()) {
	vector<Setting> vSet;
	typedef vector<Setting>::const_iterator v_cit;
	for (SF_cit itSet = fSettingMapFPGA.begin(); itSet != fSettingMapFPGA.end();
		 ++itSet) {
	  if (itSet->second.regAddr == itReg->first) {
		vSet.push_back(itSet->second);
	  }
	}  // all setting for a register
	vector<bool> vAreSet(32, false);
	typedef vector<bool>::const_iterator vA_cit;
	for (v_cit cit = vSet.cbegin(); cit != vSet.cend(); ++cit) {
	  for (uint16_t ibit = cit->startBit; ibit < (cit->startBit + cit->nBits);
		   ++ibit) {
		if (vAreSet[ibit]) {
		  cerr << "ERROR: bit " << dec << ibit
					<< " in FPGA register 0x" << hex << cit->regAddr
					<< " all ready set, check definition file for overlapping "
					   "settings ("
					<< fDefintionFileFPGA << ")" << dec << endl;
		  exit(-1);
		} else {
		  vAreSet[ibit] = true;
		}
	  }
	}
	/// are all bit set?
	bool allSet = true;
	for (vA_cit cit = vAreSet.cbegin(); cit != vAreSet.cend(); ++cit) {
	  if (*cit != true) allSet = false;
	}
	if (!allSet) {
	  cerr << "ERROR: missing bits in FPGA register 0x" << hex
				<< itReg->first << dec << endl;
	  cerr << "bit\tSet?" << endl;
	  for (vA_cit cit = vAreSet.cbegin(); cit != vAreSet.cend(); ++cit) {
		cerr << distance(vAreSet.cbegin(), cit) << "\t" << *cit
				  << endl;
	  }
	  exit(-1);
	}  // all set
	++itReg;
	}

	return TC_OK;
}
int RegisterSettings::CheckTriggerASICRegisterConsistency() {
  return CheckASICRegisterConsistency(true);
}
int RegisterSettings::CheckASICRegisterConsistency(bool isTriggerASIC) {
  /// CHECK THE TOTAL NUMBER OF REGISTERS
  if (!isTriggerASIC) {
    if (fASICNumberOfRegisters != fRegisterMapASIC.size()) {
      cerr << "ERROR: number of ASIC registers (" << hex
                << fRegisterMapASIC.size()
                << ") doesn't correspond to the value "
                   "specified in the HEADER: "
                << fASICNumberOfRegisters << endl;
    }
  } else {
    if (fTriggerASICNumberOfRegisters != fRegisterMapTriggerASIC.size()) {
      cerr << "ERROR: number of TRIGGER ASIC registers (" << hex
                << fRegisterMapTriggerASIC.size()
                << ") doesn't correspond to the value "
                   "specified in the HEADER: "
                << fTriggerASICNumberOfRegisters << endl;
    }
  }

  map<uint8_t, RegisterASIC>::iterator itReg;
  map<uint8_t, RegisterASIC>::iterator endReg;
  if (!isTriggerASIC) {
    itReg = fRegisterMapASIC.begin();
    endReg = fRegisterMapASIC.end();
  } else {
    itReg = fRegisterMapTriggerASIC.begin();
    endReg = fRegisterMapTriggerASIC.end();
  }

  /// CHECK FOR MISSING REGISTER
  vector<uint8_t> vMissingRegisters;
  uint8_t iMiss = 0;
  // TODO(Harm): Use normal for loop
  while (itReg != endReg) {
    //    cout << itReg->first << "\t" << itReg->second.val << endl;
    if (itReg->first != iMiss) {
      vMissingRegisters.push_back(iMiss);
    } else {
      ++iMiss;
    }

    ++itReg;
  }
  if (vMissingRegisters.size() != 0) {
    cerr << "ERROR: missing registers in ASIC *.def file : ";
    cerr << "First Missing register is: 0x" << hex
              << vMissingRegisters[0] << endl;
    exit(-1);
  }

  /// CHECK THAT ALL BITS ARE IN SETTINGS MAP
  if (!isTriggerASIC) {
    itReg = fRegisterMapASIC.begin();
  } else {
    itReg = fRegisterMapTriggerASIC.begin();
  }

  // TODO(Harm): Use normal for loop
  while (itReg != endReg) {
    vector<Setting> vSet;
    typedef vector<Setting>::const_iterator v_cit;
    map<string, SettingASIC>::iterator itSet;
    map<string, SettingASIC>::iterator endSet, beginSet;
    if (!isTriggerASIC) {
      beginSet = fSettingMapASIC.begin();
      endSet = fSettingMapASIC.end();
    } else {
      beginSet = fSettingMapTriggerASIC.begin();
      endSet = fSettingMapTriggerASIC.end();
    }

    for (itSet = beginSet; itSet != endSet; ++itSet) {
      if (itSet->second.settingASIC[0].regAddr == itReg->first)
        vSet.push_back(itSet->second.settingASIC[0]);
    }  // all setting for a register
    vector<bool> vAreSet(12, false);
    typedef vector<bool>::const_iterator vA_cit;
    for (v_cit cit = vSet.cbegin(); cit != vSet.cend(); ++cit) {
      for (uint16_t ibit = cit->startBit; ibit < (cit->startBit + cit->nBits);
           ++ibit) {
        if (vAreSet[ibit]) {
          string sDefFile;
          if (!isTriggerASIC) {
            sDefFile = fDefintionFileASIC;
          } else {
            sDefFile = fDefintionFileTriggerASIC;
          }
          cerr << "ERROR: bit " << dec << ibit
                    << " in ASIC register 0x" << hex << cit->regAddr
                    << " all ready set, check definition file for overlapping "
                       "settings ("
                    << sDefFile << ")" << dec << endl;
          exit(-1);
        } else {
          vAreSet[ibit] = true;
        }
      }
    }
    /// are all bit set?
    bool allSet = true;
    for (vA_cit cit = vAreSet.cbegin(); cit != vAreSet.cend(); ++cit) {
      if (*cit != true) {
        allSet = false;
      }
    }
    if (!allSet) {
      cerr << "ERROR: missing bits in ASIC register " << hex
                << itReg->first << dec << endl;
      cerr << "bit\tSet?" << endl;
      for (unsigned int i = 0; i < vAreSet.size(); ++i) {
        cerr << i << "\t" << vAreSet[i] << endl;
      }
      exit(-1);
    }  // all set
    ++itReg;
  }

  return TC_OK;
}
bool RegisterSettings::CheckRegisterPartially(uint32_t pRegisterValue,
                                              Setting pSetting) {
  // get part of the register we are want to check
  uint32_t select;
  if (pSetting.nBits == 32) {
    select = 0;
    select = ~select;
  } else {
    select = ((uint32_t((1 << pSetting.nBits) - 1)) << pSetting.startBit);
  }
  //  cout << "REG: " << bitset<32>(reg) << endl;
  //  cout << "select: " <<  bitset<32>(select) << endl;
  pRegisterValue = pRegisterValue & select;
  pRegisterValue =
      pRegisterValue >>
      pSetting.startBit;  // shift them towards to end and compare them
  if (pSetting.value != pRegisterValue) {
    cout << "CheckRegisterPartially Setting Value: " << hex
              << pSetting.value
              << " it should be (register value): " << pRegisterValue
              << endl;
    return false;
  }
  return true;
}
void RegisterSettings::GetRegisterPartially(uint32_t pRegisterValue,
                                            Setting pSetting,
                                            uint32_t& pPartialRegisterValue) {
  // get part of the register we are want to check
  uint32_t select = 0;
  if (pSetting.nBits == 32) {
    select = ~select;
  } else {
    select = ((uint32_t((1 << pSetting.nBits) - 1)) << pSetting.startBit);
  }
  pRegisterValue = pRegisterValue & select;
  // shift them towards to end
  pPartialRegisterValue = pRegisterValue >> pSetting.startBit;
}
void RegisterSettings::ModifyRegisterPartially(uint32_t& reg, Setting set) {
  // check if value is not larger that nBits
  if (set.value >= uint32_t(1 << set.nBits)) {
    cerr << "ERROR: don't be silly, you can not fit in " << dec
              << set.value << " in " << set.nBits
              << " bits ( register: " << hex
              << static_cast<int64_t>(set.regAddr) << dec << ")"
              << endl;
    PrintSetting(set);
    exit(-1);
  }
  // clean out the bits that we want to overwrite
  uint32_t clean = ~((uint32_t((1 << set.nBits) - 1)) << set.startBit);
  reg = reg & clean;
  // shift the new values towards the start bit
  uint32_t shiftedVal = set.value << set.startBit;
  // update the cleaned register
  reg = reg | shiftedVal;
}
int RegisterSettings::AddFPGASetting(const string& name, Setting setting) {
  map<string, Setting>::iterator itSet;
  itSet = fSettingMapFPGA.find(name);
  if (itSet != fSettingMapFPGA.end()) {
    cerr << "WARNING: < " << name
              << " > Setting already there check "
                 "definition file for duplicated "
                 "settings"
              << endl;
    return 0;
  }
  // updating fSettingMapFPGA
  fSettingMapFPGA[name] = setting;

  // register can already be partially set.
  // if not, the part that is not written yet
  // will be filled with zeroes
  map<uint32_t, RegisterFPGA>::iterator itReg;
  itReg = fRegisterMapFPGA.find(setting.regAddr);

  RegisterFPGA reg;
  reg.val = 0;

  if (itReg != fRegisterMapFPGA.end()) {
    reg.val = fRegisterMapFPGA[setting.regAddr].val;
    //    reg.isReadOnly = setting.isReadOnly;
    reg.access = setting.access;
  }

  // checking if we have to set the full register
  if (setting.nBits == 0x20 && setting.startBit == 0) {
    reg.val = setting.value;
  } else {  // Filling the register partially
    ModifyRegisterPartially(reg.val, setting);
  }

  // update value
  fRegisterMapFPGA[setting.regAddr] = reg;
  return 0;
}
int RegisterSettings::UpdateFPGASettingMapFromRegisterMap() {
  for (SF_cit it = fSettingMapFPGA.begin(); it != fSettingMapFPGA.end(); it++) {
    uint32_t regVal = fRegisterMapFPGA[it->second.regAddr].val;
    uint32_t setVal;
    GetRegisterPartially(regVal, it->second, setVal);
    fSettingMapFPGA[it->first].value = setVal;
  }
  return TC_OK;
}
int RegisterSettings::AddTriggerASICSetting(const string& name,
                                            SettingASIC setting) {
  return AddASICSetting(name, setting, true);
}
int RegisterSettings::AddASICSetting(const string& name,
                                     SettingASIC setting, bool isTriggerASIC) {
  map<string, SettingASIC>::iterator itSet, itEnd;
  if (!isTriggerASIC) {
    itEnd = fSettingMapASIC.end();
    itSet = fSettingMapASIC.find(name);
  } else {
    itEnd = fSettingMapTriggerASIC.end();
    itSet = fSettingMapTriggerASIC.find(name);
  }

  if (itSet != itEnd) {
    cerr << "WARNING: < " << name
              << " > Setting already there check "
                 "definition file for duplicated "
                 "settings"
              << endl;
    return TC_ERR_CONF_FAILURE;
  }

  uint16_t max12bit = (1 << 12);
  for (int i = 0; i < 4; ++i) {
    if (setting.settingASIC[i].value > max12bit) {
      cerr << "ERROR: ASIC setting <" << name
                << "> has a value larger than 12 bits. This is not allowed"
                << endl;
    }
  }

  // updating fSettingMapASIC
  if (!isTriggerASIC) {
    fSettingMapASIC[name] = setting;
  } else {
    fSettingMapTriggerASIC[name] = setting;
  }
  // register can already be partially set.
  // if not, the part that is not written yet
  // will be filled with zeroes

  RegisterASIC asicReg;
  for (uint8_t asic = 0; asic < 4; ++asic) {
    asicReg.val[asic] = 0;
  }

  uint8_t regAddr = static_cast<uint8_t>(setting.settingASIC[0].regAddr);
  map<uint8_t, RegisterASIC>::iterator itReg, endReg;
  if (!isTriggerASIC) {
    itReg = fRegisterMapASIC.find(regAddr);
    endReg = fRegisterMapASIC.end();
    // if doesnot exist, fill with zeros
    if (itReg == endReg) {
      fRegisterMapASIC[regAddr] = asicReg;
    } else
      asicReg = fRegisterMapASIC[regAddr];
  } else {
    itReg = fRegisterMapTriggerASIC.find(regAddr);
    endReg = fRegisterMapTriggerASIC.end();
    // if doesnot exist, fill with zeros
    if (itReg == endReg) {
      fRegisterMapTriggerASIC[regAddr] = asicReg;
    } else
      asicReg = fRegisterMapTriggerASIC[regAddr];
  }

  // loop over the 4 asic and update the setting
  for (uint8_t asic = 0; asic < 4; ++asic) {
    Setting set = setting.settingASIC[asic];
    if (set.nBits == 0x0C && set.startBit == 0) {
      asicReg.val[asic] = static_cast<uint16_t>(set.value);
    } else {  // Filling the register partially
      uint32_t reg = asicReg.val[asic];
      ModifyRegisterPartially(reg, set);
      asicReg.val[asic] = uint16_t(reg);
    }
  }

  // update register map
  if (!isTriggerASIC) {
    fRegisterMapASIC[regAddr] = asicReg;
  } else {
    fRegisterMapTriggerASIC[regAddr] = asicReg;
  }
  return TC_OK;
}
int RegisterSettings::ReadUserFPGAConfigFile(const string& configFile) {
  ifstream ifs(configFile.c_str());
  if (!ifs) {
    cerr << "FATAL ERROR CANNOT OPEN " << configFile
              << " specify complete path to user config-file " << endl;
    exit(-1);
  }

  string line;
  while (getline(ifs, line)) {
    if (line[0] == '#') continue;
    stringstream ss(line);
    string name;
    ss >> name;
    uint32_t val;
    ss >> hex >> val;
    ModifyFPGASetting(name, val);
  }
  return TC_OK;
}
int RegisterSettings::ReadUserTriggerASICConfigFile(
    const string& configFile) {
  return ReadUserASICConfigFile(configFile, true);
}
int RegisterSettings::ReadUserASICConfigFile(const string& configFile,
                                             bool isTriggerASIC) {
  ifstream ifs(configFile.c_str());
  if (!ifs) {
    cerr << "FATAL ERROR CANNOT OPEN " << configFile
              << " specify complete path to user config-file " << endl;
    exit(-1);
  }

  string line;
  while (getline(ifs, line)) {
    if (line[0] == '#') continue;
    stringstream ss(line);
    string name;
    ss >> name;
    uint16_t val;
    ss >> hex >> val;
    ModifyASICSetting(name, 0, val, isTriggerASIC);
    ss >> hex >> val;
    ModifyASICSetting(name, 1, val, isTriggerASIC);
    ss >> hex >> val;
    ModifyASICSetting(name, 2, val, isTriggerASIC);
    ss >> hex >> val;
    ModifyASICSetting(name, 3, val, isTriggerASIC);
  }

  return TC_OK;
}
int RegisterSettings::ModifyTriggerASICSetting(const string& name,
                                               uint8_t asic, uint16_t newVal) {
  return ModifyASICSetting(name, asic, newVal, true);
}
int RegisterSettings::ModifyASICSetting(const string& name, uint8_t asic,
                                        uint16_t newVal, bool isTriggerASIC) {
  if (asic > 3 || asic < 0) {
    cerr << "ASIC must be smaller than 4" << endl;
    exit(-1);
  }

  map<string, SettingASIC>::iterator it;
  if (!isTriggerASIC) {
    it = fSettingMapASIC.find(name);
    if (it != fSettingMapASIC.end()) {
      uint16_t max12bit = (1 << 12);
      if (newVal > max12bit) {
        cerr << "ERROR: trying to set a value larger than 12 bits"
                  << endl;
        exit(-1);
      }
      // update setting map
      if (fSettingMapASIC.find(name) != fSettingMapASIC.end()) {
        fSettingMapASIC[name].settingASIC[asic].value = newVal;
      } else {
        cerr << "ERROR: trying to modify a setting that not in the "
                     "register map!! "
                  << endl;
        exit(-1);
      }

      Setting setting = fSettingMapASIC[name].settingASIC[asic];

      map<uint8_t, RegisterASIC>::iterator itReg;
      itReg = fRegisterMapASIC.find(static_cast<uint8_t>(setting.regAddr));
      uint32_t val = 0;
      if (itReg != fRegisterMapASIC.end()) val = itReg->second.val[asic];
      // checking if we have to set the full register
      if (setting.nBits == 0x0C && setting.startBit == 0) {
        val = setting.value;
      } else {  // Filling the register partially
        ModifyRegisterPartially(val, setting);
      }
      // update
      itReg->second.val[asic] = uint16_t(val);

    } else {
      cerr << "ERROR: undefined setting name in ASIC definition file: "
                << name << endl;
      //      exit(-1);
    }
  } else {
    it = fSettingMapTriggerASIC.find(name);
    if (it != fSettingMapTriggerASIC.end()) {
      uint16_t max12bit = (1 << 12);
      if (newVal > max12bit) {
        cerr << "ERROR: trying to set a value larger than 12 bits"
                  << endl;
        return TC_ERR_CONF_FAILURE;
      }
      // update setting map
      // update setting map
      if (fSettingMapTriggerASIC.find(name) != fSettingMapTriggerASIC.end()) {
        fSettingMapTriggerASIC[name].settingASIC[asic].value = newVal;
      } else {
        cerr << "ERROR: trying to modify a setting that not in the  "
                     "trigger register map!! "
                  << endl;
        return TC_ERR_CONF_FAILURE;
      }

      Setting setting = fSettingMapTriggerASIC[name].settingASIC[asic];
      // getting the register
      map<uint8_t, RegisterASIC>::iterator itReg;
      itReg =
          fRegisterMapTriggerASIC.find(static_cast<uint8_t>(setting.regAddr));
      uint32_t val = 0;
      if (itReg != fRegisterMapTriggerASIC.end()) val = itReg->second.val[asic];
      // checking if we have to set the full register
      if (setting.nBits == 0x0C && setting.startBit == 0) {
        val = setting.value;
      } else {  // Filling the register partially
        ModifyRegisterPartially(val, setting);
      }
      // update
      itReg->second.val[asic] = uint16_t(val);

    } else {
      cerr << "ERROR: undefined setting name in ASIC definition file: "
                << name << endl;
      //      exit(-1);
    }
  }

  return TC_OK;
}
int RegisterSettings::ModifyFPGASetting(const string& name,
                                        uint32_t newVal) {
  map<string, Setting>::iterator it;
  it = fSettingMapFPGA.find(name);
  if (it != fSettingMapFPGA.end()) {
    // updating settings map

    /// do no bound check if both bounds are equal to zero
    if (!(fSettingMapFPGA[name].lowerBound == 0 &&
          fSettingMapFPGA[name].upperBound == 0)) {
      /// check bounds
      if (newVal > fSettingMapFPGA[name].upperBound ||
          newVal < fSettingMapFPGA[name].lowerBound) {
        cerr << "ERROR: Value of user setting <" << name
                  << "> is out bounds specified in " << fDefintionFileFPGA
                  << endl;
        return TC_ERR_CONF_FAILURE;
      }
    }
    // update the setting
    fSettingMapFPGA[name].value = newVal;
    Setting setting = fSettingMapFPGA[name];
    // update the register value
    map<uint32_t, RegisterFPGA>::iterator itReg;
    itReg = fRegisterMapFPGA.find(setting.regAddr);
    uint32_t val = 0;
    if (itReg != fRegisterMapFPGA.end()) {
      val = fRegisterMapFPGA[setting.regAddr].val;
    }
    // checking if we have to set the full register
    if (setting.nBits == 0x20 && setting.startBit == 0) {
      val = setting.value;
    } else {  // Filling the register partially
      ModifyRegisterPartially(val, setting);
    }
    // update
    fRegisterMapFPGA[setting.regAddr].val = val;
  } else {
    cerr << "ERROR: undefined setting name in FPGA definition file: "
              << name << endl;
    return TC_ERR_CONF_FAILURE;
  }
  return TC_OK;
}
int RegisterSettings::GetFPGARegisterValue(uint32_t addr, uint32_t& val) const {
  RF_cit cit = fRegisterMapFPGA.find(addr);
  if (cit != fRegisterMapFPGA.end()) {
    val = cit->second.val;
  } else {
    cerr << "ERROR: Unknown register address <" << addr << ">"
              << endl;
    return TC_ERR_CONF_FAILURE;
  }

  return TC_OK;
}
int RegisterSettings::GetTriggerASICRegisterValue(const string& name,
                                                  uint8_t asic,
                                                  uint16_t& val) const {
  return GetASICRegisterValue(name, asic, val);
}
int RegisterSettings::GetASICRegisterValue(const string& name,
                                           uint8_t asic, uint16_t& val,
                                           bool isTriggerASIC) const {
  if (asic > 3) {
    cerr << "ERROR, GetASICRegisterValue called with an ASIC > 3 "
              << endl;
    return TC_ERR_CONF_FAILURE;
  }

  uint8_t address;
  GetASICSettingRegisterAddress(name, address, isTriggerASIC);

  if (!isTriggerASIC) {
    RA_cit cit = fRegisterMapASIC.find(address);
    if (cit != fRegisterMapASIC.end()) {
      val = static_cast<uint16_t>(cit->second.val[asic]);
    } else {
      cerr << "ERROR: Unknown register addres 0x" << int(address) << ">"
                << endl;
      return TC_ERR_CONF_FAILURE;
    }
  } else {
    RA_cit cit = fRegisterMapTriggerASIC.find(address);
    if (cit != fRegisterMapTriggerASIC.end()) {
      val = static_cast<uint16_t>(cit->second.val[asic]);
    } else {
      cerr << "ERROR: Unknown register addres 0x" << int(address) << ">"
                << endl;
      return TC_ERR_CONF_FAILURE;
    }
  }
  return TC_OK;
}
int RegisterSettings::GetFPGASettingRegisterAddress(const string& name,
                                                    uint32_t& addr) const {
  SF_cit cit = fSettingMapFPGA.find(name);
  if (cit != fSettingMapFPGA.end()) {
    addr = cit->second.regAddr;
  } else {
    cerr << "ERROR: Unknown settings <" << name << ">" << endl;
    return TC_ERR_CONF_FAILURE;
  }
  return TC_OK;
}
int RegisterSettings::GetTriggerASICSettingRegisterAddress(
    const string& name, uint8_t& addr) const {
  return GetASICSettingRegisterAddress(name, addr, true);
}
int RegisterSettings::GetASICSettingRegisterAddress(const string& name,
                                                    uint8_t& addr,
                                                    bool isTriggerASIC) const {
  if (!isTriggerASIC) {
    SA_cit cit = fSettingMapASIC.find(name);
    if (cit != fSettingMapASIC.end()) {
      addr = static_cast<uint8_t>(cit->second.settingASIC[0].regAddr);
    } else {
      cerr << "ERROR: Unknown settings <" << name << ">" << endl;
      return TC_ERR_CONF_FAILURE;
    }
  } else {
    SA_cit cit = fSettingMapTriggerASIC.find(name);
    if (cit != fSettingMapTriggerASIC.end()) {
      addr = static_cast<uint8_t>(cit->second.settingASIC[0].regAddr);
    } else {
      cerr << "ERROR: Unknown settings <" << name << ">" << endl;
      return TC_ERR_CONF_FAILURE;
    }
  }
  return TC_OK;
}
void RegisterSettings::PrintSetting(Setting set) const {
  cout << hex << "regAddr: 0x" << set.regAddr << "\n";
  cout << hex << "nBits: 0x" << set.nBits << "(hex), " << dec
            << set.nBits << "(dec)\n";
  cout << hex << "startBit: 0x" << set.startBit << "\n";
  cout << hex << "value: 0x" << set.value << "\n";
  //  cout << hex << "access: 0x" << set.access << "\n";
  cout << hex << "lowerBound: 0x" << set.lowerBound << "\n";
  cout << hex << "upperBound: 0x" << set.upperBound << "\n";
  cout << dec << "multiplier: " << set.multiplier << "\n";
  cout << dec << "offset: " << set.offset << "\n";
  cout << "description: " << set.description << endl;
}
void RegisterSettings::PrintFPGASetting(const string& settingName) const {
  SF_cit cit = fSettingMapFPGA.find(settingName);
  if (cit != fSettingMapFPGA.end()) {
    cout << "FPGA Settings Name: " << settingName << "\n";
    PrintSetting(cit->second);
  } else {
    cerr << "WARNING: Unknown name of settings: " << settingName
              << endl;
  }
}
void RegisterSettings::PrintTriggerASICSetting(
    const string& settingName) const {
  PrintASICSetting(settingName, true);
}
void RegisterSettings::PrintASICSetting(const string& settingName,
                                        bool isTriggerASIC) const {
  if (!isTriggerASIC) {
    SA_cit cit = fSettingMapASIC.find(settingName);
    if (cit != fSettingMapASIC.end()) {
      cout << "ASIC Settings Name: " << settingName << "\n";
      for (int i = 0; i < 4; ++i) {
        cout << "ASIC " << i << "\n";
        PrintSetting(cit->second.settingASIC[i]);
      }
    } else {
      cerr << "WARNING: Unknown name of settings: " << settingName
                << endl;
    }
  } else {
    SA_cit cit = fSettingMapTriggerASIC.find(settingName);
    if (cit != fSettingMapTriggerASIC.end()) {
      cout << "ASIC Settings Name: " << settingName << "\n";
      for (int i = 0; i < 4; ++i) {
        cout << "ASIC " << i << "\n";
        PrintSetting(cit->second.settingASIC[i]);
      }
    } else {
      cerr << "WARNING: Unknown name of settings: " << settingName
                << endl;
    }
  }
}
void RegisterSettings::PrintAllSettings() const {
  cout << "\n------FPGA SETTINGS -----\n" << endl;
  PrintHeaderFPGA();
  for (SF_cit cit = fSettingMapFPGA.cbegin(); cit != fSettingMapFPGA.cend();
       ++cit) {
    PrintFPGASetting(cit->first);
  }
  cout << "\n------ASIC SETTINGS -----\n" << endl;
  PrintHeaderASIC();
  for (SA_cit cit = fSettingMapASIC.cbegin(); cit != fSettingMapASIC.end();
       ++cit) {
    PrintASICSetting(cit->first);
  }

  if (fDefintionFileTriggerASIC != "")
    cout << "\n------TRIGGER ASIC SETTINGS -----\n" << endl;
  PrintHeaderTriggerASIC();
  for (SA_cit cit = fSettingMapTriggerASIC.cbegin();
       cit != fSettingMapTriggerASIC.end(); ++cit) {
    PrintASICSetting(cit->first, true);
  }
}
void RegisterSettings::PrintAllRegisters() const {
  cout << "\n Registers FPGA " << endl;
  for (auto it = fRegisterMapFPGA.begin(); it != fRegisterMapFPGA.end(); it++) {
    cout << hex << "0x" << int(it->first) << ":\t0x" << it->second.val
              << endl;
  }

  cout << "\n ASIC " << endl;
  for (auto it = fRegisterMapASIC.begin(); it != fRegisterMapASIC.end(); it++) {
    cout << hex << "0x" << int(it->first) << ":\t0x"
              << it->second.val[0] << "\t 0x" << it->second.val[1] << "\t 0x"
              << it->second.val[2] << "\t0x" << it->second.val[3] << endl;
  }

  if (fDefintionFileTriggerASIC != "") {
    cout << "\n TRIGGER ASIC " << endl;
    for (auto it = fRegisterMapTriggerASIC.begin();
         it != fRegisterMapTriggerASIC.end(); it++) {
      cout << hex << "0x" << int(it->first) << ":\t0x"
                << it->second.val[0] << "\t 0x" << it->second.val[1] << "\t 0x"
                << it->second.val[2] << "\t0x" << it->second.val[3]
                << endl;
    }
  }
}
void RegisterSettings::PrintHeaderFPGA() const {
  cout << "HEADER FPGA\n";
  cout << "TM_FIRMWARE_VERSION: " << hex << fFPGAFirmwareVersion
            << "\n";
  cout << "DESCRIPTION: " << fFPGADescription << "\n";
  cout << "RESPONSIBLE_AUTHOR: " << fFPGAAuthor << "\n";
  cout << "NUM_REGISTERS: " << fFPGANumberOfRegisters << endl;
}
void RegisterSettings::PrintHeaderTriggerASIC() const { PrintHeaderASIC(true); }
void RegisterSettings::PrintHeaderASIC(bool isTriggerASIC) const {
  cout << "HEADER ASIC"
            << "\n";
  if (!isTriggerASIC) {
    cout << "DESCRIPTION: " << fASICDescription << "\n";
    cout << "RESPONSIBLE_AUTHOR: " << fASICAuthor << "\n";
    cout << "NUM_REGISTERS: " << fASICNumberOfRegisters << endl;
  } else {
    cout << "DESCRIPTION: " << fTriggerASICDescription << "\n";
    cout << "RESPONSIBLE_AUTHOR: " << fTriggerASICAuthor << "\n";
    cout << "NUM_REGISTERS: " << fTriggerASICNumberOfRegisters
              << endl;
  }
}
void RegisterSettings::GenerateTriggerASICMarkdown(
    const string& fname) const {
  GenerateASICMarkdown(fname, true);
}
void RegisterSettings::GenerateASICMarkdown(const string& fname,
                                            bool isTriggerASIC) const {
  map<uint32_t, string> sorted;
  typedef map<uint32_t, string>::const_iterator s_cit;
  if (!isTriggerASIC) {
    for (SA_cit cit = fSettingMapASIC.cbegin(); cit != fSettingMapASIC.cend();
         ++cit) {
      sorted[cit->second.settingASIC[0].regAddr * 100 + 32 -
             cit->second.settingASIC[0].startBit] = cit->first;
    }
  } else {
    for (SA_cit cit = fSettingMapTriggerASIC.cbegin();
         cit != fSettingMapTriggerASIC.cend(); ++cit) {
      sorted[cit->second.settingASIC[0].regAddr * 100 + 32 -
             cit->second.settingASIC[0].startBit] = cit->first;
    }
  }

  ofstream fout;
  fout.open(fname.c_str());

  fout << "Register Map of a TARGET ASIC: " << fDefintionFileASIC << "\n";
  fout << "============\n";

  //  fout << "|Register|Name|Bit Range|Default|Read Only|Description|\n";
  fout << "|Register|Name|Bit Range|Default|Access   |Description|\n";
  fout << "|--------|----|---------|-------|---------|-----------|\n";

  uint32_t previous_reg = numeric_limits<uint32_t>::max();
  for (s_cit cit = sorted.cbegin(); cit != sorted.cend(); ++cit) {
    string name = cit->second;
    Setting setting;
    if (!isTriggerASIC) {
      setting = fSettingMapASIC.at(name).settingASIC[0];
    } else {
      setting = fSettingMapTriggerASIC.at(name).settingASIC[0];
    }

    uint32_t reg = cit->first / 100;
    if (reg != previous_reg) {
      fout << "|0x" << setfill('0') << setw(2) << hex
           << uppercase << reg << "|";
      previous_reg = reg;
    } else {
      fout << "||";
    }
    fout << "`" << name << "`|";
    fout << dec;
    if (setting.nBits == 1) {
      fout << setting.startBit << "|";
    } else {
      fout << setting.startBit + setting.nBits - 1 << "--" << setting.startBit
           << "|";
    }
    fout << "0x" << setfill('0') << setw(3) << hex
         << uppercase << setting.value << "|";
    //    fout << (setting.isReadOnly ? "Yes" : "No") << "|";
    if (setting.access == eRW) {
      fout << "Read/Write"
           << "|";
    } else if (setting.access == eR) {
      fout << "Read"
           << "|";
    } else if (setting.access == eW) {
      fout << "Write"
           << "|";
    } else {
      fout << "???"
           << "|";
    }
    //    fout << (setting.isReadOnly ? "Yes" : "No") << "|";
    fout << setting.description << "|\n";
  }

  fout.close();
}
void RegisterSettings::GenerateFPGAMarkdown(const string& fname) const {
  map<uint32_t, string> sorted;
  map<string, Setting>::const_iterator cit;
  for (cit = fSettingMapFPGA.cbegin(); cit != fSettingMapFPGA.cend(); ++cit) {
    sorted[cit->second.regAddr * 100 + 32 - cit->second.startBit] = cit->first;
  }

  ofstream fout;
  fout.open(fname.c_str());

  fout << "Register Map of a TARGET FPGA: " << fDefintionFileFPGA << "\n";
  fout << "============\n";
  fout << "|Register|Name|Bit Range|Default|Read Only|Description|\n";
  fout << "|--------|----|---------|-------|---------|-----------|\n";

  map<uint32_t, string>::const_iterator cit2;
  uint32_t previous_reg = numeric_limits<uint32_t>::max();
  for (cit2 = sorted.cbegin(); cit2 != sorted.cend(); ++cit2) {
    string name = cit2->second;
    Setting setting = fSettingMapFPGA.at(name);
    uint32_t reg = cit2->first / 100;
    if (reg != previous_reg) {
      fout << "|0x" << setfill('0') << setw(2) << hex
           << uppercase << reg << "|";
      previous_reg = reg;
    } else {
      fout << "||";
    }
    fout << "`" << name << "`|";
    fout << dec;
    if (setting.nBits == 1) {
      fout << setting.startBit << "|";
    } else {
      fout << setting.startBit + setting.nBits - 1 << "--" << setting.startBit
           << "|";
    }
    fout << "0x" << setfill('0') << setw(3) << hex
         << uppercase << setting.value << "|";
    //    fout << (setting.isReadOnly ? "Yes" : "No") << "|";
    if (setting.access == eRW) {
      fout << "Read/Write"
           << "|";
    } else if (setting.access == eR) {
      fout << "Read"
           << "|";
    } else if (setting.access == eW) {
      fout << "Write"
           << "|";
    } else {
      fout << "???"
           << "|";
    }
    fout << setting.description << "|\n";
  }
  fout.close();
}

