#ifndef __BEAKER_EEPROM_HELPER__
#define __BEAKER_EEPROM_HELPER__

class EepromHelper {
	public:

	static unsigned int storeFloat(float value, unsigned int startingAddress){
    unsigned int addr = startingAddress;
    EEPROM.put(addr, value);
    addr += sizeof(float);

    return addr;
	}

  static unsigned int loadFloat(float &value, unsigned int startingAddress){
    unsigned int addr = startingAddress;
    EEPROM.get(addr, value);
    addr += sizeof(float);

    return addr;
  }

};

#endif