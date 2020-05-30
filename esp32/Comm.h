#ifndef COMM_H
#define COMM_H
class Comm{
  public:
    const byte CMD_START_SAMPLE;
    const byte CMD_STOP_SAMPLE;
    const byte CMD_CHECK_BAT;
    const byte CMD_SET_VOLTAGE;
    const byte CMD_SET_SAMPLE_PARAS;
    //pen to software
    const byte CMD_DATA;
    const byte CMD_REPORT_BAT;
    const byte CMD_SWIPE_UP;
    const byte CMD_SWIPE_DOWN;
    const byte CMD_CHANGE_SEL;
    
    const int HEADER_SIZE;
    const int HEADER_SIZE_FIELD;
    const int HEADER_TYPE_FIELD;

    void parseBigEndian(byte *input, uint32_t &output);
    void constructHeader(byte* header, const uint32_t &dataSize, const short int &dataType)
}

Comm::CMD_START_SAMPLE = 0x21;
Comm::CMD_STOP_SAMPLE = 0x22;
Comm::CMD_CHECK_BAT = 0x23;
Comm::CMD_SET_VOLTAGE = 0x41;
Comm::CMD_SET_SAMPLE_PARAS = 0x42;

Comm::CMD_DATA = 0x00;
Comm::CMD_REPORT_BAT = 0x01;
Comm::CMD_SWIPE_UP = 0x11;
Comm::CMD_SWIPE_DOWN = 0x12;
Comm::CMD_CHANGE_SEL = 0x13;

Comm::HEADER_SIZE = 5;
Comm::HEADER_SIZE_FIELD = 4;
Comm::HEADER_TYPE_FIELD = 1;



#endif
