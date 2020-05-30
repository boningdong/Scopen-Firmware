void parseBigEndian(uint8_t *input, uint32_t &output){
  output =(input[0] << 24) | (input[1] << 16) | (input[2]<<8) | (input[3]);
}
void constructHeader(uint8_t* header, const uint32_t &dataSize, const short int &dataType){
  header[3] = dataSize >> 32; 
  header[2] = dataSize >> 40; 
  header[1] = dataSize >> 48; 
  header[0] = dataSize >> 56;
  header[4] = dataType;
}
