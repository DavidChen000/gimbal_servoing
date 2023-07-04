#ifndef GIMBAL_PARSER_H
#define GIMBAL_PARSER_H

#include "command.h"
#include "crc32.h"
#include "stdio.h"
#include <inttypes.h>
#include <string.h>

typedef enum {
  PARSER_NO_ERROR = 0,
  PARSER_ERROR_PROTOCOL = 1,
  PARSER_ERROR_WRONG_CMD_SIZE = 2,
  PARSER_ERROR_BUFFER_IS_FULL = 3,
  PARSER_ERROR_WRONG_DATA_SIZE = 4,
} GIMBAL_parser_errors;

class IOStream {
public:
  // Methods need to be implemented
  virtual uint16_t getBytesAvailable() = 0;
  virtual uint8_t readByte() = 0;
  virtual void writeByte(uint8_t b) = 0;

  int16_t readWord() {
    return (uint16_t)readByte() + ((uint16_t)readByte() << 8);
  }

#ifdef SYS_LITTLE_ENDIAN
  // Optimization for little-endian machines only!
  inline void readWordArr(int16_t *arr, uint8_t size) {
    readBuf(arr, (size * 2));
  }
#else
  void readWordArr(int16_t *arr, uint8_t size) {
    for (uint8_t i = 0; i < size; i++) {
      arr[i] = readWord();
    }
  }
#endif

  int32_t readLong() {
    return (int32_t)readByte() + ((int32_t)readByte() << 8) +
           ((int32_t)readByte() << 16) + ((int32_t)readByte() << 24);
  }

  void readBuf(void *buf, uint8_t size) {
    for (uint8_t i = 0; i < size; i++) {
      ((uint8_t *)buf)[i] = readByte();
    }
  }

  float readFloat() {
    float f;
    readBuf(&f, sizeof(float));
    return f;
  }

  void skipBytes(uint8_t size) {
    while (size-- > 0) {
      readByte();
    }
  }

  void writeWord(int16_t w) {
    writeByte(w);      // low
    writeByte(w >> 8); // high
  }

#ifdef SYS_LITTLE_ENDIAN
  // Optimization for little-endian machines only!
  inline void writeWordArr(int16_t *arr, uint8_t size) {
    writeBuf(arr, (size * 2));
  }
#else
  void writeWordArr(int16_t *arr, uint8_t size) {
    for (uint8_t i = 0; i < size; i++) {
      writeWord(arr[i]);
    }
  }
#endif

  void writeLong(int32_t dw) {
    writeWord(dw);       // low word
    writeWord(dw >> 16); // high word
  }

  void writeFloat(float f) { writeBuf(&f, sizeof(float)); }

  void writeBuf(const void *buf, uint8_t size) {
    for (uint8_t i = 0; i < size; i++) {
      writeByte(((uint8_t *)buf)[i]);
    }
  }

  void writeString(const char *str) {
    uint8_t len = strlen(str);
    writeByte(len);
    writeBuf(str, len);
  }

  void writeEmptyBuf(uint8_t size) {
    while (size-- > 0) {
      writeByte(0);
    }
  }
};

/* Class to manipulate command data */
class SerialCommand : public IOStream {
public:
  uint8_t pos;
  uint8_t id;
  uint8_t version;
  uint8_t len;
  uint8_t cmd;
  uint8_t data[GIMBAL_CMD_DATA_SIZE];

  /* Check if limit reached after reading data buffer */
  inline uint8_t checkLimit() { return len == pos; }

  inline uint16_t getBytesAvailable() { return len - pos; }

  void init(uint8_t _id) {
    id = _id;
    len = 0;
    pos = 0;
  }

  inline void reset() {
    len = 0;
    pos = 0;
  }

  uint8_t readByte() {
    if (pos < len) {
      return data[pos++];
    } else {
      pos++;
      return 0;
    }
  }

  void writeByte(uint8_t b) {
    if (len < sizeof(data)) {
      data[len++] = b;
    }
  }

  static inline uint32_t data_checksum(const unsigned char *pData,
                                       unsigned short Length) {
    return CRC32Software(pData, Length);
  }
};
class GIMBAL_ComObj : public IOStream {
public:
  // Return the number of bytes received in input buffer
  virtual uint16_t getBytesAvailable() = 0;
  // Read byte from the input stream
  virtual uint8_t readByte() = 0;
  // Write byte to the output stream
  virtual void writeByte(uint8_t b) = 0;
  // Return the space available in the output buffer. Return 0xFFFF if unknown.
  virtual uint16_t getOutEmptySpace() = 0;
};

/* Optimized version of a parser, that does not require a separate buffer for
 * maintain the parsed data */
class GIMBAL_Parser {
  GIMBAL_ComObj *com_obj;
  enum {
    STATE_WAIT,
    STATE_GOT_VERSION,
    STATE_GOT_LENGTH,
    STATE_GOT_CMD,
    STATE_CHEACK_HEAD,
    STATE_GOT_DATA
  } state;
  uint16_t len;
  uint32_t checkcrc32 = 0;
  uint32_t checksum = 1;
  uint16_t parser_error_count;
  int imu_cout = 0;
  int rotor_cout = 0;

public:
  SerialCommand in_cmd; // received command is stored here

  inline void init(GIMBAL_ComObj *_com_obj) {
    com_obj = _com_obj;
    state = STATE_WAIT;
    parser_error_count = 0;
    printf("state init ..\n");
  }

  inline void onParseError(uint8_t error = PARSER_ERROR_PROTOCOL) {
    parser_error_count++;
    printf("parser_error_count\n");
  }

  /*
   * Parses next character in a stream.
   * Returns 1 if command successfully parsed, 0 otherwise.
   * Calls  onParseError() in case of errors
   */
  inline uint8_t process_char(uint8_t c) {
    switch (state) {
    case STATE_WAIT:
      if (GIMBAL_HEAD == c) {

        in_cmd.init(c); // got command id
        state = STATE_GOT_VERSION;
      } else {
        // onParseError();
        printf("onParseError STATE_WAIT:%d, %x\n", state, c);
      }
      break;
    case STATE_GOT_VERSION:

      if (GIMBAL_VERSION == c) {
        in_cmd.version = c;
        state = STATE_GOT_LENGTH;
      } else {
        state = STATE_WAIT;
      }
      break;

    case STATE_GOT_LENGTH:
      if (c < GIMBAL_CMD_MAX_BYTES) {
        len = c + 4;
        state = STATE_GOT_CMD;
      } else {
        state = STATE_WAIT;
      }
      break;
    case STATE_GOT_CMD:
      if (GIMBAL_IMU_AND_ROTOR_ANGLE == c) {
        in_cmd.cmd = c;
        state = STATE_CHEACK_HEAD;
      } else {
        state = STATE_WAIT;
      }
      break;
    case STATE_CHEACK_HEAD:
      if ((in_cmd.version + len + in_cmd.cmd - 4) == c) {
        state = STATE_GOT_DATA;
      } else {
        state = STATE_WAIT;
      }
      break;
    case STATE_GOT_DATA:
      in_cmd.data[in_cmd.len++] = c;
      len--;
      if (0 == len) {
        state = STATE_WAIT;
        checksum = SerialCommand::data_checksum(in_cmd.data, in_cmd.len - 4);
        memcpy(&checkcrc32, &in_cmd.data[in_cmd.len - 4], 4);

        if (checksum == checkcrc32) {
          return 1;
        } else {
          onParseError();
        }
      }

      break;
    }
    return 0;
  }

  /* Parse and fill SerialCommand object GIMBAL_Parser.in_cmd;
   * Returns 1 if command is parsed, 0 otherwise.
   */
  inline int8_t read_cmd() {
    while (com_obj->getBytesAvailable()) {
      if (process_char(com_obj->readByte())) {
        return 1;
      }
    }
    return 0;
  }

  /*
   * Formats and writes command to serial port.
   * If wait=1, waits even if output buffer is full.
   * If wait=0, writes only if output buffer has enough space to accept this
   * command. Returns 0 on success, PARSER_ERR_xx on fail.
   */
  uint8_t send_command(uint8_t cmd_id, uint8_t version, uint8_t cmd, void *data,
                       uint16_t size, uint8_t wait = 1) {
    if (com_obj != NULL &&
        size <= (GIMBAL_CMD_MAX_BYTES - GIMBAL_CMD_NON_PAYLOAD_BYTES)) {
      if (wait ||
          com_obj->getOutEmptySpace() >= size + GIMBAL_CMD_NON_PAYLOAD_BYTES) {

        com_obj->writeByte(cmd_id);               // command id
        com_obj->writeByte(version);              // Protocol version
        com_obj->writeByte(size);                 // data body length
        com_obj->writeByte(cmd);                  // command
        com_obj->writeByte(version + size + cmd); // header checksum

        for (uint8_t i = 0; i < size; i++) {
          com_obj->writeByte(((uint8_t *)data)[i]);
        }

        checksum = CRC32Software((uint8_t *)data, size);
        com_obj->writeLong(checksum);
        return 0;
      } else {
        return PARSER_ERROR_BUFFER_IS_FULL;
      }
    } else {
      return PARSER_ERROR_WRONG_CMD_SIZE;
    }
  }

  /*
   * Send command from the SerialCommand object to serial port. See
   * send_command() description.
   */
  inline uint8_t send_cmd(SerialCommand &cmd, uint8_t wait = 1) {
    return send_command(cmd.id, cmd.version, cmd.cmd, cmd.data, cmd.len, wait);
  }

  /*
   * Get parsing errors counter
   */
  inline uint16_t get_parse_error_count() { return parser_error_count; }

  /*
   * Resets the state of a parser
   */
  inline void reset() { state = STATE_WAIT; }

  /*
   * Returns the space available in the output buffer
   */
  inline uint16_t get_out_empty_space() {
    return com_obj != NULL ? com_obj->getOutEmptySpace() : 0;
  }
};

#endif // GIMBAL_PARSER_H
