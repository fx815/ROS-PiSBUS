/*
SBUS.cpp
Brian R Taylor
brian.taylor@bolderflight.com

Copyright (c) 2016 Bolder Flight Systems

Permission is hereby granted, free of charge, to any person obtaining a copy of this software
and associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <string.h>     // string function definitions
#include <unistd.h>     // UNIX standard function definitions
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <inttypes.h>
#include <asm/ioctls.h>
#include <asm/termbits.h>
#include <sys/ioctl.h>

#include "ROS-PiSBUS/SBUS.h"

using namespace std;

namespace SBUS
{

SBUS::SBUS(string tty)
{
    _tty = tty;

    for(int i=0;i<_numChannels;i++)
        _useWriteCoeff[i] = false;
}

SBUS::~SBUS()
{
    close(_fd);

    if (_readCoeff) {
        for (uint8_t i = 0; i < _numChannels; i++) {
            if (_readCoeff[i]) {
                delete[] _readCoeff[i];
            }
        }
        delete[] _readCoeff;
    }
    if (_writeCoeff) {
        for (uint8_t i = 0; i < _numChannels; i++) {
            if (_writeCoeff[i]) {
                delete[] _writeCoeff[i];
            }
        }
        delete[] _writeCoeff;
    }
}

/* starts the serial communication */
int SBUS::begin()
{
    // initialize parsing state
    _parserState = 0;
    // initialize default scale factors and biases
    for (uint8_t i = 0; i < _numChannels; i++)
    {
        setEndPoints(i, _defaultMin, _defaultMax);
    }

    // begin the serial port for SBUS
    #if defined(__MK20DX128__) || defined(__MK20DX256__)  // Teensy 3.0 || Teensy 3.1/3.2
        _bus->begin(_sbusBaud,SERIAL_8E1_RXINV_TXINV);
        SERIALPORT = _bus;
    #elif defined(__MK64FX512__) || defined(__MK66FX1M0__) || defined(__MKL26Z64__)  // Teensy 3.5 || Teensy 3.6 || Teensy LC
        _bus->begin(_sbusBaud,SERIAL_8E2_RXINV_TXINV);
    #elif defined(STM32L496xx) || defined(STM32L476xx) || defined(STM32L433xx) || defined(STM32L432xx)  // STM32L4
        _bus->begin(_sbusBaud,SERIAL_SBUS);
    #elif defined(_BOARD_MAPLE_MINI_H_) // Maple Mini
        _bus->begin(_sbusBaud,SERIAL_8E2);
    #elif defined(__RASPBERRYPI_ZERO_W__)
        // open file descriptor
        _fd = open(_tty.c_str(), O_RDWR| O_NOCTTY);

        if(_fd < 0 )
        {
            cerr << "Error " << errno << " opening " << _tty << ": " << strerror(errno) << endl;
            return -1;
        }

        // set custom baudrate
        struct termios2 tio;
        int r = ioctl(_fd, TCGETS2, &tio);
        if(r)
        {
            cerr << "TCGETS2" << endl;
            return -1;
        }

        tio.c_cflag &= ~CBAUD;
        tio.c_cflag |= BOTHER;
        tio.c_cflag |= CSTOPB; // 2 stop bits
        tio.c_cflag |= PARENB; // enable parity bit, even by default
        tio.c_ispeed = tio.c_ospeed = _sbusBaud;

        r = ioctl(_fd, TCSETS2, &tio);
        if(r)
        {
            cerr << "TCSETS2" << endl;
            return -1;
        }
    #endif

    return 0;
}

/* read the SBUS data and calibrate it to +/- 1 */
bool SBUS::readCal(float* calChannels, uint8_t* failsafe, uint16_t* lostFrame)
{
    uint16_t channels[_numChannels];
    // read the SBUS data
    if(read(&channels[0],failsafe,lostFrame)) {
        // linear calibration
        for (uint8_t i = 0; i < _numChannels; i++) {
            calChannels[i] = channels[i] * _sbusScale[i] + _sbusBias[i];
            if (_useReadCoeff[i]) {
                calChannels[i] = PolyVal(_readLen[i],_readCoeff[i],calChannels[i]);
            }
        }
        // return true on receiving a full packet
        return true;
  } else {
        // return false if a full packet is not received
        return false;
  }
}

/* read the SBUS data */
bool SBUS::read(uint16_t* channels, uint8_t* failsafe, uint16_t* lostFrame)
{
    // parse the SBUS packet
    if (parse()) {
        if (channels) {
            // 16 channels of 11 bit data
            channels[0]  = (uint16_t) ((_payload[0]    |_payload[1] <<8)                     & 0x07FF);
            channels[1]  = (uint16_t) ((_payload[1]>>3 |_payload[2] <<5)                     & 0x07FF);
            channels[2]  = (uint16_t) ((_payload[2]>>6 |_payload[3] <<2 |_payload[4]<<10)    & 0x07FF);
            channels[3]  = (uint16_t) ((_payload[4]>>1 |_payload[5] <<7)                     & 0x07FF);
            channels[4]  = (uint16_t) ((_payload[5]>>4 |_payload[6] <<4)                     & 0x07FF);
            channels[5]  = (uint16_t) ((_payload[6]>>7 |_payload[7] <<1 |_payload[8]<<9)     & 0x07FF);
            channels[6]  = (uint16_t) ((_payload[8]>>2 |_payload[9] <<6)                     & 0x07FF);
            channels[7]  = (uint16_t) ((_payload[9]>>5 |_payload[10]<<3)                     & 0x07FF);
            channels[8]  = (uint16_t) ((_payload[11]   |_payload[12]<<8)                     & 0x07FF);
            channels[9]  = (uint16_t) ((_payload[12]>>3|_payload[13]<<5)                     & 0x07FF);
            channels[10] = (uint16_t) ((_payload[13]>>6|_payload[14]<<2 |_payload[15]<<10)   & 0x07FF);
            channels[11] = (uint16_t) ((_payload[15]>>1|_payload[16]<<7)                     & 0x07FF);
            channels[12] = (uint16_t) ((_payload[16]>>4|_payload[17]<<4)                     & 0x07FF);
            channels[13] = (uint16_t) ((_payload[17]>>7|_payload[18]<<1 |_payload[19]<<9)    & 0x07FF);
            channels[14] = (uint16_t) ((_payload[19]>>2|_payload[20]<<6)                     & 0x07FF);
            channels[15] = (uint16_t) ((_payload[20]>>5|_payload[21]<<3)                     & 0x07FF);
        }
        if (lostFrame) {
        // count lost frames
        if (_payload[22] & _sbusLostFrame) {
        *lostFrame = true;
        } else {
                *lostFrame = false;
            }
        }
        if (failsafe) {
        // failsafe state
        if (_payload[22] & _sbusFailSafe) {
            *failsafe = true;
        }
        else{
            *failsafe = false;
        }
        }
        // return true on receiving a full packet
        return true;
    } else {
        // return false if a full packet is not received
        return false;
    }
}

/* parse the SBUS data */
bool SBUS::parse()
{
    #if !defined(__RASPBERRYPI_ZERO_W__)
        // reset the parser state if too much time has passed
        static elapsedMicros _sbusTime = 0;
        if (_sbusTime > SBUS_TIMEOUT_US) {_parserState = 0;}
        // see if serial data is available
        while (_bus->available() > 0) {
            _sbusTime = 0;
            _curByte = _bus->read();
            // find the header
            if (_parserState == 0) {
                    if ((_curByte == _sbusHeader) && ((_prevByte == _sbusFooter) || ((_prevByte & _sbus2Mask) == _sbus2Footer))) {
                        _parserState++;
                    } else {
                        _parserState = 0;
                    }
            } else {
                // strip off the data
                if ((_parserState-1) < _payloadSize) {
                    _payload[_parserState-1] = _curByte;
                    _parserState++;
                }
                // check the end byte
                if ((_parserState-1) == _payloadSize) {
                    if ((_curByte == _sbusFooter) || ((_curByte & _sbus2Mask) == _sbus2Footer)) {
                        _parserState = 0;
                        return true;
                    } else {
                        _parserState = 0;
                        return false;
                    }
                }
            }
            _prevByte = _curByte;
        }
        // return false if a partial packet
        return false;
    #endif
}

/* write SBUS packets */
void SBUS::write(uint16_t* channels)
{
    static uint8_t packet[25];
    /* assemble the SBUS packet */
    // SBUS header
    packet[0] = _sbusHeader;
    // 16 channels of 11 bit data
    if (channels)
    {
        packet[1] = (uint8_t) ((channels[0] & 0x07FF));
        packet[2] = (uint8_t) ((channels[0] & 0x07FF)>>8 | (channels[1] & 0x07FF)<<3);
        packet[3] = (uint8_t) ((channels[1] & 0x07FF)>>5 | (channels[2] & 0x07FF)<<6);
        packet[4] = (uint8_t) ((channels[2] & 0x07FF)>>2);
        packet[5] = (uint8_t) ((channels[2] & 0x07FF)>>10 | (channels[3] & 0x07FF)<<1);
        packet[6] = (uint8_t) ((channels[3] & 0x07FF)>>7 | (channels[4] & 0x07FF)<<4);
        packet[7] = (uint8_t) ((channels[4] & 0x07FF)>>4 | (channels[5] & 0x07FF)<<7);
        packet[8] = (uint8_t) ((channels[5] & 0x07FF)>>1);
        packet[9] = (uint8_t) ((channels[5] & 0x07FF)>>9 | (channels[6] & 0x07FF)<<2);
        packet[10] = (uint8_t) ((channels[6] & 0x07FF)>>6 | (channels[7] & 0x07FF)<<5);
        packet[11] = (uint8_t) ((channels[7] & 0x07FF)>>3);
        packet[12] = (uint8_t) ((channels[8] & 0x07FF));
        packet[13] = (uint8_t) ((channels[8] & 0x07FF)>>8 | (channels[9] & 0x07FF)<<3);
        packet[14] = (uint8_t) ((channels[9] & 0x07FF)>>5 | (channels[10] & 0x07FF)<<6);
        packet[15] = (uint8_t) ((channels[10] & 0x07FF)>>2);
        packet[16] = (uint8_t) ((channels[10] & 0x07FF)>>10 | (channels[11] & 0x07FF)<<1);
        packet[17] = (uint8_t) ((channels[11] & 0x07FF)>>7 | (channels[12] & 0x07FF)<<4);
        packet[18] = (uint8_t) ((channels[12] & 0x07FF)>>4 | (channels[13] & 0x07FF)<<7);
        packet[19] = (uint8_t) ((channels[13] & 0x07FF)>>1);
        packet[20] = (uint8_t) ((channels[13] & 0x07FF)>>9 | (channels[14] & 0x07FF)<<2);
        packet[21] = (uint8_t) ((channels[14] & 0x07FF)>>6 | (channels[15] & 0x07FF)<<5);
        packet[22] = (uint8_t) ((channels[15] & 0x07FF)>>3);
    }
    // flags
    packet[23] = 0x00;
    // footer
    packet[24] = _sbusFooter;
    #if defined(__MK20DX128__) || defined(__MK20DX256__) // Teensy 3.0 || Teensy 3.1/3.2
        // use ISR to send byte at a time,
        // 130 us between bytes to emulate 2 stop bits
        noInterrupts();
        memcpy(&PACKET,&packet,sizeof(packet));
        interrupts();
        serialTimer.priority(255);
        serialTimer.begin(sendByte,130);
    #elif defined(__MK64FX512__) || defined(__MK66FX1M0__) || defined(__MKL26Z64__) || defined(STM32L496xx) || defined(STM32L476xx) || defined(STM32L433xx) || defined(STM32L432xx) || defined(_BOARD_MAPLE_MINI_H_)  // Teensy 3.5 || Teensy 3.6 || Teensy LC || STM32L4 || Maple Mini
        // write packet
        _bus->write(packet,25);
    #elif defined(__RASPBERRYPI_ZERO_W__)
        ::write(_fd, packet, sizeof(packet));
    #endif
}

/* write SBUS packets from calibrated inputs */
void SBUS::writeCal(float* calChannels)
{
    uint16_t channels[_numChannels] = {0};
    // linear calibration
    if (calChannels)
    {
        for (uint8_t i=0; i<_numChannels; i++)
        {
            if (_useWriteCoeff[i])
            {
                calChannels[i] = PolyVal(_writeLen[i], _writeCoeff[i], calChannels[i]);
            }
            channels[i] = (calChannels[i] - _sbusBias[i])  / _sbusScale[i];
        }
    }

    write(channels);
}

void SBUS::setEndPoints(uint8_t channel,uint16_t min,uint16_t max)
{
    _sbusMin[channel] = min;
    _sbusMax[channel] = max;
    scaleBias(channel);
}

void SBUS::getEndPoints(uint8_t channel,uint16_t *min,uint16_t *max)
{
    if (min&&max) {
        *min = _sbusMin[channel];
        *max = _sbusMax[channel];
    }
}

void SBUS::setReadCal(uint8_t channel,float *coeff,uint8_t len)
{
    if (coeff) {
        if (!_readCoeff) {
            _readCoeff = new float*[_numChannels];
        }
        if (!_readCoeff[channel]) {
            _readCoeff[channel] = new float[len];
        } else {
            delete[] _readCoeff[channel];
            _readCoeff[channel] = new float[len];
        }
        for (uint8_t i = 0; i < len; i++) {
            _readCoeff[channel][i] = coeff[i];
        }
        _readLen[channel] = len;
        _useReadCoeff[channel] = true;
    }
}

void SBUS::getReadCal(uint8_t channel,float *coeff,uint8_t len)
{
    if (coeff) {
        for (uint8_t i = 0; (i < _readLen[channel]) && (i < len); i++) {
            coeff[i] = _readCoeff[channel][i];
        }
    }
}

void SBUS::setWriteCal(uint8_t channel,float *coeff,uint8_t len)
{
    if (coeff) {
        if (!_writeCoeff) {
            _writeCoeff = new float*[_numChannels];
        }
        if (!_writeCoeff[channel]) {
            _writeCoeff[channel] = new float[len];
        } else {
            delete[] _writeCoeff[channel];
            _writeCoeff[channel] = new float[len];
        }
        for (uint8_t i = 0; i < len; i++) {
            _writeCoeff[channel][i] = coeff[i];
        }
        _writeLen[channel] = len;
        _useWriteCoeff[channel] = true;
    }
}

void SBUS::getWriteCal(uint8_t channel,float *coeff,uint8_t len)
{
    if (coeff) {
        for (uint8_t i = 0; (i < _writeLen[channel]) && (i < len); i++) {
            coeff[i] = _writeCoeff[channel][i];
        }
    }
}

/* compute scale factor and bias from end points */
void SBUS::scaleBias(uint8_t channel)
{
    _sbusScale[channel] = 2.0f / ((float)_sbusMax[channel] - (float)_sbusMin[channel]);
    _sbusBias[channel] = -1.0f*((float)_sbusMin[channel] + ((float)_sbusMax[channel] - (float)_sbusMin[channel]) / 2.0f) * _sbusScale[channel];
}

float SBUS::PolyVal(size_t PolySize, float *Coefficients, float X) {
    if (Coefficients)
    {
        float Y = Coefficients[0];
        for (uint8_t i = 1; i < PolySize; i++)
        {
            Y = Y*X + Coefficients[i];
        }
        return(Y);
    } else
    {
        return 0;
    }
}

int SBUS::bytesAvalaible()
{
    int bytes_avail;
    ioctl(_fd, FIONREAD, &bytes_avail);
    return bytes_avail;
}

}
