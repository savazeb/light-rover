/*
 * vs_wrc201_i2c:
 *  I2C communication library for VS-WRC201
 *  Copyright (c) 2023 savazeb
 *
 * Thanks to lightrover_ros for providing the python code,
 * later be converted to this c++ code
 * https://github.com/vstoneofficial/lightrover_ros
 ***********************************************************************
 */


#include "vs_wrc201_i2c.h"

#include <iostream>
#include <wiringPi.h>
#include <wiringPII2C.h>


VsWrc201I2c::VsWrc201I2c(uint8_t dev_addr) {
    this->dev_addr = dev_addr;
    this->__i2c = wiringPiI2CSetup(this->dev_addr);
    if (this->__i2c == -1) {
        std::cout << "Failed to init I2C communication.\n";
    }
}

/*
 * set_dev_addr:
 *	set device address
 *********************************************************************************
 */
void VsWrc201I2c::set_dev_addr(uint8_t dev_addr) {
    this->dev_addr = dev_addr;
    this->__i2c = wiringPiI2CSetup(this->dev_addr);
    if (this->__i2c == -1) {
        std::cout << "Failed to init I2C communication.\n";
    }
}

/*
 * init_memmap:
 *	initiate memory map
 *********************************************************************************
 */
void VsWrc201I2c::init_memmap(uint16_t cut_off_level) {
    int cut_off_hex = int((cut_off_level/3.3) * 0xfff);
    write_memmap(MU8_O_EN, initialMemmap, 0xF0);
    read_all();
    write_s16map(MU16_SD_VI, cut_off_hex);
}

/*
 * write_1_byte:
 *  Modify specific address of memory map on VS-WRC201 (1byte)
 *********************************************************************************
 */
uint8_t VsWrc201I2c::write_1_byte(uint8_t addr, uint8_t data) {
    wiringPiI2CWriteReg8(this->__i2c, addr, data);

    return 1;
}

/*
 * write_2_byte:
 *	Modify specific address of memory map on VS-WRC201 (2byte)
 *********************************************************************************
 */
uint8_t VsWrc201I2c::write_2_byte(uint8_t addr, uint16_t data) {
    for(uint8_t i = 0; i < 2; i++){
        uint8_t write_byte = ((data >> (i*8)) & 0xff);
        write_1_byte(addr+i, write_byte);
    }

    return 1;
}

/*
 * write_4_byte:
 *	Modify specific address of memory map on VS-WRC201 (4byte)
 *********************************************************************************
 */
uint8_t VsWrc201I2c::write_4_byte(uint8_t addr, uint32_t data) {
    for(uint8_t i = 0; i < 4; i++){
        uint8_t write_byte = ((data >> (i*8)) & 0xff);
        write_1_byte(addr+i, write_byte);
    }

    return 1;
}

/*
 * write_memmap:
 *  Modify specific address range of memory map on VS-WRC201
 *********************************************************************************
 */
uint8_t VsWrc201I2c::write_memmap(uint8_t addr, uint8_t *data_array, uint8_t length) {
    if (length <= 0) {
        return -1;
    }

    for(uint8_t i = 0; i < length; i++) {
        write_1_byte(addr+i, data_array[i]);
    }

    return 1;
}

/*
 * memmap_clean:
 *	Clean all memory map on raspbery
 *********************************************************************************
 */
void VsWrc201I2c::memmap_clean(void){
    for(uint8_t i = 0; i < MAP_SIZE; i++) {
        memmap[i] = 0x00;
    }
}

/*
 * read_memmap:
 *  Read specific address range of memory map on VS-WRC201
 *  Memory is written into memory map on raspberry
 *********************************************************************************
 */
uint8_t VsWrc201I2c::read_memmap(uint8_t addr, uint8_t length) {
    uint8_t i = 0;
    for(;i < MAP_SIZE; i++) {
            memmap[addr+i] = wiringPiI2CReadReg16(this->__i2c, addr+i);
    }

    return i;
}

/*
 * read_all:
 *  Read all memory map on VS-WRC201
 *  Memory is written into memory map on raspberrypi
 *********************************************************************************
 */
uint8_t VsWrc201I2c::read_all(void) {
    read_memmap(0x00,64);
    read_memmap(0x40,64);
    read_memmap(0x80,20);

    return 1;
}

/*
 * send_write_map:
 *  Write memory map on raspberrypi to memory map on VS-WRC201
 *********************************************************************************
 */
void VsWrc201I2c::send_write_map(void) {
    for(uint8_t i = 0x12; i < 0x90; i++) {
        uint8_t head_addr = i;
        uint8_t length = 0;
        while (write_flag[i]) {
            write_flag[i] = 0x00;
            length += 1;
            i += 1;
            if (i >= MAP_SIZE){
                break;
            }
        }

        uint8_t write_map[0x90 - 0x12];
        for(uint8_t j = head_addr; j < head_addr+length; j++) {
            write_map[j - head_addr] = memmap[j];
        }

        write_memmap(head_addr, write_map, length);
    }

    for(uint8_t i = 0x0e; i < 0x12; i++) {
        uint8_t head_addr = i;
        uint8_t length = 0;
        while (write_flag[i]) {
            write_flag[i] = 0x00;
            length += 1;
            i += 1;
            if (i >= MAP_SIZE) {
                break;
            }
        }

        uint8_t write_map[head_addr+length];
        for(uint8_t j = head_addr; j < head_addr+length; j++) {
            write_map[j - head_addr] = memmap[j];
        }

        write_memmap(head_addr, write_map, length);
    }
}

/*
 * read_s8map:
 *	Read memory map on raspberrypi (1byte)
 *********************************************************************************
 */
uint8_t VsWrc201I2c::read_s8map(uint8_t addr) {
    return memmap[addr];
}

/*
 * write_s8map:
 *  Modify memory map on raspberrypi (1byte)
 *********************************************************************************
 */
uint8_t VsWrc201I2c::write_s8map(uint8_t addr, uint8_t data) {
    memmap[addr] = data;
    write_flag[addr] = 0x01;

    return memmap[addr];
}

/*
 * read_s16map:
 *	Read memory map on raspberrypi (2byte)
 *********************************************************************************
 */
uint16_t VsWrc201I2c::read_s16map(uint8_t addr) {
    return ((memmap[addr+1] << 8) | memmap[addr]);
}

/*
 * write_s16map:
 *	Modify memory map on raspberrypi (2byte)
 *********************************************************************************
 */
uint16_t VsWrc201I2c::write_s16map(uint8_t addr, uint16_t data) {
    memmap[addr] = (0xff & data);
    memmap[addr+1] = ((0xff00 & data) >> 8);
    write_flag[addr] = 0x0101;

    return ((memmap[addr+1] << 8) | memmap[addr]);
}

/*
 * read_s32map:
 *	Read memory map on raspberrypi (4byte)
 *********************************************************************************
 */
uint32_t VsWrc201I2c::read_s32map(uint8_t addr) {
    return ((memmap[addr+3] << 24) | (memmap[addr+2] << 16) | (memmap[addr+1] << 8) | memmap[addr]);
}

/*
 * write_s32map:
 *	Modify memory map on raspberrypi (4byte)
 *********************************************************************************
 */
uint32_t VsWrc201I2c::write_s32map(uint8_t addr, uint32_t data) {
    memmap[addr] = (0xff & data);
    memmap[addr+1] = ((0xff00 & data) >> 8);
    memmap[addr+2] = ((0xff0000 & data) >> 16);
    memmap[addr+3] = ((0xff000000 & data) >> 24);
    write_flag[addr] = 0x01010101;

    return ((memmap[addr+3] << 24) | (memmap[addr+2] << 16) | (memmap[addr+1] << 8) | memmap[addr]);
}

/*
 * read_encoder:
 *	Read encoder data
 *********************************************************************************
 */
Encoder VsWrc201I2c::read_encoder(void) {
    Encoder encoder;
    encoder.encL = read_s32map(MS32_M_POS0);
    encoder.encR = read_s32map(MS32_M_POS1);

    return encoder;
}

/*
 * clear_encoder:
 *	Clear encoder data
 *********************************************************************************
 */
void VsWrc201I2c::clear_encoder(void) {
    write_s8map(MU8_TRIG, read_s8map(MU8_TRIG) | 0x0C);
}

/*
 * check_write_flag:
 *  Check if data is written into memory map
 *********************************************************************************
 */
uint8_t VsWrc201I2c::check_write_flag(uint8_t addr) {
    return write_flag[addr];
}

/*
 * get_vin:
 *	Read the voltage level of VS-WRC201 battery
 *********************************************************************************
 */
float VsWrc201I2c::get_vin(void) {
    read_memmap(MU16_M_VI, 0x02);
    uint16_t memmapV = read_s16map(MU16_M_VI);

    float vin = (float(memmapV) / float(0x0fff)) * 3.3;

    return vin;
}
