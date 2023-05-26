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

#ifndef _VS_WRC201_I2C_H_
#define _VS_WRC201_I2C_H_

#include <cstdint>


// Encoder struct
//  This is use to create tuple for encoder data like the original code
struct encoder {
    uint32_t encL;
    uint32_t encR;
};


// Define encoder type
typedef encoder Encoder;


// VsWrc201I2c
//  A bunch of method to do communication with VS-WRC201
class VsWrc201I2c
{
    #define DEV_ADDR 0x10
    #define MAP_SIZE 0x100

    // VSWRC201 COMMAND LIST
    #define MU16_SYSNAME   0x00
    #define MU16_FIRMREC   0x02
    #define MU32_TRIPTIME  0x04
    #define MU8_MODE       0x0d
    #define MU16_POWOFF_T  0x0e
    #define MU8_O_EN       0x10
    #define MU8_TRIG       0x11
    #define MU16_SD_VI     0x12
    #define MU16_OD_DI     0x14
    #define MU16_SPD_T0    0x16
    #define MU16_MOVE_T0   0x18
    #define MU16_FB_PG0    0x20
    #define MU16_FB_PG1    0x22
    #define MU16_FB_ALIM0  0x24
    #define MU16_FB_ALIM1  0x26
    #define MU16_FB_DLIM0  0x28
    #define MU16_FB_DLIM1  0x2a
    #define MU16_FB_OLIM0  0x2c
    #define MU16_FB_OLIM1  0x2e
    #define MU16_FB_PCH0   0x30
    #define MU16_FB_PCH1   0x32
    #define MS32_T_POS0    0x40
    #define MS32_T_POS1    0x44
    #define MS32_A_POS0    0x48
    #define MS32_A_POS1    0x4c
    #define MS16_T_OUT0    0x50
    #define MS16_T_OUT1    0x52
    #define MS16_T_OUT2    0x54
    #define MS32_M_POS0    0x60
    #define MS32_M_POS1    0x64
    #define MS16_M_SPD0    0x68
    #define MS16_M_SPD1    0x6a
    #define MS16_M_OUT0    0x6c
    #define MS16_M_OUT1    0x6e
    #define MU16_M_DI      0x7e
    #define MS32_WP_PX     0x80
    #define MS32_WP_PY     0x84
    #define MS16_WP_TH     0x88
    #define MU16_M_VI      0x90
    #define MS32_P_DIS     0xa0
    #define MS16_P_RAD     0xa4
    #define MS16_P_SPD     0xa8
    #define MU8_P_STTS     0xaa
    #define MS16_S_XS      0xac
    #define MS16_S_ZS      0xae
    #define MS32_P_PX0     0xb0
    #define MS32_P_PX1     0xb4
    #define MS32_P_PX2     0xb8
    #define MS32_P_PX3     0xbc
    #define MS32_P_PY0     0xc0
    #define MS32_P_PY1     0xc4
    #define MS32_P_PY2     0xc8
    #define MS32_P_PY3     0xcc
    #define MS16_P_TH0     0xd0
    #define MS16_P_TH1     0xd2
    #define MS16_P_TH2     0xd4
    #define MS16_P_TH3     0xd6
    #define MS32_P_PXIN    0xd8
    #define MS32_P_PYIN    0xdc
    #define MS16_P_THIN    0xe0
    #define MU8_P_TOP      0xe2
    #define MU8_P_BTM      0xe3
    #define MU8_P_NOW      0xe4
    #define MU8_A_EN       0xf0
    #define MU8_PWN_SW     0xf1
    #define MU16_A_PCTR    0xf2
    #define MU8_TRIG2      0xf4

    public:
        VsWrc201I2c(uint8_t dev_addr);
        void     set_dev_addr      (uint8_t dev_addr);
        void     init_memmap       (uint16_t cut_off_level);
        uint8_t  write_1_byte      (uint8_t addr, uint8_t data);
        uint8_t  write_2_byte      (uint8_t addr, uint16_t data);
        uint8_t  write_4_byte      (uint8_t addr, uint32_t data);
        uint8_t  write_memmap      (uint8_t addr, uint8_t *data_array, uint8_t length);
        void     memmap_clean      (void);
        uint8_t  read_memmap       (uint8_t addr, uint8_t length);
        uint8_t  read_all          (void);
        void     send_write_map    (void);
        uint8_t  read_s8map        (uint8_t addr);
        uint8_t  write_s8map       (uint8_t addr, uint8_t data);
        uint16_t read_s16map       (uint8_t addr);
        uint16_t write_s16map      (uint8_t addr, uint16_t data);
        uint32_t read_s32map       (uint8_t addr);
        uint32_t write_s32map      (uint8_t addr, uint32_t data);
        Encoder  read_encoder      (void);
        void     clear_encoder     (void);
        uint8_t  check_write_flag  (uint8_t addr);
        float    get_vin           (void);

    protected:
        uint8_t dev_addr = DEV_ADDR;

        uint8_t memmap[MAP_SIZE];
        uint8_t write_flag[MAP_SIZE];
        uint8_t initialMemmap[MAP_SIZE] = {
                                            0x00, 0x00, 0xed, 0x05, 0xff, 0x01, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                            0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0x10, 0x00, 0x10,
                                            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
                                          };

    private:
        int __i2c;
};

#endif
