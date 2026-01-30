/************************************************************

This library is based on the LTC68041.cpp by linear technology.
http://www.linear.com/product/LTC6804-1

I modified it to make it compatible with the ESP8622
https://github.com/jontubs/EasyBMS
***********************************************************/
#pragma once

#include <Arduino.h>
#include <SPI.h>

#include <array>
#include <bitset>
#include <concepts>

#include <cmath>
#include <cstdint>

#include "debug.hpp"

#ifndef DEBUG
#define DEBUG_BEGIN(...)
#define DEBUG_PRINT(...)
#define DEBUG_PRINTLN(...)
#endif

template<std::size_t Nodes = 1>
class LTC68041 {
   private:
    static constexpr int DCTOPos = 4;
    static constexpr int MDPos = 7;
    static constexpr int DCPPos = 4;
    static constexpr int STPos = 5;
    static constexpr int PUPPos = 6;

   public:
    static constexpr int CELLNUM = 12;  // Number of cells checked by this Chip
    static constexpr int AUXNUM = 6;    // Number of Auxiliary Voltages
    static constexpr int SIZEREG = 6;   // All registers have the same length
    /**
     * @brief Discharge timeouts in DCTO bits in CFGR5
     *
     */
    enum DischargeTimeout : uint8_t {
        DISCHRG_TIMEOUT_DISABLED = 0x0 << DCTOPos,
        DISCHRG_TIMEOUT_0MIN5 = 0x1 << DCTOPos,
        DISCHRG_TIMEOUT_1MIN = 0x2 << DCTOPos,
        DISCHRG_TIMEOUT_2MIN = 0x3 << DCTOPos,
        DISCHRG_TIMEOUT_3MIN = 0x4 << DCTOPos,
        DISCHRG_TIMEOUT_4MIN = 0x5 << DCTOPos,
        DISCHRG_TIMEOUT_5MIN = 0x6 << DCTOPos,
        DISCHRG_TIMEOUT_10MIN = 0x7 << DCTOPos,
        DISCHRG_TIMEOUT_15MIN = 0x8 << DCTOPos,
        DISCHRG_TIMEOUT_20MIN = 0x9 << DCTOPos,
        DISCHRG_TIMEOUT_30MIN = 0xA << DCTOPos,
        DISCHRG_TIMEOUT_40MIN = 0xB << DCTOPos,
        DISCHRG_TIMEOUT_60MIN = 0xC << DCTOPos,
        DISCHRG_TIMEOUT_75MIN = 0xD << DCTOPos,
        DISCHRG_TIMEOUT_90MIN = 0xE << DCTOPos,
        DISCHRG_TIMEOUT_120MIN = 0xF << DCTOPos,
    };

    /**
     * @brief Discharge time left values on DCTO read in CFGR5
     *
     */
    enum DischargeTimeLeft : uint8_t {
        DISCHRG_TIME_LEFT_TIMEOUT_DISABLED = 0x0 << DCTOPos,
        DISCHRG_TIME_LEFT_0MIN_TO_0MIN5 = 0x1 << DCTOPos,
        DISCHRG_TIME_LEFT_0MIN5_TO_1MIN = 0x2 << DCTOPos,
        DISCHRG_TIME_LEFT_1MIN_TO_2MIN = 0x3 << DCTOPos,
        DISCHRG_TIME_LEFT_2MIN_TO_3MIN = 0x4 << DCTOPos,
        DISCHRG_TIME_LEFT_3MIN_TO_4MIN = 0x5 << DCTOPos,
        DISCHRG_TIME_LEFT_4MIN_TO_5MIN = 0x6 << DCTOPos,
        DISCHRG_TIME_LEFT_5MIN_TO_10MIN = 0x7 << DCTOPos,
        DISCHRG_TIME_LEFT_10MIN_TO_15MIN = 0x8 << DCTOPos,
        DISCHRG_TIME_LEFT_15MIN_TO_20MIN = 0x9 << DCTOPos,
        DISCHRG_TIME_LEFT_20MIN_TO_30MIN = 0xA << DCTOPos,
        DISCHRG_TIME_LEFT_30MIN_TO_40MIN = 0xB << DCTOPos,
        DISCHRG_TIME_LEFT_40MIN_TO_60MIN = 0xC << DCTOPos,
        DISCHRG_TIME_LEFT_60MIN_TO_75MIN5 = 0xD << DCTOPos,
        DISCHRG_TIME_LEFT_75MIN_TO_90MIN = 0xE << DCTOPos,
        DISCHRG_TIME_LEFT_90MIN_TO_120MIN = 0xF << DCTOPos,
    };

    /**
     * @brief ADC Conversion Mode
     *
     * | 27kHz Mode (Fast)           |
     * | 7kHz Mode (Normal)          |
     * | 26Hz Mode (Filtered)        |
     * | 14kHz Mode                  |
     * | 3kHz Mode                   |
     * | 2kHz Mode                   |
     */
    enum ADCFilterMode {
        FAST,
        NORMAL,
        FILTERED,
        BANDWIDTH_27KHZ = FAST,
        BANDWIDTH_7KHZ = NORMAL,
        BANDWIDTH_26HZ = FILTERED,
        BANDWIDTH_1KHZ,
        BANDWIDTH_422HZ,
        BANDWIDTH_14KHZ,
        BANDWIDTH_3KHZ,
        BANDWIDTH_2KHZ,
    };

    /**
     * @brief Cell Channels to convert
     *
     * |CH | Dec  | Channels to convert |
     * |---|------|---------------------|
     * |000| 0    | All Cells           |
     * |001| 1    | Cell 1 and Cell 7   |
     * |010| 2    | Cell 2 and Cell 8   |
     * |011| 3    | Cell 3 and Cell 9   |
     * |100| 4    | Cell 4 and Cell 10  |
     * |101| 5    | Cell 5 and Cell 11  |
     * |110| 6    | Cell 6 and Cell 12  |
     */
    enum CellChannel : uint16_t {
        CH_ALL = 0b000,
        CH_CELL_1_AND_7 = 0b001,
        CH_CELL_2_AND_8 = 0b010,
        CH_CELL_3_AND_9 = 0b011,
        CH_CELL_4_AND_10 = 0b100,
        CH_CELL_5_AND_11 = 0b101,
        CH_CELL_6_AND_12 = 0b110,
    };

    /**
     * @brief AUX Channels to convert
     *
     * |CHG | Dec  | Channels to convert  |
     * |----|------|----------------------|
     * |000 | 0    | All GPIOS and 2nd Ref|
     * |001 | 1    | GPIO 1               |
     * |010 | 2    | GPIO 2               |
     * |011 | 3    | GPIO 3               |
     * |100 | 4    | GPIO 4               |
     * |101 | 5    | GPIO 5               |
     * |110 | 6    | Vref2                |
     */
    enum AuxChannel : uint16_t {
        CHG_ALL = 0b000,
        CHG_GPIO1 = 0b001,
        CHG_GPIO2 = 0b010,
        CHG_GPIO3 = 0b011,
        CHG_GPIO4 = 0b100,
        CHG_GPIO5 = 0b101,
        CHG_VREF2 = 0b110,
    };

    /**
     * @brief Status Group to select
     *
     * |CHST| Dec  |  Status Group    |
     * |----|------|------------------|
     * |000 | 0    | OC, ITMP, VA, VD |
     * |001 | 1    | SOC              |
     * |010 | 2    | ITMP             |
     * |011 | 3    | VA               |
     * |100 | 4    | VD               |
     */
    enum StatusGroup : uint16_t {
        CHST_ALL = 0b000,
        CHST_SOC = 0b001,
        CHST_ITMP = 0b010,
        CHST_VA = 0b011,
        CHST_VD = 0b100,
    };

    /**
     * @brief Self-Test mode selection
     *
     * |ST| Dec  | Self-Test Mode |
     * |--|------|----------------|
     * |01| 1    | Self-Test 1    |
     * |10| 2    | Self-Test 2    |
     */
    enum SelfTestMode : uint16_t {
        ST_SELF_TEST_1 = (0b01 << STPos),
        ST_SELF_TEST_2 = (0b10 << STPos),
    };

    /**
     * @brief Controls if Discharging transitors are enabled
     *        or disabled during Cell conversions.
     *
     * |DCP | Discharge Permitted During conversion  |
     * |----|----------------------------------------|
     * |0   | No - discharge is not permitted        |
     * |1   | Yes - discharge is permitted           |
     */
    enum DischargeCtrl : uint16_t {
        DCP_DISABLED = (0b0 << DCPPos),
        DCP_ENABLED = (0b1 << DCPPos),
    };

    /**
     * @brief Pull up/Pull down selection for open wire test
     *
     * |PUP | Pull-Up/Pull-Down Current |
     * |    | for Open-Wire Conversions |
     * |----|---------------------------|
     * |0   | Pull-Down Current         |
     * |1   | Pull-Up Current           |
     */
    enum PUPCtrl : uint16_t {
        PUP_PULL_DOWN = (0b0 << PUPPos),
        PUP_PULL_UP = (0b1 << PUPPos),
    };

    // Methods
    /**
     * @brief Construct a new LTC68041 object
     * 
     * @param pCS chip select pin, default 10
     * @param tempOffset offset of internal temp sensor, default 0.0
     */
    explicit LTC68041(byte pCS = 10, float tempOffset = 0.0);
    /**
     * @brief Initializes the SPI bus instance used for communication
     * 
     * @param pinMOSI Pin used as MOSI
     * @param pinMISO Pin used as MISO
     * @param pinCLK Pin used as SCLK
     */
    void initSPI(byte pinMOSI, byte pinMISO, byte pinCLK);
    /**
     * @brief Deinitialize SPI bus Instance
     * 
     */
    void destroySPI();
    /**
     * @brief Wake isoSPI up from idle state
     *        Generic wakeup commannd to wake isoSPI up out of idle
     */
    void wakeup_idle() const;
    /**
     * @brief Poll for ongoing ADC conversion and wait for its end
     * 
     * @return true if conversion completed
     * @return false if timeout expired
     */
    bool waitForConversion();
    /**
     * @brief Read config register group from chain of LTC chips from (iso)SPI
     * 
     * @return true if read was successfull
     * @return false if read failed
     */
    bool cfgRead();
    /**
     * @brief Write config register group to chain of LTC chips over (iso)SPI
     * 
     */
    void cfgWrite();
    /**
     * @brief Set cell undervoltage value in config register group
     * 
     * @param Undervoltage in Volts
     */
    void cfgSetVUV(const float Undervoltage);
    /**
     * @brief Get cell undervoltage value in config register group
     * 
     * @return undervoltage as float in Volts 
     */
    float cfgGetVUV() const;
    /**
     * @brief Set cell overvoltage value in config register group
     * 
     * @param Overvoltage in Volts
     */
    void cfgSetVOV(const float Overvoltage);
    /**
     * @brief Get cell overvoltage value in config register group
     * 
     * @return overvoltage as float in Volts 
     */
    float cfgGetVOV() const;
    /**
     * @brief Set discharge timer in config register group
     * 
     * @param timeout discharge timeout as enum value of type DischargeTimeout
     */
    void cfgSetDischargeTimeout(DischargeTimeout timeout);
    /**
     * @brief Get discharge timer in config register group
     * 
     * @return discharge timeout as as enum value of type DischargeTimeLeft
     */
    DischargeTimeLeft cfgGetDischargeTimeLeft() const;
    /**
     * @brief Set reference voltage enable bit in config register group
     * 
     * @param value true or false
     */
    void cfgSetRefOn(const bool value);
    /**
     * @brief Get reference voltage enable bit in config register group
     * 
     * @return true 
     * @return false 
     */
    bool cfgGetRefOn();
    /**
     * @brief 
     * 
     * @return true 
     * @return false 
     */
    bool cfgGetSWTENPin() const;
    /**
     * @brief Set ADC filter Mode out of the six possible modes.
     *        Handles setting of MD bits in command and ADCOPT bit in CFGR0w
     * 
     * @param mode ADC mode as enum value of type ADCFilterMode
     */
    void cfgSetADCMode(ADCFilterMode mode);
    /**
     * @brief Get ADC filter mode in config register group
     * 
     * @return ADC mode as enum value of type ADCFilterMode
     */
    ADCFilterMode cfgGetADCMode() const;

    /**
     * @brief Set DCC bits for balancing cells in config register group
     * 
     * @param dcc DCC bits as std::bitset<12>
     */
    template <unsigned int N = 0>
    requires (N < Nodes)
    void cfgSetDCC(std::bitset<12> dcc);

    /**
     * @brief Get DCC bits for balancing cells in config register group
     * 
     * @return DCC bits as std::bitset<12> 
     */
    template <unsigned int N = 0>
    requires (N < Nodes)
    std::bitset<12> cfgGetDCC() const;

    /**
     * @brief Get the Cell Voltages as std::array from cell voltage register groups
     *        read from SPI if first reading after cell conversion or from cache otherwise
     * 
     * @tparam N 
     * @tparam M 
     * @param voltages std::array to store values
     * @return true if reading and parsing was successfull
     * @return false if reading failed
     */
    template <std::size_t N, unsigned int M = 0>
    //requires (N <= CELLNUM)
    bool getCellVoltages(std::array<float, N> &voltages);

    /**
     * @brief Get the Aux Voltage of specified channel
     * 
     * @tparam N 
     * @param chg aux channel
     * @return Voltage as float in Volts 
     */
    template <unsigned int N = 0>
    float getAuxVoltage(const AuxChannel chg);

    /**
     * @brief Get the Status Voltage of specified channel
     * 
     * @tparam N 
     * @param chst status channel
     * @return Voltage as float in Volts 
     */
    template <unsigned int N = 0>
    float getStatusVoltage(const StatusGroup chst);

    template <unsigned int N = 0>
    bool getStatusMUXFail();

    template <unsigned int N = 0>
    bool getStatusThermalShutdown();

    // Cell x Overvoltage Flag x = 1 to 12 Cell Voltage Compared to VOV Comparison Voltage 0 -> Cell x Not Flagged for Overvoltage Condition. 1 -> Cell x Flagged
    template <unsigned int N = 0>
    std::bitset<12> getStatusOverVoltageFlags();

    // Cell x Undervoltage Flag x = 1 to 12 Cell Voltage Compared to VUV Comparison Voltage 0 -> Cell x Not Flagged for Undervoltage Condition. 1 -> Cell x Flagged
    template <unsigned int N = 0>
    std::bitset<12> getStatusUnderVoltageFlags();

    template <unsigned int N = 0>
    int getStatusRevision();

    // debug methods
    bool checkSPI();

    void clrAuxRegs();
    void clrCellRegs();
    unsigned long startAuxConv(AuxChannel chg = AuxChannel::CHG_ALL);
    unsigned long startCellConv(DischargeCtrl dcp, CellChannel ch = CellChannel::CH_ALL);
    unsigned long startCellSocConv(DischargeCtrl dcp);
    void startCellConvTest(SelfTestMode st);
    unsigned long startCellAuxConv(DischargeCtrl dcp);
    unsigned long startStatusConv(StatusGroup chst = StatusGroup::CHST_ALL);
    void startOpenWireCheck(PUPCtrl pup, DischargeCtrl dcp, CellChannel ch = CellChannel::CH_ALL);

   private:

    /**
     * @brief Position of config and status bits in corresponding registers
     *
     */
    enum CfgBits {
        CFGR0_ADCOPT_Pos = 0,
        CFGR0_REFON_Pos = 2,
        CFGR0_GPIO1_Pos = 3,
        CFGR0_GPIO2_Pos = 4,
        CFGR0_GPIO3_Pos = 5,
        CFGR0_GPIO4_Pos = 6,
        CFGR0_GPIO5_Pos = 7,
    };

    enum BitMasks : uint8_t {
        CFG0_ADCOPT_MSK = 0x01,
        CFG0_SWTRD_MSK = 0x02,
        CFG0_REFON_MSK = 0x04,
        CFG0_GPIO1_MSK = 0x08,
        CFG0_GPIO2_MSK = 0x10,
        CFG0_GPIO3_MSK = 0x20,
        CFG0_GPIO4_MSK = 0x40,
        CFG0_GPIO5_MSK = 0x80,
        CFG1_VUV_MSK = 0xFF,
        CFG2_VUV_MSK = 0x0F,
        CFG2_VOV_MSK = 0xF0,
        CFG3_VOV_MSK = 0xFF,

        /**
         * Configuration register 4 discharge cell bitmask.
         */
        CFG4_DCC_MSK = 0xFF,

        /**
         * Configuration register 5 discharge cell bitmask.
         */
        CFG5_DCC_MSK = 0x0F,

        CFG5_DCTO_MSK = 0xF0,

        /**
         * Thermal Shutdown Status Read: 0 -> Thermal Shutdown Has Not Occurred 1 -> Thermal Shutdown Has Occurred THSD Bit Cleared to 0 on Read of Status
         * RegIster Group B
         */
        STBR5_THSD_MSK = 0x01,

        /**
         * Multiplexer Self-Test ResultRead: 0 -> Multiplexer Passed Self Test 1 -> Multiplexer Failed Self Test
         */
        STBR5_MUXFAIL_MSK = 0x02,

        STBR5_REV_MSK = 0xF0,
    };

    /**
     * @brief Register group names with index to check cache validity
     * 
     */

     enum RegGroups {
        CFGR = 0,
        CVAR,
        CVBR,
        CVCR,
        CVDR,
        AVAR,
        AVBR,
        STAR,
        STBR,
        COMM
     };

    /**
     * @brief Value names for parsing measurement values from read only registers
     *        with corresponding index in group array in lower nibble
     */
    enum ValueNames {
        // Cell voltage register group A
        C1V = 0x00,
        C2V = 0x02,
        C3V = 0x04,
        // Cell voltage register group B
        C4V = 0x10,
        C5V = 0x12,
        C6V = 0x14,
        // Cell voltage register group C
        C7V = 0x20,
        C8V = 0x22,
        C9V = 0x24,
        // Cell voltage register group D
        C10V = 0x30,
        C11V = 0x32,
        C12V = 0x34,
        // Auxiliary register group A
        G1V = 0x40,
        G2V = 0x42,
        G3V = 0x44,
        // Auxiliary register group B
        G4V = 0x50,
        G5V = 0x52,
        REF = 0x54,
        // Status register group A
        SC = 0x60,
        ITMP = 0x62,
        VA = 0x64,
        // Status register group B
        VD = 0x70
    };

    /**
     * @brief Register names in the different register groups with corresponding
     *        index in group array in lower nibble
     */
    enum RegNames {
        // Configuration Register Group
        CFGR0 = 0x00,
        CFGR1 = 0x01,
        CFGR2 = 0x02,
        CFGR3 = 0x03,
        CFGR4 = 0x04,
        CFGR5 = 0x05,
        // Cell voltage register group A
        CVAR0 = 0x10,
        CVAR1 = 0x11,
        CVAR2 = 0x12,
        CVAR3 = 0x13,
        CVAR4 = 0x14,
        CVAR5 = 0x15,
        // Cell voltage register group B
        CVBR0 = 0x20,
        CVBR1 = 0x21,
        CVBR2 = 0x22,
        CVBR3 = 0x23,
        CVBR4 = 0x24,
        CVBR5 = 0x25,
        // Cell voltage register group C
        CVCR0 = 0x30,
        CVCR1 = 0x31,
        CVCR2 = 0x32,
        CVCR3 = 0x33,
        CVCR4 = 0x34,
        CVCR5 = 0x35,
        // Cell voltage register group D
        CVDR0 = 0x40,
        CVDR1 = 0x41,
        CVDR2 = 0x42,
        CVDR3 = 0x43,
        CVDR4 = 0x44,
        CVDR5 = 0x45,
        // Auxiliary register group A
        AVAR0 = 0x50,
        AVAR1 = 0x51,
        AVAR2 = 0x52,
        AVAR3 = 0x53,
        AVAR4 = 0x54,
        AVAR5 = 0x55,
        // Auxiliary register group B
        AVBR0 = 0x60,
        AVBR1 = 0x61,
        AVBR2 = 0x62,
        AVBR3 = 0x63,
        AVBR4 = 0x64,
        AVBR5 = 0x65,
        // Status register group A
        STAR0 = 0x70,
        STAR1 = 0x71,
        STAR2 = 0x72,
        STAR3 = 0x73,
        STAR4 = 0x74,
        STAR5 = 0x75,
        // Status register group B
        STBR0 = 0x80,
        STBR1 = 0x81,
        STBR2 = 0x82,
        STBR3 = 0x83,
        STBR4 = 0x84,
        STBR5 = 0x85,
        // COMM register group
        COMM0 = 0x90,
        COMM1 = 0x91,
        COMM2 = 0x92,
        COMM3 = 0x93,
        COMM4 = 0x94,
        COMM5 = 0x95,
    };

    /**
     * @brief Command codes, for commands that have option bits (see p. 50 in manual),
     *        these bits are here set to 0 and will be OR'ed in later before sending
     *        the command to the chips
     */
    enum Commands : uint16_t {
        WRCFG = 0x0001,
        RDCFG = 0x0002,
        RDCVA = 0x0004,
        RDCVB = 0x0006,
        RDCVC = 0x0008,
        RDCVD = 0x000A,
        RDAUXA = 0x000C,
        RDAUXB = 0x000E,
        RDSTATA = 0x0010,
        RDSTATB = 0x0012,
        ADCV = 0x0260,
        ADOW = 0x0228,
        CVST = 0x0207,
        ADAX = 0x0460,
        AXST = 0x0407,
        ADSTAT = 0x0468,
        STATST = 0x040F,
        ADCVAX = 0x046F,
        ADCVSC = 0x0467,
        CLRCELL = 0x0711,
        CLRAUX = 0x0712,
        CLRSTAT = 0x0713,
        PLADC = 0x0714,
        DIAGN = 0x0715,
        WRCOMM = 0x0721,
        RDCOMM = 0x0722,
        STCOMM = 0x0723
    };

    /**
     * @brief ADC Conversion Mode
     *
     * |MD| Dec  |          ADC Conversion Mode                |
     * |--|------|---------------------|-----------------------|
     * |  |      | ADCOPT(CFGR0[0]) = 0| ADCOPT(CFGR0[0]) = 1  |
     * |--|------|---------------------|-----------------------|
     * |00|0     | 422Hz Mode          | 1khz Mode             |
     * |01| 1    | 27kHz Mode (Fast)   | 14kHz Mode            |
     * |10| 2    | 7kHz Mode (Normal)  | 3kHz Mode             |
     * |11| 3    | 26Hz Mode (Filtered)| 2kHz Mode             |
     */
    enum ADCMode : uint16_t {
        MD_OPT = (0b00 << MDPos),
        MD_FAST = (0b01 << MDPos),
        MD_NORMAL = (0b10 << MDPos),
        MD_FILTERED = (0b11 << MDPos),
    };

    /**
     * @brief Register map of internal register groups
     */
    struct Registers {
        uint8_t CFGR0w;                     // Register value of CFGR0 on next write
        uint8_t CFGR0r;                     // Register value of CFGR0 on last read
        std::array<uint8_t, SIZEREG> CFGR;  // Configuration Register Group
        std::array<uint8_t, SIZEREG> CVAR;  // Cell Voltage Register Group A
        std::array<uint8_t, SIZEREG> CVBR;  // Cell Voltage Register Group B
        std::array<uint8_t, SIZEREG> CVCR;  // Cell Voltage Register Group C
        std::array<uint8_t, SIZEREG> CVDR;  // Cell Voltage Register Group D
        std::array<uint8_t, SIZEREG> AVAR;  // Auxiliary Register Group A
        std::array<uint8_t, SIZEREG> AVBR;  // Auxiliary Register Group B
        std::array<uint8_t, SIZEREG> STAR;  // Status Register Group A
        std::array<uint8_t, SIZEREG> STBR;  // Status Register Group B
        std::array<uint8_t, SIZEREG> COMM;  // COMM Register Group
    };

    float offsetTemp;  // Offset of temperaturemeasurement

    SPIClass SPI_local;
    ADCMode md;
    byte pinCS;  // ChipSelectPin
    std::array<Registers, Nodes> regs;
    std::bitset<10> isCacheInvalid;
    bool pollADC;

    /**
     * @brief Helper function to calculate voltages in volt from register values
     *
     * @param value value to parse from registers, from ValueNames enum
     * @return value as float in Volt
     */
    template <unsigned int N = 0>
    requires (N < Nodes)
    constexpr inline float parseVoltage(const ValueNames value);

    static constexpr uint16_t crc15Table[256] = {
        0x0000, 0xc599, 0xceab, 0x0b32, 0xd8cf, 0x1d56, 0x1664, 0xd3fd, 0xf407, 0x319e, 0x3aac,  //!< precomputed CRC15 Table
        0xff35, 0x2cc8, 0xe951, 0xe263, 0x27fa, 0xad97, 0x680e, 0x633c, 0xa6a5, 0x7558, 0xb0c1, 0xbbf3, 0x7e6a, 0x5990, 0x9c09, 0x973b, 0x52a2, 0x815f, 0x44c6,
        0x4ff4, 0x8a6d, 0x5b2e, 0x9eb7, 0x9585, 0x501c, 0x83e1, 0x4678, 0x4d4a, 0x88d3, 0xaf29, 0x6ab0, 0x6182, 0xa41b, 0x77e6, 0xb27f, 0xb94d, 0x7cd4, 0xf6b9,
        0x3320, 0x3812, 0xfd8b, 0x2e76, 0xebef, 0xe0dd, 0x2544, 0x02be, 0xc727, 0xcc15, 0x098c, 0xda71, 0x1fe8, 0x14da, 0xd143, 0xf3c5, 0x365c, 0x3d6e, 0xf8f7,
        0x2b0a, 0xee93, 0xe5a1, 0x2038, 0x07c2, 0xc25b, 0xc969, 0x0cf0, 0xdf0d, 0x1a94, 0x11a6, 0xd43f, 0x5e52, 0x9bcb, 0x90f9, 0x5560, 0x869d, 0x4304, 0x4836,
        0x8daf, 0xaa55, 0x6fcc, 0x64fe, 0xa167, 0x729a, 0xb703, 0xbc31, 0x79a8, 0xa8eb, 0x6d72, 0x6640, 0xa3d9, 0x7024, 0xb5bd, 0xbe8f, 0x7b16, 0x5cec, 0x9975,
        0x9247, 0x57de, 0x8423, 0x41ba, 0x4a88, 0x8f11, 0x057c, 0xc0e5, 0xcbd7, 0x0e4e, 0xddb3, 0x182a, 0x1318, 0xd681, 0xf17b, 0x34e2, 0x3fd0, 0xfa49, 0x29b4,
        0xec2d, 0xe71f, 0x2286, 0xa213, 0x678a, 0x6cb8, 0xa921, 0x7adc, 0xbf45, 0xb477, 0x71ee, 0x5614, 0x938d, 0x98bf, 0x5d26, 0x8edb, 0x4b42, 0x4070, 0x85e9,
        0x0f84, 0xca1d, 0xc12f, 0x04b6, 0xd74b, 0x12d2, 0x19e0, 0xdc79, 0xfb83, 0x3e1a, 0x3528, 0xf0b1, 0x234c, 0xe6d5, 0xede7, 0x287e, 0xf93d, 0x3ca4, 0x3796,
        0xf20f, 0x21f2, 0xe46b, 0xef59, 0x2ac0, 0x0d3a, 0xc8a3, 0xc391, 0x0608, 0xd5f5, 0x106c, 0x1b5e, 0xdec7, 0x54aa, 0x9133, 0x9a01, 0x5f98, 0x8c65, 0x49fc,
        0x42ce, 0x8757, 0xa0ad, 0x6534, 0x6e06, 0xab9f, 0x7862, 0xbdfb, 0xb6c9, 0x7350, 0x51d6, 0x944f, 0x9f7d, 0x5ae4, 0x8919, 0x4c80, 0x47b2, 0x822b, 0xa5d1,
        0x6048, 0x6b7a, 0xaee3, 0x7d1e, 0xb887, 0xb3b5, 0x762c, 0xfc41, 0x39d8, 0x32ea, 0xf773, 0x248e, 0xe117, 0xea25, 0x2fbc, 0x0846, 0xcddf, 0xc6ed, 0x0374,
        0xd089, 0x1510, 0x1e22, 0xdbbb, 0x0af8, 0xcf61, 0xc453, 0x01ca, 0xd237, 0x17ae, 0x1c9c, 0xd905, 0xfeff, 0x3b66, 0x3054, 0xf5cd, 0x2630, 0xe3a9, 0xe89b,
        0x2d02, 0xa76f, 0x62f6, 0x69c4, 0xac5d, 0x7fa0, 0xba39, 0xb10b, 0x7492, 0x5368, 0x96f1, 0x9dc3, 0x585a, 0x8ba7, 0x4e3e, 0x450c, 0x8095};

    /**
     * @brief Calculates the CRC sum of 16 bit half word
     * 
     * @param data 16 bit half word data
     * @return crc of "data" as uint16_t  
     */
    constexpr uint16_t calcPEC15(const uint16_t data) const;

    /**
     * @brief Calculates the CRC sum of some data bytes given by the std::array "data"
     * 
     * @tparam N 
     * @param data bytes of data as std::array
     * @return crc of "data" as uint16_t 
     */
    template <std::size_t N>
    constexpr uint16_t calcPEC15(const std::array<uint8_t, N> &data) const {
        uint16_t remainder = 16, addr = 0;  // initialize the PEC

        for (const auto &element : data)  // loops for each byte in data array
        {
            addr = ((remainder >> 7) ^ element) & 0xff;  // calculate PEC table address
            remainder = (remainder << 8) ^ crc15Table[addr];
        }

        return (remainder * 2);  // The CRC15 has a 0 in the LSB so the remainder must be multiplied by 2
    }

    /**
     * @brief Execute specified read command on SPI and store data in register group cache
     * 
     * @param cmd Command to write on SPI bus
     * @return true if read successfull (PEC correct)
     * @return false if read failed (PEC incorrect)
     */
    bool spi_read_cmd(Commands cmd);

    /**
     * @brief Execute specified write command with config bit ORed in
     * 
     * @param cmd command with option bits ORed in
     */
    void spi_write_cmd(uint16_t cmd);
};

template <std::size_t Nodes>
LTC68041<Nodes>::LTC68041(byte pCS, float tempOffset) : offsetTemp(tempOffset), md(MD_NORMAL), pinCS(pCS), regs({}), SPI_local(FSPI), isCacheInvalid(0x3FFFF), pollADC{false}
{
    DEBUG_PRINT("Objekt angelegt");

    for (auto &reg : regs)
        reg.CFGR0w = 0xFE;
}

template <std::size_t Nodes>
void LTC68041<Nodes>::initSPI(byte pinMOSI, byte pinMISO, byte pinCLK) {
    pinMode(pinMOSI, OUTPUT);
    pinMode(pinMISO, INPUT);
    pinMode(pinCLK, OUTPUT);
    pinMode(pinCS, OUTPUT);

    SPI_local.begin(pinCLK, pinMISO, pinMOSI, -1);
}

template <std::size_t Nodes>
void LTC68041<Nodes>::destroySPI() {
    SPI_local.end();
}

template <std::size_t Nodes>
void LTC68041<Nodes>::wakeup_idle() const {
    digitalWrite(pinCS, LOW);
    delayMicroseconds(500);  // Guarantees the isoSPI will be in ready mode
    digitalWrite(pinCS, HIGH);
}

template <std::size_t Nodes>
constexpr uint16_t LTC68041<Nodes>::calcPEC15(const uint16_t data) const {
    uint16_t remainder = 16, addr = 0;  // initialize the PEC

    addr = ((remainder >> 7) ^ (data >> 8)) & 0xff;  // calculate PEC table address
    remainder = (remainder << 8) ^ crc15Table[addr];

    addr = ((remainder >> 7) ^ (data & 0xff)) & 0xff;  // calculate PEC table address
    remainder = (remainder << 8) ^ crc15Table[addr];

    return (remainder * 2);  // The CRC15 has a 0 in the LSB so the remainder must be multiplied by 2
}

template <std::size_t Nodes>
bool LTC68041<Nodes>::spi_read_cmd(Commands cmd) {
    uint16_t pec;
    auto start = millis();
    bool pecCorrect = false;
    std::array<uint8_t, SIZEREG> rxData;

    wakeup_idle();  // This will guarantee that the LTC6804 isoSPI port is awake, this command can be removed.

    SPI_local.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
    digitalWrite(pinCS, LOW);

    switch (cmd)
    {
    case RDCVA:
    case RDCVB:
    case RDCVC:
    case RDCVD:
    case RDAUXA:
    case RDAUXB:
    case RDSTATA:
    case RDSTATB:
        if (!pollADC)
            break;
        
        // poll ADC for ongoing conversion
        SPI_local.transfer16(PLADC);
        SPI_local.transfer16(calcPEC15(PLADC));

        // PLADC response status is valid after N = number of Nodes clock cycles, so dump bytes that are partially valid
        for(int i = 0; i < ((Nodes / 8) + 1); i++)
            SPI_local.transfer(0xFF);

        while (SPI_local.transfer(0xFF) == 0x00) {
            if((millis() - start) > 1000) {
                digitalWrite(pinCS, HIGH);
                SPI_local.endTransaction();
                return false;
            }
            }

        digitalWrite(pinCS, HIGH);
        delayMicroseconds(2);  // toggle CS between commands
        digitalWrite(pinCS, LOW);

        pollADC = false;
        break;
    default:
        break;
    }

    // issue data read command after conversion complete
    SPI_local.transfer16(cmd);
    SPI_local.transfer16(calcPEC15(cmd));

    for (auto &reg : regs)
    {
        for (auto &element : rxData) {
            element = SPI_local.transfer(1);
        }

        pec = SPI_local.transfer16(1);
        pecCorrect = (pec == calcPEC15(rxData));

        if (!pecCorrect)
            break;

        switch(cmd) {
            case RDCFG:
                reg.CFGR = rxData;
                break;
            case RDCVA:
                reg.CVAR = rxData;
                break;
            case RDCVB:
                reg.CVBR = rxData;
                break;
            case RDCVC:
                reg.CVCR = rxData;
                break;
            case RDCVD:
                reg.CVDR = rxData;
                break;
            case RDAUXA:
                reg.AVAR = rxData;
                break;
            case RDAUXB:
                reg.AVBR = rxData;
                break;
            case RDSTATA:
                reg.STAR = rxData;
                break;
            case RDSTATB:
                reg.STBR = rxData;
                break;
            case RDCOMM:
                reg.COMM = rxData;
            default:
                break;
        }
    }

    digitalWrite(pinCS, HIGH);
    SPI_local.endTransaction();

    return pecCorrect;
}

template <std::size_t Nodes>
void LTC68041<Nodes>::spi_write_cmd(const uint16_t cmd) {
    auto start = millis();

    wakeup_idle();  // This will guarantee that the LTC6804 isoSPI port is awake, this command can be removed.

    SPI_local.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
    digitalWrite(pinCS, LOW);

    if ((cmd & Commands::ADCV) == Commands::ADCV ||
        (cmd & Commands::ADAX) == Commands::ADAX ||
        (cmd & Commands::ADCVAX) == Commands::ADCVAX ||
        (cmd & Commands::ADCVSC) == Commands::ADCVSC ||
        (cmd & Commands::ADSTAT) == Commands::ADSTAT)
        {
        if (pollADC) {
            // poll ADC for ongoing conversion
            SPI_local.transfer16(PLADC);
            SPI_local.transfer16(calcPEC15(PLADC));

            // PLADC response status is valid after N = number of Nodes clock cycles, so dump bytes that are partially valid
            for(int i = 0; i < ((Nodes / 8) + 1); i++)
                SPI_local.transfer(0xFF);

            while (SPI_local.transfer(0xFF) == 0x00) {
                if((millis() - start) > 1000) {
                    digitalWrite(pinCS, HIGH);
                    SPI_local.endTransaction();
                    return;
                }
            }

            digitalWrite(pinCS, HIGH);
            delayMicroseconds(2);  // toggle CS between commands
            digitalWrite(pinCS, LOW);
        } else {
            pollADC = true;
                }
        }

    SPI_local.transfer16(cmd);
    SPI_local.transfer16(calcPEC15(cmd));

    digitalWrite(pinCS, HIGH);
    SPI_local.endTransaction();
}

template <std::size_t Nodes>
bool LTC68041<Nodes>::waitForConversion() {
    bool ret = true;
    auto start = millis();

    wakeup_idle();  // This will guarantee that the LTC6804 isoSPI port is awake, this command can be removed.

    SPI_local.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
    digitalWrite(pinCS, LOW);

    SPI_local.transfer16(PLADC);
    SPI_local.transfer16(calcPEC15(PLADC));

    // PLADC response status is valid after N = number of Nodes clock cycles, so dump bytes that are partially valid
    for(int i = 0; i < ((Nodes / 8) + 1); i++)
        SPI_local.transfer(0xFF);

    while (SPI_local.transfer(0xFF) == 0x00) {
        if((millis() - start) > 1000) {
            ret = false;
            break;
        }
    }

    digitalWrite(pinCS, HIGH);
    SPI_local.endTransaction();

    return ret;
}

template <std::size_t Nodes>
void LTC68041<Nodes>::cfgSetVUV(const float Undervoltage) {
    unsigned int VUV = static_cast<unsigned int>(Undervoltage / (0.0001f * 16.0f)) - 1;  // calc bitpattern for UV

    // regs.CFGR[CFGR0] = 0xFE;
    for (auto &reg : regs) {
        reg.CFGR[CFGR1 & 0x0F] = VUV & CFG1_VUV_MSK;  // 0x4E1 ; // 2.0V
        reg.CFGR[CFGR2 & 0x0F] = (reg.CFGR[CFGR2 & 0x0F] & (~CFG2_VUV_MSK)) | ((VUV >> 8) & CFG2_VUV_MSK);
    }
}

template <std::size_t Nodes>
float LTC68041<Nodes>::cfgGetVUV() const {
    unsigned int value;

    value = regs[0].CFGR[CFGR1 & 0x0F] | (static_cast<unsigned int>(regs[0].CFGR[CFGR2 & 0x0F] & CFG2_VUV_MSK) << 8);
    return (static_cast<float>(value + 1) * 16.0f * 0.001f);
}

template <std::size_t Nodes>
void LTC68041<Nodes>::cfgSetVOV(const float Overvoltage) {
    unsigned int VOV = static_cast<unsigned int>(Overvoltage / (0.0001f * 16.0f));  // Calc bitpattern for OV

    // regs.CFGR[CFGR0] = 0xFE;
    for( auto &reg : regs) {
        reg.CFGR[CFGR2 & 0x0F] = (reg.CFGR[CFGR2 & 0x0F] & (~CFG2_VOV_MSK)) | ((VOV << 4) & CFG2_VOV_MSK);
        reg.CFGR[CFGR3 & 0x0F] = (VOV >> 4) & CFG3_VOV_MSK;
    }
}

template <std::size_t Nodes>
float LTC68041<Nodes>::cfgGetVOV() const {
    unsigned int value;

    value = ((regs[0].CFGR[CFGR2 & 0x0F] & CFG2_VOV_MSK) >> 4) | (static_cast<unsigned int>(regs[0].CFGR[CFGR3 & 0x0F]) << 4);
    return (static_cast<float>(value) * 16.0f * 0.001f);
}

template <std::size_t Nodes>
void LTC68041<Nodes>::cfgSetDischargeTimeout(DischargeTimeout timeout) {
    for (auto &reg : regs)
        reg.CFGR[CFGR5 & 0x0F] = (reg.CFGR[CFGR5 & 0x0F] & (~CFG5_DCTO_MSK)) | timeout;
}

template <std::size_t Nodes>
void LTC68041<Nodes>::cfgSetADCMode(ADCFilterMode mode) {
    for (auto &reg : regs) {
        switch (mode) {
            case ADCFilterMode::BANDWIDTH_422HZ:
                md = MD_OPT;
                reg.CFGR0w &= ~(1 << CFGR0_ADCOPT_Pos);
                break;
            case ADCFilterMode::BANDWIDTH_27KHZ:
                md = MD_FAST;
                reg.CFGR0w &= ~(1 << CFGR0_ADCOPT_Pos);
                break;
            case ADCFilterMode::BANDWIDTH_7KHZ:
                md = MD_NORMAL;
                reg.CFGR0w &= ~(1 << CFGR0_ADCOPT_Pos);
                break;
            case ADCFilterMode::BANDWIDTH_26HZ:
                md = MD_FILTERED;
                reg.CFGR0w &= ~(1 << CFGR0_ADCOPT_Pos);
                break;
            case ADCFilterMode::BANDWIDTH_1KHZ:
                md = MD_OPT;
                reg.CFGR0w = (reg.CFGR0w & (~CFG0_ADCOPT_MSK)) | (1 << CFGR0_ADCOPT_Pos);
                break;
            case ADCFilterMode::BANDWIDTH_14KHZ:
                md = MD_FAST;
                reg.CFGR0w = (reg.CFGR0w & (~CFG0_ADCOPT_MSK)) | (1 << CFGR0_ADCOPT_Pos);
                break;
            case ADCFilterMode::BANDWIDTH_3KHZ:
                md = MD_NORMAL;
                reg.CFGR0w = (reg.CFGR0w & (~CFG0_ADCOPT_MSK)) | (1 << CFGR0_ADCOPT_Pos);
                break;
            case ADCFilterMode::BANDWIDTH_2KHZ:
                md = MD_FILTERED;
                reg.CFGR0w = (reg.CFGR0w & (~CFG0_ADCOPT_MSK)) | (1 << CFGR0_ADCOPT_Pos);
                break;
            default:
                break;
        }
    }
}

template <std::size_t Nodes>
LTC68041<Nodes>::ADCFilterMode LTC68041<Nodes>::cfgGetADCMode() const {
    switch (md) {
        case MD_OPT:
            if (regs[0].CFGR0r & CFG0_ADCOPT_MSK) {
                return ADCFilterMode::BANDWIDTH_1KHZ;
            } else {
                return ADCFilterMode::BANDWIDTH_422HZ;
            }
        case MD_FAST:
            if (regs[0].CFGR0r & CFG0_ADCOPT_MSK) {
                return ADCFilterMode::BANDWIDTH_14KHZ;
            } else {
                return ADCFilterMode::BANDWIDTH_27KHZ;
            }
        case MD_NORMAL:
            if (regs[0].CFGR0r & CFG0_ADCOPT_MSK) {
                return ADCFilterMode::BANDWIDTH_3KHZ;
            } else {
                return ADCFilterMode::BANDWIDTH_7KHZ;
            }
        case MD_FILTERED:
            if (regs[0].CFGR0r & CFG0_ADCOPT_MSK) {
                return ADCFilterMode::BANDWIDTH_2KHZ;
            } else {
                return ADCFilterMode::BANDWIDTH_26HZ;
            }
        default:
            return ADCFilterMode::NORMAL;
    }
}

template <std::size_t Nodes>
void LTC68041<Nodes>::cfgSetRefOn(const bool value) {
    for (auto &reg : regs)
        reg.CFGR0w = (reg.CFGR0w & (~CFG0_REFON_MSK)) | (value << CFGR0_REFON_Pos);
}

template <std::size_t Nodes>
bool LTC68041<Nodes>::cfgGetRefOn() {
    return (regs[0].CFGR0r & CFG0_REFON_MSK);
}

template <std::size_t Nodes>
bool LTC68041<Nodes>::cfgGetSWTENPin() const {
    return (regs[0].CFGR0r & CFG0_SWTRD_MSK);
}

/*!******************************************************************************************************
This command will write the configuration registers of the LTC6804-1.
Write the LTC6804 configuration register
  1. Load cmd array with the write configuration command and PEC
  2. Load the cmd with LTC6804 configuration data
  3. Calculate the pec for the LTC6804 configuration data being transmitted
  4. Write configuration data to the LTC6804

 uint8_t config[6] is an array of the configuration data that will be written.
*********************************************************************************************************/
template <std::size_t Nodes>
void LTC68041<Nodes>::cfgWrite()  // A two dimensional array of the configuration data that will be written
{
    uint16_t cmd = WRCFG;

    wakeup_idle();  // This will guarantee that the LTC6804 isoSPI port is awake, this command can be removed.

    SPI_local.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
    digitalWrite(pinCS, LOW);

    SPI_local.transfer16(cmd);
    SPI_local.transfer16(calcPEC15(cmd));

    for (auto &reg : regs) {
        reg.CFGR[CFGR0 & 0x0F] = reg.CFGR0w;

        for (const auto &element : reg.CFGR) {
            SPI_local.transfer(element);
        }

        SPI_local.transfer16(calcPEC15(reg.CFGR));
    }

    digitalWrite(pinCS, HIGH);
    SPI_local.endTransaction();
    /*
        //2
        cmd_index = 4;
        // the last IC on the stack. The first configuration written is
        // received by the last IC in the daisy chain

        for (uint8_t current_byte = 0; current_byte < BYTES_IN_REG; current_byte++) // executes for each of the 6 bytes in the CFGR register
        {
            // current_byte is the byte counter

            cmd[cmd_index] = config[current_byte];            //adding the config data to the array to be sent
            cmd_index = cmd_index + 1;
        }
        //3
        cfg_pec = (uint16_t)pec15_calc(BYTES_IN_REG, &config[0]);   // calculating the PEC for each ICs configuration register data
        cmd[cmd_index] = (uint8_t)(cfg_pec >> 8);
        cmd[cmd_index + 1] = (uint8_t)cfg_pec;
        cmd_index = cmd_index + 2;


        //4
        spi_write_array(CMD_LEN, cmd);
        free(cmd);
    */
}

/*!******************************************************************************************************
 Clears the LTC6804 Auxiliary registers

 The command clears the Auxiliary registers and intiallizes
 all values to 1. The register will read back hexadecimal 0xFF
 after the command is sent.
 LTC68041<Nodes>::clraux Function sequence:

  1. Load clraux command into cmd array
  2. Calculate clraux cmd PEC and load pec into cmd array
  3. send broadcast clraux command
*********************************************************************************************************/
template <std::size_t Nodes>
void LTC68041<Nodes>::clrAuxRegs() {
    // 4
    spi_write_cmd(CLRAUX);

    isCacheInvalid[RegGroups::AVAR] = true;
    isCacheInvalid[RegGroups::AVBR] = true;
}

/*!*******************************************************************************************************
The function is used to read the of the Config Register Group A  of one LTC6804.
This function sends the read commands, parses the data and stores the response in CFGR.

1. Set command to RDCFG
2. Calc the PEC
3. Wakeup Chip
4. Send command and read response
5. Check PEC
6. Copy data to object
7. Return Result -1= Error , 0= DataOkay
*********************************************************************************************************/
template <std::size_t Nodes>
bool LTC68041<Nodes>::cfgRead() {
    bool ret;

    ret = spi_read_cmd(RDCFG);

    for (auto &reg : regs) {
        reg.CFGR0r = reg.CFGR[CFGR0 & 0x0F];

        DEBUG_PRINT("Config Register Group: ");

        for (const auto &element : reg.CFGR) {
            DEBUG_PRINT(element, HEX);
            DEBUG_PRINT(" ");
        }

        DEBUG_PRINTLN();
    }

    return ret;
}

/*!*******************************************************************************************************
The command clears the cell voltage registers and intiallizes
all values to 1. The register will read back hexadecimal 0xFF
after the command is sent.

1. Load clrcell command into cmd array
2. Calculate clrcell cmd PEC and load pec into cmd array
3. send broadcast clrcell command to LTC6804
*********************************************************************************************************/
template <std::size_t Nodes>
void LTC68041<Nodes>::clrCellRegs() {
    // 4
    spi_write_cmd(CLRCELL);

    isCacheInvalid[RegGroups::CVAR] = true;
    isCacheInvalid[RegGroups::CVBR] = true;
    isCacheInvalid[RegGroups::CVCR] = true;
    isCacheInvalid[RegGroups::CVDR] = true;
}

/*!*******************************************************************************************************
A complete SPI communication check.
Reads the Status register and checks PECs of the response
No other command necessary, Just call this and get
[in] bool Result of the Check 1=Communication ok, 0=failure
1. Request for status register
2. Read fully status register
3. calc PEC from response
4. extract PEC from response
5. compare PECs
6. Send Serial message with result
*********************************************************************************************************/
template <std::size_t Nodes>
bool LTC68041<Nodes>::checkSPI() {
    bool ret = spi_read_cmd(RDCFG);

    DEBUG_PRINTLN();
    DEBUG_PRINT("RSP: ");

    for(const auto &reg : regs) {
        for (const auto &element : reg.CFGR) {
            DEBUG_PRINT(element, HEX);
            DEBUG_PRINT(" ");
        }

        DEBUG_PRINTLN();
    }

    DEBUG_PRINTLN();

    if (ret) {
        DEBUG_PRINTLN("PEC was correct");

        return true;
    } else {
        DEBUG_PRINTLN("PEC was NOT correct, check Hardware");

        return false;
    }
}

/*!*******************************************************************************************************
  Starts cell voltage ADC conversions of the LTC6804 Cpin inputs.
  The type of ADC conversion executed can be changed by setting the associated global variables:
  MD   Determines the filter corner of the ADC
  CH   Determines which cell channels are converted
  DCP  Determines if Discharge is Permitted
*********************************************************************************************************/
template <std::size_t Nodes>
unsigned long LTC68041<Nodes>::startCellConv(DischargeCtrl dcp, CellChannel ch) {
    uint16_t cmd = ADCV;
    cmd |= md;
    cmd |= dcp;
    cmd |= ch;

    isCacheInvalid[RegGroups::CVAR] = true;
    isCacheInvalid[RegGroups::CVBR] = true;
    isCacheInvalid[RegGroups::CVCR] = true;
    isCacheInvalid[RegGroups::CVDR] = true;

    // 3
    spi_write_cmd(cmd);

    switch (cfgGetADCMode()) {
        case ADCFilterMode::BANDWIDTH_27KHZ:
        case ADCFilterMode::BANDWIDTH_14KHZ:
            if (ch == CellChannel::CH_ALL) {
                return 1;
            } else {
                return 0;
            }
        case ADCFilterMode::BANDWIDTH_7KHZ:
            if (ch == CellChannel::CH_ALL) {
                return 2;
            } else {
                return 0;
            }
        case ADCFilterMode::BANDWIDTH_3KHZ:
            if (ch == CellChannel::CH_ALL) {
                return 3;
            } else {
                return 0;
            }
        case ADCFilterMode::BANDWIDTH_2KHZ:
            if (ch == CellChannel::CH_ALL) {
                return 4;
            } else {
                return 0;
            }
        case ADCFilterMode::BANDWIDTH_1KHZ:
            if (ch == CellChannel::CH_ALL) {
                return 7;
            } else {
                return 1;
            }
        case ADCFilterMode::BANDWIDTH_422HZ:
            if (ch == CellChannel::CH_ALL) {
                return 12;
            } else {
                return 2;
            }
        case ADCFilterMode::BANDWIDTH_26HZ:
            if (ch == CellChannel::CH_ALL) {
                return 200;
            } else {
                return 33;
            }
        default:
            return 0;
    }
}

template <std::size_t Nodes>
unsigned long LTC68041<Nodes>::startCellSocConv(DischargeCtrl dcp) {
    uint16_t cmd = ADCVSC;
    cmd |= md;
    cmd |= dcp;

    isCacheInvalid[RegGroups::CVAR] = true;
    isCacheInvalid[RegGroups::CVBR] = true;
    isCacheInvalid[RegGroups::CVCR] = true;
    isCacheInvalid[RegGroups::CVDR] = true;
    isCacheInvalid[RegGroups::STAR] = true;

    // 3
    spi_write_cmd(cmd);

    switch (cfgGetADCMode()) {
        case ADCFilterMode::BANDWIDTH_27KHZ:
        case ADCFilterMode::BANDWIDTH_14KHZ:
            return 1;
        case ADCFilterMode::BANDWIDTH_7KHZ:
            return 2;
        case ADCFilterMode::BANDWIDTH_3KHZ:
            return 3;
        case ADCFilterMode::BANDWIDTH_2KHZ:
            return 5;
        case ADCFilterMode::BANDWIDTH_1KHZ:
            return 8;
        case ADCFilterMode::BANDWIDTH_422HZ:
            return 14;
        case ADCFilterMode::BANDWIDTH_26HZ:
            return 234;
        default:
            return 0;
    }
}

/*!******************************************************************************************************
Starts cell voltage conversion with test values from selftest 2
*********************************************************************************************************/
template <std::size_t Nodes>
void LTC68041<Nodes>::startCellConvTest(SelfTestMode st) {
    uint16_t cmd = CVST;
    cmd |= md;
    cmd |= st;

    // 3
    spi_write_cmd(cmd);
}

/*!*******************************************************************************************************
  Starts an ADC conversions of the LTC6804 GPIO inputs.
  The type of ADC conversion executed can be changed by setting the associated global variables.
  1. Load adax command into cmd array
  2. Calculate adax cmd PEC and load pec into cmd array
  3. send broadcast adax command to LTC6804
*********************************************************************************************************/
template <std::size_t Nodes>
unsigned long LTC68041<Nodes>::startAuxConv(AuxChannel chg) {
    uint16_t cmd = ADAX;
    cmd |= md;
    cmd |= chg;

    isCacheInvalid[RegGroups::AVAR] = true;
    isCacheInvalid[RegGroups::AVBR] = true;

    spi_write_cmd(cmd);

    switch (cfgGetADCMode()) {
        case ADCFilterMode::BANDWIDTH_27KHZ:
        case ADCFilterMode::BANDWIDTH_14KHZ:
            if (chg == AuxChannel::CHG_ALL) {
                return 1;
            } else {
                return 0;
            }
        case ADCFilterMode::BANDWIDTH_7KHZ:
            if (chg == AuxChannel::CHG_ALL) {
                return 2;
            } else {
                return 0;
            }
        case ADCFilterMode::BANDWIDTH_3KHZ:
            if (chg == AuxChannel::CHG_ALL) {
                return 3;
            } else {
                return 0;
            }
        case ADCFilterMode::BANDWIDTH_2KHZ:
            if (chg == AuxChannel::CHG_ALL) {
                return 4;
            } else {
                return 0;
            }
        case ADCFilterMode::BANDWIDTH_1KHZ:
            if (chg == AuxChannel::CHG_ALL) {
                return 7;
            } else {
                return 1;
            }
        case ADCFilterMode::BANDWIDTH_422HZ:
            if (chg == AuxChannel::CHG_ALL) {
                return 12;
            } else {
                return 2;
            }
        case ADCFilterMode::BANDWIDTH_26HZ:
            if (chg == AuxChannel::CHG_ALL) {
                return 200;
            } else {
                return 33;
            }
        default:
            return 0;
    }
}

/*!*******************************************************************************************************
  Starts an ADC conversions of all cell voltages and the LTC6804 GPIO1 and GPIO2 inputs.
  The type of ADC conversion executed can be changed by setting the associated global variables.
  1. Load command into cmd array
  2. Calculate adax cmd PEC and load pec into cmd array
  3. send broadcast adax command to LTC6804
*********************************************************************************************************/
template <std::size_t Nodes>
unsigned long LTC68041<Nodes>::startCellAuxConv(DischargeCtrl dcp) {
    uint16_t cmd = ADCVAX;
    cmd |= md;
    cmd |= dcp;

    isCacheInvalid[RegGroups::CVAR] = true;
    isCacheInvalid[RegGroups::CVBR] = true;
    isCacheInvalid[RegGroups::CVCR] = true;
    isCacheInvalid[RegGroups::CVDR] = true;
    isCacheInvalid[RegGroups::AVAR] = true;

    spi_write_cmd(cmd);

    switch (cfgGetADCMode()) {
        case ADCFilterMode::BANDWIDTH_27KHZ:
        case ADCFilterMode::BANDWIDTH_14KHZ:
            return 1;
        case ADCFilterMode::BANDWIDTH_7KHZ:
            return 3;
        case ADCFilterMode::BANDWIDTH_3KHZ:
            return 4;
        case ADCFilterMode::BANDWIDTH_2KHZ:
            return 5;
        case ADCFilterMode::BANDWIDTH_1KHZ:
            return 9;
        case ADCFilterMode::BANDWIDTH_422HZ:
            return 17;
        case ADCFilterMode::BANDWIDTH_26HZ:
            return 268;
        default:
            return 0;
    }
}

/*!*******************************************************************************************************
  Starts an ADC conversions of the status values
  The type of ADC conversion executed can be changed by setting the associated global variables.
  1. Load command into cmd array
  2. Calculate adax cmd PEC and load pec into cmd array
  3. send broadcast adax command to LTC6804
*********************************************************************************************************/
template <std::size_t Nodes>
unsigned long LTC68041<Nodes>::startStatusConv(StatusGroup chst) {
    uint16_t cmd = ADSTAT;
    cmd |= md;
    cmd |= chst;

    isCacheInvalid[RegGroups::STAR] = true;
    isCacheInvalid[RegGroups::STBR] = true;

    spi_write_cmd(cmd);

    switch (cfgGetADCMode()) {
        case ADCFilterMode::BANDWIDTH_27KHZ:
        case ADCFilterMode::BANDWIDTH_14KHZ:
            if (chst == StatusGroup::CHST_ALL) {
                return 0;
            } else {
                return 0;
            }
        case ADCFilterMode::BANDWIDTH_7KHZ:
            if (chst == StatusGroup::CHST_ALL) {
                return 1;
            } else {
                return 0;
            }
        case ADCFilterMode::BANDWIDTH_3KHZ:
        case ADCFilterMode::BANDWIDTH_2KHZ:
            if (chst == StatusGroup::CHST_ALL) {
                return 2;
            } else {
                return 0;
            }
        case ADCFilterMode::BANDWIDTH_1KHZ:
            if (chst == StatusGroup::CHST_ALL) {
                return 4;
            } else {
                return 1;
            }
        case ADCFilterMode::BANDWIDTH_422HZ:
            if (chst == StatusGroup::CHST_ALL) {
                return 8;
            } else {
                return 2;
            }
        case ADCFilterMode::BANDWIDTH_26HZ:
            if (chst == StatusGroup::CHST_ALL) {
                return 134;
            } else {
                return 33;
            }
        default:
            return 0;
    }
}

/*!*******************************************************************************************************
  Starts an ADC conversions of the open wire check with pullup
  The type of ADC conversion executed can be changed by the command value
  1. Load command into cmd array
  2. Calculate adax cmd PEC and load pec into cmd array
  3. send broadcast adax command to LTC6804
*********************************************************************************************************/
template <std::size_t Nodes>
void LTC68041<Nodes>::startOpenWireCheck(PUPCtrl pup, DischargeCtrl dcp, CellChannel ch) {
    uint16_t cmd = ADOW;
    cmd |= md;
    cmd |= pup;
    cmd |= dcp;
    cmd |= ch;

    spi_write_cmd(cmd);
}

/*!******************************************************************************************************
Sets  the configuration array for cell balancing
1. Reset all Discharge Pins
2. Calculate adcv cmd PEC and load pec into cmd array
Discharge this cell (1-12), disable all other, IF -1 then all off
*********************************************************************************************************/
template <std::size_t Nodes>
template <unsigned int N>
requires (N < Nodes)
void LTC68041<Nodes>::cfgSetDCC(std::bitset<12> dcc) {
    // assert 0x0fff
    regs[N].CFGR[CFGR4 & 0x0F] = (dcc.to_ulong() & CFG4_DCC_MSK);  // (regs[N].CFGRx[CFGR1] & CFG1_DCC_INVMSK) |
    regs[N].CFGR[CFGR5 & 0x0F] = (regs[N].CFGR[CFGR5 & 0x0F] & (~CFG5_DCC_MSK)) | ((dcc.to_ulong() >> 8) & CFG5_DCC_MSK);
}

template <std::size_t Nodes>
template <unsigned int N>
requires (N < Nodes)
std::bitset<12> LTC68041<Nodes>::cfgGetDCC() const {
    return std::bitset<12>{regs[N].CFGR[CFGR4 & 0x0F] | (static_cast<unsigned long long>(regs[N].CFGR[CFGR5 & 0x0F] & CFG5_DCC_MSK) << 8)};
}

/*!*******************************************************************************************************
Reads and parses the LTC6804 cell voltage registers.

The function is used to read the cell codes of the LTC6804.
This function will send the requested read commands parse the data
and store the cell voltages in cell_codes variable.

1. Read every single cell voltage register
2. Parse raw cell voltage data in cell_codes array
3. Check the PEC of the data read back vs the calculated PEC for each read register command
4. Return pec_error flag
*********************************************************************************************************/
template <std::size_t Nodes>
template <std::size_t N, unsigned int M>
//requires (N <= LTC68041<Nodes>::CELLNUM)
bool LTC68041<Nodes>::getCellVoltages(std::array<float, N> &voltages) {
    static constexpr std::array<ValueNames, CELLNUM> cells = {C1V, C2V, C3V, C4V, C5V, C6V, C7V, C8V, C9V, C10V, C11V, C12V};

    DEBUG_PRINT("Node: ");
    DEBUG_PRINTLN(M);
    DEBUG_PRINTLN("===============================");

    if(isCacheInvalid[RegGroups::CVAR]) {
        if (!spi_read_cmd(RDCVA)) {
            return false;
        } else {
            isCacheInvalid[RegGroups::CVAR] = false;

            DEBUG_PRINT("Cell Voltage Register Group A: ");

            for (const auto &element : regs[M].CVAR) {
                DEBUG_PRINT(element, HEX);
                DEBUG_PRINT(" ");
            }

            DEBUG_PRINTLN();
        }
    }

    if(isCacheInvalid[RegGroups::CVBR]) {
        if (!spi_read_cmd(RDCVB)) {
            return false;
        } else {
            isCacheInvalid[RegGroups::CVBR] = false;

            DEBUG_PRINT("Cell Voltage Register Group B: ");

            for (const auto &element : regs[M].CVBR) {
                DEBUG_PRINT(element, HEX);
                DEBUG_PRINT(" ");
            }

            DEBUG_PRINTLN();
        }
    }

    if(isCacheInvalid[RegGroups::CVCR]) {
        if (!spi_read_cmd(RDCVC)) {
            return false;
        } else {
            isCacheInvalid[RegGroups::CVCR] = false;

            DEBUG_PRINT("Cell Voltage Register Group C: ");

            for (const auto &element : regs[M].CVCR) {
                DEBUG_PRINT(element, HEX);
                DEBUG_PRINT(" ");
            }

            DEBUG_PRINTLN();
        }
    }

    if(isCacheInvalid[RegGroups::CVDR]) {
        if (!spi_read_cmd(RDCVD)) {
            return false;
        } else {
            isCacheInvalid[RegGroups::CVDR] = false;

            DEBUG_PRINT("Cell Voltage Register Group D: ");

            for (const auto &element : regs[M].CVDR) {
                DEBUG_PRINT(element, HEX);
                DEBUG_PRINT(" ");
            }

            DEBUG_PRINTLN();
        }
    }

    auto cell = cells.cbegin();
    auto rcell = cells.crbegin();

    for (auto it = voltages.begin(); it < (voltages.cbegin() + (N / 2)); it++) {
        *it = parseVoltage<M>(*cell);
        cell++;
    }

    for (auto it = voltages.rbegin(); it < (voltages.crbegin() + (N / 2)); it++) {
        *it = parseVoltage<M>(*rcell);
        rcell++;
    }

#if DEBUG
        DEBUG_PRINTLN("Cell Voltages: ");

        if constexpr (N == 12)
            DEBUG_PRINTLN("Cell 1  Cell 2  Cell 3  Cell 4  Cell 5  Cell 6  Cell 7  Cell 8  Cell 9  Cell 10 Cell 11 Cell 12");
        else if constexpr (N == 8)
            DEBUG_PRINTLN("Cell 1  Cell 2  Cell 3  Cell 4  Cell 5  Cell 6  Cell 7  Cell 8");
        else if constexpr (N == 6)
            DEBUG_PRINTLN("Cell 1  Cell 2  Cell 3  Cell 4  Cell 5  Cell 6");

        for (const auto &element : voltages) {
            DEBUG_PRINT(element);
            DEBUG_PRINT(" V");
            DEBUG_PRINT("  ");
        }

        DEBUG_PRINTLN();
#endif

    return true;
}

/*!*******************************************************************************************************
The function is used to read the  parsed GPIO codes of the LTC6804.
This function will send the requested  read commands parse the data
and store the gpio voltages in aux_codes variable

[in] uint8_t reg; This controls which GPIO voltage register is read back.
0: Read back all auxiliary registers
1: Read back auxiliary group A
2: Read back auxiliary group B
[out] uint16_t aux_codes[][6]; A two dimensional array of the gpio voltage codes.
[return]  int8_t, PEC Status  0: No PEC error detected -1: PEC error detected, retry read

*********************************************************************************************************/
template <std::size_t Nodes>
template <unsigned int N>
float LTC68041<Nodes>::getAuxVoltage(const AuxChannel chg) {
#if DEBUG
    static bool dbgPrint = false;
#endif

    DEBUG_PRINT("Node: ");
    DEBUG_PRINTLN(N);
    DEBUG_PRINTLN("===============================");

    if(isCacheInvalid[RegGroups::AVAR]) {
        if (!spi_read_cmd(RDAUXA)) {
            return NAN;
        } else {
            isCacheInvalid[RegGroups::AVAR] = false;

            DEBUG_PRINT("Auxiliary Register Group A: ");

            for (const auto &element : regs[N].AVAR) {
                DEBUG_PRINT(element, HEX);
                DEBUG_PRINT(" ");
            }

            DEBUG_PRINTLN();

#if DEBUG
            dbgPrint = true;
#endif
        }
    }

    if(isCacheInvalid[RegGroups::AVBR]) {
        if (!spi_read_cmd(RDAUXB)) {
            return NAN;
        } else {
            isCacheInvalid[RegGroups::AVBR] = false;

            DEBUG_PRINT("Auxiliary Register Group B: ");

            for (const auto &element : regs[N].AVBR) {
                DEBUG_PRINT(element, HEX);
                DEBUG_PRINT(" ");
            }

            DEBUG_PRINTLN();

#if DEBUG
            dbgPrint = true;
#endif
        }
    }

#if DEBUG
    if (dbgPrint) {
        dbgPrint = false;
        std::array<float, AUXNUM> auxVoltage{};  // Voltage of GPIOs and VREF2 in Volt

        auxVoltage[0] = parseVoltage<N>(G1V);
        auxVoltage[1] = parseVoltage<N>(G2V);
        auxVoltage[2] = parseVoltage<N>(G3V);
        auxVoltage[3] = parseVoltage<N>(G4V);
        auxVoltage[4] = parseVoltage<N>(G5V);
        auxVoltage[5] = parseVoltage<N>(REF);

        DEBUG_PRINTLN("Auxiliary Voltages: ");
        DEBUG_PRINTLN("GPIO1   GPIO2   GPIO3   GPIO4   GPIO5   Vref2");

        for (const auto &element : auxVoltage) {
            DEBUG_PRINT(element);
            DEBUG_PRINT(" V");
            DEBUG_PRINT("  ");
        }

        DEBUG_PRINTLN();
    }
#endif

    switch (chg) {
        case AuxChannel::CHG_GPIO1:
            return parseVoltage<N>(G1V);
        case AuxChannel::CHG_GPIO2:
            return parseVoltage<N>(G2V);
        case AuxChannel::CHG_GPIO3:
            return parseVoltage<N>(G3V);
        case AuxChannel::CHG_GPIO4:
            return parseVoltage<N>(G4V);
        case AuxChannel::CHG_GPIO5:
            return parseVoltage<N>(G5V);
        case AuxChannel::CHG_VREF2:
            return parseVoltage<N>(REF);
        case AuxChannel::CHG_ALL:
            return NAN;
        default:
            return NAN;
    }
}

/*!*******************************************************************************************************
The function is used to read the of the Status Register Group A of one LTC6804.
This function sends the read commands, parses the data and stores the response in STAR.

1. Set command to RDSTATA
2. Calc the PEC
3. Wakeup Chip
4. Send command and read response
5. Check PEC
6. Copy data to object
7. Return Result -1= Error , 0= DataOkay
*********************************************************************************************************/
template <std::size_t Nodes>
template <unsigned int N>
float LTC68041<Nodes>::getStatusVoltage(const StatusGroup chst) {
#if DEBUG
    static bool dbgPrint = false;
#endif

    DEBUG_PRINT("Node: ");
    DEBUG_PRINTLN(N);
    DEBUG_PRINTLN("===============================");

    if(isCacheInvalid[RegGroups::STAR]) {
        if (!spi_read_cmd(RDSTATA)) {
            return NAN;
        } else {
            isCacheInvalid[RegGroups::STAR] = false;

            DEBUG_PRINT("RSP Status Register Group A: ");

            for (const auto &element : regs[N].STAR) {
                DEBUG_PRINT(element, HEX);
                DEBUG_PRINT(" ");
            }

            DEBUG_PRINTLN();

#if DEBUG
            dbgPrint = true;
#endif
        }
    }

    if (isCacheInvalid[RegGroups::STBR]) {
        if (!spi_read_cmd(RDSTATB)) {
            return NAN;
        } else {
            isCacheInvalid[RegGroups::STBR] = false;

            DEBUG_PRINT("RSP Status Register Group B: ");

            for (const auto &element : regs[N].STBR) {
                DEBUG_PRINT(element, HEX);
                DEBUG_PRINT(" ");
            }

            DEBUG_PRINTLN();

#if DEBUG
            dbgPrint = true;
#endif
        }
    }

#if DEBUG
    if (dbgPrint) {
        dbgPrint = false;
        DEBUG_PRINT("Internal Temperature: ");
        DEBUG_PRINT((parseVoltage<N>(ITMP) / 7.5E-3f - 273.0f) + offsetTemp);
        DEBUG_PRINTLN(" °C");

        DEBUG_PRINT("Sum of all Cells Voltage: ");
        DEBUG_PRINT(parseVoltage<N>(SC) * 20.0f);
        DEBUG_PRINTLN(" V");

        DEBUG_PRINT("Analog Supply Voltage: ");
        DEBUG_PRINT(parseVoltage<N>(VA));
        DEBUG_PRINTLN(" V");

        DEBUG_PRINT("Digital Supply Voltage: ");
        DEBUG_PRINT(parseVoltage<N>(VD));
        DEBUG_PRINTLN(" V");

        DEBUG_PRINT("Overvoltageflags: ");
        DEBUG_PRINTLN(getStatusOverVoltageFlags<N>().to_ulong(), BIN);

        DEBUG_PRINT("Undervoltageflags: ");
        DEBUG_PRINTLN(getStatusUnderVoltageFlags<N>().to_ulong(), BIN);

        DEBUG_PRINT("Chip Revision: ");
        DEBUG_PRINTLN(getStatusRevision<N>(), DEC);

        DEBUG_PRINT("Muxfail: ");
        DEBUG_PRINTLN(getStatusMUXFail<N>());

        DEBUG_PRINT("Thermalshutdown: ");
        DEBUG_PRINTLN(getStatusThermalShutdown<N>());
        DEBUG_PRINTLN();
    }
#endif

    switch (chst) {
        case StatusGroup::CHST_SOC:
            // 16-Bit ADC Measurement Value of Sum of all cell voltages Sum of all cell voltages = SOC * 100µV * 20
            return parseVoltage<N>(SC) * 20.0f;
        case StatusGroup::CHST_ITMP:
            // 16-Bit ADC Measurement Value of Internal Die Temperature Temperature Measurement (°C) = ITMP * 100µV / 7.5mV/°C - 273°C
            return (parseVoltage<N>(ITMP) / 7.5E-3f - 273.0f) + offsetTemp;
        case StatusGroup::CHST_VA:
            // 16-Bit ADC Measurement Value of Analog Power Supply Voltage Analog Power Supply Voltage = VA * 100µV Normal Range Is within 4.5V to 5.5V
            return parseVoltage<N>(VA);
        case StatusGroup::CHST_VD:
            // 16-Bit ADC Measurement Value of Digital Power Supply Voltage Digital Power Supply Voltage = VA * 100µV Normal Range Is within 2.7V to 3.6V
            return parseVoltage<N>(VD);
        case StatusGroup::CHST_ALL:
            return NAN;
        default:
            return NAN;
    }
}

template <std::size_t Nodes>
template <unsigned int N>
bool LTC68041<Nodes>::getStatusMUXFail() {
    if(isCacheInvalid[RegGroups::STBR])
        if (!spi_read_cmd(RDSTATB))
            return false;
        else
            isCacheInvalid[RegGroups::STBR] = false;

    return (regs[N].STBR[STBR5 & 0x0F] & STBR5_MUXFAIL_MSK);
}

template <std::size_t Nodes>
template  <unsigned int N>
bool LTC68041<Nodes>::getStatusThermalShutdown() {
    if(isCacheInvalid[RegGroups::STBR])
        if (!spi_read_cmd(RDSTATB))
            return false;
        else
            isCacheInvalid[RegGroups::STBR] = false;

    return (regs[N].STBR[STBR5 & 0x0F] & STBR5_THSD_MSK);
}

// Cell x Overvoltage Flag x = 1 to 12 Cell Voltage Compared to VOV Comparison Voltage 0 -> Cell x Not Flagged for Overvoltage Condition. 1 -> Cell x Flagged
template <std::size_t Nodes>
template <unsigned int N>
std::bitset<12> LTC68041<Nodes>::getStatusOverVoltageFlags() {
    std::bitset<12> ret;

    if(isCacheInvalid[RegGroups::STBR])
        if (!spi_read_cmd(RDSTATB)) {
            ret.reset();
            return ret;
        } else {
            isCacheInvalid[RegGroups::STBR] = false;
        }

    ret[0] = bitRead(regs[N].STBR[STBR2 & 0x0F], 1);
    ret[1] = bitRead(regs[N].STBR[STBR2 & 0x0F], 3);
    ret[2] = bitRead(regs[N].STBR[STBR2 & 0x0F], 5);
    ret[3] = bitRead(regs[N].STBR[STBR2 & 0x0F], 7);
    ret[4] = bitRead(regs[N].STBR[STBR3 & 0x0F], 1);
    ret[5] = bitRead(regs[N].STBR[STBR3 & 0x0F], 3);
    ret[6] = bitRead(regs[N].STBR[STBR3 & 0x0F], 5);
    ret[7] = bitRead(regs[N].STBR[STBR3 & 0x0F], 7);
    ret[8] = bitRead(regs[N].STBR[STBR4 & 0x0F], 1);
    ret[9] = bitRead(regs[N].STBR[STBR4 & 0x0F], 3);
    ret[10] = bitRead(regs[N].STBR[STBR4 & 0x0F], 5);
    ret[11] = bitRead(regs[N].STBR[STBR4 & 0x0F], 7);

    return ret;
}

// Cell x Undervoltage Flag x = 1 to 12 Cell Voltage Compared to VUV Comparison Voltage 0 -> Cell x Not Flagged for Undervoltage Condition. 1 -> Cell x Flagged
template <std::size_t Nodes>
template <unsigned int N>
std::bitset<12> LTC68041<Nodes>::getStatusUnderVoltageFlags() {
    std::bitset<12> ret;

    if(isCacheInvalid[RegGroups::STBR])
        if (!spi_read_cmd(RDSTATB)) {
            ret.reset();
            return ret;
        } else {
            isCacheInvalid[RegGroups::STBR] = false;
        }

    isCacheInvalid[RegGroups::STBR] = false;

    ret[0] = bitRead(regs[N].STBR[STBR2 & 0x0F], 0);
    ret[1] = bitRead(regs[N].STBR[STBR2 & 0x0F], 2);
    ret[2] = bitRead(regs[N].STBR[STBR2 & 0x0F], 4);
    ret[3] = bitRead(regs[N].STBR[STBR2 & 0x0F], 6);
    ret[4] = bitRead(regs[N].STBR[STBR3 & 0x0F], 0);
    ret[5] = bitRead(regs[N].STBR[STBR3 & 0x0F], 2);
    ret[6] = bitRead(regs[N].STBR[STBR3 & 0x0F], 4);
    ret[7] = bitRead(regs[N].STBR[STBR3 & 0x0F], 6);
    ret[8] = bitRead(regs[N].STBR[STBR4 & 0x0F], 0);
    ret[9] = bitRead(regs[N].STBR[STBR4 & 0x0F], 2);
    ret[10] = bitRead(regs[N].STBR[STBR4 & 0x0F], 4);
    ret[11] = bitRead(regs[N].STBR[STBR4 & 0x0F], 6);

    return ret;
}

template <std::size_t Nodes>
template <unsigned int N>
int LTC68041<Nodes>::getStatusRevision() {
    if(isCacheInvalid[RegGroups::STBR])
        if (!spi_read_cmd(RDSTATB)) {
            return 0;
        } else {
            isCacheInvalid[RegGroups::STBR] = false;
        }

    return ((regs[N].STBR[STBR5 & 0x0F] & STBR5_REV_MSK) >> 4);
}

/**
 * @brief Helper function to calculate voltages in volt from register values
 *
 * @param value value to parse from registers, from ValueNames enum
 * @retval value as float in Volt
 */
template <std::size_t Nodes>
template <unsigned int N>
requires (N < Nodes)
constexpr inline float LTC68041<Nodes>::parseVoltage(const ValueNames value) {
    switch (value) {
        case ValueNames::C1V:
        case ValueNames::C2V:
        case ValueNames::C3V:
            return static_cast<float>(regs[N].CVAR[value & 0x0F] | (static_cast<unsigned int>(regs[N].CVAR[(value & 0x0F) + 1]) << 8u)) * 100E-6f;
        case ValueNames::C4V:
        case ValueNames::C5V:
        case ValueNames::C6V:
            return static_cast<float>(regs[N].CVBR[value & 0x0F] | (static_cast<unsigned int>(regs[N].CVBR[(value & 0x0F) + 1]) << 8u)) * 100E-6f;
        case ValueNames::C7V:
        case ValueNames::C8V:
        case ValueNames::C9V:
            return static_cast<float>(regs[N].CVCR[value & 0x0F] | (static_cast<unsigned int>(regs[N].CVCR[(value & 0x0F) + 1]) << 8u)) * 100E-6f;
        case ValueNames::C10V:
        case ValueNames::C11V:
        case ValueNames::C12V:
            return static_cast<float>(regs[N].CVDR[value & 0x0F] | (static_cast<unsigned int>(regs[N].CVDR[(value & 0x0F) + 1]) << 8u)) * 100E-6f;
        case ValueNames::G1V:
        case ValueNames::G2V:
        case ValueNames::G3V:
            return static_cast<float>(regs[N].AVAR[value & 0x0F] | (static_cast<unsigned int>(regs[N].AVAR[(value & 0x0F) + 1]) << 8u)) * 100E-6f;
        case ValueNames::G4V:
        case ValueNames::G5V:
        case ValueNames::REF:
            return static_cast<float>(regs[N].AVBR[value & 0x0F] | (static_cast<unsigned int>(regs[N].AVBR[(value & 0x0F) + 1]) << 8u)) * 100E-6f;
        case ValueNames::SC:
        case ValueNames::ITMP:
        case ValueNames::VA:
            return static_cast<float>(regs[N].STAR[value & 0x0F] | (static_cast<unsigned int>(regs[N].STAR[(value & 0x0F) + 1]) << 8u)) * 100E-6f;
        case ValueNames::VD:
            return static_cast<float>(regs[N].STBR[value & 0x0F] | (static_cast<unsigned int>(regs[N].STBR[(value & 0x0F) + 1]) << 8u)) * 100E-6f;
        default:
        break;
    }

    return NAN;
}

inline void serialPrint(uint8_t data) {
    DEBUG_PRINT(data, HEX);
}

inline void serialPrint(bool data) {
    DEBUG_PRINT(data);
}

inline void serialPrint(float data) {
    DEBUG_PRINT(data);
}

template <typename T, std::size_t N>
void printArray(std::array<T, N> &arr) {
    DEBUG_PRINTLN();
    DEBUG_PRINT("Array Content | ");

    for (const auto &element : arr) {
        serialPrint(element);
        DEBUG_PRINT("\t");
    }

    DEBUG_PRINT(" |END \n");
}
