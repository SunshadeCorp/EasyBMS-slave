/************************************************************

This library is based on the LTC68041.cpp by linear technology.
http://www.linear.com/product/LTC6804-1

I modified it make it compatible with the ESP8622

https://github.com/jontubs/EasyBMS
***********************************************************/

#include "LTC68041.h"

#include <cmath>
#include <cstdint>
#include <concepts>

/**
 * @brief Creating of the object LTC68041
 *
 * @param pCS Pin used as chip select
 */
template <std::size_t Nodes>
LTC68041::LTC68041(byte pCS, float tempOffset) : offsetTemp(tempOffset), md(MD_NORMAL), pinCS(pCS), regs({}), SPI_local(FSPI), isCacheInvalid(0x3FFFF) {
    Serial.print("Objekt angelegt");

    for (auto &reg : regs)
        reg.CFGR0w = 0xFE;
}

/**
 * @brief Initializes the SPI instance used for communication
 *
 * @param pinMOSI Pin used as MOSI
 * @param pinMISO Pin used as MISO
 * @param pinCLK Pin used as SCK
 */
void LTC68041::initSPI(byte pinMOSI, byte pinMISO, byte pinCLK) {
    pinMode(pinMOSI, OUTPUT);
    pinMode(pinMISO, INPUT);
    pinMode(pinCLK, OUTPUT);
    pinMode(pinCS, OUTPUT);

    SPI_local.begin(pinCLK, pinMISO, pinMOSI, -1);
}

/**
 * @brief Uninitializes the used SPI instance
 *
 */
void LTC68041::destroySPI() {
    SPI_local.end();
}

/**
 * @brief Wake isoSPI up from idle state
 *        Generic wakeup commannd to wake isoSPI up out of idle
 */
void LTC68041::wakeup_idle() const {
    digitalWrite(pinCS, LOW);
    delayMicroseconds(2);  // Guarantees the isoSPI will be in ready mode
    digitalWrite(pinCS, HIGH);
}

/*!******************************************************************************************************
Calculates the CRC sum of some data bytes given by the array "data"
*********************************************************************************************************/
constexpr uint16_t LTC68041::calcPEC15(const uint16_t data) const {
    uint16_t remainder = 16, addr = 0;  // initialize the PEC

    addr = ((remainder >> 7) ^ (data >> 8)) & 0xff;  // calculate PEC table address
    remainder = (remainder << 8) ^ crc15Table[addr];

    addr = ((remainder >> 7) ^ (data & 0xff)) & 0xff;  // calculate PEC table address
    remainder = (remainder << 8) ^ crc15Table[addr];

    return (remainder * 2);  // The CRC15 has a 0 in the LSB so the remainder must be multiplied by 2
}

/*!******************************************************************************************************
Calculates the CRC sum of some data bytes given by the array "data"
*********************************************************************************************************/
template <std::size_t N>
constexpr uint16_t LTC68041::calcPEC15(const std::array<uint8_t, N> &data) const {
    uint16_t remainder = 16, addr = 0;  // initialize the PEC

    for (const auto &element : data)  // loops for each byte in data array
    {
        addr = ((remainder >> 7) ^ element) & 0xff;  // calculate PEC table address
        remainder = (remainder << 8) ^ crc15Table[addr];
    }

    return (remainder * 2);  // The CRC15 has a 0 in the LSB so the remainder must be multiplied by 2
}

/*!******************************************************************************************************
Writes and read a set number of bytes using the SPI port.
Tested and runs fine
[in] std::array<uint8_t, N1> &tx_Data array of data to be written on the SPI port
[out] std::array<uint8_t, N2> &rx_data array that read data will be written too.
*********************************************************************************************************/
bool LTC68041::spi_read_cmd(Commands cmd) {
    uint16_t pec = calcPEC15(cmd);
    bool pecCorrect = false;
    std::array<uint8_t, SIZEREG> rxData;

    wakeup_idle();  // This will guarantee that the LTC6804 isoSPI port is awake, this command can be removed.

    SPI_local.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
    digitalWrite(pinCS, LOW);

    SPI_local.transfer16(cmd);
    SPI_local.transfer16(pec);

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

/*!******************************************************************************************************
Writes and read a set number of bytes using the SPI port without expecting an answer
std::array<uint8_t, N> &data //Array of bytes to be written on the SPI port
*********************************************************************************************************/
void LTC68041::spi_write_cmd(const uint16_t cmd) {
    uint16_t pec = calcPEC15(cmd);

    wakeup_idle();  // This will guarantee that the LTC6804 isoSPI port is awake, this command can be removed.

    SPI_local.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
    digitalWrite(pinCS, LOW);

    SPI_local.transfer16(cmd);
    SPI_local.transfer16(pec);

    digitalWrite(pinCS, HIGH);
    SPI_local.endTransaction();
}

/*!******************************************************************************************************
calculates the bitpattern in the config for Undervoltage detection
the config has to be written to the chip after this!
*********************************************************************************************************/
void LTC68041::cfgSetVUV(const float Undervoltage) {
    unsigned int VUV = static_cast<unsigned int>(Undervoltage / (0.0001f * 16.0f)) - 1;  // calc bitpattern for UV

    // regs.CFGR[CFGR0] = 0xFE;
    for (auto &reg : regs) {
        reg.CFGR[CFGR1 & 0x0F] = VUV & CFG1_VUV_MSK;  // 0x4E1 ; // 2.0V
        reg.CFGR[CFGR2 & 0x0F] = (reg.CFGR[CFGR2 & 0x0F] & (~CFG2_VUV_MSK)) | ((VUV >> 8) & CFG2_VUV_MSK);
    }
}

float LTC68041::cfgGetVUV() const {
    unsigned int value;

    value = regs[0].CFGR[CFGR1 & 0x0F] | (static_cast<unsigned int>(regs[0].CFGR[CFGR2 & 0x0F] & CFG2_VUV_MSK) << 8);
    return (static_cast<float>(value + 1) * 16.0f * 0.001f);
}

/*!******************************************************************************************************
calculates the bitpattern in the config for Overvoltage detection
the config has to be written to the chip after this!
*********************************************************************************************************/
void LTC68041::cfgSetVOV(const float Overvoltage) {
    // float Undervoltage=3.123;
    // float Overvoltage=3.923;
    unsigned int VOV = static_cast<unsigned int>(Overvoltage / (0.0001f * 16.0f));  // Calc bitpattern for OV

    // regs.CFGR[CFGR0] = 0xFE;
    for( auto &reg : regs) {
        reg.CFGR[CFGR2 & 0x0F] = (reg.CFGR[CFGR2 & 0x0F] & (~CFG2_VOV_MSK)) | ((VOV << 4) & CFG2_VOV_MSK);
        reg.CFGR[CFGR3 & 0x0F] = (VOV >> 4) & CFG3_VOV_MSK;
    }
}

float LTC68041::cfgGetVOV() const {
    unsigned int value;

    value = ((regs[0].CFGR[CFGR2 & 0x0F] & CFG2_VOV_MSK) >> 4) | (static_cast<unsigned int>(regs[0].CFGR[CFGR3 & 0x0F]) << 4);
    return (static_cast<float>(value) * 16.0f * 0.001f);
}

/*!******************************************************************************************************
Sets  the configuration array for cell balancing
  1. Reset all Discharge Pins
  2. Calculate adcv cmd PEC and load pec into cmd array
  Discharge this cell (1-12), disable all other, IF -1 then all off
*********************************************************************************************************/
void LTC68041::cfgSetDCC(std::bitset<12> dcc) {
    // assert 0x0fff
    for (auto &reg : regs) {
        reg.CFGR[CFGR4 & 0x0F] = (dcc.to_ulong() & CFG4_DCC_MSK);  // (reg.CFGRx[CFGR1] & CFG1_DCC_INVMSK) |
        reg.CFGR[CFGR5 & 0x0F] = (reg.CFGR[CFGR5 & 0x0F] & (~CFG5_DCC_MSK)) | ((dcc.to_ulong() >> 8) & CFG5_DCC_MSK);
    }
}

std::bitset<12> LTC68041::cfgGetDCC() const {
    return std::bitset<12>{regs[0].CFGR[CFGR4 & 0x0F] | (static_cast<unsigned long long>(regs[0].CFGR[CFGR5 & 0x0F] & CFG5_DCC_MSK) << 8)};
}

void LTC68041::cfgSetDischargeTimeout(DischargeTimeout timeout) {
    for (auto &reg : regs)
        reg.CFGR[CFGR5 & 0x0F] = (reg.CFGR[CFGR5 & 0x0F] & (~CFG5_DCTO_MSK)) | timeout;
}

/**
 * @brief Set ADC filter Mode out of the six possible modes.
 *        Handles setting of MD bits in command and ADCOPT bit in CFGR0w
 *
 * @param mode ADC mode as enum value of type ADCFilterMode
 */
void LTC68041::cfgSetADCMode(ADCFilterMode mode) {
    for (auto &reg : regs) {
        switch (mode) {
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

void LTC68041::cfgSetRefOn(const bool value) {
    for (auto &reg : regs)
        reg.CFGR0w = (reg.CFGR0w & (~CFG0_REFON_MSK)) | (value << CFGR0_REFON_Pos);
}

bool LTC68041::cfgGetRefOn() {
    return (regs[0].CFGR0r & CFG0_REFON_MSK);
}

bool LTC68041::cfgGetSWTENPin() const {
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
void LTC68041::cfgWrite()  // A two dimensional array of the configuration data that will be written
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

/*!*******************************************************************************************************
Perform LUT lookup of cell SOC based on cell open circuit voltage.
SOC based on OCV lookup table.
SOC as a function of open cell voltage is non=linear, a lookup table seems to
be the best way to map between these two values.
voc: Cell open circuit voltage, Volts.
return Function returns SOC from 0-1.

Funktion ist kaputt , muss ich mal fixen
TODO: refactor
*********************************************************************************************************/
float LTC68041::cellComputeSOC(float voc) {
    /*const double t_offset = 3.00205;
    const double t_step = 0.01;
    const double t_gain = 1000;
    u_int16_t tbl[] =
    {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,17,18,19,21,22,23,25,26,28,30,31,33,35,37,39,41,44,46,49,52,55,58,61,65,69,73,78,84,90,97,105,115,126,141,159,182,210,242,277,314,352,389,427,465,503,541,579,616,654,692,730,768,806,843,878,905,923,935,943,949,954,958,961,964,967,969,971,973,975,977,978,979,981,982,983,984,985,986,987,988,989,990,991,991,992,993,994,994,995,996,996,997,997,998,};*/
    // offset: 2.41498V, step size:0.01V
    const float t_offset = 2.415;
    const float t_step = 0.01;
    const float t_gain = 1000;
    float result;
    uint16_t tbl[] = {
        0,   1,   2,   3,   5,   6,   7,   8,   10,  11,  12,  14,  15,  17,  18,  20,  22,  24,  25,  27,  29,  32,  34,
        36,  39,  42,  45,  48,  51,  55,  59,  63,  68,  73,  79,  87,  95,  105, 117, 133, 153, 181, 216, 258, 303, 349,
        396, 443, 490, 537, 584, 632, 679, 726, 773, 820, 865, 902, 925, 938, 947, 953, 958, 962, 966, 969, 971, 974, 976,
        978, 979, 981, 983, 984, 985, 986, 988, 989, 990, 991, 992, 993, 994, 994, 995, 996, 997, 997, 998, 999, 999, 1000,
    };

    // number of elements in lut
    if (voc <= t_offset) {
        return 0.0;
    }
    // table size
    unsigned int n = sizeof(tbl) / sizeof(tbl[0]);
    // compute index
    unsigned int index = (unsigned int)((voc - t_offset) / t_step);
    // compute fractional index
    float f_index = ((voc - t_offset) / t_step) - index;
    // limit to valid table index, -1
    index = (index < n - 2) ? index : n - 2;
    f_index = (f_index < 1.0) ? f_index : 1.0;  // don't extrapolate beyond LUT limits
    // compute local slope
    float delta = (tbl[index + 1] - tbl[index]) / t_gain;
    result = tbl[index] / t_gain + f_index * delta;
    Serial.print("\nresult:");
    Serial.print(result);
    return result;
}

/*!******************************************************************************************************
 Clears the LTC6804 Auxiliary registers

 The command clears the Auxiliary registers and intiallizes
 all values to 1. The register will read back hexadecimal 0xFF
 after the command is sent.
 LTC68041::clraux Function sequence:

  1. Load clraux command into cmd array
  2. Calculate clraux cmd PEC and load pec into cmd array
  3. send broadcast clraux command
*********************************************************************************************************/
void LTC68041::clrAuxRegs() {
    // 4
    spi_write_cmd(CLRAUX);
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
template <std::size_t N, unsigned int M = 0>
requires N <= CELLNUM
bool LTC68041::getCellVoltages(std::array<float, N> &voltages) {
    static constexpr std::array<ValueNames, CELLNUM> cells = {C1V, C2V, C3V, C4V, C5V, C6V, C7V, C8V, C9V, C10V, C11V, C12V};

    if(isCacheInvalid[RegGroups::CVAR])
        if (!spi_read_cmd(RDCVA))
            return false;
        else
            isCacheInvalid[RegGroups::CVAR] = false;

    if(isCacheInvalid[RegGroups::CVBR])
        if (!spi_read_cmd(RDCVB))
            return false;
        else
            isCacheInvalid[RegGroups::CVBR] = false;

    if(isCacheInvalid[RegGroups::CVCR])
        if (!spi_read_cmd(RDCVC))
            return false;
        else
            isCacheInvalid[RegGroups::CVCR] = false;

    if(isCacheInvalid[RegGroups::CVDR])
        if (!spi_read_cmd(RDCVD))
            return false;
        else
            isCacheInvalid[RegGroups::CVDR] = false;

    auto cell = cells.cbegin();
    auto rcell = cells.crbegin();

    for (auto it = voltages.begin(); it < (voltages.cbegin() + ((N / 2) - 1)); it++) {
        it* = parseVoltage<M>(cell*);
        cell++;
    }

    for (auto it = voltages.rbegin(); it < (voltages.crbegin() + ((N / 2) - 1)); it++) {
        it* = parseVoltage<M>(rcell*);
        rcell++;
    }

    return true;
}

/**
 * @brief Helper function to calculate voltages in volt from register values
 *
 * @param value value to parse from registers, from ValueNames enum
 * @retval value as float in Volt
 */
template <unsigned int N>
requires N < Nodes
constexpr inline float LTC68041::parseVoltage(const ValueNames value) {
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
bool LTC68041::cfgRead() {
    bool ret;

    ret = spi_read_cmd(RDCFG);

    for (auto &reg : regs)
        reg.CFGR0r = reg.CFGR[CFGR0 & 0x0F];

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
void LTC68041::clrCellRegs() {
    // 4
    spi_write_cmd(CLRCELL);
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
bool LTC68041::checkSPI(const bool dbgOut) {
    if (dbgOut) digitalWrite(LED_BUILTIN, HIGH);

    bool ret = spi_read_cmd(RDCFG);

    if (dbgOut) {
        digitalWrite(LED_BUILTIN, LOW);
        Serial.println();
        Serial.print("RSP: ");

        for (const auto &element : regs.CFGR) {
            Serial.print(element, HEX);
            Serial.print(" ");
        }

        Serial.println();
    }

    if (ret) {
        if (dbgOut) Serial.println("PEC was correct");

        return true;
    } else {
        if (dbgOut) Serial.println("PEC was NOT correct, check Hardware");

        return false;
    }
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
template <unsigned int N = 0>
float LTC68041::getAuxVoltage(const AuxChannel chg) {
    if(isCacheInvalid[RegGroups::AVAR])
        if (!spi_read_cmd(RDAUXA))
            return NAN;
         else
            isCacheInvalid[RegGroups::AVAR] = false;

    if(isCacheInvalid[RegGroups::AVBR])
        if (!spi_read_cmd(RDAUXB))
            return NAN;
         else
            isCacheInvalid[RegGroups::AVBR] = false;

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
template <unsigned int N = 0>
float LTC68041::getStatusVoltage(const StatusGroup chst) {
    if(isCacheInvalid[RegGroups::STAR])
        if (!spi_read_cmd(RDSTATA))
            return NAN;
         else
            isCacheInvalid[RegGroups::STAR] = false;

    if(isCacheInvalid[RegGroups::STBR])
        if (!spi_read_cmd(RDSTATB))
            return NAN;
         else
            isCacheInvalid[RegGroups::STBR] = false;

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

template <unsigned int N = 0>
bool LTC68041::getStatusMUXFail() {
    if(isCacheInvalid[RegGroups::STBR])
        if (!spi_read_cmd(RDSTATB))
            return false;
        else
            isCacheInvalid[RegGroups::STBR] = false;

    return (regs[N].STBR[STBR5 & 0x0F] & STBR5_MUXFAIL_MSK);
}

template <unsigned int N = 0>
bool LTC68041::getStatusThermalShutdown() {
    if(isCacheInvalid[RegGroups::STBR])
        if (!spi_read_cmd(RDSTATB))
            return false;
         else
            isCacheInvalid[RegGroups::STBR] = false;

    return (regs[N].STBR[STBR5 & 0x0F] & STBR5_THSD_MSK);
}

// Cell x Overvoltage Flag x = 1 to 12 Cell Voltage Compared to VOV Comparison Voltage 0 -> Cell x Not Flagged for Overvoltage Condition. 1 -> Cell x Flagged
template <unsigned int N = 0>
std::bitset<12> LTC68041::getStatusOverVoltageFlags() {
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
template <unsigned int N = 0>
std::bitset<12> LTC68041::getStatusUnderVoltageFlags() {
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

template <unsigned int N = 0>
int LTC68041::getStatusRevision() {
    if(isCacheInvalid[RegGroups::STBR])
        if (!spi_read_cmd(RDSTATB)) {
            return 0;
        } else {
            isCacheInvalid[RegGroups::STBR] = false;
        }

    return ((regs[N].STBR[STBR5 & 0x0F] & STBR5_REV_MSK) >> 4);
}

/*!*******************************************************************************************************
  Starts cell voltage ADC conversions of the LTC6804 Cpin inputs.
  The type of ADC conversion executed can be changed by setting the associated global variables:
  MD   Determines the filter corner of the ADC
  CH   Determines which cell channels are converted
  DCP  Determines if Discharge is Permitted
*********************************************************************************************************/
void LTC68041::startCellConv(DischargeCtrl dcp, CellChannel ch) {
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
}

/*!******************************************************************************************************
Starts cell voltage conversion with test values from selftest 2
*********************************************************************************************************/
void LTC68041::startCellConvTest(SelfTestMode st) {
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
void LTC68041::startAuxConv(AuxChannel chg) {
    uint16_t cmd = ADAX;
    cmd |= md;
    cmd |= chg;

    isCacheInvalid[RegGroups::AVAR] = true;
    isCacheInvalid[RegGroups::AVBR] = true;

    spi_write_cmd(cmd);
}

/*!*******************************************************************************************************
  Starts an ADC conversions of all cell voltages and the LTC6804 GPIO1 and GPIO2 inputs.
  The type of ADC conversion executed can be changed by setting the associated global variables.
  1. Load command into cmd array
  2. Calculate adax cmd PEC and load pec into cmd array
  3. send broadcast adax command to LTC6804
*********************************************************************************************************/
void LTC68041::startCellAuxConv(DischargeCtrl dcp) {
    uint16_t cmd = ADCVAX;
    cmd |= md;
    cmd |= dcp;

    isCacheInvalid[RegGroups::CVAR] = true;
    isCacheInvalid[RegGroups::CVBR] = true;
    isCacheInvalid[RegGroups::CVCR] = true;
    isCacheInvalid[RegGroups::CVDR] = true;
    isCacheInvalid[RegGroups::AVAR] = true;

    spi_write_cmd(cmd);
}

/*!*******************************************************************************************************
  Starts an ADC conversions of the status values
  The type of ADC conversion executed can be changed by setting the associated global variables.
  1. Load command into cmd array
  2. Calculate adax cmd PEC and load pec into cmd array
  3. send broadcast adax command to LTC6804
*********************************************************************************************************/
void LTC68041::startStatusConv(StatusGroup chst) {
    uint16_t cmd = ADSTAT;
    cmd |= md;
    cmd |= chst;

    isCacheInvalid[RegGroups::STAR] = true;
    isCacheInvalid[RegGroups::STBR] = true;

    spi_write_cmd(cmd);
}

/*!*******************************************************************************************************
  Starts an ADC conversions of the open wire check with pullup
  The type of ADC conversion executed can be changed by the command value
  1. Load command into cmd array
  2. Calculate adax cmd PEC and load pec into cmd array
  3. send broadcast adax command to LTC6804
*********************************************************************************************************/
void LTC68041::startOpenWireCheck(PUPCtrl pup, DischargeCtrl dcp, CellChannel ch) {
    uint16_t cmd = ADOW;
    cmd |= md;
    cmd |= pup;
    cmd |= dcp;
    cmd |= ch;

    spi_write_cmd(cmd);
}

/*!*******************************************************************************************************
Prints out configuration registers of a LTC6804
Giving additional Debug infos via Serial
*********************************************************************************************************/
void LTC68041::readCfgDbg() {
    Serial.println();
    Serial.print("Config Register Group: ");

    for (const auto &element : regs[0].CFGR) {
        Serial.print(element, HEX);
        Serial.print(" ");
    }

    Serial.println();
}

/*!******************************************************************************************************
Prints out Status Register Groups and parsed values
*********************************************************************************************************/
void LTC68041::readStatusDbg() {
    Serial.println();
    Serial.print("RSP Status Register Group A: ");

    for (const auto &element : regs[0].STAR) {
        Serial.print(element, HEX);
        Serial.print(" ");
    }

    Serial.println();
    Serial.print("RSP Status Register Group B: ");

    for (const auto &element : regs[0].STBR) {
        Serial.print(element, HEX);
        Serial.print(" ");
    }

    Serial.println();
    Serial.print("Internal Temperature: ");
    Serial.print(getStatusVoltage(StatusGroup::CHST_ITMP));
    Serial.println(" °C");

    Serial.print("Sum of all Cells Voltage: ");
    Serial.print(getStatusVoltage(StatusGroup::CHST_SOC));
    Serial.println(" V");

    Serial.print("Analog Supply Voltage: ");
    Serial.print(getStatusVoltage(StatusGroup::CHST_VA));
    Serial.println(" V");

    Serial.print("Digital Supply Voltage: ");
    Serial.print(getStatusVoltage(StatusGroup::CHST_VD));
    Serial.println(" V");

    Serial.print("Overvoltageflags: ");
    Serial.println(getStatusOverVoltageFlags().to_ulong(), BIN);

    Serial.print("Undervoltageflags: ");
    Serial.println(getStatusUnderVoltageFlags().to_ulong(), BIN);

    Serial.print("Chip Revision: ");
    Serial.println(getStatusRevision(), DEC);

    Serial.print("Muxfail: ");
    Serial.println(getStatusMUXFail());

    Serial.print("Thermalshutdown: ");
    Serial.println(getStatusThermalShutdown());
}

void LTC68041::readAuxDbg() {
    std::array<float, AUXNUM> auxVoltage{};  // Voltage of GPIOS and VREF2 in Volt

    auxVoltage[0] = getAuxVoltage(CHG_GPIO1);
    auxVoltage[1] = getAuxVoltage(CHG_GPIO2);
    auxVoltage[2] = getAuxVoltage(CHG_GPIO3);
    auxVoltage[3] = getAuxVoltage(CHG_GPIO4);
    auxVoltage[4] = getAuxVoltage(CHG_GPIO5);
    auxVoltage[5] = getAuxVoltage(CHG_VREF2);

    Serial.println();
    Serial.print("Auxiliary Register Group A: ");

    for (const auto &element : regs.AVAR) {
        Serial.print(element, HEX);
        Serial.print(" ");
    }

    Serial.println();
    Serial.print("Auxiliary Register Group B: ");

    for (const auto &element : regs.AVBR) {
        Serial.print(element, HEX);
        Serial.print(" ");
    }

    Serial.println();
    Serial.println("Auxiliary Voltages: ");
    Serial.println("GPIO1   GPIO2   GPIO3   GPIO4   GPIO5   Vref2");

    for (const auto &element : auxVoltage) {
        Serial.print(element);
        Serial.print(" V");
        Serial.print("  ");
    }

    Serial.println();
}

/*!******************************************************************************************************
Reads and parses the LTC6804 cell voltage registers and returns some additional infos via Serial

 The function is used to print out Cell Voltage Register Groups
 and Cell Voltage values in Volt.
*********************************************************************************************************/
void LTC68041::readCellsDbg()  // Array of the parsed cell codes
{
    std::array<float, 12> cellVoltages{};

    getCellVoltages(cellVoltages);

    Serial.println();
    Serial.print("Cell Voltage Register Group A: ");

    for (const auto &element : regs.CVAR) {
        Serial.print(element, HEX);
        Serial.print(" ");
    }

    Serial.println();
    Serial.print("Cell Voltage Register Group B: ");

    for (const auto &element : regs.CVBR) {
        Serial.print(element, HEX);
        Serial.print(" ");
    }

    Serial.println();
    Serial.print("Cell Voltage Register Group C: ");

    for (const auto &element : regs.CVCR) {
        Serial.print(element, HEX);
        Serial.print(" ");
    }

    Serial.println();
    Serial.print("Cell Voltage Register Group D: ");

    for (const auto &element : regs.CVDR) {
        Serial.print(element, HEX);
        Serial.print(" ");
    }

    Serial.println();
    Serial.println("Cell Voltages: ");
    Serial.println("Cell 1  Cell 2  Cell 3  Cell 4  Cell 5  Cell 6  Cell 7  Cell 8  Cell 9  Cell 10 Cell 11 Cell 12");

    for (const auto &element : cellVoltages) {
        Serial.print(element);
        Serial.print(" V");
        Serial.print("  ");
    }

    Serial.println();
}

inline void serialPrint(uint8_t data) {
    Serial.print(data, HEX);
}

inline void serialPrint(bool data) {
    Serial.print(data);
}

inline void serialPrint(float data) {
    Serial.print(data);
}

template <typename T, std::size_t N>
void printArray(std::array<T, N> &arr) {
    Serial.println();
    Serial.print("Array Content | ");

    for (const auto &element : arr) {
        serialPrint(element);
        Serial.print("\t");
    }

    Serial.print(" |END \n");
}
