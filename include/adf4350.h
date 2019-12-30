#ifndef _ADF4350_HANDLER
#define _ADF4350_HANDLER

//Register definitions from the ADF4350 data sheet
//The names should be as per the data sheet to avoid confusion.

#define REG0 0
#define REG0_FRAC_BIT                         3
//FRAC is a 12 bit value
#define REG0_INT_BIT                          15
//INT is a 16 bit value
//Bit 31 is reserved




#define REG1 1
#define REG1_MODULUS_BIT                      3
//Modulus is a 12 bit field

#define REG1_PHASE_BIT                        15
//Phase is a 12 bit field
#define PHASE_RECOMMENDED                     1

#define REG1_PRESCALER_BIT                    27
#define REG1_PRESCALLER_4_5                   0
#define REG1_PRESCALLER_8_9                   1
//Prescaler, 4/5 when PLL <= 3GHz otherwise 8/9
//Bits 28-31 are reserved




#define REG2 2
#define REG2_COUNTER_RESET_BIT                  3
#define REG2_COUNTER_RESET_DISABLED             0
#define REG2_COUNTER_RESET_ENABLED              1

#define REG2_CP_THREE_STATE_BIT                 4
#define REG2_CP_THREE_STATE_DISABLED            0
#define REG2_CP_THREE_STATE_ENABLED             1

#define REG2_POWER_DOWN_BIT                     5
#define REG2_POWER_DOWN_DISABLED                0
#define REG2_POWER_DOWN_ENABLED                 1

#define REG2_PD_POLARITY_BIT                    6
#define REG2_PD_POLARITY_NEGATIVE               0
#define REG2_PD_POLARITY_POSITIVE               1

#define REG2_LDP_BIT                            7
#define REG2_LDP_10_NS                          0
#define REG2_LDP_6_NS                           1

#define REG2_LDF_BIT                            8
#define REG2_LDF_FRAC_N                         0
#define REG2_LDF_INT_N                          1

#define REG2_CHARGE_PUMP_CURRENT_BIT            9
#define REG2_CHARGE_PUMP_CURRENT_0_31_MA        0
#define REG2_CHARGE_PUMP_CURRENT_0_63_MA        1
#define REG2_CHARGE_PUMP_CURRENT_0_94_MA        2
#define REG2_CHARGE_PUMP_CURRENT_1_25_MA        3
#define REG2_CHARGE_PUMP_CURRENT_1_56_MA        4
#define REG2_CHARGE_PUMP_CURRENT_1_88_MA        5
#define REG2_CHARGE_PUMP_CURRENT_2_19_MA        6
#define REG2_CHARGE_PUMP_CURRENT_2_50_MA        7
#define REG2_CHARGE_PUMP_CURRENT_2_81_MA        8
#define REG2_CHARGE_PUMP_CURRENT_3_13_MA        9
#define REG2_CHARGE_PUMP_CURRENT_3_44_MA        10
#define REG2_CHARGE_PUMP_CURRENT_3_75_MA        11
#define REG2_CHARGE_PUMP_CURRENT_4_06_MA        12
#define REG2_CHARGE_PUMP_CURRENT_4_38_MA        13
#define REG2_CHARGE_PUMP_CURRENT_4_69_MA        14
#define REG2_CHARGE_PUMP_CURRENT_5_00_MA        15

#define REG2_DOUBLE_BUFF_BIT                    13
#define REG2_DOUBLE_BUFF_DISABLED               0
#define REG2_DOUBLE_BUFF_ENABLED                1

#define REG2_R_COUNTER_BIT                      14
//10 bit R counter

#define REG2_RDIV2_BIT                          24 //Reference divide by two bit
#define REG2_RDIV2_DISABLED                     0
#define REG2_RDIV2_ENABLED                      1

#define REG2_REFERENCE_DOUBLER_BIT              25
#define REG2_REFERENCE_DOUBLER_DISABLED         0
#define REG2_REFERENCE_DOUBLER_ENABLED          1

#define REG2_MUXOUT_BIT                         26
#define REG2_MUXOUT_THREE_STATE_OUTPUT          0
#define REG2_MUXOUT_DV                          1
#define REG2_MUXOUT_DGND                        2
#define REG2_MUXOUT_R_DIVIDER_OUTPUT            3
#define REG2_MUXOUT_N_DIVIDER_OUTPUT            4
#define REG2_MUXOUT_ANALOG_LOCK_DETECT          5
#define REG2_MUXOUT_DIGITAL_LOCK_DETECT         6
#define REG2_MUXOUT_RESERVED                    7

#define REG2_LOW_NOISE_AND_LOW_SPUR_MODES_BIT   29
#define REG2_LOW_NOISE_MODE                     0
#define REG2_LOW_SPUR_MODE                      3
//Bit 31 is reserved




#define REG3                                    3
//Bits 19 to 31 are reserved
//Sub registers available and their values
#define REG3_12_BIT_CLOCK_DIVIDER_VALUE_BIT     3
//Values from 0 - 4095 can be set

#define REG3_CLK_DIV_MODE_BIT                   15
#define REG3_CLK_DIV_MODE_CLOCK_DIVIDER_OFF     0
#define REG3_CLK_DIV_MODE_FAST_LOCK_ENABLE      1
#define REG3_CLK_DIV_MODE_RESYNC_ENABLE         2
#define REG3_CLK_DIV_MODE_RESERVED              3

#define REG3_CSR_BIT                            18
#define REG3_CSR_DISABLED                       0
#define REG3_CSR_ENABLED                        1




#define REG4 4
//Bits 24-31 are reserved
//Sub registers available and their values
#define REG4_OUTPUT_POWER_BIT                    3 //base reg bit
#define REG4_OUTPUT_POWER_M4_DBM                 0
#define REG4_OUTPUT_POWER_M1_DBM                 1
#define REG4_OUTPUT_POWER_2_DBM                  2
#define REG4_OUTPUT_POWER_5_DBM                  3

#define REG4_RF_OUTPUT_ENABLE_BIT                5 //base reg bit
#define REG4_RF_OUTPUT_ENABLE_DISABLED           0
#define REG4_RF_OUTPUT_ENABLE_ENABLED            1

#define REG4_AUX_OUTPUT_POWER_BIT                6 //base reg bit
//values are the same as the OUTPUT_POWER reg
#define REG4_AUX_OUTPUT_ENABLE_BIT               8 //base reg bit
//Enabled/disabled values are the same as for the RF_OUTPUT_ENABLE reg

#define REG4_AUX_OUTPUT_SELECT_BIT               9 //base reg bit
#define REG4_AUX_OUTPUT_SELECT_DIVIDED_OUTPUT    0
#define REG4_AUX_OUTPUT_SELECT_FUNDAMENTAL       1

#define REG4_MTLD_BIT                            10 //base reg bit, stands for mute till lock detect
#define REG4_MTLD_DISABLED                       0
#define REG4_MTLD_ENABLED                        1

#define REG4_VCO_POWER_DOWN_BIT                  11 //base reg bit
#define REG4_VCO_POWERED_UP                      0
#define REG4_VCO_POWERED_DOWN                    1

#define REG4_8_BIT_BAND_SELECT_CLOCK_DIVIDER_VALUE_BIT  12 //base reg bit
//Valid values are 1-255

#define REG4_DIVIDER_SELECT_DBB_BIT              20 //base reg bit
#define REG4_DIVIDER_SELECT_DBB_DIV_1            0
#define REG4_DIVIDER_SELECT_DBB_DIV_2            1
#define REG4_DIVIDER_SELECT_DBB_DIV_4            2
#define REG4_DIVIDER_SELECT_DBB_DIV_8            3
#define REG4_DIVIDER_SELECT_DBB_DIV_16           4

#define REG4_FEEDBACK_SELECT_BIT                 23 //base reg bit
#define REG4_FEEDBACK_SELECT_DIVIDED             0
#define REG4_FEEDBACK_SELECT_FUNDAMENTAL         1




#define REG5                                    5
//Sub registers available and their values
#define REG5_LD_PIN_MODE_BIT                     22
//Bits 3-21 and 24-31 are reserved
#define REG5_LOCK_DETECT_PIN_OP_LOW1             0
#define REG5_LOCK_DETECT_PIN_OP_DIGITAL          1
#define REG5_LOCK_DETECT_PIN_OP_LOW2             2
#define REG5_LOCK_DETECT_PIN_OP_HIGH             3

#define REF_FREQ_MHZ                            10.0 //The ADF4350 reference clock frequency

#define SPI_CLOCK_HZ                            10000000    //10 MHz SPI clock

#define MIN_FREQ_MHZ                            137.5       //The minimum freq
#define MAX_FREQ_MHZ                            4400.0

//Structure to hold the state of all the ADF4530 registers
typedef struct _ADF4350Config {

    struct _Reg5 {
        uint8_t LD_PIN_MODE;
    } Reg5;

    struct _Reg4 {
        uint8_t OUTPUT_POWER;
        uint8_t RF_OUTPUT_ENABLE;
        uint8_t AUX_OUTPUT_POWER;
        uint8_t AUX_OUTPUT_ENABLE;
        uint8_t AUX_OUTPUT_SELECT;
        uint8_t MTLD;
        uint8_t VCO_POWER_DOWN;
        uint8_t _8_BIT_BAND_SELECT_CLOCK_DIVIDER_VALUE;
        uint8_t DIVIDER_SELECT_DBB;
        uint8_t FEEDBACK_SELECT;
    } Reg4;

    struct _Reg3 {
        uint16_t _12_BIT_CLOCK_DIVIDER_VALUE;
        uint8_t  CLK_DIV_MODE;
        uint8_t  CSR;
    } Reg3;

    struct _Reg2 {
        uint8_t  COUNTER_RESET;
        uint8_t  CP_THREE_STATE;
        uint8_t  POWER_DOWN;
        uint8_t  PD_POLARITY;
        uint8_t  LDP;
        uint8_t  LDF;
        uint8_t  CHARGE_PUMP_CURRENT;
        uint8_t  DOUBLE_BUFF;
        uint16_t R_COUNTER;
        uint8_t  RDIV2;
        uint8_t  REFERENCE_DOUBLER;
        uint8_t  MUXOUT;
        uint8_t  LOW_NOISE_AND_LOW_SPUR_MODES;
    } Reg2;

    struct _Reg1 {
        uint16_t MODULUS;
        uint8_t  PHASE;
        uint8_t  PRESCALER;
    } Reg1;

    struct _Reg0 {
        uint16_t FRAC;
        uint16_t INT;
    } Reg0;

} ADF4350Config;

typedef struct _Fraction {
    int numerator;
    int denominator;
} Fraction;

bool mgos_adf4350_init(void);
bool mgos_adf4350_freq(float freqMHz);

#endif
