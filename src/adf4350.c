#include <mgos.h>
#include <mgos_spi.h>
#include <adf4350.h>

static struct mgos_spi *spi;        //The spi bus from Mongoose os
static struct mgos_spi_txn txn;     //The structure used to define SPI communication with the Mongoose OS SPI bus.

static ADF4350Config adf4350Config;

/**
 * @brief activate/deactivate the CE pin on the ADF4350 device.
 * @param active If true then the CE pin is activated, else the pin is
 *               deactivated.
 * @return void
 **/
static void mgos_adf4350_activate_ce(bool active) {
     mgos_gpio_write(mgos_sys_config_get_adf4350_ce_pin(), active);
     LOG(LL_INFO, ("activateCE(%d)", active) );
}

/**
 * @brief Set the default settings for an ADF4350Config instance.
 *        This sets the same defaults as the AD example application defaults.
 * @param ADF4350Config A pointer to an ADF4350 config structure instance.
 * @return None
 */
static void setDefaultADF4350Config(ADF4350Config *ADF4350Config) {
    //Define the defaults for the ADF4350
    ADF4350Config->Reg5.LD_PIN_MODE=REG5_LOCK_DETECT_PIN_OP_DIGITAL;

    ADF4350Config->Reg4.OUTPUT_POWER=REG4_OUTPUT_POWER_5_DBM;
    ADF4350Config->Reg4.RF_OUTPUT_ENABLE=REG4_RF_OUTPUT_ENABLE_ENABLED;
    ADF4350Config->Reg4.AUX_OUTPUT_POWER=REG4_OUTPUT_POWER_M4_DBM;
    ADF4350Config->Reg4.AUX_OUTPUT_ENABLE=REG4_RF_OUTPUT_ENABLE_DISABLED;
    ADF4350Config->Reg4.AUX_OUTPUT_SELECT=REG4_AUX_OUTPUT_SELECT_DIVIDED_OUTPUT;
    ADF4350Config->Reg4.MTLD=REG4_MTLD_DISABLED;
    ADF4350Config->Reg4.VCO_POWER_DOWN=REG4_VCO_POWERED_UP;
    ADF4350Config->Reg4._8_BIT_BAND_SELECT_CLOCK_DIVIDER_VALUE=80;
    ADF4350Config->Reg4.DIVIDER_SELECT_DBB=REG4_DIVIDER_SELECT_DBB_DIV_1;
    ADF4350Config->Reg4.FEEDBACK_SELECT=REG4_FEEDBACK_SELECT_FUNDAMENTAL;

    ADF4350Config->Reg3._12_BIT_CLOCK_DIVIDER_VALUE=150;
    ADF4350Config->Reg3.CLK_DIV_MODE=REG3_CLK_DIV_MODE_CLOCK_DIVIDER_OFF;
    ADF4350Config->Reg3.CSR=REG3_CSR_DISABLED;

    ADF4350Config->Reg2.COUNTER_RESET=REG2_COUNTER_RESET_DISABLED;
    ADF4350Config->Reg2.CP_THREE_STATE=REG2_CP_THREE_STATE_DISABLED;
    ADF4350Config->Reg2.POWER_DOWN=REG2_POWER_DOWN_DISABLED;
    ADF4350Config->Reg2.PD_POLARITY=REG2_PD_POLARITY_POSITIVE;
    ADF4350Config->Reg2.LDP=REG2_LDP_10_NS;
    ADF4350Config->Reg2.LDF=REG2_LDF_FRAC_N;
    ADF4350Config->Reg2.CHARGE_PUMP_CURRENT=REG2_CHARGE_PUMP_CURRENT_2_50_MA;
    ADF4350Config->Reg2.DOUBLE_BUFF=REG2_DOUBLE_BUFF_DISABLED;
    ADF4350Config->Reg2.R_COUNTER=1;
    ADF4350Config->Reg2.RDIV2=REG2_RDIV2_DISABLED;
    ADF4350Config->Reg2.REFERENCE_DOUBLER=REG2_REFERENCE_DOUBLER_DISABLED;
    ADF4350Config->Reg2.MUXOUT=REG2_MUXOUT_THREE_STATE_OUTPUT;
    ADF4350Config->Reg2.LOW_NOISE_AND_LOW_SPUR_MODES=REG2_LOW_NOISE_MODE;

    ADF4350Config->Reg1.MODULUS=0;
    ADF4350Config->Reg1.PHASE=PHASE_RECOMMENDED;
    ADF4350Config->Reg1.PRESCALER=REG1_PRESCALLER_8_9;

    ADF4350Config->Reg0.FRAC=0;
    ADF4350Config->Reg0.INT=0;
}

/**
 * @brief Initialise the ADF4350 driver.
 *        This is called automatically when the ADF4350 library is included
 *        in the project.
 * @return true
 */
bool mgos_adf4350_init(void) {
    LOG(LL_INFO, ("mgos_adf4350_init()") );

    memset(&txn, 0 , sizeof(txn));

    //Set the CE pin as an output
    mgos_gpio_set_mode(mgos_sys_config_get_adf4350_ce_pin(), MGOS_GPIO_MODE_OUTPUT);
    //Initially set inactive
    mgos_adf4350_activate_ce(false);

    spi = mgos_spi_get_global();

    return true;
}

/**
 * @brief Get the contents of ADF4350 register 5
 */
static uint32_t getADF4350Reg5(ADF4350Config *adf4350Config) {
    uint32_t reg5;
    reg5 = 0x00180000 | REG5; //The Analog devices tool sets reserved bits so we set the same here
    reg5 |= adf4350Config->Reg5.LD_PIN_MODE<<REG5_LD_PIN_MODE_BIT;
    return reg5;
}

/**
 * @brief Get the contents of ADF4350 register 4
 */
static uint32_t getADF4350Reg4(ADF4350Config *adf4350Config) {
    uint32_t reg4;
    reg4 = REG4;
    reg4 |= adf4350Config->Reg4.OUTPUT_POWER<<REG4_OUTPUT_POWER_BIT;
    reg4 |= adf4350Config->Reg4.RF_OUTPUT_ENABLE<<REG4_RF_OUTPUT_ENABLE_BIT;
    reg4 |= adf4350Config->Reg4.AUX_OUTPUT_POWER<<REG4_AUX_OUTPUT_POWER_BIT;
    reg4 |= adf4350Config->Reg4.AUX_OUTPUT_ENABLE<<REG4_AUX_OUTPUT_ENABLE_BIT;
    reg4 |= adf4350Config->Reg4.AUX_OUTPUT_SELECT<<REG4_AUX_OUTPUT_SELECT_BIT;
    reg4 |= adf4350Config->Reg4.MTLD<<REG4_MTLD_BIT;
    reg4 |= adf4350Config->Reg4.VCO_POWER_DOWN<<REG4_VCO_POWER_DOWN_BIT;
    reg4 |= adf4350Config->Reg4._8_BIT_BAND_SELECT_CLOCK_DIVIDER_VALUE<<REG4_8_BIT_BAND_SELECT_CLOCK_DIVIDER_VALUE_BIT;
    reg4 |= adf4350Config->Reg4.DIVIDER_SELECT_DBB<<REG4_DIVIDER_SELECT_DBB_BIT;
    reg4 |= adf4350Config->Reg4.FEEDBACK_SELECT<<REG4_FEEDBACK_SELECT_BIT;

    return reg4;
}

/**
 * @brief Get the contents of ADF4350 register 3
 */
static uint32_t getADF4350Reg3(ADF4350Config *adf4350Config) {
    uint32_t reg3;
    reg3 = REG3;
    reg3 |= adf4350Config->Reg3._12_BIT_CLOCK_DIVIDER_VALUE<<REG3_12_BIT_CLOCK_DIVIDER_VALUE_BIT;
    reg3 |= adf4350Config->Reg3.CLK_DIV_MODE<<REG3_CLK_DIV_MODE_BIT;
    reg3 |= adf4350Config->Reg3.CSR<<REG3_CSR_BIT;
    return reg3;
}

/**
 * @brief Get the contents of ADF4350 register 2
 */
static uint32_t getADF4350Reg2(ADF4350Config *adf4350Config) {
    uint32_t reg2;
    reg2 = REG2;
    reg2 |= adf4350Config->Reg2.COUNTER_RESET<<REG2_COUNTER_RESET_BIT;
    reg2 |= adf4350Config->Reg2.CP_THREE_STATE<<REG2_CP_THREE_STATE_BIT;
    reg2 |= adf4350Config->Reg2.POWER_DOWN<<REG2_POWER_DOWN_BIT;
    reg2 |= adf4350Config->Reg2.PD_POLARITY<<REG2_PD_POLARITY_BIT;
    reg2 |= adf4350Config->Reg2.LDP<<REG2_LDP_BIT;
    reg2 |= adf4350Config->Reg2.LDF<<REG2_LDF_BIT;
    reg2 |= adf4350Config->Reg2.CHARGE_PUMP_CURRENT<<REG2_CHARGE_PUMP_CURRENT_BIT;
    reg2 |= adf4350Config->Reg2.DOUBLE_BUFF<<REG2_DOUBLE_BUFF_BIT;
    reg2 |= adf4350Config->Reg2.R_COUNTER<<REG2_R_COUNTER_BIT;
    reg2 |= adf4350Config->Reg2.RDIV2<<REG2_RDIV2_BIT;
    reg2 |= adf4350Config->Reg2.REFERENCE_DOUBLER<<REG2_REFERENCE_DOUBLER_BIT;
    reg2 |= adf4350Config->Reg2.MUXOUT<<REG2_MUXOUT_BIT;
    reg2 |= adf4350Config->Reg2.LOW_NOISE_AND_LOW_SPUR_MODES<<REG2_LOW_NOISE_AND_LOW_SPUR_MODES_BIT;

    return reg2;
}

/**
 * @brief Get the contents of ADF4350 register 1
 */
static uint32_t getADF4350Reg1(ADF4350Config *adf4350Config) {
    uint32_t reg1;
    reg1 = 0x80000000 | REG1; //The Analog devices tool sets MS bit (reserved bit) so we set the same here
    reg1 |= adf4350Config->Reg1.MODULUS<<REG1_MODULUS_BIT;
    reg1 |= adf4350Config->Reg1.PHASE<<REG1_PHASE_BIT;
    reg1 |= adf4350Config->Reg1.PRESCALER<<REG1_PRESCALER_BIT;
    return reg1;
}

/**
 * @brief Get the contents of ADF4350 register 0
 */
static uint32_t getADF4350Reg0(ADF4350Config *adf4350Config) {
    uint32_t reg0;
    reg0 = REG0;
    reg0 |= adf4350Config->Reg0.FRAC<<REG0_FRAC_BIT;
    reg0 |= adf4350Config->Reg0.INT<<REG0_INT_BIT;
    return reg0;
}

/**
 * @brief Get the value of the FRAC and MOD (numerator and denominator) values
 *        to give the required fraction.
 */
static void getFraction(float targetFrac, Fraction *fraction) {
    float       calcFrac=0;
    float       err=0;
    float       minErr=1E32;
    uint16_t    _mod=0;
    uint16_t    _frac=0;
#ifdef ADF4350_DEBUG
    double      startTime = mgos_uptime();
    double      stopTime = 0;
#endif

    for( _mod=2 ; _mod<4095 ; _mod++ ) {
        for( _frac=0 ; _frac<4095 ; _frac++) {
            if( _frac <= _mod ) { //frac cannot be more than mod
                calcFrac=((double)_frac)/((double)_mod);
                if( calcFrac > targetFrac ) {
                    err = calcFrac-targetFrac;
                }
                else {
                    err = targetFrac-calcFrac;
                }
                if( err < minErr ) {
                    minErr=err;
                    fraction->numerator=_frac;
                    fraction->denominator=_mod;
                    //When we're close enough exit (don't wate time)
                    if( err < 1E-3 ) {
                        break;
                    }
                }
            }
        }
        //When we're close enough exit (don't wate time)
        if( err < 1E-3 ) {
            break;
        }
    }

#ifdef ADF4350_DEBUG
  stopTime = mgos_uptime();
  LOG(LL_ERROR, ("%s: Execution took %f seconds", __FUNCTION__, stopTime-startTime) );
#endif
}

/**
 * @brief Write a 32bit value to a ADF4350 device over the SPI bus.
 * @param value The 32 bit value to write.
 * @return NULL
 */
static uint32_t mgos_adf4350_reg_wr(uint32_t value) {
  txn.cs = mgos_sys_config_get_adf4350_spi_cs();
  txn.mode = 3;
  txn.freq = SPI_CLOCK_HZ;

#ifdef ADF4350_DEBUG
  LOG(LL_ERROR, ("%s: value: 0x%08x", __FUNCTION__, value) );
#endif

  value = htonl(value);
  txn.fd.tx_data = txn.fd.rx_data = &value;
  txn.fd.len    = sizeof(value);

  if (!mgos_spi_run_txn(spi, true /* full duplex */, &txn)) {
    LOG(LL_ERROR, ("SPI transaction failed"));
  }

  value = ntohl(value);
  return value;
}

/**
 * @brief Set the frequency in MHz with (100 kHz channels)
 * @param freqMHz The freq required in MHz. 100 kHz accuracy.
 * @return true if the frequency requested has been set.
 **/
bool mgos_adf4350_freq(float freqMHz) {
  Fraction fraction;
  float Div=0.0;
  uint16_t intReg;

  fraction.numerator=-1;
  fraction.denominator=-1;

#ifdef ADF4350_DEBUG
  LOG(LL_ERROR, ("%s: freqMHz %f", __FUNCTION__, freqMHz) );
#endif

  mgos_adf4350_activate_ce(true);

  setDefaultADF4350Config(&adf4350Config);

  if( freqMHz >= 2200) {
      Div=1.0;
  }
  else if( freqMHz >= 1100) {
      Div=2.0;
  }
  else if( freqMHz >= 550 ) {
      Div=4.0;
  }
  else if( freqMHz >= 275 ) {
      Div=8.0;
  }
  else {
      Div=16.0;
  }

#ifdef ADF4350_DEBUG
  LOG(LL_ERROR, ("%s: Div %f", __FUNCTION__, Div) );
#endif


  if( freqMHz >= 2200) {
      adf4350Config.Reg4.DIVIDER_SELECT_DBB=REG4_DIVIDER_SELECT_DBB_DIV_1;
  }
  else if( freqMHz >= 1100) {
      adf4350Config.Reg4.DIVIDER_SELECT_DBB=REG4_DIVIDER_SELECT_DBB_DIV_2;
  }
  else if( freqMHz >= 550 ) {
      adf4350Config.Reg4.DIVIDER_SELECT_DBB=REG4_DIVIDER_SELECT_DBB_DIV_4;
  }
  else if( freqMHz >= 275 ) {
      adf4350Config.Reg4.DIVIDER_SELECT_DBB=REG4_DIVIDER_SELECT_DBB_DIV_8;
  }
  else {
      adf4350Config.Reg4.DIVIDER_SELECT_DBB=REG4_DIVIDER_SELECT_DBB_DIV_16;
  }

  intReg = (uint16_t)( freqMHz/(REF_FREQ_MHZ/Div) );
  adf4350Config.Reg0.INT=intReg;


  float targetFrac = (freqMHz/(REF_FREQ_MHZ/Div))-intReg;
#ifdef ADF4350_DEBUG
  LOG(LL_ERROR, ("%s: targetFrac = %f", __FUNCTION__, targetFrac) );
#endif

  getFraction(targetFrac, &fraction);
  //If no FRAC and MOD value was found
  if( fraction.numerator == -1 && fraction.denominator == -1 ) {
      return false;
  }

  adf4350Config.Reg0.FRAC=fraction.numerator;
  adf4350Config.Reg1.MODULUS=fraction.denominator;

#ifdef ADF4350_DEBUG
  LOG(LL_ERROR, ("%s: adf4350Config.Reg0.INT     = %d", __FUNCTION__, adf4350Config.Reg0.INT) );
  LOG(LL_ERROR, ("%s: adf4350Config.Reg0.FRAC    = %d", __FUNCTION__, adf4350Config.Reg0.FRAC) );
  LOG(LL_ERROR, ("%s: adf4350Config.Reg1.MODULUS = %d", __FUNCTION__, adf4350Config.Reg1.MODULUS) );
#endif

//Registers programmed in 5 4,3,2,1,0 order as per data sheet.
#ifdef ADF4350_DEBUG
  LOG(LL_ERROR, ("%s: ADF4350 REG 5", __FUNCTION__) );
#endif
  uint32_t reg5 = getADF4350Reg5(&adf4350Config);
  mgos_adf4350_reg_wr(reg5);

#ifdef ADF4350_DEBUG
  LOG(LL_ERROR, ("%s: ADF4350 REG 4", __FUNCTION__) );
#endif
  uint32_t reg4 = getADF4350Reg4(&adf4350Config);
  mgos_adf4350_reg_wr(reg4);

#ifdef ADF4350_DEBUG
  LOG(LL_ERROR, ("%s: ADF4350 REG 3", __FUNCTION__) );
#endif
  uint32_t reg3 = getADF4350Reg3(&adf4350Config);
  mgos_adf4350_reg_wr(reg3);

#ifdef ADF4350_DEBUG
  LOG(LL_ERROR, ("%s: ADF4350 REG 2", __FUNCTION__) );
#endif
  uint32_t reg2 = getADF4350Reg2(&adf4350Config);
  mgos_adf4350_reg_wr(reg2);

#ifdef ADF4350_DEBUG
  LOG(LL_ERROR, ("%s: ADF4350 REG 1", __FUNCTION__) );
#endif
  uint32_t reg1 = getADF4350Reg1(&adf4350Config);
  mgos_adf4350_reg_wr(reg1);

#ifdef ADF4350_DEBUG
  LOG(LL_ERROR, ("%s: ADF4350 REG 0", __FUNCTION__) );
#endif
  uint32_t reg0 = getADF4350Reg0(&adf4350Config);
  mgos_adf4350_reg_wr(reg0);

  return true;
}

/**
 * @brief Enable/disable RF output.
 *        Note !!! disabling RF output sets output level ~ 40 dB down.
 *             Powering off the device removes the output power (drops by > 70 dB)
 * @param enabled If 0 disable output, else enable.
 * @return void
 */
void mgos_enable_output(bool enabled) {
    uint32_t reg4 = getADF4350Reg4(&adf4350Config);

    if( enabled ) {
        reg4|=1<<REG4_RF_OUTPUT_ENABLE_BIT;
    }
    else {
        reg4&=~(1<<REG4_RF_OUTPUT_ENABLE_BIT);
    }


#ifdef ADF4350_DEBUG
  LOG(LL_ERROR, ("%s: ADF4350 REG 4", __FUNCTION__) );
#endif
    mgos_adf4350_reg_wr(reg4);

}

/**
 * @brief Set the RF output level.
 * @param dBm The output level in dBm. A limited range is available.
 *            Only -4, -1, 2 and 5 dBm are valid.
 * @return 0 on success, -1 on error (I.E if one of the above values was not set).
 */
int8_t mgos_set_output_level(int8_t dBm) {
    int32_t rc=-1;

    if( dBm == -4 || dBm == -1 || dBm == 2 || dBm == 5 ) {
        uint32_t regPwr = 0;
        switch(dBm) {
            case -4:
                regPwr = REG4_OUTPUT_POWER_M4_DBM;
                break;
            case -1:
                regPwr = REG4_OUTPUT_POWER_M1_DBM;
                break;
            case 2:
                regPwr = REG4_OUTPUT_POWER_2_DBM;
                break;
            case 5:
                regPwr = REG4_OUTPUT_POWER_5_DBM;
                break;
        }
        uint32_t reg4 = getADF4350Reg4(&adf4350Config);
        reg4&=~(3<<REG4_OUTPUT_POWER_BIT);
        reg4|=regPwr<<REG4_OUTPUT_POWER_BIT;
#ifdef ADF4350_DEBUG
  LOG(LL_ERROR, ("%s: ADF4350 REG 4", __FUNCTION__) );
#endif
        mgos_adf4350_reg_wr(reg4);
        rc=10;
    }
    return rc;
}

/**
 * @brief Power down the ADF4350 device
 * @param enabled If 0 do not power down, else power down.
 * @return void
 */
void mgos_power_down(bool power_down) {
    uint32_t reg2 = getADF4350Reg2(&adf4350Config);

    if( power_down ) {
        reg2|=1<<REG2_POWER_DOWN_BIT;
    }
    else {
        reg2&=~(1<<REG2_POWER_DOWN_BIT);
    }


#ifdef ADF4350_DEBUG
  LOG(LL_ERROR, ("%s: ADF4350 REG 2", __FUNCTION__) );
#endif
    mgos_adf4350_reg_wr(reg2);

}
