use crate::{
    gpio::TouchPin,
    peripherals::{LPWR, RTC_IO, SENS, TOUCH},
    private::{Internal, Sealed},
    rtc_cntl::Rtc,
};

/*
Start op low level functions to deal with the touch sensor of the ESP32S3, alle code is replicated from the IDF C code.
*/
pub const SOC_TOUCH_SENSOR_NUM: u8 = 14;

pub const TOUCH_LL_READ_RAW: u8 = 0x00;
pub const TOUCH_LL_READ_BENCHMARK: u8 = 0x02;
pub const TOUCH_LL_READ_SMOOTH: u8 = 0x03;

pub const TOUCH_LL_TIMER_FORCE_DONE: u8 = 0x3;
pub const TOUCH_LL_TIMER_DONE: u8 = 0x0;
pub const TOUCH_LL_PAD_MEASURE_WAIT_MAX: u8 = 0xFF;

pub const TOUCH_PAD_THRESHOLD_MAX: u32 = 0x1FFFFF;
pub const TOUCH_PAD_MEASURE_CYCLE_DEFAULT: u16 = 500;
pub const TOUCH_PAD_SLEEP_CYCLE_DEFAULT: u16 = 0xF;
pub const TOUCH_PAD_MAX: u8 = 14;
pub const TOUCH_PAD_BIT_MASK_ALL: u16 = ((1 << SOC_TOUCH_SENSOR_NUM) - 1);
pub const TOUCH_PAD_SLOPE_DEFAULT: u8 = TouchCntSlope::TOUCH_PAD_SLOPE_7 as u8;
pub const TOUCH_PAD_TIE_OPT_DEFAULT: TouchTieOption = TouchTieOption::TOUCH_PAD_TIE_OPT_LOW;

pub const TOUCH_PAD_HIGH_VOLTAGE_THRESHOLD: u8 = TouchHighVolt::TOUCH_HVOLT_2V7 as u8; //TOUCH_HVOLT_2V7)
pub const TOUCH_PAD_LOW_VOLTAGE_THRESHOLD: u8 = TouchLowVolt::TOUCH_LVOLT_0V5 as u8; //   (TOUCH_LVOLT_0V5)
pub const TOUCH_PAD_ATTEN_VOLTAGE_THRESHOLD: u8 = TouchHVoltAtten::TOUCH_HVOLT_ATTEN_0V5 as u8; // (TOUCH_HVOLT_ATTEN_0V5)
pub const TOUCH_PAD_IDLE_CH_CONNECT_DEFAULT: bool = TouchPadConnType::TOUCH_PAD_CONN_GND as u8 == 1; //  (TOUCH_PAD_CONN_GND)

#[repr(i8)]
pub enum TouchLowVolt {
    // Touch sensor low reference voltage, no change
    TOUCH_LVOLT_KEEP = -1,

    // Touch sensor low reference voltage, 0.5V
    TOUCH_LVOLT_0V5 = 0,

    // Touch sensor low reference voltage, 0.6V
    TOUCH_LVOLT_0V6,

    // Touch sensor low reference voltage, 0.7V
    TOUCH_LVOLT_0V7,

    // Touch sensor low reference voltage, 0.8V
    TOUCH_LVOLT_0V8,
}

#[repr(i8)]
pub enum TouchHighVolt {
    // Touch sensor high reference voltage, no change
    TOUCH_HVOLT_KEEP = -1,

    // Touch sensor high reference voltage, 2.4V
    TOUCH_HVOLT_2V4 = 0,

    // Touch sensor high reference voltage, 2.5V
    TOUCH_HVOLT_2V5,

    // Touch sensor high reference voltage, 2.6V
    TOUCH_HVOLT_2V6,

    // Touch sensor high reference voltage, 2.7V
    TOUCH_HVOLT_2V7,
}

// Touch sensor high reference voltage attenuation
#[repr(i8)]
pub enum TouchHVoltAtten {
    TOUCH_HVOLT_ATTEN_KEEP = -1, // no change
    TOUCH_HVOLT_ATTEN_1V5 = 0,   // 1.5V attenuation
    TOUCH_HVOLT_ATTEN_1V,        // 1.0V attenuation
    TOUCH_HVOLT_ATTEN_0V5,       // 0.5V attenuation
    TOUCH_HVOLT_ATTEN_0V,        //  0V attenuation
}

// Touch channel idle state configuration
#[repr(i8)]
pub enum TouchPadConnType {
    TOUCH_PAD_CONN_HIGHZ = 0, //touch channel is high resistance state
    TOUCH_PAD_CONN_GND = 1,   //touch channel is ground connection
}

// Touch sensor FSM mode
#[repr(i8)]
#[derive(PartialEq)]
pub enum TouchFSMMode {
    TOUCH_FSM_MODE_TIMER = 0, // To start touch FSM by timer
    TOUCH_FSM_MODE_SW = 1,    // To start touch FSM by software trigger
}

// Touch sensor charge/discharge speed
#[repr(i8)]
pub enum TouchCntSlope {
    TOUCH_PAD_SLOPE_0 = 0, // Touch sensor charge / discharge speed, always zero
    TOUCH_PAD_SLOPE_1 = 1, // Touch sensor charge / discharge speed, slowest
    TOUCH_PAD_SLOPE_2 = 2, // Touch sensor charge / discharge speed
    TOUCH_PAD_SLOPE_3 = 3, // Touch sensor charge / discharge speed
    TOUCH_PAD_SLOPE_4 = 4, // Touch sensor charge / discharge speed
    TOUCH_PAD_SLOPE_5 = 5, // Touch sensor charge / discharge speed
    TOUCH_PAD_SLOPE_6 = 6, // Touch sensor charge / discharge speed
    TOUCH_PAD_SLOPE_7 = 7, // Touch sensor charge / discharge speed, fast
}

// Touch sensor initial charge level
#[repr(i8)]
#[derive(PartialEq)]
pub enum TouchTieOption {
    TOUCH_PAD_TIE_OPT_LOW = 0,  // Initial level of charging voltage, low level
    TOUCH_PAD_TIE_OPT_HIGH = 1, // Initial level of charging voltage, high level
}

// Stop touch sensor FSM timer.
// The measurement action can be triggered by the hardware timer, as well as by the software instruction.
pub fn touch_ll_stop_fsm() {
    // taken from idf - esp32s3/hal/include/touch_sensor_ll.h
    /*
       RTCCNTL.touch_ctrl2.touch_start_en = 0; //stop touch fsm
       RTCCNTL.touch_ctrl2.touch_slp_timer_en = 0;
       RTCCNTL.touch_ctrl2.touch_timer_force_done = TOUCH_LL_TIMER_FORCE_DONE;
       RTCCNTL.touch_ctrl2.touch_timer_force_done = TOUCH_LL_TIMER_DONE;

       #define TOUCH_LL_READ_RAW           0x0
       #define TOUCH_LL_READ_BENCHMARK     0x2
       #define TOUCH_LL_READ_SMOOTH        0x3
       #define TOUCH_LL_TIMER_FORCE_DONE   0x3
       #define TOUCH_LL_TIMER_DONE         0x0
    */
    LPWR::regs().touch_ctrl2().write(|w| unsafe {
        w.touch_start_en()
            .clear_bit()
            .touch_slp_timer_en()
            .clear_bit()
            .touch_timer_force_done()
            .bits(TOUCH_LL_TIMER_FORCE_DONE)
            .touch_timer_force_done()
            .bits(TOUCH_LL_TIMER_DONE)
    });
}

// To disable touch pad interrupt.
pub fn touch_ll_intr_disable() {
    // taken from idf - esp32s3/hal/include/touch_sensor_ll.h
    /*

    // TODO: replace by ll macro
    typedef pub enum {
        TOUCH_PAD_INTR_MASK_DONE = BIT(0),      /*!<Measurement done for one of the enabled channels. */
        TOUCH_PAD_INTR_MASK_ACTIVE = BIT(1),    /*!<Active for one of the enabled channels. */
        TOUCH_PAD_INTR_MASK_INACTIVE = BIT(2),  /*!<Inactive for one of the enabled channels. */
        TOUCH_PAD_INTR_MASK_SCAN_DONE = BIT(3), /*!<Measurement done for all the enabled channels. */
        TOUCH_PAD_INTR_MASK_TIMEOUT = BIT(4),   /*!<Timeout for one of the enabled channels. */
    #if SOC_TOUCH_PROXIMITY_MEAS_DONE_SUPPORTED
        TOUCH_PAD_INTR_MASK_PROXI_MEAS_DONE = BIT(5),   /*!<For proximity sensor, when the number of measurements reaches the set count of measurements, an interrupt will be generated. */
        TOUCH_PAD_INTR_MASK_MAX
    #define TOUCH_PAD_INTR_MASK_ALL (TOUCH_PAD_INTR_MASK_TIMEOUT    \
                                    | TOUCH_PAD_INTR_MASK_SCAN_DONE \
                                    | TOUCH_PAD_INTR_MASK_INACTIVE  \
                                    | TOUCH_PAD_INTR_MASK_ACTIVE    \
                                    | TOUCH_PAD_INTR_MASK_DONE      \
                                    | TOUCH_PAD_INTR_MASK_PROXI_MEAS_DONE) /*!<All touch interrupt type enable. */
    #else
        TOUCH_PAD_INTR_MASK_MAX
    #define TOUCH_PAD_INTR_MASK_ALL (TOUCH_PAD_INTR_MASK_TIMEOUT    \
                                    | TOUCH_PAD_INTR_MASK_SCAN_DONE \
                                    | TOUCH_PAD_INTR_MASK_INACTIVE  \
                                    | TOUCH_PAD_INTR_MASK_ACTIVE    \
                                    | TOUCH_PAD_INTR_MASK_DONE) /*!<All touch interrupt type enable. */



          if (int_mask & TOUCH_PAD_INTR_MASK_DONE) {
            RTCCNTL.int_ena_w1tc.rtc_touch_done_w1tc = 1;
        }
        if (int_mask & TOUCH_PAD_INTR_MASK_ACTIVE) {
            RTCCNTL.int_ena_w1tc.rtc_touch_active_w1tc = 1;
        }
        if (int_mask & TOUCH_PAD_INTR_MASK_INACTIVE) {
            RTCCNTL.int_ena_w1tc.rtc_touch_inactive_w1tc = 1;
        }
        if (int_mask & TOUCH_PAD_INTR_MASK_SCAN_DONE) {
            RTCCNTL.int_ena_w1tc.rtc_touch_scan_done_w1tc = 1;
        }
        if (int_mask & TOUCH_PAD_INTR_MASK_TIMEOUT) {
            RTCCNTL.int_ena_w1tc.rtc_touch_timeout_w1tc = 1;
        }
        if (int_mask & TOUCH_PAD_INTR_MASK_PROXI_MEAS_DONE) {
            RTCCNTL.int_ena_w1tc.rtc_touch_approach_loop_done_w1tc = 1;
        }
    */

    LPWR::regs().int_ena_rtc_w1tc().write(|w| {
        w.touch_done()
            .clear_bit_by_one()
            .touch_active()
            .clear_bit_by_one()
            .touch_inactive()
            .clear_bit_by_one()
            .touch_scan_done()
            .clear_bit_by_one()
            .touch_timeout()
            .clear_bit_by_one()
            .touch_approach_loop_done()
            .clear_bit_by_one()
    });
}

// Clear touch sensor interrupt
pub fn touch_ll_intr_clear() {
    // taken from idf - esp32s3/hal/include/touch_sensor_ll.h
    /*
    if (int_mask & TOUCH_PAD_INTR_MASK_DONE) {
        RTCCNTL.int_clr.rtc_touch_done = 1;
    }
    if (int_mask & TOUCH_PAD_INTR_MASK_ACTIVE) {
        RTCCNTL.int_clr.rtc_touch_active = 1;
    }
    if (int_mask & TOUCH_PAD_INTR_MASK_INACTIVE) {
        RTCCNTL.int_clr.rtc_touch_inactive = 1;
    }
    if (int_mask & TOUCH_PAD_INTR_MASK_SCAN_DONE) {
        RTCCNTL.int_clr.rtc_touch_scan_done = 1;
    }
    if (int_mask & TOUCH_PAD_INTR_MASK_TIMEOUT) {
        RTCCNTL.int_clr.rtc_touch_timeout = 1;
    }
    if (int_mask & TOUCH_PAD_INTR_MASK_PROXI_MEAS_DONE) {
        RTCCNTL.int_clr.rtc_touch_approach_loop_done = 1;
    }
    */

    LPWR::regs().int_clr().write(|w| {
        w.touch_done()
            .clear_bit_by_one()
            .touch_active()
            .clear_bit_by_one()
            .touch_inactive()
            .clear_bit_by_one()
            .touch_scan_done()
            .clear_bit_by_one()
            .touch_timeout()
            .clear_bit_by_one()
            .touch_approach_loop_done()
            .clear_bit_by_one()
    });
}

// Disable touch sensor channel by bitmask.
pub fn touch_ll_clear_channel_mask() {
    /*
     SENS.sar_touch_conf.touch_outen &= ~(disable_mask & TOUCH_PAD_BIT_MASK_ALL);
    RTCCNTL.touch_scan_ctrl.touch_scan_pad_map  &= ~(disable_mask & TOUCH_PAD_BIT_MASK_ALL);
     */

    SENS::regs()
        .sar_touch_conf()
        .write(|w| unsafe { w.sar_touch_outen().bits(0x0) });

    LPWR::regs()
        .touch_scan_ctrl()
        .write(|w| unsafe { w.touch_scan_pad_map().bits(0x0) });
}

// Clear all touch sensor status.
pub fn touch_ll_clear_trigger_status_mask() {
    /*

    SENS.sar_touch_conf.touch_status_clr = 1;

     */

    SENS::regs()
        .sar_touch_conf()
        .write(|w| w.sar_touch_status_clr().set_bit());
}

// Set touch sensor touch sensor times of charge and discharge.
// @param meas_timers The times of charge and discharge in each measure process of touch channels.
//                     The timer frequency is 8Mhz. Range: 0 ~ 0xffff.
pub fn touch_ll_set_meas_times(meas_time: u16) {
    /*
    //The times of charge and discharge in each measure process of touch channels.
    HAL_FORCE_MODIFY_U32_REG_FIELD(RTCCNTL.touch_ctrl1, touch_meas_num, meas_time);
    //the waiting cycles (in 8MHz) between TOUCH_START and TOUCH_XPD
    HAL_FORCE_MODIFY_U32_REG_FIELD(RTCCNTL.touch_ctrl2, touch_xpd_wait, TOUCH_LL_PAD_MEASURE_WAIT_MAX); //wait volt stable
    */
    LPWR::regs()
        .touch_ctrl1()
        .write(|w| unsafe { w.touch_meas_num().bits(meas_time) });
    LPWR::regs()
        .touch_ctrl2()
        .write(|w| unsafe { w.touch_xpd_wait().bits(TOUCH_LL_PAD_MEASURE_WAIT_MAX) });
}

//  Set touch sensor sleep time.
// The touch sensor will sleep after each measurement.
//    sleep_cycle decide the interval between each measurement.
//                     t_sleep = sleep_cycle / (RTC_SLOW_CLK frequency).
pub fn touch_ll_set_sleep_time(sleep_time: u16) {
    /*
    // touch sensor sleep cycle Time = sleep_cycle / RTC_SLOW_CLK(150k)
    HAL_FORCE_MODIFY_U32_REG_FIELD(RTCCNTL.touch_ctrl1, touch_sleep_cycles, sleep_time);
    */
    LPWR::regs()
        .touch_ctrl1()
        .write(|w| unsafe { w.touch_sleep_cycles().bits(sleep_time) });
}

// Select touch sensor dbias to save power in sleep mode
// If change the dbias, the reading of touch sensor will changed. Users should make sure the threshold.
pub fn touch_ll_sleep_low_power(is_low_power: bool) {
    /*
        RTCCNTL.touch_ctrl2.touch_dbias = is_low_power;
    */
    LPWR::regs()
        .touch_ctrl2()
        .write(|w| w.touch_dbias().bit(is_low_power));
}

// Set touch sensor high voltage threshold of chanrge.
// The touch sensor measures the channel capacitance value by charging and discharging the channel.
// So the high threshold should be less than the supply voltage.
//
// refh The high voltage threshold of chanrge.
//
pub fn touch_ll_set_voltage_high(reph: u8) {
    /*
    RTCCNTL.touch_ctrl2.touch_drefh = refh;
     */

    LPWR::regs()
        .touch_ctrl2()
        .write(|w| unsafe { w.touch_drefh().bits(reph) });
}

// Set touch sensor low voltage threshold of discharge.
// The touch sensor measures the channel capacitance value by charging and discharging the channel.
//
// refl The low voltage threshold of discharge.
//
pub fn touch_ll_set_voltage_low(refl: u8) {
    /*
    RTCCNTL.touch_ctrl2.touch_drefl = refl;
     */

    LPWR::regs()
        .touch_ctrl2()
        .write(|w| unsafe { w.touch_drefl().bits(refl) });
}

// Set touch sensor high voltage attenuation of chanrge. The actual charge threshold is high voltage threshold minus attenuation value.
// The touch sensor measures the channel capacitance value by charging and discharging the channel.
// So the high threshold should be less than the supply voltage.
pub fn touch_ll_set_voltage_attenuation(atten: u8) {
    /*
           RTCCNTL.touch_ctrl2.touch_drange = atten;
    */

    LPWR::regs()
        .touch_ctrl2()
        .write(|w| unsafe { w.touch_drange().bits(atten) });
}

// Set connection type of touch channel in idle status.
//        When a channel is in measurement mode, other initialized channels are in idle mode.
//        The touch channel is generally adjacent to the trace, so the connection state of the idle channel
//        affects the stability and sensitivity of the test channel.
//        The `CONN_HIGHZ`(high resistance) setting increases the sensitivity of touch channels. (false)
//        The `CONN_GND`(grounding) setting increases the stability of touch channels. (true)
//
//      type  Select idle channel connect to high resistance state or ground.
pub fn touch_ll_set_idle_channel_connect(conn_type: bool) {
    /*

    RTCCNTL.touch_scan_ctrl.touch_inactive_connection = type;
     */
    LPWR::regs()
        .touch_scan_ctrl()
        .write(|w| w.touch_inactive_connection().bit(conn_type));
}
// Enable/disable clock gate of touch sensor.
//
//  enable true/false.
pub fn touch_ll_clkgate(enable: bool) {
    /*
    RTCCNTL.touch_ctrl2.touch_clkgate_en = enable; //enable touch clock for FSM. or force enable.
     */

    LPWR::regs()
        .touch_ctrl2()
        .write(|w| w.touch_clkgate_en().bit(enable));
}

// Force reset benchmark to raw data of touch sensor.
//
// If call this API, make sure enable clock gate(`touch_ll_clkgate`) first.
// touch_num touch pad index
//                  - TOUCH_PAD_MAX Reset basaline of all channels.
//
pub fn touch_ll_reset_benchmark(touch_num: u8) {
    /*
     if (touch_num == TOUCH_PAD_MAX) {
        SENS.sar_touch_chn_st.touch_channel_clr = TOUCH_PAD_BIT_MASK_ALL;
    } else {
        SENS.sar_touch_chn_st.touch_channel_clr = (1U << touch_num);
    }
     */

    // Clear touch channels to initialize the channel value (benchmark, raw_data).
    if touch_num == TOUCH_PAD_MAX {
        SENS::regs()
            .sar_touch_chn_st()
            .write(|w| unsafe { w.sar_touch_channel_clr().bits(TOUCH_PAD_BIT_MASK_ALL) });
    } else {
        SENS::regs()
            .sar_touch_chn_st()
            .write(|w| unsafe { w.sar_touch_channel_clr().bits(1 << touch_num) });
    }
}

pub fn touch_ll_sleep_reset_benchmark() {
    /*
        RTCCNTL.touch_approach.touch_slp_channel_clr = 1;
    */

    LPWR::regs()
        .touch_approach()
        .write(|w| w.touch_slp_channel_clr().set_bit());
}

// Set touch sensor FSM mode.
//        The measurement action can be triggered by the hardware timer, as well as by the software instruction.
//
//  mode FSM mode.
pub fn touch_ll_set_fsm_mode(mode: TouchFSMMode) {
    /*
    RTCCNTL.touch_ctrl2.touch_start_force = mode;
    */
    LPWR::regs()
        .touch_ctrl2()
        .write(|w| w.touch_start_force().bit(mode as i8 == 1));
}

// Get touch sensor FSM mode.
//        The measurement action can be triggered by the hardware timer, as well as by the software instruction.
//
// @param mode FSM mode.

pub fn touch_ll_get_fsm_mode() -> TouchFSMMode {
    /*
     *mode = (touch_fsm_mode_t)RTCCNTL.touch_ctrl2.touch_start_force;
     */
    if LPWR::regs()
        .touch_ctrl2()
        .read()
        .touch_start_force()
        .bit_is_set()
    {
        TouchFSMMode::TOUCH_FSM_MODE_SW
    } else {
        TouchFSMMode::TOUCH_FSM_MODE_TIMER
    }
}

/**
 * Get touch sensor raw data (touch sensor counter value) from register. No block.
 *
 * @param touch_num touch pad index.
 * @return touch_value pointer to accept touch sensor value.
 */
pub fn touch_ll_read_raw_data(touch_num: u8) -> u32 {
    /*
        SENS.sar_touch_conf.touch_data_sel = TOUCH_LL_READ_RAW;
    return SENS.sar_touch_status[touch_num - 1].touch_pad_data;
     */
    SENS::regs()
        .sar_touch_conf()
        .write(|w| unsafe { w.sar_touch_data_sel().bits(TOUCH_LL_READ_RAW) });
    SENS::regs()
        .sar_touch_status(touch_num as usize)
        .read()
        .data()
        .bits()
}

// Trigger a touch sensor measurement, only support in SW mode of FSM.
pub fn touch_ll_start_sw_meas() {
    /*
    RTCCNTL.touch_ctrl2.touch_start_en = 0;
    RTCCNTL.touch_ctrl2.touch_start_en = 1;
    */
    LPWR::regs()
        .touch_ctrl2()
        .write(|w| w.touch_start_en().clear_bit());
    LPWR::regs()
        .touch_ctrl2()
        .write(|w| w.touch_start_en().set_bit());
}

// Get touch sensor measure status. No block.
//
//  If touch sensors measure done.
pub fn touch_ll_is_measure_done() -> bool {
    /*
    return (bool)SENS.sar_touch_chn_st.touch_meas_done;
    */
    SENS::regs()
        .sar_touch_chn_st()
        .read()
        .sar_touch_meas_done()
        .bit()
}

// Start touch sensor FSM timer.
//        The measurement action can be triggered by the hardware timer, as well as by the software instruction.
pub fn touch_ll_start_fsm() {
    /*
    RTCCNTL.touch_ctrl2.touch_timer_force_done = TOUCH_LL_TIMER_FORCE_DONE;
    RTCCNTL.touch_ctrl2.touch_timer_force_done = TOUCH_LL_TIMER_DONE;
    RTCCNTL.touch_ctrl2.touch_slp_timer_en = (RTCCNTL.touch_ctrl2.touch_start_force == TOUCH_FSM_MODE_TIMER ? 1 : 0);
     */
    // Touch timer trigger measurement and always wait measurement done.
    // Force done for touch timer ensures that the timer always can get the measurement done signal.
    LPWR::regs()
        .touch_ctrl2()
        .write(|w| unsafe { w.touch_timer_force_done().bits(TOUCH_LL_TIMER_FORCE_DONE) });
    LPWR::regs()
        .touch_ctrl2()
        .write(|w| unsafe { w.touch_timer_force_done().bits(TOUCH_LL_TIMER_DONE) });
    if touch_ll_get_fsm_mode() == TouchFSMMode::TOUCH_FSM_MODE_SW {
        LPWR::regs()
            .touch_ctrl2()
            .write(|w| w.touch_slp_timer_en().set_bit());
    } else {
        LPWR::regs()
            .touch_ctrl2()
            .write(|w| w.touch_slp_timer_en().clear_bit());
    }
}

// Set the trigger threshold of touch sensor.
// The threshold determines the sensitivity of the touch sensor.
// The threshold is the original value of the trigger state minus the benchmark value.
//
// @note  If set "TOUCH_PAD_THRESHOLD_MAX", the touch is never be triggered.
// @param touch_num touch pad index
// @param threshold threshold of touch sensor.
pub fn touch_ll_set_threshold(touch_number: u8, threshold: u32) {
    /*
    SENS.touch_thresh[touch_num - 1].thresh = threshold;}
     */
    SENS::regs()
        .sar_touch_thres((touch_number - 1) as usize)
        .write(|w| unsafe { w.bits(threshold) });
}

//Set touch sensor charge/discharge speed(currents) for each pad.
//       If the slope is 0, the counter would always be zero.
//       If the slope is 1, the charging and discharging would be slow. The measurement time becomes longer.
//       If the slope is set 7, which is the maximum value, the charging and discharging would be fast.
//       The measurement time becomes shorter.
//
//@note The higher the charge and discharge current, the greater the immunity of the touch channel,
//      but it will increase the system power consumption.
//@param touch_num Touch pad index.
//@param slope touch pad charge/discharge speed(currents).
//
pub fn touch_ll_set_slope(touch_number: u8, slope: u8) {
    /*


    static inline void touch_ll_set_slope(touch_pad_t touch_num, touch_cnt_slope_t slope)
    {
    #define PAD_SLOP_MASK(val, num) ((val) << (29 - (num) * 3))
        uint32_t curr_slop = 0;
        if (touch_num < TOUCH_PAD_NUM10) {
            curr_slop = RTCCNTL.touch_dac.val;
            curr_slop &= ~PAD_SLOP_MASK(0x07, touch_num);  // clear the old value
            RTCCNTL.touch_dac.val = curr_slop | PAD_SLOP_MASK(slope, touch_num);
        } else {
            curr_slop = RTCCNTL.touch_dac1.val;
            curr_slop &= ~PAD_SLOP_MASK(0x07, touch_num - TOUCH_PAD_NUM10);  // clear the old value
            RTCCNTL.touch_dac1.val = curr_slop | PAD_SLOP_MASK(slope, touch_num - TOUCH_PAD_NUM10);
        }
    #undef PAD_SLOP_MASK
    }
         */

    if touch_number < 10 {
        LPWR::regs().touch_dac().write(|w| unsafe {
            match touch_number {
                0 => w.touch_pad0_dac().bits(slope),
                1 => w.touch_pad1_dac().bits(slope),
                2 => w.touch_pad2_dac().bits(slope),
                3 => w.touch_pad3_dac().bits(slope),
                4 => w.touch_pad4_dac().bits(slope),
                5 => w.touch_pad5_dac().bits(slope),
                6 => w.touch_pad6_dac().bits(slope),
                7 => w.touch_pad7_dac().bits(slope),
                8 => w.touch_pad8_dac().bits(slope),
                9 => w.touch_pad9_dac().bits(slope),
                _ => {
                    todo!()
                }
            }
        });
    } else {
        LPWR::regs().touch_dac1().write(|w| unsafe {
            match touch_number {
                10 => w.touch_pad10_dac().bits(slope),
                11 => w.touch_pad11_dac().bits(slope),
                12 => w.touch_pad12_dac().bits(slope),
                13 => w.touch_pad13_dac().bits(slope),
                14 => w.touch_pad14_dac().bits(slope),
                _ => {
                    todo!()
                }
            }
        });
    }
}

// Set initial voltage state of touch channel for each measurement.
//
// @param touch_num Touch pad index.
// @param opt Initial voltage state.
pub fn touch_ll_set_tie_option(touch_number: u8, tie_option: TouchTieOption) {
    /*
        RTCIO.touch_pad[touch_num].tie_opt = opt;
    */
    match tie_option {
        TouchTieOption::TOUCH_PAD_TIE_OPT_LOW => {
            RTC_IO::regs()
                .touch_pad(touch_number as usize)
                .write(|w| w.tie_opt().clear_bit());
        }
        TouchTieOption::TOUCH_PAD_TIE_OPT_HIGH => {
            RTC_IO::regs()
                .touch_pad(touch_number as usize)
                .write(|w| w.tie_opt().set_bit());
        }
    }
}

// Enable touch sensor channel. Register touch channel into touch sensor measurement group.
// The working mode of the touch sensor is simultaneous measurement.
// This function will set the measure bits according to the given bitmask.
//
// @note  If set this mask, the FSM timer should be stop firsty.
// @note  The touch sensor that in scan map, should be deinit GPIO function firstly.
// @param enable_mask bitmask of touch sensor scan group.
//        e.g. TOUCH_PAD_NUM1 -> BIT(1)
pub fn touch_ll_set_channel_mask(enable_mask: u16) {
    /*
           RTCCNTL.touch_scan_ctrl.touch_scan_pad_map  |= (enable_mask & TOUCH_PAD_BIT_MASK_ALL);
       SENS.sar_touch_conf.touch_outen |= (enable_mask & TOUCH_PAD_BIT_MASK_ALL);
    */

    let mut mask = LPWR::regs()
        .touch_scan_ctrl()
        .read()
        .touch_scan_pad_map()
        .bits();
    mask |= enable_mask & TOUCH_PAD_BIT_MASK_ALL;

    LPWR::regs()
        .touch_scan_ctrl()
        .write(|w| unsafe { w.touch_scan_pad_map().bits(mask) });

    SENS::regs()
        .sar_touch_conf()
        .write(|w| unsafe { w.sar_touch_outen().bits(mask) });
}
