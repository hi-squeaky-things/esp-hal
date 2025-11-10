// The HAL layer for touch sensor (common part), not for public API use

mod touch_lowlevel;
pub use touch_lowlevel::*;

/**
Initialize touch module.

If default parameter don't match the usage scenario, it can be changed after this function.
*/
pub fn touch_init_hal() {
    touch_ll_stop_fsm();
    touch_ll_intr_disable();
    touch_ll_intr_clear();
    touch_ll_clear_channel_mask();
    touch_ll_clear_trigger_status_mask();
    touch_ll_set_meas_times(TOUCH_PAD_MEASURE_CYCLE_DEFAULT);
    touch_ll_set_sleep_time(TOUCH_PAD_SLEEP_CYCLE_DEFAULT);

    // Configure the touch-sensor power domain into self-bias since bandgap-bias
    // level is different under sleep-mode compared to running-mode. self-bias is
    // always on after chip startup.
    touch_ll_sleep_low_power(true);
    touch_ll_set_voltage_high(TOUCH_PAD_HIGH_VOLTAGE_THRESHOLD);
    touch_ll_set_voltage_low(TOUCH_PAD_LOW_VOLTAGE_THRESHOLD);
    touch_ll_set_voltage_attenuation(TOUCH_PAD_ATTEN_VOLTAGE_THRESHOLD);
    touch_ll_set_idle_channel_connect(TOUCH_PAD_IDLE_CH_CONNECT_DEFAULT);

    // Clear touch channels to initialize the channel value (benchmark, raw_data).
    // Note: Should call it after enable clock gate.
    touch_ll_clkgate(true); // Enable clock gate for touch sensor.
    touch_ll_reset_benchmark(TOUCH_PAD_MAX);
    touch_ll_sleep_reset_benchmark();
}

pub fn touch_pad_set_fsm_mode(mode: TouchFSMMode) {
    touch_ll_set_fsm_mode(mode);
}

pub fn touch_pad_fsm_start() {
    touch_ll_start_fsm();
}


pub fn touch_pad_fsm_stop() {
    touch_ll_stop_fsm();
}
    
pub fn touch_pad_sw_start() {
    touch_ll_start_sw_meas();
}

pub fn touch_pad_meas_is_done() -> bool {
    touch_ll_is_measure_done()
}

pub fn touch_pad_config(touch_number: u8) {
    touch_pad_io_init(touch_number);
    touch_hal_config(touch_number);
    touch_hal_set_channel_mask(touch_number);
}

pub fn touch_hal_set_channel_mask(touch_number: u8) {
    let mask = 1 << touch_number;
    touch_ll_set_channel_mask(mask as u16);
}

pub fn touch_pad_read_raw_data(touch_number: u8) -> u32 {
    touch_ll_read_raw_data(touch_number)
}

pub fn touch_pad_io_init(touch_number: u8) {
    //this is done in GPIO, no implementation here, see gpio.rs --> set_touch
}

pub fn touch_hal_config(touch_number: u8) {
    touch_ll_set_threshold(touch_number, TOUCH_PAD_THRESHOLD_MAX);
    touch_ll_set_slope(touch_number, TOUCH_PAD_SLOPE_DEFAULT);
    touch_ll_set_tie_option(touch_number, TOUCH_PAD_TIE_OPT_DEFAULT);
}