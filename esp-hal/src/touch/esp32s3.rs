//! # Capacitive Touch Sensor
//!
//! ## Overview
//!
//! The touch sensor peripheral allows for cheap and robust user interfaces by
//! e.g., dedicating a part of the pcb as touch button.
//!
//! ## Examples
//!
//! ```rust, no_run
//! # {before_snippet}
//! # use esp_hal::touch::{Touch, TouchPad};
//! let touch_pin0 = peripherals.GPIO2;
//! let touch = Touch::continuous_mode(peripherals.TOUCH, None);
//! let mut touchpad = TouchPad::new(touch_pin0, &touch);
//! // ... give the peripheral some time for the measurement
//! let touch_val = touchpad.read();
//! # {after_snippet}
//! ```
//!
//! ## Implementation State:
//!
//! Mostly feature complete, missing:
//!
//! - Touch sensor slope control
//! - Deep Sleep support (wakeup from Deep Sleep)

use core::marker::PhantomData;


use crate::{
    Async, Blocking, DriverMode,
    gpio::TouchPin,
    peripherals::{LPWR, SENS, TOUCH},
    private::{Internal, Sealed},
    rtc_cntl::Rtc,
};

/// A marker trait describing the mode the touch pad is set to.
pub trait TouchMode: Sealed {}

/// Marker struct for the touch peripherals manual trigger mode. In the
/// technical reference manual, this is referred to as "start FSM via software".
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct OneShot;

/// Marker struct for the touch peripherals continuous reading mode. In the
/// technical reference manual, this is referred to as "start FSM via timer".
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Continuous;

impl TouchMode for OneShot {}
impl TouchMode for Continuous {}
impl Sealed for OneShot {}
impl Sealed for Continuous {}

/// Touchpad threshold type.
#[derive(Debug, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ThresholdMode {
    /// Pad is considered touched if value is greater than threshold.
    GreaterThan,
    /// Pad is considered touched if value is less than threshold.
    LessThan,
}

/// Configurations for the touch pad driver
#[derive(Debug, Copy, Clone, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TouchConfig {
    /// The [`ThresholdMode`] for the pads. Defaults to
    /// `ThresholdMode::LessThan`
    pub threshold_mode: Option<ThresholdMode>,
    /// Duration of a single measurement (in cycles of the 8 MHz touch clock).
    /// Defaults to `0x7fff`
    pub measurement_duration: Option<u16>,
    /// Sleep cycles for the touch timer in [`Continuous`]-mode. Defaults to
    /// `0x100`
    pub sleep_cycles: Option<u16>,
}

/// This struct marks a successfully initialized touch peripheral
pub struct Touch<'d, Tm: TouchMode, Dm: DriverMode> {
    _inner: TOUCH<'d>,
    _touch_mode: PhantomData<Tm>,
    _mode: PhantomData<Dm>,
}
impl<Tm: TouchMode, Dm: DriverMode> Touch<'_, Tm, Dm> {
    /// Common initialization of the touch peripheral.
    fn initialize_common(config: Option<TouchConfig>) {
        let mut threshold_mode = false;
        let mut meas_dur = 0x7fff;

        if let Some(config) = config {
            threshold_mode = match config.threshold_mode {
                Some(ThresholdMode::LessThan) => false,
                Some(ThresholdMode::GreaterThan) => true,
                None => false,
            };
            if let Some(dur) = config.measurement_duration {
                meas_dur = dur;
            }
        }

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

    /// Common parts of the continuous mode initialization.
    fn initialize_common_continuous(config: Option<TouchConfig>) {
        Self::initialize_common(config);
    }
}
// Async mode and OneShot does not seem to be a sensible combination....
impl<'d> Touch<'d, OneShot, Blocking> {
    #[procmacros::doc_replace]
    /// Initializes the touch peripheral and returns this marker struct.
    /// Optionally accepts configuration options.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// # use esp_hal::touch::{Touch, TouchConfig};
    /// let touch_cfg = Some(TouchConfig {
    ///     measurement_duration: Some(0x2000),
    ///     ..Default::default()
    /// });
    /// let touch = Touch::one_shot_mode(peripherals.TOUCH, touch_cfg);
    /// # {after_snippet}
    /// ```
    pub fn one_shot_mode(touch_peripheral: TOUCH<'d>, config: Option<TouchConfig>) -> Self {
        Self::initialize_common(config);
        touch_pad_set_fsm_mode(TouchFSMMode::TOUCH_FSM_MODE_SW);
        Self {
            _inner: touch_peripheral,
            _mode: PhantomData,
            _touch_mode: PhantomData,
        }
    }
}
impl<'d> Touch<'d, Continuous, Blocking> {
    #[procmacros::doc_replace]
    /// Initializes the touch peripheral in continuous mode and returns this
    /// marker struct. Optionally accepts configuration options.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// # use esp_hal::touch::{Touch, TouchConfig};
    /// let touch_cfg = Some(TouchConfig {
    ///     measurement_duration: Some(0x3000),
    ///     ..Default::default()
    /// });
    /// let touch = Touch::continuous_mode(peripherals.TOUCH, touch_cfg);
    /// # {after_snippet}
    /// ```
    pub fn continuous_mode(touch_peripheral: TOUCH<'d>, config: Option<TouchConfig>) -> Self {
        Self::initialize_common_continuous(config);

        Self {
            _inner: touch_peripheral,
            _mode: PhantomData,
            _touch_mode: PhantomData,
        }
    }
}
impl<'d> Touch<'d, Continuous, Async> {
    #[procmacros::doc_replace]
    /// Initializes the touch peripheral in continuous async mode and returns
    /// this marker struct.
    ///
    /// ## Warning:
    ///
    /// This uses [`RTC_CORE`](crate::peripherals::Interrupt::RTC_CORE)
    /// interrupts under the hood. So the whole async part breaks if you install
    /// an interrupt handler with [`Rtc::set_interrupt_handler()`][1].
    ///
    /// [1]: ../rtc_cntl/struct.Rtc.html#method.set_interrupt_handler
    ///
    /// ## Parameters:
    ///
    /// - `rtc`: The RTC peripheral is needed to configure the required interrupts.
    /// - `config`: Optional configuration options.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// # use esp_hal::rtc_cntl::Rtc;
    /// # use esp_hal::touch::{Touch, TouchConfig};
    /// let mut rtc = Rtc::new(peripherals.LPWR);
    /// let touch = Touch::async_mode(peripherals.TOUCH, &mut rtc, None);
    /// # {after_snippet}
    /// ```
    pub fn async_mode(
        touch_peripheral: TOUCH<'d>,
        rtc: &mut Rtc<'_>,
        config: Option<TouchConfig>,
    ) -> Self {
        Self::initialize_common_continuous(config);

        rtc.set_interrupt_handler(asynch::handle_touch_interrupt);

        Self {
            _inner: touch_peripheral,
            _mode: PhantomData,
            _touch_mode: PhantomData,
        }
    }
}

/// A pin that is configured as a TouchPad.
pub struct TouchPad<P: TouchPin, Tm: TouchMode, Dm: DriverMode> {
    pin: P,
    _touch_mode: PhantomData<Tm>,
    _mode: PhantomData<Dm>,
}
impl<P: TouchPin> TouchPad<P, OneShot, Blocking> {
    /// (Re-)Start a touch measurement on the pin. You can get the result by
    /// calling [`read`](Self::read) once it is finished.
    pub fn start_measurement(&mut self) {
        //TODO:
        touch_pad_config(self.pin.number());
        //touch_pad_set_cnt_mode();
        touch_pad_fsm_start();
        touch_pad_sw_start();
        while !touch_pad_meas_is_done() {}
    }
}
impl<P: TouchPin, Tm: TouchMode, Dm: DriverMode> TouchPad<P, Tm, Dm> {
    /// Construct a new instance of [`TouchPad`].
    ///
    /// ## Parameters:
    /// - `pin`: The pin that gets configured as touch pad
    /// - `touch`: The [`Touch`] struct indicating that touch is configured.
    pub fn new(pin: P, _touch: &Touch<'_, Tm, Dm>) -> Self {
        // TODO revert this on drop
        pin.set_touch(Internal);

        Self {
            pin,
            _mode: PhantomData,
            _touch_mode: PhantomData,
        }
    }

    /// Read the current touch pad capacitance counter.
    ///
    /// Usually a lower value means higher capacitance, thus indicating touch
    /// event.
    ///
    /// Returns `None` if the value is not yet ready. (Note: Measurement must be
    /// started manually with [`start_measurement`](Self::start_measurement) if
    /// the touch peripheral is in [`OneShot`] mode).
    pub fn try_read(&mut self) -> Option<u32> {
        if unsafe { &*crate::peripherals::SENS::ptr() }
            .sar_touch_chn_st()
            .read()
            .sar_touch_meas_done()
            .bit_is_set()
        {
            Some(self.pin.touch_measurement(Internal))
        } else {
            None
        }
    }
}
impl<P: TouchPin, Tm: TouchMode> TouchPad<P, Tm, Blocking> {
    /// Blocking read of the current touch pad capacitance counter.
    ///
    /// Usually a lower value means higher capacitance, thus indicating touch
    /// event.
    ///
    /// ## Note for [`OneShot`] mode:
    ///
    /// This function might block forever, if
    /// [`start_measurement`](Self::start_measurement) was not called before. As
    /// measurements are not cleared, the touch values might also be
    /// outdated, if it has been some time since the last call to that
    /// function.
    pub fn read(&mut self) -> u32 {
        self.pin.touch_measurement(Internal)
    }

    /// check if ready
    pub fn ready(&mut self) -> bool {
        unsafe { &*crate::peripherals::SENS::ptr() }
            .sar_touch_chn_st()
            .read()
            .sar_touch_meas_done()
            .bit_is_set()
    }

    /// check active
    pub fn active(&mut self) -> u16 {
        unsafe { &*crate::peripherals::SENS::ptr() }
            .sar_touch_chn_st()
            .read()
            .sar_touch_pad_active()
            .bits()
    }
    // RTC_CNTL_TOUCH_MEAS_NUM,

    /// ds
    pub fn outen(&mut self) -> u16 {
        unsafe { &*crate::peripherals::SENS::ptr() }
            .sar_touch_conf()
            .read()
            .sar_touch_outen()
            .bits()
    }

    /// Listens for the touch_pad interrupt.
    ///
    /// The raised interrupt is actually
    /// [`RTC_CORE`](crate::peripherals::Interrupt::RTC_CORE). A handler can
    /// be installed with [`Rtc::set_interrupt_handler()`][1].
    ///
    /// [1]: ../rtc_cntl/struct.Rtc.html#method.set_interrupt_handler
    ///
    /// ## Parameters:
    /// - `threshold`: The threshold above/below which the pin is considered touched. Above/below
    ///   depends on the configuration of `touch` in [`new`](Self::new) (defaults to below).
    ///
    /// ## Example
    pub fn listen(&mut self, threshold: u16) {
        self.pin.set_threshold(threshold, Internal);
        listen(self.pin.touch_nr(Internal))
    }

    /// Unlisten for the touch pad's interrupt.
    ///
    /// If no other touch pad interrupts are active, the touch interrupt is
    /// disabled completely.
    pub fn unlisten(&mut self) {
        unlisten(self.pin.touch_nr(Internal))
    }

    /// Clears a pending touch interrupt.
    ///
    /// ## Note on interrupt clearing behaviour:
    ///
    /// There is only a single interrupt for the touch pad.
    /// [`is_interrupt_set`](Self::is_interrupt_set) can be used to check
    /// which pins are touchted. However, this function clears the interrupt
    /// status for all pins. So only call it when all pins are handled.
    pub fn clear_interrupt(&mut self) {
        internal_clear_interrupt()
    }

    /// Checks if the pad is touched, based on the configured threshold value.
    pub fn is_interrupt_set(&mut self) -> bool {
        internal_is_interrupt_set(self.pin.touch_nr(Internal))
    }
}

fn listen(touch_nr: u8) {
    // enable touch interrupts
    // LPWR::regs().int_ena().write(|w| w.touch().set_bit());
    //
    // SENS::regs().sar_touch_enable().modify(|r, w| unsafe {
    // w.touch_pad_outen1()
    // .bits(r.touch_pad_outen1().bits() | (1 << touch_nr))
    // });
}

fn unlisten(touch_nr: u8) {
    // SENS::regs().sar_touch_enable().modify(|r, w| unsafe {
    // w.touch_pad_outen1()
    // .bits(r.touch_pad_outen1().bits() & !(1 << touch_nr))
    // });
    // if SENS::regs()
    // .sar_touch_enable()
    // .read()
    // .touch_pad_outen1()
    // .bits()
    // == 0
    // {
    //
    // LPWR::regs().int_ena().write(|w| w.touch().clear_bit());
    // }
}

fn internal_unlisten() {
    // SENS::regs()
    // .sar_touch_enable()
    // .write(|w| unsafe { w.touch_pad_outen1().bits(0) });
    // if SENS::regs()
    // .sar_touch_enable()
    // .read()
    // .touch_pad_outen1()
    // .bits()
    // == 0
    // {
    // LPWR::regs().int_ena().write(|w| w.touch().clear_bit());
    // }
}

fn internal_clear_interrupt() {
    // LPWR::regs()
    // .int_clr()
    // .write(|w| w.touch().clear_bit_by_one());
    // SENS::regs()
    // .sar_touch_ctrl2()
    // .write(|w| w.touch_meas_en_clr().set_bit());
}

fn internal_pins_touched() -> u16 {
    // Only god knows, why the "interrupt flag" register is called "meas_en" on this
    // chip...
    // SENS::regs().sar_touch_ctrl2().read().touch_meas_en().bits()
    0
}

fn internal_is_interrupt_set(touch_nr: u8) -> bool {
    internal_pins_touched() & (1 << touch_nr) != 0
}

/*
Start op low level functions to deal with the touch sensor of the ESP32S3, alle code is replicated from the IDF C code.
*/
const TOUCH_LL_READ_RAW: u8 = 0x00;
const TOUCH_LL_READ_BENCHMARK: u8 = 0x02;
const TOUCH_LL_READ_SMOOTH: u8 = 0x03;

const TOUCH_LL_TIMER_FORCE_DONE: u8 = 0x3;
const TOUCH_LL_TIMER_DONE: u8 = 0x0;
const TOUCH_PAD_MEASURE_CYCLE_DEFAULT: u16 = 500;
const TOUCH_PAD_SLEEP_CYCLE_DEFAULT: u16 = 0xF;
const TOUCH_LL_PAD_MEASURE_WAIT_MAX: u8 = 0xFF;

const TOUCH_PAD_THRESHOLD_MAX: u32 = 0x1FFFFF;

#[repr(i8)]
enum TouchLowVolt {
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
enum TouchHighVolt {
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
enum TouchHVoltAtten {
    TOUCH_HVOLT_ATTEN_KEEP = -1, // no change
    TOUCH_HVOLT_ATTEN_1V5 = 0,   // 1.5V attenuation
    TOUCH_HVOLT_ATTEN_1V,        // 1.0V attenuation
    TOUCH_HVOLT_ATTEN_0V5,       // 0.5V attenuation
    TOUCH_HVOLT_ATTEN_0V,        //  0V attenuation
}

// Touch channel idle state configuration
#[repr(i8)]
enum TouchPadConnType {
    TOUCH_PAD_CONN_HIGHZ = 0, //touch channel is high resistance state
    TOUCH_PAD_CONN_GND = 1,   //touch channel is ground connection
}

// Touch sensor FSM mode
#[repr(i8)]
#[derive(PartialEq)]
enum TouchFSMMode {
    TOUCH_FSM_MODE_TIMER = 0, // To start touch FSM by timer
    TOUCH_FSM_MODE_SW = 1,    // To start touch FSM by software trigger
}

/** Touch sensor charge/discharge speed */
#[repr(i8)]
enum TouchCntSlope{
    TOUCH_PAD_SLOPE_0 = 0,       // Touch sensor charge / discharge speed, always zero 
    TOUCH_PAD_SLOPE_1 = 1,       // Touch sensor charge / discharge speed, slowest 
    TOUCH_PAD_SLOPE_2 = 2,       // Touch sensor charge / discharge speed
    TOUCH_PAD_SLOPE_3 = 3,       // Touch sensor charge / discharge speed 
    TOUCH_PAD_SLOPE_4 = 4,       // Touch sensor charge / discharge speed 
    TOUCH_PAD_SLOPE_5 = 5,       // Touch sensor charge / discharge speed 
    TOUCH_PAD_SLOPE_6 = 6,       // Touch sensor charge / discharge speed 
    TOUCH_PAD_SLOPE_7 = 7,       // Touch sensor charge / discharge speed, fast 
}


const TOUCH_PAD_HIGH_VOLTAGE_THRESHOLD: u8 = TouchHighVolt::TOUCH_HVOLT_2V7 as u8; //TOUCH_HVOLT_2V7)
const TOUCH_PAD_LOW_VOLTAGE_THRESHOLD: u8 = TouchLowVolt::TOUCH_LVOLT_0V5 as u8; //   (TOUCH_LVOLT_0V5)
const TOUCH_PAD_ATTEN_VOLTAGE_THRESHOLD: u8 = TouchHVoltAtten::TOUCH_HVOLT_ATTEN_0V5 as u8; // (TOUCH_HVOLT_ATTEN_0V5)
const TOUCH_PAD_IDLE_CH_CONNECT_DEFAULT: bool = TouchPadConnType::TOUCH_PAD_CONN_GND as u8 == 1; //  (TOUCH_PAD_CONN_GND)
const SOC_TOUCH_SENSOR_NUM: u8 = 14;
const TOUCH_PAD_MAX: u8 = 14;
const TOUCH_PAD_BIT_MASK_ALL: u16 = ((1 << SOC_TOUCH_SENSOR_NUM) - 1);
const TOUCH_PAD_SLOPE_DEFAULT: u8 = TouchCntSlope::TOUCH_PAD_SLOPE_7 as u8;



// Stop touch sensor FSM timer.
// The measurement action can be triggered by the hardware timer, as well as by the software instruction.
fn touch_ll_stop_fsm() {
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
fn touch_ll_intr_disable() {
    // taken from idf - esp32s3/hal/include/touch_sensor_ll.h
    /*

    // TODO: replace by ll macro
    typedef enum {
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

    LPWR::regs().int_ena_rtc_w1tc().write(|w| unsafe {
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
fn touch_ll_intr_clear() {
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

    LPWR::regs().int_clr().write(|w| unsafe {
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
fn touch_ll_clear_channel_mask() {
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
fn touch_ll_clear_trigger_status_mask() {
    /*

    SENS.sar_touch_conf.touch_status_clr = 1;

     */

    SENS::regs()
        .sar_touch_conf()
        .write(|w| unsafe { w.sar_touch_status_clr().set_bit() });
}

// Set touch sensor touch sensor times of charge and discharge.
// @param meas_timers The times of charge and discharge in each measure process of touch channels.
//                     The timer frequency is 8Mhz. Range: 0 ~ 0xffff.
fn touch_ll_set_meas_times(meas_time: u16) {
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
fn touch_ll_set_sleep_time(sleep_time: u16) {
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
fn touch_ll_sleep_low_power(is_low_power: bool) {
    /*
        RTCCNTL.touch_ctrl2.touch_dbias = is_low_power;
    */
    LPWR::regs()
        .touch_ctrl2()
        .write(|w| unsafe { w.touch_dbias().bit(is_low_power) });
}

// Set touch sensor high voltage threshold of chanrge.
// The touch sensor measures the channel capacitance value by charging and discharging the channel.
// So the high threshold should be less than the supply voltage.
//
// refh The high voltage threshold of chanrge.
//
fn touch_ll_set_voltage_high(reph: u8) {
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
fn touch_ll_set_voltage_low(refl: u8) {
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
fn touch_ll_set_voltage_attenuation(atten: u8) {
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
fn touch_ll_set_idle_channel_connect(conn_type: bool) {
    /*

    RTCCNTL.touch_scan_ctrl.touch_inactive_connection = type;
     */
    LPWR::regs()
        .touch_scan_ctrl()
        .write(|w| unsafe { w.touch_inactive_connection().bit(conn_type) });
}
// Enable/disable clock gate of touch sensor.
//
//  enable true/false.
fn touch_ll_clkgate(enable: bool) {
    /*
    RTCCNTL.touch_ctrl2.touch_clkgate_en = enable; //enable touch clock for FSM. or force enable.
     */

    LPWR::regs()
        .touch_ctrl2()
        .write(|w| unsafe { w.touch_clkgate_en().bit(enable) });
}

// Force reset benchmark to raw data of touch sensor.
//
// If call this API, make sure enable clock gate(`touch_ll_clkgate`) first.
// touch_num touch pad index
//                  - TOUCH_PAD_MAX Reset basaline of all channels.
//
fn touch_ll_reset_benchmark(touch_num: u8) {
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

fn touch_ll_sleep_reset_benchmark() {
    /*
        RTCCNTL.touch_approach.touch_slp_channel_clr = 1;
    */

    LPWR::regs()
        .touch_approach()
        .write(|w| unsafe { w.touch_slp_channel_clr().set_bit() });
}

// Set touch sensor FSM mode.
//        The measurement action can be triggered by the hardware timer, as well as by the software instruction.
//
//  mode FSM mode.
fn touch_ll_set_fsm_mode(mode: TouchFSMMode) {
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

fn touch_ll_get_fsm_mode() -> TouchFSMMode {
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
fn touch_ll_read_raw_data(touch_num: u8) -> u32 {
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
fn touch_ll_start_sw_meas() {
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
fn touch_ll_is_measure_done() -> bool {
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
fn touch_ll_start_fsm() {
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
            .write(|w| unsafe { w.touch_slp_timer_en().set_bit() });
    } else {
        LPWR::regs()
            .touch_ctrl2()
            .write(|w| unsafe { w.touch_slp_timer_en().clear_bit() });
    }
}

fn touch_pad_set_fsm_mode(mode: TouchFSMMode) {
    touch_ll_set_fsm_mode(mode);
}

fn touch_pad_fsm_start() {
    touch_ll_start_fsm();
}

fn touch_pad_sw_start() {
    touch_ll_start_sw_meas();
}

fn touch_pad_meas_is_done() -> bool {
    touch_ll_is_measure_done()
}

fn touch_pad_config(touch_number: u8) {
    touch_pad_io_init(touch_number);
    touch_hal_config(touch_number);
    // TODO
    // touch_hal_set_channel_mask(touch_number);
}

fn touch_pad_io_init(touch_number: u8) {
    //done in GPIO libray?

    /*
        esp_err_t touch_pad_io_init(touch_pad_t touch_num)
    {
        TOUCH_CHANNEL_CHECK(touch_num);
        gpio_num_t gpio_num = TOUCH_GET_IO_NUM(touch_num);
        rtc_gpio_init(gpio_num);
        rtc_gpio_set_direction(gpio_num, RTC_GPIO_MODE_DISABLED);
        rtc_gpio_pulldown_dis(gpio_num);
        rtc_gpio_pullup_dis(gpio_num);
        return ESP_OK;
    }

         */
}

fn touch_hal_config(touch_number: u8) {
    touch_ll_set_threshold(touch_number, TOUCH_PAD_THRESHOLD_MAX);
    touch_ll_set_slope(touch_number, TOUCH_PAD_SLOPE_DEFAULT);
    // touch_ll_set_tie_option(touch_number, TOUCH_PAD_TIE_OPT_DEFAULT);
}

// Set the trigger threshold of touch sensor.
// The threshold determines the sensitivity of the touch sensor.
// The threshold is the original value of the trigger state minus the benchmark value.
//
// @note  If set "TOUCH_PAD_THRESHOLD_MAX", the touch is never be triggered.
// @param touch_num touch pad index
// @param threshold threshold of touch sensor.
fn touch_ll_set_threshold(touch_number: u8, threshold: u32) {
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
fn touch_ll_set_slope(touch_number: u8, slope: u8) {
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
                _ => { todo!() },
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
                _ => { todo!() },
            }
        });
    }
}

mod asynch {
    use core::{
        sync::atomic::{AtomicU16, Ordering},
        task::{Context, Poll},
    };

    use super::*;
    use crate::{Async, asynch::AtomicWaker, handler, ram};

    const NUM_TOUCH_PINS: usize = 10;

    static TOUCH_WAKERS: [AtomicWaker; NUM_TOUCH_PINS] =
        [const { AtomicWaker::new() }; NUM_TOUCH_PINS];

    // Helper variable to store which pins need handling.
    static TOUCHED_PINS: AtomicU16 = AtomicU16::new(0);

    #[must_use = "futures do nothing unless you `.await` or poll them"]
    pub struct TouchFuture {
        touch_nr: u8,
    }

    impl TouchFuture {
        pub fn new(touch_nr: u8) -> Self {
            Self { touch_nr }
        }
    }

    impl core::future::Future for TouchFuture {
        type Output = ();

        fn poll(self: core::pin::Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
            TOUCH_WAKERS[self.touch_nr as usize].register(cx.waker());

            let pins = TOUCHED_PINS.load(Ordering::Acquire);

            if pins & (1 << self.touch_nr) != 0 {
                // clear the pin to signal that this pin was handled.
                TOUCHED_PINS.fetch_and(!(1 << self.touch_nr), Ordering::Release);
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        }
    }

    #[handler]
    #[ram]
    pub(super) fn handle_touch_interrupt() {
        let touch_pads = internal_pins_touched();
        for (i, waker) in TOUCH_WAKERS.iter().enumerate() {
            if touch_pads & (1 << i) != 0 {
                waker.wake();
            }
        }
        TOUCHED_PINS.store(touch_pads, Ordering::Relaxed);
        internal_clear_interrupt();
        internal_unlisten();
    }

    impl<P: TouchPin, Tm: TouchMode> TouchPad<P, Tm, Async> {
        /// Wait for the pad to be touched.
        pub async fn wait_for_touch(&mut self, threshold: u16) {
            self.pin.set_threshold(threshold, Internal);
            let touch_nr = self.pin.touch_nr(Internal);
            listen(touch_nr);
            TouchFuture::new(touch_nr).await;
        }
    }
}
