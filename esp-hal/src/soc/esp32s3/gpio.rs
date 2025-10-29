//! # GPIO configuration module (ESP32-S3)
//!
//! ## Overview
//!
//! The `GPIO` module provides functions and configurations for controlling the
//! `General Purpose Input/Output` pins on the `ESP32-S3` chip. It allows you to
//! configure pins as inputs or outputs, set their state and read their state.
//!
//! Let's get through the functionality and configurations provided by this GPIO
//! module:
//!   - `gpio` block:
//!       * Defines the pin configurations for various GPIO pins. Each line represents a pin and its
//!         associated options such as input/output mode, analog capability, and corresponding
//!         functions.
//!   - `analog` block:
//!       * Block defines the analog capabilities of various GPIO pins. Each line represents a pin
//!         and its associated options such as mux selection, function selection, and input enable.
//!   - `enum InputSignal`:
//!       * This enumeration defines input signals for the GPIO mux. Each input signal is assigned a
//!         specific value.
//!   - `enum OutputSignal`:
//!       * This enumeration defines output signals for the GPIO mux. Each output signal is assigned
//!         a specific value.
//!
//! This trait provides functions to read the interrupt status and NMI status
//! registers for both the `PRO CPU` and `APP CPU`. The implementation uses the
//! `gpio` peripheral to access the appropriate registers.

macro_rules! rtcio_analog {
    ($pin_num:expr, $pin_reg:expr, $hold:ident $(, $analog:literal)?) => {
        paste::paste! {
            #[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
            impl $crate::gpio::RtcPin for $crate::peripherals::[<GPIO $pin_num>]<'_> {
                fn rtc_number(&self) -> u8 {
                    $pin_num
                }

                /// Set the RTC properties of the pin. If `mux` is true then then pin is
                /// routed to RTC, when false it is routed to IO_MUX.
                fn rtc_set_config(&self, input_enable: bool, mux: bool, func: $crate::gpio::RtcFunction) {
                    enable_iomux_clk_gate();

                    // We need `paste` to rewrite something in each function, so that rustc
                    // doesn't trip over trying to substitute a partial expression as `$pin_reg`
                    $crate::peripherals::[<RTC _IO>]::regs()
                        .$pin_reg.modify(|_,w| unsafe {
                            w.fun_ie().bit(input_enable);
                            w.mux_sel().bit(mux);
                            w.fun_sel().bits(func as u8)
                        });
                }

                fn rtcio_pad_hold(&self, enable: bool) {
                    $crate::peripherals::LPWR::regs()
                        .pad_hold()
                        .modify(|_, w| w.$hold().bit(enable));
                }
            }

            #[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
            impl $crate::gpio::RtcPinWithResistors for $crate::peripherals::[<GPIO $pin_num>]<'_> {
                fn rtcio_pullup(&self, enable: bool) {
                    $crate::peripherals::[<RTC _IO>]::regs()
                        .$pin_reg.modify(|_, w| w.rue().bit(enable));
                }

                fn rtcio_pulldown(&self, enable: bool) {
                    $crate::peripherals::[<RTC _IO>]::regs()
                        .$pin_reg.modify(|_, w| w.rde().bit(enable));
                }
            }

            $(
                $crate::ignore!($analog);
                impl $crate::peripherals::[<GPIO $pin_num>]<'_> {
                    /// Configures the pin for analog mode.
                    #[cfg(feature = "unstable")]
                    pub(crate) fn set_analog_impl(&self) {
                        use $crate::gpio::RtcPin;
                        enable_iomux_clk_gate();

                        let rtcio = $crate::peripherals::[<RTC _IO>]::regs();

                        // disable output
                        rtcio.enable_w1tc().write(|w| unsafe { w.enable_w1tc().bits(1 << self.rtc_number()) });

                        // disable open drain
                        rtcio.pin(self.rtc_number() as usize).modify(|_,w| w.pad_driver().bit(false));

                        rtcio.$pin_reg.modify(|_,w| {
                            w.fun_ie().clear_bit();

                            // Connect pin to analog / RTC module instead of standard GPIO
                            w.mux_sel().set_bit();

                            // Select function "RTC function 1" (GPIO) for analog use
                            unsafe { w.fun_sel().bits(0b00) };

                            // Disable pull-up and pull-down resistors on the pin
                            w.rue().bit(false);
                            w.rde().bit(false);

                            w
                        });
                    }
                }
            )?
        }
    };

    (
        $( ( $pin_num:expr, $pin_reg:expr, $hold:ident $(, $analog:literal)? ) )+
    ) => {
        $(
            rtcio_analog!($pin_num, $pin_reg, $hold $(, $analog)?);
        )+
    };
}

rtcio_analog! {
    ( 0, touch_pad(0),   touch_pad0 )
    ( 1, touch_pad(1),   touch_pad1 , true)
    ( 2, touch_pad(2),   touch_pad2 , true)
    ( 3, touch_pad(3),   touch_pad3 , true)
    ( 4, touch_pad(4),   touch_pad4 , true)
    ( 5, touch_pad(5),   touch_pad5 , true)
    ( 6, touch_pad(6),   touch_pad6 , true)
    ( 7, touch_pad(7),   touch_pad7 , true)
    ( 8, touch_pad(8),   touch_pad8 , true)
    ( 9, touch_pad(9),   touch_pad9 , true)
    ( 10, touch_pad(10),  touch_pad10, true)
    ( 11, touch_pad(11),  touch_pad11, true)
    ( 12, touch_pad(12),  touch_pad12, true)
    ( 13, touch_pad(13),  touch_pad13, true)
    ( 14, touch_pad(14),  touch_pad14, true)
    ( 15, xtal_32p_pad(), x32p       , true)
    ( 16, xtal_32n_pad(), x32n       , true)
    ( 17, pad_dac1(),     pdac1      , true)
    ( 18, pad_dac2(),     pdac2      , true)
    ( 19, rtc_pad19(),    pad19      , true)
    ( 20, rtc_pad20(),    pad20      , true)
    ( 21, rtc_pad21(),    pad21      )
}


macro_rules! touch {
    (
        $(
            ($touch_num:tt, $pin_peri:ident $(, $pad_register_has_all_fields:literal)?)
        )+
    ) => {
        $(
        impl $crate::gpio::TouchPin for $crate::peripherals::$pin_peri<'_> {
            fn set_touch(&self, _: $crate::private::Internal) {
                use $crate::peripherals::{GPIO, RTC_IO, SENS};
                use $crate::gpio::RtcPin;

                let gpio = GPIO::regs();
                let rtcio = RTC_IO::regs();
                let sens = SENS::regs();

                // Pad to normal mode (not open-drain)
                gpio.pin(self.rtc_number() as usize).write(|w| w.pad_driver().clear_bit());

                // clear output
                rtcio
                    .enable_w1tc()
                    .write(|w| unsafe { w.enable_w1tc().bits(1 << self.rtc_number()) });

                /*     
                touch_thres_reg!(sens, $touch_num).write(|w| unsafe {
                 /*    touch_out_th_field!(w, $touch_num).bits(
                        0b0 // Default: 0 for esp32 gets overridden later anyway.
                    )
                    */
                     
                });
                */
               

                
                touch_pad_reg!(RTC_IO::regs(), $touch_num).write(
                    #[allow(unused_unsafe)]
                    |w| unsafe {
                        w.xpd().set_bit();
                        w.tie_opt().clear_bit();

                        // touch_pad8 and 9 are missing a few fields
                        $(
                            crate::ignore!($pad_register_has_all_fields);

                            // clear input_enable
                            w.fun_ie().clear_bit();
                            // Connect pin to analog / RTC module instead of standard GPIO
                            w.mux_sel().set_bit();
                            // Disable pull-up and pull-down resistors on the pin
                            w.rue().clear_bit();
                            w.rde().clear_bit();
                            // Select function "RTC function 1" (GPIO) for analog use
                            w.fun_sel().bits(0b00);
                        )?

                        w
                    }
                );

                // enable the pin
                /* 
                sens.sar_touch_enable().modify(|r, w| unsafe {
                    w.touch_pad_worken().bits(
                        r.touch_pad_worken().bits() | ( 1 << $touch_num )
                    )
                });
                */
            }

            fn touch_measurement(&self, _: $crate::private::Internal) -> u16 {
             //   let regs = $crate::peripherals::SENS::regs();
            //    let reg = touch_out_reg!(regs, $touch_num).read();
              //  touch_meas_out_field!(reg, $touch_num).bits()
                0
            }

            fn touch_nr(&self, _: $crate::private::Internal) -> u8 {
                $touch_num
            }

            fn set_threshold(&self, threshold: u16, _: $crate::private::Internal) {
             /*    let sens = $crate::peripherals::SENS::regs();
                touch_thres_reg!(sens, $touch_num).write(|w| unsafe {
                    touch_out_th_field!(w, $touch_num).bits(threshold)
                });
                */
            }
        })+
    };
}



#[rustfmt::skip]
macro_rules! touch_out_reg {
    ($regs:expr, 1) => { $regs.sar_touch_status1() };
    ($regs:expr, 2) => { $regs.sar_touch_status2() };
    ($regs:expr, 3) => { $regs.sar_touch_status2() };
    ($regs:expr, 4) => { $regs.sar_touch_status3() };
    ($regs:expr, 5) => { $regs.sar_touch_status3() };
    ($regs:expr, 6) => { $regs.sar_touch_status4() };
    ($regs:expr, 7) => { $regs.sar_touch_status4() };
    ($regs:expr, 8) => { $regs.sar_touch_status5() };
    ($regs:expr, 9) => { $regs.sar_touch_status5() };
    ($regs:expr, 10) => { $regs.sar_touch_status6() };
    ($regs:expr, 11) => { $regs.sar_touch_status6() };
    ($regs:expr, 12) => { $regs.sar_touch_status7() };
    ($regs:expr, 13) => { $regs.sar_touch_status7() };
    ($regs:expr, 14) => { $regs.sar_touch_status8() };
}

#[rustfmt::skip]
macro_rules! touch_thres_reg {
    ($regs:expr, 0) => { $regs.sar_touch_thres0() };
    ($regs:expr, 1) => { $regs.sar_touch_thres1() };
    ($regs:expr, 2) => { $regs.sar_touch_thres2() };
    ($regs:expr, 3) => { $regs.sar_touch_thres3() };
    ($regs:expr, 4) => { $regs.sar_touch_thres4() };
    ($regs:expr, 5) => { $regs.sar_touch_thres5() };
    ($regs:expr, 6) => { $regs.sar_touch_thres6() };
    ($regs:expr, 7) => { $regs.sar_touch_thres7() };
    ($regs:expr, 8) => { $regs.sar_touch_thres8() };
    ($regs:expr, 9) => { $regs.sar_touch_thres9() };
    ($regs:expr, 10) => { $regs.sar_touch_thres10() };
    ($regs:expr, 11) => { $regs.sar_touch_thres11() };
    ($regs:expr, 12) => { $regs.sar_touch_thres12() };
    ($regs:expr, 13) => { $regs.sar_touch_thres13() };
    ($regs:expr, 14) => { $regs.sar_touch_thres14() };
    
}

#[rustfmt::skip]
macro_rules! touch_pad_reg {
    ($regs:expr, 0) => { $regs.touch_pad(0) };
    ($regs:expr, 1) => { $regs.touch_pad(1) };
    ($regs:expr, 2) => { $regs.touch_pad(2) };
    ($regs:expr, 3) => { $regs.touch_pad(3) };
    ($regs:expr, 4) => { $regs.touch_pad(4) };
    ($regs:expr, 5) => { $regs.touch_pad(5) };
    ($regs:expr, 6) => { $regs.touch_pad(6) };
    ($regs:expr, 7) => { $regs.touch_pad(7) };
    ($regs:expr, 8) => { $regs.touch_pad(8) };
    ($regs:expr, 9) => { $regs.touch_pad(9) };
    ($regs:expr, 10) => { $regs.touch_pad(10) };
    ($regs:expr, 11) => { $regs.touch_pad(11) };
    ($regs:expr, 12) => { $regs.touch_pad(12) };
    ($regs:expr, 13) => { $regs.touch_pad(13) };
    ($regs:expr, 14) => { $regs.touch_pad(14) };
    
}

#[rustfmt::skip]
macro_rules! touch_out_th_field {
    ($accessor:expr, 0) => { $accessor.touch_out_th0() };
    ($accessor:expr, 1) => { $accessor.touch_out_th1() };
    ($accessor:expr, 2) => { $accessor.touch_out_th0() };
    ($accessor:expr, 3) => { $accessor.touch_out_th1() };
    ($accessor:expr, 4) => { $accessor.touch_out_th0() };
    ($accessor:expr, 5) => { $accessor.touch_out_th1() };
    ($accessor:expr, 6) => { $accessor.touch_out_th0() };
    ($accessor:expr, 7) => { $accessor.touch_out_th1() };
    ($accessor:expr, 8) => { $accessor.touch_out_th0() };
    ($accessor:expr, 9) => { $accessor.touch_out_th1() };
    ($accessor:expr, 10) => { $accessor.touch_out_th0() };
    ($accessor:expr, 11) => { $accessor.touch_out_th1() };
    ($accessor:expr, 12) => { $accessor.touch_out_th0() };
    ($accessor:expr, 13) => { $accessor.touch_out_th1() };
    ($accessor:expr, 14) => { $accessor.touch_out_th0() };
    


}

#[rustfmt::skip]
macro_rules! touch_meas_out_field {
    ($accessor:expr, 0) => { $accessor.touch_meas_out0() };
    ($accessor:expr, 1) => { $accessor.touch_meas_out1() };
    ($accessor:expr, 2) => { $accessor.touch_meas_out0() };
    ($accessor:expr, 3) => { $accessor.touch_meas_out1() };
    ($accessor:expr, 4) => { $accessor.touch_meas_out0() };
    ($accessor:expr, 5) => { $accessor.touch_meas_out1() };
    ($accessor:expr, 6) => { $accessor.touch_meas_out0() };
    ($accessor:expr, 7) => { $accessor.touch_meas_out1() };
    ($accessor:expr, 8) => { $accessor.touch_meas_out0() };
    ($accessor:expr, 9) => { $accessor.touch_meas_out1() };
    ($accessor:expr, 10) => { $accessor.touch_meas_out0() };
    ($accessor:expr, 11) => { $accessor.touch_meas_out1() };
    ($accessor:expr, 12) => { $accessor.touch_meas_out0() };
    ($accessor:expr, 13) => { $accessor.touch_meas_out1() };
    ($accessor:expr, 14) => { $accessor.touch_meas_out0() };


}



touch! {
        // touch_nr, pin_nr, normal_pin
    (1, GPIO1,  true)
    (2, GPIO2,  true)
    (3, GPIO3,  true)
    (4, GPIO4, true)
    (5, GPIO5, true)
    (6, GPIO6, true)
    (7, GPIO7, true)
    (8, GPIO8, true)
    (9, GPIO9, true)
    (10, GPIO10, true)
    (11, GPIO11, true)
    (12, GPIO12, true)
    (13, GPIO13, true)
    (14, GPIO14, true)
 

}

fn enable_iomux_clk_gate() {
    crate::peripherals::SENS::regs()
        .sar_peri_clk_gate_conf()
        .modify(|_, w| w.iomux_clk_en().set_bit());
}
