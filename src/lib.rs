//! S2-LP Radio Driver
//! Copyright 2018 Ryan Kurte

#![cfg_attr(not(feature="std"), no_std)]

use core::fmt::Debug;
use core::marker::PhantomData;

use embedded_hal::{delay::blocking::DelayUs, spi::{Mode, Phase, Polarity}};
use radio::{BasicChannel, Channel as _, Registers as _};

#[cfg(not(feature="defmt"))]
use log::debug;

#[cfg(feature="defmt")]
use defmt::debug;

pub mod base;
pub use base::{Base, Io};

pub mod device;
use device::*;


/// S2lp SPI operating mode
pub const SPI_MODE: Mode = Mode {
    polarity: Polarity::IdleLow,
    phase: Phase::CaptureOnFirstTransition,
};


/// S2lp device instance
pub struct S2lp<Hal, SpiErr: Debug, PinErr: Debug, DelayErr: Debug> {
    hal: Hal,
    _err: PhantomData<Error<SpiErr, PinErr, DelayErr>>,
}

/// S2lp configuration
pub struct Config {
    /// Radio frequency for communication
    pub rf_freq_mhz: u16,
    // Clock / Crystal frequency
    pub clock_freq: ClockFreq,
}

impl Default for Config {
    fn default() -> Self {
        Self { 
            rf_freq_mhz: 433, 
            clock_freq: ClockFreq::Clock48MHz,
        }
    }
}

/// S2lp device errors
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "thiserror", derive(thiserror::Error))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error<SpiErr: Debug, PinErr: Debug, DelayErr: Debug> {
    #[cfg_attr(feature = "thiserror", error("SPI error: {0}"))]
    /// Communications (SPI or UART) error
    Spi(SpiErr),

    #[cfg_attr(feature = "thiserror", error("Pin error: {0}"))]
    /// Pin control error
    Pin(PinErr),

    #[cfg_attr(feature = "thiserror", error("Delay error: {0}"))]
    /// Delay error
    Delay(DelayErr),

    #[cfg_attr(feature = "thiserror", error("Unexpected register value (reg: 0x{0:02x} val: 0x:{1:02x}"))]
    /// Unexpected response from device
    UnexpectedValue(u8, u8),

    #[cfg_attr(feature = "thiserror", error("Unsupported operation"))]
    /// Unsupported operation
    Unsupported,

    #[cfg_attr(feature = "thiserror", error("Device communication failed"))]
    /// Device communication failed
    NoCommunication,
}


impl<Hal, SpiErr, PinErr, DelayErr> S2lp<Hal, SpiErr, PinErr, DelayErr>
where
    Hal: Base<SpiErr, PinErr, DelayErr>,
    SpiErr: Debug,
    PinErr: Debug,
    DelayErr: Debug,
{
    /// Create a new device with the provided HAL and config
    pub fn new(hal: Hal, _config: Config) -> Result<Self, Error<SpiErr, PinErr, DelayErr>> {
        let mut s2lp = S2lp { hal, _err: PhantomData };

        debug!("Resetting device");

        // Reset device
        s2lp.hal.reset()?;

        debug!("Fetching device info");

        // Attempt to read part information to check connectivity
        let i = s2lp.info()?;
        if i.part_no == 0x00 || i.part_no == 0xFF {
            return Err(Error::NoCommunication)
        }
        debug!("Found device: {:?}", i);

        // TODO: actually configure the thing

        Ok(s2lp)
    }

    /// Send a command strobe
    pub fn cmd_strobe(&mut self, cmd: device::Command) -> Result<(), Error<SpiErr, PinErr, DelayErr>>  {
        // Override SMPS frequency for RX and TX commands
        match cmd {
            device::Command::Rx => {
                // sets the SMPS switching frequency to 3.12MHz.
                self.write_register::<PmConf3>(0x76.into())?;
            },
            device::Command::Tx => {
                //sets the SMPS switching frequency to 5.46MHz about for ETSI regulation compliance.
                self.write_register::<PmConf3>(0x9c.into())?;
            },
            _ => {}
        };

        // Execute command strobe
        self.hal.spi_write(&[SpiCommand::Strobe as u8, cmd.clone() as u8], &[])?;

        Ok(())
    }

    /// Fetch device information (part number and version)
    pub fn info(&mut self) -> Result<Info, Error<SpiErr, PinErr, DelayErr>> {
        Ok(Info{
            part_no: self.read_register::<DeviceInfo1>().map(|r| r.part_no() )?,
            version: self.read_register::<DeviceInfo0>().map(|r| r.version() )?,
        })
    }

    /// Set radio operating channel
    pub fn set_channel(&mut self, channel: u8) -> Result<(), Error<SpiErr, PinErr, DelayErr>> {
        self.write_register::<ChNum>(ChNum::new().with_ch_num(channel))?;
        Ok(())
    }
    
    /// Fetch radio operating channel
    pub fn get_channel(&mut self) -> Result<u8, Error<SpiErr, PinErr, DelayErr>> {
        self.read_register::<ChNum>().map(|r| r.ch_num() )
    }
}

#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Info {
    pub part_no: u8,
    pub version: u8,
}

impl<Hal, SpiErr, PinErr, DelayErr> radio::Registers<u8> for S2lp<Hal, SpiErr, PinErr, DelayErr>
where
    Hal: Base<SpiErr, PinErr, DelayErr>,
    SpiErr: Debug,
    PinErr: Debug,
    DelayErr: Debug,
{
    type Error = Error<SpiErr, PinErr, DelayErr>;

    /// Read a register value
    fn read_register<R: radio::Register<Word = u8>>(&mut self) -> Result<R, Self::Error> {
        let mut d = [0u8];
        self.hal.spi_read(
            &[SpiCommand::Read as u8, R::ADDRESS], 
            &mut d)?;

        R::try_from(d[0]).map_err(|_e| Error::UnexpectedValue(R::ADDRESS, d[0]))
    }

    /// Write a register value
    fn write_register<R: radio::Register<Word = u8>>(
        &mut self,
        value: R,
    ) -> Result<(), Self::Error> {
        self.hal.spi_write(
            &[SpiCommand::Write as u8, R::ADDRESS as u8],
            &[value.into()],
        )
    }
}


impl<Hal, SpiErr, PinErr, DelayErr> radio::State for S2lp<Hal, SpiErr, PinErr, DelayErr>
where
    Hal: Base<SpiErr, PinErr, DelayErr>,
    SpiErr: Debug,
    PinErr: Debug,
    DelayErr: Debug,
{
    type Error = Error<SpiErr, PinErr, DelayErr>;

    type State = device::State;

    fn set_state(&mut self, _state: Self::State) -> Result<(), Self::Error> {
        todo!()
    }

    fn get_state(&mut self) -> Result<Self::State, Self::Error> {
        todo!()
    }
}

impl<Hal, SpiErr, PinErr, DelayErr> radio::Channel for S2lp<Hal, SpiErr, PinErr, DelayErr>
where
    Hal: Base<SpiErr, PinErr, DelayErr>,
    SpiErr: Debug,
    PinErr: Debug,
    DelayErr: Debug,
{
    type Channel = radio::BasicChannel;
    type Error = Error<SpiErr, PinErr, DelayErr>;

    fn set_channel(&mut self, _channel: &Self::Channel) -> Result<(), Self::Error> {
        todo!()
    }
}

impl<Hal, SpiErr, PinErr, DelayErr> radio::Power for S2lp<Hal, SpiErr, PinErr, DelayErr>
where
    Hal: Base<SpiErr, PinErr, DelayErr>,
    SpiErr: Debug,
    PinErr: Debug,
    DelayErr: Debug,
{
    type Error = Error<SpiErr, PinErr, DelayErr>;

    fn set_power(&mut self, power: i8) -> Result<(), Self::Error> {
        todo!()
    }
}

impl<Hal, SpiErr, PinErr, DelayErr> radio::Rssi for S2lp<Hal, SpiErr, PinErr, DelayErr>
where
    Hal: Base<SpiErr, PinErr, DelayErr>,
    SpiErr: Debug,
    PinErr: Debug,
    DelayErr: Debug,
{
    type Error = Error<SpiErr, PinErr, DelayErr>;

    fn poll_rssi(&mut self) -> Result<i16, Self::Error> {
        todo!()
    }
}

impl<Hal, SpiErr, PinErr, DelayErr> radio::Transmit for S2lp<Hal, SpiErr, PinErr, DelayErr>
where
    Hal: Base<SpiErr, PinErr, DelayErr>,
    SpiErr: Debug,
    PinErr: Debug,
    DelayErr: Debug,
{
    type Error = Error<SpiErr, PinErr, DelayErr>;

    fn start_transmit(&mut self, _data: &[u8]) -> Result<(), Self::Error> {
        todo!()
    }

    fn check_transmit(&mut self) -> Result<bool, Self::Error> {
        todo!()
    }
}


impl<Hal, SpiErr, PinErr, DelayErr> radio::Receive for S2lp<Hal, SpiErr, PinErr, DelayErr>
where
    Hal: Base<SpiErr, PinErr, DelayErr>,
    SpiErr: Debug,
    PinErr: Debug,
    DelayErr: Debug,
{
    type Error = Error<SpiErr, PinErr, DelayErr>;

    type Info = radio::BasicInfo;

    fn start_receive(&mut self) -> Result<(), Self::Error> {
        todo!()
    }

    fn check_receive(&mut self, _restart: bool) -> Result<bool, Self::Error> {
        todo!()
    }

    fn get_received(&mut self, _buff: &mut [u8]) -> Result<(usize, Self::Info), Self::Error> {
        todo!()
    }
}

impl<Hal, SpiErr, PinErr, DelayErr> DelayUs for S2lp<Hal, SpiErr, PinErr, DelayErr>
where
    Hal: Base<SpiErr, PinErr, DelayErr>,
    SpiErr: Debug,
    PinErr: Debug,
    DelayErr: Debug,
{
    type Error = Error<SpiErr, PinErr, DelayErr>;

    fn delay_us(&mut self, us: u32) -> Result<(), Self::Error> {
        self.hal.delay_us(us)
    }
}

/// 28-bit SYNT word and band select bit from frequency
pub fn synt_from_freq( f_xo : u32, freq: usize, divider_enabled: bool) -> (u32, bool) {
    let band = if freq > HIGH_BAND_LOWER_LIMIT {
        (4,false)
    } else {
        (8,true)
    };
    let refdiv = if divider_enabled { 2 } else { 1 };
    let synt = (((freq as u64) << 19) * (band.0 as u64) * (refdiv as u64)) / (f_xo as u64);
    (synt as u32, band.1)
}

/// Frequency from SYNT word
pub fn freq_from_synt( f_xo: u32, synt: u32, mid_band_selected: bool, divider_enabled: bool) -> u32 {
    let band = if mid_band_selected { 8 } else { 4 };
    let refdiv = if divider_enabled { 2  } else { 1 };
    let freq = ((f_xo as u64 * synt as u64) >> 19) / band / refdiv;
    freq as u32
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn check_freq() {
        let fxo50 = 50_000_000;
        let fxo24 = 24_000_000;
        assert_eq!((0x2499999,false), synt_from_freq(fxo50, 915_000_000, false));
        assert_eq!((0x4933333,false), synt_from_freq(fxo50, 915_000_000, true));
        assert_eq!((0x22A3D70,true), synt_from_freq(fxo50, 433_000_000, false));
        assert_eq!((0x4547AE1,true), synt_from_freq(fxo50, 433_000_000, true));
        assert_eq!((0x4C40000,false), synt_from_freq(fxo24, 915_000_000, false));
        assert_eq!((0x9055555,true), synt_from_freq(fxo24, 433_000_000, true));
        assert_eq!((0x2A3D70A,false), synt_from_freq(fxo50, 1_056_000_000, false));
        assert_eq!((0xAFB3333,true), synt_from_freq(fxo24, 527_100_000, true));
    }
    #[test]
    fn check_synt() {
        let fxo50 = 50_000_000;
        let fxo24 = 24_000_000;
        assert_eq!(914_999_985, freq_from_synt(fxo50, 0x2499999, false, false));
        assert_eq!(915_000_009, freq_from_synt(fxo50, 0x249999A, false, false));
        assert_eq!(433_000_001, freq_from_synt(fxo24, 0x9055556, true, true));
    }

}