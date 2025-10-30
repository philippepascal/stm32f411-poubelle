#![allow(dead_code)]

use crc32fast::Hasher;
use serde::{Deserialize, Serialize};
use stm32f4xx_hal::pac::FLASH;
use stm32f4xx_hal::pac::flash::RegisterBlock as FlashRB;

// ======== constants ========
const FLASH_USER_START: u32 = 0x0806_0000;
const CFG_MAGIC: u32 = 0x43_46_47_31; // 'CFG1'
const CFG_VERSION: u16 = 1;
const CFG_MAX_SIZE: usize = 256;

// ======== user config struct ========
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct Config {
    // pub step_rate_hz: u16,
    // pub direction_default_opening: bool,
    // pub pulses_per_rev: u16,
    pub travel_step_count: u32,
}

// ======== internal header ========
#[repr(C)]
#[derive(Clone, Copy)]
struct ConfigHeader {
    magic: u32,
    version: u16,
    len: u16,
    crc32: u32,
}

// ======== helpers ========
fn calc_crc32(data: &[u8]) -> u32 {
    let mut h = Hasher::new();
    h.update(data);
    h.finalize()
}

fn serialize_config(cfg: &Config, out: &mut [u8]) -> Result<usize, postcard::Error> {
    postcard::to_slice(cfg, out).map(|s| s.len())
}

// ======== low-level FLASH helpers (PAC-based) ========
#[inline(always)]
fn flash() -> &'static FlashRB {
    unsafe { &*FLASH::ptr() }
}

#[inline(always)]
fn flash_wait_not_busy() {
    let f = flash();
    while f.sr().read().bsy().bit_is_set() {}
}

#[inline(always)]
fn flash_unlock() {
    let f = flash();
    if f.cr().read().lock().bit_is_set() {
        // Unlock sequence
        f.keyr().write(|w| unsafe { w.bits(0x4567_0123) });
        f.keyr().write(|w| unsafe { w.bits(0xCDEF_89AB) });
    }
}

#[inline(always)]
fn flash_lock() {
    let f = flash();
    // Set LOCK bit
    f.cr().modify(|_, w| w.lock().set_bit());
}

/// Erase a single sector containing the user area (Sector 7 @ 0x0806_0000 on F411)
fn flash_erase_sector7() -> Result<(), ()> {
    let f = flash();
    flash_wait_not_busy();
    // SER = 1 (sector erase), SNB = 7, STRT = 1
    f.cr().modify(|_, w| unsafe {
        w.ser().set_bit()
         .snb().bits(7)
         .strt().set_bit()
    });
    flash_wait_not_busy();
    // Clear SER
    f.cr().modify(|_, w| w.ser().clear_bit());
    // Check for errors (optional: PGSERR, PGERR, WRPRTERR on some F4 variants)
    Ok(())
}

/// Program a buffer at `addr` (must be 4-byte aligned). Pads to 32-bit.
fn flash_program(addr: u32, data: &[u8]) -> Result<(), ()> {
    if (addr & 3) != 0 { return Err(()); }

    let f = flash();
    let mut i = 0;
    while i < data.len() {
        // assemble a 32-bit word (little-endian), pad missing bytes with 0xFF (erased state)
        let b0 = data.get(i).copied().unwrap_or(0xFF);
        let b1 = data.get(i+1).copied().unwrap_or(0xFF);
        let b2 = data.get(i+2).copied().unwrap_or(0xFF);
        let b3 = data.get(i+3).copied().unwrap_or(0xFF);
        let word = (b0 as u32)
                 | ((b1 as u32) << 8)
                 | ((b2 as u32) << 16)
                 | ((b3 as u32) << 24);

        flash_wait_not_busy();
        // Set PSIZE=word (0b10) and PG=1
        f.cr().modify(|_, w| unsafe { w.psize().bits(0b10).pg().set_bit() });
        unsafe { core::ptr::write_volatile((addr + i as u32) as *mut u32, word) };
        flash_wait_not_busy();
        // Clear PG
        f.cr().modify(|_, w| w.pg().clear_bit());

        i += 4;
    }
    Ok(())
}
pub fn load_config() -> Option<Config> {
    let hdr = unsafe { core::ptr::read_volatile(FLASH_USER_START as *const ConfigHeader) };
    if hdr.magic != CFG_MAGIC || hdr.version == 0 || hdr.len as usize > CFG_MAX_SIZE {
        return None;
    }

    let len = hdr.len as usize;
    let payload_addr = FLASH_USER_START + core::mem::size_of::<ConfigHeader>() as u32;
    let mut buf = [0u8; CFG_MAX_SIZE];
    for i in 0..len {
        buf[i] = unsafe { core::ptr::read_volatile((payload_addr + i as u32) as *const u8) };
    }

    if calc_crc32(&buf[..len]) != hdr.crc32 {
        return None;
    }

    postcard::from_bytes::<Config>(&buf[..len]).ok()
}

pub fn save_config_blocking(cfg: &Config) -> Result<(), ()> {
    let mut payload = [0u8; CFG_MAX_SIZE];
    let len = serialize_config(cfg, &mut payload).map_err(|_| ())?;
    let crc = calc_crc32(&payload[..len]);
    let hdr = ConfigHeader {
        magic: CFG_MAGIC,
        version: CFG_VERSION,
        len: len as u16,
        crc32: crc,
    };

    cortex_m::interrupt::free(|_| {
        flash_unlock();
        let res = (|| {
            flash_erase_sector7()?;
            // Program header
            let hdr_bytes = unsafe {
                core::slice::from_raw_parts(
                    (&hdr as *const ConfigHeader) as *const u8,
                    core::mem::size_of::<ConfigHeader>(),
                )
            };
            flash_program(FLASH_USER_START, hdr_bytes)?;
            // Program payload
            let payload_addr = FLASH_USER_START + core::mem::size_of::<ConfigHeader>() as u32;
            flash_program(payload_addr, &payload[..len])?;
            Ok(())
        })();
        flash_lock();
        res
    })
}