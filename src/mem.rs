use core::ptr;

#[repr(C)]
#[derive(Copy, Clone)]
pub struct DoorCalibrationData {
    pub max_open_steps: u16,
    pub checksum: u16,
}

// Pick a flash location reserved for config.
// TODO: update this address for your chip's last flash sector.
const CONFIG_ADDR: u32 = 0x0806_0000;

impl DoorCalibrationData {
    pub fn read_from_flash() -> Self {
        // SAFETY: We're just reading raw flash as if it's a struct.
        // If it's never been written, contents will be whatever was in flash.
        unsafe {
            ptr::read_volatile(CONFIG_ADDR as *const DoorCalibrationData)
        }
    }

    pub fn is_valid(&self) -> bool {
        // Super simple validity check:
        // checksum = 0xABCD ^ max_open_steps
        (self.checksum == (0xABCDu16 ^ self.max_open_steps))
            && self.max_open_steps != 0
            && self.max_open_steps < 10_000  // sanity bound
    }
}