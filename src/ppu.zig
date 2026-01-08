const std = @import("std");

pub const PPU = struct {
    vram: [0x2000]u8 = [_]u8{0} ** 0x2000,
    oam: [0xA0]u8 = [_]u8{0} ** 0xA0,

    // PPU Registers
    ly: u8 = 0,
    lcdc: u8 = 0,
    scy: u8 = 0,
    scx: u8 = 0,

    dot_counter: u32 = 0,

    pub fn readRegister(self: *PPU, addr: u16) u8 {
        return switch (addr) {
            0xFF40 => self.lcdc,
            0xFF42 => self.scy,
            0xFF43 => self.scx,
            0xFF44 => self.ly,
            else => 0xFF,
        };
    }

    pub fn writeRegister(self: *PPU, addr: u16, val: u8) void {
        switch (addr) {
            0xFF40 => self.lcdc = val,
            0xFF42 => self.scy = val,
            0xFF43 => self.scx = val,
            0xFF44 => {}, // LY is read-only for the CPU
            else => {},
        }
    }

    pub fn tick(self: *PPU, m_cycles: u32, if_reg: *u8) void {
        self.dot_counter += m_cycles * 4;

        if (self.dot_counter >= 456) {
            self.dot_counter -= 456;
            self.ly = (self.ly + 1) % 154;

            // Now you can use the pointer we just added
            if (self.ly == 144) {
                if_reg.* |= 0x01; // Request V-Blank
            }
        }
    }
};
