const std = @import("std");

pub const Bus = struct {
    memory: [65536]u8 = [_]u8{0} ** 65536,

    pub fn read(self: *Bus, addr: u16) u8 {
        // THE HACK: If the CPU reads the LY register,
        // return a fake value to bypass the "wait for screen" loops.
        if (addr == 0xFF44) {
            // Change 0x90 to 0x94
            return 0x94;
        }
        return self.memory[addr];
    }

    pub fn write(self: *Bus, addr: u16, val: u8) void {
        // Prevent writing to ROM (0x0000 - 0x7FFF)
        if (addr < 0x8000) return;

        self.memory[addr] = val;
    }

    pub fn loadROM(self: *Bus, data: []const u8) void {
        const size = @min(data.len, 0x8000); // Max ROM size for now
        @memcpy(self.memory[0..size], data[0..size]);
    }
};
