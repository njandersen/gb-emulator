const std = @import("std");
const PPU = @import("ppu.zig").PPU;

pub const Bus = struct {
    wram: [0x2000]u8 = [_]u8{0} ** 0x2000, // Work RAM (0xC000-0xDFFF)
    hram: [0x7F]u8 = [_]u8{0} ** 0x7F, // High RAM (0xFF80-0xFFFE)
    rom: [0x8000]u8 = [_]u8{0} ** 0x8000, // Basic 32KB ROM

    // Interrupt Registers
    ie: u8 = 0, // 0xFFFF
    if_flag: u8 = 0, // 0xFF0F

    ppu: PPU = .{},

    pub fn init() Bus {
        var bus = Bus{};
        // Initialize hardware registers to post-boot state
        bus.ie = 0x00;
        bus.if_flag = 0xE1; // Initial state
        bus.ppu.lcdc = 0x91;
        bus.ppu.ly = 0x00;
        // ... other registers
        return bus;
    }

    pub fn read(self: *Bus, addr: u16) u8 {
        const val = switch (addr) {
            0x0000...0x7FFF => self.rom[addr],
            0x8000...0x9FFF => self.ppu.vram[addr - 0x8000],
            0xC000...0xDFFF => self.wram[addr - 0xC000],
            0xFE00...0xFE9F => self.ppu.oam[addr - 0xFE00],
            0xFF0F => {
                return self.if_flag;
            },
            0xFF40...0xFF4B => self.ppu.readRegister(addr),
            0xFF80...0xFFFE => self.hram[addr - 0xFF80],
            0xFFFF => self.ie,
            else => 0xFF,
        };
        // IF THE CPU IS READING LY, WE MUST SEE THIS
        if (addr == 0xFF44) {
            std.debug.print(">>> BUS HARDWARE: Address 0xFF44 read. Returning: {d}\n", .{val});
        }
        return val;
    }

    pub fn write(self: *Bus, addr: u16, val: u8) void {
        // Add specific HRAM write tracking
        if (addr >= 0xFF80 and addr <= 0xFFFE) {
            std.debug.print(">>> HRAM WRITE: Addr 0x{X:0>4} Val 0x{X:0>2}\n", .{ addr, val });
        }
        // Only print "important" hardware register writes
        if (addr >= 0xFF00) {
            std.debug.print("--- HW WRITE: Addr 0x{X:0>4} Val 0x{X:0>2} ---\n", .{ addr, val });
        }
        switch (addr) {
            0x0000...0x7FFF => {}, // ROM is read-only
            0x8000...0x9FFF => self.ppu.vram[addr - 0x8000] = val,
            0xC000...0xDFFF => self.wram[addr - 0xC000] = val,
            0xFE00...0xFE9F => self.ppu.oam[addr - 0xFE00] = val,
            0xFF0F => {
                std.debug.print(">>> WARNING: IF Register written! Old: 0x{X:0>2} New: 0x{X:0>2}\n", .{ self.if_flag, val });
                self.if_flag = val;
            },
            0xFF40...0xFF4B => self.ppu.writeRegister(addr, val),
            0xFF80...0xFFFE => self.hram[addr - 0xFF80] = val,
            0xFFFF => {
                std.debug.print(">>> BUS WRITE: IE Register (0xFFFF) set to 0x{X:0>2}\n", .{val});
                self.ie = val;
            },
            else => {},
        }
    }
    pub fn loadROM(self: *Bus, data: []const u8) void {
        // Copy data specifically into the rom field
        const size = @min(data.len, self.rom.len);
        @memcpy(self.rom[0..size], data[0..size]);
    }

    // pub fn loadROM(self: *Bus, data: []const u8) void {
    //     const size = @min(data.len, 0x8000); // Max ROM size for now
    //     @memcpy(self.memory[0..size], data[0..size]);
    // }
};
