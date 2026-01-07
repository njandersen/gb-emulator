const std = @import("std");
const gb_emulator = @import("gb_emulator");

const GameBoy = struct {
    // Registers: The GB has 8-bit registers that sometimes combine into 16-bit registers.
    a: u8 = 0,
    b: u8 = 0,
    c: u8 = 0,
    d: u8 = 0,
    e: u8 = 0,
    h: u8 = 0,
    l: u8 = 0,
    f: Flags = @bitCast(@as(u8, 0)), // Flags register
    pc: u16 = 0x100, // Program Counter starts at 0x100
    sp: u16 = 0xFFFE, // Stack Pointer
    // Memory: 64KB of addressable memory
    bus: [65536]u8 = [_]u8{0} ** 65536,
    ime: bool = false,

    fn fetchByte(self: *GameBoy) u8 {
        const val = self.bus[self.pc];
        self.pc += 1;
        return val;
    }
    fn fetchU16(self: *GameBoy) u16 {
        const low = @as(u16, self.bus[self.pc]);
        const high = @as(u16, self.bus[self.pc + 1]);
        self.pc += 2;
        return (high << 8) | low;
    }
    fn getBC(self: *GameBoy) u16 {
        return (@as(u16, self.b) << 8) | @as(u16, self.c);
    }
    fn setBC(self: *GameBoy, value: u16) void {
        self.b = @truncate(value >> 8);
        self.c = @truncate(value & 0xFF);
    }
    pub fn step(self: *GameBoy) void {
        // 1. Fetch the opcode and MOVE PC immediately
        const opcode = self.fetchByte();
        // 2. Adjust print to show where the opcode actually was
        std.debug.print("Executing opcode: 0x{X:0>2} at PC: 0x{X:0>4}\n", .{ opcode, self.pc - 1 });
        switch (opcode) {
            0x00 => {
                // NOP
            },
            0x31 => {
                // LD SP, d16 (3 bytes: opcode + 2 bytes for address)
                self.sp = self.fetchU16();
            },
            0xAF => { // XOR A (1 byte)
                // This is a common trick to set Register A to 0.
                // Logic: A XOR A is always 0.
                self.a = self.a ^ self.a;
                // Set F register directly using the struct
                // We overwrite all flags because XOR clears N, H, and C
                self.f = .{
                    .z = true,
                    .n = false,
                    .h = false,
                    .c = false,
                };
            },
            0x21 => { // LD HL, d16 (3 bytes)
                // Load an immediate 16-bit value into the H and L registers
                const val = self.fetchU16();
                self.h = @truncate(val >> 8);
                self.l = @truncate(val & 0xFF);
            },
            0xC3 => { // JP nn (jump to 16-bit address)
                self.pc = self.fetchU16();
            },
            0x0E => { // LD C, d8
                self.c = self.fetchByte();
            },
            0x06 => { // LD B, d8
                self.b = self.fetchByte();
            },
            0x32 => { // LD (HL-), A (1 byte)
                // 1. Get the 16-bit address currently held by H and L
                const hl = (@as(u16, self.h) << 8) | self.l;
                // 2. Write the value of Register A to that memory address
                self.bus[hl] = self.a;
                // 3. Decrement the HL pair
                const new_hl = hl -% 1; // -% is "Wrapping Subtraction" in Zig
                self.h = @as(u8, @truncate(new_hl >> 8));
                self.l = @as(u8, @truncate(new_hl & 0xFF));
            },
            0x05 => { // DEC B (1 byte)
                self.b = self.b -% 1;
                // Set the "notes" for the next instruction
                self.f.z = (self.b == 0); // Is it zero?
                self.f.n = true; // Yes, we subtracted.
            },
            0x20 => { // JR NZ, s8 (2 bytes)
                const offset = @as(i8, @bitCast(self.fetchByte()));
                if (!self.f.z) {
                    const new_pc = @as(i16, @intCast(self.pc)) + offset;
                    self.pc = @as(u16, @intCast(new_pc));
                    std.debug.print("NZ Flag is set! Looping back to 0x{X:0>4}\n", .{self.pc});
                } else {
                    // If it IS zero, the loop is finished. Just keep going.
                    std.debug.print("Loop finished, continuing...\n", .{});
                }
            },
            0x0D => {
                // DEC C (1 byte)
                self.c = self.c -% 1;
                // Set the "notes" for the next instruction
                self.f.z = (self.c == 0); // Is it zero?
                self.f.n = true; // Yes, we subtracted.
            },
            0x3E => {
                // LD A, d8 (2 bytes)
                self.a = self.fetchByte();
            },
            0xF3 => { // DI (1 byte)
                // In a full emulator, you'd set a flag like: self.interrupts_enabled = false;
                // For now, just move the needle.
            },
            0xE0 => { // LDH (a8), A (2 bytes)
                const offset = self.fetchByte();
                const address = 0xFF00 + @as(u16, offset);
                self.bus[address] = self.a;
            },
            0xF0 => { // LDH A, (a8)
                const offset = self.fetchByte();
                const address = 0xFF00 + @as(u16, offset);
                // Read from the bus and put it in A
                self.a = self.bus[address];
                // Ready for next opcode
            },
            0xFE => {
                // CP d8 (2 bytes)
                const value = self.fetchByte();
                const result = self.a -% value;
                _ = result; // We don't store the result, just set flags
                self.f.z = (self.a == value); // Set Z flag if equal
                self.f.n = true; // Subtraction
                self.f.c = (self.a < value); // Set C flag if borrow
                self.f.h = ((self.a & 0x0F) < (value & 0x0F)); // Set H flag if borrow from bit 4
            },
            0x36 => { // LD (HL), d8
                const value = self.fetchByte();
                // Get the address from HL
                const hl = (@as(u16, self.h) << 8) | self.l;
                // Write the value to that address
                self.bus[hl] = value;
                // PC is already handled by fetchByte
            },
            0xEA => { // LD (a16), A
                // Use your helper to grab the next two bytes as a u16
                const address = self.fetchU16();
                // Pour the value of A into that specific bucket
                self.bus[address] = self.a;
                // No pc += 3 needed because fetchU16 handles the movement!
            },
            0x77 => { // LD (HL), A
                const hl = (@as(u16, self.h) << 8) | self.l;
                self.bus[hl] = self.a;
            },
            0x2A => { // LD A, (HL+)
                const hl = (@as(u16, self.h) << 8) | self.l;
                // 1. Get the data from memory and put it in A
                self.a = self.bus[hl];
                // 2. Increment the HL pointer
                const new_hl = hl +% 1;
                self.h = @as(u8, @truncate(new_hl >> 8));
                self.l = @as(u8, @truncate(new_hl & 0xFF));
                // PC was already moved by the fetchByte at the start of step()
            },
            0xE2 => { // LD (C), A
                const address = 0xFF00 + @as(u16, self.c);
                self.bus[address] = self.a;
            },
            0x0C => { // INC C
                const old_val = self.c;
                self.c = self.c +% 1;
                self.f.z = (self.c == 0);
                self.f.n = false;
                // Half-carry: did bit 3 overflow?
                self.f.h = (old_val & 0x0F) == 0x0F;
                // Note: Carry flag (C) is NOT affected by INC instructions!
            },
            0xCD => { // CALL nn
                const target_addr = self.fetchU16();
                const return_addr = self.pc;
                self.sp -%= 1;
                self.bus[self.sp] = @truncate(return_addr >> 8);
                self.sp -%= 1;
                self.bus[self.sp] = @truncate(return_addr & 0xFF);
                self.pc = target_addr;
                std.debug.print("Calling subroutine at 0x{X:0>4}, return address 0x{X:0>4}\n", .{ target_addr, return_addr });
            },
            0x01 => { // LD BC, d16
                const value = self.fetchU16();
                self.setBC(value);
                // PC is now at 0x279B
            },
            0x0B => { // DEC BC
                const val = self.getBC();
                self.setBC(val -% 1); // Use wrapping subtraction
                // No flags are changed!
            },
            0x78 => { // LD A, B
                self.a = self.b;
            },
            0xB1 => { // OR C
                self.a = self.a | self.c;

                // Update flags
                self.f.z = (self.a == 0);
                self.f.n = false;
                self.f.h = false;
                self.f.c = false;
            },
            0xC9 => { // RET
                // 1. Pop the Low Byte
                const low = @as(u16, self.bus[self.sp]);
                self.sp +%= 1;

                // 2. Pop the High Byte
                const high = @as(u16, self.bus[self.sp]);
                self.sp +%= 1;

                // 3. Jump back home!
                self.pc = (high << 8) | low;

                std.debug.print("Returning from subroutine to 0x{X:0>4}\n", .{self.pc});
            },
            0xFB => { // EI
                // For a basic emulator, we can just set it to true.
                self.ime = true;
            },
            0x2F => {
                // CPL
                self.a = ~self.a; // ~ bitwise NOT operator
                self.f.n = true;
                self.f.h = true;
            },
            0xE6 => { // AND d8
                const mask = self.fetchByte();
                self.a &= mask;

                // Update flags
                self.f.z = (self.a == 0);
                self.f.n = false;
                self.f.h = true; // Yes, H is always true for AND!
                self.f.c = false;
            },
            else => {
                std.debug.print("CRASH: Unknown opcode 0x{X:0>2} at PC 0x{X:0>4}\n", .{ opcode, self.pc });
                std.process.exit(1);
            },
        }
    }
};
const Flags = packed struct(u8) {
    unused: u4 = 0, // Bottom 4 bits are always zero
    c: bool,
    h: bool,
    n: bool,
    z: bool,
};
pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    var gb = GameBoy{};
    const file = try std.fs.cwd().openFile("roms/test.gb", .{});
    defer file.close();
    const bytes_read = try file.readAll(&gb.bus);
    std.debug.print("Loaded {d} bytes into memory\n", .{bytes_read});
    while (true) {
        // 1. THE HACK: Manually increment the LY register (Scanline)
        // On a real GB, this happens every 456 clock cycles.
        // Here, we'll just do it every CPU instruction for simplicity.
        const ly_reg = &gb.bus[0xFF44];
        ly_reg.* = ly_reg.* +% 1;
        if (ly_reg.* > 153) {
            ly_reg.* = 0;
        }
        gb.step();
    }
}
