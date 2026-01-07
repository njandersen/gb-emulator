const std = @import("std");
const Bus = @import("bus.zig").Bus;

pub const Flags = packed struct(u8) {
    _unused: u4 = 0, // Bits 0-3 are always 0 on Game Boy
    c: bool = false, // Bit 4
    h: bool = false, // Bit 5
    n: bool = false, // Bit 6
    z: bool = false, // Bit 7
};

pub const CPU = struct {
    a: u8 = 0,
    b: u8 = 0,
    c: u8 = 0,
    d: u8 = 0,
    e: u8 = 0,
    h: u8 = 0,
    l: u8 = 0,
    f: Flags = .{},
    pc: u16 = 0x0100,
    sp: u16 = 0xFFFE,
    ime: bool = false,
    bus: *Bus, // The CPU just holds a pointer to the Bus

    pub fn init(bus: *Bus) CPU {
        return CPU{ .bus = bus };
    }

    // --- Memory Helpers ---
    fn readByte(self: *CPU, addr: u16) u8 {
        return self.bus.read(addr);
    }

    fn writeByte(self: *CPU, addr: u16, val: u8) void {
        self.bus.write(addr, val);
    }

    fn fetchByte(self: *CPU) u8 {
        const val = self.readByte(self.pc);
        self.pc +%= 1;
        return val;
    }
    fn fetchU16(self: *CPU) u16 {
        const low = @as(u16, self.readByte(self.pc));
        const high = @as(u16, self.readByte(self.pc +% 1));
        self.pc +%= 2;
        return (high << 8) | low;
    }
    fn getBC(self: *CPU) u16 {
        return (@as(u16, self.b) << 8) | @as(u16, self.c);
    }
    fn setBC(self: *CPU, value: u16) void {
        self.b = @truncate(value >> 8);
        self.c = @truncate(value & 0xFF);
    }

    fn ret(self: *CPU) void {
        const low = @as(u16, self.readByte(self.sp));
        self.sp +%= 1;
        const high = @as(u16, self.readByte(self.sp));
        self.sp +%= 1;
        self.pc = (high << 8) | low;
    }

    // --- Instruction Methods ---
    fn push(self: *CPU, value: u16) void {
        self.sp -%= 1;
        self.writeByte(self.sp, @truncate(value >> 8));
        self.sp -%= 1;
        self.writeByte(self.sp, @truncate(value & 0xFF));
    }

    fn rl_r8(self: *CPU, reg: *u8) void {
        const old_carry: u8 = if (self.f.c) 1 else 0;
        const msb = (reg.* >> 7) & 1;

        // Shift left and bring in the old carry to bit 0
        reg.* = (reg.* << 1) | old_carry;

        // Update flags
        self.f.z = (reg.* == 0);
        self.f.n = false;
        self.f.h = false;
        self.f.c = (msb == 1);
    }

    fn swap_r8(self: *CPU, reg: *u8) void {
        const low = reg.* & 0x0F;
        const high = (reg.* & 0xF0) >> 4;
        reg.* = (low << 4) | high;

        self.f.z = (reg.* == 0);
        self.f.n = false;
        self.f.h = false;
        self.f.c = false;
    }

    // --- The Main Step ---
    pub fn step(self: *CPU) !void {
        const current_pc = self.pc;
        const opcode = self.fetchByte();
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
                self.writeByte(hl, self.a);
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
                self.writeByte(address, self.a);
            },
            0xF0 => { // LDH A, (a8)
                const offset = self.fetchByte();
                const address = 0xFF00 + @as(u16, offset);
                // Read from the bus and put it in A
                self.a = self.readByte(address);
                // Ready for next opcode
            },
            0xFE => { // CP d8
                const pc_before = self.pc;
                const value = self.fetchByte();
                std.debug.print("CP Check: A=0x{X:0>2}, ValueRead=0x{X:0>2} from PC: 0x{X:0>4}\n", .{ self.a, value, pc_before });

                self.f.z = (self.a == value);
                self.f.n = true;
                // ... rest of flags
            },
            0x36 => { // LD (HL), d8
                const value = self.fetchByte();
                // Get the address from HL
                const hl = (@as(u16, self.h) << 8) | self.l;
                // Write the value to that address
                self.writeByte(hl, value);
                // PC is already handled by fetchByte
            },
            0xEA => { // LD (a16), A
                // Use your helper to grab the next two bytes as a u16
                const address = self.fetchU16();
                // Pour the value of A into that specific bucket
                self.writeByte(address, self.a);

                // No pc += 3 needed because fetchU16 handles the movement!
            },
            0x77 => { // LD (HL), A
                const hl = (@as(u16, self.h) << 8) | self.l;
                self.writeByte(hl, self.a);
            },
            0x2A => { // LD A, (HL+)
                const hl = (@as(u16, self.h) << 8) | self.l;
                // 1. Get the data from memory and put it in A
                self.a = self.readByte(hl);
                // 2. Increment the HL pointer
                const new_hl = hl +% 1;
                self.h = @as(u8, @truncate(new_hl >> 8));
                self.l = @as(u8, @truncate(new_hl & 0xFF));
                // PC was already moved by the fetchByte at the start of step()
            },
            0xE2 => { // LD (C), A
                const address = 0xFF00 + @as(u16, self.c);
                self.writeByte(address, self.a);
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

                // Push High Byte
                self.sp -%= 1;
                self.writeByte(self.sp, @truncate(return_addr >> 8));

                // Push Low Byte
                self.sp -%= 1;
                self.writeByte(self.sp, @truncate(return_addr & 0xFF));

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
                const low = @as(u16, self.readByte(self.sp));
                self.sp +%= 1;

                // 2. Pop the High Byte
                const high = @as(u16, self.readByte(self.sp));
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
            0x37 => { // SCF (Set Carry Flag)
                self.f.n = false;
                self.f.h = false;
                self.f.c = true;
                // Note: Z is NOT affected by SCF
            },
            0x47 => { // LD B, A
                self.b = self.a;
            },
            0x4F => {
                // LD C, A
                self.c = self.a;
            },
            0x57 => {
                // LD D, A
                self.d = self.a;
            },
            0x5F => {
                // LD E, A
                self.e = self.a;
            },
            0x67 => {
                // LD H, A
                self.h = self.a;
            },
            0x6F => {
                // LD L, A
                self.l = self.a;
            },
            0xB0 => { // OR B
                self.a |= self.b;

                // Update Flags
                self.f.z = (self.a == 0);
                self.f.n = false;
                self.f.h = false;
                self.f.c = false;
            },
            0xA9 => { // XOR C
                self.a ^= self.c;

                // Update Flags
                self.f.z = (self.a == 0);
                self.f.n = false;
                self.f.h = false;
                self.f.c = false;
            },
            0xA1 => { // AND C
                self.a &= self.c;

                // Update Flags
                self.f.z = (self.a == 0);
                self.f.n = false;
                self.f.h = true; // AND always sets H flag
                self.f.c = false;
            },
            0x79 => { // LD A, C
                self.a = self.c;
            },
            0xEF => { // RST 28h
                const return_addr = self.pc; // The address of the NEXT instruction

                // Push return address onto the stack (High byte then Low byte)
                self.sp -%= 1;
                self.writeByte(self.sp, @truncate(return_addr >> 8));
                self.sp -%= 1;
                self.writeByte(self.sp, @truncate(return_addr & 0xFF));

                // Jump to the fixed vector address
                self.pc = 0x0028;

                std.debug.print("Restarting at vector 0x0028, return address 0x{X:0>4}\n", .{return_addr});
            },
            0xCB => try self.decodeCB(),
            else => {
                std.debug.print("CRASH: Unknown Opcode 0x{X:0>2} at PC 0x{X:0>4}\n", .{ opcode, current_pc });
                return error.UnknownOpcode;
            },
        }
    }

    fn decodeCB(self: *CPU) !void {
        const cb_opcode = self.fetchByte();
        switch (cb_opcode) {
            0x11 => self.rl_r8(&self.c), // Use a pointer to register!
            0x37 => self.swap_r8(&self.a), // Add this!
            0x7C => self.bit_test(7, self.h),
            else => {
                std.debug.print("CRASH: Unknown CB Opcode 0x{X:0>2} at PC 0x{X:0>4}\n", .{ cb_opcode, self.pc });
                return error.UnknownOpcode;
            },
        }
    }

    // A generic "BIT" tester for the CB-table
    fn bit_test(self: *CPU, bit: u3, val: u8) void {
        self.f.z = (val & (@as(u8, 1) << bit)) == 0;
        self.f.n = false;
        self.f.h = true;
    }
};
