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
    fake_ly: u8 = 0,

    pub fn init(bus: *Bus) CPU {
        return CPU{ .bus = bus };
    }

    // --- Memory Helpers ---
    fn readByte(self: *CPU, addr: u16) u8 {
        if (addr == 0xFF44) {
            // Increment every read to simulate time passing
            self.fake_ly +%= 1;
            // Real GB LY goes up to 153 then resets to 0
            if (self.fake_ly > 153) self.fake_ly = 0;
            return self.fake_ly;
        }
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

    fn getDE(self: *CPU) u16 {
        return (@as(u16, self.d) << 8) | self.e;
    }
    fn setDE(self: *CPU, val: u16) void {
        self.d = @truncate(val >> 8);
        self.e = @truncate(val & 0xFF);
    }

    fn setHL(self: *CPU, val: u16) void {
        self.h = @truncate(val >> 8);
        self.l = @truncate(val & 0xFF);
    }
    fn getHL(self: *CPU) u16 {
        return (@as(u16, self.h) << 8) | @as(u16, self.l);
    }

    fn getFlagsAsByte(self: *CPU) u8 {
        var res: u8 = 0;
        if (self.f.z) res |= 0x80;
        if (self.f.n) res |= 0x40;
        if (self.f.h) res |= 0x20;
        if (self.f.c) res |= 0x10;
        return res;
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

    fn setFlagsFromByte(self: *CPU, byte: u8) void {
        self.f.z = (byte & 0x80) != 0;
        self.f.n = (byte & 0x40) != 0;
        self.f.h = (byte & 0x20) != 0;
        self.f.c = (byte & 0x10) != 0;
    }

    //===============================================================================================
    // Interrupts
    //===============================================================================================
    pub fn handleInterrupts(self: *CPU) void {
        if (!self.ime) return; // Master enable must be on

        const ie = self.readByte(0xFFFF); // Interrupt Enable
        const if_reg = self.readByte(0xFF0F); // Interrupt Flag
        const pending = ie & if_reg;

        if (pending > 0) {
            // V-Blank has highest priority (Bit 0)
            if (pending & 0x01 != 0) {
                self.serviceInterrupt(0, 0x0040);
            }
            // ... handle other bits (LCD Stat, Timer, etc.)
        }
    }

    fn serviceInterrupt(self: *CPU, bit: u3, vector: u16) void {
        self.ime = false; // Disable interrupts

        // Clear the specific bit we are servicing in IF
        var if_reg = self.readByte(0xFF0F);
        if_reg &= ~(@as(u8, 1) << bit);
        self.writeByte(0xFF0F, if_reg);

        // Push PC to stack
        self.sp -%= 1;
        self.writeByte(self.sp, @as(u8, @truncate(self.pc >> 8)));
        self.sp -%= 1;
        self.writeByte(self.sp, @as(u8, @truncate(self.pc & 0xFF)));

        // Jump to the interrupt vector
        self.pc = vector;
    }

    //===============================================================================================
    // Main Step
    //===============================================================================================
    pub fn step(self: *CPU) u8 {
        const opcode = self.fetchByte();
        if (self.pc == 0x27AD) { // 0x27AD is the instruction AFTER the loop
            std.debug.print("I FINALLY ESCAPED THE LOOP! BC is now: {X:0>4}\n", .{self.getBC()});
        }

        switch (opcode) {
            0x00 => {
                // NOP
                return 1;
            },
            0x31 => {
                // LD SP, d16 (3 bytes: opcode + 2 bytes for address)
                self.sp = self.fetchU16();
                return 3;
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

                return 1;
            },
            0x21 => { // LD HL, d16 (3 bytes)
                // Load an immediate 16-bit value into the H and L registers
                const val = self.fetchU16();
                self.h = @truncate(val >> 8);
                self.l = @truncate(val & 0xFF);
                return 3;
            },
            0xC3 => { // JP nn (jump to 16-bit address)
                self.pc = self.fetchU16();
                return 4;
            },
            0x0E => { // LD C, d8
                self.c = self.fetchByte();
                return 2;
            },
            0x06 => { // LD B, d8
                self.b = self.fetchByte();
                return 2;
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
                return 2;
            },
            0x05 => { // DEC B (1 byte)
                self.b = self.b -% 1;
                // Set the "notes" for the next instruction
                self.f.z = (self.b == 0); // Is it zero?
                self.f.n = true; // Yes, we subtracted.

                return 1;
            },
            0x20 => { // JR NZ, r8
                const offset = @as(i8, @bitCast(self.fetchByte()));
                if (!self.f.z) {
                    // Use wrapping addition to handle the jump
                    const current_pc_i16 = @as(i16, @bitCast(self.pc));
                    const new_pc_i16 = current_pc_i16 + offset;
                    self.pc = @as(u16, @bitCast(new_pc_i16));
                    return 3;
                }
                return 2;
            },
            0x0D => {
                // DEC C (1 byte)
                self.c = self.c -% 1;
                // Set the "notes" for the next instruction
                self.f.z = (self.c == 0); // Is it zero?
                self.f.n = true; // Yes, we subtracted.
                return 1;
            },
            0x3E => {
                // LD A, d8 (2 bytes)
                self.a = self.fetchByte();
                return 2;
            },
            0xF3 => { // DI (1 byte)
                self.ime = false;
                return 1;
            },
            0xE0 => { // LDH (a8), A (2 bytes)
                const offset = self.fetchByte();
                const address = 0xFF00 + @as(u16, offset);
                self.writeByte(address, self.a);
                return 3;
            },
            0xF0 => { // LDH A, (a8)
                const offset = self.fetchByte();
                const address = 0xFF00 + @as(u16, offset);
                // Read from the bus and put it in A
                self.a = self.readByte(address);
                // Ready for next opcode
                return 3;
            },
            0xFE => { // CP d8
                const value = self.fetchByte();

                // Z: Set if A == value (result is 0)
                self.f.z = (self.a == value);

                // N: Always set for CP (it is a subtraction)
                self.f.n = true;

                // H: Set if there was no borrow from bit 4
                // (Calculation: (A & 0x0F) < (value & 0x0F))
                self.f.h = (self.a & 0x0F) < (value & 0x0F);

                // C: Set if A < value (a "borrow" occurred)
                self.f.c = self.a < value;
                return 2;
            },
            0x36 => { // LD (HL), d8
                const value = self.fetchByte();
                // Get the address from HL
                const hl = (@as(u16, self.h) << 8) | self.l;
                // Write the value to that address
                self.writeByte(hl, value);
                // PC is already handled by fetchByte
                return 3;
            },
            0xEA => { // LD (a16), A
                // Use your helper to grab the next two bytes as a u16
                const address = self.fetchU16();
                // Pour the value of A into that specific bucket
                self.writeByte(address, self.a);

                // No pc += 3 needed because fetchU16 handles the movement!
                return 4;
            },
            0x77 => { // LD (HL), A
                const hl = (@as(u16, self.h) << 8) | self.l;
                self.writeByte(hl, self.a);
                return 2;
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
                return 2;
            },
            0xE2 => { // LD (C), A
                const address = 0xFF00 + @as(u16, self.c);
                self.writeByte(address, self.a);
                return 2;
            },
            0x0C => { // INC C
                const old_val = self.c;
                self.c = self.c +% 1;
                self.f.z = (self.c == 0);
                self.f.n = false;
                // Half-carry: did bit 3 overflow?
                self.f.h = (old_val & 0x0F) == 0x0F;
                // Note: Carry flag (C) is NOT affected by INC instructions!
                return 1;
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
                return 6;
            },
            0x01 => { // LD BC, d16
                const value = self.fetchU16();
                self.setBC(value);
                // PC is now at 0x279B
                return 3;
            },
            0x0B => { // DEC BC
                const val = self.getBC();
                self.setBC(val -% 1); // Use wrapping subtraction
                // No flags are changed!
                return 2;
            },
            0x78 => { // LD A, B
                self.a = self.b;
                return 1;
            },
            0xB1 => { // OR C
                self.a |= self.c;

                // Update flags
                self.f.z = (self.a == 0);
                self.f.n = false;
                self.f.h = false;
                self.f.c = false;
                return 1;
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
                return 4;
            },
            0xFB => { // EI
                // For a basic emulator, we can just set it to true.
                self.ime = true;
                return 1;
            },
            0x2F => {
                // CPL
                self.a = ~self.a; // ~ bitwise NOT operator
                self.f.n = true;
                self.f.h = true;
                return 1;
            },
            0xE6 => { // AND d8
                const mask = self.fetchByte();
                self.a &= mask;

                // Update flags
                self.f.z = (self.a == 0);
                self.f.n = false;
                self.f.h = true; // Yes, H is always true for AND!
                self.f.c = false;
                return 2;
            },
            0x37 => { // SCF (Set Carry Flag)
                self.f.n = false;
                self.f.h = false;
                self.f.c = true;
                // Note: Z is NOT affected by SCF
                return 1;
            },
            0x47 => { // LD B, A
                self.b = self.a;
                return 1;
            },
            0x4F => {
                // LD C, A
                self.c = self.a;
                return 1;
            },
            0x57 => {
                // LD D, A
                self.d = self.a;
                return 1;
            },
            0x5F => {
                // LD E, A
                self.e = self.a;
                return 1;
            },
            0x67 => {
                // LD H, A
                self.h = self.a;
                return 1;
            },
            0x6F => {
                // LD L, A
                self.l = self.a;
                return 1;
            },
            0xB0 => { // OR B
                self.a |= self.b;

                // Update Flags
                self.f.z = (self.a == 0);
                self.f.n = false;
                self.f.h = false;
                self.f.c = false;
                return 1;
            },
            0xA9 => { // XOR C
                self.a ^= self.c;

                // Update Flags
                self.f.z = (self.a == 0);
                self.f.n = false;
                self.f.h = false;
                self.f.c = false;
                return 1;
            },
            0xA1 => { // AND C
                self.a &= self.c;

                // Update Flags
                self.f.z = (self.a == 0);
                self.f.n = false;
                self.f.h = true; // AND always sets H flag
                self.f.c = false;
                return 1;
            },
            0x79 => { // LD A, C
                self.a = self.c;
                return 1;
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
                return 4;
            },
            0x87 => { // ADD A, A
                const a = self.a;
                const res = a +% a;

                // Update Flags
                self.f.z = (res == 0);
                self.f.n = false;
                // Half-Carry: did we carry from bit 3 to bit 4?
                self.f.h = ((a & 0x0F) + (a & 0x0F)) > 0x0F;
                // Carry: did we carry from bit 7? (Or simply: was bit 7 set?)
                self.f.c = (@as(u16, a) + @as(u16, a)) > 0xFF;

                self.a = res;
                return 1;
            },
            0xE1 => { // POP HL
                // 1. Pop the Low byte into L
                self.l = self.readByte(self.sp);
                self.sp +%= 1;

                // 2. Pop the High byte into H
                self.h = self.readByte(self.sp);
                self.sp +%= 1;
                return 3;
            },
            0x16 => { // LD D, d8
                const val = self.fetchByte();
                self.d = val;
                return 2;
            },
            0x19 => { // ADD HL, DE
                const hl = self.getHL();
                const de = self.getDE();
                const res = hl +% de;

                // Update Flags
                self.f.n = false;
                // Half-Carry: Carry from bit 11 to bit 12
                self.f.h = ((hl & 0x0FFF) + (de & 0x0FFF)) > 0x0FFF;
                // Carry: Carry from bit 15
                self.f.c = (@as(u32, hl) + @as(u32, de)) > 0xFFFF;
                // Z is NOT affected

                self.setHL(res);
                return 2;
            },
            0x5E => { // LD E, (HL)
                const address = self.getHL();
                self.e = self.readByte(address);
            },
            0x46 => {
                // LD B, (HL)
                self.b = self.readByte(self.getHL());
            },
            0x4E => {
                // LD C, (HL)
                self.c = self.readByte(self.getHL());
            },
            0x56 => {
                // LD D, (HL)
                self.d = self.readByte(self.getHL());
            },
            0x66 => {
                // LD H, (HL)
                self.h = self.readByte(self.getHL());
            },
            0x6E => {
                // LD L, (HL)
                self.l = self.readByte(self.getHL());
            },
            0x7E => {
                // LD A, (HL)
                self.a = self.readByte(self.getHL());
            },
            0x23 => { // INC HL
                const val = self.getHL();
                self.setHL(val +% 1);
                // No flags affected
            },
            0xD5 => { // PUSH DE
                // 1. Decrement SP and write High byte (D)
                self.sp -%= 1;
                self.writeByte(self.sp, self.d);

                // 2. Decrement SP and write Low byte (E)
                self.sp -%= 1;
                self.writeByte(self.sp, self.e);
            },
            0xE9 => { // JP (HL)
                self.pc = self.getHL();
            },
            0x11 => { // LD DE, d16
                const low = self.fetchByte();
                const high = self.fetchByte();
                self.e = low;
                self.d = high;
            },
            0x12 => { // LD (DE), A
                const address = self.getDE();
                self.writeByte(address, self.a);
            },
            0x02 => {
                // LD (BC), A
                const addr = self.getBC();
                self.writeByte(addr, self.a);
            },
            0x13 => {
                // INC DE
                const val = self.getDE();
                self.setDE(val +% 1);
            },
            0xE5 => {
                // PUSH HL
                // 1. Decrement SP and write High byte (H)
                self.sp -%= 1;
                self.writeByte(self.sp, self.h);

                // 2. Decrement SP and write Low byte (L)
                self.sp -%= 1;
                self.writeByte(self.sp, self.l);
            },
            0x1A => {
                // LD A, (DE)
                self.a = self.readByte(self.getDE());
            },
            0x22 => { // LD (HL+), A
                const addr = self.getHL();
                self.writeByte(addr, self.a);
                self.setHL(addr +% 1);
            },
            0xD1 => {
                // POP DE
                self.e = self.readByte(self.sp);
                self.sp +%= 1;

                self.d = self.readByte(self.sp);
                self.sp +%= 1;
            },
            0x7C => {
                // LD A, H
                self.a = self.h;
            },
            0xF5 => {
                // PUSH AF
                self.sp -%= 1;
                self.writeByte(self.sp, self.a);

                self.sp -%= 1;
                self.writeByte(self.sp, self.getFlagsAsByte()); // Low byte (F)
            },
            0xC5 => {
                // PUSH BC
                self.sp -%= 1;
                self.writeByte(self.sp, self.b);

                self.sp -%= 1;
                self.writeByte(self.sp, self.c);
            },
            0xFA => { // LD A, (nn)
                const addr = self.fetchU16();
                self.a = self.readByte(addr);
            },
            0x28 => { // JR Z, r8
                const offset = @as(i8, @bitCast(self.fetchByte()));
                if (self.f.z) {
                    // Only jump if Zero flag is true
                    self.pc = @as(u16, @bitCast(@as(i16, @intCast(self.pc)) + offset));
                }
            },
            0xA7 => {
                // AND A
                self.a &= self.a;

                // Update Flags
                self.f.z = (self.a == 0);
                self.f.n = false;
                self.f.h = true; // AND always sets H flag
                self.f.c = false;
            },
            0x1C => {
                // INC E
                const old_val = self.e;
                self.e = self.e +% 1;
                self.f.z = (self.e == 0);
                self.f.n = false;
                // Half-carry: did bit 3 overflow?
                self.f.h = (old_val & 0x0F) == 0x0F;
                // Note: Carry flag (C) is NOT affected by INC instructions!
            },
            0xCA => {
                // JP Z, a16
                const low = self.fetchByte();
                const high = self.fetchByte();
                const address = (@as(u16, high) << 8) | low;

                if (self.f.z) {
                    self.pc = address;
                }
            },
            0xC8 => {
                // RET Z
                if (self.f.z) {
                    const low = self.readByte(self.sp);
                    self.sp +%= 1;
                    const high = self.readByte(self.sp);
                    self.sp +%= 1;
                    self.pc = (@as(u16, high) << 8) | low;
                }
            },
            0x18 => { // JR s8
                const offset = @as(i8, @bitCast(self.fetchByte()));
                // We cast to i16 to handle the signed math before putting it back into the u16 PC
                const new_pc = @as(i16, @intCast(self.pc)) + offset;
                self.pc = @as(u16, @bitCast(new_pc));
            },
            0xC1 => {
                // POP BC
                self.c = self.readByte(self.sp);
                self.sp +%= 1;

                self.b = self.readByte(self.sp);
                self.sp +%= 1;
            },
            0xF1 => { // POP AF
                // 1. Pop the Low byte (Flags)
                const f_raw = self.readByte(self.sp);
                self.sp +%= 1;

                // 2. Pop the High byte (Accumulator)
                self.a = self.readByte(self.sp);
                self.sp +%= 1;

                // 3. Mask the flags: only Z, N, H, C (top 4 bits) are kept
                self.setFlagsFromByte(f_raw & 0xF0);
            },
            0xCB => self.decodeCB(),
            else => {
                std.debug.print("CRASH: Unknown Opcode 0x{X:0>2} at PC 0x{X:0>4}\n", .{ opcode, self.pc - 1 });
                @panic("Unknown Opcode");
            },
        }
    }

    fn decodeCB(self: *CPU) u8 {
        const cb_opcode = self.fetchByte();
        switch (cb_opcode) {
            0x11 => self.rl_r8(&self.c), // Use a pointer to register!
            0x37 => self.swap_r8(&self.a), // Add this!
            0x7C => self.bit_test(7, self.h),
            0x87 => { // RES 0, A (Reset bit 0 of Register A)
                self.a &= ~(@as(u8, 1) << 0);
            },
            else => {
                std.debug.print("CRASH: Unknown CB Opcode 0x{X:0>2} at PC 0x{X:0>4}\n", .{ cb_opcode, self.pc - 1 });
                @panic("Unknown CB Opcode");
            },
        }
        return 2;
    }

    // A generic "BIT" tester for the CB-table
    fn bit_test(self: *CPU, bit: u3, val: u8) void {
        self.f.z = (val & (@as(u8, 1) << bit)) == 0;
        self.f.n = false;
        self.f.h = true;
    }
};
