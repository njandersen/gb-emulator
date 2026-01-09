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
    halted: bool = false,

    pub fn init(bus: *Bus) CPU {
        return CPU{
            .bus = bus,
            .a = 0x01, // Post-boot state
            .f = .{ .z = true, .n = false, .h = true, .c = true },
            .b = 0x00,
            .c = 0x13,
            .d = 0x00,
            .e = 0xD8,
            .h = 0x01,
            .l = 0x4D,
            .sp = 0xFFFE,
            .pc = 0x0100,
        };
    }

    // ===============================================================================================
    // --- Main ---
    // ===============================================================================================
    pub fn step(self: *CPU) u8 {
        if (self.pc == 0x02ED) {
            const ie = self.bus.read(0xFFFF);
            const if_reg = self.bus.read(0xFF0F);
            std.debug.print("--- AT 0x02ED --- IME: {} | IE: 0x{X:0>2} | IF: 0x{X:0>2}\n", .{ self.ime, ie, if_reg });
        }
        if (self.halted) {
            return 1; // Return 1 M-cycle while asleep
        }
        const opcode = self.fetchByte();

        // // Print ALL instructions, not just at specific PC
        // std.debug.print("PC: 0x{X:0>4} | Op: 0x{X:0>2} | A:{X:0>2} B:{X:0>2} C:{X:0>2} D:{X:0>2} E:{X:0>2} H:{X:0>2} L:{X:0>2} | Z:{} N:{} H:{} C:{}\n", .{ self.pc -% 1, opcode, self.a, self.b, self.c, self.d, self.e, self.h, self.l, self.f.z, self.f.n, self.f.h, self.f.c });
        return switch (opcode) {
            // ===============================================================================================
            // --- CONTROL/MISC ---
            // ===============================================================================================
            0x00 => {
                return 1;
            }, // NOP
            0xF3 => {
                self.ime = false;
                return 1;
            }, // DI
            0xFB => {
                self.ime = true;
                return 1;
            }, // EI
            0x2F => {
                self.a = ~self.a;
                self.f.n = true;
                self.f.h = true;
                return 1;
            }, // CPL
            0x37 => {
                self.f.n = false;
                self.f.h = false;
                self.f.c = true;
                return 1;
            }, // SCF

            // ===============================================================================================
            // --- 8-BIT ---
            // ===============================================================================================

            // --- 2. 8-Bit Loads (The BIG cleanup) ---
            // Groups all LD r, r' including LD r, (HL) and LD (HL), r
            0x40...0x75, 0x77...0x7F => {
                const src = self.getReg(@truncate(opcode & 0x07));
                self.setReg(@truncate((opcode >> 3) & 0x07), src);
                return if ((opcode & 0x07) == 6 or ((opcode >> 3) & 0x07) == 6) 2 else 1;
            },

            // --- 8-Bit Immediate Loads (LD r, d8) ---
            // Pattern: 0x06, 0x0E, 0x16, 0x1E, 0x26, 0x2E, 0x36, 0x3E
            0x06, 0x0E, 0x16, 0x1E, 0x26, 0x2E, 0x36, 0x3E => {
                const val = self.fetchByte();
                const reg_index: u3 = @truncate(opcode >> 3);
                self.setReg(reg_index, val);
                return if (reg_index == 6) 3 else 2; // (HL) takes 3 cycles, others take 2
            },

            // --- 8-Bit Absolute Loads (LD (nn), A and LD A, (nn)) ---
            0xEA => { // LD (nn), A
                const addr = self.fetchU16();
                self.writeByte(addr, self.a);
                return 4;
            },
            0xFA => { // LD A, (nn)
                const addr = self.fetchU16();
                self.a = self.readByte(addr);
                return 4;
            },

            // --- 8-Bit High RAM Offset Loads (LD (C), A and LD A, (C)) ---
            0xE2 => { // LD ($FF00+C), A
                const addr = 0xFF00 + @as(u16, self.c);
                self.writeByte(addr, self.a);
                return 2;
            },
            0xF2 => { // LD A, ($FF00+C)
                const addr = 0xFF00 + @as(u16, self.c);
                self.a = self.readByte(addr);
                return 2;
            },

            // --- 8-Bit Indirect Loads (Register as Pointer) ---
            0x02 => { // LD (BC), A
                self.writeByte(self.getBC(), self.a);
                return 2;
            },
            0x12 => { // LD (DE), A
                self.writeByte(self.getDE(), self.a);
                return 2;
            },
            0x0A => { // LD A, (BC)
                self.a = self.readByte(self.getBC());
                return 2;
            },
            0x1A => { // LD A, (DE)
                self.a = self.readByte(self.getDE());
                return 2;
            },

            // --- 3. 8-Bit Arithmetic Groups ---
            0x80...0x87 => {
                self.add_a_r8(self.getReg(@truncate(opcode & 0x07)));
                return if ((opcode & 0x07) == 6) 2 else 1;
            },
            0x88...0x8F => |op| {
                const reg_idx: u3 = @truncate(op & 0x7);
                const val = self.getReg(reg_idx);
                self.adc_a_r8(val);
                return if (reg_idx == 6) 2 else 1; // (HL) takes 2 cycles, registers take 1
            },
            0xA0...0xA7 => {
                self.and_a_r8(self.getReg(@truncate(opcode & 0x07)));
                return if ((opcode & 0x07) == 6) 2 else 1;
            },
            0xA8...0xAF => {
                self.xor_a_r8(self.getReg(@truncate(opcode & 0x07)));
                return if ((opcode & 0x07) == 6) 2 else 1;
            },
            0xB0...0xB7 => {
                self.or_a_r8(self.getReg(@truncate(opcode & 0x07)));
                return if ((opcode & 0x07) == 6) 2 else 1;
            },
            0xB8...0xBF => {
                self.sub_a_r8(self.getReg(@truncate(opcode & 0x07)));
                return if ((opcode & 0x07) == 6) 2 else 1;
            },

            // --- 8-Bit Arithmetic Immediate (op A, d8) ---
            0xC6 => {
                self.add_a_r8(self.fetchByte());
                return 2;
            }, // ADD A, d8
            0xCE => { // ADC implementation
                return 2;
            }, // ADC A, d8
            0xD6 => {
                self.sub_a_r8(self.fetchByte());
                return 2;
            }, // SUB d8
            0xDE => { // SBC implementation
                return 2;
            }, // SBC A, d8
            0xE6 => {
                self.and_a_r8(self.fetchByte());
                return 2;
            }, // AND d8
            0xEE => {
                self.xor_a_r8(self.fetchByte());
                return 2;
            }, // XOR d8
            0xF6 => {
                self.or_a_r8(self.fetchByte());
                return 2;
            }, // OR d8
            0xFE => { // CP d8
                const val = self.fetchByte();
                const a = self.a;
                self.f.z = (a == val);
                self.f.n = true;
                self.f.h = (a & 0x0F) < (val & 0x0F);
                self.f.c = a < val;
                return 2;
            },

            // --- 8-Bit HL-Pointer Loads (LD (HL+/-), A and LD A, (HL+/-)) ---
            0x22 => { // LD (HL+), A
                const addr = self.getHL();
                self.writeByte(addr, self.a);
                self.setHL(addr +% 1);
                return 2;
            },
            0x32 => { // LD (HL-), A
                const addr = self.getHL();
                self.writeByte(addr, self.a);
                self.setHL(addr -% 1);
                return 2;
            },
            0x2A => { // LD A, (HL+)
                const addr = self.getHL();
                self.a = self.readByte(addr);
                self.setHL(addr +% 1);
                return 2;
            },
            0x3A => { // LD A, (HL-)
                const addr = self.getHL();
                self.a = self.readByte(addr);
                self.setHL(addr -% 1);
                return 2;
            },

            // --- 8-Bit INC/DEC r8 ---
            0x04,
            0x0C,
            0x14,
            0x1C,
            0x24,
            0x2C,
            0x34,
            0x3C, // INC
            0x05,
            0x0D,
            0x15,
            0x1D,
            0x25,
            0x2D,
            0x35,
            0x3D, // DEC
            => |op| {
                const reg_idx: u3 = @truncate(op >> 3);
                const old_val = self.getReg(reg_idx);
                const is_inc = (op & 0x01) == 0;

                const new_val = if (is_inc) old_val +% 1 else old_val -% 1;
                self.setReg(reg_idx, new_val);

                // Flags
                self.f.z = (new_val == 0);
                self.f.n = !is_inc;
                self.f.h = if (is_inc) (old_val & 0x0F) == 0x0F else (old_val & 0x0F) == 0;
                // Note: Carry flag is NOT affected

                return if (reg_idx == 6) 3 else 1;
            },

            // ===============================================================================================
            // --- 16-BIT ---
            // ===============================================================================================

            // --- 4. 16-Bit Loads / Arithmetic ---
            0x01, 0x11, 0x21, 0x31 => { // LD rr, d16
                const val = self.fetchU16();
                switch (opcode) {
                    0x01 => self.setBC(val),
                    0x11 => self.setDE(val),
                    0x21 => self.setHL(val),
                    0x31 => self.sp = val,
                    else => unreachable,
                }
                return 3;
            },
            0x09, 0x19, 0x29, 0x39 => { // ADD HL, rr
                const hl = self.getHL();
                const rr = switch (opcode) {
                    0x09 => self.getBC(),
                    0x19 => self.getDE(),
                    0x29 => self.getHL(),
                    0x39 => self.sp,
                    else => unreachable,
                };
                const res = hl +% rr;
                self.f.n = false;
                self.f.h = ((hl & 0x0FFF) + (rr & 0x0FFF)) > 0x0FFF;
                self.f.c = (@as(u32, hl) + @as(u32, rr)) > 0xFFFF;
                self.setHL(res);
                return 2;
            },

            // --- 16-Bit INC/DEC ---
            0x03,
            0x13,
            0x23,
            0x33, // INC BC, DE, HL, SP
            0x0B,
            0x1B,
            0x2B,
            0x3B, // DEC BC, DE, HL, SP
            => |op| {
                const is_inc = (op & 0x08) == 0;
                switch (op & 0x30) {
                    0x00 => if (is_inc) self.setBC(self.getBC() +% 1) else self.setBC(self.getBC() -% 1),
                    0x10 => if (is_inc) self.setDE(self.getDE() +% 1) else self.setDE(self.getDE() -% 1),
                    0x20 => if (is_inc) self.setHL(self.getHL() +% 1) else self.setHL(self.getHL() -% 1),
                    0x30 => {
                        if (is_inc) self.sp +%= 1 else self.sp -%= 1;
                    },
                    else => unreachable,
                }
                return 2;
            },

            // --- 16-bit PUSH (BC, DE, HL, AF) ---
            0xC5, 0xD5, 0xE5, 0xF5 => |op| {
                const val = switch (op) {
                    0xC5 => self.getBC(),
                    0xD5 => self.getDE(),
                    0xE5 => self.getHL(),
                    0xF5 => (@as(u16, self.a) << 8) | @as(u16, self.getFlagsAsByte()),
                    else => unreachable,
                };
                self.push(val);
                return 4;
            },

            // --- 16-bit POP (BC, DE, HL, AF) ---
            0xC1, 0xD1, 0xE1, 0xF1 => |op| {
                const val = self.pop();
                switch (op) {
                    0xC1 => self.setBC(val),
                    0xD1 => self.setDE(val),
                    0xE1 => self.setHL(val),
                    0xF1 => {
                        self.a = @truncate(val >> 8);
                        // Crucial: The lower 4 bits of the F register are ALWAYS zero on Game Boy
                        self.setFlagsFromByte(@truncate(val & 0xF0));
                    },
                    else => unreachable,
                }
                return 3;
            },

            // --- 16-Bit Conditional Jumps (JP NZ/Z/NC/C, nn) ---
            0xC2, 0xCA, 0xD2, 0xDA => {
                const dest = self.fetchU16();
                const condition = switch (opcode) {
                    0xC2 => !self.f.z,
                    0xCA => self.f.z,
                    0xD2 => !self.f.c,
                    0xDA => self.f.c,
                    else => unreachable,
                };
                if (condition) {
                    self.pc = dest;
                    return 4;
                }
                return 3;
            },

            // ===============================================================================================
            // --- JUMPS & CALLS ---
            // ===============================================================================================

            0xC3 => {
                self.pc = self.fetchU16();
                return 4;
            }, // JP nn
            0xCD => { // CALL nn
                const dest = self.fetchU16();
                self.push(self.pc);
                self.pc = dest;
                return 6;
            },
            // --- Jump to HL ---
            0xE9 => {
                self.pc = self.getHL();
                return 1;
            },

            // --- Conditional Calls (CALL NZ, Z, NC, C) ---
            0xC4, 0xCC, 0xD4, 0xDC => {
                const dest = self.fetchU16();
                const condition = switch (opcode) {
                    0xC4 => !self.f.z,
                    0xCC => self.f.z,
                    0xD4 => !self.f.c,
                    0xDC => self.f.c,
                    else => unreachable,
                };
                if (condition) {
                    self.push(self.pc);
                    self.pc = dest;
                    return 6;
                }
                return 3;
            },

            // --- Relative Jumps (JR) ---
            0x18 => { // JR s8 (Always jump)
                const offset = @as(i8, @bitCast(self.fetchByte()));
                self.pc = self.pc +% @as(u16, @bitCast(@as(i16, offset)));
                return 3;
            },
            0x20, 0x28, 0x30, 0x38 => {
                const offset = @as(i8, @bitCast(self.fetchByte()));
                const condition = switch (opcode) {
                    0x20 => !self.f.z,
                    0x28 => self.f.z,
                    0x30 => !self.f.c,
                    0x38 => self.f.c,
                    else => unreachable,
                };

                if (condition) {
                    // Correct way to add a signed i8 to a u16 in Zig:
                    const casted_offset = @as(i16, offset);
                    const new_pc = @as(i32, @intCast(self.pc)) + casted_offset;
                    self.pc = @as(u16, @intCast(new_pc));
                    return 3;
                }
                return 2;
            },

            // ===============================================================================================
            // --- RETURNS ---
            // ===============================================================================================

            0xC9 => { // RET (Unconditional)
                self.pc = self.pop();
                return 4;
            },
            0xC0, 0xC8, 0xD0, 0xD8 => { // RET NZ, Z, NC, C
                const condition = switch (opcode) {
                    0xC0 => !self.f.z,
                    0xC8 => self.f.z,
                    0xD0 => !self.f.c,
                    0xD8 => self.f.c,
                    else => unreachable,
                };
                if (condition) {
                    self.pc = self.pop();
                    return 5; // Conditional return takes more cycles if taken
                }
                return 2;
            },
            0xD9 => { // RETI (Return from Interrupt)
                self.pc = self.pop();
                self.ime = true; // Re-enable interrupts
                return 4;
            },

            // ===============================================================================================
            // --- RESTARTS ---
            // ===============================================================================================j

            0xC7, 0xCF, 0xD7, 0xDF, 0xE7, 0xEF, 0xF7, 0xFF => |op| {
                self.push(self.pc);
                // The jump destination is encoded in bits 3, 4, and 5 of the opcode
                self.pc = @as(u16, op & 0x38);
                return 4;
            },

            // ===============================================================================================
            // --- MAIN ROTATES ---
            // ===============================================================================================
            0x07 => { // RLCA
                self.a = self.rlc_logic(self.a);
                self.f.z = false; // IMPORTANT: Main table rotates always clear Z
                return 1;
            },
            0x0F => { // RRCA
                self.a = self.rrc_logic(self.a);
                self.f.z = false;
                return 1;
            },
            0x17 => { // RLA
                self.a = self.rl_logic(self.a);
                self.f.z = false;
                return 1;
            },
            0x1F => { // RRA
                self.a = self.rr_logic(self.a);
                self.f.z = false;
                return 1;
            },

            // ===============================================================================================
            // --- REST/MISC ---
            // ===============================================================================================

            0xE0 => {
                self.writeByte(0xFF00 + @as(u16, self.fetchByte()), self.a);
                return 3;
            }, // LDH (n), A
            0xF0 => {
                const port_byte = self.fetchByte();
                const target_addr = 0xFF00 + @as(u16, port_byte);
                const result = self.readByte(target_addr);

                // NO IF STATEMENT - PRINT EVERYTHING
                std.debug.print("!!! LDH EXECUTION !!! PC: 0x{X:0>4} | Port: 0x{X:0>2} | Val: {d}\n", .{ self.pc - 2, port_byte, result });

                self.a = result;
                return 3;
            },
            0x27 => { // DAA
                self.daa_logic();
                return 1;
            },
            0x76 => { // HALT
                self.halted = true;
                return 1;
            },
            0xCB => self.decodeCB(),

            else => {
                std.debug.print("CRASH: Unknown Opcode 0x{X:0>2} at PC 0x{X:0>4}\n", .{ opcode, self.pc - 1 });
                @panic("Unknown Opcode");
            },
        };
    }

    fn decodeCB(self: *CPU) u8 {
        const cb_opcode = self.fetchByte();
        const bit: u3 = @truncate((cb_opcode >> 3) & 0x7);
        const reg_idx: u3 = @truncate(cb_opcode & 0x7);

        return switch (cb_opcode) {
            // --- 0x40 - 0x7F: BIT n, r ---
            0x40...0x7F => {
                const val = self.getReg(reg_idx);
                self.f.z = (val & (@as(u8, 1) << bit)) == 0;
                self.f.n = false;
                self.f.h = true;
                // Carry flag is not affected
                return if (reg_idx == 6) 3 else 2;
            },

            // --- 0x80 - 0xBF: RES n, r (Reset bit) ---
            0x80...0xBF => {
                var val = self.getReg(reg_idx);
                val &= ~(@as(u8, 1) << bit);
                self.setReg(reg_idx, val);
                return if (reg_idx == 6) 4 else 2;
            },

            // --- 0xC0 - 0xFF: SET n, r (Set bit) ---
            0xC0...0xFF => {
                var val = self.getReg(reg_idx);
                val |= (@as(u8, 1) << bit);
                self.setReg(reg_idx, val);
                return if (reg_idx == 6) 4 else 2;
            },

            // --- 0x00 - 0x3F: Rotates and Shifts ---
            0x00...0x3F => |op| {
                const operation: u3 = @truncate(op >> 3);
                var val = self.getReg(reg_idx);

                switch (operation) {
                    0 => val = self.rlc_logic(val), // RLC
                    1 => val = self.rrc_logic(val), // RRC
                    2 => val = self.rl_logic(val), // RL
                    3 => val = self.rr_logic(val), // RR
                    4 => val = self.sla_logic(val), // SLA
                    5 => val = self.sra_logic(val), // SRA
                    6 => val = self.swap_logic(val), // SWAP
                    7 => val = self.srl_logic(val), // SRL
                }

                self.setReg(reg_idx, val);
                return if (reg_idx == 6) 4 else 2;
            },
        };
    }
    // ===============================================================================================
    // --- Memory Helpers ---
    // ===============================================================================================
    fn readByte(self: *CPU, addr: u16) u8 {
        // if (addr == 0xFF44) {
        //     // Increment every read to simulate time passing
        //     self.fake_ly +%= 1;
        //     // Real GB LY goes up to 153 then resets to 0
        //     if (self.fake_ly > 153) self.fake_ly = 0;
        //     return self.fake_ly;
        // }
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

    // ===============================================================================================
    // --- REGISTER & MEMORY ACCESS ---
    // ===============================================================================================
    fn getBC(self: *CPU) u16 {
        return (@as(u16, self.b) << 8) | self.c;
    }
    fn getDE(self: *CPU) u16 {
        return (@as(u16, self.d) << 8) | self.e;
    }
    fn getHL(self: *CPU) u16 {
        return (@as(u16, self.h) << 8) | self.l;
    }

    fn setBC(self: *CPU, val: u16) void {
        self.b = @truncate(val >> 8);
        self.c = @truncate(val & 0xFF);
    }
    fn setDE(self: *CPU, val: u16) void {
        self.d = @truncate(val >> 8);
        self.e = @truncate(val & 0xFF);
    }
    fn setHL(self: *CPU, val: u16) void {
        self.h = @truncate(val >> 8);
        self.l = @truncate(val & 0xFF);
    }

    fn getReg(self: *CPU, index: u3) u8 {
        return switch (index) {
            0 => self.b,
            1 => self.c,
            2 => self.d,
            3 => self.e,
            4 => self.h,
            5 => self.l,
            6 => self.readByte(self.getHL()),
            7 => self.a,
        };
    }

    fn setReg(self: *CPU, index: u3, val: u8) void {
        switch (index) {
            0 => self.b = val,
            1 => self.c = val,
            2 => self.d = val,
            3 => self.e = val,
            4 => self.h = val,
            5 => self.l = val,
            6 => self.writeByte(self.getHL(), val),
            7 => self.a = val,
        }
    }

    // ===============================================================================================
    // --- STACK OPERATIONS ---
    // ===============================================================================================
    fn push(self: *CPU, value: u16) void {
        self.sp -%= 1;
        self.writeByte(self.sp, @truncate(value >> 8));
        self.sp -%= 1;
        self.writeByte(self.sp, @truncate(value & 0xFF));
    }

    fn pop(self: *CPU) u16 {
        const low = @as(u16, self.readByte(self.sp));
        self.sp +%= 1;
        const high = @as(u16, self.readByte(self.sp));
        self.sp +%= 1;
        return (high << 8) | low;
    }

    fn ret(self: *CPU) void {
        const low = @as(u16, self.readByte(self.sp));
        self.sp +%= 1;
        const high = @as(u16, self.readByte(self.sp));
        self.sp +%= 1;
        self.pc = (high << 8) | low;
    }

    // ===============================================================================================
    // --- FLAG OPERATIONS ---
    // ===============================================================================================
    fn getFlagsAsByte(self: *CPU) u8 {
        var res: u8 = 0;
        if (self.f.z) res |= 0x80;
        if (self.f.n) res |= 0x40;
        if (self.f.h) res |= 0x20;
        if (self.f.c) res |= 0x10;
        return res;
    }

    fn setFlagsFromByte(self: *CPU, byte: u8) void {
        self.f.z = (byte & 0x80) != 0;
        self.f.n = (byte & 0x40) != 0;
        self.f.h = (byte & 0x20) != 0;
        self.f.c = (byte & 0x10) != 0;
    }

    // ===============================================================================================
    // --- 8-BIT ARITHMETIC LOGIC (ALU) ---
    // ===============================================================================================
    fn add_a_r8(self: *CPU, val: u8) void {
        const a = self.a;
        const res = a +% val;
        self.f.z = (res == 0);
        self.f.n = false;
        self.f.h = ((a & 0x0F) + (val & 0x0F)) > 0x0F;
        self.f.c = (@as(u16, a) + @as(u16, val)) > 0xFF;
        self.a = res;
    }

    fn adc_a_r8(self: *CPU, val: u8) void {
        const a = self.a;
        const c: u8 = if (self.f.c) 1 else 0;
        const res = a +% val +% c;
        self.f.z = (res == 0);
        self.f.n = false;
        self.f.h = (a & 0x0F) + (val & 0x0F) + c > 0x0F;
        self.f.c = @as(u16, a) + @as(u16, val) + @as(u16, c) > 0xFF;
        self.a = res;
    }

    fn sub_a_r8(self: *CPU, val: u8) void {
        const a = self.a;
        const res = a -% val;
        self.f.z = (res == 0);
        self.f.n = true;
        self.f.h = (a & 0x0F) < (val & 0x0F);
        self.f.c = a < val;
        self.a = res;
    }

    fn sbc_a_r8(self: *CPU, val: u8) void {
        const a = self.a;
        const c: u8 = if (self.f.c) 1 else 0;
        const res = a -% val -% c;
        self.f.z = (res == 0);
        self.f.n = true;
        self.f.h = @as(i32, a & 0x0F) - @as(i32, val & 0x0F) - @as(i32, c) < 0;
        self.f.c = @as(i32, a) - @as(i32, val) - @as(i32, c) < 0;
        self.a = res;
    }

    fn and_a_r8(self: *CPU, val: u8) void {
        self.a &= val;
        self.f = .{ .z = (self.a == 0), .n = false, .h = true, .c = false };
    }

    fn xor_a_r8(self: *CPU, val: u8) void {
        self.a ^= val;
        self.f = .{ .z = (self.a == 0), .n = false, .h = false, .c = false };
    }

    fn or_a_r8(self: *CPU, val: u8) void {
        self.a |= val;
        self.f = .{ .z = (self.a == 0), .n = false, .h = false, .c = false };
    }

    fn cp_a_r8(self: *CPU, val: u8) void {
        const a = self.a;
        self.f.z = (a == val);
        self.f.n = true;
        self.f.h = (a & 0x0F) < (val & 0x0F);
        self.f.c = a < val;
    }
    fn daa_logic(self: *CPU) void {
        var a = @as(u16, self.a);

        if (!self.f.n) {
            // After an addition, adjust if carry occurred or if digit > 9
            if (self.f.h or (a & 0x0F) > 0x09) {
                a += 0x06;
            }
            if (self.f.c or a > 0x9F) {
                a += 0x60;
                self.f.c = true;
            }
        } else {
            // After a subtraction, adjust if carry occurred
            if (self.f.h) {
                a = (a -% 0x06) & 0xFF;
            }
            if (self.f.c) {
                a = (a -% 0x60) & 0xFF;
            }
        }

        self.a = @truncate(a);
        self.f.z = (self.a == 0);
        self.f.h = false;
    }

    // ===============================================================================================
    // --- BITWISE / SHIFT / ROTATE LOGIC ---
    // ===============================================================================================
    fn rlc_logic(self: *CPU, val: u8) u8 {
        const msb = (val & 0x80) != 0;
        const res = (val << 1) | (if (msb) @as(u8, 1) else 0);
        self.f = .{ .z = (res == 0), .n = false, .h = false, .c = msb };
        return res;
    }

    fn rrc_logic(self: *CPU, val: u8) u8 {
        const lsb = (val & 0x01) != 0;
        const res = (val >> 1) | (if (lsb) @as(u8, 0x80) else 0);
        self.f = .{ .z = (res == 0), .n = false, .h = false, .c = lsb };
        return res;
    }

    fn rl_logic(self: *CPU, val: u8) u8 {
        const msb = (val & 0x80) != 0;
        const res = (val << 1) | (if (self.f.c) @as(u8, 1) else 0);
        self.f = .{ .z = (res == 0), .n = false, .h = false, .c = msb };
        return res;
    }

    fn rr_logic(self: *CPU, val: u8) u8 {
        const lsb = (val & 0x01) != 0;
        const res = (val >> 1) | (if (self.f.c) @as(u8, 0x80) else 0);
        self.f = .{ .z = (res == 0), .n = false, .h = false, .c = lsb };
        return res;
    }

    fn sla_logic(self: *CPU, val: u8) u8 {
        const msb = (val & 0x80) != 0;
        const res = val << 1;
        self.f = .{ .z = (res == 0), .n = false, .h = false, .c = msb };
        return res;
    }

    fn sra_logic(self: *CPU, val: u8) u8 {
        const lsb = (val & 0x01) != 0;
        const res = (val >> 1) | (val & 0x80); // Keep MSB
        self.f = .{ .z = (res == 0), .n = false, .h = false, .c = lsb };
        return res;
    }

    fn srl_logic(self: *CPU, val: u8) u8 {
        const lsb = (val & 0x01) != 0;
        const res = val >> 1;
        self.f = .{ .z = (res == 0), .n = false, .h = false, .c = lsb };
        return res;
    }

    fn swap_logic(self: *CPU, val: u8) u8 {
        const res = (val << 4) | (val >> 4);
        self.f = .{ .z = (res == 0), .n = false, .h = false, .c = false };
        return res;
    }

    fn bit_test(self: *CPU, bit: u3, val: u8) void {
        self.f.z = (val & (@as(u8, 1) << bit)) == 0;
        self.f.n = false;
        self.f.h = true;
    }

    //===============================================================================================
    // Interrupts
    //===============================================================================================
    pub fn handleInterrupts(self: *CPU) void {
        const ie = self.readByte(0xFFFF);
        const if_reg = self.readByte(0xFF0F);
        const pending = ie & if_reg;

        // DEBUG: Only print when a VBlank is requested to avoid flooding
        if (if_reg & 0x01 != 0) {
            std.debug.print("INT CHECK: IME: {}, IE: 0x{X:0>2}, IF: 0x{X:0>2}, Pending: 0x{X:0>2}\n", .{ self.ime, ie, if_reg, pending });
        }

        if (pending > 0) {
            self.halted = false;

            if (!self.ime) return;

            // V-Blank has highest priority (Bit 0)
            if ((pending & 0x01) != 0) {
                std.debug.print("--- SERVICING VBLANK ---\n", .{});
                self.serviceInterrupt(0, 0x0040);
                return; // Only service one interrupt per check
            }

            // LCD Stat (Bit 1)
            if ((pending & 0x02) != 0) {
                self.serviceInterrupt(1, 0x0048);
                return;
            }

            // Timer (Bit 2)
            if ((pending & 0x04) != 0) {
                self.serviceInterrupt(2, 0x0050);
                return;
            }

            // Serial (Bit 3) and Joypad (Bit 4) would follow...
        }
    }

    fn serviceInterrupt(self: *CPU, bit: u3, vector: u16) void {
        self.ime = false;

        // Clear the IF bit
        const if_reg = self.readByte(0xFF0F);
        self.writeByte(0xFF0F, if_reg & ~(@as(u8, 1) << bit));

        // Use your helper! It handles the SP decrement and the two writes
        self.push(self.pc);

        self.pc = vector;
    }
};
