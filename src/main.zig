const std = @import("std");
const gb_emulator = @import("gb_emulator");
const cpu = @import("cpu.zig");
const bus = @import("bus.zig");

pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    var newBus = bus.Bus{};
    var newCpu = cpu.CPU.init(&newBus);

    const file = try std.fs.cwd().openFile("roms/test.gb", .{});
    defer file.close();

    const romData = try file.readToEndAlloc(allocator, 1024 * 1024);
    defer allocator.free(romData);

    newBus.loadROM(romData);
    std.debug.print("ROM Loaded successfully. Starting emulation...\n", .{});

    while (true) {
        const cycles = newCpu.step();

        newBus.ppu.tick(cycles, &newBus.if_flag);

        newCpu.handleInterrupts();
    }
}
