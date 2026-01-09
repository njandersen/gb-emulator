const std = @import("std");
const gb_emulator = @import("gb_emulator");
const cpu = @import("cpu.zig");
const bus = @import("bus.zig");
const Display = @import("display.zig").Display;

pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    const newBus = try allocator.create(bus.Bus);
    newBus.* = bus.Bus.init();
    defer allocator.destroy(newBus);
    var newCpu = cpu.CPU.init(newBus);

    const file = try std.fs.cwd().openFile("roms/test.gb", .{});
    defer file.close();

    const romData = try file.readToEndAlloc(allocator, 1024 * 1024);
    defer allocator.free(romData);

    newBus.loadROM(romData);

    var display = try Display.init();
    defer display.deinit();

    std.debug.print("ROM Loaded successfully. Starting emulation...\n", .{});

    // Remove most debug prints for performance!
    while (!display.shouldClose()) {
        // Run multiple cycles per frame for better performance
        var cycles_this_frame: u32 = 0;
        const cycles_per_frame = 70224; // ~60 FPS

        while (cycles_this_frame < cycles_per_frame) {
            const cycles = newCpu.step();
            newBus.ppu.tick(cycles, &newBus.if_flag);
            newCpu.handleInterrupts();
            cycles_this_frame += cycles * 4; // Convert M-cycles to T-cycles
        }

        display.render(&newBus.ppu);
    }
}
