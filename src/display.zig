const std = @import("std");
const rl = @import("raylib");
const PPU = @import("ppu.zig").PPU;

pub const Display = struct {
    const SCALE = 4; // 4x scale for visibility
    const GB_WIDTH = 160;
    const GB_HEIGHT = 144;
    const WINDOW_WIDTH = GB_WIDTH * SCALE;
    const WINDOW_HEIGHT = GB_HEIGHT * SCALE;

    // Game Boy colors (classic green palette)
    const COLORS = [4]rl.Color{
        rl.Color{ .r = 155, .g = 188, .b = 15, .a = 255 }, // Lightest
        rl.Color{ .r = 139, .g = 172, .b = 15, .a = 255 }, // Light
        rl.Color{ .r = 48, .g = 98, .b = 48, .a = 255 }, // Dark
        rl.Color{ .r = 15, .g = 56, .b = 15, .a = 255 }, // Darkest
    };

    framebuffer: [GB_HEIGHT][GB_WIDTH]u8, // Store color indices (0-3)
    texture: rl.Texture2D,
    image: rl.Image,

    pub fn init() !Display {
        rl.initWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "Game Boy Emulator");
        rl.setTargetFPS(60);

        // Create an image to hold our framebuffer
        const image = rl.genImageColor(GB_WIDTH, GB_HEIGHT, COLORS[0]);
        const texture = try rl.loadTextureFromImage(image);

        return Display{
            .framebuffer = undefined,
            .texture = texture,
            .image = image,
        };
    }

    pub fn deinit(self: *Display) void {
        rl.unloadTexture(self.texture);
        rl.unloadImage(self.image);
        rl.closeWindow();
    }

    pub fn shouldClose(self: *Display) bool {
        _ = self;
        return rl.windowShouldClose();
    }

    // Render the current PPU state to the display
    pub fn render(self: *Display, ppu: *PPU) void {
        // For now, just render background tiles
        self.renderBackground(ppu);

        // Update texture from framebuffer
        self.updateTexture();

        // Draw to screen
        rl.beginDrawing();
        defer rl.endDrawing();

        rl.clearBackground(COLORS[3]);

        // Draw the texture scaled up
        const dest = rl.Rectangle{
            .x = 0,
            .y = 0,
            .width = WINDOW_WIDTH,
            .height = WINDOW_HEIGHT,
        };
        const source = rl.Rectangle{
            .x = 0,
            .y = 0,
            .width = GB_WIDTH,
            .height = GB_HEIGHT,
        };

        rl.drawTexturePro(self.texture, source, dest, rl.Vector2{ .x = 0, .y = 0 }, 0, rl.Color.white);

        // Show FPS
        rl.drawFPS(10, 10);
    }

    fn renderBackground(self: *Display, ppu: *PPU) void {
        // Only render if LCD is on
        if ((ppu.lcdc & 0x80) == 0) {
            // LCD off - fill with white
            for (0..GB_HEIGHT) |y| {
                for (0..GB_WIDTH) |x| {
                    self.framebuffer[y][x] = 0;
                }
            }
            return;
        }

        // Check if background is enabled
        const bg_enabled = (ppu.lcdc & 0x01) != 0;
        if (!bg_enabled) return;

        // Get tile data and map addresses based on LCDC
        const bg_tile_map_base: u16 = if ((ppu.lcdc & 0x08) != 0) 0x1C00 else 0x1800;
        const tile_data_base: u16 = if ((ppu.lcdc & 0x10) != 0) 0x0000 else 0x1000;
        const use_signed = (ppu.lcdc & 0x10) == 0;

        // Render each pixel
        for (0..GB_HEIGHT) |screen_y| {
            for (0..GB_WIDTH) |screen_x| {
                // Apply scroll
                const map_x = (@as(u16, @intCast(screen_x)) + ppu.scx) & 0xFF;
                const map_y = (@as(u16, @intCast(screen_y)) + ppu.scy) & 0xFF;

                // Get tile from tilemap
                const tile_map_x = map_x / 8;
                const tile_map_y = map_y / 8;
                const tile_map_addr = bg_tile_map_base + (tile_map_y * 32) + tile_map_x;
                const tile_index = ppu.vram[tile_map_addr];

                // Get tile data address
                const tile_addr = if (use_signed) blk: {
                    const signed_index = @as(i8, @bitCast(tile_index));
                    const offset = @as(u16, @intCast(@as(i16, signed_index) * 16));
                    break :blk tile_data_base +% offset;
                } else blk: {
                    break :blk tile_data_base + (@as(u16, tile_index) * 16);
                };

                // Get pixel within tile
                const tile_pixel_x = @as(u3, @intCast(map_x % 8));
                const tile_pixel_y = @as(u3, @intCast(map_y % 8));

                // Each tile is 16 bytes (2 bytes per row)
                const tile_row_addr = tile_addr + (@as(u16, tile_pixel_y) * 2);
                const byte1 = ppu.vram[tile_row_addr];
                const byte2 = ppu.vram[tile_row_addr + 1];

                // Get color for this pixel (bit 0 is rightmost pixel)
                const bit_pos = 7 - tile_pixel_x;
                const color_low = (byte1 >> bit_pos) & 1;
                const color_high = (byte2 >> bit_pos) & 1;
                const color_id = (color_high << 1) | color_low;

                self.framebuffer[screen_y][screen_x] = color_id;
            }
        }
    }

    fn updateTexture(self: *Display) void {
        // Convert framebuffer to color data
        var pixels: [GB_WIDTH * GB_HEIGHT]rl.Color = undefined;

        for (0..GB_HEIGHT) |y| {
            for (0..GB_WIDTH) |x| {
                const color_id = self.framebuffer[y][x];
                pixels[y * GB_WIDTH + x] = COLORS[color_id];
            }
        }

        rl.updateTexture(self.texture, &pixels);
    }
};
