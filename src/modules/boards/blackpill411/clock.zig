const std = @import("std");
pub const micro = @import("microzig");
pub const chip = micro.chip;
pub const regs = chip.registers;

pub const Config = struct {
    m: u16,
    n: u16,
    p: u16,
    q: u16,
    latency: u3, // chapter 3.4
    ahb_prescaler: ahb_prescaler, // chapter 6.3.3
    apb1_prescaler: apb_prescaler,
    apb2_prescaler: apb_prescaler,
    frequencies: Frequencies,
};

pub const hse_frequency: u32 = 25_000_000;

pub var frequencies: Frequencies = .{};

// defaults after restart
pub const Frequencies = struct {
    cpu: u32 = 16_000_000,
    ahb: u32 = 16_000_000,
    apb1: u32 = 16_000_000,
    apb2: u32 = 16_000_000,
};

// everyting to the max, using hse clock
pub const hse_high = .{
    .m = 25,
    .n = 192,
    .p = 2,
    .q = 4,
    .latency = 3,
    .ahb_prescaler = .not_divided,
    .apb1_prescaler = .div_2,
    .apb2_prescaler = .not_divided,
    .frequencies = .{
        .cpu = 96_000_000,
        .ahb = 96_000_000,
        .apb1 = 48_000_000,
        .apb2 = 96_000_000,
    },
};

pub const apb_prescaler = enum(u3) {
    not_divided = 0,
    div_2 = 0b100,
    div_4 = 0b101,
    div_8 = 0b110,
    div_16 = 0b111,
};

pub const ahb_prescaler = enum(u4) {
    not_divided = 0,
    div_2 = 0b1000,
    div_4 = 0b1001,
    div_8 = 0b1010,
    div_16 = 0b1011,
    div_64 = 0b1100,
    div_128 = 0b1101,
    div_256 = 0b1110,
    div_512 = 0b1111,
};

pub fn init(cfg: Config) void {
    regs.RCC.CR.modify(.{ .HSION = 1 }); // Enable HSI
    while (regs.RCC.CR.read().HSIRDY != 1) {} // Wait for HSI ready
    regs.RCC.CFGR.modify(.{ .SW0 = 0, .SW1 = 0 }); // Select HSI as clock source

    regs.RCC.CR.modify(.{ .HSEON = 1 }); // Enable external high-speed oscillator (HSE)
    while (regs.RCC.CR.read().HSERDY != 1) {} // Wait for HSE ready

    regs.RCC.CFGR.modify(.{
        .HPRE = @enumToInt(cfg.ahb_prescaler),
        .PPRE1 = @enumToInt(cfg.apb1_prescaler),
        .PPRE2 = @enumToInt(cfg.apb2_prescaler),
    }); // Set prescalers

    initPLL(cfg);
    frequencies = cfg.frequencies;

    regs.RCC.CR.modify(.{ .HSION = 0 }); // Disable HSI
}

// Fv = source * n / m
// Fsystem = Fv / p
// Fusb = Fv / q  must be e.q. 48Mhz
//
// m 2..63
// n 50..432
// p 2,4,6,8
// q 2..15
//
// hse source = 25Mhz
// Fsystem max = 100Mhz
fn initPLL(cfg: Config) void {
    if (!((cfg.m >= 2 and cfg.m <= 63) and
        (cfg.n >= 50 and cfg.n <= 432) and
        (cfg.p == 2 or cfg.p == 4 or cfg.p == 6 or cfg.p == 8) and
        (cfg.q >= 2 and cfg.q <= 15) and
        (cfg.latency >= 0 and cfg.latency <= 6)))
    {
        // TODO: invalid config
    }

    regs.RCC.CR.modify(.{ .PLLON = 0 }); // Disable PLL before changing its configuration

    var p0: u1 = if (cfg.p == 4 or cfg.p == 8) 1 else 0;
    var p1: u1 = if (cfg.p == 6 or cfg.p == 8) 1 else 0;
    regs.RCC.PLLCFGR.modify(.{
        .PLLSRC = 1,
        // PLLM
        .PLLM0 = bitOf(cfg.m, 0),
        .PLLM1 = bitOf(cfg.m, 1),
        .PLLM2 = bitOf(cfg.m, 2),
        .PLLM3 = bitOf(cfg.m, 3),
        .PLLM4 = bitOf(cfg.m, 4),
        .PLLM5 = bitOf(cfg.m, 5),
        // PLLN
        .PLLN0 = bitOf(cfg.n, 0),
        .PLLN1 = bitOf(cfg.n, 1),
        .PLLN2 = bitOf(cfg.n, 2),
        .PLLN3 = bitOf(cfg.n, 3),
        .PLLN4 = bitOf(cfg.n, 4),
        .PLLN5 = bitOf(cfg.n, 5),
        .PLLN6 = bitOf(cfg.n, 6),
        .PLLN7 = bitOf(cfg.n, 7),
        .PLLN8 = bitOf(cfg.n, 8),
        // PLLP
        .PLLP0 = p0,
        .PLLP1 = p1,
        // PLLQ
        .PLLQ0 = bitOf(cfg.q, 0),
        .PLLQ1 = bitOf(cfg.q, 1),
        .PLLQ2 = bitOf(cfg.q, 2),
        .PLLQ3 = bitOf(cfg.q, 3),
    });

    regs.RCC.CR.modify(.{ .PLLON = 1 }); // Enable PLL
    while (regs.RCC.CR.read().PLLRDY != 1) {} // Wait for PLL ready

    // Set flash latency wait states
    // depends on clock and voltage range, chapter 3.4 page 45
    regs.FLASH.ACR.modify(.{ .LATENCY = cfg.latency });

    regs.RCC.CFGR.modify(.{ .SW1 = 1, .SW0 = 0 }); // // Select PLL as clock source
    // // Wait for PLL selected as clock source
    var cfgr = regs.RCC.CFGR.read();
    while (cfgr.SWS1 != 1 and cfgr.SWS0 != 0) : (cfgr = regs.RCC.CFGR.read()) {}
}

// use this terminal command to view possible pll config
// zig test --test-filter pll blackpill411.zig 2>&1 | grep "^fs" | sort | uniq -f 11
test "show available hse pll parameters" {
    var pa = [_]u64{ 2, 4, 6, 8 };

    const source: u64 = 25_000_000; // hse
    //const source: u64 = 16_000_000; // hsi
    const fs_max: u64 = 100_000_000;
    const fusb: u64 = 48_000_000;

    var m: u64 = 2;
    while (m <= 63) : (m += 1) {
        var n: u64 = 50;
        while (n <= 432) : (n += 1) {
            var q: u64 = 2;
            while (q <= 15) : (q += 1) {
                for (pa) |p| {
                    var fv = source * n / m;
                    var fs = fv / p;
                    var fu = fv / q;
                    if (fu == fusb and fs <= fs_max) {
                        std.debug.print("fs: {d} MHz ", .{fs / 1_000_000});
                        std.debug.print("m: {d:2}, n: {d:3}, q: {d:2}, p: {d}", .{ m, n, q, p });
                        std.debug.print("  {d}\r\n", .{fs});
                    }
                }
            }
        }
    }
}

fn bitOf(x: u16, index: u4) u1 {
    const mask = @as(u16, 1) << index;
    return if (x & mask == mask) 1 else 0;
}

test "is bit set" {
    try std.testing.expectEqual(bitOf(336, 0), 0);
    try std.testing.expectEqual(bitOf(336, 1), 0);
    try std.testing.expectEqual(bitOf(336, 2), 0);
    try std.testing.expectEqual(bitOf(336, 3), 0);
    try std.testing.expectEqual(bitOf(336, 5), 0);
    try std.testing.expectEqual(bitOf(336, 7), 0);

    try std.testing.expectEqual(bitOf(336, 4), 1);
    try std.testing.expectEqual(bitOf(336, 6), 1);
    try std.testing.expectEqual(bitOf(336, 8), 1);
}

// Fv = source * n / m
// Fsystem = Fv / p
// Fusb = Fv / q  must be e.q. 48Mhz
//
// m 2..63
// n 50..432
// p 2,4,6,8
// q 2..15
//
// hse source = 25Mhz
// Fsystem max = 100Mhz
fn setPllCfgr(comptime m: u16, comptime n: u16, comptime p: u16, comptime q: u16, comptime latency: u3) void {
    if (!((m >= 2 and m <= 63) and
        (n >= 50 and n <= 432) and
        (p == 2 or p == 4 or p == 6 or p == 8) and
        (q >= 2 and q <= 15) and
        (latency >= 0 and latency <= 6)))
    {
        @compileError("wrong RCC PLL configuration register values");
    }

    // assuming 25 MHz of external clock
    //@compileLog("setting system clock to [MHz] ", 25 * n / m / p);
    //@compileLog("USB clock to [MHz] ", 25 * n / m / q);

    // Disable PLL before changing its configuration
    regs.RCC.CR.modify(.{ .PLLON = 0 });

    regs.RCC.PLLCFGR.modify(.{
        .PLLSRC = 1,
        // PLLM
        .PLLM0 = bitOf(m, 0),
        .PLLM1 = bitOf(m, 1),
        .PLLM2 = bitOf(m, 2),
        .PLLM3 = bitOf(m, 3),
        .PLLM4 = bitOf(m, 4),
        .PLLM5 = bitOf(m, 5),
        // PLLN
        .PLLN0 = bitOf(n, 0),
        .PLLN1 = bitOf(n, 1),
        .PLLN2 = bitOf(n, 2),
        .PLLN3 = bitOf(n, 3),
        .PLLN4 = bitOf(n, 4),
        .PLLN5 = bitOf(n, 5),
        .PLLN6 = bitOf(n, 6),
        .PLLN7 = bitOf(n, 7),
        .PLLN8 = bitOf(n, 8),
        // PLLP
        .PLLP0 = if (p == 4 or p == 8) 1 else 0,
        .PLLP1 = if (p == 6 or p == 8) 1 else 0,
        // PLLQ
        .PLLQ0 = bitOf(q, 0),
        .PLLQ1 = bitOf(q, 1),
        .PLLQ2 = bitOf(q, 2),
        .PLLQ3 = bitOf(q, 3),
    });
    // Enable PLL
    regs.RCC.CR.modify(.{ .PLLON = 1 });

    // Wait for PLL ready
    while (regs.RCC.CR.read().PLLRDY != 1) {}

    // Set flash latency wait states
    // depends on clock and voltage range, chapter 3.4 page 45
    regs.FLASH.ACR.modify(.{ .LATENCY = latency });

    // // Select PLL as clock source
    regs.RCC.CFGR.modify(.{ .SW1 = 1, .SW0 = 0 });

    // // Wait for PLL selected as clock source
    var cfgr = regs.RCC.CFGR.read();
    while (cfgr.SWS1 != 1 and cfgr.SWS0 != 0) : (cfgr = regs.RCC.CFGR.read()) {}
}
