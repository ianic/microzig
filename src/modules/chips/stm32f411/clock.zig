// Ref:
// inspired and started as as a copy of: https://github.com/rbino/zig-stm32-blink

const std = @import("std");
const regs = @import("registers.zig").registers;

pub const hsi_frequency = 16_000_000;
pub const max_cpu_frequency = 100_000_000; // chapter 6.3.1 in "DocID026289 Rev 7"
pub const max_apb1_frequency = 50_000_000;

pub const Config = struct {
    source: ClockSource,
    pll: Pll,
    latency: u3, // chapter 3.4
    prescaler: Prescaler,
    frequencies: Frequencies,
};

pub const Pll = struct {
    m: u16,
    n: u16,
    p: u16,
    q: u16,
};

pub const Prescaler = struct { // chapter 6.3.3
    ahb: u16 = 1,
    apb1: u8 = 2,
    apb2: u8 = 1,
};

pub const ClockSource = enum {
    hsi,
    hse,
};

pub const Frequencies = struct {
    source: u32 = hsi_frequency,
    cpu: u32,
    ahb: u32,
    apb1: u32,
    apb2: u32,
    usb: u32,
};

// this ensures that usb has required 48MHz
pub const hsi_96 = .{
    .source = .hsi,
    .pll = .{
        .m = 2,
        .n = 72,
        .p = 6,
        .q = 12,
    },
    .latency = 3,
    .prescaler = .{
        .ahb = 1,
        .apb1 = 2,
        .apb2 = 1,
    },
    .frequencies = .{
        .source = hsi_frequency,
        .cpu = 96_000_000,
        .ahb = 96_000_000,
        .apb1 = 48_000_000,
        .apb2 = 96_000_000,
        .usb = 48_000_000,
    },
};

// to calculate pll parameters use test "show available pll ..."
pub const hsi_100 = .{
    .source = .hsi,
    .pll = .{
        .m = 16,
        .n = 400,
        .p = 4,
        .q = 8,
    },
    .latency = 3,
    .prescaler = .{
        .ahb = 1,
        .apb1 = 2,
        .apb2 = 1,
    },
    .frequencies = .{
        .source = hsi_frequency,
        .cpu = 100_000_000,
        .ahb = 100_000_000,
        .apb1 = 50_000_000,
        .apb2 = 100_000_000,
        .usb = 50_000_000,
    },
};

pub fn init(comptime cfg: Config) void {
    checkConfig(cfg);

    regs.RCC.APB1ENR.modify(.{ .PWREN = 1 }); // enable power interface clock
    regs.PWR.CR.modify(.{ .VOS = voltage_scaling_output(cfg.frequencies.cpu) }); // voltage scaling output

    regs.RCC.CR.modify(.{ .HSION = 1 }); // Enable HSI
    while (regs.RCC.CR.read().HSIRDY != 1) {} // Wait for HSI ready
    regs.RCC.CFGR.modify(.{ .SW0 = 0, .SW1 = 0 }); // Select HSI as clock source

    if (cfg.source == .hse) {
        regs.RCC.CR.modify(.{ .HSEON = 1 }); // Enable external high-speed oscillator (HSE)
        while (regs.RCC.CR.read().HSERDY != 1) {} // Wait for HSE ready
    }
    // set prescalers
    regs.RCC.CFGR.modify(.{
        .HPRE = prescaler_bits(u4, cfg.prescaler.ahb), // AHB prescaler
        .PPRE1 = prescaler_bits(u3, cfg.prescaler.apb1), // APB Low speed prescaler APB1
        .PPRE2 = prescaler_bits(u3, cfg.prescaler.apb2), // APB high-speed prescaler APB2
    });

    initPLL(cfg);
    if (cfg.source == .hse) {
        regs.RCC.CR.modify(.{ .HSION = 0 }); // Disable HSI
    }
}

fn voltage_scaling_output(cpu_freq: u32) u2 {
    return switch (cpu_freq) {
        0...64 => 0b01, // 01: Scale 3 mode <= 64 MHz
        65...84 => 0b10, // 10: Scale 2 mode (reset value) <= 84 MHz
        else => 0b11, // 11: Scale 1 mode <= 100 MHz
    };
}

pub fn checkConfig(comptime cfg: Config) void {
    const pll = cfg.pll;

    // check ranges for pll parameters
    if (!((pll.m >= 2 and pll.m <= 63) and
        (pll.n >= 50 and pll.n <= 432) and
        (pll.p == 2 or pll.p == 4 or pll.p == 6 or pll.p == 8) and
        (pll.q >= 2 and pll.q <= 15)))
    {
        @compileError("wrong RCC PLL configuration values");
    }
    // check range for latency
    if (!(cfg.latency >= 0 and cfg.latency <= 6)) {
        @compileError("wrong latency value");
    }

    // check frequencies calculations and max values
    const fv = cfg.frequencies.source / pll.m * pll.n;
    const cpu = fv / pll.p;
    if (cfg.frequencies.cpu != cpu) {
        @compileError("wrong cpu frequency");
    }
    if (cpu > max_cpu_frequency) {
        @compileError("max cpu frequency exceeded");
    }

    if (cfg.frequencies.ahb != cpu / cfg.prescaler.ahb) {
        @compileError("wrong ahb frequency");
    }
    if (cfg.frequencies.apb1 != cpu / cfg.prescaler.apb1) {
        @compileError("wrong apb1 frequency");
    }
    if (cfg.frequencies.apb1 > max_apb1_frequency) {
        @compileError("max apb1 frequency exceeded");
    }
    if (cfg.frequencies.apb2 != cpu / cfg.prescaler.apb2) {
        @compileError("wrong apb2 frequency");
    }

    const usb = fv / pll.q;
    if (cfg.frequencies.usb != usb) {
        @compileError("wrong usb frequency");
    }

    // depends on clock and voltage range, chapter 3.4 page 45 in RM0383
    const min_latency = switch (cpu / 1_000_000) {
        0...30 => 0,
        31...64 => 1,
        65...90 => 2,
        else => 3,
    };
    if (cfg.latency < min_latency) {
        @compileError("flash memory min latency not satisfied");
    }
}

fn initPLL(comptime cfg: Config) void {
    regs.RCC.CR.modify(.{ .PLLON = 0 }); // Disable PLL before changing its configuration

    const pll = cfg.pll;
    var p0: u1 = if (pll.p == 4 or pll.p == 8) 1 else 0;
    var p1: u1 = if (pll.p == 6 or pll.p == 8) 1 else 0;
    var src: u1 = if (cfg.source == .hse) 1 else 0;
    regs.RCC.PLLCFGR.modify(.{
        .PLLSRC = src,
        // PLLM
        .PLLM0 = bitOf(pll.m, 0),
        .PLLM1 = bitOf(pll.m, 1),
        .PLLM2 = bitOf(pll.m, 2),
        .PLLM3 = bitOf(pll.m, 3),
        .PLLM4 = bitOf(pll.m, 4),
        .PLLM5 = bitOf(pll.m, 5),
        // PLLN
        .PLLN0 = bitOf(pll.n, 0),
        .PLLN1 = bitOf(pll.n, 1),
        .PLLN2 = bitOf(pll.n, 2),
        .PLLN3 = bitOf(pll.n, 3),
        .PLLN4 = bitOf(pll.n, 4),
        .PLLN5 = bitOf(pll.n, 5),
        .PLLN6 = bitOf(pll.n, 6),
        .PLLN7 = bitOf(pll.n, 7),
        .PLLN8 = bitOf(pll.n, 8),
        // PLLP
        .PLLP0 = p0,
        .PLLP1 = p1,
        // PLLQ
        .PLLQ0 = bitOf(pll.q, 0),
        .PLLQ1 = bitOf(pll.q, 1),
        .PLLQ2 = bitOf(pll.q, 2),
        .PLLQ3 = bitOf(pll.q, 3),
    });

    regs.RCC.CR.modify(.{ .PLLON = 1 }); // Enable PLL
    while (regs.RCC.CR.read().PLLRDY != 1) {} // Wait for PLL ready

    regs.FLASH.ACR.modify(.{ .LATENCY = cfg.latency }); // Set flash latency wait states

    regs.RCC.CFGR.modify(.{ .SW1 = 1, .SW0 = 0 }); //Select PLL as clock source

    var cfgr = regs.RCC.CFGR.read(); // Wait for PLL selected as clock source
    while (cfgr.SWS1 != 1 and cfgr.SWS0 != 0) : (cfgr = regs.RCC.CFGR.read()) {}
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

fn prescaler_bits(comptime DestType: type, comptime value: u16) DestType {
    return switch (DestType) {
        u3 => {
            return switch (value) {
                1 => 0b000,
                2 => 0b100,
                4 => 0b101,
                8 => 0b111,
                else => @compileError("invalid prescaler value"),
            };
        },
        u4 => {
            return switch (value) {
                1 => 0b0000,
                2 => 0b1000,
                4 => 0b1001,
                8 => 0b1010,
                16 => 0b1011,
                64 => 0b1100,
                128 => 0b1101,
                256 => 0b1110,
                512 => 0b1111,
                else => @compileError("invalid prescaler value"),
            };
        },
        else => unreachable,
    };
}

test "prescaler_bits" {
    try std.testing.expectEqual(prescaler_bits(u3, 4), 0b101);
    try std.testing.expectEqual(prescaler_bits(u4, 4), 0b1001);

    try std.testing.expectEqual(prescaler_bits(u3, 8), 0b111);
    try std.testing.expectEqual(prescaler_bits(u4, 8), 0b1010);
    try std.testing.expectEqual(prescaler_bits(u4, 256), 0b1110);
}

// init SysTick interrupt every ms milliseconds
pub fn initSysTick(comptime ahb_freq: u32, comptime ms: u16) void {
    const reload: u24 = @intCast(u24, ahb_freq / 8) / 1000 * ms - 1;
    //const reload: u24 = @intCast(u24, ahb_freq / 1000) * ms - 1;  // for clocksource = 1
    regs.SCS.SysTick.LOAD.modify(.{ .RELOAD = reload }); // set counter
    regs.SCB.SHPR3.modify(.{ .PRI_15 = 0xf0 }); // SysTick IRQ priority
    regs.SCS.SysTick.VAL.modify(.{ .CURRENT = 0 }); // start from 0
    regs.SCS.SysTick.CTRL.modify(.{
        .ENABLE = 1,
        .CLKSOURCE = 0, // 0: AHB/8, 1: AHB (processor clock)
        .TICKINT = 1, // 1: Counting down to zero to asserts the SysTick exception request
    });
}

// use this terminal command to view possible pll config
// zig test --test-filter pll clock.zig 2>&1 | grep "^fs" | sort | uniq -f 11
test "show available pll parameters" {
    var pa = [_]u64{ 2, 4, 6, 8 };

    //const source: u64 = 8_000_000; // hse
    const source: u64 = hsi_frequency; // hsi
    const fs_max: u64 = max_cpu_frequency;
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
                        std.debug.print("m: {d:2}, n: {d:3}, p: {d:2}, q: {d}", .{ m, n, p, q });
                        std.debug.print("  {d}\r\n", .{fs});
                    }
                }
            }
        }
    }
}

test "configs are valid" {
    checkConfig(hsi_100);
    checkConfig(hsi_96);
}
