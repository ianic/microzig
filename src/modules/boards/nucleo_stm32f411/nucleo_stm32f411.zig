const std = @import("std");
pub const micro = @import("microzig");
pub const chip = micro.chip;
//pub const chip = @import("../../chips/stm32f411/stm32f411re.zig");
pub const gpio = chip.gpio;

pub const Config = struct {
    led_enable: bool = true,
    button_enable: bool = true,
};

//pub fn init(cfg: Config) void {
// if (cfg.led_enable) {
//     led = Led.init(.{});
//     //led.init();
// }
// if (cfg.button_enable) {
//     button.init();
// }
//}

//pub const led = gpio.PA5.Output(.{});
pub const Led = gpio.pa5.Output();
pub const Button = gpio.pc13.Input();

//pub var led: Led = undefined;
//pub const button = gpio.PC13.Input(.{ .irq_enable = true });

// this board uses crystal oscilator from st-link part of the board for hse
pub const hse_frequency = 8_000_000; // 8 MHz

// To use hse put something like this in app init:
//    const micro = @import("microzig");
//    const chip = micro.chip;
//    const board = micro.board;
//    ...
//    const ccfg: chip.Config = .{ .clock = board.hse_96 };
//    chip.init(ccfg);
//
pub const hse_100 = .{
    .source = .hse,
    .pll = .{ .m = 2, .n = 100, .p = 4, .q = 8 },
    .latency = 3,
    .prescaler = .{ .ahb = 1, .apb1 = 2, .apb2 = 1 },
    .frequencies = .{
        .source = hse_frequency,
        .cpu = 100_000_000,
        .ahb = 100_000_000,
        .apb1 = 50_000_000,
        .apb2 = 100_000_000,
        .usb = 50_000_000,
    },
};

pub const hse_96 = .{
    .source = .hse,
    .pll = .{ .m = 2, .n = 96, .p = 4, .q = 8 },
    .latency = 3,
    .prescaler = .{ .ahb = 1, .apb1 = 2, .apb2 = 1 },
    .frequencies = .{
        .source = hse_frequency,
        .cpu = 96_000_000,
        .ahb = 96_000_000,
        .apb1 = 48_000_000,
        .apb2 = 96_000_000,
        .usb = 48_000_000,
    },
};

// TODO how to run tests
// uncommet chip import by path (and comment previous two lines) and then run this: (ubicusemajkemi)
// zig test --main-pkg-path ~/code/zig/embeded/nucleo-stm32f411/lib/microzig/src/ nucleo_stm32f411.zig
test "configs are valid" {
    chip.clk.checkConfig(hse_100);
    chip.clk.checkConfig(hse_96);
}
