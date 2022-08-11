const std = @import("std");
pub const micro = @import("microzig");
pub const chip = micro.chip;
pub const regs = chip.registers;
pub const irq = chip.irq;

pub const Config = struct {
    led_enabled: bool = true,
    key_enabled: bool = true,
};

pub fn init(cfg: Config) void {
    if (cfg.led_enabled) {
        led.init();
        //  TODO high speed opcija za gpio
        regs.GPIOA.OSPEEDR.modify(.{ .OSPEEDR5 = 0b10 });
    }
    if (cfg.key_enabled) {
        keyInit();
    }
}

pub const pin_map = .{
    .led = "PA5", // on-board user led
    .button = "PC13", // on-board user key
};

pub const led = micro.Gpio(micro.Pin(pin_map.led), .{
    .mode = .output,
    .initial_state = .high,
});

fn keyInit() void {
    regs.RCC.APB2ENR.modify(.{ .SYSCFGEN = 1 }); // Enable SYSCFG Clock
    // init key as input
    micro.Gpio(micro.Pin(pin_map.button), .{ .mode = .input }).init();
    // above micro is same as below regs two lines
    // // PC13 enable input
    // regs.RCC.AHB1ENR.modify(.{ .GPIOCEN = 1 });
    //
    // regs.GPIOC.MODER.modify(.{ .MODER13 = 0b00 });
    // TODO napravi ovo opecenito i objasni zasto mora biti ovdje
    regs.NVIC.IPR10.modify(.{ .IPR_N0 = 0xf0 }); // set interrupt priority

    regs.SYSCFG.EXTICR4.modify(.{ .EXTI13 = 0b0010 }); // pc_13
    regs.EXTI.FTSR.modify(.{ .TR13 = 1 });
    regs.EXTI.IMR.modify(.{ .MR13 = 1 });
    irq.enable(.exti15_10);
}

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
    .m = 2,
    .n = 100,
    .p = 4,
    .q = 8,
    .latency = 3,
    .ahb_prescaler = 1,
    .apb1_prescaler = 2,
    .apb2_prescaler = 1,
    .frequencies = .{
        .source = hse_frequency,
        .cpu = 100_000_000,
        .ahb = 100_000_000,
        .apb1 = 50_000_000,
        .apb2 = 100_000_000,
    },
};

pub const hse_96 = .{
    .source = .hse,
    .m = 2,
    .n = 96,
    .p = 4,
    .q = 8,
    .latency = 3,
    .ahb_prescaler = 1,
    .apb1_prescaler = 2,
    .apb2_prescaler = 1,
    .frequencies = .{
        .source = hse_frequency,
        .cpu = 96_000_000,
        .ahb = 96_000_000,
        .apb1 = 48_000_000,
        .apb2 = 96_000_000,
    },
};
