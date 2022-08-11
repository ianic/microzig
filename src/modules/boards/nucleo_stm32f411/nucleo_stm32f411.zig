const std = @import("std");
pub const micro = @import("microzig");
pub const chip = micro.chip;
pub const regs = chip.registers;
pub const clock = @import("clock.zig");
pub const irq = chip.irq;

// // HSI defaults after restart
// const cpu_clock = 84_000_000;
// pub const clock_frequencies = .{
//     .cpu = cpu_clock,
//     .ahb = cpu_clock / 1,
//     .apb1 = cpu_clock / 2,
//     .apb2 = cpu_clock / 1,
// };

pub const Config = struct {
    //clock: clock.Config = clock.hsi_high,
    led_enabled: bool = true,
    key_enabled: bool = true,
    //systick_enabled: bool = true,
};

pub fn init(cfg: Config) void {
    //initFeatures();
    // TODO: clock spada u chip, ovdje ostaju hsi i hse frekvencije
    // zapravo samo hse
    //clock.init(cfg.clock);
    // if (cfg.systick_enabled) {
    //     initSysTick(1);
    // }
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
    .@"LD2" = "PA5", // on-board user led
    .@"KEY1" = "PC13", // on-board user key
};

pub const led = micro.Gpio(micro.Pin(pin_map.LD2), .{
    .mode = .output,
    .initial_state = .high,
});

fn keyInit() void {
    regs.RCC.APB2ENR.modify(.{ .SYSCFGEN = 1 }); // Enable SYSCFG Clock
    // init key as input
    micro.Gpio(micro.Pin(pin_map.KEY1), .{ .mode = .input }).init();
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

// fn initFeatures() void {
//     regs.FLASH.ACR.modify(.{ .DCEN = 1, .ICEN = 1, .PRFTEN = 1 }); // Enable flash data and instruction cache
//     regs.SCB.AIRCR.modify(.{ .PRIGROUP = 0b011, .VECTKEYSTAT = 0x5FA }); // Set Interrupt Group Priority
//     regs.FPU_CPACR.CPACR.modify(.{ .CP = 0b1111 }); // Enable FPU coprocessor
// }

// // init SysTick interrupt every ms milliseconds
// fn initSysTick(ms: u16) void {
//     const reload: u24 = @intCast(u24, clock.frequencies.ahb / 8) / 1000 * ms - 1;
//     //const reload: u24 = @intCast(u24, clock.frequencies.ahb / 1000) * ms - 1;  // for clocksource = 1
//     regs.SCS.SysTick.LOAD.modify(.{ .RELOAD = reload }); // set counter
//     regs.SCB.SHPR3.modify(.{ .PRI_15 = 0xf0 }); // SysTick IRQ priority
//     regs.SCS.SysTick.VAL.modify(.{ .CURRENT = 0 }); // start from 0
//     // CLKSOURCE:
//     //   0: AHB/8
//     //   1: Processor clock (AHB)
//     // TICKINT:
//     //   0: Counting down to zero does not assert the SysTick exception request
//     //   1: Counting down to zero to asserts the SysTick exception request.
//     //
//     regs.SCS.SysTick.CTRL.modify(.{
//         .ENABLE = 1,
//         .CLKSOURCE = 0,
//         .TICKINT = 1,
//     });
// }

pub const hse_frequency = 8_000_000; // 8MHz

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
