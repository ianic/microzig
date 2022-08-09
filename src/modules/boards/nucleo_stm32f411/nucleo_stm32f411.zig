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
    clock: clock.Config = clock.hsi_high,
    led_enabled: bool = true,
    key_enabled: bool = true,
    systick_enabled: bool = true,
};

pub fn init(cfg: Config) void {
    clock.init(cfg.clock);
    //regs.PWR.CR.modify(.{ .VOS = 0b11 });
    //regs.RCC.APB1ENR.modify(.{ .PWREN = 1 });

    if (cfg.led_enabled) {
        led.init();
    }
    if (cfg.key_enabled) {
        keyInit();
    }
    if (cfg.systick_enabled) {
        initSysTick(1);
    }
    initFeatures();
}

pub const pin_map = .{
    .@"LD2" = "PA5", // on-board user led
    .@"KEY1" = "PC13", // on-board user key
};

pub const led = micro.Gpio(micro.Pin(pin_map.LD2), .{
    .mode = .output,
    .initial_state = .high,
});

// board has key connected to the port PA0
// activate key
fn keyInit() void {
    // init key as input
    //micro.Gpio(micro.Pin("PC13"), .{ .mode = .input }).init();
    // above micro is same as below regs two lines

    regs.RCC.APB2ENR.modify(.{ .SYSCFGEN = 1 }); // Enable SYSCFG Clock
    // PC13 enable input
    regs.RCC.AHB1ENR.modify(.{ .GPIOCEN = 1 });
    regs.GPIOC.MODER.modify(.{ .MODER13 = 0b00 });

    //regs.SYSCFG.EXTICR1.modify(.{ .EXTI0 = 1 });
    regs.SYSCFG.EXTICR4.modify(.{ .EXTI13 = 0b0010 }); // pc_13
    //regs.EXTI.RTSR.modify(.{ .TR13 = 1 });
    regs.EXTI.FTSR.modify(.{ .TR13 = 1 });
    regs.EXTI.IMR.modify(.{ .MR13 = 1 });
    //regs.NVIC.ISER0.modify(.{ .SETENA = 0x40 });
    irq.enable(.exti15_10);
}

fn initFeatures() void {
    regs.FPU_CPACR.CPACR.modify(.{ .CP = 0b11 }); // Enable FPU coprocessor
    regs.FLASH.ACR.modify(.{ .DCEN = 1, .ICEN = 1, .PRFTEN = 1 }); // Enable flash data and instruction cache
}

// init SysTick interrupt every ms milliseconds
fn initSysTick(ms: u16) void {
    //const reload: u24 = @intCast(u24, clock.frequencies.ahb / 8) / 1000 * ms - 1;
    const reload: u24 = @intCast(u24, clock.frequencies.ahb / 1000) * ms - 1;
    regs.SCS.SysTick.LOAD.modify(.{ .RELOAD = reload }); // set counter
    regs.SCS.SysTick.VAL.modify(.{ .CURRENT = 0 }); // start from 0
    // CLKSOURCE:
    //   0: AHB/8
    //   1: Processor clock (AHB)
    // TICKINT:
    //   0: Counting down to zero does not assert the SysTick exception request
    //   1: Counting down to zero to asserts the SysTick exception request.
    //
    regs.SCS.SysTick.CTRL.modify(.{
        .ENABLE = 1,
        .CLKSOURCE = 1,
        .TICKINT = 1,
        // .COUNTFLAG = 0,
    });
}
