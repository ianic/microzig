const std = @import("std");
pub const micro = @import("microzig");
pub const chip = micro.chip;
pub const regs = chip.registers;

pub const clock = @import("clock.zig");
pub const irq = chip.irq;

pub const Config = struct {
    clock: clock.Config = clock.hse_high,
    led_enabled: bool = true,
    key_enabled: bool = true,
    systick_enabled: bool = true,
};

pub fn init(cfg: Config) void {
    initFeatures();
    clock.init(cfg.clock);
    if (cfg.key_enabled) {
        keyInit();
    }
    if (cfg.led_enabled) {
        led.init();
    }
    if (cfg.systick_enabled) {
        initSysTick(1);
    }
}

pub const pin_map = .{
    .@"LD1" = "PC13", // on-board user led
    .@"KEY1" = "PA0", // on-board user key
};

pub const led = micro.Gpio(micro.Pin("PC13"), .{
    .mode = .output,
    .initial_state = .high,
});

// board has key connected to the port PA0
// activate key
fn keyInit() void {
    // init key as input
    //micro.Gpio(micro.Pin("PA0"), .{ .mode = .input }).init();
    // above micro is same as below regs two lines

    // PA0 enable input
    regs.RCC.AHB1ENR.modify(.{ .GPIOAEN = 1 });
    regs.GPIOA.MODER.modify(.{ .MODER0 = 0b00 });

    // PA0 interupt enabling
    regs.SYSCFG.EXTICR1.modify(.{ .EXTI0 = 1 });
    regs.EXTI.RTSR.modify(.{ .TR0 = 1 });
    regs.EXTI.FTSR.modify(.{ .TR0 = 0 });
    regs.EXTI.IMR.modify(.{ .MR0 = 1 });
    //regs.NVIC.ISER0.modify(.{ .SETENA = 0x40 });
    irq.enable(.exti0);
}

fn initFeatures() void {
    regs.FPU_CPACR.CPACR.modify(.{ .CP = 0b11 }); // Enable FPU coprocessor
    regs.FLASH.ACR.modify(.{ .DCEN = 1, .ICEN = 1 }); // Enable flash data and instruction cache
}

// init SysTick interrupt every ms milliseconds
fn initSysTick(ms: u16) void {
    const reload: u24 = @intCast(u24, clock.frequencies.ahb / 8) / 1000 * ms - 1;
    regs.SCS.SysTick.LOAD.modify(.{ .RELOAD = reload });

    // CLKSOURCE:
    //   0: AHB/8
    //   1: Processor clock (AHB)
    // TICKINT:
    //   0: Counting down to zero does not assert the SysTick exception request
    //   1: Counting down to zero to asserts the SysTick exception request.
    //
    regs.SCS.SysTick.CTRL.modify(.{ .ENABLE = 1, .CLKSOURCE = 0, .TICKINT = 1 });
}
