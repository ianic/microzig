const std = @import("std");

const rzig = @import("registers.zig");
pub usingnamespace rzig; // re-export VectorTable and InterruptVector for microzig
pub const regs = rzig.registers;

pub const clk = @import("clock.zig").Chip(chip_frequencies); // clock hal initialized with chip frequencies

//pub const irq = @import("irq.zig");
pub const adc = @import("adc.zig");

pub const gpio = @import("stm32f411re_gpio.zig").gpio;

pub const Config = struct {
    clock: clk.Config = hsi_100,
    systick_enabled: bool = true,

    // Prigroup determines the split of group priority (preemption priority) from sub-priority.
    // Refer to page 229 in "PM0214 Rev 10" for possible values.
    // 0b0xx => 16 group / 0 sub priorities,
    // 0b100 => 8/2,
    // 0b101 => 4/4,
    // 0b110 => 2/8,
    // 0b111 => 0/16
    prigroup: u3 = 0b000,
};

pub fn init(comptime cfg: Config) void {
    initFeatures(cfg);

    clk.init(cfg.clock);
    // frequencies = cfg.clock.frequencies;
    if (cfg.systick_enabled) {
        clk.initSysTick(cfg.clock.frequencies.ahb, 1);
    }
}

// pub var frequencies: clk.Frequencies = undefined;

fn initFeatures(comptime cfg: Config) void {
    regs.FPU_CPACR.CPACR.modify(.{ .CP = 0b1111 }); // Enable FPU coprocessor
    regs.FLASH.ACR.modify(.{ .DCEN = 1, .ICEN = 1, .PRFTEN = 1 }); // Enable flash data and instruction cache

    // On writes, write 0x5FA to VECTKEY, otherwise the write is ignored.
    regs.SCB.AIRCR.modify(.{ .PRIGROUP = cfg.prigroup, .VECTKEYSTAT = 0x5FA });
}

// -------------- clock configuration
// chip clock frequency operating conditions
const chip_frequencies = struct {
    pub const hsi = 16_000_000; // chapter 6.3.9 in "DocID026289 Rev 7"
    pub const max_cpu = 100_000_000; // chapter 6.3.1 in "DocID026289 Rev 7"
    pub const max_apb1 = 50_000_000;
};

// max frequency using hsi as source
pub const hsi_100 = .{
    .source = .hsi,
    .pll = .{ .m = 16, .n = 400, .p = 4, .q = 8 },
    .latency = 3,
    .prescaler = .{ .ahb = 1, .apb1 = 2, .apb2 = 1 },
    .frequencies = .{
        .source = chip_frequencies.hsi,
        .cpu = 100_000_000,
        .ahb = 100_000_000,
        .apb1 = 50_000_000,
        .apb2 = 100_000_000,
        .usb = 50_000_000,
    },
};

// this configuration ensures that usb has required 48MHz
pub const hsi_96 = .{
    .source = .hsi,
    .pll = .{ .m = 2, .n = 72, .p = 6, .q = 12 },
    .latency = 3,
    .prescaler = .{ .ahb = 1, .apb1 = 2, .apb2 = 1 },
    .frequencies = .{
        .source = chip_frequencies.hsi,
        .cpu = 96_000_000,
        .ahb = 96_000_000,
        .apb1 = 48_000_000,
        .apb2 = 96_000_000,
        .usb = 48_000_000,
    },
};

test "configs are valid" {
    clk.checkConfig(hsi_100);
    clk.checkConfig(hsi_96);
}
// -------------- clock configuration

// -------------- tick counter
pub fn ticker() Ticker {
    return .{ .ticks = 0 };
}

pub const Ticker = struct {
    ticks: u32 = 0,

    const Self = @This();
    pub fn inc(self: *Self) void {
        _ = @addWithOverflow(u32, self.ticks, 1, &self.ticks);
    }

    pub fn interval(self: *Self, delay: u32) TickerInterval {
        var tc = TickerInterval{
            .ticks = &self.ticks,
        };
        tc.set(self.ticks, delay);
        return tc;
    }
};

pub const TickerInterval = struct {
    ticks: *u32 = undefined,
    next: u32 = 0,
    overflow: bool = false,

    const Self = @This();

    pub fn ready(self: *Self, delay: u32) bool {
        var current = self.ticks.*;
        if (self.overflow and current > 0x7FFFFFFF) {
            return false;
        }
        if (current >= self.next) {
            self.set(current, delay);
            return true;
        }
        return false;
    }

    fn set(self: *Self, current: u32, delay: u32) void {
        self.overflow = @addWithOverflow(u32, current, delay, &self.next);
    }
};

test "ticker overflow" {
    var t = ticker();
    t.ticks = 0xFFFFFFFF;
    t.inc();
    try std.testing.expectEqual(t.ticks, 0);
}

test "counter with overflow" {
    var t = ticker();
    t.ticks = 0xFFFFFFFF;

    var counter = t.interval(2);
    try std.testing.expect(!counter.ready(2));
    t.inc();
    try std.testing.expect(!counter.ready(2));
    try std.testing.expectEqual(t.ticks, 0);
    try std.testing.expectEqual(counter.next, 1);
    try std.testing.expectEqual(counter.overflow, true);
    t.inc();
    try std.testing.expect(counter.ready(2));
    try std.testing.expectEqual(counter.next, 3);
    try std.testing.expectEqual(counter.overflow, false);
}
// -------------- tick counter

// -------------- event loop
pub fn tasks(comptime no: u32, ticks_ptr: *u32) [no]Task {
    var a = [_]Task{Task{ .ticks = ticks_ptr }} ** 2;
    return a;
}

pub fn scheduler(tsks: []Task, ticks_ptr: *u32) Scheduler {
    return Scheduler.init(tsks, ticks_ptr);
}

pub const Scheduler = struct {
    tasks: []Task = undefined,
    ticks: *u32 = undefined,

    const Self = @This();

    pub fn init(tsks: []Task, ticks_ptr: *u32) Self {
        return .{ .tasks = tsks, .ticks = ticks_ptr };
    }

    pub fn getTask(self: *Self, no: u8) *Task {
        var task = &self.tasks[no];
        task.ticks = self.ticks;
        return task;
    }

    pub fn tick(self: *Self) void {
        // TODO: handle overrun in ticks
        //self.ticks += 1;
        self.run();
    }

    pub fn run(self: *Self) void {
        var i: u8 = 0;
        while (i < self.tasks.len) : (i += 1) {
            var task = &self.tasks[i];
            task.try_run();
        }
    }
};

pub const Task = struct {
    ticks: *u32 = undefined,
    frame: anyframe = undefined,
    resume_at: u32 = 0,

    const Self = @This();

    fn try_run(self: *Self) void {
        if (self.frame == undefined or
            self.resume_at == 0 or
            self.resume_at > self.ticks.*)
        {
            return;
        }

        var fr = self.frame;
        self.resume_at = 0;
        self.frame = undefined;
        resume fr;
    }

    pub fn sleep(self: *Self, ms: u32) void {
        suspend {
            self.resume_at = self.ticks.* + ms;
            self.frame = @frame();
        }
    }
};
// -------------- event loop

// // -------------- gpio
// pub const Pin = @import("gpio.zig").Pin;
// pub const Pin2 = @import("gpio.zig").Pin2;

// // Definition of available pins for stm32f411.
// pub const gpio = struct {
//     // pub const pa5 = struct {
//     //     const pin = Pin2("PA5");
//     //     pub const Output = pin.Output;
//     // };

//     // pub const pc13 = struct {
//     //     const pin = Pin2("PC13");
//     //     pub const Input = pin.Input;
//     // };

//     pub const PA0 = Pin("PA0"); // ADC1.IN0, TIM2.CH1(1), TIM2.ETR(1), TIM5.CH1(2), USART2.CTS(7),
//     pub const PA1 = Pin("PA1"); // ADC1.IN1, SPI4.MOSI(5), TIM2.CH2(1), TIM5.CH2(2), USART2.RTS(7),
//     pub const PA2 = Pin("PA2"); // ADC1.IN2, TIM2.CH3(1), TIM5.CH3(2), TIM9.CH1(3), USART2.TX(7),
//     pub const PA3 = Pin("PA3"); // ADC1.IN3, TIM2.CH4(1), TIM5.CH4(2), TIM9.CH2(3), USART2.RX(7),
//     pub const PA4 = Pin("PA4"); // ADC1.IN4, SPI1.NSS(5), SPI3.NSS(6), USART2.CK(7),
//     pub const PA5 = Pin("PA5"); // ADC1.IN5, SPI1.SCK(5), TIM2.CH1(1), TIM2.ETR(1),
//     pub const PA6 = Pin("PA6"); // ADC1.IN6, SDIO.CMD(12), SPI1.MISO(5), TIM1.BKIN(1), TIM3.CH1(2),
//     pub const PA7 = Pin("PA7"); // ADC1.IN7, SPI1.MOSI(5), TIM1.CH1N(1), TIM3.CH2(2),
//     pub const PA8 = Pin("PA8"); // I2C3.SCL(4), RCC.MCO_1, SDIO.D1(12), TIM1.CH1(1), USART1.CK(7), USB_OTG_FS.SOF(10),
//     pub const PA9 = Pin("PA9"); // I2C3.SMBA(4), SDIO.D2(12), TIM1.CH2(1), USART1.TX(7), USB_OTG_FS.VBUS(10),
//     pub const PA10 = Pin("PA10"); // SPI5.MOSI(6), TIM1.CH3(1), USART1.RX(7), USB_OTG_FS.ID(10),
//     pub const PA11 = Pin("PA11"); // SPI4.MISO(6), TIM1.CH4(1), USART1.CTS(7), USART6.TX(8), USB_OTG_FS.DM(10),
//     pub const PA12 = Pin("PA12"); // SPI5.MISO(6), TIM1.ETR(1), USART1.RTS(7), USART6.RX(8), USB_OTG_FS.DP(10),
//     pub const PA13 = Pin("PA13"); //
//     pub const PA14 = Pin("PA14"); //
//     pub const PA15 = Pin("PA15"); // SPI1.NSS(5), SPI3.NSS(6), TIM2.CH1(1), TIM2.ETR(1), USART1.TX(7),
//     pub const PB0 = Pin("PB0"); // ADC1.IN8, SPI5.SCK(6), TIM1.CH2N(1), TIM3.CH3(2),
//     pub const PB1 = Pin("PB1"); // ADC1.IN9, SPI5.NSS(6), TIM1.CH3N(1), TIM3.CH4(2),
//     pub const PB2 = Pin("PB2"); //
//     pub const PB3 = Pin("PB3"); // I2C2.SDA(9), SPI1.SCK(5), SPI3.SCK(6), TIM2.CH2(1), USART1.RX(7),
//     pub const PB4 = Pin("PB4"); // I2C3.SDA(9), SDIO.D0(12), SPI1.MISO(5), SPI3.MISO(6), TIM3.CH1(2),
//     pub const PB5 = Pin("PB5"); // I2C1.SMBA(4), SDIO.D3(12), SPI1.MOSI(5), SPI3.MOSI(6), TIM3.CH2(2),
//     pub const PB6 = Pin("PB6"); // I2C1.SCL(4), TIM4.CH1(2), USART1.TX(7),
//     pub const PB7 = Pin("PB7"); // I2C1.SDA(4), SDIO.D0(12), TIM4.CH2(2), USART1.RX(7),
//     pub const PB8 = Pin("PB8"); // I2C1.SCL(4), I2C3.SDA(9), SDIO.D4(12), SPI5.MOSI(6), TIM10.CH1(3), TIM4.CH3(2),
//     pub const PB9 = Pin("PB9"); // I2C1.SDA(4), I2C2.SDA(9), SDIO.D5(12), SPI2.NSS(5), TIM11.CH1(3), TIM4.CH4(2),
//     pub const PB10 = Pin("PB10"); // I2C2.SCL(4), SDIO.D7(12), SPI2.SCK(5), TIM2.CH3(1),
//     pub const PB12 = Pin("PB12"); // I2C2.SMBA(4), SPI2.NSS(5), SPI3.SCK(7), SPI4.NSS(6), TIM1.BKIN(1),
//     pub const PB13 = Pin("PB13"); // SPI2.SCK(5), SPI4.SCK(6), TIM1.CH1N(1),
//     pub const PB14 = Pin("PB14"); // SDIO.D6(12), SPI2.MISO(5), TIM1.CH2N(1),
//     pub const PB15 = Pin("PB15"); // RTC.REFIN, SDIO.CK(12), SPI2.MOSI(5), TIM1.CH3N(1),
//     pub const PC0 = Pin("PC0"); // ADC1.IN10,
//     pub const PC1 = Pin("PC1"); // ADC1.IN11,
//     pub const PC2 = Pin("PC2"); // ADC1.IN12, SPI2.MISO(5),
//     pub const PC3 = Pin("PC3"); // ADC1.IN13, SPI2.MOSI(5),
//     pub const PC4 = Pin("PC4"); // ADC1.IN14,
//     pub const PC5 = Pin("PC5"); // ADC1.IN15,
//     pub const PC6 = Pin("PC6"); // SDIO.D6(12), TIM3.CH1(2), USART6.TX(8),
//     pub const PC7 = Pin("PC7"); // SDIO.D7(12), SPI2.SCK(5), TIM3.CH2(2), USART6.RX(8),
//     pub const PC8 = Pin("PC8"); // SDIO.D0(12), TIM3.CH3(2), USART6.CK(8),
//     pub const PC9 = Pin("PC9"); // I2C3.SDA(4), RCC.MCO_2, SDIO.D1(12), TIM3.CH4(2),
//     pub const PC10 = Pin("PC10"); // SDIO.D2(12), SPI3.SCK(6),
//     pub const PC11 = Pin("PC11"); // SDIO.D3(12), SPI3.MISO(6),
//     pub const PC12 = Pin("PC12"); // SDIO.CK(12), SPI3.MOSI(6),
//     pub const PC13 = Pin("PC13"); // RTC.AF1,
//     pub const PC14 = Pin("PC14"); // RCC.OSC32_IN,
//     pub const PC15 = Pin("PC15"); // RCC.OSC32_OUT,
//     pub const PD2 = Pin("PD2"); // SDIO.CMD(12), TIM3.ETR(2),
//     pub const PH0 = Pin("PH0"); // RCC.OSC_IN,
//     pub const PH1 = Pin("PH1"); // RCC.OSC_OUT,
//     pub const I2C1 = struct {
//         pub const SMBA = struct {
//             pub const PB5 = Pin("PB5").AlternateFunction(.{ .af = 4 });
//         };
//         pub const SCL = struct {
//             pub const PB6 = Pin("PB6").AlternateFunction(.{ .af = 4 });
//             pub const PB8 = Pin("PB8").AlternateFunction(.{ .af = 4 });
//         };
//         pub const SDA = struct {
//             pub const PB7 = Pin("PB7").AlternateFunction(.{ .af = 4 });
//             pub const PB9 = Pin("PB9").AlternateFunction(.{ .af = 4 });
//         };
//     };
//     pub const I2C2 = struct {
//         pub const SDA = struct {
//             pub const PB3 = Pin("PB3").AlternateFunction(.{ .af = 9 });
//             pub const PB9 = Pin("PB9").AlternateFunction(.{ .af = 9 });
//         };
//     };
//     pub const I2C3 = struct {
//         pub const SCL = struct {
//             pub const PA8 = Pin("PA8").AlternateFunction(.{ .af = 4 });
//         };
//         pub const SMBA = struct {
//             pub const PA9 = Pin("PA9").AlternateFunction(.{ .af = 4 });
//         };
//         pub const SDA = struct {
//             pub const PB4 = Pin("PB4").AlternateFunction(.{ .af = 9 });
//             pub const PB8 = Pin("PB8").AlternateFunction(.{ .af = 9 });
//             pub const PC9 = Pin("PC9").AlternateFunction(.{ .af = 4 });
//         };
//     };
//     pub const SDIO = struct {
//         pub const CMD = struct {
//             pub const PA6 = Pin("PA6").AlternateFunction(.{ .af = 12 });
//             pub const PD2 = Pin("PD2").AlternateFunction(.{ .af = 12 });
//         };
//         pub const D1 = struct {
//             pub const PA8 = Pin("PA8").AlternateFunction(.{ .af = 12 });
//             pub const PC9 = Pin("PC9").AlternateFunction(.{ .af = 12 });
//         };
//         pub const D2 = struct {
//             pub const PA9 = Pin("PA9").AlternateFunction(.{ .af = 12 });
//             pub const PC10 = Pin("PC10").AlternateFunction(.{ .af = 12 });
//         };
//         pub const D0 = struct {
//             pub const PB4 = Pin("PB4").AlternateFunction(.{ .af = 12 });
//             pub const PB7 = Pin("PB7").AlternateFunction(.{ .af = 12 });
//             pub const PC8 = Pin("PC8").AlternateFunction(.{ .af = 12 });
//         };
//         pub const D3 = struct {
//             pub const PB5 = Pin("PB5").AlternateFunction(.{ .af = 12 });
//             pub const PC11 = Pin("PC11").AlternateFunction(.{ .af = 12 });
//         };
//     };
//     pub const SPI1 = struct {
//         pub const NSS = struct {
//             pub const PA4 = Pin("PA4").AlternateFunction(.{ .af = 5 });
//             pub const PA15 = Pin("PA15").AlternateFunction(.{ .af = 5 });
//         };
//         pub const SCK = struct {
//             pub const PA5 = Pin("PA5").AlternateFunction(.{ .af = 5 });
//             pub const PB3 = Pin("PB3").AlternateFunction(.{ .af = 5 });
//         };
//         pub const MISO = struct {
//             pub const PA6 = Pin("PA6").AlternateFunction(.{ .af = 5 });
//             pub const PB4 = Pin("PB4").AlternateFunction(.{ .af = 5 });
//         };
//         pub const MOSI = struct {
//             pub const PA7 = Pin("PA7").AlternateFunction(.{ .af = 5 });
//             pub const PB5 = Pin("PB5").AlternateFunction(.{ .af = 5 });
//         };
//     };
//     pub const SPI2 = struct {
//         pub const NSS = struct {
//             pub const PB9 = Pin("PB9").AlternateFunction(.{ .af = 5 });
//             pub const PB12 = Pin("PB12").AlternateFunction(.{ .af = 5 });
//         };
//         pub const SCK = struct {
//             pub const PB10 = Pin("PB10").AlternateFunction(.{ .af = 5 });
//             pub const PB13 = Pin("PB13").AlternateFunction(.{ .af = 5 });
//             pub const PC7 = Pin("PC7").AlternateFunction(.{ .af = 5 });
//         };
//     };
//     pub const SPI3 = struct {
//         pub const NSS = struct {
//             pub const PA4 = Pin("PA4").AlternateFunction(.{ .af = 6 });
//             pub const PA15 = Pin("PA15").AlternateFunction(.{ .af = 6 });
//         };
//     };
//     pub const SPI4 = struct {
//         pub const MOSI = struct {
//             pub const PA1 = Pin("PA1").AlternateFunction(.{ .af = 5 });
//         };
//         pub const MISO = struct {
//             pub const PA11 = Pin("PA11").AlternateFunction(.{ .af = 6 });
//         };
//         pub const NSS = struct {
//             pub const PB12 = Pin("PB12").AlternateFunction(.{ .af = 6 });
//         };
//         pub const SCK = struct {
//             pub const PB13 = Pin("PB13").AlternateFunction(.{ .af = 6 });
//         };
//     };
//     pub const SPI5 = struct {
//         pub const MOSI = struct {
//             pub const PA10 = Pin("PA10").AlternateFunction(.{ .af = 6 });
//             pub const PB8 = Pin("PB8").AlternateFunction(.{ .af = 6 });
//         };
//         pub const MISO = struct {
//             pub const PA12 = Pin("PA12").AlternateFunction(.{ .af = 6 });
//         };
//         pub const SCK = struct {
//             pub const PB0 = Pin("PB0").AlternateFunction(.{ .af = 6 });
//         };
//         pub const NSS = struct {
//             pub const PB1 = Pin("PB1").AlternateFunction(.{ .af = 6 });
//         };
//     };
//     pub const TIM1 = struct {
//         pub const BKIN = struct {
//             pub const PA6 = Pin("PA6").AlternateFunction(.{ .af = 1 });
//             pub const PB12 = Pin("PB12").AlternateFunction(.{ .af = 1 });
//         };
//         pub const CH1N = struct {
//             pub const PA7 = Pin("PA7").AlternateFunction(.{ .af = 1 });
//             pub const PB13 = Pin("PB13").AlternateFunction(.{ .af = 1 });
//         };
//         pub const CH1 = struct {
//             pub const PA8 = Pin("PA8").AlternateFunction(.{ .af = 1 });
//         };
//         pub const CH2 = struct {
//             pub const PA9 = Pin("PA9").AlternateFunction(.{ .af = 1 });
//         };
//         pub const CH3 = struct {
//             pub const PA10 = Pin("PA10").AlternateFunction(.{ .af = 1 });
//         };
//         pub const CH4 = struct {
//             pub const PA11 = Pin("PA11").AlternateFunction(.{ .af = 1 });
//         };
//         pub const ETR = struct {
//             pub const PA12 = Pin("PA12").AlternateFunction(.{ .af = 1 });
//         };
//         pub const CH2N = struct {
//             pub const PB0 = Pin("PB0").AlternateFunction(.{ .af = 1 });
//             pub const PB14 = Pin("PB14").AlternateFunction(.{ .af = 1 });
//         };
//         pub const CH3N = struct {
//             pub const PB1 = Pin("PB1").AlternateFunction(.{ .af = 1 });
//             pub const PB15 = Pin("PB15").AlternateFunction(.{ .af = 1 });
//         };
//     };
//     pub const TIM10 = struct {
//         pub const CH1 = struct {
//             pub const PB8 = Pin("PB8").AlternateFunction(.{ .af = 3 });
//         };
//     };
//     pub const TIM11 = struct {
//         pub const CH1 = struct {
//             pub const PB9 = Pin("PB9").AlternateFunction(.{ .af = 3 });
//         };
//     };
//     pub const TIM2 = struct {
//         pub const CH1 = struct {
//             pub const PA0 = Pin("PA0").AlternateFunction(.{ .af = 1 });
//             pub const PA5 = Pin("PA5").AlternateFunction(.{ .af = 1 });
//             pub const PA15 = Pin("PA15").AlternateFunction(.{ .af = 1 });
//         };
//         pub const ETR = struct {
//             pub const PA0 = Pin("PA0").AlternateFunction(.{ .af = 1 });
//             pub const PA5 = Pin("PA5").AlternateFunction(.{ .af = 1 });
//             pub const PA15 = Pin("PA15").AlternateFunction(.{ .af = 1 });
//         };
//         pub const CH2 = struct {
//             pub const PA1 = Pin("PA1").AlternateFunction(.{ .af = 1 });
//             pub const PB3 = Pin("PB3").AlternateFunction(.{ .af = 1 });
//         };
//         pub const CH3 = struct {
//             pub const PA2 = Pin("PA2").AlternateFunction(.{ .af = 1 });
//             pub const PB10 = Pin("PB10").AlternateFunction(.{ .af = 1 });
//         };
//         pub const CH4 = struct {
//             pub const PA3 = Pin("PA3").AlternateFunction(.{ .af = 1 });
//         };
//     };
//     pub const TIM3 = struct {
//         pub const CH1 = struct {
//             pub const PA6 = Pin("PA6").AlternateFunction(.{ .af = 2 });
//             pub const PB4 = Pin("PB4").AlternateFunction(.{ .af = 2 });
//             pub const PC6 = Pin("PC6").AlternateFunction(.{ .af = 2 });
//         };
//         pub const CH2 = struct {
//             pub const PA7 = Pin("PA7").AlternateFunction(.{ .af = 2 });
//             pub const PB5 = Pin("PB5").AlternateFunction(.{ .af = 2 });
//             pub const PC7 = Pin("PC7").AlternateFunction(.{ .af = 2 });
//         };
//         pub const CH3 = struct {
//             pub const PB0 = Pin("PB0").AlternateFunction(.{ .af = 2 });
//             pub const PC8 = Pin("PC8").AlternateFunction(.{ .af = 2 });
//         };
//         pub const CH4 = struct {
//             pub const PB1 = Pin("PB1").AlternateFunction(.{ .af = 2 });
//             pub const PC9 = Pin("PC9").AlternateFunction(.{ .af = 2 });
//         };
//     };
//     pub const TIM4 = struct {
//         pub const CH1 = struct {
//             pub const PB6 = Pin("PB6").AlternateFunction(.{ .af = 2 });
//         };
//         pub const CH2 = struct {
//             pub const PB7 = Pin("PB7").AlternateFunction(.{ .af = 2 });
//         };
//         pub const CH3 = struct {
//             pub const PB8 = Pin("PB8").AlternateFunction(.{ .af = 2 });
//         };
//         pub const CH4 = struct {
//             pub const PB9 = Pin("PB9").AlternateFunction(.{ .af = 2 });
//         };
//     };
//     pub const TIM5 = struct {
//         pub const CH1 = struct {
//             pub const PA0 = Pin("PA0").AlternateFunction(.{ .af = 2 });
//         };
//         pub const CH2 = struct {
//             pub const PA1 = Pin("PA1").AlternateFunction(.{ .af = 2 });
//         };
//         pub const CH3 = struct {
//             pub const PA2 = Pin("PA2").AlternateFunction(.{ .af = 2 });
//         };
//         pub const CH4 = struct {
//             pub const PA3 = Pin("PA3").AlternateFunction(.{ .af = 2 });
//         };
//     };
//     pub const TIM9 = struct {
//         pub const CH1 = struct {
//             pub const PA2 = Pin("PA2").AlternateFunction(.{ .af = 3 });
//         };
//         pub const CH2 = struct {
//             pub const PA3 = Pin("PA3").AlternateFunction(.{ .af = 3 });
//         };
//     };
//     pub const USART1 = struct {
//         pub const CK = struct {
//             pub const PA8 = Pin("PA8").AlternateFunction(.{ .af = 7 });
//         };
//         pub const TX = struct {
//             pub const PA9 = Pin("PA9").AlternateFunction(.{ .af = 7 });
//             pub const PA15 = Pin("PA15").AlternateFunction(.{ .af = 7 });
//             pub const PB6 = Pin("PB6").AlternateFunction(.{ .af = 7 });
//         };
//         pub const RX = struct {
//             pub const PA10 = Pin("PA10").AlternateFunction(.{ .af = 7 });
//             pub const PB3 = Pin("PB3").AlternateFunction(.{ .af = 7 });
//             pub const PB7 = Pin("PB7").AlternateFunction(.{ .af = 7 });
//         };
//         pub const CTS = struct {
//             pub const PA11 = Pin("PA11").AlternateFunction(.{ .af = 7 });
//         };
//         pub const RTS = struct {
//             pub const PA12 = Pin("PA12").AlternateFunction(.{ .af = 7 });
//         };
//     };
//     pub const USART2 = struct {
//         pub const CTS = struct {
//             pub const PA0 = Pin("PA0").AlternateFunction(.{ .af = 7 });
//         };
//         pub const RTS = struct {
//             pub const PA1 = Pin("PA1").AlternateFunction(.{ .af = 7 });
//         };
//         pub const TX = struct {
//             pub const PA2 = Pin("PA2").AlternateFunction(.{ .af = 7 });
//         };
//         pub const RX = struct {
//             pub const PA3 = Pin("PA3").AlternateFunction(.{ .af = 7 });
//         };
//         pub const CK = struct {
//             pub const PA4 = Pin("PA4").AlternateFunction(.{ .af = 7 });
//         };
//     };
//     pub const USART6 = struct {
//         pub const TX = struct {
//             pub const PA11 = Pin("PA11").AlternateFunction(.{ .af = 8 });
//             pub const PC6 = Pin("PC6").AlternateFunction(.{ .af = 8 });
//         };
//         pub const RX = struct {
//             pub const PA12 = Pin("PA12").AlternateFunction(.{ .af = 8 });
//             pub const PC7 = Pin("PC7").AlternateFunction(.{ .af = 8 });
//         };
//     };
//     pub const USB_OTG_FS = struct {
//         pub const SOF = struct {
//             pub const PA8 = Pin("PA8").AlternateFunction(.{ .af = 10 });
//         };
//         pub const VBUS = struct {
//             pub const PA9 = Pin("PA9").AlternateFunction(.{ .af = 10 });
//         };
//         pub const ID = struct {
//             pub const PA10 = Pin("PA10").AlternateFunction(.{ .af = 10 });
//         };
//         pub const DM = struct {
//             pub const PA11 = Pin("PA11").AlternateFunction(.{ .af = 10 });
//         };
//         pub const DP = struct {
//             pub const PA12 = Pin("PA12").AlternateFunction(.{ .af = 10 });
//         };
//     };

//     pub const pa0 = struct {
//         const pin = Pin2("PA0");
//         pub const Output = pin.Output;
//         pub const Input = pin.Input;
//         pub const Analog = pin.Analog;
//         pub const adc1 = struct {
//             pub const In0 = pin.Analog;
//         };
//         pub const tim2 = struct {
//             pub fn Ch1() type {
//                 return pin.Function(1);
//             }
//             pub fn Etr() type {
//                 return pin.Function(1);
//             }
//         };
//         pub const tim5 = struct {
//             pub fn Ch1() type {
//                 return pin.Function(2);
//             }
//         };
//         pub const usart2 = struct {
//             pub fn Cts() type {
//                 return pin.Function(7);
//             }
//         };
//     };
//     pub const pa1 = struct {
//         const pin = Pin2("PA1");
//         pub const Output = pin.Output;
//         pub const Input = pin.Input;
//         pub const Analog = pin.Analog;
//         pub const adc1 = struct {
//             pub const In1 = pin.Analog;
//         };
//         pub const spi4 = struct {
//             pub fn Mosi() type {
//                 return pin.Function(5);
//             }
//         };
//         pub const tim2 = struct {
//             pub fn Ch2() type {
//                 return pin.Function(1);
//             }
//         };
//         pub const tim5 = struct {
//             pub fn Ch2() type {
//                 return pin.Function(2);
//             }
//         };
//         pub const usart2 = struct {
//             pub fn Rts() type {
//                 return pin.Function(7);
//             }
//         };
//     };
//     pub const pa2 = struct {
//         const pin = Pin2("PA2");
//         pub const Output = pin.Output;
//         pub const Input = pin.Input;
//         pub const Analog = pin.Analog;
//         pub const adc1 = struct {
//             pub const In2 = pin.Analog;
//         };
//         pub const tim2 = struct {
//             pub fn Ch3() type {
//                 return pin.Function(1);
//             }
//         };
//         pub const tim5 = struct {
//             pub fn Ch3() type {
//                 return pin.Function(2);
//             }
//         };
//         pub const tim9 = struct {
//             pub fn Ch1() type {
//                 return pin.Function(3);
//             }
//         };
//         pub const usart2 = struct {
//             pub fn Tx() type {
//                 return pin.Function(7);
//             }
//         };
//     };
//     pub const pa3 = struct {
//         const pin = Pin2("PA3");
//         pub const Output = pin.Output;
//         pub const Input = pin.Input;
//         pub const Analog = pin.Analog;
//         pub const adc1 = struct {
//             pub const In3 = pin.Analog;
//         };
//         pub const tim2 = struct {
//             pub fn Ch4() type {
//                 return pin.Function(1);
//             }
//         };
//         pub const tim5 = struct {
//             pub fn Ch4() type {
//                 return pin.Function(2);
//             }
//         };
//         pub const tim9 = struct {
//             pub fn Ch2() type {
//                 return pin.Function(3);
//             }
//         };
//         pub const usart2 = struct {
//             pub fn Rx() type {
//                 return pin.Function(7);
//             }
//         };
//     };
//     pub const pa4 = struct {
//         const pin = Pin2("PA4");
//         pub const Output = pin.Output;
//         pub const Input = pin.Input;
//         pub const Analog = pin.Analog;
//         pub const adc1 = struct {
//             pub const In4 = pin.Analog;
//         };
//         pub const spi1 = struct {
//             pub fn Nss() type {
//                 return pin.Function(5);
//             }
//         };
//         pub const spi3 = struct {
//             pub fn Nss() type {
//                 return pin.Function(6);
//             }
//         };
//         pub const usart2 = struct {
//             pub fn Ck() type {
//                 return pin.Function(7);
//             }
//         };
//     };
//     pub const pa5 = struct {
//         const pin = Pin2("PA5");
//         pub const Output = pin.Output;
//         pub const Input = pin.Input;
//         pub const Analog = pin.Analog;
//         pub const adc1 = struct {
//             pub const In5 = pin.Analog;
//         };
//         pub const spi1 = struct {
//             pub fn Sck() type {
//                 return pin.Function(5);
//             }
//         };
//         pub const tim2 = struct {
//             pub fn Ch1() type {
//                 return pin.Function(1);
//             }
//             pub fn Etr() type {
//                 return pin.Function(1);
//             }
//         };
//     };
//     pub const pa6 = struct {
//         const pin = Pin2("PA6");
//         pub const Output = pin.Output;
//         pub const Input = pin.Input;
//         pub const Analog = pin.Analog;
//         pub const adc1 = struct {
//             pub const In6 = pin.Analog;
//         };
//         pub const sdio = struct {
//             pub fn Cmd() type {
//                 return pin.Function(12);
//             }
//         };
//         pub const spi1 = struct {
//             pub fn Miso() type {
//                 return pin.Function(5);
//             }
//         };
//         pub const tim1 = struct {
//             pub fn Bkin() type {
//                 return pin.Function(1);
//             }
//         };
//         pub const tim3 = struct {
//             pub fn Ch1() type {
//                 return pin.Function(2);
//             }
//         };
//     };
//     pub const pa7 = struct {
//         const pin = Pin2("PA7");
//         pub const Output = pin.Output;
//         pub const Input = pin.Input;
//         pub const Analog = pin.Analog;
//         pub const adc1 = struct {
//             pub const In7 = pin.Analog;
//         };
//         pub const spi1 = struct {
//             pub fn Mosi() type {
//                 return pin.Function(5);
//             }
//         };
//         pub const tim1 = struct {
//             pub fn Ch1n() type {
//                 return pin.Function(1);
//             }
//         };
//         pub const tim3 = struct {
//             pub fn Ch2() type {
//                 return pin.Function(2);
//             }
//         };
//     };
//     pub const pa8 = struct {
//         const pin = Pin2("PA8");
//         pub const Output = pin.Output;
//         pub const Input = pin.Input;
//         pub const Analog = pin.Analog;
//         pub const i2c3 = struct {
//             pub fn Scl() type {
//                 return pin.Function(4);
//             }
//         };
//         pub const rcc = struct {
//             pub fn Mco1() type {
//                 return pin.Function(0);
//             }
//         };
//         pub const sdio = struct {
//             pub fn D1() type {
//                 return pin.Function(12);
//             }
//         };
//         pub const tim1 = struct {
//             pub fn Ch1() type {
//                 return pin.Function(1);
//             }
//         };
//         pub const usart1 = struct {
//             pub fn Ck() type {
//                 return pin.Function(7);
//             }
//         };
//         pub const usb_otg_fs = struct {
//             pub fn Sof() type {
//                 return pin.Function(10);
//             }
//         };
//     };
//     pub const pa9 = struct {
//         const pin = Pin2("PA9");
//         pub const Output = pin.Output;
//         pub const Input = pin.Input;
//         pub const Analog = pin.Analog;
//         pub const i2c3 = struct {
//             pub fn Smba() type {
//                 return pin.Function(4);
//             }
//         };
//         pub const sdio = struct {
//             pub fn D2() type {
//                 return pin.Function(12);
//             }
//         };
//         pub const tim1 = struct {
//             pub fn Ch2() type {
//                 return pin.Function(1);
//             }
//         };
//         pub const usart1 = struct {
//             pub fn Tx() type {
//                 return pin.Function(7);
//             }
//         };
//         pub const usb_otg_fs = struct {
//             pub fn Vbus() type {
//                 return pin.Function(10);
//             }
//         };
//     };
//     pub const pa10 = struct {
//         const pin = Pin2("PA10");
//         pub const Output = pin.Output;
//         pub const Input = pin.Input;
//         pub const Analog = pin.Analog;
//         pub const spi5 = struct {
//             pub fn Mosi() type {
//                 return pin.Function(6);
//             }
//         };
//         pub const tim1 = struct {
//             pub fn Ch3() type {
//                 return pin.Function(1);
//             }
//         };
//         pub const usart1 = struct {
//             pub fn Rx() type {
//                 return pin.Function(7);
//             }
//         };
//         pub const usb_otg_fs = struct {
//             pub fn Id() type {
//                 return pin.Function(10);
//             }
//         };
//     };
//     pub const pa11 = struct {
//         const pin = Pin2("PA11");
//         pub const Output = pin.Output;
//         pub const Input = pin.Input;
//         pub const Analog = pin.Analog;
//         pub const spi4 = struct {
//             pub fn Miso() type {
//                 return pin.Function(6);
//             }
//         };
//         pub const tim1 = struct {
//             pub fn Ch4() type {
//                 return pin.Function(1);
//             }
//         };
//         pub const usart1 = struct {
//             pub fn Cts() type {
//                 return pin.Function(7);
//             }
//         };
//         pub const usart6 = struct {
//             pub fn Tx() type {
//                 return pin.Function(8);
//             }
//         };
//         pub const usb_otg_fs = struct {
//             pub fn Dm() type {
//                 return pin.Function(10);
//             }
//         };
//     };
//     pub const pa12 = struct {
//         const pin = Pin2("PA12");
//         pub const Output = pin.Output;
//         pub const Input = pin.Input;
//         pub const Analog = pin.Analog;
//         pub const spi5 = struct {
//             pub fn Miso() type {
//                 return pin.Function(6);
//             }
//         };
//         pub const tim1 = struct {
//             pub fn Etr() type {
//                 return pin.Function(1);
//             }
//         };
//         pub const usart1 = struct {
//             pub fn Rts() type {
//                 return pin.Function(7);
//             }
//         };
//         pub const usart6 = struct {
//             pub fn Rx() type {
//                 return pin.Function(8);
//             }
//         };
//         pub const usb_otg_fs = struct {
//             pub fn Dp() type {
//                 return pin.Function(10);
//             }
//         };
//     };
//     pub const pa13 = struct {
//         const pin = Pin2("PA13");
//         pub const Output = pin.Output;
//         pub const Input = pin.Input;
//         pub const Analog = pin.Analog;
//     };
//     pub const pa14 = struct {
//         const pin = Pin2("PA14");
//         pub const Output = pin.Output;
//         pub const Input = pin.Input;
//         pub const Analog = pin.Analog;
//     };
//     pub const pa15 = struct {
//         const pin = Pin2("PA15");
//         pub const Output = pin.Output;
//         pub const Input = pin.Input;
//         pub const Analog = pin.Analog;
//         pub const spi1 = struct {
//             pub fn Nss() type {
//                 return pin.Function(5);
//             }
//         };
//         pub const spi3 = struct {
//             pub fn Nss() type {
//                 return pin.Function(6);
//             }
//         };
//         pub const tim2 = struct {
//             pub fn Ch1() type {
//                 return pin.Function(1);
//             }
//             pub fn Etr() type {
//                 return pin.Function(1);
//             }
//         };
//         pub const usart1 = struct {
//             pub fn Tx() type {
//                 return pin.Function(7);
//             }
//         };
//     };
//     pub const pb0 = struct {
//         const pin = Pin2("PB0");
//         pub const Output = pin.Output;
//         pub const Input = pin.Input;
//         pub const Analog = pin.Analog;
//         pub const adc1 = struct {
//             pub const In8 = pin.Analog;
//         };
//         pub const spi5 = struct {
//             pub fn Sck() type {
//                 return pin.Function(6);
//             }
//         };
//         pub const tim1 = struct {
//             pub fn Ch2n() type {
//                 return pin.Function(1);
//             }
//         };
//         pub const tim3 = struct {
//             pub fn Ch3() type {
//                 return pin.Function(2);
//             }
//         };
//     };
//     pub const pb1 = struct {
//         const pin = Pin2("PB1");
//         pub const Output = pin.Output;
//         pub const Input = pin.Input;
//         pub const Analog = pin.Analog;
//         pub const adc1 = struct {
//             pub const In9 = pin.Analog;
//         };
//         pub const spi5 = struct {
//             pub fn Nss() type {
//                 return pin.Function(6);
//             }
//         };
//         pub const tim1 = struct {
//             pub fn Ch3n() type {
//                 return pin.Function(1);
//             }
//         };
//         pub const tim3 = struct {
//             pub fn Ch4() type {
//                 return pin.Function(2);
//             }
//         };
//     };
//     pub const pb2 = struct {
//         const pin = Pin2("PB2");
//         pub const Output = pin.Output;
//         pub const Input = pin.Input;
//         pub const Analog = pin.Analog;
//     };
//     pub const pb3 = struct {
//         const pin = Pin2("PB3");
//         pub const Output = pin.Output;
//         pub const Input = pin.Input;
//         pub const Analog = pin.Analog;
//         pub const i2c2 = struct {
//             pub fn Sda() type {
//                 return pin.Function(9);
//             }
//         };
//         pub const spi1 = struct {
//             pub fn Sck() type {
//                 return pin.Function(5);
//             }
//         };
//         pub const spi3 = struct {
//             pub fn Sck() type {
//                 return pin.Function(6);
//             }
//         };
//         pub const tim2 = struct {
//             pub fn Ch2() type {
//                 return pin.Function(1);
//             }
//         };
//         pub const usart1 = struct {
//             pub fn Rx() type {
//                 return pin.Function(7);
//             }
//         };
//     };
//     pub const pb4 = struct {
//         const pin = Pin2("PB4");
//         pub const Output = pin.Output;
//         pub const Input = pin.Input;
//         pub const Analog = pin.Analog;
//         pub const i2c3 = struct {
//             pub fn Sda() type {
//                 return pin.Function(9);
//             }
//         };
//         pub const sdio = struct {
//             pub fn D0() type {
//                 return pin.Function(12);
//             }
//         };
//         pub const spi1 = struct {
//             pub fn Miso() type {
//                 return pin.Function(5);
//             }
//         };
//         pub const spi3 = struct {
//             pub fn Miso() type {
//                 return pin.Function(6);
//             }
//         };
//         pub const tim3 = struct {
//             pub fn Ch1() type {
//                 return pin.Function(2);
//             }
//         };
//     };
//     pub const pb5 = struct {
//         const pin = Pin2("PB5");
//         pub const Output = pin.Output;
//         pub const Input = pin.Input;
//         pub const Analog = pin.Analog;
//         pub const i2c1 = struct {
//             pub fn Smba() type {
//                 return pin.Function(4);
//             }
//         };
//         pub const sdio = struct {
//             pub fn D3() type {
//                 return pin.Function(12);
//             }
//         };
//         pub const spi1 = struct {
//             pub fn Mosi() type {
//                 return pin.Function(5);
//             }
//         };
//         pub const spi3 = struct {
//             pub fn Mosi() type {
//                 return pin.Function(6);
//             }
//         };
//         pub const tim3 = struct {
//             pub fn Ch2() type {
//                 return pin.Function(2);
//             }
//         };
//     };
//     pub const pb6 = struct {
//         const pin = Pin2("PB6");
//         pub const Output = pin.Output;
//         pub const Input = pin.Input;
//         pub const Analog = pin.Analog;
//         pub const i2c1 = struct {
//             pub fn Scl() type {
//                 return pin.Function(4);
//             }
//         };
//         pub const tim4 = struct {
//             pub fn Ch1() type {
//                 return pin.Function(2);
//             }
//         };
//         pub const usart1 = struct {
//             pub fn Tx() type {
//                 return pin.Function(7);
//             }
//         };
//     };
//     pub const pb7 = struct {
//         const pin = Pin2("PB7");
//         pub const Output = pin.Output;
//         pub const Input = pin.Input;
//         pub const Analog = pin.Analog;
//         pub const i2c1 = struct {
//             pub fn Sda() type {
//                 return pin.Function(4);
//             }
//         };
//         pub const sdio = struct {
//             pub fn D0() type {
//                 return pin.Function(12);
//             }
//         };
//         pub const tim4 = struct {
//             pub fn Ch2() type {
//                 return pin.Function(2);
//             }
//         };
//         pub const usart1 = struct {
//             pub fn Rx() type {
//                 return pin.Function(7);
//             }
//         };
//     };
//     pub const pb8 = struct {
//         const pin = Pin2("PB8");
//         pub const Output = pin.Output;
//         pub const Input = pin.Input;
//         pub const Analog = pin.Analog;
//         pub const i2c1 = struct {
//             pub fn Scl() type {
//                 return pin.Function(4);
//             }
//         };
//         pub const i2c3 = struct {
//             pub fn Sda() type {
//                 return pin.Function(9);
//             }
//         };
//         pub const sdio = struct {
//             pub fn D4() type {
//                 return pin.Function(12);
//             }
//         };
//         pub const spi5 = struct {
//             pub fn Mosi() type {
//                 return pin.Function(6);
//             }
//         };
//         pub const tim10 = struct {
//             pub fn Ch1() type {
//                 return pin.Function(3);
//             }
//         };
//         pub const tim4 = struct {
//             pub fn Ch3() type {
//                 return pin.Function(2);
//             }
//         };
//     };
//     pub const pb9 = struct {
//         const pin = Pin2("PB9");
//         pub const Output = pin.Output;
//         pub const Input = pin.Input;
//         pub const Analog = pin.Analog;
//         pub const i2c1 = struct {
//             pub fn Sda() type {
//                 return pin.Function(4);
//             }
//         };
//         pub const i2c2 = struct {
//             pub fn Sda() type {
//                 return pin.Function(9);
//             }
//         };
//         pub const sdio = struct {
//             pub fn D5() type {
//                 return pin.Function(12);
//             }
//         };
//         pub const spi2 = struct {
//             pub fn Nss() type {
//                 return pin.Function(5);
//             }
//         };
//         pub const tim11 = struct {
//             pub fn Ch1() type {
//                 return pin.Function(3);
//             }
//         };
//         pub const tim4 = struct {
//             pub fn Ch4() type {
//                 return pin.Function(2);
//             }
//         };
//     };
//     pub const pb10 = struct {
//         const pin = Pin2("PB10");
//         pub const Output = pin.Output;
//         pub const Input = pin.Input;
//         pub const Analog = pin.Analog;
//         pub const i2c2 = struct {
//             pub fn Scl() type {
//                 return pin.Function(4);
//             }
//         };
//         pub const sdio = struct {
//             pub fn D7() type {
//                 return pin.Function(12);
//             }
//         };
//         pub const spi2 = struct {
//             pub fn Sck() type {
//                 return pin.Function(5);
//             }
//         };
//         pub const tim2 = struct {
//             pub fn Ch3() type {
//                 return pin.Function(1);
//             }
//         };
//     };
//     pub const pb12 = struct {
//         const pin = Pin2("PB12");
//         pub const Output = pin.Output;
//         pub const Input = pin.Input;
//         pub const Analog = pin.Analog;
//         pub const i2c2 = struct {
//             pub fn Smba() type {
//                 return pin.Function(4);
//             }
//         };
//         pub const spi2 = struct {
//             pub fn Nss() type {
//                 return pin.Function(5);
//             }
//         };
//         pub const spi3 = struct {
//             pub fn Sck() type {
//                 return pin.Function(7);
//             }
//         };
//         pub const spi4 = struct {
//             pub fn Nss() type {
//                 return pin.Function(6);
//             }
//         };
//         pub const tim1 = struct {
//             pub fn Bkin() type {
//                 return pin.Function(1);
//             }
//         };
//     };
//     pub const pb13 = struct {
//         const pin = Pin2("PB13");
//         pub const Output = pin.Output;
//         pub const Input = pin.Input;
//         pub const Analog = pin.Analog;
//         pub const spi2 = struct {
//             pub fn Sck() type {
//                 return pin.Function(5);
//             }
//         };
//         pub const spi4 = struct {
//             pub fn Sck() type {
//                 return pin.Function(6);
//             }
//         };
//         pub const tim1 = struct {
//             pub fn Ch1n() type {
//                 return pin.Function(1);
//             }
//         };
//     };
//     pub const pb14 = struct {
//         const pin = Pin2("PB14");
//         pub const Output = pin.Output;
//         pub const Input = pin.Input;
//         pub const Analog = pin.Analog;
//         pub const sdio = struct {
//             pub fn D6() type {
//                 return pin.Function(12);
//             }
//         };
//         pub const spi2 = struct {
//             pub fn Miso() type {
//                 return pin.Function(5);
//             }
//         };
//         pub const tim1 = struct {
//             pub fn Ch2n() type {
//                 return pin.Function(1);
//             }
//         };
//     };
//     pub const pb15 = struct {
//         const pin = Pin2("PB15");
//         pub const Output = pin.Output;
//         pub const Input = pin.Input;
//         pub const Analog = pin.Analog;
//         pub const rtc = struct {
//             pub fn Refin() type {
//                 return pin.Function(0);
//             }
//         };
//         pub const sdio = struct {
//             pub fn Ck() type {
//                 return pin.Function(12);
//             }
//         };
//         pub const spi2 = struct {
//             pub fn Mosi() type {
//                 return pin.Function(5);
//             }
//         };
//         pub const tim1 = struct {
//             pub fn Ch3n() type {
//                 return pin.Function(1);
//             }
//         };
//     };
//     pub const pc0 = struct {
//         const pin = Pin2("PC0");
//         pub const Output = pin.Output;
//         pub const Input = pin.Input;
//         pub const Analog = pin.Analog;
//         pub const adc1 = struct {
//             pub const In10 = pin.Analog;
//         };
//     };
//     pub const pc1 = struct {
//         const pin = Pin2("PC1");
//         pub const Output = pin.Output;
//         pub const Input = pin.Input;
//         pub const Analog = pin.Analog;
//         pub const adc1 = struct {
//             pub const In11 = pin.Analog;
//         };
//     };
//     pub const pc2 = struct {
//         const pin = Pin2("PC2");
//         pub const Output = pin.Output;
//         pub const Input = pin.Input;
//         pub const Analog = pin.Analog;
//         pub const adc1 = struct {
//             pub const In12 = pin.Analog;
//         };
//         pub const spi2 = struct {
//             pub fn Miso() type {
//                 return pin.Function(5);
//             }
//         };
//     };
//     pub const pc3 = struct {
//         const pin = Pin2("PC3");
//         pub const Output = pin.Output;
//         pub const Input = pin.Input;
//         pub const Analog = pin.Analog;
//         pub const adc1 = struct {
//             pub const In13 = pin.Analog;
//         };
//         pub const spi2 = struct {
//             pub fn Mosi() type {
//                 return pin.Function(5);
//             }
//         };
//     };
//     pub const pc4 = struct {
//         const pin = Pin2("PC4");
//         pub const Output = pin.Output;
//         pub const Input = pin.Input;
//         pub const Analog = pin.Analog;
//         pub const adc1 = struct {
//             pub const In14 = pin.Analog;
//         };
//     };
//     pub const pc5 = struct {
//         const pin = Pin2("PC5");
//         pub const Output = pin.Output;
//         pub const Input = pin.Input;
//         pub const Analog = pin.Analog;
//         pub const adc1 = struct {
//             pub const In15 = pin.Analog;
//         };
//     };
//     pub const pc6 = struct {
//         const pin = Pin2("PC6");
//         pub const Output = pin.Output;
//         pub const Input = pin.Input;
//         pub const Analog = pin.Analog;
//         pub const sdio = struct {
//             pub fn D6() type {
//                 return pin.Function(12);
//             }
//         };
//         pub const tim3 = struct {
//             pub fn Ch1() type {
//                 return pin.Function(2);
//             }
//         };
//         pub const usart6 = struct {
//             pub fn Tx() type {
//                 return pin.Function(8);
//             }
//         };
//     };
//     pub const pc7 = struct {
//         const pin = Pin2("PC7");
//         pub const Output = pin.Output;
//         pub const Input = pin.Input;
//         pub const Analog = pin.Analog;
//         pub const sdio = struct {
//             pub fn D7() type {
//                 return pin.Function(12);
//             }
//         };
//         pub const spi2 = struct {
//             pub fn Sck() type {
//                 return pin.Function(5);
//             }
//         };
//         pub const tim3 = struct {
//             pub fn Ch2() type {
//                 return pin.Function(2);
//             }
//         };
//         pub const usart6 = struct {
//             pub fn Rx() type {
//                 return pin.Function(8);
//             }
//         };
//     };
//     pub const pc8 = struct {
//         const pin = Pin2("PC8");
//         pub const Output = pin.Output;
//         pub const Input = pin.Input;
//         pub const Analog = pin.Analog;
//         pub const sdio = struct {
//             pub fn D0() type {
//                 return pin.Function(12);
//             }
//         };
//         pub const tim3 = struct {
//             pub fn Ch3() type {
//                 return pin.Function(2);
//             }
//         };
//         pub const usart6 = struct {
//             pub fn Ck() type {
//                 return pin.Function(8);
//             }
//         };
//     };
//     pub const pc9 = struct {
//         const pin = Pin2("PC9");
//         pub const Output = pin.Output;
//         pub const Input = pin.Input;
//         pub const Analog = pin.Analog;
//         pub const i2c3 = struct {
//             pub fn Sda() type {
//                 return pin.Function(4);
//             }
//         };
//         pub const rcc = struct {
//             pub fn Mco2() type {
//                 return pin.Function(0);
//             }
//         };
//         pub const sdio = struct {
//             pub fn D1() type {
//                 return pin.Function(12);
//             }
//         };
//         pub const tim3 = struct {
//             pub fn Ch4() type {
//                 return pin.Function(2);
//             }
//         };
//     };
//     pub const pc10 = struct {
//         const pin = Pin2("PC10");
//         pub const Output = pin.Output;
//         pub const Input = pin.Input;
//         pub const Analog = pin.Analog;
//         pub const sdio = struct {
//             pub fn D2() type {
//                 return pin.Function(12);
//             }
//         };
//         pub const spi3 = struct {
//             pub fn Sck() type {
//                 return pin.Function(6);
//             }
//         };
//     };
//     pub const pc11 = struct {
//         const pin = Pin2("PC11");
//         pub const Output = pin.Output;
//         pub const Input = pin.Input;
//         pub const Analog = pin.Analog;
//         pub const sdio = struct {
//             pub fn D3() type {
//                 return pin.Function(12);
//             }
//         };
//         pub const spi3 = struct {
//             pub fn Miso() type {
//                 return pin.Function(6);
//             }
//         };
//     };
//     pub const pc12 = struct {
//         const pin = Pin2("PC12");
//         pub const Output = pin.Output;
//         pub const Input = pin.Input;
//         pub const Analog = pin.Analog;
//         pub const sdio = struct {
//             pub fn Ck() type {
//                 return pin.Function(12);
//             }
//         };
//         pub const spi3 = struct {
//             pub fn Mosi() type {
//                 return pin.Function(6);
//             }
//         };
//     };
//     pub const pc13 = struct {
//         const pin = Pin2("PC13");
//         pub const Output = pin.Output;
//         pub const Input = pin.Input;
//         pub const Analog = pin.Analog;
//         pub const rtc = struct {
//             pub const Af1 = pin.Analog;
//         };
//     };
//     pub const pc14 = struct {
//         const pin = Pin2("PC14");
//         pub const Output = pin.Output;
//         pub const Input = pin.Input;
//         pub const Analog = pin.Analog;
//         pub const rcc = struct {
//             pub const Osc32In = pin.Analog;
//         };
//     };
//     pub const pc15 = struct {
//         const pin = Pin2("PC15");
//         pub const Output = pin.Output;
//         pub const Input = pin.Input;
//         pub const Analog = pin.Analog;
//         pub const rcc = struct {
//             pub const Osc32Out = pin.Analog;
//         };
//     };
//     pub const pd2 = struct {
//         const pin = Pin2("PD2");
//         pub const Output = pin.Output;
//         pub const Input = pin.Input;
//         pub const Analog = pin.Analog;
//         pub const sdio = struct {
//             pub fn Cmd() type {
//                 return pin.Function(12);
//             }
//         };
//         pub const tim3 = struct {
//             pub fn Etr() type {
//                 return pin.Function(2);
//             }
//         };
//     };
//     pub const ph0 = struct {
//         const pin = Pin2("PH0");
//         pub const Output = pin.Output;
//         pub const Input = pin.Input;
//         pub const Analog = pin.Analog;
//         pub const rcc = struct {
//             pub const OscIn = pin.Analog;
//         };
//     };
//     pub const ph1 = struct {
//         const pin = Pin2("PH1");
//         pub const Output = pin.Output;
//         pub const Input = pin.Input;
//         pub const Analog = pin.Analog;
//         pub const rcc = struct {
//             pub const OscOut = pin.Analog;
//         };
//     };
//     pub const i2c1 = struct {
//         pub const smba = struct {
//             pub const Pb5 = pb5.i2c1.Smba;
//         };
//         pub const scl = struct {
//             pub const Pb6 = pb6.i2c1.Scl;
//             pub const Pb8 = pb8.i2c1.Scl;
//         };
//         pub const sda = struct {
//             pub const Pb7 = pb7.i2c1.Sda;
//             pub const Pb9 = pb9.i2c1.Sda;
//         };
//     };
//     pub const i2c2 = struct {
//         pub const sda = struct {
//             pub const Pb3 = pb3.i2c2.Sda;
//             pub const Pb9 = pb9.i2c2.Sda;
//         };
//     };
//     pub const i2c3 = struct {
//         pub const scl = struct {
//             pub const Pa8 = pa8.i2c3.Scl;
//         };
//         pub const smba = struct {
//             pub const Pa9 = pa9.i2c3.Smba;
//         };
//         pub const sda = struct {
//             pub const Pb4 = pb4.i2c3.Sda;
//             pub const Pb8 = pb8.i2c3.Sda;
//             pub const Pc9 = pc9.i2c3.Sda;
//         };
//     };
//     pub const sdio = struct {
//         pub const cmd = struct {
//             pub const Pa6 = pa6.sdio.Cmd;
//             pub const Pd2 = pd2.sdio.Cmd;
//         };
//         pub const d1 = struct {
//             pub const Pa8 = pa8.sdio.D1;
//             pub const Pc9 = pc9.sdio.D1;
//         };
//         pub const d2 = struct {
//             pub const Pa9 = pa9.sdio.D2;
//             pub const Pc10 = pc10.sdio.D2;
//         };
//         pub const d0 = struct {
//             pub const Pb4 = pb4.sdio.D0;
//             pub const Pb7 = pb7.sdio.D0;
//             pub const Pc8 = pc8.sdio.D0;
//         };
//         pub const d3 = struct {
//             pub const Pb5 = pb5.sdio.D3;
//             pub const Pc11 = pc11.sdio.D3;
//         };
//     };
//     pub const spi1 = struct {
//         pub const nss = struct {
//             pub const Pa4 = pa4.spi1.Nss;
//             pub const Pa15 = pa15.spi1.Nss;
//         };
//         pub const sck = struct {
//             pub const Pa5 = pa5.spi1.Sck;
//             pub const Pb3 = pb3.spi1.Sck;
//         };
//         pub const miso = struct {
//             pub const Pa6 = pa6.spi1.Miso;
//             pub const Pb4 = pb4.spi1.Miso;
//         };
//         pub const mosi = struct {
//             pub const Pa7 = pa7.spi1.Mosi;
//             pub const Pb5 = pb5.spi1.Mosi;
//         };
//     };
//     pub const spi2 = struct {
//         pub const nss = struct {
//             pub const Pb9 = pb9.spi2.Nss;
//             pub const Pb12 = pb12.spi2.Nss;
//         };
//         pub const sck = struct {
//             pub const Pb10 = pb10.spi2.Sck;
//             pub const Pb13 = pb13.spi2.Sck;
//             pub const Pc7 = pc7.spi2.Sck;
//         };
//     };
//     pub const spi3 = struct {
//         pub const nss = struct {
//             pub const Pa4 = pa4.spi3.Nss;
//             pub const Pa15 = pa15.spi3.Nss;
//         };
//     };
//     pub const spi4 = struct {
//         pub const mosi = struct {
//             pub const Pa1 = pa1.spi4.Mosi;
//         };
//         pub const miso = struct {
//             pub const Pa11 = pa11.spi4.Miso;
//         };
//         pub const nss = struct {
//             pub const Pb12 = pb12.spi4.Nss;
//         };
//         pub const sck = struct {
//             pub const Pb13 = pb13.spi4.Sck;
//         };
//     };
//     pub const spi5 = struct {
//         pub const mosi = struct {
//             pub const Pa10 = pa10.spi5.Mosi;
//             pub const Pb8 = pb8.spi5.Mosi;
//         };
//         pub const miso = struct {
//             pub const Pa12 = pa12.spi5.Miso;
//         };
//         pub const sck = struct {
//             pub const Pb0 = pb0.spi5.Sck;
//         };
//         pub const nss = struct {
//             pub const Pb1 = pb1.spi5.Nss;
//         };
//     };
//     pub const tim1 = struct {
//         pub const bkin = struct {
//             pub const Pa6 = pa6.tim1.Bkin;
//             pub const Pb12 = pb12.tim1.Bkin;
//         };
//         pub const ch1n = struct {
//             pub const Pa7 = pa7.tim1.Ch1n;
//             pub const Pb13 = pb13.tim1.Ch1n;
//         };
//         pub const ch1 = struct {
//             pub const Pa8 = pa8.tim1.Ch1;
//         };
//         pub const ch2 = struct {
//             pub const Pa9 = pa9.tim1.Ch2;
//         };
//         pub const ch3 = struct {
//             pub const Pa10 = pa10.tim1.Ch3;
//         };
//         pub const ch4 = struct {
//             pub const Pa11 = pa11.tim1.Ch4;
//         };
//         pub const etr = struct {
//             pub const Pa12 = pa12.tim1.Etr;
//         };
//         pub const ch2n = struct {
//             pub const Pb0 = pb0.tim1.Ch2n;
//             pub const Pb14 = pb14.tim1.Ch2n;
//         };
//         pub const ch3n = struct {
//             pub const Pb1 = pb1.tim1.Ch3n;
//             pub const Pb15 = pb15.tim1.Ch3n;
//         };
//     };
//     pub const tim10 = struct {
//         pub const ch1 = struct {
//             pub const Pb8 = pb8.tim10.Ch1;
//         };
//     };
//     pub const tim11 = struct {
//         pub const ch1 = struct {
//             pub const Pb9 = pb9.tim11.Ch1;
//         };
//     };
//     pub const tim2 = struct {
//         pub const ch1 = struct {
//             pub const Pa0 = pa0.tim2.Ch1;
//             pub const Pa5 = pa5.tim2.Ch1;
//             pub const Pa15 = pa15.tim2.Ch1;
//         };
//         pub const etr = struct {
//             pub const Pa0 = pa0.tim2.Etr;
//             pub const Pa5 = pa5.tim2.Etr;
//             pub const Pa15 = pa15.tim2.Etr;
//         };
//         pub const ch2 = struct {
//             pub const Pa1 = pa1.tim2.Ch2;
//             pub const Pb3 = pb3.tim2.Ch2;
//         };
//         pub const ch3 = struct {
//             pub const Pa2 = pa2.tim2.Ch3;
//             pub const Pb10 = pb10.tim2.Ch3;
//         };
//         pub const ch4 = struct {
//             pub const Pa3 = pa3.tim2.Ch4;
//         };
//     };
//     pub const tim3 = struct {
//         pub const ch1 = struct {
//             pub const Pa6 = pa6.tim3.Ch1;
//             pub const Pb4 = pb4.tim3.Ch1;
//             pub const Pc6 = pc6.tim3.Ch1;
//         };
//         pub const ch2 = struct {
//             pub const Pa7 = pa7.tim3.Ch2;
//             pub const Pb5 = pb5.tim3.Ch2;
//             pub const Pc7 = pc7.tim3.Ch2;
//         };
//         pub const ch3 = struct {
//             pub const Pb0 = pb0.tim3.Ch3;
//             pub const Pc8 = pc8.tim3.Ch3;
//         };
//         pub const ch4 = struct {
//             pub const Pb1 = pb1.tim3.Ch4;
//             pub const Pc9 = pc9.tim3.Ch4;
//         };
//     };
//     pub const tim4 = struct {
//         pub const ch1 = struct {
//             pub const Pb6 = pb6.tim4.Ch1;
//         };
//         pub const ch2 = struct {
//             pub const Pb7 = pb7.tim4.Ch2;
//         };
//         pub const ch3 = struct {
//             pub const Pb8 = pb8.tim4.Ch3;
//         };
//         pub const ch4 = struct {
//             pub const Pb9 = pb9.tim4.Ch4;
//         };
//     };
//     pub const tim5 = struct {
//         pub const ch1 = struct {
//             pub const Pa0 = pa0.tim5.Ch1;
//         };
//         pub const ch2 = struct {
//             pub const Pa1 = pa1.tim5.Ch2;
//         };
//         pub const ch3 = struct {
//             pub const Pa2 = pa2.tim5.Ch3;
//         };
//         pub const ch4 = struct {
//             pub const Pa3 = pa3.tim5.Ch4;
//         };
//     };
//     pub const tim9 = struct {
//         pub const ch1 = struct {
//             pub const Pa2 = pa2.tim9.Ch1;
//         };
//         pub const ch2 = struct {
//             pub const Pa3 = pa3.tim9.Ch2;
//         };
//     };
//     pub const usart1 = struct {
//         pub const ck = struct {
//             pub const Pa8 = pa8.usart1.Ck;
//         };
//         pub const tx = struct {
//             pub const Pa9 = pa9.usart1.Tx;
//             pub const Pa15 = pa15.usart1.Tx;
//             pub const Pb6 = pb6.usart1.Tx;
//         };
//         pub const rx = struct {
//             pub const Pa10 = pa10.usart1.Rx;
//             pub const Pb3 = pb3.usart1.Rx;
//             pub const Pb7 = pb7.usart1.Rx;
//         };
//         pub const cts = struct {
//             pub const Pa11 = pa11.usart1.Cts;
//         };
//         pub const rts = struct {
//             pub const Pa12 = pa12.usart1.Rts;
//         };
//     };
//     pub const usart2 = struct {
//         pub const cts = struct {
//             pub const Pa0 = pa0.usart2.Cts;
//         };
//         pub const rts = struct {
//             pub const Pa1 = pa1.usart2.Rts;
//         };
//         pub const tx = struct {
//             pub const Pa2 = pa2.usart2.Tx;
//         };
//         pub const rx = struct {
//             pub const Pa3 = pa3.usart2.Rx;
//         };
//         pub const ck = struct {
//             pub const Pa4 = pa4.usart2.Ck;
//         };
//     };
//     pub const usart6 = struct {
//         pub const tx = struct {
//             pub const Pa11 = pa11.usart6.Tx;
//             pub const Pc6 = pc6.usart6.Tx;
//         };
//         pub const rx = struct {
//             pub const Pa12 = pa12.usart6.Rx;
//             pub const Pc7 = pc7.usart6.Rx;
//         };
//     };
//     pub const usb_otg_fs = struct {
//         pub const sof = struct {
//             pub const Pa8 = pa8.usb_otg_fs.Sof;
//         };
//         pub const vbus = struct {
//             pub const Pa9 = pa9.usb_otg_fs.Vbus;
//         };
//         pub const id = struct {
//             pub const Pa10 = pa10.usb_otg_fs.Id;
//         };
//         pub const dm = struct {
//             pub const Pa11 = pa11.usb_otg_fs.Dm;
//         };
//         pub const dp = struct {
//             pub const Pa12 = pa12.usb_otg_fs.Dp;
//         };
//     };
// };
// // -------------- gpio

// -------------- irq
pub const hal = @import("hal.zig");

pub const Irq = enum(u8) {
    // zig fmt: off
    wwdg               = 0,      // Window WatchDog Interrupt
    pvd                = 1,      // PVD through EXTI Line detection Interrupt
    tamp_stamp         = 2,      // Tamper and TimeStamp interrupts through the EXTI line
    rtc_wkup           = 3,      // RTC Wakeup interrupt through the EXTI line
    flash              = 4,      // FLASH global Interrupt
    rcc                = 5,      // RCC global Interrupt
    exti0              = 6,      // EXTI Line0 Interrupt
    exti1              = 7,      // EXTI Line1 Interrupt
    exti2              = 8,      // EXTI Line2 Interrupt
    exti3              = 9,      // EXTI Line3 Interrupt
    exti4              = 10,     // EXTI Line4 Interrupt
    dma1_stream0       = 11,     // DMA1 Stream 0 global Interrupt
    dma1_stream1       = 12,     // DMA1 Stream 1 global Interrupt
    dma1_stream2       = 13,     // DMA1 Stream 2 global Interrupt
    dma1_stream3       = 14,     // DMA1 Stream 3 global Interrupt
    dma1_stream4       = 15,     // DMA1 Stream 4 global Interrupt
    dma1_stream5       = 16,     // DMA1 Stream 5 global Interrupt
    dma1_stream6       = 17,     // DMA1 Stream 6 global Interrupt
    adc                = 18,     // ADC1, ADC2 and ADC3 global Interrupts
    exti9_5            = 23,     // External Line[9:5] Interrupts
    tim1_brk_tim9      = 24,     // TIM1 Break interrupt and TIM9 global interrupt
    tim1_up_tim10      = 25,     // TIM1 Update Interrupt and TIM10 global interrupt
    tim1_trg_com_tim11 = 26,     // TIM1 Trigger and Commutation Interrupt and TIM11 global interrupt
    tim1_cc            = 27,     // TIM1 Capture Compare Interrupt
    tim2               = 28,     // TIM2 global Interrupt
    tim3               = 29,     // TIM3 global Interrupt
    tim4               = 30,     // TIM4 global Interrupt
    i2c1_ev            = 31,     // I2C1 Event Interrupt
    i2c1_er            = 32,     // I2C1 Error Interrupt
    i2c2_ev            = 33,     // I2C2 Event Interrupt
    i2c2_er            = 34,     // I2C2 Error Interrupt
    spi1               = 35,     // SPI1 global Interrupt
    spi2               = 36,     // SPI2 global Interrupt
    usart1             = 37,     // USART1 global Interrupt
    usart2             = 38,     // USART2 global Interrupt
    exti15_10          = 40,     // External Line[15:10] Interrupts
    rtc_alarm          = 41,     // RTC Alarm (A and B) through EXTI Line Interrupt
    otg_fs_wkup        = 42,     // USB OTG FS Wakeup through EXTI line interrupt
    dma1_stream7       = 47,     // DMA1 Stream7 Interrupt
    sdio               = 49,     // SDIO global Interrupt
    tim5               = 50,     // TIM5 global Interrupt
    spi3               = 51,     // SPI3 global Interrupt
    dma2_stream0       = 56,     // DMA2 Stream 0 global Interrupt
    dma2_stream1       = 57,     // DMA2 Stream 1 global Interrupt
    dma2_stream2       = 58,     // DMA2 Stream 2 global Interrupt
    dma2_stream3       = 59,     // DMA2 Stream 3 global Interrupt
    dma2_stream4       = 60,     // DMA2 Stream 4 global Interrupt
    otg_fs             = 67,     // USB OTG FS global Interrupt
    dma2_stream5       = 68,     // DMA2 Stream 5 global interrupt
    dma2_stream6       = 69,     // DMA2 Stream 6 global interrupt
    dma2_stream7       = 70,     // DMA2 Stream 7 global interrupt
    usart6             = 71,     // USART6 global interrupt
    i2c3_ev            = 72,     // I2C3 event interrupt
    i2c3_er            = 73,     // I2C3 error interrupt
    fpu                = 81,     // FPU global interrupt
    spi4               = 84,     // SPI4 global Interrupt
    spi5               = 85,     // SPI5 global Interrupt
    // zig fmt: on

    pub fn enable(self: Irq) void {
        hal.irq.enable(@enumToInt(self));
    }
    pub fn disable(self: Irq) void {
        hal.irq.disable(@enumToInt(self));
    }
    pub fn setPriority(self: Irq, pri: u4) void {
        hal.irq.setPriority(@enumToInt(self), pri);
    }
};
// -------------- irq

// -------------- uart
const uart_hal = @import("uart.zig");

pub const uart = struct {
    pub fn Uart1(comptime config: uart_hal.Config, comptime freq: clk.Frequencies) type {
        const data = struct {
            pub const name = "USART1";
            pub const rcc = "APB2";
            pub const pin = struct {
                pub const tx = [_]type{ gpio.USART1.TX.PA9, gpio.USART1.TX.PA15, gpio.USART1.TX.PB6 };
                pub const rx = [_]type{ gpio.USART1.RX.PA10, gpio.USART1.RX.PB3, gpio.USART1.RX.PB7 };
            };

            pub const irq = Irq.usart1;

            pub const dma = struct {
                pub const controller = "DMA2";
                pub const rx = struct {
                    pub const stream = 5;
                    pub const channel = 4;
                    pub const irq = Irq.dma2_stream5;
                };
                pub const tx = struct {
                    pub const stream = 7;
                    pub const channel = 4;
                    pub const irq = Irq.dma2_stream7;
                };
            };
        };

        const bus_frequency = @field(freq, "apb" ++ data.rcc[3..]); // apb2 or apb1
        return uart_hal.UartX(data, config, bus_frequency);
    }
};
// -------------- uart
