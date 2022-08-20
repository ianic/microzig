const std = @import("std");
const regs = @import("registers.zig").registers;
const irq = @import("irq.zig");

// Definition of available pins for stm32f411.
// From "RM0383 Rev 3" page 145:
//   "GPIO F/G/H/I/J/K (except GPIOH0 and GPIOH1) are not available in STM32F411xC/E."
//
// Example usage:
// pub const gpio = chip.gpio;
// ...
// pub const led = gpio.PA5.Output(.{});
// pub const button = gpio.PC13.Input(.{ .irq_enable = true });
//
pub const PA0 = Pin("PA0");
pub const PA1 = Pin("PA1");
pub const PA2 = Pin("PA2");
pub const PA3 = Pin("PA3");
pub const PA4 = Pin("PA4");
pub const PA5 = Pin("PA5");
pub const PA6 = Pin("PA6");
pub const PA7 = Pin("PA7");
pub const PA8 = Pin("PA8");
pub const PA9 = Pin("PA9");
pub const PA10 = Pin("PA10");
pub const PA11 = Pin("PA11");
pub const PA12 = Pin("PA12");
pub const PA13 = Pin("PA13");
pub const PA14 = Pin("PA14");
pub const PA15 = Pin("PA15");
pub const PB0 = Pin("PB0");
pub const PB1 = Pin("PB1");
pub const PB2 = Pin("PB2");
pub const PB3 = Pin("PB3");
pub const PB4 = Pin("PB4");
pub const PB5 = Pin("PB5");
pub const PB6 = Pin("PB6");
pub const PB7 = Pin("PB7");
pub const PB8 = Pin("PB8");
pub const PB9 = Pin("PB9");
pub const PB10 = Pin("PB10");
pub const PB11 = Pin("PB11");
pub const PB12 = Pin("PB12");
pub const PB13 = Pin("PB13");
pub const PB14 = Pin("PB14");
pub const PB15 = Pin("PB15");
pub const PC0 = Pin("PC0");
pub const PC1 = Pin("PC1");
pub const PC2 = Pin("PC2");
pub const PC3 = Pin("PC3");
pub const PC4 = Pin("PC4");
pub const PC5 = Pin("PC5");
pub const PC6 = Pin("PC6");
pub const PC7 = Pin("PC7");
pub const PC8 = Pin("PC8");
pub const PC9 = Pin("PC9");
pub const PC10 = Pin("PC10");
pub const PC11 = Pin("PC11");
pub const PC12 = Pin("PC12");
pub const PC13 = Pin("PC13");
pub const PC14 = Pin("PC14");
pub const PC15 = Pin("PC15");
pub const PD0 = Pin("PD0");
pub const PD1 = Pin("PD1");
pub const PD2 = Pin("PD2");
pub const PD3 = Pin("PD3");
pub const PD4 = Pin("PD4");
pub const PD5 = Pin("PD5");
pub const PD6 = Pin("PD6");
pub const PD7 = Pin("PD7");
pub const PD8 = Pin("PD8");
pub const PD9 = Pin("PD9");
pub const PD10 = Pin("PD10");
pub const PD11 = Pin("PD11");
pub const PD12 = Pin("PD12");
pub const PD13 = Pin("PD13");
pub const PD14 = Pin("PD14");
pub const PD15 = Pin("PD15");
pub const PE0 = Pin("PE0");
pub const PE1 = Pin("PE1");
pub const PE2 = Pin("PE2");
pub const PE3 = Pin("PE3");
pub const PE4 = Pin("PE4");
pub const PE5 = Pin("PE5");
pub const PE6 = Pin("PE6");
pub const PE7 = Pin("PE7");
pub const PE8 = Pin("PE8");
pub const PE9 = Pin("PE9");
pub const PE10 = Pin("PE10");
pub const PE11 = Pin("PE11");
pub const PE12 = Pin("PE12");
pub const PE13 = Pin("PE13");
pub const PE14 = Pin("PE14");
pub const PE15 = Pin("PE15");
pub const PH0 = Pin("PH0");
pub const PH1 = Pin("PH1");
pub const PH2 = Pin("PH2");

pub const InputConfig = struct {
    pull: Pull = .none,
    irq_enable: bool = true,
    irq_priority: u4 = 0xf,
    irq_trigger: IrqTrigger = .falling,
};

pub const OutputConfig = struct {
    pull: Pull = .none,
    speed: OutputSpeed = .low,
    @"type": OutputType = .push_pull,
};

pub const AlternateFunctionConfig = struct {
    pull: Pull = .none,
    af: u4,
};

pub const Pull = enum(u2) {
    none,
    up,
    down,
};

pub const OutputType = enum(u1) {
    push_pull,
    open_drain,
};

pub const OutputSpeed = enum(u2) {
    low,
    medium,
    fast,
    high,
};

pub const IrqTrigger = enum {
    falling,
    rising,
    both,
};

const Mode = enum(u2) {
    input,
    output,
    alternate_function,
    analog,
};

// spec is already asserted that it is valid port pin combination
fn ParsedPin(comptime spec: []const u8) type {
    return struct {
        const pin_number: comptime_int = std.fmt.parseInt(u4, spec[2..], 10) catch unreachable;
        const port_number = @intCast(u4, spec[1..2][0] - 65); // A = 0, B = 1, ...
        const name = "GPIO" ++ spec[1..2]; // GPIOx = GPIOA or GPIOB or ...
        const reg = @field(regs, name); // regs.GPIOx
        const suffix = std.fmt.comptimePrint("{d}", .{pin_number}); // pin as string '0', '1', ...
        const cr_suffix = std.fmt.comptimePrint("{d}", .{pin_number / 4 + 1}); // 0-3 => '1', 4-7 => '2', ...

        // regs.GPIOx.[reg_name].[field]y = value
        fn set(comptime reg_name: []const u8, comptime prefix: []const u8, value: anytype) void {
            @field(reg, reg_name).set(prefix ++ suffix, value);
        }

        fn initClock() void {
            regs.RCC.AHB1ENR.set(name ++ "EN", 1); // regs.RCC.AHB1ENR.GPIOxEN = 1
        }

        fn initMode(mode: Mode, pull: Pull) void {
            set("MODER", "MODER", @enumToInt(mode)); // regs.GPIOx.MODER.MODERy = z
            set("PUPDR", "PUPDR", @enumToInt(pull)); // regs.GPIOx.PUPDR.PUPDRy = z
        }

        fn initOutput(comptime c: OutputConfig) void {
            initMode(.output, c.pull);
            set("OTYPER", "OT", @enumToInt(c.type)); // regs.GPIOx.OTYPER.OTy = z
            set("OSPEEDR", "OSPEEDR", @enumToInt(c.speed)); // regs.GPIOx.OSPEEDR.OSPEEDRy = z
        }

        fn initInput(comptime c: InputConfig) void {
            initMode(.input, c.pull);
        }

        fn initAlternateFunction(comptime c: AlternateFunctionConfig) void {
            initMode(.alternate_function, c.pull);
            const reg_name = if (pin_number < 8) "AFRL" else "AFRH";
            set(reg_name, reg_name, c.af); // regs.GPIOx.AFRL.AFRLy = z
        }

        fn read() u1 {
            return @field(reg.IDR.read(), "IDR" ++ suffix); // regs.GPIOx.IDR.read().IDRy
        }

        fn write(value: u1) void {
            switch (value) {
                0 => set("BSRR", "BR", 1), // regs.GPIOx.BSRR.BRy = 1
                1 => set("BSRR", "BS", 1), // regs.GPIOx.BSRR.BSy = 1
            }
        }
    };
}

// spec is pin name e.g. "PA0", "PA1"...
fn Pin(comptime spec: []const u8) type {
    const pp = ParsedPin(spec);
    return struct {
        pub fn Input(comptime c: InputConfig) type {
            const exti = Exti(pp);
            return struct {
                pub fn init() void {
                    pp.initClock();
                    pp.initInput(c);
                    if (c.irq_enable) {
                        exti.enable(c.irq_trigger);
                        exti.setPriority(c.irq_priority);
                    }
                }
                pub const read = pp.read;
                pub fn irq_pending() bool {
                    if (!c.irq_enable) {
                        return false;
                    }
                    return exti.pending();
                }
            };
        }
        pub fn Output(comptime c: OutputConfig) type {
            return struct {
                pub const write = pp.write;
                pub const read = pp.read;

                pub fn init() void {
                    pp.initClock();
                    pp.initOutput(c);
                }
                pub fn setToHigh() void {
                    pp.write(1);
                }
                pub fn setToLow() void {
                    pp.write(0);
                }
                pub fn toggle() void {
                    switch (pp.read()) {
                        1 => pp.write(0),
                        0 => pp.write(1),
                    }
                }
                pub fn on() void {
                    pp.write(1);
                }
                pub fn off() void {
                    pp.write(0);
                }
            };
        }
        pub fn AlternateFunction(comptime c: AlternateFunctionConfig) type {
            return struct {
                pub fn init() void {
                    pp.initClock();
                    pp.initAlternateFunction(c);
                }
            };
        }
        pub fn Analog() type {
            return struct {
                pub fn init() void {
                    pp.initClock();
                    pp.initMode(.analog, .none);
                }
            };
        }
    };
}

// exti interrupt enabling for pin
fn Exti(comptime pp: anytype) type {
    const irqn = switch (pp.pin_number) {
        10...15 => .exti15_10,
        5...9 => .exti9_5,
        else => @intToEnum(irq.Irq, pp.pin_number + 6),
    };
    return struct {
        pub fn enable(trigger: IrqTrigger) void {
            regs.RCC.APB2ENR.modify(.{ .SYSCFGEN = 1 }); // Enable SYSCFG Clock

            // regs.SYSCFG.EXTICR[cr_reg_no].modify(.{ .EXTI[suffix] = [port_number] });
            const cr_reg = @field(regs.SYSCFG, "EXTICR" ++ pp.cr_suffix);
            cr_reg.set("EXTI" ++ pp.suffix, pp.port_number);

            if (trigger == .falling or .trigger == .both) {
                regs.EXTI.FTSR.set("TR" ++ pp.suffix, 1); // regs.EXTI.FTSR.modify(.{ .TR[suffix] = 1 });
            }
            if (trigger == .rising or .trigger == .both) {
                regs.EXTI.RTSR.set("TR" ++ pp.suffix, 1); // regs.EXTI.RTSR.modify(.{ .TR[suffix] = 1 });
            }
            regs.EXTI.IMR.set("MR" ++ pp.suffix, 1); // regs.EXTI.IMR.modify(.{ .MR[suffix] = 1 });

            irq.enable(irqn);
        }

        // regs.NVIC.IPR[x].modify(.{ .IPR_N[y] = value });
        pub fn setPriority(pri: u4) void {
            irq.setPriority(irqn, pri);
        }

        // get and clear pending exti interrupt
        // regs.EXTI.PR.read().PR[suffix]
        pub fn pending() bool {
            const field_name = "PR" ++ pp.suffix;
            var reg_value = regs.EXTI.PR.read();
            const is_pending = @field(reg_value, field_name) == 1;
            if (is_pending) {
                // clear pending bit
                @field(reg_value, field_name) = 1;
                regs.EXTI.PR.write(reg_value);
            }
            return is_pending;
        }
    };
}

test "irq params" {
    const pp = ParsedPin("PC13");
    try std.testing.expectEqual(4, pp.pin_number / 4 + 1);
    try std.testing.expectEqual(2, pp.port_number);

    const pp2 = ParsedPin("PH5");
    try std.testing.expectEqual(2, pp2.pin_number / 4 + 1);
    try std.testing.expectEqual(0b111, pp2.port_number);
}
