const std = @import("std");
const regs = @import("registers.zig").registers;
const irq = @import("irq.zig");

pub const Mode = enum(u2) {
    input,
    output,
    alternate_function,
    analog,
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

pub const Pull = enum(u2) {
    none,
    up,
    down,
};

pub const InputConfig = struct {
    pull: Pull = .none,
    irq_enable: bool = true,
    irq_priority: u8 = 0xf0,
};

pub const OutputConfig = struct {
    pull: Pull = .none,
    speed: OutputSpeed = .low,
    @"type": OutputType = .push_pull,
};

pub const AFConfig = struct {
    pull: Pull = .none,
    af: u4,
};

// spec is already asserted that it is valid port pin combination
fn ParsedPin(comptime spec: []const u8) type {
    return struct {
        pub const pin_number: comptime_int = std.fmt.parseInt(u4, spec[2..], 10) catch unreachable;
        pub const port_number = @intCast(u4, spec[1..2][0] - 65); // A = 0, B = 1, ...
        pub const name = "GPIO" ++ spec[1..2]; // GPIOx = GPIOA or GPIOB or ...
        pub const reg = @field(regs, name); // regs.GPIOx
        pub const suffix = std.fmt.comptimePrint("{d}", .{pin_number}); // pin as string '0', '1', ...
        pub const cr_suffix = std.fmt.comptimePrint("{d}", .{pin_number / 4 + 1}); // 0-3 => '1', 4-7 => '2', ...

        // regs.GPIOx.[reg_name].[field]y = value
        fn set(
            comptime reg_name: []const u8,
            comptime field: []const u8,
            value: anytype,
        ) void {
            setRegField(@field(reg, reg_name), field ++ suffix, value);
        }

        fn initClock() void {
            setRegField(regs.RCC.AHB1ENR, name ++ "EN", 1); // regs.RCC.AHB1ENR.GPIOxEN = 1
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

        fn initAlternateFunction(comptime c: AFConfig) void {
            initMode(.alternate_function, c.none);
            if (c.af < 8) {
                c.set("AFRL", "AFRL", @enumToInt(c.af)); // regs.GPIOx.AFRL.AFRLy = z
            } else {
                c.set("AFRH", "AFRH", @enumToInt(c.af)); // regs.GPIOx.AFRH.AFRHy = z
            }
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

pub fn Pin(comptime spec: []const u8) type {
    const pp = ParsedPin(spec);
    return struct {
        pub fn Input(comptime c: InputConfig) type {
            const exti = Exti(pp);
            return struct {
                pub fn init() void {
                    pp.initClock();
                    pp.initInput(c);
                    if (c.irq_enable) {
                        exti.enable();
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
        pub fn AlternateFunction(comptime c: AFConfig) type {
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

fn Exti(comptime pp: anytype) type {
    const irqn = switch (pp.pin_number) {
        10...15 => .exti15_10,
        5...9 => .exti9_5,
        else => @intToEnum(irq.Irq, pp.pin_number + 6),
    };
    return struct {
        pub fn enable() void {
            regs.RCC.APB2ENR.modify(.{ .SYSCFGEN = 1 }); // Enable SYSCFG Clock

            // regs.SYSCFG.EXTICR[cr_reg_no].modify(.{ .EXTI[suffix] = [port_number] });
            const cr_reg = @field(regs.SYSCFG, "EXTICR" ++ pp.cr_suffix);
            setRegField(cr_reg, "EXTI" ++ pp.suffix, pp.port_number);

            setRegField(regs.EXTI.FTSR, "TR" ++ pp.suffix, 1); // regs.EXTI.FTSR.modify(.{ .TR[suffix] = 1 });
            setRegField(regs.EXTI.IMR, "MR" ++ pp.suffix, 1); // regs.EXTI.IMR.modify(.{ .MR[suffix] = 1 });

            irq.enable(irqn);
        }

        // regs.NVIC.IPR[x].modify(.{ .IPR_N[y] = value });
        pub fn setPriority(value: u8) void {
            irq.setPriority(irqn, value);
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

fn setRegField(reg: anytype, comptime field_name: anytype, value: anytype) void {
    var temp = reg.read();
    @field(temp, field_name) = value;
    reg.write(temp);
}

test "irq params" {
    const pp = ParsedPin("PC13");
    try std.testing.expectEqual(4, pp.pin_number / 4 + 1);
    try std.testing.expectEqual(2, pp.port_number);

    const pp2 = ParsedPin("PH5");
    try std.testing.expectEqual(2, pp2.pin_number / 4 + 1);
    try std.testing.expectEqual(0b111, pp2.port_number);
}
