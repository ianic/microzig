const std = @import("std");
const regs = @import("registers.zig").registers;

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

pub const Port = enum(u8) {
    a,
    b,
    c,
    d,
    e,
    h,
};

pub const InputConfig = struct {
    pull: Pull = .none,
    irq_enabled: bool = true,
    irq_priority: u4 = 0xf,
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

pub const Config = struct {
    port: Port,
    pin: u4,
    mode: Mode = .input,
    output_type: OutputType = .push_pull,
    output_speed: OutputSpeed = .low,
    pull: Pull = .none,
    alternate_function: u4 = 0,

    //output: OutputMode = undefined,
    //input: InputMode = undefined,

    fn parse(comptime c: Config) type {
        return struct {
            const name = "GPIO" ++ [_]u8{@enumToInt(c.port) + 65}; // GPIOx = GPIOA or GPIOB or ...
            const reg = @field(regs, name); // regs.GPIOx
            const pin = std.fmt.comptimePrint("{d}", .{c.pin}); // pin as string '0', '1', ...
        };
    }

    // regs.GPIOx.[reg_name].[field]y = value
    fn set(
        comptime c: Config,
        comptime reg_name: []const u8,
        comptime field: []const u8,
        value: anytype,
    ) void {
        const p = c.parse();
        setRegField(@field(p.reg, reg_name), field ++ p.pin, value);
    }

    fn init(comptime c: Config) void {
        c.initClock();
        c.initMode();
        c.initOutput();
        c.initAlternateFunction();
    }

    fn initClock(comptime c: Config) void {
        setRegField(regs.RCC.AHB1ENR, c.parse().name ++ "EN", 1); // regs.RCC.AHB1ENR.GPIOxEN = 1
    }

    fn initMode(comptime c: Config) void {
        c.set("MODER", "MODER", @enumToInt(c.mode)); // regs.GPIOx.MODER.MODERy = z
        c.set("PUPDR", "PUPDR", @enumToInt(c.pull)); // regs.GPIOx.PUPDR.PUPDRy = z
    }

    fn initOutput(comptime c: Config) void {
        if (c.mode != .output) {
            return;
        }
        c.set("OTYPER", "OT", @enumToInt(c.output_type)); // regs.GPIOx.OTYPER.OTy = z
        c.set("OSPEEDR", "OSPEEDR", @enumToInt(c.output_speed)); // regs.GPIOx.OSPEEDR.OSPEEDRy = z
    }

    fn initAlternateFunction(comptime c: Config) void {
        if (c.mode != .alternate_function) {
            return;
        }
        if (c.alternate_function < 8) {
            c.set("AFRL", "AFRL", @enumToInt(c.alternate_function)); // regs.GPIOx.AFRL.AFRLy = z
        } else {
            c.set("AFRH", "AFRH", @enumToInt(c.alternate_function)); // regs.GPIOx.AFRH.AFRHy = z
        }
    }

    fn read(comptime c: Config) u1 {
        const p = c.parse();
        return @field(p.reg.IDR.read(), "IDR" ++ p.pin); // read regs.GPIOx.IDR.read().IDRy
    }

    fn write(comptime c: Config, value: u1) void {
        switch (value) {
            0 => c.set("BSRR", "BR", 1), // regs.GPIOx.BSRR.BRy = 1
            1 => c.set("BSRR", "BS", 1), // regs.GPIOx.BSRR.BSy = 1
        }
    }

    pub fn input(comptime c: Config, comptime ic: InputConfig) type {
        _ = ic;
        return struct {
            pub fn init() void {
                c.initClock();
                c.initInput();
            }
            pub const read = c.read;
        };
    }
};

pub fn parsePin(comptime spec: []const u8) type {
    const invalid_format_msg = "The given pin '" ++ spec ++ "' has an invalid format. Pins must follow the format \"P{Port}{Pin}\" scheme.";

    if (spec[0] != 'P')
        @compileError(invalid_format_msg);
    if (spec[1] < 'A' or spec[1] > 'H')
        @compileError(invalid_format_msg);

    return struct {
        const pin_number: comptime_int = std.fmt.parseInt(u4, spec[2..], 10) catch @compileError(invalid_format_msg);
        /// 'A'...'H'
        const gpio_port_name = spec[1..2];
        const gpio_port = @field(regs, "GPIO" ++ gpio_port_name);
        const suffix = std.fmt.comptimePrint("{d}", .{pin_number});
    };
}

//fn ParsedPin(port: Port, pn: u4) type {
//    return struct {
//        const name = "GPIO" ++ [_]u8{@enumToInt(port) + 65}; // GPIOx = GPIOA or GPIOB or ...
//        const reg = @field(regs, name); // regs.GPIOx
//        const suffix = std.fmt.comptimePrint("{d}", .{pn}); // pin as string '0', '1', ...
fn ParsedPin(comptime spec: []const u8) type {
    const invalid_format_msg = "The given pin '" ++ spec ++ "' has an invalid format. Pins must follow the format \"P{Port}{Pin}\" scheme.";

    if (spec[0] != 'P')
        @compileError(invalid_format_msg);
    if (spec[1] < 'A' or spec[1] > 'H')
        @compileError(invalid_format_msg);

    const pin_number: comptime_int = std.fmt.parseInt(u4, spec[2..], 10) catch @compileError(invalid_format_msg);
    return struct {
        const name = "GPIO" ++ spec[1..2]; // GPIOx = GPIOA or GPIOB or ...
        const reg = @field(regs, name); // regs.GPIOx
        const suffix = std.fmt.comptimePrint("{d}", .{pin_number}); // pin as string '0', '1', ...

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

pub fn pin(comptime cfg: Config) type {
    switch (cfg.mode) {
        .input => return struct {
            pub const init = cfg.init;
            pub const read = cfg.read;
        },
        .output => return struct {
            pub const init = cfg.init;
            pub fn setToHigh() void {
                cfg.write(1);
            }
            pub fn setToLow() void {
                cfg.write(0);
            }
            pub fn toggle() void {
                switch (cfg.read()) {
                    1 => cfg.write(0),
                    0 => cfg.write(1),
                }
            }
            pub fn on() void {
                cfg.write(1);
            }
            pub fn off() void {
                cfg.write(0);
            }
        },
        .alternate_function => return struct {
            pub const init = cfg.init;
        },
        .analog => return struct {
            pub const init = cfg.init;
        },
    }
}

pub fn Pin(comptime spec: []const u8) type {
    const pp = ParsedPin(spec);
    return struct {
        pub fn Input(comptime c: InputConfig) type {
            return struct {
                pub fn init() void {
                    pp.initClock();
                    pp.initInput(c);
                }
                pub const read = pp.read;
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

fn setRegField(reg: anytype, comptime field_name: anytype, value: anytype) void {
    var temp = reg.read();
    @field(temp, field_name) = value;
    reg.write(temp);
}

// test "port name" {
//     const cfg = Config{
//         .port = .b,
//         .pin = 9,
//     };
//     try std.testing.expectEqualStrings("GPIOB", cfg.port.name());

//     try std.testing.expectEqualStrings("GPIOB", Port.b.name());
//     try std.testing.expectEqualStrings("GPIOC", Port.c.name());
// }

// test "set" {
//     //set("MODER", "MODER", .b, 8, 0b01);
//     const cfg = Config{
//         .port = .b,
//         .pin = 8,
//     };
//     cfg.init();
// }

test "union for mode" {
    const led = (Config{ .port = .a, .pin = 0xA }).input(.{});
    _ = led;

    const led2 = Pin("PA1").Output(.{});
    led2.init();
}
