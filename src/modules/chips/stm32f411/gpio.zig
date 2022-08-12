const std = @import("std");
const regs = @import("registers.zig").registers;

pub const Mode = enum(u2) {
    input = 0b00,
    output = 0b01,
    alternate_function = 0b10,
    analog = 0b11,
};

pub const OutputType = enum(u1) {
    push_pull = 0,
    open_drain = 1,
};

pub const OutputSpeed = enum(u2) {
    low = 0b00,
    medium = 0b01,
    fast = 0b10,
    high = 0b11,
};

pub const Pull = enum(u2) {
    none,
    up,
    down,
};

pub const AlternateFunction = enum(u4) {
    af0,
    af1,
    af2,
    af3,
    af4,
    af5,
    af6,
    af7,
    af8,
    af9,
    af10,
    af11,
    af12,
    af13,
    af14,
    af15,
};

pub const Port = enum(u8) {
    a = 'A',
    b = 'B',
    c = 'C',
    d = 'D',
    e = 'E',
    h = 'H',
};

pub const Config = struct {
    port: Port,
    pin: u4,
    mode: Mode = .input,
    output_type: OutputType = .push_pull,
    output_speed: OutputSpeed = .low,
    pull: Pull = .none,
    alternate_function: u4 = 0,

    fn parse(comptime c: Config) type {
        return struct {
            const name = "GPIO" ++ [_]u8{@enumToInt(c.port)}; // GPIOx = GPIOA or GPIOB or ...
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
        c.set("MODER", "MODER", @enumToInt(c.mode)); // regs.GPIOx.MODER.MODERy = z
        c.set("PUPDR", "PUPDR", @enumToInt(c.pull)); // regs.GPIOx.PUPDR.PUPDRy = z
        if (c.mode == .output) {
            c.initOutput();
        }
        if (c.mode == .alternate_function) {
            c.initAlternateFunction();
        }
    }

    fn initClock(comptime c: Config) void {
        setRegField(regs.RCC.AHB1ENR, c.parse().name ++ "EN", 1); // regs.RCC.AHB1ENR.GPIOxEN = 1
    }

    fn initOutput(comptime c: Config) void {
        c.set("OTYPER", "OT", @enumToInt(c.output_type)); // regs.GPIOx.OTYPER.OTy = z
        c.set("OSPEEDR", "OSPEEDR", @enumToInt(c.output_speed)); // regs.GPIOx.OSPEEDR.OSPEEDRy = z
    }

    fn initAlternateFunction(comptime c: Config) void {
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
};

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
