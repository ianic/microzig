const std = @import("std");
const gpio = @import("gpio.zig");
const regs = @import("registers.zig").registers;
const clk = @import("clock.zig");
const Frequencies = clk.Frequencies;

pub const Config = struct {
    // at least one of this is required
    tx: ?type = null,
    rx: ?type = null,

    clock_frequencies: Frequencies,
    // defaults
    baud_rate: u32 = 9600,
    stop_bits: StopBits = .one,
    parity: ?Parity = null,
    data_bits: DataBits = .eight,
};

pub const DataBits = enum {
    seven,
    eight,
    nine,
};

/// uses the values of USART_CR2.STOP
pub const StopBits = enum(u2) {
    one = 0b00,
    half = 0b01,
    two = 0b10,
    one_and_half = 0b11,
};

/// uses the values of USART_CR1.PS
pub const Parity = enum(u1) {
    even = 0,
    odd = 1,
};

const uart1_data = struct {
    const index = 1;
    const name = "USART1";
    const pin = struct {
        const af = 7;
        const tx = [_]type{ gpio.PA9, gpio.PA15, gpio.PB6 };
        const rx = [_]type{ gpio.PA10, gpio.PB3, gpio.PB7 };
    };
    const clock = struct {
        const reg = "APB2";
        const bus = "apb2";
    };
};

pub fn Uart1(comptime pins: Config) type {
    return UartX(uart1_data, pins);
}

fn usartDiv(bus_frequency: u32, baud_rate: u32) u16 {
    // Despite the reference manual talking about fractional calculation and other buzzwords,
    // it is actually just a simple divider. Just ignore DIV_Mantissa and DIV_Fraction and
    // set the result of the division as the lower 16 bits of BRR.
    // TODO: We assume the default OVER8=0 configuration above (i.e. 16x oversampling).
    // TODO: Do some checks to see if the baud rate is too high (or perhaps too low)
    // TODO: Do a rounding div, instead of a truncating div?
    return @intCast(u16, @divTrunc(bus_frequency, baud_rate));
}

test "usart div" {
    try std.testing.expectEqual(100_000_000, @field(clk.hsi_100.frequencies, "apb2"));
    try std.testing.expectEqual(50_000_000, @field(clk.hsi_100.frequencies, "apb1"));

    try std.testing.expectEqual(usartDiv(clk.hsi_100.frequencies.apb2, 9600), 10416);
    try std.testing.expectEqual(usartDiv(clk.hsi_100.frequencies.apb1, 9600), 5208);
    try std.testing.expectEqual(usartDiv(clk.hsi_100.frequencies.apb2, 153600), 651);
}

fn wordLength(comptime config: Config) u1 {
    // Per the reference manual, M means
    // - 0: 1 start bit, 8 data bits (7 data + 1 parity, or 8 data), n stop bits, the chip default
    // - 1: 1 start bit, 9 data bits (8 data + 1 parity, or 9 data), n stop bits
    return if (config.data_bits == .nine or (config.data_bits == .eight and config.parity != null)) 1 else 0;
}

fn UartX(comptime data: type, comptime config: Config) type {
    assertValidPins(data, config.tx, config.rx);
    assertConfig(config);

    const usartdiv = usartDiv(@field(config.clock_frequencies, data.clock.bus), config.baud_rate);

    // TODO dodaj ostale funkcije
    // ako nije definirao rx nemoj mu napraviti read funkciju ili nesto tako
    // vratis razliciti struct iz init
    // uf oce mi to razjebati onaj type koji sam zapamtio u main

    return struct {
        parity_read_mask: u8,

        const Self = @This();

        pub fn init() Self {
            // The following must all be written when the USART is disabled (UE=0).
            if (@field(regs, data.name).CR1.read().UE == 1) {
                @field(regs, data.name).CR1.modify(.{ .UE = 0 });
                // TODO: do we need something more here
            }

            // enable the USART clock:  reg.RCC.APBxENR.modify(.{.USARTyEN = 1});
            setRegField(@field(regs.RCC, data.clock.reg ++ "ENR"), data.name ++ "EN", 1);

            // clear configuration to its default
            @field(regs, data.name).CR1.raw = 0;
            @field(regs, data.name).CR2.raw = 0;
            @field(regs, data.name).CR3.raw = 0;

            // set word length
            @field(regs, data.name).CR1.modify(.{ .M = wordLength(config) });

            // set parity
            if (config.parity) |parity| {
                @field(regs, data.name).CR1.modify(.{ .PCE = 1, .PS = @enumToInt(parity) });
            } // otherwise, no need to set no parity since we reset Control Registers above, and it's the default

            // set number of stop bits
            @field(regs, data.name).CR2.modify(.{ .STOP = @enumToInt(config.stop_bits) });
            // set baud rate
            @field(regs, data.name).BRR.raw = usartdiv;

            // enable transmitter, receiver and USART
            if (config.tx) |pin| {
                pin.AlternateFunction(.{ .af = data.pin.af }).init();
                @field(regs, data.name).CR1.modify(.{ .TE = 1 });
            }
            if (config.rx) |pin| {
                pin.AlternateFunction(.{ .af = data.pin.af }).init();
                @field(regs, data.name).CR1.modify(.{ .RE = 1 });
            }
            @field(regs, data.name).CR1.modify(.{ .UE = 1 });

            // For code simplicity, at cost of one or more register reads,
            // we read back the actual configuration from the registers,
            // instead of using the `config` values.
            return readFromRegisters();
        }

        fn readFromRegisters() Self {
            const cr1 = @field(regs, data.name).CR1.read();
            // As documented in `init()`, M0==1 means 'the 9th bit (not the 8th bit) is the parity bit'.
            // So we always mask away the 9th bit, and if parity is enabled and it is in the 8th bit,
            // then we also mask away the 8th bit.
            return Self{ .parity_read_mask = if (cr1.PCE == 1 and cr1.M == 0) 0x7F else 0xFF };
        }

        pub fn canWrite(_: Self) bool {
            return @field(regs, data.name).SR.read().TXE == 1;
        }

        pub fn tx(self: Self, ch: u8) void {
            while (!self.canWrite()) {} // Wait for Previous transmission
            @field(regs, data.name).DR.modify(ch);
        }

        pub fn txflush(_: Self) void {
            while (@field(regs, data.name).SR.read().TC == 0) {}
        }

        pub fn canRead(_: Self) bool {
            return @field(regs, data.name).SR.read().RXNE == 1;
        }

        pub fn rx(self: Self) u8 {
            while (!self.canRead()) {} // Wait till the data is received
            const data_with_parity_bit: u9 = @field(regs, data.name).DR.read();
            return @intCast(u8, data_with_parity_bit & self.parity_read_mask);
        }
    };
}

fn assertConfig(comptime config: Config) void {
    if ((config.data_bits == .nine and config.parity != null) or
        (config.data_bits == .seven and config.parity == null))
    {
        @compileError("unsupported word size");
    }
}

test "assertConfig" {
    assertConfig(.{ .baud_rate = 1, .data_bits = .nine, .clock_frequencies = clk.hsi_100.frequencies });
    assertConfig(.{ .baud_rate = 1, .data_bits = .seven, .parity = .odd, .clock_frequencies = clk.hsi_100.frequencies });
}

fn assertValidPins(comptime data: type, comptime pin_tx: ?type, comptime pin_rx: ?type) void {
    if (pin_tx == null and pin_rx == null) {
        @compileError("UART pins are not specified");
    }
    if (pin_tx) |tx| brk: {
        inline for (data.pin.tx) |valid_tx| {
            if (valid_tx == tx) {
                break :brk;
            }
        }
        @compileError(comptime std.fmt.comptimePrint("Tx pin {?} is not valid for UART{}", .{ tx, data.index }));
    }
    if (pin_rx) |rx| brk: {
        inline for (data.pin.rx) |valid_rx| {
            if (valid_rx == rx) {
                break :brk;
            }
        }
        @compileError(comptime std.fmt.comptimePrint("Rx pin {?} is not valid for UART{}", .{ rx, data.index }));
    }
}

test "assertValidPins" {
    assertValidPins(uart1_data, gpio.PA9, null);
    assertValidPins(uart1_data, null, gpio.PB7);
    assertValidPins(uart1_data, gpio.PA9, gpio.PA10);
    assertValidPins(uart1_data, gpio.PA15, gpio.PB3);
    assertValidPins(uart1_data, gpio.PB6, gpio.PB7);

    // afs = assertValidPins(2, .{ .tx = gpio.PA2, .rx = gpio.PA3 });
    // try std.testing.expectEqual(afs.tx.?, 7);
    // try std.testing.expectEqual(afs.rx.?, 7);

    // afs = assertValidPins(6, .{ .tx = gpio.PA11, .rx = gpio.PA12 });
    // try std.testing.expectEqual(afs.tx.?, 8);
    // try std.testing.expectEqual(afs.rx.?, 8);
    // afs = assertValidPins(6, .{ .tx = gpio.PC6, .rx = gpio.PC7 });
    // try std.testing.expectEqual(afs.tx.?, 8);
    // try std.testing.expectEqual(afs.rx.?, 8);
}

fn setRegField(reg: anytype, comptime field_name: anytype, value: anytype) void {
    var temp = reg.read();
    @field(temp, field_name) = value;
    reg.write(temp);
}
