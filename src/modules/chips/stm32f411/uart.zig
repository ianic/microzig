const std = @import("std");
const gpio = @import("gpio.zig");
const irq = @import("irq.zig");
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

    dma_enable: bool = false,

    const Self = @This();

    fn parity_control_enable(comptime self: Self) u1 {
        return if (self.parity != null) 1 else 0;
    }
    fn parity_selection(comptime self: Self) u1 {
        return if (self.parity != null) @enumToInt(self.parity) else 0;
    }
    fn wordLength(comptime self: Config) u1 {
        // Per the reference manual, M means
        // - 0: 1 start bit, 8 data bits (7 data + 1 parity, or 8 data), n stop bits, the chip default
        // - 1: 1 start bit, 9 data bits (8 data + 1 parity, or 9 data), n stop bits
        return if (self.data_bits == .nine or (self.data_bits == .eight and self.parity != null)) 1 else 0;
    }
    fn tx_enable(comptime self: Config) u1 {
        return if (self.tx != null) 1 else 0;
    }
    fn rx_enable(comptime self: Config) u1 {
        return if (self.rx != null) 1 else 0;
    }
    fn parity_read_mask(comptime self: Config) u8 {
        // m ==1 means 'the 9th bit (not the 8th bit) is the parity bit'.
        // So we always mask away the 9th bit, and if parity is enabled and it is in the 8th bit,
        // then we also mask away the 8th bit.
        return if (self.parity != null and self.wordLength() == 0) 0x7F else 0xFF;
    }
    fn usartdiv(comptime self: Self, comptime clock_bus: []const u8) u16 {
        // Despite the reference manual talking about fractional calculation and other buzzwords,
        // it is actually just a simple divider. Just ignore DIV_Mantissa and DIV_Fraction and
        // set the result of the division as the lower 16 bits of BRR.
        // TODO: We assume the default OVER8=0 configuration above (i.e. 16x oversampling).
        // TODO: Do some checks to see if the baud rate is too high (or perhaps too low)
        // TODO: Do a rounding div, instead of a truncating div?
        const bus_frequency = @field(self.clock_frequencies, clock_bus);
        return @intCast(u16, @divTrunc(bus_frequency, self.baud_rate));
    }
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

pub const IrqEnableDisable = enum {
    enable,
    disable,
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
    const dma = struct {
        const controller = "DMA2";
        const tx = struct {
            const stream = 7;
            const channel = 4;
        };
        const rx = struct {
            const stream = 2;
            const channel = 4;
        };
    };
    const irqn = irq.Irq.usart1;
};

pub fn Uart1(comptime config: Config) type {
    return UartX(uart1_data, config);
}

test "Config.usartdiv" {
    try std.testing.expectEqual(100_000_000, @field(clk.hsi_100.frequencies, "apb2"));
    try std.testing.expectEqual(50_000_000, @field(clk.hsi_100.frequencies, "apb1"));

    const cfg1 = Config{ .clock_frequencies = clk.hsi_100.frequencies };
    try std.testing.expectEqual(cfg1.usartdiv("apb2"), 10416);
    try std.testing.expectEqual(cfg1.usartdiv("apb1"), 5208);

    const cfg2 = Config{ .baud_rate = 153600, .clock_frequencies = clk.hsi_100.frequencies };
    try std.testing.expectEqual(cfg2.usartdiv("apb2"), 651);
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

    if (config.dma_enable) {
        return Dma(data, config);
    } else {
        return Pooling(data, config);
    }
}

fn Pooling(comptime data: type, comptime config: Config) type {
    //const usartdiv = usartDiv(@field(config.clock_frequencies, data.clock.bus), config.baud_rate);

    // regs.USARTx
    const reg = @field(regs, data.name);

    return struct {
        parity_read_mask: u8,

        const Self = @This();

        pub fn init() Self {
            // The following must all be written when the USART is disabled (UE=0).
            if (reg.CR1.read().UE == 1) {
                reg.CR1.modify(.{ .UE = 0 });
                // TODO: do we need something more here
            }

            // enable the USART clock: reg.RCC.APBxENR.modify(.{.USARTyEN = 1});
            @field(regs.RCC, data.clock.reg ++ "ENR").set(data.name ++ "EN", 1);

            // clear configuration to its default
            reg.CR1.raw = 0;
            reg.CR2.raw = 0;
            reg.CR3.raw = 0;

            reg.BRR.raw = config.usartdiv(data.clock.bus); // set baud rate
            reg.CR1.modify(.{
                .M = config.wordLength(),
                .PCE = config.parity_control_enable(),
                .PS = config.parity_selection(),
                .TE = config.tx_enable(),
                .RE = config.rx_enable(),
            });
            reg.CR2.modify(.{ .STOP = @enumToInt(config.stop_bits) }); // set number of stop bits
            if (config.dma_enable) {
                reg.CR3.modify(.{
                    .DMAT = 1, // enable dma transmitter
                    .DMAR = 1, // enable dma receiver
                });
            }

            // configure gpio pins to required alternate function
            if (config.tx) |pin| { // transmitter
                pin.AlternateFunction(.{ .af = data.pin.af }).init();
            }
            if (config.rx) |pin| { // receiver
                pin.AlternateFunction(.{ .af = data.pin.af }).init();
            }

            reg.CR1.modify(.{ .UE = 1 }); // enable the USART
            irq.enable(data.irqn);

            return Self{ .parity_read_mask = config.parity_read_mask() };
        }

        pub fn txIrq(_: Self, v: IrqEnableDisable) void {
            switch (v) {
                .enable => reg.CR1.modify(.{ .TXEIE = 1 }),
                .disable => reg.CR1.modify(.{ .TXEIE = 0 }),
            }
        }

        pub fn rxIrq(_: Self, v: IrqEnableDisable) void {
            switch (v) {
                .enable => reg.CR1.modify(.{ .RXNEIE = 1 }),
                .disable => reg.CR1.modify(.{ .RXNEIE = 0 }),
            }
        }

        pub fn txReady(_: Self) bool {
            return reg.SR.read().TXE == 1;
        }

        pub fn tx(self: Self, ch: u8) void {
            while (!self.txReady()) {} // Wait for Previous transmission
            reg.DR.modify(ch);
        }

        pub fn txflush(_: Self) void {
            while (reg.SR.read().TC == 0) {}
        }

        pub fn rxReady(_: Self) bool {
            return reg.SR.read().RXNE == 1;
        }

        pub fn rx(self: Self) u8 {
            while (!self.rxReady()) {} // Wait till the data is received
            const data_with_parity_bit: u9 = reg.DR.read();
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

fn Dma(comptime data: type, comptime config: Config) type {
    const Uart = Pooling(data, config);

    const ctl = data.dma.controller;
    const stream = std.fmt.comptimePrint("{d}", .{data.dma.tx.stream}); // to string

    const cr_reg = @field(@field(regs, ctl), "S" ++ stream ++ "CR"); // control register
    const ndtr_reg = @field(@field(regs, ctl), "S" ++ stream ++ "NDTR"); // number of data register
    const pa_reg = @field(@field(regs, ctl), "S" ++ stream ++ "PAR"); // peripheral address register
    const ma_reg = @field(@field(regs, ctl), "S" ++ stream ++ "M0AR"); // memory address register

    const hl_pre = if (data.dma.tx.stream > 3) "H" else "L"; // high/low interrupt status/flag register preffix
    const st_reg = @field(@field(regs, ctl), hl_pre ++ "ISR"); // interrupt status register
    const fc_reg = @field(@field(regs, ctl), hl_pre ++ "IFCR"); // interrupt flag clear register

    const peripheral_address = @ptrToInt(@field(regs, data.name).DR);

    return struct {
        const Self = @This();

        // transfer complete flag is set
        pub fn txComplete(_: Self) bool {
            return readIntFlag("TC");
        }

        // Returns true if transfer is started. False if can't be started now,
        // previous is still in progress.
        //
        // We are not waiting for previous transfer to complete, that decision
        // is pushed to the caller. To decide whether to wait or do something
        // while waiting.
        pub fn tx(self: Self, buf: []u8) bool {
            if (!self.txReady()) {
                return false;
            }

            const memory_address = @ptrToInt(&buf[0]);
            const number_of_data_items = @intCast(u16, buf.len);

            cr_reg.modify(.{ .EN = 0 }); // disable stream
            while (cr_reg.read().EN == 1) {} // wait for disable

            clearIntFlags();
            pa_reg.modify(.{ .PA = peripheral_address });
            ma_reg.modify(.{ .M0A = memory_address });
            ndtr_reg.modify(.{ .NDT = number_of_data_items });

            cr_reg.modify(.{ .EN = 1 }); // enable stream
            return true;
        }

        pub fn enabled(_: Self) bool {
            return cr_reg.read().EN == 1;
        }

        pub fn txReady(self: Self) bool {
            return if (self.enabled()) self.txComplete() else true;
        }

        // read interrupt flag, where flag is TC, HT, TE, DME or FE
        fn readIntFlag(comptime flag: []const u8) bool {
            return @field(st_reg.read(), flag ++ "IF" ++ stream) == 1;
        }

        // clear interrupt flag, where flag is TC, HT, TE, DME or FE
        fn clearIntFlag(comptime flag: []const u8) void {
            fc_reg.set("C" ++ flag ++ "IF" ++ stream, 1);
        }

        fn clearIntFlags() void {
            clearIntFlag("TC"); // transfer complete
            clearIntFlag("HT"); // half transfer
            clearIntFlag("TE"); // transfer error
            clearIntFlag("DME"); // direct mode error
            clearIntFlag("FE"); // fifo error
        }

        pub fn init() Self {
            _ = Uart.init();
            // enable clock
            // regs.RCC.AHB1ENR.modify(.{ .DMAxEN = 1 });
            regs.RCC.AHB1ENR.set(ctl ++ "EN", 1);

            cr_reg.modify(.{ .EN = 0 }); // disable stream
            while (cr_reg.read().EN == 1) {} // wait for disable

            clearIntFlags();

            cr_reg.modify(.{
                .CHSEL = 4, // channel
                .DIR = 0b01, // direction: memory to periperal
                .CIRC = 0, // circular mode disabled
                .PINC = 0, // periperal increment mode: fixed
                .MINC = 1, // memory address pointer is incremented after each data transfer
                .PSIZE = 0b00, // peripheral data size: byte
                .MSIZE = 0b00, // memory data size: byte
                //.TCIE = 1, // transfer complete interrupt enable
            });
            //irq.enable(.dma2_stream7);
            return Self{};
        }
    };
}
