//! For now we keep all clock settings on the chip defaults.
//! This code currently assumes the STM32F405xx / STM32F407xx clock configuration.
//! TODO: Do something useful for other STM32F40x chips.
//!
//! Specifically, TIM6 is running on a 16 MHz clock,
//! HSI = 16 MHz is the SYSCLK after reset
//! default AHB prescaler = /1 (= values 0..7):
//!
//! ```
//! regs.RCC.CFGR.modify(.{ .HPRE = 0 });
//! ```
//!
//! so also HCLK = 16 MHz.
//! And with the default APB1 prescaler = /1:
//!
//! ```
//! regs.RCC.CFGR.modify(.{ .PPRE1 = 0 });
//! ```
//!
//! results in PCLK1 = 16 MHz.
//!
//! The above default configuration makes U(S)ART2..5
//! receive a 16 MHz clock by default.
//!
//! USART1 and USART6 use PCLK2, which uses the APB2 prescaler on HCLK,
//! default APB2 prescaler = /1:
//!
//! ```
//! regs.RCC.CFGR.modify(.{ .PPRE2 = 0 });
//! ```
//!
//! and therefore USART1 and USART6 receive a 16 MHz clock.
//!

const std = @import("std");
const micro = @import("microzig");
const chip = @import("registers.zig");
const regs = chip.registers;
pub const irq = @import("irq.zig");
pub const clk = @import("clock.zig");
pub const Frequencies = clk.Frequencies;
//pub const hsi_100 = clk.hsi_100;
//pub const hsi_96 = clk.hsi_96;

pub const gpioPin = @import("gpio.zig").pin;
pub const Pin = @import("gpio.zig").Pin;

pub usingnamespace chip;

pub const clock = struct {
    pub const Domain = enum {
        cpu,
        ahb,
        apb1,
        apb2,
    };
};

pub const Config = struct {
    clock: clk.Config = clk.hsi_100,
    systick_enabled: bool = true,
};

pub fn init(comptime cfg: Config) void {
    initFeatures();
    clk.init(cfg.clock);
    if (cfg.systick_enabled) {
        clk.initSysTick(cfg.clock.frequencies.ahb, 1);
    }
}

fn initFeatures() void {
    regs.FLASH.ACR.modify(.{ .DCEN = 1, .ICEN = 1, .PRFTEN = 1 }); // Enable flash data and instruction cache
    regs.SCB.AIRCR.modify(.{ .PRIGROUP = 0b011, .VECTKEYSTAT = 0x5FA }); // Set Interrupt Group Priority
    regs.FPU_CPACR.CPACR.modify(.{ .CP = 0b1111 }); // Enable FPU coprocessor
}

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

fn setRegField(reg: anytype, comptime field_name: anytype, value: anytype) void {
    var temp = reg.read();
    @field(temp, field_name) = value;
    reg.write(temp);
}

pub const gpio = struct {
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

    pub fn setOutput(comptime pin: type) void {
        setRegField(regs.RCC.AHB1ENR, "GPIO" ++ pin.gpio_port_name ++ "EN", 1);
        setRegField(@field(pin.gpio_port, "MODER"), "MODER" ++ pin.suffix, 0b01);
    }

    pub fn setInput(comptime pin: type) void {
        setRegField(regs.RCC.AHB1ENR, "GPIO" ++ pin.gpio_port_name ++ "EN", 1);
        setRegField(@field(pin.gpio_port, "MODER"), "MODER" ++ pin.suffix, 0b00);
    }

    pub fn setAlternateFunction(comptime pin: type, af: AlternateFunction) void {
        setRegField(regs.RCC.AHB1ENR, "GPIO" ++ pin.gpio_port_name ++ "EN", 1);
        setRegField(@field(pin.gpio_port, "MODER"), "MODER" ++ pin.suffix, 0b10);
        if (pin.pin_number < 8) {
            setRegField(@field(pin.gpio_port, "AFRL"), "AFRL" ++ pin.suffix, @enumToInt(af));
        } else {
            setRegField(@field(pin.gpio_port, "AFRH"), "AFRH" ++ pin.suffix, @enumToInt(af));
        }
    }

    pub fn read(comptime pin: type) micro.gpio.State {
        const idr_reg = pin.gpio_port.IDR;
        const reg_value = @field(idr_reg.read(), "IDR" ++ pin.suffix); // TODO extract to getRegField()?
        return @intToEnum(micro.gpio.State, reg_value);
    }

    pub fn write(comptime pin: type, state: micro.gpio.State) void {
        _ = pin;
        switch (state) {
            .low => setRegField(pin.gpio_port.BSRR, "BR" ++ pin.suffix, 1),
            .high => setRegField(pin.gpio_port.BSRR, "BS" ++ pin.suffix, 1),
        }
    }
};

pub const uart = struct {
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

    const PinDirection = std.meta.FieldEnum(micro.uart.Pins);

    /// Checks if a pin is valid for a given uart index and direction
    pub fn isValidPin(comptime pin: type, comptime index: usize, comptime direction: PinDirection) bool {
        const pin_name = pin.name;

        return switch (direction) {
            .tx => switch (index) {
                1 => std.mem.eql(u8, pin_name, "PA9") or std.mem.eql(u8, pin_name, "PB6"),
                2 => std.mem.eql(u8, pin_name, "PA2") or std.mem.eql(u8, pin_name, "PD5"),
                3 => std.mem.eql(u8, pin_name, "PB10") or std.mem.eql(u8, pin_name, "PC10") or std.mem.eql(u8, pin_name, "PD8"),
                4 => std.mem.eql(u8, pin_name, "PA0") or std.mem.eql(u8, pin_name, "PC10"),
                5 => std.mem.eql(u8, pin_name, "PC12"),
                6 => std.mem.eql(u8, pin_name, "PC6") or std.mem.eql(u8, pin_name, "PG14"),
                else => unreachable,
            },
            // Valid RX pins for the UARTs
            .rx => switch (index) {
                1 => std.mem.eql(u8, pin_name, "PA10") or std.mem.eql(u8, pin_name, "PB7"),
                2 => std.mem.eql(u8, pin_name, "PA3") or std.mem.eql(u8, pin_name, "PD6"),
                3 => std.mem.eql(u8, pin_name, "PB11") or std.mem.eql(u8, pin_name, "PC11") or std.mem.eql(u8, pin_name, "PD9"),
                4 => std.mem.eql(u8, pin_name, "PA1") or std.mem.eql(u8, pin_name, "PC11"),
                5 => std.mem.eql(u8, pin_name, "PD2"),
                6 => std.mem.eql(u8, pin_name, "PC7") or std.mem.eql(u8, pin_name, "PG9"),
                else => unreachable,
            },
        };
    }
};

pub fn Uart(comptime index: usize, comptime pins: micro.uart.Pins) type {
    if (index < 1 or index > 6) @compileError("Valid USART index are 1..6");

    const usart_name = std.fmt.comptimePrint("USART{d}", .{index});
    const tx_pin =
        if (pins.tx) |tx|
        if (uart.isValidPin(tx, index, .tx))
            tx
        else
            @compileError(std.fmt.comptimePrint("Tx pin {s} is not valid for UART{}", .{ tx.name, index }))
    else switch (index) {
        // Provide default tx pins if no pin is specified
        1 => micro.Pin("PA9"),
        2 => micro.Pin("PA2"),
        3 => micro.Pin("PB10"),
        4 => micro.Pin("PA0"),
        5 => micro.Pin("PC12"),
        6 => micro.Pin("PC6"),
        else => unreachable,
    };

    const rx_pin =
        if (pins.rx) |rx|
        if (uart.isValidPin(rx, index, .rx))
            rx
        else
            @compileError(std.fmt.comptimePrint("Rx pin {s} is not valid for UART{}", .{ rx.name, index }))
    else switch (index) {
        // Provide default rx pins if no pin is specified
        1 => micro.Pin("PA10"),
        2 => micro.Pin("PA3"),
        3 => micro.Pin("PB11"),
        4 => micro.Pin("PA1"),
        5 => micro.Pin("PD2"),
        6 => micro.Pin("PC7"),
        else => unreachable,
    };

    // USART1..3 are AF7, USART 4..6 are AF8
    const alternate_function = if (index <= 3) .af7 else .af8;

    const tx_gpio = micro.Gpio(tx_pin, .{
        .mode = .alternate_function,
        .alternate_function = alternate_function,
    });
    const rx_gpio = micro.Gpio(rx_pin, .{
        .mode = .alternate_function,
        .alternate_function = alternate_function,
    });

    return struct {
        parity_read_mask: u8,

        const Self = @This();

        pub fn init(config: micro.uart.Config) !Self {
            // The following must all be written when the USART is disabled (UE=0).
            if (@field(regs, usart_name).CR1.read().UE == 1)
                @panic("Trying to initialize " ++ usart_name ++ " while it is already enabled");
            // LATER: Alternatively, set UE=0 at this point?  Then wait for something?
            // Or add a destroy() function which disables the USART?

            // enable the USART clock
            const clk_enable_reg = switch (index) {
                1, 6 => regs.RCC.APB2ENR,
                2...5 => regs.RCC.APB1ENR,
                else => unreachable,
            };
            setRegField(clk_enable_reg, usart_name ++ "EN", 1);

            tx_gpio.init();
            rx_gpio.init();

            // clear USART configuration to its default
            @field(regs, usart_name).CR1.raw = 0;
            @field(regs, usart_name).CR2.raw = 0;
            @field(regs, usart_name).CR3.raw = 0;

            // Return error for unsupported combinations
            if (config.data_bits == .nine and config.parity != null) {
                // TODO: should we consider this an unsupported word size or unsupported parity?
                return error.UnsupportedWordSize;
            } else if (config.data_bits == .seven and config.parity == null) {
                // TODO: should we consider this an unsupported word size or unsupported parity?
                return error.UnsupportedWordSize;
            }

            // set word length
            // Per the reference manual, M means
            // - 0: 1 start bit, 8 data bits (7 data + 1 parity, or 8 data), n stop bits, the chip default
            // - 1: 1 start bit, 9 data bits (8 data + 1 parity, or 9 data), n stop bits
            const m: u1 = if (config.data_bits == .nine or (config.data_bits == .eight and config.parity != null)) 1 else 0;
            @field(regs, usart_name).CR1.modify(.{ .M = m });

            // set parity
            if (config.parity) |parity| {
                @field(regs, usart_name).CR1.modify(.{ .PCE = 1, .PS = @enumToInt(parity) });
            } // otherwise, no need to set no parity since we reset Control Registers above, and it's the default

            // set number of stop bits
            @field(regs, usart_name).CR2.modify(.{ .STOP = @enumToInt(config.stop_bits) });

            // set the baud rate
            // Despite the reference manual talking about fractional calculation and other buzzwords,
            // it is actually just a simple divider. Just ignore DIV_Mantissa and DIV_Fraction and
            // set the result of the division as the lower 16 bits of BRR.
            // TODO: We assume the default OVER8=0 configuration above (i.e. 16x oversampling).
            // TODO: Do some checks to see if the baud rate is too high (or perhaps too low)
            // TODO: Do a rounding div, instead of a truncating div?
            const clocks = micro.clock.get();
            const bus_frequency = switch (index) {
                1, 6 => clocks.apb2,
                2...5 => clocks.apb1,
                else => unreachable,
            };
            const usartdiv = @intCast(u16, @divTrunc(bus_frequency, config.baud_rate));
            @field(regs, usart_name).BRR.raw = usartdiv;

            // enable USART, and its transmitter and receiver
            @field(regs, usart_name).CR1.modify(.{ .UE = 1 });
            @field(regs, usart_name).CR1.modify(.{ .TE = 1 });
            @field(regs, usart_name).CR1.modify(.{ .RE = 1 });

            // For code simplicity, at cost of one or more register reads,
            // we read back the actual configuration from the registers,
            // instead of using the `config` values.
            return readFromRegisters();
        }

        pub fn getOrInit(config: micro.uart.Config) !Self {
            if (@field(regs, usart_name).CR1.read().UE == 1) {
                // UART1 already enabled, don't reinitialize and disturb things;
                // instead read and use the actual configuration.
                return readFromRegisters();
            } else return Self.init(config);
        }

        fn readFromRegisters() Self {
            const cr1 = @field(regs, usart_name).CR1.read();
            // As documented in `init()`, M0==1 means 'the 9th bit (not the 8th bit) is the parity bit'.
            // So we always mask away the 9th bit, and if parity is enabled and it is in the 8th bit,
            // then we also mask away the 8th bit.
            return Self{ .parity_read_mask = if (cr1.PCE == 1 and cr1.M == 0) 0x7F else 0xFF };
        }

        pub fn canWrite(self: Self) bool {
            _ = self;
            return switch (@field(regs, usart_name).SR.read().TXE) {
                1 => true,
                0 => false,
            };
        }

        pub fn tx(self: Self, ch: u8) void {
            while (!self.canWrite()) {} // Wait for Previous transmission
            @field(regs, usart_name).DR.modify(ch);
        }

        pub fn txflush(_: Self) void {
            while (@field(regs, usart_name).SR.read().TC == 0) {}
        }

        pub fn canRead(self: Self) bool {
            _ = self;
            return switch (@field(regs, usart_name).SR.read().RXNE) {
                1 => true,
                0 => false,
            };
        }

        pub fn rx(self: Self) u8 {
            while (!self.canRead()) {} // Wait till the data is received
            const data_with_parity_bit: u9 = @field(regs, usart_name).DR.read();
            return @intCast(u8, data_with_parity_bit & self.parity_read_mask);
        }
    };
}

pub const i2c = struct {
    const PinLine = std.meta.FieldEnum(micro.i2c.Pins);

    /// Checks if a pin is valid for a given i2c index and line
    pub fn isValidPin(comptime pin: type, comptime index: usize, comptime line: PinLine) bool {
        const pin_name = pin.name;

        return switch (line) {
            .scl => switch (index) {
                1 => std.mem.eql(u8, pin_name, "PB6") or std.mem.eql(u8, pin_name, "PB8"),
                2 => std.mem.eql(u8, pin_name, "PB10") or std.mem.eql(u8, pin_name, "PF1") or std.mem.eql(u8, pin_name, "PH4"),
                3 => std.mem.eql(u8, pin_name, "PA8") or std.mem.eql(u8, pin_name, "PH7"),
                else => unreachable,
            },
            // Valid RX pins for the UARTs
            .sda => switch (index) {
                1 => std.mem.eql(u8, pin_name, "PB7") or std.mem.eql(u8, pin_name, "PB9"),
                2 => std.mem.eql(u8, pin_name, "PB11") or std.mem.eql(u8, pin_name, "PF0") or std.mem.eql(u8, pin_name, "PH5"),
                3 => std.mem.eql(u8, pin_name, "PC9") or std.mem.eql(u8, pin_name, "PH8"),
                else => unreachable,
            },
        };
    }
};

pub fn I2CController(comptime index: usize, comptime pins: micro.i2c.Pins) type {
    if (index < 1 or index > 3) @compileError("Valid I2C index are 1..3");

    const i2c_name = std.fmt.comptimePrint("I2C{d}", .{index});
    const scl_pin =
        if (pins.scl) |scl|
        if (uart.isValidPin(scl, index, .scl))
            scl
        else
            @compileError(std.fmt.comptimePrint("SCL pin {s} is not valid for I2C{}", .{ scl.name, index }))
    else switch (index) {
        // Provide default scl pins if no pin is specified
        1 => micro.Pin("PB6"),
        2 => micro.Pin("PB10"),
        3 => micro.Pin("PA8"),
        else => unreachable,
    };

    const sda_pin =
        if (pins.sda) |sda|
        if (uart.isValidPin(sda, index, .sda))
            sda
        else
            @compileError(std.fmt.comptimePrint("SDA pin {s} is not valid for UART{}", .{ sda.name, index }))
    else switch (index) {
        // Provide default sda pins if no pin is specified
        1 => micro.Pin("PB7"),
        2 => micro.Pin("PB11"),
        3 => micro.Pin("PC9"),
        else => unreachable,
    };

    const scl_gpio = micro.Gpio(scl_pin, .{
        .mode = .alternate_function,
        .alternate_function = .af4,
    });
    const sda_gpio = micro.Gpio(sda_pin, .{
        .mode = .alternate_function,
        .alternate_function = .af4,
    });

    // Base field of the specific I2C peripheral
    const i2c_base = @field(regs, i2c_name);

    return struct {
        const Self = @This();

        pub fn init(config: micro.i2c.Config) !Self {
            // Configure I2C

            // 1. Enable the I2C CLOCK and GPIO CLOCK
            regs.RCC.APB1ENR.modify(.{ .I2C1EN = 1 });
            regs.RCC.AHB1ENR.modify(.{ .GPIOBEN = 1 });

            // 2. Configure the I2C PINs
            // This takes care of setting them alternate function mode with the correct AF
            scl_gpio.init();
            sda_gpio.init();

            // TODO: the stuff below will probably use the microzig gpio API in the future
            const scl = scl_pin.source_pin;
            const sda = sda_pin.source_pin;
            // Select Open Drain Output
            setRegField(@field(scl.gpio_port, "OTYPER"), "OT" ++ scl.suffix, 1);
            setRegField(@field(sda.gpio_port, "OTYPER"), "OT" ++ sda.suffix, 1);
            // Select High Speed
            setRegField(@field(scl.gpio_port, "OSPEEDR"), "OSPEEDR" ++ scl.suffix, 0b10);
            setRegField(@field(sda.gpio_port, "OSPEEDR"), "OSPEEDR" ++ sda.suffix, 0b10);
            // Activate Pull-up
            setRegField(@field(scl.gpio_port, "PUPDR"), "PUPDR" ++ scl.suffix, 0b01);
            setRegField(@field(sda.gpio_port, "PUPDR"), "PUPDR" ++ sda.suffix, 0b01);

            // 3. Reset the I2C
            i2c_base.CR1.modify(.{ .PE = 0 });
            while (i2c_base.CR1.read().PE == 1) {}

            // 4. Configure I2C timing
            const bus_frequency_hz = micro.clock.get().apb1;
            const bus_frequency_mhz: u6 = @intCast(u6, @divExact(bus_frequency_hz, 1_000_000));

            if (bus_frequency_mhz < 2 or bus_frequency_mhz > 50) {
                return error.InvalidBusFrequency;
            }

            // .FREQ is set to the bus frequency in Mhz
            i2c_base.CR2.modify(.{ .FREQ = bus_frequency_mhz });

            switch (config.target_speed) {
                10_000...100_000 => {
                    // CCR is bus_freq / (target_speed * 2). We use floor to avoid exceeding the target speed.
                    const ccr = @intCast(u12, @divFloor(bus_frequency_hz, config.target_speed * 2));
                    i2c_base.CCR.modify(.{ .CCR = ccr });
                    // Trise is bus frequency in Mhz + 1
                    i2c_base.TRISE.modify(bus_frequency_mhz + 1);
                },
                100_001...400_000 => {
                    // TODO: handle fast mode
                    return error.InvalidSpeed;
                },
                else => return error.InvalidSpeed,
            }

            // 5. Program the I2C_CR1 register to enable the peripheral
            i2c_base.CR1.modify(.{ .PE = 1 });

            return Self{};
        }

        pub const WriteState = struct {
            address: u7,
            buffer: [255]u8 = undefined,
            buffer_size: u8 = 0,

            pub fn start(address: u7) !WriteState {
                return WriteState{ .address = address };
            }

            pub fn writeAll(self: *WriteState, bytes: []const u8) !void {
                std.debug.assert(self.buffer_size < 255);
                for (bytes) |b| {
                    self.buffer[self.buffer_size] = b;
                    self.buffer_size += 1;
                    if (self.buffer_size == 255) {
                        try self.sendBuffer();
                    }
                }
            }

            fn sendBuffer(self: *WriteState) !void {
                if (self.buffer_size == 0) @panic("write of 0 bytes not supported");

                // Wait for the bus to be free
                while (i2c_base.SR2.read().BUSY == 1) {}

                // Send start
                i2c_base.CR1.modify(.{ .START = 1 });

                // Wait for the end of the start condition, master mode selected, and BUSY bit set
                while ((i2c_base.SR1.read().SB == 0 or
                    i2c_base.SR2.read().MSL == 0 or
                    i2c_base.SR2.read().BUSY == 0))
                {}

                // Write the address to bits 7..1, bit 0 stays at 0 to indicate write operation
                i2c_base.DR.modify(@intCast(u8, self.address) << 1);

                // Wait for address confirmation
                while (i2c_base.SR1.read().ADDR == 0) {}

                // Read SR2 to clear address condition
                _ = i2c_base.SR2.read();

                for (self.buffer[0..self.buffer_size]) |b| {
                    // Write data byte
                    i2c_base.DR.modify(b);
                    // Wait for transfer finished
                    while (i2c_base.SR1.read().BTF == 0) {}
                }
                self.buffer_size = 0;
            }

            pub fn stop(self: *WriteState) !void {
                try self.sendBuffer();
                // Communication STOP
                i2c_base.CR1.modify(.{ .STOP = 1 });
                while (i2c_base.SR2.read().BUSY == 1) {}
            }

            pub fn restartRead(self: *WriteState) !ReadState {
                try self.sendBuffer();
                return ReadState{ .address = self.address };
            }
            pub fn restartWrite(self: *WriteState) !WriteState {
                try self.sendBuffer();
                return WriteState{ .address = self.address };
            }
        };

        pub const ReadState = struct {
            address: u7,

            pub fn start(address: u7) !ReadState {
                return ReadState{ .address = address };
            }

            /// Fails with ReadError if incorrect number of bytes is received.
            pub fn readNoEof(self: *ReadState, buffer: []u8) !void {
                std.debug.assert(buffer.len < 256);

                // Send start and enable ACK
                i2c_base.CR1.modify(.{ .START = 1, .ACK = 1 });

                // Wait for the end of the start condition, master mode selected, and BUSY bit set
                while ((i2c_base.SR1.read().SB == 0 or
                    i2c_base.SR2.read().MSL == 0 or
                    i2c_base.SR2.read().BUSY == 0))
                {}

                // Write the address to bits 7..1, bit 0 set to 1 to indicate read operation
                i2c_base.DR.modify((@intCast(u8, self.address) << 1) | 1);

                // Wait for address confirmation
                while (i2c_base.SR1.read().ADDR == 0) {}

                // Read SR2 to clear address condition
                _ = i2c_base.SR2.read();

                for (buffer) |_, i| {
                    if (i == buffer.len - 1) {
                        // Disable ACK
                        i2c_base.CR1.modify(.{ .ACK = 0 });
                    }

                    // Wait for data to be received
                    while (i2c_base.SR1.read().RxNE == 0) {}

                    // Read data byte
                    buffer[i] = i2c_base.DR.read();
                }
            }

            pub fn stop(_: *ReadState) !void {
                // Communication STOP
                i2c_base.CR1.modify(.{ .STOP = 1 });
                while (i2c_base.SR2.read().BUSY == 1) {}
            }

            pub fn restartRead(self: *ReadState) !ReadState {
                return ReadState{ .address = self.address };
            }
            pub fn restartWrite(self: *ReadState) !WriteState {
                return WriteState{ .address = self.address };
            }
        };
    };
}

// pub fn init() void {
//     initFeatures();
//     initClock();
//     initKey();
//     led_pin.init();
// }

// // board has key connected to the port PA0
// // activate key
// fn initKey() void {
//     // init key as input
//     //micro.Gpio(micro.Pin("PA0"), .{ .mode = .input }).init();
//     // above micro is same as below regs two lines

//     // PA0 enable input
//     regs.RCC.AHB1ENR.modify(.{ .GPIOAEN = 1 });
//     regs.GPIOA.MODER.modify(.{ .MODER0 = 0b00 });

//     // PA0 interupt enabling
//     regs.SYSCFG.EXTICR1.modify(.{ .EXTI0 = 1 });
//     regs.EXTI.RTSR.modify(.{ .TR0 = 1 });
//     regs.EXTI.FTSR.modify(.{ .TR0 = 0 });
//     regs.EXTI.IMR.modify(.{ .MR0 = 1 });
//     regs.NVIC.ISER0.modify(.{ .SETENA = 0x40 });
// }

// fn initFeatures() void {
//     // Enable FPU coprocessor
//     // WARN: currently not supported in qemu, comment if testing it there
//     regs.FPU_CPACR.CPACR.modify(.{ .CP = 0b11 });

//     // Enable flash data and instruction cache
//     regs.FLASH.ACR.modify(.{ .DCEN = 1, .ICEN = 1 });
// }

// // Clock frqencies set in initClock
// // prescalers are set in systemInit
// pub const clock_frequencies = .{
//     .cpu = 48_000_000,
//     .ahb = 48_000_000 / 1, // .HPRE
//     .apb1 = 48_000_000 / 4, // .PPRE1
//     .apb2 = 48_000_000 / 2, // .PPRE2
// };

// fn initClock() void {
//     // Enable HSI
//     regs.RCC.CR.modify(.{ .HSION = 1 });

//     // Wait for HSI ready
//     while (regs.RCC.CR.read().HSIRDY != 1) {}

//     // Select HSI as clock source
//     regs.RCC.CFGR.modify(.{ .SW0 = 0, .SW1 = 0 });

//     // Enable external high-speed oscillator (HSE)
//     regs.RCC.CR.modify(.{ .HSEON = 1 });

//     // Wait for HSE ready
//     while (regs.RCC.CR.read().HSERDY != 1) {}

//     // Set prescalers for 48 MHz: HPRE = 0, PPRE1 = DIV_4, PPRE2 = DIV_2
//     regs.RCC.CFGR.modify(.{ .HPRE = 0, .PPRE1 = 0b101, .PPRE2 = 0b100 });

//     // few working options
//     //setPllCfgr(25, 384, 4, 8, 3); // 96 Mhz
//     setPllCfgr(25, 384, 8, 8, 1); // 48 Mhz
//     //setPllCfgr(50, 384, 8, 4, 0); // 24 Mhz

//     // Disable HSI
//     regs.RCC.CR.modify(.{ .HSION = 0 });

//     // Enable LSE
//     //regs.RCC.BDCR.modify(.{ .LSEON = 1 });
//     //while (regs.RCC.BDCR.read().LSERDY != 1) {}
// }

// // Fv = source * n / m
// // Fsystem = Fv / p
// // Fusb = Fv / q  must be e.q. 48Mhz
// //
// // m 2..63
// // n 50..432
// // p 2,4,6,8
// // q 2..15
// //
// // hse source = 25Mhz
// // Fsystem max = 100Mhz
// fn setPllCfgr(comptime m: u16, comptime n: u16, comptime p: u16, comptime q: u16, comptime latency: u3) void {
//     if (!((m >= 2 and m <= 63) and
//         (n >= 50 and n <= 432) and
//         (p == 2 or p == 4 or p == 6 or p == 8) and
//         (q >= 2 and q <= 15) and
//         (latency >= 0 and latency <= 6)))
//     {
//         @compileError("wrong RCC PLL configuration register values");
//     }

//     // assuming 25 MHz of external clock
//     //@compileLog("setting system clock to [MHz] ", 25 * n / m / p);
//     //@compileLog("USB clock to [MHz] ", 25 * n / m / q);

//     // Disable PLL before changing its configuration
//     regs.RCC.CR.modify(.{ .PLLON = 0 });

//     regs.RCC.PLLCFGR.modify(.{
//         .PLLSRC = 1,
//         // PLLM
//         .PLLM0 = bitOf(m, 0),
//         .PLLM1 = bitOf(m, 1),
//         .PLLM2 = bitOf(m, 2),
//         .PLLM3 = bitOf(m, 3),
//         .PLLM4 = bitOf(m, 4),
//         .PLLM5 = bitOf(m, 5),
//         // PLLN
//         .PLLN0 = bitOf(n, 0),
//         .PLLN1 = bitOf(n, 1),
//         .PLLN2 = bitOf(n, 2),
//         .PLLN3 = bitOf(n, 3),
//         .PLLN4 = bitOf(n, 4),
//         .PLLN5 = bitOf(n, 5),
//         .PLLN6 = bitOf(n, 6),
//         .PLLN7 = bitOf(n, 7),
//         .PLLN8 = bitOf(n, 8),
//         // PLLP
//         .PLLP0 = if (p == 4 or p == 8) 1 else 0,
//         .PLLP1 = if (p == 6 or p == 8) 1 else 0,
//         // PLLQ
//         .PLLQ0 = bitOf(q, 0),
//         .PLLQ1 = bitOf(q, 1),
//         .PLLQ2 = bitOf(q, 2),
//         .PLLQ3 = bitOf(q, 3),
//     });
//     // Enable PLL
//     regs.RCC.CR.modify(.{ .PLLON = 1 });

//     // Wait for PLL ready
//     while (regs.RCC.CR.read().PLLRDY != 1) {}

//     // Set flash latency wait states
//     // depends on clock and voltage range, chapter 3.4 page 45
//     regs.FLASH.ACR.modify(.{ .LATENCY = latency });

//     // // Select PLL as clock source
//     regs.RCC.CFGR.modify(.{ .SW1 = 1, .SW0 = 0 });

//     // // Wait for PLL selected as clock source
//     var cfgr = regs.RCC.CFGR.read();
//     while (cfgr.SWS1 != 1 and cfgr.SWS0 != 0) : (cfgr = regs.RCC.CFGR.read()) {}
// }

// test "show available hse pll parameters" {
//     var pa = [_]u64{ 2, 4, 6, 8 };

//     const source: u64 = 25_000_000; // hse
//     //const source: u64 = 16_000_000; // hsi
//     const fs_max: u64 = 100_000_000;
//     const fusb: u64 = 48_000_000;

//     var m: u64 = 2;
//     while (m <= 63) : (m += 1) {
//         var n: u64 = 50;
//         while (n <= 432) : (n += 1) {
//             var q: u64 = 2;
//             while (q <= 15) : (q += 1) {
//                 for (pa) |p| {
//                     var fv = source * n / m;
//                     var fs = fv / p;
//                     var fu = fv / q;
//                     if (fu == fusb and fs <= fs_max) {
//                         std.debug.print("fs: {d} MHz {d}  ", .{ fs / 1_000_000, fs });
//                         std.debug.print("m: {d:2}, n: {d:3}, q: {d:2}, p: {d}\n", .{ m, n, q, p });
//                     }
//                 }
//             }
//         }
//     }
// }

// fn bitOf(x: u16, index: u4) u1 {
//     const mask = @as(u16, 1) << index;
//     return if (x & mask == mask) 1 else 0;
// }

// test "is bit set" {
//     try std.testing.expectEqual(bitOf(336, 0), 0);
//     try std.testing.expectEqual(bitOf(336, 1), 0);
//     try std.testing.expectEqual(bitOf(336, 2), 0);
//     try std.testing.expectEqual(bitOf(336, 3), 0);
//     try std.testing.expectEqual(bitOf(336, 5), 0);
//     try std.testing.expectEqual(bitOf(336, 7), 0);

//     try std.testing.expectEqual(bitOf(336, 4), 1);
//     try std.testing.expectEqual(bitOf(336, 6), 1);
//     try std.testing.expectEqual(bitOf(336, 8), 1);
// }

// pub const delay = struct {
//     pub fn sleep(ms: u32) void {
//         const loop_ops = 5; // estimate number of instruction per loop
//         var ticks: u32 = clock_frequencies.cpu / 1000 * ms / loop_ops;
//         while (ticks > 0) : (ticks -= 1) {
//             asm volatile ("nop");
//         }
//     }

//     pub fn long() void {
//         sleep(1000);
//     }

//     pub fn short() void {
//         sleep(100);
//     }
// };

// const led_pin = micro.Gpio(micro.Pin("PC13"), .{
//     .mode = .output,
//     .initial_state = .high,
// });

// pub const led = struct {
//     pub fn blink(times: u32) void {
//         var i: u32 = 0;
//         while (i < times) : (i += 1) {
//             led_pin.setToLow();
//             delay.short();
//             led_pin.setToHigh();
//             delay.short();
//         }
//     }

//     pub fn on() void {
//         led_pin.setToLow();
//     }

//     pub fn off() void {
//         led_pin.setToHigh();
//     }
// };

// // init SysTick interrupt every ms milliseconds
// pub fn systick(ms: u16) void {
//     // 48 Mhz / 8 = 6 Mhz
//     // 6000 = 1ms
//     const reload: u24 = @intCast(u24, clock_frequencies.ahb / 8) / 1000 * ms - 1;
//     regs.SCS.SysTick.LOAD.modify(.{ .RELOAD = reload });

//     // CLKSOURCE:
//     //   0: AHB/8
//     //   1: Processor clock (AHB)
//     // TICKINT:
//     //   0: Counting down to zero does not assert the SysTick exception request
//     //   1: Counting down to zero to asserts the SysTick exception request.
//     //
//     regs.SCS.SysTick.CTRL.modify(.{ .ENABLE = 1, .CLKSOURCE = 0, .TICKINT = 1 });
// }

test "bit mask" {
    var val: u32 = 1;
    const mask: u32 = 0x10;
    val = val | mask;
    std.debug.print("mask: {x} {x}\n", .{ val, ~mask });
}

// const microzig = micro;
// //const VectorTable = @import("registers.zig").VectorTable;
// const app = microzig.app;

// fn isValidField(field_name: []const u8) bool {
//     return !std.mem.startsWith(u8, field_name, "reserved") and
//         !std.mem.eql(u8, field_name, "initial_stack_pointer") and
//         !std.mem.eql(u8, field_name, "reset");
// }

// // will be imported by microzig.zig to allow system startup.
// pub var vector_table: chip.VectorTable = blk: {
//     var tmp: microzig.chip.VectorTable = .{
//         .initial_stack_pointer = microzig.config.end_of_stack,
//         .Reset = .{ .C = microzig.cpu.startup_logic._start },
//     };
//     if (@hasDecl(app, "interrupts")) {
//         if (@typeInfo(app.interrupts) != .Struct)
//             @compileLog("root.interrupts must be a struct");

//         inline for (@typeInfo(app.interrupts).Struct.decls) |decl| {
//             const function = @field(app.interrupts, decl.name);

//             if (!@hasField(chip.VectorTable, decl.name)) {
//                 var msg: []const u8 = "There is no such interrupt as '" ++ decl.name ++ "'. Declarations in 'interrupts' must be one of:\n";
//                 inline for (std.meta.fields(chip.VectorTable)) |field| {
//                     if (isValidField(field.name)) {
//                         msg = msg ++ "    " ++ field.name ++ "\n";
//                     }
//                 }

//                 @compileError(msg);
//             }

//             if (!isValidField(decl.name))
//                 @compileError("You are not allowed to specify '" ++ decl.name ++ "' in the vector table, for your sins you must now pay a $5 fine to the ZSF: https://github.com/sponsors/ziglang");

//             // @compileLog("creating interrupt handler na pravom mjestu ", decl.name);
//             // if (decl.name[0] == 69) {
//             //     @field(tmp, decl.name) = irq.interrupts.EXTI0(function);
//             // } else {
//             @field(tmp, decl.name) = createInterruptVector(function);
//             //}
//         }
//     }
//     break :blk tmp;
// };

// fn createInterruptVector(
//     comptime function: anytype,
// ) microzig.chip.InterruptVector {
//     const calling_convention = @typeInfo(@TypeOf(function)).Fn.calling_convention;
//     //@compileLog("calling_convention ", calling_convention);
//     return switch (calling_convention) {
//         .C => .{ .C = function },
//         .Naked => .{ .Naked = function },
//         // for unspecified calling convention we are going to generate small wrapper
//         .Unspecified => .{
//             .C = struct {
//                 fn wrapper() callconv(.C) void {
//                     if (calling_convention == .Unspecified) // TODO: workaround for some weird stage1 bug
//                         @call(.{ .modifier = .always_inline }, function, .{});
//                 }
//             }.wrapper,
//         },

//         else => |val| {
//             const conv_name = inline for (std.meta.fields(std.builtin.CallingConvention)) |field| {
//                 if (val == @field(std.builtin.CallingConvention, field.name))
//                     break field.name;
//             } else unreachable;

//             @compileError("unsupported calling convention for interrupt vector: " ++ conv_name);
//         },
//     };
// }

pub fn ticker() Ticker {
    return .{ .ticks = 0 };
}

pub const Ticker = struct {
    ticks: u32 = 0,

    const Self = @This();
    pub fn inc(self: *Self) void {
        if (self.ticks == 0xFFFFFFFF) {
            self.ticks = 0;
            return;
        }
        self.ticks += 1;
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

    fn set(self: *Self, current: u32, count: u32) void {
        self.overflow = @addWithOverflow(u32, current, count, &self.next);
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

    var counter = t.every(2);
    try std.testing.expect(!counter.ready());
    t.inc();
    try std.testing.expect(!counter.ready());
    try std.testing.expectEqual(t.ticks, 0);
    try std.testing.expectEqual(counter.next, 1);
    try std.testing.expectEqual(counter.overflow, true);
    t.inc();
    try std.testing.expect(counter.ready());
    try std.testing.expectEqual(counter.next, 3);
    try std.testing.expectEqual(counter.overflow, false);
}

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
