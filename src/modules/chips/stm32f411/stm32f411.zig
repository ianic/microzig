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
pub const adc = @import("adc.zig");

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
    // 0b0xx => 16 group / 0 sub priorities, 0b100 => 8/2, 0b101 => 4/4, 0b110 => 2/8, 0b111 => 0/16
    prigroup: u3 = 0b000,
};

pub fn init(comptime cfg: Config) void {
    initFeatures(cfg);
    clk.init(cfg.clock);
    if (cfg.systick_enabled) {
        clk.initSysTick(cfg.clock.frequencies.ahb, 1);
    }
}

fn initFeatures(comptime cfg: Config) void {
    regs.FPU_CPACR.CPACR.modify(.{ .CP = 0b1111 }); // Enable FPU coprocessor
    regs.FLASH.ACR.modify(.{ .DCEN = 1, .ICEN = 1, .PRFTEN = 1 }); // Enable flash data and instruction cache

    // Prigroup determines the split of group priority (preemption priority) from sub-priority.
    // Refer to page 229 in PM0214 Rev 10 for possible values.
    // On writes, write 0x5FA to VECTKEY, otherwise the write is ignored.
    regs.SCB.AIRCR.modify(.{ .PRIGROUP = cfg.prigroup, .VECTKEYSTAT = 0x5FA });
}

pub fn Pin(comptime spec: []const u8) type {
    assertValidPin(spec);
    return @import("gpio.zig").Pin(spec);
}

fn assertValidPin(comptime spec: []const u8) void {
    if (spec[0] != 'P') @compileError("pin name should start with P");
    if (!(spec.len == 3 or spec.len == 4)) @compileError("invalid pin name len");
    const port = spec[1];
    // A..E and H are valid
    if (!((port >= 'A' and port <= 'E') or port == 'H'))
        @compileError("invalid port name");
    // only PH0 and PH1 are valid
    if (port == 'H')
        if (!(spec.len == 3 and (spec[2] == '0' or spec[2] == '1')))
            @compileError("in port H only PH0 and PH1 pins are available");
    // suffix is number 0..15
    const wrap = struct {
        const pn = std.fmt.parseInt(u4, spec[2..], 10) catch @compileError("pin must be number 0...15");
    };
    if (wrap.pn > 15) @compileError("pin number must be 0...15");
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

test "assertValidPin" {
    assertValidPin("PA0");
    assertValidPin("PA15");
    assertValidPin("PB0");
    assertValidPin("PB15");
    assertValidPin("PC0");
    assertValidPin("PC15");
    assertValidPin("PD0");
    assertValidPin("PD15");
    assertValidPin("PE0");
    assertValidPin("PE15");
    assertValidPin("PH0");
    assertValidPin("PH1");
}
