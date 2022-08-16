const std = @import("std");
const regs = @import("registers.zig").registers;
const Pin = @import("stm32f411.zig").Pin;

pub const Config = struct {
    irq_enable: bool = true, // enable rising adc interrupt on each conversion
    //apb2_clock: u32, // for calculating prescaler
    inputs: u32, // enabled inputs channels 0...18 bits
    data: *[16]u16, // where results of conversion will be stored

    // value for the ADC sample time register SMPRx
    // 000: 3 cycles 001: 15 cycles 010: 28 cycles 011: 56 cycles 100: 84 cycles 101: 112 cycles 110: 144 cycles 111: 480 cycles
    sample_time: u3 = 0b111,
};

pub fn init(comptime cfg: Config) void {
    if (cfg.inputs == 0) {
        @compileError("no inputs defined");
    }
    const data_items = adc_init(cfg);
    dma_init(data_items, @ptrToInt(cfg.data));
}

fn adc_init(cfg: Config) u16 {
    regs.RCC.APB2ENR.modify(.{ .ADC1EN = 1 }); // enable clock for adc1

    regs.ADC_Common.CCR.modify(.{ .TSVREFE = 1 }); // wake up temperature sensor from power down

    regs.ADC1.CR2.modify(.{ .ADON = 0 }); // disable ADC
    regs.ADC1.CR2.modify(.{ .CONT = 1 }); // continous mode, 0 for one conversion only
    regs.ADC1.CR2.modify(.{ .DMA = 1, .DDS = 1 }); // DMA mode enabled
    regs.ADC1.CR2.modify(.{ .EOCS = 1 }); // The EOC bit is set at the end of each regular conversion.

    const sequence_len = initInputs(cfg.inputs, cfg.sample_time);

    // regs.ADC1.SQR3.modify(.{ .SQ1 = 18, .SQ2 = 17 }); // select ADC input channel
    // regs.ADC1.SMPR1.modify(.{ .SMP18 = 0b111, .SMP17 = 0b111 }); // 480 cycles for this channels (default 3)
    // const sequence_len = 1;

    regs.ADC1.SQR1.modify(.{ .L = sequence_len }); // sequnce 0 = 1 conversions ... 15 = 16 conversions
    regs.ADC1.CR1.modify(.{ .SCAN = 1 }); // scan mode must be set, if you are using more than 1 channel for the ADC
    regs.ADC1.CR1.modify(.{ .RES = 0b000 }); // resolution of the conversion, 12-bit (15 ADCCLK cycles), 0 to 4095

    // TODO calc prescaler based on input frequency
    // ADCCLK = APB2 / prescaler, max 18 MHz
    // This clock is generated from the APB2 clock divided by a programmable prescaler
    // that allows the ADC to work at fPCLK2/2, /4, /6 or /8. Max clock is 18 MHz
    regs.ADC_Common.CCR.modify(.{ .ADCPRE = 0b11 }); // prescaler to divide clock by 8

    if (cfg.irq_enable) {
        regs.ADC1.CR1.modify(.{ .EOCIE = 1 }); // enable interrupt
        // enable IRQ ADC_IRQn = 18
        // TODO za ovo bi mogao iskoristiti irq
        regs.NVIC.ISER0.modify(.{ .SETENA = 0x4_0000 });
    }

    regs.ADC1.SR.modify(.{ .EOC = 0, .OVR = 0, .STRT = 0 }); // clear status register
    regs.ADC1.CR2.modify(.{ .ADON = 1 }); // enable ADC
    regs.ADC1.CR2.modify(.{ .SWSTART = 1 }); // start the ADC conversion

    return @intCast(u16, sequence_len) + 1;
}

fn dma_init(data_items: u16, destination: usize) void {
    regs.RCC.AHB1ENR.modify(.{ .DMA2EN = 1 }); // enable clock for dma2

    regs.DMA2.S0CR.modify(.{ .EN = 0 }); // disable DMA
    regs.DMA2.S0CR.modify(.{
        .DIR = 0b00, // direction peripheral to memoy
        .CIRC = 1, // circular mode
        .MINC = 1, // memory increment mode
        .PINC = 0, // periperal no incerment
        .MSIZE = 0b01, // memory data size 16 bits
        .PSIZE = 0b01, // peripheral data size 16 bits
        .CHSEL = 0b000, // channel 0 sleected
    });
    regs.DMA2.S0NDTR.modify(.{ .NDT = data_items }); // number of data items, that we want transfer using the DMA
    regs.DMA2.S0PAR.modify(.{ .PA = @ptrToInt(regs.ADC1.DR) }); // address of the Peripheral Register, ADC1 DR
    regs.DMA2.S0M0AR.modify(.{ .M0A = destination }); // memory destination address
    regs.DMA2.S0CR.modify(.{ .EN = 1 }); // enable DMA
}

test "inputs enabling" {
    const inputs = IN0 | PA6 | IN14;
    var channel: u8 = 0;
    var position: u8 = 0;
    while (channel <= 18) : (channel += 1) {
        const channel_mask = @intCast(u32, 1) << @intCast(u5, channel);
        if (inputs & channel_mask == channel_mask) {
            std.debug.print("{d} {d} is set\n", .{ channel, position });
            position += 1;
        }
    }
}

fn initInputs(inputs: u32, smp: u3) u4 {
    var channel: u5 = 0;
    var position: u4 = 0;
    while (channel <= 18) : (channel += 1) {
        const channel_mask = @intCast(u32, 1) << @intCast(u5, channel);
        if (inputs & channel_mask == channel_mask) { // if channel x is enbled
            setSequence(channel, position);
            setSampleTime(channel, smp);
            if (channel < 16) {
                setInputPin(channel);
            }
            if (position == 15) {
                break;
            }
            position += 1;
        }
    }
    return position - 1;
}

fn setSequence(channel: u5, position: u8) void {
    switch (position) {
        0 => regs.ADC1.SQR3.modify(.{ .SQ1 = channel }),
        1 => regs.ADC1.SQR3.modify(.{ .SQ2 = channel }),
        2 => regs.ADC1.SQR3.modify(.{ .SQ3 = channel }),
        3 => regs.ADC1.SQR3.modify(.{ .SQ4 = channel }),
        4 => regs.ADC1.SQR3.modify(.{ .SQ5 = channel }),
        5 => regs.ADC1.SQR3.modify(.{ .SQ6 = channel }),

        6 => regs.ADC1.SQR2.modify(.{ .SQ7 = channel }),
        7 => regs.ADC1.SQR2.modify(.{ .SQ8 = channel }),
        8 => regs.ADC1.SQR2.modify(.{ .SQ9 = channel }),
        9 => regs.ADC1.SQR2.modify(.{ .SQ10 = channel }),
        10 => regs.ADC1.SQR2.modify(.{ .SQ11 = channel }),
        11 => regs.ADC1.SQR2.modify(.{ .SQ12 = channel }),

        12 => regs.ADC1.SQR1.modify(.{ .SQ13 = channel }),
        13 => regs.ADC1.SQR1.modify(.{ .SQ14 = channel }),
        14 => regs.ADC1.SQR1.modify(.{ .SQ15 = channel }),
        15 => regs.ADC1.SQR1.modify(.{ .SQ16 = channel }),
        else => unreachable, //@compileError("only 16 positions available"),
    }
}

fn setSampleTime(channel: u5, smp: u3) void {
    switch (channel) {
        0 => regs.ADC1.SMPR2.modify(.{ .SMP0 = smp }),
        1 => regs.ADC1.SMPR2.modify(.{ .SMP1 = smp }),
        2 => regs.ADC1.SMPR2.modify(.{ .SMP2 = smp }),
        3 => regs.ADC1.SMPR2.modify(.{ .SMP3 = smp }),
        4 => regs.ADC1.SMPR2.modify(.{ .SMP4 = smp }),
        5 => regs.ADC1.SMPR2.modify(.{ .SMP5 = smp }),
        6 => regs.ADC1.SMPR2.modify(.{ .SMP6 = smp }),
        7 => regs.ADC1.SMPR2.modify(.{ .SMP7 = smp }),
        8 => regs.ADC1.SMPR2.modify(.{ .SMP8 = smp }),
        9 => regs.ADC1.SMPR2.modify(.{ .SMP9 = smp }),

        10 => regs.ADC1.SMPR1.modify(.{ .SMP10 = smp }),
        11 => regs.ADC1.SMPR1.modify(.{ .SMP11 = smp }),
        12 => regs.ADC1.SMPR1.modify(.{ .SMP12 = smp }),
        13 => regs.ADC1.SMPR1.modify(.{ .SMP13 = smp }),
        14 => regs.ADC1.SMPR1.modify(.{ .SMP14 = smp }),
        15 => regs.ADC1.SMPR1.modify(.{ .SMP15 = smp }),
        16 => regs.ADC1.SMPR1.modify(.{ .SMP16 = smp }),
        17 => regs.ADC1.SMPR1.modify(.{ .SMP17 = smp }),
        18 => regs.ADC1.SMPR1.modify(.{ .SMP18 = smp }),
        else => unreachable, //@compileError("unknown channel"),
    }
}

fn setInputPin(channel: u5) void {
    _ = switch (channel) {
        0 => Pin("PA0").Analog(),
        1 => Pin("PA1").Analog(),
        2 => Pin("PA2").Analog(),
        3 => Pin("PA3").Analog(),
        4 => Pin("PA4").Analog(),
        5 => Pin("PA5").Analog(),
        6 => Pin("PA6").Analog(),
        7 => Pin("PA7").Analog(),
        8 => Pin("PB0").Analog(),
        9 => Pin("PB1").Analog(),
        10 => Pin("PC0").Analog(),
        11 => Pin("PC1").Analog(),
        12 => Pin("PC2").Analog(),
        13 => Pin("PC3").Analog(),
        14 => Pin("PC4").Analog(),
        15 => Pin("PC5").Analog(),
        else => unreachable, //@compileError("unknow pin"),
    };
}

pub const IN0 = 1 << 0; // PA0
pub const IN1 = 1 << 1; // PA1
pub const IN2 = 1 << 2; // PA2
pub const IN3 = 1 << 3; // PA3
pub const IN4 = 1 << 4; // PA4
pub const IN5 = 1 << 5; // PA5
pub const IN6 = 1 << 6; // PA6
pub const IN7 = 1 << 7; // PA7
pub const IN8 = 1 << 8; // PB0
pub const IN9 = 1 << 9; // PB1
pub const IN10 = 1 << 10; // PC0
pub const IN11 = 1 << 11; // PC1
pub const IN12 = 1 << 12; // PC2
pub const IN13 = 1 << 13; // PC3
pub const IN14 = 1 << 14; // PC4
pub const IN15 = 1 << 15; // PC5
pub const PA0 = 1 << 0; // IN0
pub const PA1 = 1 << 1; // IN1
pub const PA2 = 1 << 2; // IN2
pub const PA3 = 1 << 3; // IN3
pub const PA4 = 1 << 4; // IN4
pub const PA5 = 1 << 5; // IN5
pub const PA6 = 1 << 6; // IN6
pub const PA7 = 1 << 7; // IN7
pub const PB0 = 1 << 8; // IN8
pub const PB1 = 1 << 9; // IN9
pub const PC0 = 1 << 10; // IN10
pub const PC1 = 1 << 11; // IN11
pub const PC2 = 1 << 12; // IN12
pub const PC3 = 1 << 13; // IN13
pub const PC4 = 1 << 14; // IN14
pub const PC5 = 1 << 15; // IN15
// 16?
pub const VREF = 1 << 17; //  channel 17 internal reference voltage VREFINT
pub const TEMP = 1 << 18; // channel 18 internal temperature sensor
pub const VBAT = 1 << 18; // channel 18 battery voltage
// The VBAT and temperature sensor are connected to the same ADC internal
// channel (ADC1_IN18). Only one conversion, either temperature sensor or VBAT,
// must be selected at a time. When both conversion are enabled simultaneously,
// only the VBAT conversion is performed.

const Input = enum {
    in1,
    in2,
    in3,
};

test "enum array" {
    const inputs = [_]Input{ .in1, .in3 };
    printInputs(inputs[0..]);
}

fn printInputs(inputs: []const Input) void {
    for (inputs) |i| {
        std.debug.print("{}\n", .{i});
    }
}
