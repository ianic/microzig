const std = @import("std");
const micro = @import("microzig");
const chip = @import("registers.zig");
const regs = chip.registers;

// zig fmt: off
///****  Cortex-M4 Processor Exceptions Numbers ***************************************************************
// NonMaskableInt_IRQn         = -14,    //!< 2 Non Maskable Interrupt
// MemoryManagement_IRQn       = -12,    //!< 4 Cortex-M4 Memory Management Interrupt
// BusFault_IRQn               = -11,    //!< 5 Cortex-M4 Bus Fault Interrupt
// UsageFault_IRQn             = -10,    //!< 6 Cortex-M4 Usage Fault Interrupt
// SVCall_IRQn                 = -5,     //!< 11 Cortex-M4 SV Call Interrupt
// DebugMonitor_IRQn           = -4,     //!< 12 Cortex-M4 Debug Monitor Interrupt
// PendSV_IRQn                 = -2,     //!< 14 Cortex-M4 Pend SV Interrupt
// SysTick_IRQn                = -1,     //!< 15 Cortex-M4 System Tick Interrupt
///****  STM32 specific Interrupt Numbers *********************************************************************
pub const irq = enum(u8) {
    WWDG               = 0,      // Window WatchDog Interrupt
    PVD                = 1,      // PVD through EXTI Line detection Interrupt
    TAMP_STAMP         = 2,      // Tamper and TimeStamp interrupts through the EXTI line
    RTC_WKUP           = 3,      // RTC Wakeup interrupt through the EXTI line
    FLASH              = 4,      // FLASH global Interrupt
    RCC                = 5,      // RCC global Interrupt
    EXTI0              = 6,      // EXTI Line0 Interrupt
    EXTI1              = 7,      // EXTI Line1 Interrupt
    EXTI2              = 8,      // EXTI Line2 Interrupt
    EXTI3              = 9,      // EXTI Line3 Interrupt
    EXTI4              = 10,     // EXTI Line4 Interrupt
    DMA1_Stream0       = 11,     // DMA1 Stream 0 global Interrupt
    DMA1_Stream1       = 12,     // DMA1 Stream 1 global Interrupt
    DMA1_Stream2       = 13,     // DMA1 Stream 2 global Interrupt
    DMA1_Stream3       = 14,     // DMA1 Stream 3 global Interrupt
    DMA1_Stream4       = 15,     // DMA1 Stream 4 global Interrupt
    DMA1_Stream5       = 16,     // DMA1 Stream 5 global Interrupt
    DMA1_Stream6       = 17,     // DMA1 Stream 6 global Interrupt
    ADC                = 18,     // ADC1, ADC2 and ADC3 global Interrupts
    EXTI9_5            = 23,     // External Line[9:5] Interrupts
    TIM1_BRK_TIM9      = 24,     // TIM1 Break interrupt and TIM9 global interrupt
    TIM1_UP_TIM10      = 25,     // TIM1 Update Interrupt and TIM10 global interrupt
    TIM1_TRG_COM_TIM11 = 26,     // TIM1 Trigger and Commutation Interrupt and TIM11 global interrupt
    TIM1_CC            = 27,     // TIM1 Capture Compare Interrupt
    TIM2               = 28,     // TIM2 global Interrupt
    TIM3               = 29,     // TIM3 global Interrupt
    TIM4               = 30,     // TIM4 global Interrupt
    I2C1_EV            = 31,     // I2C1 Event Interrupt
    I2C1_ER            = 32,     // I2C1 Error Interrupt
    I2C2_EV            = 33,     // I2C2 Event Interrupt
    I2C2_ER            = 34,     // I2C2 Error Interrupt
    SPI1               = 35,     // SPI1 global Interrupt
    SPI2               = 36,     // SPI2 global Interrupt
    USART1             = 37,     // USART1 global Interrupt
    USART2             = 38,     // USART2 global Interrupt
    EXTI15_10          = 40,     // External Line[15:10] Interrupts
    RTC_Alarm          = 41,     // RTC Alarm (A and B) through EXTI Line Interrupt
    OTG_FS_WKUP        = 42,     // USB OTG FS Wakeup through EXTI line interrupt
    DMA1_Stream7       = 47,     // DMA1 Stream7 Interrupt
    SDIO               = 49,     // SDIO global Interrupt
    TIM5               = 50,     // TIM5 global Interrupt
    SPI3               = 51,     // SPI3 global Interrupt
    DMA2_Stream0       = 56,     // DMA2 Stream 0 global Interrupt
    DMA2_Stream1       = 57,     // DMA2 Stream 1 global Interrupt
    DMA2_Stream2       = 58,     // DMA2 Stream 2 global Interrupt
    DMA2_Stream3       = 59,     // DMA2 Stream 3 global Interrupt
    DMA2_Stream4       = 60,     // DMA2 Stream 4 global Interrupt
    OTG_FS             = 67,     // USB OTG FS global Interrupt
    DMA2_Stream5       = 68,     // DMA2 Stream 5 global interrupt
    DMA2_Stream6       = 69,     // DMA2 Stream 6 global interrupt
    DMA2_Stream7       = 70,     // DMA2 Stream 7 global interrupt
    USART6             = 71,     // USART6 global interrupt
    I2C3_EV            = 72,     // I2C3 event interrupt
    I2C3_ER            = 73,     // I2C3 error interrupt
    FPU                = 81,     // FPU global interrupt
    SPI4               = 84,     // SPI4 global Interrupt
    SPI5               = 85,     // SPI5 global Interrupt
};
// zig fmt: on

pub fn enable(irqn: irq) void {
    const bit = irq_bit(irqn);
    switch (@enumToInt(irqn)) {
        0...31 => regs.NVIC.ISER0.modify(.{ .SETENA = bit }),
        32...63 => regs.NVIC.ISER1.modify(.{ .SETENA = bit }),
        else => regs.NVIC.ISER2.modify(.{ .SETENA = bit }),
    }
}

pub const pending_irq = enum {
    exti0,
    exti1,
    exti2,
    exti3,
    exti4,
    exti5,
    exti6,
    exti7,
    exti8,
    exti9,
    exti10,
    exti11,
    exti12,
    exti13,
    exti14,
    exti15,
    // TODO add other irqs
};

pub fn pending(comptime piq: pending_irq) bool {
    const i = @enumToInt(piq);
    if (i >= @enumToInt(pending_irq.exti0) and i <= @enumToInt(pending_irq.exti15)) {
        const suffix = i - @enumToInt(pending_irq.exti0);
        const field_name = "PR" ++ std.fmt.comptimePrint("{d}", .{suffix});
        // read PRx field of the EXTI.PR register
        var reg_value = regs.EXTI.PR.read();
        const is_pending = @field(reg_value, field_name) == 1;
        if (is_pending) {
            // clear pending bit
            @field(reg_value, field_name) = 1;
            regs.EXTI.PR.write(reg_value);
        }
        return is_pending;
    }
    // TODO pending for other interrupt types
    unreachable;

    return switch (piq) {
        .exti0 => {
            const is_pending = regs.EXTI.PR.read().PR0 == 1;
            if (is_pending) {
                regs.EXTI.PR.modify(.{ .PR0 = 1 });
            }
            return is_pending;
        },
        else => unreachable,
    };
}

fn irq_bit(irqn: irq) u32 {
    return @intCast(u32, 1) << @intCast(u5, (@enumToInt(irqn) & 0x1f));
}

const expectEqual = std.testing.expectEqual;
test "irq_bit" {
    try expectEqual(irq_bit(.EXTI0), 0x40);
    try expectEqual(irq_bit(.I2C1_ER), 0x1);
    try expectEqual(irq_bit(.ADC), 0x40000);
}

// NVIC->ISER[IRQn >> 5UL)] = (1UL << (IRQn & 0x1F));

// NVIC_ISER0 bits 0 to 31 are for interrupt 0 to 31, respectively
// NVIC_ISER1 bits 0 to 31 are for interrupt 32 to 63, respectively
// ...

pub const ticks: u32 = 0;

pub const interrupts = struct {
    pub fn SysTick() void {
        ticks += 1;
        // TODO: call handler
    }

    pub fn EXTI0(comptime function: anytype) micro.chip.InterruptVector {
        return .{
            .C = struct {
                fn wrapper() callconv(.C) void {
                    if (regs.EXTI.PR.read().PR0 == 1) {
                        regs.EXTI.PR.modify(.{ .PR0 = 1 });
                        @call(.{ .modifier = .always_inline }, function, .{});
                    }
                }
            }.wrapper,
        };
    }
};
