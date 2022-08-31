const std = @import("std");
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
pub const Irq = enum(u8) {
    wwdg               = 0,      // Window WatchDog Interrupt
    pvd                = 1,      // PVD through EXTI Line detection Interrupt
    tamp_stamp         = 2,      // Tamper and TimeStamp interrupts through the EXTI line
    rtc_wkup           = 3,      // RTC Wakeup interrupt through the EXTI line
    flash              = 4,      // FLASH global Interrupt
    rcc                = 5,      // RCC global Interrupt
    exti0              = 6,      // EXTI Line0 Interrupt
    exti1              = 7,      // EXTI Line1 Interrupt
    exti2              = 8,      // EXTI Line2 Interrupt
    exti3              = 9,      // EXTI Line3 Interrupt
    exti4              = 10,     // EXTI Line4 Interrupt
    dma1_stream0       = 11,     // DMA1 Stream 0 global Interrupt
    dma1_stream1       = 12,     // DMA1 Stream 1 global Interrupt
    dma1_stream2       = 13,     // DMA1 Stream 2 global Interrupt
    dma1_stream3       = 14,     // DMA1 Stream 3 global Interrupt
    dma1_stream4       = 15,     // DMA1 Stream 4 global Interrupt
    dma1_stream5       = 16,     // DMA1 Stream 5 global Interrupt
    dma1_stream6       = 17,     // DMA1 Stream 6 global Interrupt
    adc                = 18,     // ADC1, ADC2 and ADC3 global Interrupts
    exti9_5            = 23,     // External Line[9:5] Interrupts
    tim1_brk_tim9      = 24,     // TIM1 Break interrupt and TIM9 global interrupt
    tim1_up_tim10      = 25,     // TIM1 Update Interrupt and TIM10 global interrupt
    tim1_trg_com_tim11 = 26,     // TIM1 Trigger and Commutation Interrupt and TIM11 global interrupt
    tim1_cc            = 27,     // TIM1 Capture Compare Interrupt
    tim2               = 28,     // TIM2 global Interrupt
    tim3               = 29,     // TIM3 global Interrupt
    tim4               = 30,     // TIM4 global Interrupt
    i2c1_ev            = 31,     // I2C1 Event Interrupt
    i2c1_er            = 32,     // I2C1 Error Interrupt
    i2c2_ev            = 33,     // I2C2 Event Interrupt
    i2c2_er            = 34,     // I2C2 Error Interrupt
    spi1               = 35,     // SPI1 global Interrupt
    spi2               = 36,     // SPI2 global Interrupt
    usart1             = 37,     // USART1 global Interrupt
    usart2             = 38,     // USART2 global Interrupt
    exti15_10          = 40,     // External Line[15:10] Interrupts
    rtc_alarm          = 41,     // RTC Alarm (A and B) through EXTI Line Interrupt
    otg_fs_wkup        = 42,     // USB OTG FS Wakeup through EXTI line interrupt
    dma1_stream7       = 47,     // DMA1 Stream7 Interrupt
    sdio               = 49,     // SDIO global Interrupt
    tim5               = 50,     // TIM5 global Interrupt
    spi3               = 51,     // SPI3 global Interrupt
    dma2_stream0       = 56,     // DMA2 Stream 0 global Interrupt
    dma2_stream1       = 57,     // DMA2 Stream 1 global Interrupt
    dma2_stream2       = 58,     // DMA2 Stream 2 global Interrupt
    dma2_stream3       = 59,     // DMA2 Stream 3 global Interrupt
    dma2_stream4       = 60,     // DMA2 Stream 4 global Interrupt
    otg_fs             = 67,     // USB OTG FS global Interrupt
    dma2_stream5       = 68,     // DMA2 Stream 5 global interrupt
    dma2_stream6       = 69,     // DMA2 Stream 6 global interrupt
    dma2_stream7       = 70,     // DMA2 Stream 7 global interrupt
    usart6             = 71,     // USART6 global interrupt
    i2c3_ev            = 72,     // I2C3 event interrupt
    i2c3_er            = 73,     // I2C3 error interrupt
    fpu                = 81,     // FPU global interrupt
    spi4               = 84,     // SPI4 global Interrupt
    spi5               = 85,     // SPI5 global Interrupt
};
// zig fmt: on

// 0...31  => regs.NVIC.ISER0.modify(.{ .SETENA = bit }),
// 32...63 => regs.NVIC.ISER1.modify(.{ .SETENA = bit }),
// ...
pub fn enable(comptime irq: Irq) void {
    setSERBit(@ptrToInt(regs.NVIC.ISER0), irq);
}

// 0...31  => regs.NVIC.ICER0.modify(.{ .CLRENA = bit }),
// 32...63 => regs.NVIC.ICER1.modify(.{ .CLRENA = bit }),
// ...
pub fn disable(comptime irq: Irq) void {
    setSERBit(@ptrToInt(regs.NVIC.ICER0), irq);
}

// Cortex-M3/4/7 cores use only the four upper bits.
// reg.SCB.AIRCR.PRIGOROUP defines split between preemption priority and sub-priority
// regs.NVIC.IPR[x].modify(.{ .IPR_N[y] = value });
pub fn setPriority(comptime irq: Irq, pri: u4) void {
    const base_addr = @ptrToInt(regs.NVIC.IPR0); // start from addres of the first register IPR0
    const addr = base_addr + 0x1 * @intCast(u32, @enumToInt(irq)); // advance by byte for each irq
    const reg = @intToPtr(*u8, addr); // get pointer to location
    const value: u8 = @intCast(u8, pri) << 4; // shift pri to upper 4 bits, only those are used
    reg.* = value; // update value
}

// start from base_addr
// find word (4 byte) where irqn is
// set bit in that word
// for example 40 is second word ISER1/ICER1, 8 bit (0b1_0000_0000)
fn setSERBit(base_addr: u32, irq: Irq) void {
    const reg_no = @enumToInt(irq) >> 5; // reg 0, 1, 2 ... (every 32 bits is another)
    const addr = base_addr + 0x4 * reg_no; // switch to the address of the register no x
    const reg = @intToPtr(*u32, addr); // get pointer to register
    reg.* = irq_bit(irq); // set bits in that word
}

fn irq_bit(irqn: Irq) u32 {
    return @intCast(u32, 1) << @intCast(u5, (@enumToInt(irqn) & 0x1f));
}

const expectEqual = std.testing.expectEqual;
test "irq_bit" {
    try expectEqual(irq_bit(.exti0), 0x40);
    try expectEqual(irq_bit(.exti15_10), 0b1_0000_0000);
    try expectEqual(irq_bit(.i2c1_er), 0x1);
    try expectEqual(irq_bit(.adc), 0x40000);
}
