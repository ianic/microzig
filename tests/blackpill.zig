const std = @import("std");
const micro = @import("microzig");
const chip = micro.chip;
const regs = chip.registers;
const board = micro.board;

pub const interrupts = struct {
    pub fn SysTick() void {
        ticks += 1;
    }

    pub fn EXTI0() void {
        if (chip.irq.pending(.exti0)) {
            blink_enabled = !blink_enabled;
        }
    }
};

var ticks: u32 = 0;
var blink_enabled = true;

pub fn main() void {
    board.init(.{});

    var last: u32 = 0;
    while (true) {
        var current = ticks;
        if (current != last and blink_enabled and current % 500 == 0) {
            last = current;
            board.led.toggle();
        }
    }
}
