const std = @import("std");
const micro = @import("microzig");
const chip = micro.chip;
const regs = chip.registers;
const board = micro.board;

pub const interrupts = struct {
    pub fn SysTick() void {
        ticker.inc();
    }

    pub fn EXTI0() void {
        if (chip.irq.pending(.exti0)) {
            blink_enabled = !blink_enabled;
        }
    }
};

var ticker = chip.ticker();
var blink_enabled = true;

pub fn main() void {
    board.init(.{});

    var counter = ticker.every(500);
    while (true) {
        if (counter.ready() and blink_enabled) {
            board.led.toggle();
        }
    }
}
