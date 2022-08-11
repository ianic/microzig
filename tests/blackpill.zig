const std = @import("std");
const micro = @import("microzig");
const chip = micro.chip;
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

    var itv = ticker.interval(200);
    while (true) {
        if (itv.ready(200) and blink_enabled) {
            board.led.toggle();
        }
    }
}
