const Pin2 = @import("gpio.zig").Pin2;

pub const gpio = struct {
    pub const pa0 = struct {
        const pin = Pin2("PA0");
        pub const Output = pin.Output;
        pub const Input = pin.Input;
        pub const Analog = pin.Analog;
        pub const adc1 = struct {
            pub const In0 = pin.Analog;
        };
        pub const tim2 = struct {
            pub fn Ch1() type {
                return pin.Function(1);
            }
            pub fn Etr() type {
                return pin.Function(1);
            }
        };
        pub const tim5 = struct {
            pub fn Ch1() type {
                return pin.Function(2);
            }
        };
        pub const usart2 = struct {
            pub fn Cts() type {
                return pin.Function(7);
            }
        };
    };
    pub const pa1 = struct {
        const pin = Pin2("PA1");
        pub const Output = pin.Output;
        pub const Input = pin.Input;
        pub const Analog = pin.Analog;
        pub const adc1 = struct {
            pub const In1 = pin.Analog;
        };
        pub const spi4 = struct {
            pub fn Mosi() type {
                return pin.Function(5);
            }
        };
        pub const tim2 = struct {
            pub fn Ch2() type {
                return pin.Function(1);
            }
        };
        pub const tim5 = struct {
            pub fn Ch2() type {
                return pin.Function(2);
            }
        };
        pub const usart2 = struct {
            pub fn Rts() type {
                return pin.Function(7);
            }
        };
    };
    pub const pa2 = struct {
        const pin = Pin2("PA2");
        pub const Output = pin.Output;
        pub const Input = pin.Input;
        pub const Analog = pin.Analog;
        pub const adc1 = struct {
            pub const In2 = pin.Analog;
        };
        pub const tim2 = struct {
            pub fn Ch3() type {
                return pin.Function(1);
            }
        };
        pub const tim5 = struct {
            pub fn Ch3() type {
                return pin.Function(2);
            }
        };
        pub const tim9 = struct {
            pub fn Ch1() type {
                return pin.Function(3);
            }
        };
        pub const usart2 = struct {
            pub fn Tx() type {
                return pin.Function(7);
            }
        };
    };
    pub const pa3 = struct {
        const pin = Pin2("PA3");
        pub const Output = pin.Output;
        pub const Input = pin.Input;
        pub const Analog = pin.Analog;
        pub const adc1 = struct {
            pub const In3 = pin.Analog;
        };
        pub const tim2 = struct {
            pub fn Ch4() type {
                return pin.Function(1);
            }
        };
        pub const tim5 = struct {
            pub fn Ch4() type {
                return pin.Function(2);
            }
        };
        pub const tim9 = struct {
            pub fn Ch2() type {
                return pin.Function(3);
            }
        };
        pub const usart2 = struct {
            pub fn Rx() type {
                return pin.Function(7);
            }
        };
    };
    pub const pa4 = struct {
        const pin = Pin2("PA4");
        pub const Output = pin.Output;
        pub const Input = pin.Input;
        pub const Analog = pin.Analog;
        pub const adc1 = struct {
            pub const In4 = pin.Analog;
        };
        pub const spi1 = struct {
            pub fn Nss() type {
                return pin.Function(5);
            }
        };
        pub const spi3 = struct {
            pub fn Nss() type {
                return pin.Function(6);
            }
        };
        pub const usart2 = struct {
            pub fn Ck() type {
                return pin.Function(7);
            }
        };
    };
    pub const pa5 = struct {
        const pin = Pin2("PA5");
        pub const Output = pin.Output;
        pub const Input = pin.Input;
        pub const Analog = pin.Analog;
        pub const adc1 = struct {
            pub const In5 = pin.Analog;
        };
        pub const spi1 = struct {
            pub fn Sck() type {
                return pin.Function(5);
            }
        };
        pub const tim2 = struct {
            pub fn Ch1() type {
                return pin.Function(1);
            }
            pub fn Etr() type {
                return pin.Function(1);
            }
        };
    };
    pub const pa6 = struct {
        const pin = Pin2("PA6");
        pub const Output = pin.Output;
        pub const Input = pin.Input;
        pub const Analog = pin.Analog;
        pub const adc1 = struct {
            pub const In6 = pin.Analog;
        };
        pub const sdio = struct {
            pub fn Cmd() type {
                return pin.Function(12);
            }
        };
        pub const spi1 = struct {
            pub fn Miso() type {
                return pin.Function(5);
            }
        };
        pub const tim1 = struct {
            pub fn Bkin() type {
                return pin.Function(1);
            }
        };
        pub const tim3 = struct {
            pub fn Ch1() type {
                return pin.Function(2);
            }
        };
    };
    pub const pa7 = struct {
        const pin = Pin2("PA7");
        pub const Output = pin.Output;
        pub const Input = pin.Input;
        pub const Analog = pin.Analog;
        pub const adc1 = struct {
            pub const In7 = pin.Analog;
        };
        pub const spi1 = struct {
            pub fn Mosi() type {
                return pin.Function(5);
            }
        };
        pub const tim1 = struct {
            pub fn Ch1n() type {
                return pin.Function(1);
            }
        };
        pub const tim3 = struct {
            pub fn Ch2() type {
                return pin.Function(2);
            }
        };
    };
    pub const pa8 = struct {
        const pin = Pin2("PA8");
        pub const Output = pin.Output;
        pub const Input = pin.Input;
        pub const Analog = pin.Analog;
        pub const i2c3 = struct {
            pub fn Scl() type {
                return pin.Function(4);
            }
        };
        pub const rcc = struct {
            pub fn Mco1() type {
                return pin.Function(0);
            }
        };
        pub const sdio = struct {
            pub fn D1() type {
                return pin.Function(12);
            }
        };
        pub const tim1 = struct {
            pub fn Ch1() type {
                return pin.Function(1);
            }
        };
        pub const usart1 = struct {
            pub fn Ck() type {
                return pin.Function(7);
            }
        };
        pub const usb_otg_fs = struct {
            pub fn Sof() type {
                return pin.Function(10);
            }
        };
    };
    pub const pa9 = struct {
        const pin = Pin2("PA9");
        pub const Output = pin.Output;
        pub const Input = pin.Input;
        pub const Analog = pin.Analog;
        pub const i2c3 = struct {
            pub fn Smba() type {
                return pin.Function(4);
            }
        };
        pub const sdio = struct {
            pub fn D2() type {
                return pin.Function(12);
            }
        };
        pub const tim1 = struct {
            pub fn Ch2() type {
                return pin.Function(1);
            }
        };
        pub const usart1 = struct {
            pub fn Tx() type {
                return pin.Function(7);
            }
        };
        pub const usb_otg_fs = struct {
            pub fn Vbus() type {
                return pin.Function(10);
            }
        };
    };
    pub const pa10 = struct {
        const pin = Pin2("PA10");
        pub const Output = pin.Output;
        pub const Input = pin.Input;
        pub const Analog = pin.Analog;
        pub const spi5 = struct {
            pub fn Mosi() type {
                return pin.Function(6);
            }
        };
        pub const tim1 = struct {
            pub fn Ch3() type {
                return pin.Function(1);
            }
        };
        pub const usart1 = struct {
            pub fn Rx() type {
                return pin.Function(7);
            }
        };
        pub const usb_otg_fs = struct {
            pub fn Id() type {
                return pin.Function(10);
            }
        };
    };
    pub const pa11 = struct {
        const pin = Pin2("PA11");
        pub const Output = pin.Output;
        pub const Input = pin.Input;
        pub const Analog = pin.Analog;
        pub const spi4 = struct {
            pub fn Miso() type {
                return pin.Function(6);
            }
        };
        pub const tim1 = struct {
            pub fn Ch4() type {
                return pin.Function(1);
            }
        };
        pub const usart1 = struct {
            pub fn Cts() type {
                return pin.Function(7);
            }
        };
        pub const usart6 = struct {
            pub fn Tx() type {
                return pin.Function(8);
            }
        };
        pub const usb_otg_fs = struct {
            pub fn Dm() type {
                return pin.Function(10);
            }
        };
    };
    pub const pa12 = struct {
        const pin = Pin2("PA12");
        pub const Output = pin.Output;
        pub const Input = pin.Input;
        pub const Analog = pin.Analog;
        pub const spi5 = struct {
            pub fn Miso() type {
                return pin.Function(6);
            }
        };
        pub const tim1 = struct {
            pub fn Etr() type {
                return pin.Function(1);
            }
        };
        pub const usart1 = struct {
            pub fn Rts() type {
                return pin.Function(7);
            }
        };
        pub const usart6 = struct {
            pub fn Rx() type {
                return pin.Function(8);
            }
        };
        pub const usb_otg_fs = struct {
            pub fn Dp() type {
                return pin.Function(10);
            }
        };
    };
    pub const pa13 = struct {
        const pin = Pin2("PA13");
        pub const Output = pin.Output;
        pub const Input = pin.Input;
        pub const Analog = pin.Analog;
    };
    pub const pa14 = struct {
        const pin = Pin2("PA14");
        pub const Output = pin.Output;
        pub const Input = pin.Input;
        pub const Analog = pin.Analog;
    };
    pub const pa15 = struct {
        const pin = Pin2("PA15");
        pub const Output = pin.Output;
        pub const Input = pin.Input;
        pub const Analog = pin.Analog;
        pub const spi1 = struct {
            pub fn Nss() type {
                return pin.Function(5);
            }
        };
        pub const spi3 = struct {
            pub fn Nss() type {
                return pin.Function(6);
            }
        };
        pub const tim2 = struct {
            pub fn Ch1() type {
                return pin.Function(1);
            }
            pub fn Etr() type {
                return pin.Function(1);
            }
        };
        pub const usart1 = struct {
            pub fn Tx() type {
                return pin.Function(7);
            }
        };
    };
    pub const pb0 = struct {
        const pin = Pin2("PB0");
        pub const Output = pin.Output;
        pub const Input = pin.Input;
        pub const Analog = pin.Analog;
        pub const adc1 = struct {
            pub const In8 = pin.Analog;
        };
        pub const spi5 = struct {
            pub fn Sck() type {
                return pin.Function(6);
            }
        };
        pub const tim1 = struct {
            pub fn Ch2n() type {
                return pin.Function(1);
            }
        };
        pub const tim3 = struct {
            pub fn Ch3() type {
                return pin.Function(2);
            }
        };
    };
    pub const pb1 = struct {
        const pin = Pin2("PB1");
        pub const Output = pin.Output;
        pub const Input = pin.Input;
        pub const Analog = pin.Analog;
        pub const adc1 = struct {
            pub const In9 = pin.Analog;
        };
        pub const spi5 = struct {
            pub fn Nss() type {
                return pin.Function(6);
            }
        };
        pub const tim1 = struct {
            pub fn Ch3n() type {
                return pin.Function(1);
            }
        };
        pub const tim3 = struct {
            pub fn Ch4() type {
                return pin.Function(2);
            }
        };
    };
    pub const pb2 = struct {
        const pin = Pin2("PB2");
        pub const Output = pin.Output;
        pub const Input = pin.Input;
        pub const Analog = pin.Analog;
    };
    pub const pb3 = struct {
        const pin = Pin2("PB3");
        pub const Output = pin.Output;
        pub const Input = pin.Input;
        pub const Analog = pin.Analog;
        pub const i2c2 = struct {
            pub fn Sda() type {
                return pin.Function(9);
            }
        };
        pub const spi1 = struct {
            pub fn Sck() type {
                return pin.Function(5);
            }
        };
        pub const spi3 = struct {
            pub fn Sck() type {
                return pin.Function(6);
            }
        };
        pub const tim2 = struct {
            pub fn Ch2() type {
                return pin.Function(1);
            }
        };
        pub const usart1 = struct {
            pub fn Rx() type {
                return pin.Function(7);
            }
        };
    };
    pub const pb4 = struct {
        const pin = Pin2("PB4");
        pub const Output = pin.Output;
        pub const Input = pin.Input;
        pub const Analog = pin.Analog;
        pub const i2c3 = struct {
            pub fn Sda() type {
                return pin.Function(9);
            }
        };
        pub const sdio = struct {
            pub fn D0() type {
                return pin.Function(12);
            }
        };
        pub const spi1 = struct {
            pub fn Miso() type {
                return pin.Function(5);
            }
        };
        pub const spi3 = struct {
            pub fn Miso() type {
                return pin.Function(6);
            }
        };
        pub const tim3 = struct {
            pub fn Ch1() type {
                return pin.Function(2);
            }
        };
    };
    pub const pb5 = struct {
        const pin = Pin2("PB5");
        pub const Output = pin.Output;
        pub const Input = pin.Input;
        pub const Analog = pin.Analog;
        pub const i2c1 = struct {
            pub fn Smba() type {
                return pin.Function(4);
            }
        };
        pub const sdio = struct {
            pub fn D3() type {
                return pin.Function(12);
            }
        };
        pub const spi1 = struct {
            pub fn Mosi() type {
                return pin.Function(5);
            }
        };
        pub const spi3 = struct {
            pub fn Mosi() type {
                return pin.Function(6);
            }
        };
        pub const tim3 = struct {
            pub fn Ch2() type {
                return pin.Function(2);
            }
        };
    };
    pub const pb6 = struct {
        const pin = Pin2("PB6");
        pub const Output = pin.Output;
        pub const Input = pin.Input;
        pub const Analog = pin.Analog;
        pub const i2c1 = struct {
            pub fn Scl() type {
                return pin.Function(4);
            }
        };
        pub const tim4 = struct {
            pub fn Ch1() type {
                return pin.Function(2);
            }
        };
        pub const usart1 = struct {
            pub fn Tx() type {
                return pin.Function(7);
            }
        };
    };
    pub const pb7 = struct {
        const pin = Pin2("PB7");
        pub const Output = pin.Output;
        pub const Input = pin.Input;
        pub const Analog = pin.Analog;
        pub const i2c1 = struct {
            pub fn Sda() type {
                return pin.Function(4);
            }
        };
        pub const sdio = struct {
            pub fn D0() type {
                return pin.Function(12);
            }
        };
        pub const tim4 = struct {
            pub fn Ch2() type {
                return pin.Function(2);
            }
        };
        pub const usart1 = struct {
            pub fn Rx() type {
                return pin.Function(7);
            }
        };
    };
    pub const pb8 = struct {
        const pin = Pin2("PB8");
        pub const Output = pin.Output;
        pub const Input = pin.Input;
        pub const Analog = pin.Analog;
        pub const i2c1 = struct {
            pub fn Scl() type {
                return pin.Function(4);
            }
        };
        pub const i2c3 = struct {
            pub fn Sda() type {
                return pin.Function(9);
            }
        };
        pub const sdio = struct {
            pub fn D4() type {
                return pin.Function(12);
            }
        };
        pub const spi5 = struct {
            pub fn Mosi() type {
                return pin.Function(6);
            }
        };
        pub const tim10 = struct {
            pub fn Ch1() type {
                return pin.Function(3);
            }
        };
        pub const tim4 = struct {
            pub fn Ch3() type {
                return pin.Function(2);
            }
        };
    };
    pub const pb9 = struct {
        const pin = Pin2("PB9");
        pub const Output = pin.Output;
        pub const Input = pin.Input;
        pub const Analog = pin.Analog;
        pub const i2c1 = struct {
            pub fn Sda() type {
                return pin.Function(4);
            }
        };
        pub const i2c2 = struct {
            pub fn Sda() type {
                return pin.Function(9);
            }
        };
        pub const sdio = struct {
            pub fn D5() type {
                return pin.Function(12);
            }
        };
        pub const spi2 = struct {
            pub fn Nss() type {
                return pin.Function(5);
            }
        };
        pub const tim11 = struct {
            pub fn Ch1() type {
                return pin.Function(3);
            }
        };
        pub const tim4 = struct {
            pub fn Ch4() type {
                return pin.Function(2);
            }
        };
    };
    pub const pb10 = struct {
        const pin = Pin2("PB10");
        pub const Output = pin.Output;
        pub const Input = pin.Input;
        pub const Analog = pin.Analog;
        pub const i2c2 = struct {
            pub fn Scl() type {
                return pin.Function(4);
            }
        };
        pub const sdio = struct {
            pub fn D7() type {
                return pin.Function(12);
            }
        };
        pub const spi2 = struct {
            pub fn Sck() type {
                return pin.Function(5);
            }
        };
        pub const tim2 = struct {
            pub fn Ch3() type {
                return pin.Function(1);
            }
        };
    };
    pub const pb12 = struct {
        const pin = Pin2("PB12");
        pub const Output = pin.Output;
        pub const Input = pin.Input;
        pub const Analog = pin.Analog;
        pub const i2c2 = struct {
            pub fn Smba() type {
                return pin.Function(4);
            }
        };
        pub const spi2 = struct {
            pub fn Nss() type {
                return pin.Function(5);
            }
        };
        pub const spi3 = struct {
            pub fn Sck() type {
                return pin.Function(7);
            }
        };
        pub const spi4 = struct {
            pub fn Nss() type {
                return pin.Function(6);
            }
        };
        pub const tim1 = struct {
            pub fn Bkin() type {
                return pin.Function(1);
            }
        };
    };
    pub const pb13 = struct {
        const pin = Pin2("PB13");
        pub const Output = pin.Output;
        pub const Input = pin.Input;
        pub const Analog = pin.Analog;
        pub const spi2 = struct {
            pub fn Sck() type {
                return pin.Function(5);
            }
        };
        pub const spi4 = struct {
            pub fn Sck() type {
                return pin.Function(6);
            }
        };
        pub const tim1 = struct {
            pub fn Ch1n() type {
                return pin.Function(1);
            }
        };
    };
    pub const pb14 = struct {
        const pin = Pin2("PB14");
        pub const Output = pin.Output;
        pub const Input = pin.Input;
        pub const Analog = pin.Analog;
        pub const sdio = struct {
            pub fn D6() type {
                return pin.Function(12);
            }
        };
        pub const spi2 = struct {
            pub fn Miso() type {
                return pin.Function(5);
            }
        };
        pub const tim1 = struct {
            pub fn Ch2n() type {
                return pin.Function(1);
            }
        };
    };
    pub const pb15 = struct {
        const pin = Pin2("PB15");
        pub const Output = pin.Output;
        pub const Input = pin.Input;
        pub const Analog = pin.Analog;
        pub const rtc = struct {
            pub fn Refin() type {
                return pin.Function(0);
            }
        };
        pub const sdio = struct {
            pub fn Ck() type {
                return pin.Function(12);
            }
        };
        pub const spi2 = struct {
            pub fn Mosi() type {
                return pin.Function(5);
            }
        };
        pub const tim1 = struct {
            pub fn Ch3n() type {
                return pin.Function(1);
            }
        };
    };
    pub const pc0 = struct {
        const pin = Pin2("PC0");
        pub const Output = pin.Output;
        pub const Input = pin.Input;
        pub const Analog = pin.Analog;
        pub const adc1 = struct {
            pub const In10 = pin.Analog;
        };
    };
    pub const pc1 = struct {
        const pin = Pin2("PC1");
        pub const Output = pin.Output;
        pub const Input = pin.Input;
        pub const Analog = pin.Analog;
        pub const adc1 = struct {
            pub const In11 = pin.Analog;
        };
    };
    pub const pc2 = struct {
        const pin = Pin2("PC2");
        pub const Output = pin.Output;
        pub const Input = pin.Input;
        pub const Analog = pin.Analog;
        pub const adc1 = struct {
            pub const In12 = pin.Analog;
        };
        pub const spi2 = struct {
            pub fn Miso() type {
                return pin.Function(5);
            }
        };
    };
    pub const pc3 = struct {
        const pin = Pin2("PC3");
        pub const Output = pin.Output;
        pub const Input = pin.Input;
        pub const Analog = pin.Analog;
        pub const adc1 = struct {
            pub const In13 = pin.Analog;
        };
        pub const spi2 = struct {
            pub fn Mosi() type {
                return pin.Function(5);
            }
        };
    };
    pub const pc4 = struct {
        const pin = Pin2("PC4");
        pub const Output = pin.Output;
        pub const Input = pin.Input;
        pub const Analog = pin.Analog;
        pub const adc1 = struct {
            pub const In14 = pin.Analog;
        };
    };
    pub const pc5 = struct {
        const pin = Pin2("PC5");
        pub const Output = pin.Output;
        pub const Input = pin.Input;
        pub const Analog = pin.Analog;
        pub const adc1 = struct {
            pub const In15 = pin.Analog;
        };
    };
    pub const pc6 = struct {
        const pin = Pin2("PC6");
        pub const Output = pin.Output;
        pub const Input = pin.Input;
        pub const Analog = pin.Analog;
        pub const sdio = struct {
            pub fn D6() type {
                return pin.Function(12);
            }
        };
        pub const tim3 = struct {
            pub fn Ch1() type {
                return pin.Function(2);
            }
        };
        pub const usart6 = struct {
            pub fn Tx() type {
                return pin.Function(8);
            }
        };
    };
    pub const pc7 = struct {
        const pin = Pin2("PC7");
        pub const Output = pin.Output;
        pub const Input = pin.Input;
        pub const Analog = pin.Analog;
        pub const sdio = struct {
            pub fn D7() type {
                return pin.Function(12);
            }
        };
        pub const spi2 = struct {
            pub fn Sck() type {
                return pin.Function(5);
            }
        };
        pub const tim3 = struct {
            pub fn Ch2() type {
                return pin.Function(2);
            }
        };
        pub const usart6 = struct {
            pub fn Rx() type {
                return pin.Function(8);
            }
        };
    };
    pub const pc8 = struct {
        const pin = Pin2("PC8");
        pub const Output = pin.Output;
        pub const Input = pin.Input;
        pub const Analog = pin.Analog;
        pub const sdio = struct {
            pub fn D0() type {
                return pin.Function(12);
            }
        };
        pub const tim3 = struct {
            pub fn Ch3() type {
                return pin.Function(2);
            }
        };
        pub const usart6 = struct {
            pub fn Ck() type {
                return pin.Function(8);
            }
        };
    };
    pub const pc9 = struct {
        const pin = Pin2("PC9");
        pub const Output = pin.Output;
        pub const Input = pin.Input;
        pub const Analog = pin.Analog;
        pub const i2c3 = struct {
            pub fn Sda() type {
                return pin.Function(4);
            }
        };
        pub const rcc = struct {
            pub fn Mco2() type {
                return pin.Function(0);
            }
        };
        pub const sdio = struct {
            pub fn D1() type {
                return pin.Function(12);
            }
        };
        pub const tim3 = struct {
            pub fn Ch4() type {
                return pin.Function(2);
            }
        };
    };
    pub const pc10 = struct {
        const pin = Pin2("PC10");
        pub const Output = pin.Output;
        pub const Input = pin.Input;
        pub const Analog = pin.Analog;
        pub const sdio = struct {
            pub fn D2() type {
                return pin.Function(12);
            }
        };
        pub const spi3 = struct {
            pub fn Sck() type {
                return pin.Function(6);
            }
        };
    };
    pub const pc11 = struct {
        const pin = Pin2("PC11");
        pub const Output = pin.Output;
        pub const Input = pin.Input;
        pub const Analog = pin.Analog;
        pub const sdio = struct {
            pub fn D3() type {
                return pin.Function(12);
            }
        };
        pub const spi3 = struct {
            pub fn Miso() type {
                return pin.Function(6);
            }
        };
    };
    pub const pc12 = struct {
        const pin = Pin2("PC12");
        pub const Output = pin.Output;
        pub const Input = pin.Input;
        pub const Analog = pin.Analog;
        pub const sdio = struct {
            pub fn Ck() type {
                return pin.Function(12);
            }
        };
        pub const spi3 = struct {
            pub fn Mosi() type {
                return pin.Function(6);
            }
        };
    };
    pub const pc13 = struct {
        const pin = Pin2("PC13");
        pub const Output = pin.Output;
        pub const Input = pin.Input;
        pub const Analog = pin.Analog;
        pub const rtc = struct {
            pub const Af1 = pin.Analog;
        };
    };
    pub const pc14 = struct {
        const pin = Pin2("PC14");
        pub const Output = pin.Output;
        pub const Input = pin.Input;
        pub const Analog = pin.Analog;
        pub const rcc = struct {
            pub const Osc32In = pin.Analog;
        };
    };
    pub const pc15 = struct {
        const pin = Pin2("PC15");
        pub const Output = pin.Output;
        pub const Input = pin.Input;
        pub const Analog = pin.Analog;
        pub const rcc = struct {
            pub const Osc32Out = pin.Analog;
        };
    };
    pub const pd2 = struct {
        const pin = Pin2("PD2");
        pub const Output = pin.Output;
        pub const Input = pin.Input;
        pub const Analog = pin.Analog;
        pub const sdio = struct {
            pub fn Cmd() type {
                return pin.Function(12);
            }
        };
        pub const tim3 = struct {
            pub fn Etr() type {
                return pin.Function(2);
            }
        };
    };
    pub const ph0 = struct {
        const pin = Pin2("PH0");
        pub const Output = pin.Output;
        pub const Input = pin.Input;
        pub const Analog = pin.Analog;
        pub const rcc = struct {
            pub const OscIn = pin.Analog;
        };
    };
    pub const ph1 = struct {
        const pin = Pin2("PH1");
        pub const Output = pin.Output;
        pub const Input = pin.Input;
        pub const Analog = pin.Analog;
        pub const rcc = struct {
            pub const OscOut = pin.Analog;
        };
    };

    pub const adc1 = struct {
        pub const in0 = struct {
            pub const Pa0 = pa0.adc1.In0;
        };
        pub const in1 = struct {
            pub const Pa1 = pa1.adc1.In1;
        };
        pub const in2 = struct {
            pub const Pa2 = pa2.adc1.In2;
        };
        pub const in3 = struct {
            pub const Pa3 = pa3.adc1.In3;
        };
        pub const in4 = struct {
            pub const Pa4 = pa4.adc1.In4;
        };
        pub const in5 = struct {
            pub const Pa5 = pa5.adc1.In5;
        };
        pub const in6 = struct {
            pub const Pa6 = pa6.adc1.In6;
        };
        pub const in7 = struct {
            pub const Pa7 = pa7.adc1.In7;
        };
        pub const in8 = struct {
            pub const Pb0 = pb0.adc1.In8;
        };
        pub const in9 = struct {
            pub const Pb1 = pb1.adc1.In9;
        };
        pub const in10 = struct {
            pub const Pc0 = pc0.adc1.In10;
        };
        pub const in11 = struct {
            pub const Pc1 = pc1.adc1.In11;
        };
        pub const in12 = struct {
            pub const Pc2 = pc2.adc1.In12;
        };
        pub const in13 = struct {
            pub const Pc3 = pc3.adc1.In13;
        };
        pub const in14 = struct {
            pub const Pc4 = pc4.adc1.In14;
        };
        pub const in15 = struct {
            pub const Pc5 = pc5.adc1.In15;
        };
    };
    pub const i2c1 = struct {
        pub const smba = struct {
            pub const Pb5 = pb5.i2c1.Smba;
        };
        pub const scl = struct {
            pub const Pb6 = pb6.i2c1.Scl;
            pub const Pb8 = pb8.i2c1.Scl;
        };
        pub const sda = struct {
            pub const Pb7 = pb7.i2c1.Sda;
            pub const Pb9 = pb9.i2c1.Sda;
        };
    };
    pub const i2c2 = struct {
        pub const sda = struct {
            pub const Pb3 = pb3.i2c2.Sda;
            pub const Pb9 = pb9.i2c2.Sda;
        };
        pub const scl = struct {
            pub const Pb10 = pb10.i2c2.Scl;
        };
        pub const smba = struct {
            pub const Pb12 = pb12.i2c2.Smba;
        };
    };
    pub const i2c3 = struct {
        pub const scl = struct {
            pub const Pa8 = pa8.i2c3.Scl;
        };
        pub const smba = struct {
            pub const Pa9 = pa9.i2c3.Smba;
        };
        pub const sda = struct {
            pub const Pb4 = pb4.i2c3.Sda;
            pub const Pb8 = pb8.i2c3.Sda;
            pub const Pc9 = pc9.i2c3.Sda;
        };
    };
    pub const rcc = struct {
        pub const mco_1 = struct {
            pub const Pa8 = pa8.rcc.Mco1;
        };
        pub const mco_2 = struct {
            pub const Pc9 = pc9.rcc.Mco2;
        };
        pub const osc32_in = struct {
            pub const Pc14 = pc14.rcc.Osc32In;
        };
        pub const osc32_out = struct {
            pub const Pc15 = pc15.rcc.Osc32Out;
        };
        pub const osc_in = struct {
            pub const Ph0 = ph0.rcc.OscIn;
        };
        pub const osc_out = struct {
            pub const Ph1 = ph1.rcc.OscOut;
        };
    };
    pub const rtc = struct {
        pub const refin = struct {
            pub const Pb15 = pb15.rtc.Refin;
        };
        pub const af1 = struct {
            pub const Pc13 = pc13.rtc.Af1;
        };
    };
    pub const sdio = struct {
        pub const cmd = struct {
            pub const Pa6 = pa6.sdio.Cmd;
            pub const Pd2 = pd2.sdio.Cmd;
        };
        pub const d1 = struct {
            pub const Pa8 = pa8.sdio.D1;
            pub const Pc9 = pc9.sdio.D1;
        };
        pub const d2 = struct {
            pub const Pa9 = pa9.sdio.D2;
            pub const Pc10 = pc10.sdio.D2;
        };
        pub const d0 = struct {
            pub const Pb4 = pb4.sdio.D0;
            pub const Pb7 = pb7.sdio.D0;
            pub const Pc8 = pc8.sdio.D0;
        };
        pub const d3 = struct {
            pub const Pb5 = pb5.sdio.D3;
            pub const Pc11 = pc11.sdio.D3;
        };
        pub const d4 = struct {
            pub const Pb8 = pb8.sdio.D4;
        };
        pub const d5 = struct {
            pub const Pb9 = pb9.sdio.D5;
        };
        pub const d7 = struct {
            pub const Pb10 = pb10.sdio.D7;
            pub const Pc7 = pc7.sdio.D7;
        };
        pub const d6 = struct {
            pub const Pb14 = pb14.sdio.D6;
            pub const Pc6 = pc6.sdio.D6;
        };
        pub const ck = struct {
            pub const Pb15 = pb15.sdio.Ck;
            pub const Pc12 = pc12.sdio.Ck;
        };
    };
    pub const spi1 = struct {
        pub const nss = struct {
            pub const Pa4 = pa4.spi1.Nss;
            pub const Pa15 = pa15.spi1.Nss;
        };
        pub const sck = struct {
            pub const Pa5 = pa5.spi1.Sck;
            pub const Pb3 = pb3.spi1.Sck;
        };
        pub const miso = struct {
            pub const Pa6 = pa6.spi1.Miso;
            pub const Pb4 = pb4.spi1.Miso;
        };
        pub const mosi = struct {
            pub const Pa7 = pa7.spi1.Mosi;
            pub const Pb5 = pb5.spi1.Mosi;
        };
    };
    pub const spi2 = struct {
        pub const nss = struct {
            pub const Pb9 = pb9.spi2.Nss;
            pub const Pb12 = pb12.spi2.Nss;
        };
        pub const sck = struct {
            pub const Pb10 = pb10.spi2.Sck;
            pub const Pb13 = pb13.spi2.Sck;
            pub const Pc7 = pc7.spi2.Sck;
        };
        pub const miso = struct {
            pub const Pb14 = pb14.spi2.Miso;
            pub const Pc2 = pc2.spi2.Miso;
        };
        pub const mosi = struct {
            pub const Pb15 = pb15.spi2.Mosi;
            pub const Pc3 = pc3.spi2.Mosi;
        };
    };
    pub const spi3 = struct {
        pub const nss = struct {
            pub const Pa4 = pa4.spi3.Nss;
            pub const Pa15 = pa15.spi3.Nss;
        };
        pub const sck = struct {
            pub const Pb3 = pb3.spi3.Sck;
            pub const Pb12 = pb12.spi3.Sck;
            pub const Pc10 = pc10.spi3.Sck;
        };
        pub const miso = struct {
            pub const Pb4 = pb4.spi3.Miso;
            pub const Pc11 = pc11.spi3.Miso;
        };
        pub const mosi = struct {
            pub const Pb5 = pb5.spi3.Mosi;
            pub const Pc12 = pc12.spi3.Mosi;
        };
    };
    pub const spi4 = struct {
        pub const mosi = struct {
            pub const Pa1 = pa1.spi4.Mosi;
        };
        pub const miso = struct {
            pub const Pa11 = pa11.spi4.Miso;
        };
        pub const nss = struct {
            pub const Pb12 = pb12.spi4.Nss;
        };
        pub const sck = struct {
            pub const Pb13 = pb13.spi4.Sck;
        };
    };
    pub const spi5 = struct {
        pub const mosi = struct {
            pub const Pa10 = pa10.spi5.Mosi;
            pub const Pb8 = pb8.spi5.Mosi;
        };
        pub const miso = struct {
            pub const Pa12 = pa12.spi5.Miso;
        };
        pub const sck = struct {
            pub const Pb0 = pb0.spi5.Sck;
        };
        pub const nss = struct {
            pub const Pb1 = pb1.spi5.Nss;
        };
    };
    pub const tim1 = struct {
        pub const bkin = struct {
            pub const Pa6 = pa6.tim1.Bkin;
            pub const Pb12 = pb12.tim1.Bkin;
        };
        pub const ch1n = struct {
            pub const Pa7 = pa7.tim1.Ch1n;
            pub const Pb13 = pb13.tim1.Ch1n;
        };
        pub const ch1 = struct {
            pub const Pa8 = pa8.tim1.Ch1;
        };
        pub const ch2 = struct {
            pub const Pa9 = pa9.tim1.Ch2;
        };
        pub const ch3 = struct {
            pub const Pa10 = pa10.tim1.Ch3;
        };
        pub const ch4 = struct {
            pub const Pa11 = pa11.tim1.Ch4;
        };
        pub const etr = struct {
            pub const Pa12 = pa12.tim1.Etr;
        };
        pub const ch2n = struct {
            pub const Pb0 = pb0.tim1.Ch2n;
            pub const Pb14 = pb14.tim1.Ch2n;
        };
        pub const ch3n = struct {
            pub const Pb1 = pb1.tim1.Ch3n;
            pub const Pb15 = pb15.tim1.Ch3n;
        };
    };
    pub const tim10 = struct {
        pub const ch1 = struct {
            pub const Pb8 = pb8.tim10.Ch1;
        };
    };
    pub const tim11 = struct {
        pub const ch1 = struct {
            pub const Pb9 = pb9.tim11.Ch1;
        };
    };
    pub const tim2 = struct {
        pub const ch1 = struct {
            pub const Pa0 = pa0.tim2.Ch1;
            pub const Pa5 = pa5.tim2.Ch1;
            pub const Pa15 = pa15.tim2.Ch1;
        };
        pub const etr = struct {
            pub const Pa0 = pa0.tim2.Etr;
            pub const Pa5 = pa5.tim2.Etr;
            pub const Pa15 = pa15.tim2.Etr;
        };
        pub const ch2 = struct {
            pub const Pa1 = pa1.tim2.Ch2;
            pub const Pb3 = pb3.tim2.Ch2;
        };
        pub const ch3 = struct {
            pub const Pa2 = pa2.tim2.Ch3;
            pub const Pb10 = pb10.tim2.Ch3;
        };
        pub const ch4 = struct {
            pub const Pa3 = pa3.tim2.Ch4;
        };
    };
    pub const tim3 = struct {
        pub const ch1 = struct {
            pub const Pa6 = pa6.tim3.Ch1;
            pub const Pb4 = pb4.tim3.Ch1;
            pub const Pc6 = pc6.tim3.Ch1;
        };
        pub const ch2 = struct {
            pub const Pa7 = pa7.tim3.Ch2;
            pub const Pb5 = pb5.tim3.Ch2;
            pub const Pc7 = pc7.tim3.Ch2;
        };
        pub const ch3 = struct {
            pub const Pb0 = pb0.tim3.Ch3;
            pub const Pc8 = pc8.tim3.Ch3;
        };
        pub const ch4 = struct {
            pub const Pb1 = pb1.tim3.Ch4;
            pub const Pc9 = pc9.tim3.Ch4;
        };
        pub const etr = struct {
            pub const Pd2 = pd2.tim3.Etr;
        };
    };
    pub const tim4 = struct {
        pub const ch1 = struct {
            pub const Pb6 = pb6.tim4.Ch1;
        };
        pub const ch2 = struct {
            pub const Pb7 = pb7.tim4.Ch2;
        };
        pub const ch3 = struct {
            pub const Pb8 = pb8.tim4.Ch3;
        };
        pub const ch4 = struct {
            pub const Pb9 = pb9.tim4.Ch4;
        };
    };
    pub const tim5 = struct {
        pub const ch1 = struct {
            pub const Pa0 = pa0.tim5.Ch1;
        };
        pub const ch2 = struct {
            pub const Pa1 = pa1.tim5.Ch2;
        };
        pub const ch3 = struct {
            pub const Pa2 = pa2.tim5.Ch3;
        };
        pub const ch4 = struct {
            pub const Pa3 = pa3.tim5.Ch4;
        };
    };
    pub const tim9 = struct {
        pub const ch1 = struct {
            pub const Pa2 = pa2.tim9.Ch1;
        };
        pub const ch2 = struct {
            pub const Pa3 = pa3.tim9.Ch2;
        };
    };
    pub const usart1 = struct {
        pub const ck = struct {
            pub const Pa8 = pa8.usart1.Ck;
        };
        pub const tx = struct {
            pub const Pa9 = pa9.usart1.Tx;
            pub const Pa15 = pa15.usart1.Tx;
            pub const Pb6 = pb6.usart1.Tx;
        };
        pub const rx = struct {
            pub const Pa10 = pa10.usart1.Rx;
            pub const Pb3 = pb3.usart1.Rx;
            pub const Pb7 = pb7.usart1.Rx;
        };
        pub const cts = struct {
            pub const Pa11 = pa11.usart1.Cts;
        };
        pub const rts = struct {
            pub const Pa12 = pa12.usart1.Rts;
        };
    };
    pub const usart2 = struct {
        pub const cts = struct {
            pub const Pa0 = pa0.usart2.Cts;
        };
        pub const rts = struct {
            pub const Pa1 = pa1.usart2.Rts;
        };
        pub const tx = struct {
            pub const Pa2 = pa2.usart2.Tx;
        };
        pub const rx = struct {
            pub const Pa3 = pa3.usart2.Rx;
        };
        pub const ck = struct {
            pub const Pa4 = pa4.usart2.Ck;
        };
    };
    pub const usart6 = struct {
        pub const tx = struct {
            pub const Pa11 = pa11.usart6.Tx;
            pub const Pc6 = pc6.usart6.Tx;
        };
        pub const rx = struct {
            pub const Pa12 = pa12.usart6.Rx;
            pub const Pc7 = pc7.usart6.Rx;
        };
        pub const ck = struct {
            pub const Pc8 = pc8.usart6.Ck;
        };
    };
    pub const usb_otg_fs = struct {
        pub const sof = struct {
            pub const Pa8 = pa8.usb_otg_fs.Sof;
        };
        pub const vbus = struct {
            pub const Pa9 = pa9.usb_otg_fs.Vbus;
        };
        pub const id = struct {
            pub const Pa10 = pa10.usb_otg_fs.Id;
        };
        pub const dm = struct {
            pub const Pa11 = pa11.usb_otg_fs.Dm;
        };
        pub const dp = struct {
            pub const Pa12 = pa12.usb_otg_fs.Dp;
        };
    };
};
