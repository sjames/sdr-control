// VCO controller.
// A simple struct with some state to manage the VCO frequency.
// Hardware interface via closures

// Sojan James October 2020


pub enum Multiplier {
    Unit,
    Ten,
    Hundred,
    Thousand,
    TenThousand,
    HundredThousand,
    Million,
}

pub struct Vco {
    multiplier_pow: u8,
    frequency_hz: i32,
    encoder_value: Option<i32>,
}

impl Vco {
    pub fn create(freq: u32) -> Vco {
        Vco {
            multiplier_pow: 1,
            frequency_hz: freq as i32,
            encoder_value: None,
        }
    }
    pub fn increment_multiplier(&mut self) {
        if self.multiplier_pow < 6 {
            self.multiplier_pow += 1;
        }
    }
    pub fn decrement_multiplier(&mut self) {
        if self.multiplier_pow > 0 {
            self.multiplier_pow -= 1;
        }
    }

    pub fn set_multiplier(&mut self, mult: Multiplier) {
        self.multiplier_pow = match mult {
            Multiplier::Unit => 0,
            Multiplier::Ten => 1,
            Multiplier::Hundred => 2,
            Multiplier::Thousand => 3,
            Multiplier::TenThousand => 4,
            Multiplier::HundredThousand => 5,
            Multiplier::Million => 6,
        }
    }

    pub fn run_once<E, S>(&mut self, mut get_encoder: E, mut set_freq: S)
    where
        E: FnMut() -> u16,
        S: FnMut(u32),
    {
        // first time
        if self.encoder_value.is_none() {
            self.encoder_value = Some(get_encoder() as i32);
            set_freq(self.frequency_hz as u32);
        } else {
            let encoder = get_encoder();
            let diff: i32 = encoder as i32 - self.encoder_value.unwrap() as i32;
            self.encoder_value = Some(encoder as i32);
            if diff != 0 {
                let delta = diff * 10i32.pow(self.multiplier_pow.into());
                self.frequency_hz = self.frequency_hz + delta;
                set_freq(self.frequency_hz as u32);
            }
        }
    }
}
