const FEEDER_1ST_OUT_BIT: u16 = 0x8000;
const FEEDER_BIT_4: u16 = 0x0008;
const FEEDER_BIT_5: u16 = 0x0010;
const FEEDER_BIT_6: u16 = 0x0020;
const FEEDER_BIT_MASK: u16 = 0x0001 + FEEDER_BIT_4 + FEEDER_BIT_5 + FEEDER_BIT_6;

pub(crate) struct Feeder1st {
    fsm: Feeder1stFsm,
}

pub(crate) struct Feeder1stFsm {
    // state: Feeder1stState,
    state: fn(&mut Feeder1stFsm, d_in: u16) -> u16,
    d_out: u16,
}

impl Feeder1st {
    fn new() -> Self {
        Feeder1st {
            fsm: Feeder1stFsm {
                state: Feeder1stFsm::fsm_state_feeder_stop,
                d_out: 0,
            },
        }
    }

    pub(crate) fn update(&mut self, d_in: u16) -> u16 {
        (self.fsm.state)(&mut self.fsm, d_in)
    }
}

impl Default for Feeder1st {
    fn default() -> Self {
        Self::new()
    }
}

impl Feeder1stFsm {
    fn fsm_state_feeder_run(&mut self, d_in: u16) -> u16 {
        if d_in & FEEDER_BIT_MASK != 0 {
            self.state = Feeder1stFsm::fsm_state_feeder_stop;
            self.d_out &= !FEEDER_1ST_OUT_BIT;
        }
        self.d_out
    }

    fn fsm_state_feeder_stop(&mut self, d_in: u16) -> u16 {
        if d_in & FEEDER_BIT_MASK == 0 {
            self.state = Feeder1stFsm::fsm_state_feeder_run;
            self.d_out |= FEEDER_1ST_OUT_BIT;
        }
        self.d_out
    }
}
