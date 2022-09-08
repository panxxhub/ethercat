pub(crate) const FEEDER_1ST_IN_BIT_MASK: u16 = 0x0001;
pub(crate) const FEEDER_1ST_OUT_BIT: u16 = 0x0020;

struct Feeder1st {
    fsm: Feeder1stFsm,
}

struct Feeder1stFsm {
    // state: Feeder1stState,
    state: fn(&mut Feeder1stFsm, d_in: &[u8; 2], d_out: &mut [u8; 2]),
}

impl Feeder1st {
    fn new() -> Self {
        Feeder1st {
            fsm: Feeder1stFsm {
                state: Feeder1stFsm::fsm_state_feeder_stop,
            },
        }
    }

    fn update(&mut self, d_in: &[u8; 2], d_out: &mut [u8; 2]) {
        (self.fsm.state)(&mut self.fsm, d_in, d_out)
    }
}

impl Default for Feeder1st {
    fn default() -> Self {
        Self::new()
    }
}

impl Feeder1stFsm {
    fn fsm_state_feeder_run(&mut self, d_in: &[u8; 2], d_out: &mut [u8; 2]) {
        let d_in_u16 = u16::from_le_bytes(*d_in);
        if d_in_u16 & FEEDER_1ST_IN_BIT_MASK == FEEDER_1ST_IN_BIT_MASK {
            self.state = Feeder1stFsm::fsm_state_feeder_stop;
        }
        use std::mem::transmute;

        let d_out_u16: &mut u16 = unsafe { transmute::<&mut [u8; 2], &mut u16>(d_out) };
        *d_out_u16 |= FEEDER_1ST_OUT_BIT;
    }

    fn fsm_state_feeder_stop(&mut self, d_in: &[u8; 2], d_out: &mut [u8; 2]) {
        let d_in_u16 = u16::from_le_bytes(*d_in);
        if d_in_u16 & FEEDER_1ST_IN_BIT_MASK == 0 {
            self.state = Feeder1stFsm::fsm_state_feeder_run;
        }
        use std::mem::transmute;

        let d_out_u16: &mut u16 = unsafe { transmute::<&mut [u8; 2], &mut u16>(d_out) };
        *d_out_u16 &= !FEEDER_1ST_OUT_BIT;
    }
}
