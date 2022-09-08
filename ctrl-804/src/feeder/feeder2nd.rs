use crate::{
    feeder::feeder1st::FEEDER_1ST_IN_BIT_MASK,
    servo::{
        servo::ServoMover,
        servo_pdo::{ServoRxPdo, ServoTxPdo},
    },
};

const FEEDER_2ND_OUT_BIT_1: u16 = 0x0001;
const FEEDER_2ND_OUT_BIT_2: u16 = 0x0002;
const FEEDER_2ND_OUT_BIT_3: u16 = 0x0004;
const FEEDER_2ND_OUT_BIT_BKR: u16 = 0x0008;

const FEEDER_2ND_OUT_BIT_MASK: u16 =
    FEEDER_2ND_OUT_BIT_1 | FEEDER_2ND_OUT_BIT_2 | FEEDER_2ND_OUT_BIT_3 | FEEDER_2ND_OUT_BIT_BKR;

const FEEDER_2ND_POS_START: i32 = 0;
const FEEDER_2ND_POS_END: i32 = 167772160;

struct Feeder2nd {
    fsm: Feeder2ndFsm,
}

impl Feeder2nd {
    fn new() -> Self {
        Feeder2nd {
            fsm: Default::default(),
        }
    }
    fn update(
        &mut self,
        servo_tx: &ServoTxPdo,
        servo_rx: &mut ServoRxPdo,
        d_in: &[u8; 2],
        d_out: &mut [u8; 2],
    ) {
        (self.fsm.state)(&mut self.fsm, servo_tx, servo_rx, d_in, d_out);
    }

    fn trigger_next(&mut self) {
        self.fsm.next_trigger = true;
    }
}

impl Default for Feeder2nd {
    fn default() -> Self {
        Self::new()
    }
}

struct Feeder2ndFsm {
    kicker_count: u8,
    next_trigger: bool,

    state: fn(
        &mut Feeder2ndFsm,
        servo_tx: &ServoTxPdo,
        servo_rx: &mut ServoRxPdo,
        d_in: &[u8; 2],
        d_out: &mut [u8; 2],
    ),

    servo_mover: ServoMover,
}

impl Feeder2ndFsm {
    fn new() -> Self {
        Feeder2ndFsm {
            kicker_count: 0,
            next_trigger: false,
            state: Feeder2ndFsm::fsm_state_init,
            servo_mover: Default::default(),
        }
    }

    fn fsm_state_init(
        &mut self,
        servo_tx: &ServoTxPdo,
        servo_rx: &mut ServoRxPdo,
        _d_in: &[u8; 2],
        d_out: &mut [u8; 2],
    ) {
        use std::mem::transmute;
        let d_out_u16: &mut u16 = unsafe { transmute::<&mut [u8; 2], &mut u16>(d_out) };

        *d_out_u16 = (*d_out_u16 & !FEEDER_2ND_OUT_BIT_MASK) | FEEDER_2ND_OUT_BIT_BKR;

        self.servo_mover.target_position = FEEDER_2ND_POS_START;
        if self.servo_mover.update(servo_tx, servo_rx) {
            self.state = Feeder2ndFsm::fsm_state_start_pending;
        }
    }

    fn fsm_state_start_pending(
        &mut self,
        servo_tx: &ServoTxPdo,
        servo_rx: &mut ServoRxPdo,
        d_in: &[u8; 2],
        d_out: &mut [u8; 2],
    ) {
        let d_in_u16 = u16::from_le_bytes(*d_in);
        if d_in_u16 & FEEDER_1ST_IN_BIT_MASK == FEEDER_1ST_IN_BIT_MASK {
            self.state = Feeder2ndFsm::fsm_state_start_kick_01;
            self.kicker_count = 80;
        }
        use std::mem::transmute;
        let d_out_u16: &mut u16 = unsafe { transmute::<&mut [u8; 2], &mut u16>(d_out) };
        *d_out_u16 = (*d_out_u16 & !FEEDER_2ND_OUT_BIT_MASK) | FEEDER_2ND_OUT_BIT_BKR;
        self.servo_mover.update(servo_tx, servo_rx);
    }

    fn fsm_state_start_kick_01(
        &mut self,
        servo_tx: &ServoTxPdo,
        servo_rx: &mut ServoRxPdo,
        _d_in: &[u8; 2],
        d_out: &mut [u8; 2],
    ) {
        self.kicker_count -= 1;
        if self.kicker_count == 0 {
            self.state = Feeder2ndFsm::fsm_state_start_kick_02;
            self.kicker_count = 80;
        }
        use std::mem::transmute;
        let d_out_u16: &mut u16 = unsafe { transmute::<&mut [u8; 2], &mut u16>(d_out) };
        *d_out_u16 = (*d_out_u16 & !FEEDER_2ND_OUT_BIT_MASK) | FEEDER_2ND_OUT_BIT_1;

        self.servo_mover.update(servo_tx, servo_rx);
    }

    fn fsm_state_start_kick_02(
        &mut self,
        servo_tx: &ServoTxPdo,
        servo_rx: &mut ServoRxPdo,
        _d_in: &[u8; 2],
        d_out: &mut [u8; 2],
    ) {
        self.kicker_count -= 1;
        if self.kicker_count == 0 {
            self.state = Feeder2ndFsm::fsm_state_start_kick_03;
            self.kicker_count = 80;
        }
        use std::mem::transmute;
        let d_out_u16: &mut u16 = unsafe { transmute::<&mut [u8; 2], &mut u16>(d_out) };
        *d_out_u16 = (*d_out_u16 & !FEEDER_2ND_OUT_BIT_MASK) | FEEDER_2ND_OUT_BIT_2;

        self.servo_mover.update(servo_tx, servo_rx);
    }

    fn fsm_state_start_kick_03(
        &mut self,
        servo_tx: &ServoTxPdo,
        servo_rx: &mut ServoRxPdo,
        _d_in: &[u8; 2],
        d_out: &mut [u8; 2],
    ) {
        self.kicker_count -= 1;
        if self.kicker_count == 0 {
            self.state = Feeder2ndFsm::fsm_state_start_move;
            self.kicker_count = 80;
        }
        use std::mem::transmute;
        let d_out_u16: &mut u16 = unsafe { transmute::<&mut [u8; 2], &mut u16>(d_out) };

        *d_out_u16 = (*d_out_u16 & !FEEDER_2ND_OUT_BIT_MASK) | FEEDER_2ND_OUT_BIT_3;

        self.servo_mover.update(servo_tx, servo_rx);
    }

    fn fsm_state_start_move(
        &mut self,
        servo_tx: &ServoTxPdo,
        servo_rx: &mut ServoRxPdo,
        _d_in: &[u8; 2],
        d_out: &mut [u8; 2],
    ) {
        // todo: shall we check feeder 3rd position?

        use std::mem::transmute;
        let d_out_u16: &mut u16 = unsafe { transmute::<&mut [u8; 2], &mut u16>(d_out) };
        *d_out_u16 = (*d_out_u16 & !FEEDER_2ND_OUT_BIT_MASK) | FEEDER_2ND_OUT_BIT_BKR;

        self.servo_mover.target_position = FEEDER_2ND_POS_START;
        if self.servo_mover.update(servo_tx, servo_rx) {
            self.state = Feeder2ndFsm::fsm_state_end_pending;
        }
    }

    /// .Wait for the feeder 3rd to take the part
    fn fsm_state_end_pending(
        &mut self,
        servo_tx: &ServoTxPdo,
        servo_rx: &mut ServoRxPdo,
        _d_in: &[u8; 2],
        _d_out: &mut [u8; 2],
    ) {
        if self.next_trigger {
            self.state = Feeder2ndFsm::fsm_state_end_move_2_start;
            self.next_trigger = false;
        }
        self.servo_mover.update(servo_tx, servo_rx);
    }

    fn fsm_state_end_move_2_start(
        &mut self,
        servo_tx: &ServoTxPdo,
        servo_rx: &mut ServoRxPdo,
        _d_in: &[u8; 2],
        d_out: &mut [u8; 2],
    ) {
        use std::mem::transmute;
        let d_out_u16: &mut u16 = unsafe { transmute::<&mut [u8; 2], &mut u16>(d_out) };
        *d_out_u16 = (*d_out_u16 & !FEEDER_2ND_OUT_BIT_MASK) | FEEDER_2ND_OUT_BIT_BKR;

        self.servo_mover.target_position = FEEDER_2ND_POS_START;
        if self.servo_mover.update(servo_tx, servo_rx) {
            self.state = Feeder2ndFsm::fsm_state_start_pending;
        }
    }
}

impl Default for Feeder2ndFsm {
    fn default() -> Self {
        Self::new()
    }
}
