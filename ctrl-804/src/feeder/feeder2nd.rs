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

pub(crate) struct Feeder2nd {
    fsm: Feeder2ndFsm,
}

impl Feeder2nd {
    fn new() -> Self {
        Feeder2nd {
            fsm: Default::default(),
        }
    }

    pub(crate) fn update(&mut self, servo_tx: ServoTxPdo, d_in: u16) -> (u16, ServoRxPdo) {
        (self.fsm.state)(&mut self.fsm, servo_tx, d_in)
    }

    // pub(crate) fn trigger_next(&mut self) {
    //     self.fsm.next_trigger = true;
    // }
}

impl Default for Feeder2nd {
    fn default() -> Self {
        Self::new()
    }
}

struct Feeder2ndFsm {
    kicker_count: u8,
    next_trigger: bool,
    state: fn(&mut Feeder2ndFsm, servo_tx: ServoTxPdo, d_in: u16) -> (u16, ServoRxPdo),
    servo_mover: ServoMover,
    last_out: u16,
    last_servo_rx: ServoRxPdo,
}

impl Feeder2ndFsm {
    fn new() -> Self {
        Feeder2ndFsm {
            kicker_count: 0,
            next_trigger: false,
            state: Feeder2ndFsm::fsm_state_init,
            servo_mover: Default::default(),
            last_out: 0,
            last_servo_rx: ServoRxPdo {
                control_word: 0xF,
                target_position: 0,
                profile_velocity: 0,
                profile_acceleration: 0,
                profile_deceleration: 0,
                mode_of_operation: 1,
            },
        }
    }

    fn fsm_state_init(&mut self, servo_tx: ServoTxPdo, _d_in: u16) -> (u16, ServoRxPdo) {
        // let mut servo_rx: ServoRxPdo = self.last_servo_rx;
        self.servo_mover.target_position = FEEDER_2ND_POS_START;
        if self.servo_mover.update(servo_tx, &mut self.last_servo_rx) {
            self.state = Feeder2ndFsm::fsm_state_start_pending;
        }
        (self.last_out, self.last_servo_rx) // copy once
    }

    fn fsm_state_start_pending(&mut self, servo_tx: ServoTxPdo, d_in: u16) -> (u16, ServoRxPdo) {
        if d_in & FEEDER_1ST_IN_BIT_MASK == FEEDER_1ST_IN_BIT_MASK {
            self.state = Feeder2ndFsm::fsm_state_start_kick_01;
            self.kicker_count = 80;
        }

        // self.servo_mover.reset();
        self.servo_mover.update(servo_tx, &mut self.last_servo_rx);
        (self.last_out, self.last_servo_rx)
    }

    fn fsm_state_start_kick_01(&mut self, servo_tx: ServoTxPdo, _d_in: u16) -> (u16, ServoRxPdo) {
        self.kicker_count -= 1;
        if self.kicker_count == 0 {
            self.state = Feeder2ndFsm::fsm_state_start_kick_02;
            self.kicker_count = 80;
        }

        self.servo_mover.update(servo_tx, &mut self.last_servo_rx);
        self.last_out = (self.last_out & !FEEDER_2ND_OUT_BIT_MASK) | FEEDER_2ND_OUT_BIT_1;

        (self.last_out, self.last_servo_rx)
    }

    fn fsm_state_start_kick_02(&mut self, servo_tx: ServoTxPdo, _d_in: u16) -> (u16, ServoRxPdo) {
        self.kicker_count -= 1;
        if self.kicker_count == 0 {
            self.state = Feeder2ndFsm::fsm_state_start_kick_03;
            self.kicker_count = 80;
        }

        self.last_out = (self.last_out & !FEEDER_2ND_OUT_BIT_MASK) | FEEDER_2ND_OUT_BIT_2;
        self.servo_mover.update(servo_tx, &mut self.last_servo_rx);
        (self.last_out, self.last_servo_rx)
    }

    fn fsm_state_start_kick_03(&mut self, servo_tx: ServoTxPdo, _d_in: u16) -> (u16, ServoRxPdo) {
        self.kicker_count -= 1;
        if self.kicker_count == 0 {
            self.state = Feeder2ndFsm::fsm_state_start_move;
            self.kicker_count = 80;
        }
        self.last_out = (self.last_out & !FEEDER_2ND_OUT_BIT_MASK) | FEEDER_2ND_OUT_BIT_3;

        self.servo_mover.update(servo_tx, &mut self.last_servo_rx);
        (self.last_out, self.last_servo_rx)
    }

    fn fsm_state_start_move(&mut self, servo_tx: ServoTxPdo, _d_in: u16) -> (u16, ServoRxPdo) {
        // todo: shall we check feeder 3rd position?

        self.last_out = (self.last_out & !FEEDER_2ND_OUT_BIT_MASK) | FEEDER_2ND_OUT_BIT_BKR;

        self.servo_mover.target_position = FEEDER_2ND_POS_END;
        if self.servo_mover.update(servo_tx, &mut self.last_servo_rx) {
            self.state = Feeder2ndFsm::fsm_state_end_pending;
        }
        (self.last_out, self.last_servo_rx)
    }

    /// .Wait for the feeder 3rd to take the part
    fn fsm_state_end_pending(&mut self, servo_tx: ServoTxPdo, _d_in: u16) -> (u16, ServoRxPdo) {
        if self.next_trigger {
            self.state = Feeder2ndFsm::fsm_state_end_move_2_start;
            self.next_trigger = false;
        }
        self.servo_mover.update(servo_tx, &mut self.last_servo_rx);

        (self.last_out, self.last_servo_rx)
    }

    fn fsm_state_end_move_2_start(
        &mut self,
        servo_tx: ServoTxPdo,
        // servo_rx: &mut ServoRxPdo,
        _d_in: u16, // d_out: &mut [u8; 2],
    ) -> (u16, ServoRxPdo) {
        self.last_out = (self.last_out & !FEEDER_2ND_OUT_BIT_MASK) | FEEDER_2ND_OUT_BIT_BKR;

        self.servo_mover.target_position = FEEDER_2ND_POS_START;
        if self.servo_mover.update(servo_tx, &mut self.last_servo_rx) {
            self.state = Feeder2ndFsm::fsm_state_start_pending;
        }

        (self.last_out, self.last_servo_rx)
    }
}

impl Default for Feeder2ndFsm {
    fn default() -> Self {
        Self::new()
    }
}
