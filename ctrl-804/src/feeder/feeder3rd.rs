use crate::servo::{
    servo::ServoMover,
    servo_pdo::{ServoRxPdo, ServoTxPdo},
};
const FEEDER_3RD_POS_01: i32 = 800000;
const FEEDER_3RD_POS_02: i32 = 700000;
const FEEDER_3RD_POS_03: i32 = 0;
const FEEDER_3RD_POS_04: i32 = -100000;

pub(crate) struct Feeder3rd {
    fsm: Feeder3rdFsm,
}

struct Feeder3rdFsm {
    kick_count: u32,
    next_trigger: bool,
    state: fn(&mut Self, servo_tx: ServoTxPdo, d_in: u16) -> (u16, &ServoRxPdo),
    servo_mover: ServoMover,

    d_out: u16,
    rx_pdo: ServoRxPdo,
}

impl Feeder3rd {
    fn new() -> Self {
        Self {
            fsm: Feeder3rdFsm::new(),
        }
    }

    pub(crate) fn update(&mut self, servo_tx: ServoTxPdo, d_in: u16) -> (u16, &ServoRxPdo) {
        (self.fsm.state)(&mut self.fsm, servo_tx, d_in)
    }
}

impl Default for Feeder3rd {
    fn default() -> Self {
        Self::new()
    }
}

impl Feeder3rdFsm {
    fn new() -> Self {
        Self {
            kick_count: 0,
            next_trigger: false,
            state: Self::fsm_state_init,
            servo_mover: Default::default(),
            d_out: 0,
            rx_pdo: ServoRxPdo::default(),
        }
    }

    fn fsm_state_init(&mut self, servo_tx: ServoTxPdo, _d_in: u16) -> (u16, &ServoRxPdo) {
        self.state = Feeder3rdFsm::fsm_state_start_pending;
        self.servo_mover.target_position = FEEDER_3RD_POS_03;

        self.servo_mover.update(servo_tx, &mut self.rx_pdo);

        (self.d_out, &self.rx_pdo)
    }

    fn fsm_state_start_pending(&mut self, servo_tx: ServoTxPdo, _d_in: u16) -> (u16, &ServoRxPdo) {
        if self.next_trigger {
            self.state = Feeder3rdFsm::fsm_state_start_move_04;
            self.next_trigger = false;
        }

        self.servo_mover.update(servo_tx, &mut self.rx_pdo);
        (self.d_out, &self.rx_pdo)
    }

    fn fsm_state_start_move_04(&mut self, servo_tx: ServoTxPdo, _d_in: u16) -> (u16, &ServoRxPdo) {
        self.servo_mover.target_position = FEEDER_3RD_POS_04;
        if self.servo_mover.update(servo_tx, &mut self.rx_pdo) {
            self.state = Feeder3rdFsm::fsm_state_start_move_04_clip;
        }
        (self.d_out, &self.rx_pdo)
    }

    fn fsm_state_start_move_04_clip(
        &mut self,
        servo_tx: ServoTxPdo,
        _d_in: u16,
    ) -> (u16, &ServoRxPdo) {
        self.kick_count -= 1;
        if self.kick_count == 0 {
            self.state = Feeder3rdFsm::fsm_state_start_move_01;
            self.kick_count = 0;
        }
        self.servo_mover.update(servo_tx, &mut self.rx_pdo);
        (self.d_out, &self.rx_pdo)
    }

    fn fsm_state_start_move_01(&mut self, servo_tx: ServoTxPdo, _d_in: u16) -> (u16, &ServoRxPdo) {
        self.servo_mover.target_position = FEEDER_3RD_POS_01;
        if self.servo_mover.update(servo_tx, &mut self.rx_pdo) {
            self.state = Feeder3rdFsm::fsm_state_start_move_02;
        }
        (self.d_out, &self.rx_pdo)
    }

    fn fsm_state_start_move_02(&mut self, servo_tx: ServoTxPdo, _d_in: u16) -> (u16, &ServoRxPdo) {
        self.servo_mover.target_position = FEEDER_3RD_POS_02;
        if self.servo_mover.update(servo_tx, &mut self.rx_pdo) {
            self.state = Feeder3rdFsm::fsm_state_start_move_02;
        }
        (self.d_out, &self.rx_pdo)
    }
}
