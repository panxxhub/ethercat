use crate::servo::{
    servo::ServoMover,
    servo_pdo::{ServoRxPdo, ServoTxPdo},
};

const CLIP_01_BIT: u16 = 0x8;
const CLIP_02_BIT: u16 = 0x10;
const SENSOR_3: u16 = 0x4;

const FEEDER_3RD_POS_01: i32 = -271099293;
const FEEDER_3RD_POS_02: i32 = -258935271;
const FEEDER_3RD_POS_03: i32 = -1784845;
const FEEDER_3RD_POS_04: i32 = 39721881;

pub(crate) struct Feeder3rd {
    fsm: Feeder3rdFsm,
}

struct Feeder3rdFsm {
    kick_count: u32,
    next_trigger: bool,
    state: fn(&mut Self, servo_tx: ServoTxPdo, d_in: u16) -> (u16, &ServoRxPdo, bool),
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

    pub(crate) fn update(&mut self, servo_tx: ServoTxPdo, d_in: u16) -> (u16, &ServoRxPdo, bool) {
        (self.fsm.state)(&mut self.fsm, servo_tx, d_in)
    }
    pub(crate) fn trigger_next(&mut self) {
        self.fsm.next_trigger = true;
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
            rx_pdo: ServoRxPdo {
                control_word: 0xF,
                target_position: 0,
                profile_velocity: 0,
                profile_acceleration: 0,
                profile_deceleration: 0,
                mode_of_operation: 1,
            },
        }
    }

    fn fsm_state_init(&mut self, servo_tx: ServoTxPdo, _d_in: u16) -> (u16, &ServoRxPdo, bool) {
        self.servo_mover.set_target(FEEDER_3RD_POS_03);
        if self.servo_mover.update(servo_tx, &mut self.rx_pdo) {
            self.state = Feeder3rdFsm::fsm_state_start_pending;
            self.d_out |= CLIP_01_BIT;
        }
        (self.d_out, &self.rx_pdo, false)
    }

    fn fsm_state_start_pending(
        &mut self,
        _servo_tx: ServoTxPdo,
        _d_in: u16,
    ) -> (u16, &ServoRxPdo, bool) {
        if self.next_trigger {
            self.state = Feeder3rdFsm::fsm_state_start_move_04;
            self.servo_mover.set_target(FEEDER_3RD_POS_04);
            self.next_trigger = false;
        }
        (self.d_out, &self.rx_pdo, false)
    }

    fn fsm_state_start_move_04(
        &mut self,
        servo_tx: ServoTxPdo,
        _d_in: u16,
    ) -> (u16, &ServoRxPdo, bool) {
        if self.servo_mover.update(servo_tx, &mut self.rx_pdo) {
            self.state = Feeder3rdFsm::fsm_state_start_move_04_clip;
            self.d_out &= !CLIP_01_BIT; // toggle off clip, open clip
            self.kick_count = 200;
        }
        (self.d_out, &self.rx_pdo, false)
    }

    fn fsm_state_start_move_04_clip(
        &mut self,
        _servo_tx: ServoTxPdo,
        _d_in: u16,
    ) -> (u16, &ServoRxPdo, bool) {
        self.kick_count -= 1;
        if self.kick_count == 0 {
            self.state = Feeder3rdFsm::fsm_state_start_move_01;
            self.servo_mover.set_target(FEEDER_3RD_POS_01);
            self.kick_count = 0;
        }
        (self.d_out, &self.rx_pdo, false)
    }

    fn fsm_state_start_move_01(
        &mut self,
        servo_tx: ServoTxPdo,
        _d_in: u16,
    ) -> (u16, &ServoRxPdo, bool) {
        if self.servo_mover.update(servo_tx, &mut self.rx_pdo) {
            self.state = Feeder3rdFsm::fsm_state_pos_1_holding;
        }
        (self.d_out, &self.rx_pdo, false)
    }

    fn fsm_state_pos_1_holding(
        &mut self,
        _servo_tx: ServoTxPdo,
        d_in: u16,
    ) -> (u16, &ServoRxPdo, bool) {
        #[cfg(feature = "sensor_3")]
        {
            if d_in & SENSOR_3 != 0 {
                self.state = Feeder3rdFsm::fsm_state_pos_1_to_take_part;
                self.kick_count = 200;
                self.d_out |= CLIP_02_BIT;
            }
        }

        #[cfg(not(feature = "sensor_3"))]
        {
            self.state = Feeder3rdFsm::fsm_state_pos_1_kick;
            self.kick_count = 200;
            self.d_out |= CLIP_02_BIT;
        }

        (self.d_out, &self.rx_pdo, false)
    }

    fn fsm_state_pos_1_to_take_part(
        &mut self,
        _servo_tx: ServoTxPdo,
        _d_in: u16,
    ) -> (u16, &ServoRxPdo, bool) {
        self.kick_count -= 1;
        if self.kick_count == 0 {
            self.servo_mover.set_target(FEEDER_3RD_POS_02);
            self.state = Feeder3rdFsm::fsm_state_start_move_02;
        }
        (self.d_out, &self.rx_pdo, self.kick_count == 0)
    }

    fn fsm_state_start_move_02(
        &mut self,
        servo_tx: ServoTxPdo,
        _d_in: u16,
    ) -> (u16, &ServoRxPdo, bool) {
        if self.servo_mover.update(servo_tx, &mut self.rx_pdo) {
            // target reached
            self.state = Feeder3rdFsm::fsm_state_02_release;
            self.kick_count = 200;
            self.d_out |= CLIP_01_BIT;
        }
        (self.d_out, &self.rx_pdo, false)
    }

    fn fsm_state_02_release(
        &mut self,
        _servo_tx: ServoTxPdo,
        _d_in: u16,
    ) -> (u16, &ServoRxPdo, bool) {
        self.kick_count -= 1;
        if self.kick_count == 0 {
            self.servo_mover.set_target(FEEDER_3RD_POS_01);
            self.state = Feeder3rdFsm::fsm_state_pos_01_empty;
            self.kick_count = 300;
        }
        (self.d_out, &self.rx_pdo, false)
    }

    fn fsm_state_pos_01_empty(
        &mut self,
        servo_tx: ServoTxPdo,
        _d_in: u16,
    ) -> (u16, &ServoRxPdo, bool) {
        if self.kick_count > 0 {
            self.kick_count -= 1;
        } else {
            if self.servo_mover.update(servo_tx, &mut self.rx_pdo) {
                self.state = Feeder3rdFsm::fsm_state_pos_01_empty_pending_step_1;
                self.kick_count = 200;
            }
        }
        (self.d_out, &self.rx_pdo, false)
    }

    fn fsm_state_pos_01_empty_pending_step_1(
        &mut self,
        _servo_tx: ServoTxPdo,
        _d_in: u16,
    ) -> (u16, &ServoRxPdo, bool) {
        self.kick_count -= 1;
        if self.kick_count == 0 {
            self.state = Feeder3rdFsm::fsm_state_pos_01_empty_pending_step_2;
            self.d_out = 0;
            self.kick_count = 300;
        }
        (self.d_out, &self.rx_pdo, false)
    }

    fn fsm_state_pos_01_empty_pending_step_2(
        &mut self,
        _servo_tx: ServoTxPdo,
        _d_in: u16,
    ) -> (u16, &ServoRxPdo, bool) {
        self.kick_count -= 1;
        if self.kick_count == 0 {
            self.state = Feeder3rdFsm::fsm_state_init;
        }
        (self.d_out, &self.rx_pdo, false)
    }
}
