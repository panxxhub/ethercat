use crate::servo::{
    servo::ServoMover,
    servo_pdo::{ServoRxPdo, ServoTxPdo},
};
const FEEDER_SENSOR_BIT_1: u16 = 0x0001;
const FEEDER_SENSOR_BIT_2: u16 = 0x0002;
const FEEDER_SENSOR_BIT_4: u16 = 0x0008;
const FEEDER_SENSOR_BIT_5: u16 = 0x0010;
const FEEDER_SENSOR_BIT_6: u16 = 0x0020;

const FEEDER_SENSOR_BIT_MASK: u16 =
    FEEDER_SENSOR_BIT_1 + FEEDER_SENSOR_BIT_4 + FEEDER_SENSOR_BIT_5 + FEEDER_SENSOR_BIT_6;

const FEEDER_2ND_OUT_BIT_1: u16 = 0x0001;
const FEEDER_2ND_OUT_BIT_2: u16 = 0x0002;
const FEEDER_2ND_OUT_BIT_3: u16 = 0x0004;
const FEEDER_2ND_OUT_BIT_BKR: u16 = 0x4000;

const FEEDER_2ND_OUT_BIT_MASK: u16 =
    FEEDER_2ND_OUT_BIT_1 | FEEDER_2ND_OUT_BIT_2 | FEEDER_2ND_OUT_BIT_3 | FEEDER_2ND_OUT_BIT_BKR;

const FEEDER_2ND_POS_START: i32 = 132544062;
const FEEDER_2ND_POS_END: i32 = -29858310;

pub(crate) struct Feeder2nd {
    last_d_in: u16,
    fsm: Feeder2ndFsm,
}

impl Feeder2nd {
    fn new() -> Self {
        Feeder2nd {
            last_d_in: 0,
            fsm: Default::default(),
        }
    }

    pub(crate) fn update(
        &mut self,
        servo_tx: ServoTxPdo,
        d_in: u16,
        is_manual: bool,
    ) -> (u16, &ServoRxPdo, bool) {
        if ((self.last_d_in ^ d_in) & FEEDER_2ND_OUT_BIT_2 != 0)
            && (d_in & FEEDER_SENSOR_BIT_2 != 0)
        {
            self.fsm.start_product_passed_count_down(is_manual);
        }
        if self.fsm.count_down() {
            self.fsm.toggle_product_passed();
        }
        self.last_d_in = d_in;
        (self.fsm.state)(&mut self.fsm, servo_tx, d_in, is_manual)
    }

    pub(crate) fn trigger_next(&mut self) {
        self.fsm.next_trigger = true;
    }
}

impl Default for Feeder2nd {
    fn default() -> Self {
        Self::new()
    }
}

struct Feeder2ndFsm {
    kicker_count: u32,
    next_trigger: bool,
    product_passed: bool,
    product_passed_delay: u32,
    state: fn(
        &mut Feeder2ndFsm,
        servo_tx: ServoTxPdo,
        d_in: u16,
        is_manual: bool,
    ) -> (u16, &ServoRxPdo, bool),
    servo_mover: ServoMover,
    d_out: u16,
    rx_pdo: ServoRxPdo,
}

impl Feeder2ndFsm {
    fn new() -> Self {
        Feeder2ndFsm {
            kicker_count: 0,
            next_trigger: false,
            product_passed: false,
            product_passed_delay: 350,
            state: Feeder2ndFsm::fsm_state_init,
            servo_mover: Default::default(),
            d_out: FEEDER_2ND_OUT_BIT_BKR, // we start with the breaker on
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

    fn start_product_passed_count_down(&mut self, is_manual: bool) {
        self.product_passed_delay = if is_manual { 1 } else { 1000 };
    }
    fn toggle_product_passed(&mut self) {
        self.product_passed = true;
    }

    fn count_down(&mut self) -> bool {
        if self.product_passed_delay > 0 {
            self.product_passed_delay -= 1;
            return self.product_passed_delay == 0;
        }
        false
    }

    fn fsm_state_init(
        &mut self,
        servo_tx: ServoTxPdo,
        _d_in: u16,
        _is_manual: bool,
    ) -> (u16, &ServoRxPdo, bool) {
        // let mut servo_rx: ServoRxPdo = self.last_servo_rx;
        self.d_out |= FEEDER_2ND_OUT_BIT_BKR;
        self.servo_mover.set_target(FEEDER_2ND_POS_START);
        self.servo_mover.set_profile_velocity(1500);
        if self.servo_mover.update(servo_tx, &mut self.rx_pdo) {
            self.state = Feeder2ndFsm::fsm_state_start_pending;
        }
        (self.d_out, &self.rx_pdo, false) // copy once
    }

    fn fsm_state_start_pending(
        &mut self,
        _servo_tx: ServoTxPdo,
        d_in: u16,
        is_manual: bool,
    ) -> (u16, &ServoRxPdo, bool) {
        // self.d_out &= !FEEDER_2ND_OUT_BIT_BKR;
        if (d_in & FEEDER_SENSOR_BIT_MASK) > 0 || self.product_passed {
            self.state = Feeder2ndFsm::fsm_state_start_kick_01;
            self.d_out = (self.d_out & !FEEDER_2ND_OUT_BIT_MASK) | FEEDER_2ND_OUT_BIT_1;
            self.kicker_count = if is_manual { 1 } else { 780 };
            self.product_passed = false;
        }
        (self.d_out, &self.rx_pdo, false)
    }

    fn fsm_state_start_kick_01(
        &mut self,
        _servo_tx: ServoTxPdo,
        _d_in: u16,
        is_manual: bool,
    ) -> (u16, &ServoRxPdo, bool) {
        self.kicker_count -= 1;
        if self.kicker_count == 0 {
            self.state = Feeder2ndFsm::fsm_state_start_kick_02;
            self.d_out = (self.d_out & !FEEDER_2ND_OUT_BIT_MASK) | FEEDER_2ND_OUT_BIT_2;
            self.kicker_count = if is_manual { 1 } else { 450 };
        }
        (self.d_out, &self.rx_pdo, false)
    }

    fn fsm_state_start_kick_02(
        &mut self,
        _servo_tx: ServoTxPdo,
        _d_in: u16,
        is_manual: bool,
    ) -> (u16, &ServoRxPdo, bool) {
        self.kicker_count -= 1;
        if self.kicker_count == 0 {
            self.state = Feeder2ndFsm::fsm_state_start_kick_03;
            self.d_out = (self.d_out & !FEEDER_2ND_OUT_BIT_MASK)
                | FEEDER_2ND_OUT_BIT_2
                | FEEDER_2ND_OUT_BIT_3;
            self.kicker_count = if is_manual { 3 } else { 850 };
        }
        (self.d_out, &self.rx_pdo, false)
    }

    fn fsm_state_start_kick_03(
        &mut self,
        _servo_tx: ServoTxPdo,
        _d_in: u16,
        _is_manual: bool,
    ) -> (u16, &ServoRxPdo, bool) {
        self.kicker_count -= 1;
        if (self.kicker_count == 200) || (self.kicker_count == 2) {
            self.d_out = (self.d_out & !FEEDER_2ND_OUT_BIT_MASK) | FEEDER_2ND_OUT_BIT_BKR;
        }
        if self.kicker_count == 0 {
            self.state = Feeder2ndFsm::fsm_state_move_to_end;
            self.servo_mover.set_target(FEEDER_2ND_POS_END);
        }
        (self.d_out, &self.rx_pdo, false)
    }

    fn fsm_state_move_to_end(
        &mut self,
        servo_tx: ServoTxPdo,
        _d_in: u16,
        _is_manual: bool,
    ) -> (u16, &ServoRxPdo, bool) {
        // todo: shall we check feeder 3rd position?
        let target_reached = self.servo_mover.update(servo_tx, &mut self.rx_pdo);
        if target_reached {
            self.state = Feeder2ndFsm::fsm_state_end_pending;
        }
        (self.d_out, &self.rx_pdo, target_reached)
    }

    /// .Wait for the feeder 3rd to take the part
    fn fsm_state_end_pending(
        &mut self,
        _servo_tx: ServoTxPdo,
        _d_in: u16,
        _is_manual: bool,
    ) -> (u16, &ServoRxPdo, bool) {
        if self.next_trigger {
            self.state = Feeder2ndFsm::fsm_state_move_to_start;
            self.servo_mover.set_target(FEEDER_2ND_POS_START);
            self.d_out = (self.d_out & !FEEDER_2ND_OUT_BIT_MASK) | FEEDER_2ND_OUT_BIT_BKR;
            self.next_trigger = false;
        }
        (self.d_out, &self.rx_pdo, false)
    }

    fn fsm_state_move_to_start(
        &mut self,
        servo_tx: ServoTxPdo,
        _d_in: u16,
        _is_manual: bool,
    ) -> (u16, &ServoRxPdo, bool) {
        if self.servo_mover.update(servo_tx, &mut self.rx_pdo) {
            self.state = Feeder2ndFsm::fsm_state_start_pending;
        }
        (self.d_out, &self.rx_pdo, false)
    }
}

impl Default for Feeder2ndFsm {
    fn default() -> Self {
        Self::new()
    }
}
