const FEEDER_1ST_IN_BIT_MASK: u16 = 0x0001;
const FEEDER_1ST_OUT_BIT: u16 = 0x0020;

const PROFILE_VELOCITY: u32 = 139810133_u32; //1000_u32 * (1 << 23) / 60; // 1000 rpm -> puu/s
const PROFILE_DECELERATION: u32 = (PROFILE_VELOCITY as f64 / 0.42) as u32; // 0.42s to stop
const PROFILE_ACCELERATION: u32 = (PROFILE_VELOCITY as f64 / 0.42) as u32; // 0.42s to start

const CTRL_WORD_NEW_SET_POINT: u16 = 0x000F;

const STATUS_SET_POINT_BIT: u16 = 0x0800;
const STATUS_TARGET_REACHED_BIT: u16 = 0x0200;

#[repr(C, packed)]
struct ServoTxPdo {
    status_word: u16,
    position_actual_value: i32,
}

#[repr(C, packed)]
#[derive(Clone, Copy)]
struct ServoRxPdo {
    control_word: u16,
    target_position: i32,
    profile_velocity: u32,
    profile_acceleration: u32,
    profile_deceleration: u32,
    mode_of_operation: u8,
}

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

struct Feeder3rd {}

const FEEDER_3RD_POS_01: i32 = 800000;
const FEEDER_3RD_POS_02: i32 = 700000;
const FEEDER_3RD_POS_03: i32 = 0;
const FEEDER_3RD_POS_04: i32 = -100000;

struct Feeder3rdFsm {
    kick_count: u32,
    next_trigger: bool,
    state: fn(&mut Self, &ServoTxPdo, &mut ServoRxPdo, &[u8; 2], &mut [u8; 2]),
    servo_mover: ServoMover,
}

impl Feeder3rdFsm {
    fn fsm_state_init(
        &mut self,
        servo_tx: &ServoTxPdo,
        servo_rx: &mut ServoRxPdo,
        _d_in: &[u8; 2],
        _d_out: &mut [u8; 2],
    ) {
        self.state = Feeder3rdFsm::fsm_state_start_pending;
        self.servo_mover.target_position = FEEDER_3RD_POS_03;
        self.servo_mover.update(servo_tx, servo_rx);
    }

    fn fsm_state_start_pending(
        &mut self,
        servo_tx: &ServoTxPdo,
        servo_rx: &mut ServoRxPdo,
        _d_in: &[u8; 2],
        _d_out: &mut [u8; 2],
    ) {
        if self.next_trigger {
            self.state = Feeder3rdFsm::fsm_state_start_move_04;
            self.next_trigger = false;
        }
        self.servo_mover.update(servo_tx, servo_rx);
    }

    fn fsm_state_start_move_04(
        &mut self,
        servo_tx: &ServoTxPdo,
        servo_rx: &mut ServoRxPdo,
        _d_in: &[u8; 2],
        _d_out: &mut [u8; 2],
    ) {
        self.servo_mover.target_position = FEEDER_3RD_POS_04;
        if self.servo_mover.update(servo_tx, servo_rx) {
            self.state = Feeder3rdFsm::fsm_state_start_move_04_clip;
        }
    }

    fn fsm_state_start_move_04_clip(
        &mut self,
        servo_tx: &ServoTxPdo,
        servo_rx: &mut ServoRxPdo,
        _d_in: &[u8; 2],
        _d_out: &mut [u8; 2],
    ) {
        self.kick_count -= 1;
        if self.kick_count == 0 {
            self.state = Feeder3rdFsm::fsm_state_start_move_01;
            self.kick_count = 0;
        }
        self.servo_mover.update(servo_tx, servo_rx);
    }

    fn fsm_state_start_move_01(
        &mut self,
        servo_tx: &ServoTxPdo,
        servo_rx: &mut ServoRxPdo,
        _d_in: &[u8; 2],
        _d_out: &mut [u8; 2],
    ) {
        self.servo_mover.target_position = FEEDER_3RD_POS_01;
        if self.servo_mover.update(servo_tx, servo_rx) {
            self.state = Feeder3rdFsm::fsm_state_start_move_02;
        }
    }

    fn fsm_state_start_move_02(
        &mut self,
        servo_tx: &ServoTxPdo,
        servo_rx: &mut ServoRxPdo,
        _d_in: &[u8; 2],
        _d_out: &mut [u8; 2],
    ) {
        self.servo_mover.target_position = FEEDER_3RD_POS_02;
        if self.servo_mover.update(servo_tx, servo_rx) {
            self.state = Feeder3rdFsm::fsm_state_start_move_02;
        }
    }
}

struct ServoMover {
    target_position: i32,
    fsm: ServoMoverFsm,
}

struct ServoMoverFsm {
    state: fn(
        &mut ServoMoverFsm,
        target_pos: i32,
        servo_tx: &ServoTxPdo,
        servo_rx: &mut ServoRxPdo,
    ) -> bool,
}

impl ServoMover {
    fn new() -> Self {
        ServoMover {
            target_position: 0,
            fsm: ServoMoverFsm {
                state: ServoMoverFsm::fsm_state_servo_mover_init,
            },
        }
    }

    fn update(&mut self, servo_tx: &ServoTxPdo, servo_rx: &mut ServoRxPdo) -> bool {
        (self.fsm.state)(&mut self.fsm, self.target_position, servo_tx, servo_rx)
    }
}

impl Default for ServoMover {
    fn default() -> Self {
        Self::new()
    }
}

impl ServoMoverFsm {
    fn fsm_state_servo_mover_init(
        &mut self,
        target_pos: i32,
        servo_tx: &ServoTxPdo,
        servo_rx: &mut ServoRxPdo,
    ) -> bool {
        if (servo_tx.position_actual_value - target_pos).abs() < 14200 {
            return true;
        }

        if servo_tx.status_word & STATUS_TARGET_REACHED_BIT == STATUS_TARGET_REACHED_BIT {
            self.state = ServoMoverFsm::fsm_state_servo_profile;
        }

        servo_rx.control_word = 0x000F;
        servo_rx.target_position = servo_tx.position_actual_value;
        servo_rx.profile_velocity = 0;
        servo_rx.profile_acceleration = 0;
        servo_rx.profile_deceleration = 0;
        servo_rx.mode_of_operation = 1;

        false
    }

    fn fsm_state_servo_profile(
        &mut self,
        target_pos: i32,
        _servo_tx: &ServoTxPdo,
        servo_rx: &mut ServoRxPdo,
    ) -> bool {
        servo_rx.control_word = 0x000F;
        servo_rx.target_position = target_pos;
        servo_rx.profile_velocity = PROFILE_VELOCITY;
        servo_rx.profile_acceleration = PROFILE_ACCELERATION;
        servo_rx.profile_deceleration = PROFILE_DECELERATION;
        servo_rx.mode_of_operation = 1;

        self.state = ServoMoverFsm::fsm_state_servo_trigger_new_set_point;

        false
    }

    fn fsm_state_servo_trigger_new_set_point(
        &mut self,
        target_pos: i32,
        servo_tx: &ServoTxPdo,
        servo_rx: &mut ServoRxPdo,
    ) -> bool {
        if servo_tx.status_word & STATUS_SET_POINT_BIT == STATUS_SET_POINT_BIT {
            self.state = ServoMoverFsm::fsm_state_servo_wait_for_target_reached;
        }
        servo_rx.control_word = 0x000F | CTRL_WORD_NEW_SET_POINT;
        servo_rx.target_position = target_pos;
        servo_rx.profile_velocity = PROFILE_VELOCITY;
        servo_rx.profile_acceleration = PROFILE_ACCELERATION;
        servo_rx.profile_deceleration = PROFILE_DECELERATION;
        servo_rx.mode_of_operation = 1;

        false
    }

    fn fsm_state_servo_wait_for_target_reached(
        &mut self,
        target_pos: i32,
        servo_tx: &ServoTxPdo,
        servo_rx: &mut ServoRxPdo,
    ) -> bool {
        servo_rx.control_word = 0x000F;
        servo_rx.target_position = target_pos;
        servo_rx.profile_velocity = PROFILE_VELOCITY;
        servo_rx.profile_acceleration = PROFILE_ACCELERATION;
        servo_rx.profile_deceleration = PROFILE_DECELERATION;
        servo_rx.mode_of_operation = 1;

        if servo_tx.status_word & STATUS_TARGET_REACHED_BIT == STATUS_TARGET_REACHED_BIT {
            self.state = ServoMoverFsm::fsm_state_servo_mover_init;
        }
        false
    }
}

// test

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {
        let mut v: [u8; 2] = [0, 0];

        let mut feeder: Feeder1st = Default::default();
        feeder.update(&[0, 0], &mut v);
        feeder.update(&[0, 0], &mut v);
        feeder.update(&[0, 0], &mut v);

        assert_eq!(v, [0x20, 0]);
        feeder.update(&[1, 0], &mut v);
        feeder.update(&[1, 0], &mut v);

        assert_eq!(v, [0, 0]);
    }

    #[test]
    fn feeder_2nd_fsm_works() {
        let f2nd: Feeder2nd = Default::default();
        // f2nd.update(servo_tx, servo_rx, d_in, d_out)
    }
}
