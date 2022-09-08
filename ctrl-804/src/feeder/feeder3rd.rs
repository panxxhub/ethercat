use crate::servo::servo_pdo::{ServoRxPdo, ServoTxPdo};

const PROFILE_VELOCITY: u32 = 139810133_u32; //1000_u32 * (1 << 23) / 60; // 1000 rpm -> puu/s
const PROFILE_DECELERATION: u32 = (PROFILE_VELOCITY as f64 / 0.42) as u32; // 0.42s to stop
const PROFILE_ACCELERATION: u32 = (PROFILE_VELOCITY as f64 / 0.42) as u32; // 0.42s to start

const CTRL_WORD_NEW_SET_POINT: u16 = 0x000F;

const FEEDER_3RD_POS_01: i32 = 800000;
const FEEDER_3RD_POS_02: i32 = 700000;
const FEEDER_3RD_POS_03: i32 = 0;
const FEEDER_3RD_POS_04: i32 = -100000;

const STATUS_SET_POINT_BIT: u16 = 0x0800;
const STATUS_TARGET_REACHED_BIT: u16 = 0x0200;

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
