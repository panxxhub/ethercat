const CTRL_SWITCH_ON_BIT: u16 = 0x0001;
const CTRL_ENABLE_VOLTAGE_BIT: u16 = 0x0002;
const CTRL_QUICK_STOP_BIT: u16 = 0x0004;
const CTRL_ENABLE_OPERATION_BIT: u16 = 0x0008;
const CTRL_FAULT_REST_BIT: u16 = 0x0080;
// const CTRL_HALT_BIT: u16 = 0x0100;

const MODE_OP_PP: u8 = 0x01;

const STATUS_READY_TO_SWITCH_ON_BIT: u16 = 0x0001;
const STATUS_SWITCHED_ON_BIT: u16 = 0x0002;
const STATUS_OPERATION_ENABLED_BIT: u16 = 0x0004;
const STATUS_FAULT_BIT: u16 = 0x0008;
const STATUS_VOLTAGE_ENABLED_BIT: u16 = 0x0010;
const STATUS_QUICK_STOP_BIT: u16 = 0x0020;

// const FEEDER_1ST_IN_BIT_MASK: u16 = 0x0001;
// const FEEDER_1ST_OUT_BIT: u16 = 0x0020;

const PROFILE_VELOCITY: u32 = 139810133_u32; //1000_u32 * (1 << 23) / 60; // 1000 rpm -> puu/s
const PROFILE_DECELERATION: u32 = (PROFILE_VELOCITY as f64 / 0.42) as u32; // 0.42s to stop
const PROFILE_ACCELERATION: u32 = (PROFILE_VELOCITY as f64 / 0.42) as u32; // 0.42s to start

const CTRL_WORD_NEW_SET_POINT: u16 = 0x0010;

// const STATUS_SET_POINT_BIT: u16 = 0x0800;
const STATUS_TARGET_REACHED_BIT: u16 = 0x0200;

pub(crate) const PDO_SIZE: usize =
    std::mem::size_of::<ServoRxPdo>() + std::mem::size_of::<ServoTxPdo>();

use super::servo_pdo::{ServoPdo, ServoRxPdo, ServoTxPdo};

#[derive(Default)]
pub(crate) struct ServoInitializer {
    pub fsm: ServoInitializerFsm,
}

pub(crate) struct ServoInitializerFsm {
    state: fn(&mut ServoInitializerFsm, &ServoTxPdo, &mut ServoRxPdo) -> bool,
}

impl ServoInitializer {
    pub(crate) fn new() -> Self {
        Self {
            fsm: ServoInitializerFsm::new(),
        }
    }

    pub(crate) fn update(&mut self, pdo: &mut ServoPdo) -> bool {
        (self.fsm.state)(&mut self.fsm, &pdo.tx, &mut pdo.rx)
    }
}

impl ServoInitializerFsm {
    fn new() -> Self {
        Self {
            state: ServoInitializerFsm::fsm_state_start,
        }
    }

    fn fsm_state_start(&mut self, _tx: &ServoTxPdo, rx: &mut ServoRxPdo) -> bool {
        self.state = ServoInitializerFsm::fsm_state_fault_reset;
        rx.control_word = 0;
        rx.target_position = 0;
        rx.profile_velocity = 0;
        rx.profile_acceleration = 0;
        rx.profile_deceleration = 0;
        rx.mode_of_operation = 0;
        false
    }

    fn fsm_state_fault_reset(&mut self, tx: &ServoTxPdo, rx: &mut ServoRxPdo) -> bool {
        if tx.status_word & STATUS_FAULT_BIT == 0 {
            self.state = ServoInitializerFsm::fsm_state_ready_to_switch_on;
        }
        rx.control_word = CTRL_FAULT_REST_BIT;
        rx.target_position = 0;
        rx.profile_velocity = 0;
        rx.profile_acceleration = 0;
        rx.profile_deceleration = 0;
        rx.mode_of_operation = 0;
        false
    }

    fn fsm_state_ready_to_switch_on(&mut self, tx: &ServoTxPdo, rx: &mut ServoRxPdo) -> bool {
        // 0x21
        const CONDITION: u16 = STATUS_QUICK_STOP_BIT | STATUS_READY_TO_SWITCH_ON_BIT;
        const CTRL_WORD: u16 = CTRL_QUICK_STOP_BIT | CTRL_ENABLE_VOLTAGE_BIT;
        if tx.status_word & CONDITION == CONDITION {
            self.state = ServoInitializerFsm::fsm_state_switch_on;
        }
        rx.control_word = CTRL_WORD;

        rx.target_position = 0;
        rx.profile_velocity = 0;
        rx.profile_acceleration = 0;
        rx.profile_deceleration = 0;

        rx.mode_of_operation = MODE_OP_PP;
        false
    }

    fn fsm_state_switch_on(&mut self, tx: &ServoTxPdo, rx: &mut ServoRxPdo) -> bool {
        // 0x33
        const CONDITION: u16 = STATUS_VOLTAGE_ENABLED_BIT
            | STATUS_QUICK_STOP_BIT
            | STATUS_SWITCHED_ON_BIT
            | STATUS_READY_TO_SWITCH_ON_BIT;

        const CTRL_WORD: u16 = CTRL_QUICK_STOP_BIT | CTRL_ENABLE_VOLTAGE_BIT | CTRL_SWITCH_ON_BIT;

        if tx.status_word & CONDITION == CONDITION {
            self.state = ServoInitializerFsm::fsm_state_enable_operation;
        }
        rx.control_word = CTRL_WORD;

        rx.target_position = 0;
        rx.profile_velocity = 0;
        rx.profile_acceleration = 0;
        rx.profile_deceleration = 0;

        rx.mode_of_operation = MODE_OP_PP;
        false
    }

    fn fsm_state_enable_operation(&mut self, tx: &ServoTxPdo, rx: &mut ServoRxPdo) -> bool {
        // 0x37
        const CONDITION: u16 = STATUS_VOLTAGE_ENABLED_BIT
            | STATUS_QUICK_STOP_BIT
            | STATUS_OPERATION_ENABLED_BIT
            | STATUS_SWITCHED_ON_BIT
            | STATUS_READY_TO_SWITCH_ON_BIT;

        const CTRL_WORD: u16 = CTRL_QUICK_STOP_BIT
            | CTRL_ENABLE_VOLTAGE_BIT
            | CTRL_SWITCH_ON_BIT
            | CTRL_ENABLE_OPERATION_BIT;

        if tx.status_word & CONDITION == CONDITION {
            self.state = ServoInitializerFsm::fsm_state_operation_enabled;
        }
        rx.control_word = CTRL_WORD;

        rx.target_position = 0;
        rx.profile_velocity = 0;
        rx.profile_acceleration = 0;
        rx.profile_deceleration = 0;

        rx.mode_of_operation = MODE_OP_PP;
        false
    }

    fn fsm_state_operation_enabled(&mut self, _tx: &ServoTxPdo, rx: &mut ServoRxPdo) -> bool {
        const CTRL_WORD: u16 = CTRL_QUICK_STOP_BIT
            | CTRL_ENABLE_VOLTAGE_BIT
            | CTRL_SWITCH_ON_BIT
            | CTRL_ENABLE_OPERATION_BIT;

        rx.control_word = CTRL_WORD;

        rx.target_position = 0;
        rx.profile_velocity = 0;
        rx.profile_acceleration = 0;
        rx.profile_deceleration = 0;

        rx.mode_of_operation = MODE_OP_PP;
        true
    }
}

impl Default for ServoInitializerFsm {
    fn default() -> Self {
        Self::new()
    }
}

pub(crate) struct ServoMover {
    target_position: i32,
    profile_velocity: u32,
    ready: bool,
    fsm: ServoMoverFsm,
}

struct ServoMoverFsm {
    state: fn(
        &mut ServoMoverFsm,
        target_pos: i32,
        profile_velocity: u32,
        servo_tx: ServoTxPdo,
        servo_rx: &mut ServoRxPdo,
    ) -> bool,
}

impl ServoMover {
    pub(crate) fn set_target(&mut self, target_position: i32) -> bool {
        if !self.ready {
            return false;
        }
        self.target_position = target_position;
        self.fsm.state = ServoMoverFsm::fsm_state_servo_mover_init;
        true
    }
    pub(crate) fn set_profile_velocity(&mut self, rpm: u32) {
        self.profile_velocity = (rpm * (1 << 23)) / 60;
    }

    pub(crate) fn new() -> Self {
        ServoMover {
            ready: true,
            target_position: 0,
            profile_velocity: PROFILE_VELOCITY,
            fsm: ServoMoverFsm {
                state: ServoMoverFsm::fsm_state_servo_mover_init,
            },
        }
    }

    pub(crate) fn update(&mut self, servo_tx: ServoTxPdo, servo_rx: &mut ServoRxPdo) -> bool {
        self.ready = (self.fsm.state)(
            &mut self.fsm,
            self.target_position,
            self.profile_velocity,
            servo_tx,
            servo_rx,
        );
        self.ready
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
        _profile_velocity: u32,
        servo_tx: ServoTxPdo,
        servo_rx: &mut ServoRxPdo,
    ) -> bool {
        let actual_pos = servo_tx.position_actual_value;
        log::debug!(
            "fsm_state_servo_mover_init with actual_pos{} target_pos{}",
            actual_pos,
            target_pos,
        );

        if (actual_pos - target_pos).abs() < 14200 {
            return true;
        }

        if servo_tx.status_word & STATUS_TARGET_REACHED_BIT == STATUS_TARGET_REACHED_BIT {
            self.state = ServoMoverFsm::fsm_state_servo_profile;
        }

        servo_rx.control_word = 0x000F;
        servo_rx.target_position = actual_pos;
        servo_rx.profile_velocity = 0;
        servo_rx.profile_acceleration = 0;
        servo_rx.profile_deceleration = 0;
        servo_rx.mode_of_operation = 1;

        false
    }

    fn fsm_state_servo_profile(
        &mut self,
        target_pos: i32,
        profile_velocity: u32,
        _servo_tx: ServoTxPdo,
        servo_rx: &mut ServoRxPdo,
    ) -> bool {
        log::debug!("fsm_state_servo_profile target_pos{}", target_pos);

        servo_rx.control_word = 0x000F;
        servo_rx.target_position = target_pos;
        servo_rx.profile_velocity = profile_velocity;
        servo_rx.profile_acceleration = PROFILE_ACCELERATION;
        servo_rx.profile_deceleration = PROFILE_DECELERATION;
        servo_rx.mode_of_operation = 1;

        self.state = ServoMoverFsm::fsm_state_servo_trigger_new_set_point;

        false
    }

    fn fsm_state_servo_trigger_new_set_point(
        &mut self,
        target_pos: i32,
        profile_velocity: u32,
        _servo_tx: ServoTxPdo,
        servo_rx: &mut ServoRxPdo,
    ) -> bool {
        // if servo_tx.status_word & STATUS_SET_POINT_BIT == STATUS_SET_POINT_BIT {
        log::debug!("set point triggered",);
        self.state = ServoMoverFsm::fsm_state_servo_wait_for_target_reached;
        // }
        servo_rx.control_word = 0x000F | CTRL_WORD_NEW_SET_POINT;
        servo_rx.target_position = target_pos;
        servo_rx.profile_velocity = profile_velocity;
        servo_rx.profile_acceleration = PROFILE_ACCELERATION;
        servo_rx.profile_deceleration = PROFILE_DECELERATION;
        servo_rx.mode_of_operation = 1;

        false
    }

    fn fsm_state_servo_wait_for_target_reached(
        &mut self,
        target_pos: i32,
        profile_velocity: u32,
        servo_tx: ServoTxPdo,
        servo_rx: &mut ServoRxPdo,
    ) -> bool {
        log::debug!("fsm_state_servo_wait_for_target_reached");

        servo_rx.control_word = 0x000F;
        servo_rx.target_position = target_pos;
        servo_rx.profile_velocity = profile_velocity;
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
        let mut servo: ServoInitializer = Default::default();
        let mut v: [u8; PDO_SIZE] = [
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 129, 212, 253, 254, 80, 2,
        ];
        use std::mem::transmute;
        let servo_pdo: &mut ServoPdo = unsafe { transmute(v.as_mut_ptr()) };

        assert_eq!(servo.update(servo_pdo), false);
        assert_eq!(servo.update(servo_pdo), false);
    }
}
