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

const FEEDER_1ST_IN_BIT_MASK: u16 = 0x0001;
const FEEDER_1ST_OUT_BIT: u16 = 0x0020;

const PROFILE_VELOCITY: u32 = 139810133_u32; //1000_u32 * (1 << 23) / 60; // 1000 rpm -> puu/s
const PROFILE_DECELERATION: u32 = (PROFILE_VELOCITY as f64 / 0.42) as u32; // 0.42s to stop
const PROFILE_ACCELERATION: u32 = (PROFILE_VELOCITY as f64 / 0.42) as u32; // 0.42s to start

const CTRL_WORD_NEW_SET_POINT: u16 = 0x000F;

const STATUS_SET_POINT_BIT: u16 = 0x0800;
const STATUS_TARGET_REACHED_BIT: u16 = 0x0200;

pub const PDO_SIZE: usize = std::mem::size_of::<ServoRxPdo>() + std::mem::size_of::<ServoTxPdo>();
pub const RX_PDO_SIZE: usize = std::mem::size_of::<ServoRxPdo>();
pub const TX_PDO_SIZE: usize = std::mem::size_of::<ServoTxPdo>();

use super::servo_pdo::{ServoRxPdo, ServoTxPdo};

#[derive(Default)]
pub(crate) struct Servo {
    pub fsm: ServoFSM,
}

pub(crate) struct ServoFSM {
    state: fn(&mut ServoFSM, &ServoTxPdo, &mut ServoRxPdo) -> bool,
}

impl Servo {
    pub fn update(&mut self, pdo: &mut [u8; PDO_SIZE]) -> bool {
        // transmute
        use std::mem::transmute;
        let tx_pdo: &ServoTxPdo = unsafe { transmute(pdo[RX_PDO_SIZE..].as_ptr()) };
        let rx_pdo: &mut ServoRxPdo = unsafe { transmute(pdo[..RX_PDO_SIZE].as_mut_ptr()) };
        (self.fsm.state)(&mut self.fsm, tx_pdo, rx_pdo)
    }
}

impl ServoFSM {
    /// Creates a new [`ServoFSM`].
    pub fn new() -> Self {
        Self {
            state: ServoFSM::fsm_state_start,
        }
    }

    fn fsm_state_start(&mut self, _tx: &ServoTxPdo, rx: &mut ServoRxPdo) -> bool {
        self.state = ServoFSM::fsm_state_fault_reset;
        rx.control_word = 0;
        rx.target_position = 0;
        rx.profile_velocity = 0;
        rx.mode_of_operation = 0;
        false
    }

    fn fsm_state_fault_reset(&mut self, tx: &ServoTxPdo, rx: &mut ServoRxPdo) -> bool {
        if tx.status_word & STATUS_FAULT_BIT == 0 {
            self.state = ServoFSM::fsm_state_ready_to_switch_on;
        }
        rx.control_word = CTRL_FAULT_REST_BIT;
        rx.mode_of_operation = 0;
        false
    }

    fn fsm_state_ready_to_switch_on(&mut self, tx: &ServoTxPdo, rx: &mut ServoRxPdo) -> bool {
        // 0x21
        const CONDITION: u16 = STATUS_QUICK_STOP_BIT | STATUS_READY_TO_SWITCH_ON_BIT;
        const CTRL_WORD: u16 = CTRL_QUICK_STOP_BIT | CTRL_ENABLE_VOLTAGE_BIT;
        if tx.status_word & CONDITION == CONDITION {
            self.state = ServoFSM::fsm_state_switch_on;
        }
        rx.control_word = CTRL_WORD;
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
            self.state = ServoFSM::fsm_state_enable_operation;
        }
        rx.control_word = CTRL_WORD;
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
            self.state = ServoFSM::fsm_state_operation_enabled;
        }
        rx.control_word = CTRL_WORD;
        rx.mode_of_operation = MODE_OP_PP;
        false
    }

    fn fsm_state_operation_enabled(&mut self, _tx: &ServoTxPdo, rx: &mut ServoRxPdo) -> bool {
        const CTRL_WORD: u16 = CTRL_QUICK_STOP_BIT
            | CTRL_ENABLE_VOLTAGE_BIT
            | CTRL_SWITCH_ON_BIT
            | CTRL_ENABLE_OPERATION_BIT;

        rx.control_word = CTRL_WORD;
        rx.mode_of_operation = MODE_OP_PP;
        true
    }
}

impl Default for ServoFSM {
    fn default() -> Self {
        Self::new()
    }
}

pub(crate) struct ServoMover {
    pub target_position: i32,
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
    pub fn new() -> Self {
        ServoMover {
            target_position: 0,
            fsm: ServoMoverFsm {
                state: ServoMoverFsm::fsm_state_servo_mover_init,
            },
        }
    }

    pub fn update(&mut self, servo_tx: &ServoTxPdo, servo_rx: &mut ServoRxPdo) -> bool {
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
        let mut servo: Servo = Default::default();
        let mut v: [u8; PDO_SIZE] = [
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 129, 212, 253, 254, 80, 2,
        ];

        assert_eq!(servo.update(&mut v), false);
        assert_eq!(servo.update(&mut v), false);
    }
}
