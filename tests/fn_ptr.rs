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
// const STATUS_WARNING_BIT: u16 = 0x0040;

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
    profile_velocity: i32,
    mode_of_operation: u8,
}

struct Servo {
    fsm: ServoFSM,
}
struct ServoFSM {
    state: fn(&mut ServoFSM, &ServoTxPdo, &mut ServoRxPdo) -> bool,
}

impl Servo {
    pub fn new() -> Servo {
        Servo {
            fsm: Default::default(),
        }
    }
    pub fn update(&mut self, pdo: &mut [u8]) -> bool {
        // transmute
        use std::mem::transmute;
        const RX_PDO_SIZE: usize = std::mem::size_of::<ServoRxPdo>();
        let tx_pdo: &ServoTxPdo = unsafe { transmute(pdo[RX_PDO_SIZE..].as_ptr()) };
        let rx_pdo: &mut ServoRxPdo = unsafe { transmute(pdo.as_mut_ptr()) };
        (self.fsm.state)(&mut self.fsm, tx_pdo, rx_pdo)
    }
}

impl Default for Servo {
    fn default() -> Self {
        Self::new()
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

// test

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {
        let mut servo: Servo = Default::default();
        let mut v: [u8; 17] = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 129, 212, 253, 254, 80, 2];

        assert_eq!(servo.update(&mut v), false);
        assert_eq!(servo.update(&mut v), false);
    }
}
