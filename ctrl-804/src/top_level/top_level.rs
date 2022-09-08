use crate::servo::servo::{PDO_SIZE, RX_PDO_SIZE, TX_PDO_SIZE};

const DOMAIN_SIZE: usize = 2 * (PDO_SIZE) + 2 * 2;

const MODE_SWITCH_BIT: u16 = 0x8000;
const OP_BUTTON_BIT: u16 = 0x4000;
const BUTTON_MASK: u16 = MODE_SWITCH_BIT | OP_BUTTON_BIT;

struct TopLevel {
    last_input: u16,
    op_pressed_count: u16,
    fsm1: MachineRunnerFsm,
    state: fn(&mut TopLevel, &mut [u8; DOMAIN_SIZE]),
}

struct MachineRunnerFsm {
    state: fn(&mut MachineRunnerFsm, &mut [u8; DOMAIN_SIZE]),
}

impl TopLevel {
    fn react(&mut self, data: &mut [u8; DOMAIN_SIZE]) {
        const DIO_START: usize = (RX_PDO_SIZE + TX_PDO_SIZE) * 2 + 2;
        let dio: u16 = (data[DIO_START] as u16) | ((data[DIO_START + 1] as u16) << 8);
        // check if the input is dirty
        let input = dio & MODE_SWITCH_BIT;
        let last_input = self.last_input & MODE_SWITCH_BIT;

        if input != last_input {
            if input & MODE_SWITCH_BIT != 0 {
                // mode switch
                self.state = TopLevel::state_auto;
            } else {
                self.state = TopLevel::state_manual;
            }
        } else {
            (self.state)(self, data);
        }
    }

    fn state_auto(&mut self, data: &mut [u8; DOMAIN_SIZE]) {
        const DIO_START: usize = (RX_PDO_SIZE + TX_PDO_SIZE) * 2 + 2;
        let dio: u16 = (data[DIO_START] as u16) | ((data[DIO_START + 1] as u16) << 8);
        // check if the input is dirty
        let input = dio & OP_BUTTON_BIT;
        let last_input = self.last_input & OP_BUTTON_BIT;

        if last_input != input {
            self.op_pressed_count += 1;
        }

        if self.op_pressed_count % 2 == 0 {
            // execute the machine
        }
    }
    fn state_manual(&mut self, data: &mut [u8; DOMAIN_SIZE]) {
        const DIO_START: usize = (RX_PDO_SIZE + TX_PDO_SIZE) * 2 + 2;
        let dio: u16 = (data[DIO_START] as u16) | ((data[DIO_START + 1] as u16) << 8);
        // check if the input is dirty
        let input = dio & OP_BUTTON_BIT;
        let last_input = self.last_input & OP_BUTTON_BIT;

        if last_input != input {
            // execute machine
        }
    }
}

impl MachineRunnerFsm {
    fn react(&mut self, data: &mut [u8; DOMAIN_SIZE]) {
        (self.state)(self, data);
    }

    fn state_machine_init(&mut self, data: &mut [u8; DOMAIN_SIZE]) {
        // check if the input is dirty
    }
}
