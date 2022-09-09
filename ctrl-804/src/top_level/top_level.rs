use crate::servo::{
    servo::{ServoInitializer, PDO_SIZE, RX_PDO_SIZE, TX_PDO_SIZE},
    servo_pdo::ServoPdo,
};

pub const DOMAIN_SIZE: usize = 2 * (PDO_SIZE) + 2 * 2;

const MODE_SWITCH_BIT: u16 = 0x8000;
const OP_BUTTON_BIT: u16 = 0x4000;

#[repr(C, packed)]
pub struct DomainData {
    servos: [ServoPdo; 2],
    digital_outputs: u16,
    digital_inputs: u16,
}

pub struct TopLevel {
    pub machine: MachineRunnerFsm,
    state: fn(&mut TopLevel, &mut DomainData),
    servo_initializers: [ServoInitializer; 2],
}

pub struct MachineRunnerFsm {
    pub last_input: u16,
    pub op_pressed_count: u16,
    state: fn(&mut MachineRunnerFsm, &mut DomainData),
}

impl TopLevel {
    fn new() -> Self {
        Self {
            machine: MachineRunnerFsm::new(),
            state: TopLevel::state_init,
            servo_initializers: [ServoInitializer::new(), ServoInitializer::new()],
        }
    }

    pub fn react(&mut self, data: &mut DomainData) {
        (self.state)(self, data);
    }

    fn state_init(&mut self, data: &mut DomainData) {
        if self
            .servo_initializers
            .iter_mut()
            .enumerate()
            .map(|(i, servo)| servo.update(&mut data.servos[i]))
            .reduce(|a, b| a && b)
            .unwrap()
        {
            self.state = TopLevel::state_run;
        }
    }

    fn state_run(&mut self, data: &mut DomainData) {
        self.machine.react(data);
    }
}

impl Default for TopLevel {
    fn default() -> Self {
        Self::new()
    }
}

impl MachineRunnerFsm {
    fn new() -> Self {
        Self {
            last_input: 0,
            op_pressed_count: 0,
            state: MachineRunnerFsm::state_manual,
        }
    }

    fn react(&mut self, data: &mut DomainData) {
        // check if the input is dirty
        let input = data.digital_inputs & MODE_SWITCH_BIT;
        let last_input = data.digital_outputs & MODE_SWITCH_BIT;

        if input != last_input {
            if input & MODE_SWITCH_BIT != 0 {
                // mode switch
                self.state = MachineRunnerFsm::state_auto;
            } else {
                self.state = MachineRunnerFsm::state_manual;
            }
        } else {
            (self.state)(self, data);
        }
    }

    fn state_auto(&mut self, data: &mut DomainData) {
        // check if the input is dirty
        let input = data.digital_inputs;
        let last_input = self.last_input;

        if (input ^ last_input) & OP_BUTTON_BIT != 0 {
            self.op_pressed_count += 1;
        }

        if self.op_pressed_count % 2 == 0 {
            self.run_once(data);
        }
        self.last_input = input;
    }
    fn state_manual(&mut self, data: &mut DomainData) {
        // check if the input is dirty
        //& OP_BUTTON_BIT;
        let input = data.digital_inputs;
        let last_input = self.last_input;

        if (input ^ last_input) & OP_BUTTON_BIT != 0 {
            // execute machine
            self.run_once(data);
        }

        self.last_input = input;
    }

    fn run_once(&mut self, data: &mut DomainData) {
        // run feeder1st, feeder2nd, feeder3rd
    }
}
