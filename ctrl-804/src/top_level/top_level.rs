use crate::{
    feeder::{feeder1st::Feeder1st, feeder2nd::Feeder2nd, feeder3rd::Feeder3rd},
    servo::{
        servo::{ServoInitializer, PDO_SIZE},
        servo_pdo::ServoPdo,
    },
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

    feeder1st: Feeder1st,
    feeder2nd: Feeder2nd,
    feeder3rd: Feeder3rd,
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
            log::debug!("init finished!");
            self.state = TopLevel::state_run;
        }
    }

    // maybe we shall add a state for machine homing

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
            feeder1st: Default::default(),
            feeder2nd: Default::default(),
            feeder3rd: Default::default(),
        }
    }

    fn react(&mut self, data: &mut DomainData) {
        // check if the input is dirty
        let input = data.digital_inputs;
        let last_input = self.last_input;

        if (input ^ last_input) & MODE_SWITCH_BIT != 0 {
            if input & MODE_SWITCH_BIT != 0 {
                // mode switch
                log::debug!("mode switch auto");
                self.state = MachineRunnerFsm::state_auto;
            } else {
                log::debug!("mode switch manual");
                self.state = MachineRunnerFsm::state_manual;
            }
        } else {
            (self.state)(self, data);
        }

        self.last_input = input;
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

        // to suppress warning for the packed struct
        let d_in = data.digital_inputs;
        let d_out_1st = self.feeder1st.update(data.digital_inputs);

        let (d_out2nd, servo0_rx_pdo) = self.feeder2nd.update(data.servos[0].tx, d_in);

        let (d_out3rd, servo1_rx_pdo) = self.feeder3rd.update(data.servos[1].tx, d_in);

        let d_out = (d_out_1st) | (d_out2nd) | (d_out3rd);

        data.digital_outputs = d_out;
        data.servos[0].rx = servo0_rx_pdo;
        data.servos[1].rx = *servo1_rx_pdo;
    }
}
