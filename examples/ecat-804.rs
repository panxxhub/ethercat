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

use std::{collections::HashMap, convert::TryFrom, io};

use ethercat::{
    AlState, DomainIdx, Idx, Master, MasterAccess, Offset, PdoCfg, PdoEntryIdx, PdoEntryInfo,
    PdoEntryPos, PdoIdx, SlaveAddr, SlaveId, SlavePos, SmCfg, SubIdx,
};
use tokio::time::{sleep_until, Duration, Instant};

#[tokio::main]
pub async fn main() -> Result<(), std::io::Error> {
    env_logger::init();
    sleep_until(Instant::now() + Duration::from_millis(100)).await;
    let (mut master, domain_idx) = init_master()?;

    // spawn a task to cyclically read data from the EtherCAT master

    let handle = tokio::spawn(async move {
        let cycle_time = Duration::from_micros(25_000);
        let mut next_cycle = Instant::now() + cycle_time;
        let mut servo: Servo = Default::default();

        loop {
            master.receive().unwrap();
            master.domain(domain_idx).process().unwrap();
            let m_state = master.state().unwrap();
            // let _d_state = master.domain(domain_idx).state();
            // log::debug!("Master state: {:?}", m_state);
            // log::debug!("Domain state: {:?}", d_state);
            if m_state.link_up && AlState::try_from(m_state.al_states).unwrap() == AlState::Op {
                // do something with the data
                let raw_data = master.domain_data(domain_idx).unwrap();
                cyclic_work(&mut servo, raw_data);
            }
            master.domain(domain_idx).queue().unwrap();
            master.send().unwrap();

            sleep_until(next_cycle).await;
            next_cycle += cycle_time;
        }
    });

    // join
    handle.await.unwrap();

    Ok(())
}

fn cyclic_work(servo: &mut Servo, domain_data: &mut [u8]) {
    servo.update(domain_data);
}

enum SlaveType {
    Servo,
    DigitalIO,
}

struct SlaveSetting {
    slave_pos_u16: u16,
    vendor_id: u32,
    product_code: u32,
    slave_type: SlaveType,
}

const SLAVE_SETTINGS: [SlaveSetting; 3] = [
    SlaveSetting {
        slave_pos_u16: 0,
        vendor_id: 0x066F_u32,
        product_code: 0x6038_0006_u32,
        slave_type: SlaveType::Servo,
    },
    SlaveSetting {
        slave_pos_u16: 0,
        vendor_id: 0x066F_u32,
        product_code: 0x6038_0006_u32,
        slave_type: SlaveType::Servo,
    },
    SlaveSetting {
        slave_pos_u16: 0,
        vendor_id: 0x066F_u32,
        product_code: 0x6038_0006_u32,
        slave_type: SlaveType::Servo,
    },
];

fn init_master() -> Result<(Master, DomainIdx), io::Error> {
    log::debug!("open master0");
    let mut master = Master::open(0_u32, MasterAccess::ReadWrite)?;
    log::debug!("Reserve master");
    master.reserve()?;

    log::debug!("Create domain");
    let domain_idx = master.create_domain()?;
    log::debug!("Create domain done {:?}", domain_idx);

    let mut offsets: HashMap<SlavePos, HashMap<PdoEntryIdx, (u8, Offset)>> = HashMap::new();

    for setting in SLAVE_SETTINGS {
        let slave_pos = SlavePos::from(setting.slave_pos_u16);
        log::debug!("Request PreOp state for {:?}", slave_pos);
        master.request_state(slave_pos, AlState::PreOp)?;
        let slave_info = master.get_slave_info(slave_pos)?;
        log::info!("Found device {:?}", slave_info);

        let slave_addr = SlaveAddr::ByPos(setting.slave_pos_u16);
        let slave_id = SlaveId {
            vendor_id: setting.vendor_id,
            product_code: setting.product_code,
        };
        log::info!("Configure slave {:?}", slave_id);

        let mut config = master.configure_slave(slave_addr, slave_id)?;
        let mut entry_offsets: HashMap<PdoEntryIdx, (u8, Offset)> = HashMap::new();
        match setting.slave_type {
            SlaveType::DigitalIO => {
                let sm0 = SmCfg::output(0.into());
                let sm1 = SmCfg::output(1.into());
                let sm2 = SmCfg::input(2.into());

                let sm0_pdos = vec![PdoCfg {
                    idx: PdoIdx::from(0x1600),
                    entries: vec![
                        // do
                        PdoEntryInfo {
                            entry_idx: PdoEntryIdx {
                                idx: Idx::from(0x7001),
                                sub_idx: SubIdx::from(1),
                            },
                            bit_len: 8,
                            name: "digital_output_0".to_string(),
                            pos: PdoEntryPos::from(0),
                        },
                    ],
                }];
                let sm1_pdos = vec![PdoCfg {
                    idx: PdoIdx::from(0x1A00),
                    entries: vec![
                        // do
                        PdoEntryInfo {
                            entry_idx: PdoEntryIdx {
                                idx: Idx::from(0x7001),
                                sub_idx: SubIdx::from(2),
                            },
                            bit_len: 8,
                            name: "digital_output_1".to_string(),
                            pos: PdoEntryPos::from(0),
                        },
                    ],
                }];
                let sm2_pdos = vec![PdoCfg {
                    idx: PdoIdx::from(0x1C12),
                    entries: vec![
                        // di
                        PdoEntryInfo {
                            entry_idx: PdoEntryIdx {
                                idx: Idx::from(0x6001),
                                sub_idx: SubIdx::from(1),
                            },
                            bit_len: 8,
                            name: "digital_input_0".to_string(),
                            pos: PdoEntryPos::from(0),
                        },
                        // di
                        PdoEntryInfo {
                            entry_idx: PdoEntryIdx {
                                idx: Idx::from(0x6001),
                                sub_idx: SubIdx::from(2),
                            },
                            bit_len: 8,
                            name: "digital_input_1".to_string(),
                            pos: PdoEntryPos::from(1),
                        },
                    ],
                }];
                config.clear_pdo_assignments(0.into())?;
                config.clear_pdo_assignments(1.into())?;
                config.clear_pdo_assignments(2.into())?;

                config.config_sm_pdos(sm0, &sm0_pdos)?;
                config.config_sm_pdos(sm1, &sm1_pdos)?;
                config.config_sm_pdos(sm2, &sm2_pdos)?;

                for pdo in &sm0_pdos {
                    // Positions of RX PDO
                    log::debug!("Positions of RX PDO 0x{:X}:", u16::from(pdo.idx));
                    for entry in &pdo.entries {
                        let offset = config.register_pdo_entry(entry.entry_idx, domain_idx)?;
                        entry_offsets.insert(entry.entry_idx, (entry.bit_len, offset));
                    }
                }
                for pdo in &sm1_pdos {
                    // Positions of RX PDO
                    log::debug!("Positions of RX PDO 0x{:X}:", u16::from(pdo.idx));
                    for entry in &pdo.entries {
                        let offset = config.register_pdo_entry(entry.entry_idx, domain_idx)?;
                        entry_offsets.insert(entry.entry_idx, (entry.bit_len, offset));
                    }
                }
                for pdo in &sm2_pdos {
                    // Positions of TX PDO
                    log::debug!("Positions of TX PDO 0x{:X}:", u16::from(pdo.idx));
                    for entry in &pdo.entries {
                        let offset = config.register_pdo_entry(entry.entry_idx, domain_idx)?;
                        entry_offsets.insert(entry.entry_idx, (entry.bit_len, offset));
                    }
                }
            }

            SlaveType::Servo => {
                let sm2 = SmCfg::output(2.into());
                let sm3 = SmCfg::input(3.into());

                let sm2_pdos = vec![PdoCfg {
                    idx: PdoIdx::from(0x1600),
                    entries: vec![
                        // control word
                        PdoEntryInfo {
                            entry_idx: PdoEntryIdx {
                                idx: Idx::from(0x6040),
                                sub_idx: SubIdx::from(0),
                            },
                            bit_len: 16,
                            name: "Controlword".to_string(),
                            pos: PdoEntryPos::from(0),
                        },
                        // target position
                        PdoEntryInfo {
                            entry_idx: PdoEntryIdx {
                                idx: Idx::from(0x607A),
                                sub_idx: SubIdx::from(0),
                            },
                            bit_len: 32,
                            name: "Target position".to_string(),
                            pos: PdoEntryPos::from(1),
                        },
                        // profile velocity
                        PdoEntryInfo {
                            entry_idx: PdoEntryIdx {
                                idx: Idx::from(0x6081),
                                sub_idx: SubIdx::from(0),
                            },
                            bit_len: 32,
                            name: "Profile velocity".to_string(),
                            pos: PdoEntryPos::from(2),
                        },
                        // profile acceleration
                        PdoEntryInfo {
                            entry_idx: PdoEntryIdx {
                                idx: Idx::from(0x6083),
                                sub_idx: SubIdx::from(0),
                            },
                            bit_len: 32,
                            name: "Profile acceleration".to_string(),
                            pos: PdoEntryPos::from(3),
                        },
                        // profile deceleration
                        PdoEntryInfo {
                            entry_idx: PdoEntryIdx {
                                idx: Idx::from(0x6084),
                                sub_idx: SubIdx::from(0),
                            },
                            bit_len: 32,
                            name: "Profile deceleration".to_string(),
                            pos: PdoEntryPos::from(4),
                        },
                        // mode of operation
                        PdoEntryInfo {
                            entry_idx: PdoEntryIdx {
                                idx: Idx::from(0x6060),
                                sub_idx: SubIdx::from(0),
                            },
                            bit_len: 8,
                            name: "mode of operation".to_string(),
                            pos: PdoEntryPos::from(5),
                        },
                    ],
                }];

                let sm3_pdos = vec![PdoCfg {
                    idx: PdoIdx::from(0x1A00),
                    entries: vec![
                        // status word
                        PdoEntryInfo {
                            entry_idx: PdoEntryIdx {
                                idx: Idx::from(0x6041),
                                sub_idx: SubIdx::from(0),
                            },
                            bit_len: 16,
                            name: "Statusword".to_string(),
                            pos: PdoEntryPos::from(0),
                        },
                        // actual position
                        PdoEntryInfo {
                            entry_idx: PdoEntryIdx {
                                idx: Idx::from(0x6064),
                                sub_idx: SubIdx::from(0),
                            },
                            bit_len: 32,
                            name: "Actual position".to_string(),
                            pos: PdoEntryPos::from(1),
                        },
                    ],
                }];

                config.clear_emerg()?;

                config.clear_pdo_assignments(2.into())?;
                config.clear_pdo_assignments(3.into())?;

                config.config_sm_pdos(sm2, &sm2_pdos)?;
                config.config_sm_pdos(sm3, &sm3_pdos)?;

                for pdo in &sm2_pdos {
                    // Positions of RX PDO
                    log::debug!("Positions of RX PDO 0x{:X}:", u16::from(pdo.idx));
                    for entry in &pdo.entries {
                        let offset = config.register_pdo_entry(entry.entry_idx, domain_idx)?;
                        entry_offsets.insert(entry.entry_idx, (entry.bit_len, offset));
                    }
                }
                for pdo in &sm3_pdos {
                    // Positions of TX PDO
                    log::debug!("Positions of TX PDO 0x{:X}:", u16::from(pdo.idx));
                    for entry in &pdo.entries {
                        let offset = config.register_pdo_entry(entry.entry_idx, domain_idx)?;
                        entry_offsets.insert(entry.entry_idx, (entry.bit_len, offset));
                    }
                }
            }
        }

        let cfg_index = config.index();
        let cfg_info = master.get_config_info(cfg_index)?;
        log::info!("Config info: {:#?}", cfg_info);
        if cfg_info.slave_position.is_none() {
            return Err(io::Error::new(
                io::ErrorKind::Other,
                "Unable to configure slave",
            ));
        }
        offsets.insert(slave_pos, entry_offsets);
    }

    // app_time nanoseconds from 2000, calc it
    let app_time = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_nanos() as u64
        - 946_684_800_000_000_000_u64;

    master.set_application_time(app_time)?;
    master.sync_reference_clock()?;
    master.sync_slave_clocks()?;

    master.activate()?;

    Ok((master, domain_idx))
}

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
