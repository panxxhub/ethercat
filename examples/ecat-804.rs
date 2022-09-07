use byteorder::{BigEndian, ReadBytesExt};
use std::{collections::HashMap, convert::TryFrom, io};

use ethercat::{
    AlState, DomainIdx, Idx, Master, MasterAccess, Offset, PdoCfg, PdoEntryIdx, PdoEntryInfo,
    PdoEntryPos, PdoIdx, SlaveAddr, SlaveId, SlavePos, SmCfg, SubIdx,
};
use tokio::time::{sleep_until, Duration, Instant};

#[repr(C, packed)]
struct MyRxPdo {
    mode_of_operation: u8,
    control_word: u16,
    target_position: i32,
    profile_velocity: i32,
}
#[repr(C, packed)]
struct MyTxPdo {
    status_word: u16,
    position_actual_value: i32,
}
#[repr(C, packed)]
struct MyDomainData {
    rx: MyRxPdo,
    tx: MyTxPdo,
}

// impl try from
impl TryFrom<&mut [u8]> for MyRxPdo {
    type Error = io::Error;

    fn try_from(value: &mut [u8]) -> Result<Self, Self::Error> {
        let mut rdr = io::Cursor::new(value);
        Ok(Self {
            mode_of_operation: rdr.read_u8()?,
            control_word: rdr.read_u16::<BigEndian>()?,
            target_position: rdr.read_i32::<BigEndian>()?,
            profile_velocity: rdr.read_i32::<BigEndian>()?,
        })
    }
}
impl TryFrom<&[u8]> for MyTxPdo {
    type Error = io::Error;

    fn try_from(value: &[u8]) -> Result<Self, Self::Error> {
        let mut rdr = io::Cursor::new(value);
        Ok(Self {
            status_word: rdr.read_u16::<BigEndian>()?,
            position_actual_value: rdr.read_i32::<BigEndian>()?,
        })
    }
}

impl TryFrom<&mut [u8]> for MyDomainData {
    type Error = io::Error;

    fn try_from(value: &mut [u8]) -> Result<Self, Self::Error> {
        Ok(Self {
            rx: MyRxPdo::try_from(&mut value[0..11])?,
            tx: MyTxPdo::try_from(&value[11..17])?,
        })
    }
}

// read

#[tokio::main]
pub async fn main() -> Result<(), std::io::Error> {
    env_logger::init();
    sleep_until(Instant::now() + Duration::from_millis(100)).await;
    let (mut master, domain_idx) = init_master()?;

    // spawn a task to cyclically read data from the EtherCAT master

    let handle = tokio::spawn(async move {
        let cycle_time = Duration::from_micros(25_000);
        let mut next_cycle = Instant::now() + cycle_time;
        loop {
            master.receive().unwrap();
            master.domain(domain_idx).process().unwrap();
            master.domain(domain_idx).queue().unwrap();
            master.send().unwrap();
            let m_state = master.state().unwrap();
            let _d_state = master.domain(domain_idx).state();
            // log::debug!("Master state: {:?}", m_state);
            // log::debug!("Domain state: {:?}", d_state);
            if m_state.link_up && AlState::try_from(m_state.al_states).unwrap() == AlState::Op {
                // do something with the data
                let raw_data = master.domain_data(domain_idx).unwrap();
                cyclic_work(raw_data);
            }
            sleep_until(next_cycle).await;
            next_cycle += cycle_time;
        }
    });

    // join
    handle.await.unwrap();

    Ok(())
}

fn cyclic_work(domain_data: &mut [u8]) {
    let ptr = domain_data.as_ptr();
    // read data from the domain

    let data = MyDomainData::try_from(domain_data).unwrap();
    let status_word = data.tx.status_word;
    let actual_position = data.tx.position_actual_value;
    log::debug!("tx: status:{}, actual_pos {}", status_word, actual_position);

    // check data's pointer and domain_data's pointer are the same

    log::debug!("data: {:p}, domain_data: {:p}", &data as *const _, ptr);
}

fn init_master() -> Result<(Master, DomainIdx), io::Error> {
    log::debug!("open master0");
    let mut master = Master::open(0_u32, MasterAccess::ReadWrite)?;
    log::debug!("Reserve master");
    master.reserve()?;

    log::debug!("Create domain");
    let domain_idx = master.create_domain()?;
    log::debug!("Create domain done {:?}", domain_idx);
    let mut offsets: HashMap<SlavePos, HashMap<PdoEntryIdx, (u8, Offset)>> = HashMap::new();

    let slave_pos = SlavePos::from(0_u16);
    log::debug!("Request PreOp state for {:?}", slave_pos);
    master.request_state(slave_pos, AlState::PreOp)?;
    let slave_info = master.get_slave_info(slave_pos)?;
    log::info!("Found device {:?}", slave_info);

    let slave_addr = SlaveAddr::ByPos(0_u16);
    let slave_id = SlaveId {
        vendor_id: 0x066F_u32,
        product_code: 0x6038_0006_u32,
    };

    let mut config = master.configure_slave(slave_addr, slave_id)?;
    let mut entry_offsets: HashMap<PdoEntryIdx, (u8, Offset)> = HashMap::new();

    let output = SmCfg::output(2.into());
    let input = SmCfg::input(3.into());

    let s1_rx_pdos = vec![PdoCfg {
        idx: PdoIdx::from(0x1600),
        entries: vec![
            // mode of operation
            PdoEntryInfo {
                entry_idx: PdoEntryIdx {
                    idx: Idx::from(0x6060),
                    sub_idx: SubIdx::from(0),
                },
                bit_len: 8,
                name: "mode of operation".to_string(),
                pos: PdoEntryPos::from(0),
            },
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
        ],
    }];

    let s1_tx_pdos = vec![PdoCfg {
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

    config.config_sm_pdos(output, &s1_rx_pdos)?;
    config.config_sm_pdos(input, &s1_tx_pdos)?;

    for pdo in &s1_rx_pdos {
        // Positions of RX PDO
        log::debug!("Positions of RX PDO 0x{:X}:", u16::from(pdo.idx));
        for entry in &pdo.entries {
            let offset = config.register_pdo_entry(entry.entry_idx, domain_idx)?;
            entry_offsets.insert(entry.entry_idx, (entry.bit_len, offset));
        }
    }
    for pdo in &s1_tx_pdos {
        // Positions of TX PDO
        log::debug!("Positions of TX PDO 0x{:X}:", u16::from(pdo.idx));
        for entry in &pdo.entries {
            let offset = config.register_pdo_entry(entry.entry_idx, domain_idx)?;
            entry_offsets.insert(entry.entry_idx, (entry.bit_len, offset));
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
