use std::{env, fs::File, io::Read};

use ethercat_esi::EtherCatInfo;

pub fn main() -> Result<(), std::io::Error> {
    env_logger::init();
    let args: Vec<_> = env::args().collect();
    let esi_dir = match args.len() {
        2 => &args[1],
        _ => "/home/pan/ethercat_ws/ecat-rs/assets", //     return Ok(());
    };

    {
        let path: &str = &esi_dir;
        let paths = std::fs::read_dir(path).unwrap();

        for path in paths {
            // test if the file is a xml file
            let path = path.unwrap().path();
            let file_name = path.file_name().unwrap().to_str().unwrap();
            if !file_name.ends_with(".xml") {
                continue;
            }

            // read the xml file
            let mut esi_file = File::open(path)?;
            let mut esi_xml_string = String::new();
            esi_file.read_to_string(&mut esi_xml_string)?;
            let esi = EtherCatInfo::from_xml_str(&esi_xml_string)?;
            for device in esi.description.devices.iter().enumerate() {
                let (idx, device) = device;
                println!("{}: {}", idx, device.name);
            }
        }
    };

    Ok(())
}

// fn read_all_xml_at_dir(path: &str) -> Result<(), io::Error> {
//     let paths = std::fs::read_dir(path).unwrap();

//     for path in paths {
//         // test if the file is a xml file
//         let path = path.unwrap().path();
//         let file_name = path.file_name().unwrap().to_str().unwrap();
//         if !file_name.ends_with(".xml") {
//             continue;
//         }

//         // read the xml file
//         let mut esi_file = File::open(path)?;
//         let mut esi_xml_string = String::new();
//         esi_file.read_to_string(&mut esi_xml_string)?;
//         let esi = EtherCatInfo::from_xml_str(&esi_xml_string)?;
//         for device in esi.description.devices.iter().enumerate() {
//             let (idx, device) = device;
//             println!("{}: {}", idx, device.name);
//         }
//     }

//     Ok(())
// }
