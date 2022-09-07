#[repr(C, packed)]
pub struct MyRxPdo {
    mode_of_operation: u8,
    control_word: u16,
    target_position: i32,
    profile_velocity: i32,
}
#[repr(C, packed)]
pub struct MyTxPdo {
    position_actual_value: i32,
    status_word: u16,
}
#[repr(C, packed)]
pub struct DomainData {
    pub rx: MyRxPdo,
    pub tx: MyTxPdo,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {
        let mut v: [u8; 17] = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 129, 212, 253, 254, 80, 2];
        let v_ptr = v.as_mut_ptr();

        use std::mem::transmute;
        let s = unsafe { transmute::<*mut u8, &mut DomainData>(v_ptr) };
        let status_word = s.tx.status_word;
        let position_actual_value = s.tx.position_actual_value;

        // modify the value
        assert_eq!(status_word, 592);
        assert_eq!(position_actual_value, -16919423);
    }
}
