#[repr(C, packed)]
#[derive(Clone, Copy)]
pub(crate) struct ServoTxPdo {
    pub status_word: u16,
    pub position_actual_value: i32,
}

#[repr(C, packed)]
#[derive(Default, Clone, Copy)]
pub(crate) struct ServoRxPdo {
    pub control_word: u16,
    pub target_position: i32,
    pub profile_velocity: u32,
    pub profile_acceleration: u32,
    pub profile_deceleration: u32,
    pub mode_of_operation: u8,
}

#[repr(C, packed)]
pub(crate) struct ServoPdo {
    pub rx: ServoRxPdo,
    pub tx: ServoTxPdo,
}
