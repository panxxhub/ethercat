// pub fn add(left: usize, right: usize) -> usize {
//     left + right
// }

#[cfg(test)]
mod tests {

    #[test]
    fn it_works() {
        let v: [u8; 4] = [1, 2, 3, 4];
        let v_ptr = v.as_ptr();

        use std::mem::transmute;
        let s: *const u32 = unsafe { transmute(v_ptr) };

        assert_eq!(unsafe { *s }, 0x04030201);
        // pointer eq
        assert_eq!(s, v_ptr as *const u32);
    }
}
