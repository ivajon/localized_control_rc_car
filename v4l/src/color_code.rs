pub trait ColorCode: Clone {
    type Marker: Clone;
    fn into_grayscale(current: Vec<u8>) -> Vec<u8>;
    fn from_rgb(current: &Vec<u8>) -> Vec<u8>;
}

#[derive(Clone)]
pub struct RGB {}
impl ColorCode for RGB {
    type Marker = [u8; 3];

    fn into_grayscale(current: Vec<u8>) -> Vec<u8> {
        let mut ret = Vec::new();
        let mut cnt = 1;
        let mut sum = 0;
        for el in current {
            sum += el;
            if cnt % 3 == 0 {
                let avg = sum / 3;
                ret.push(avg);
                sum = 0;
            }

            cnt += 1;
        }
        ret
    }

    fn from_rgb(current: &Vec<u8>) -> Vec<u8> {
        current.clone()
    }
}

#[derive(Clone)]
pub struct Red {}
impl ColorCode for Red {
    type Marker = u8;

    fn into_grayscale(current: Vec<u8>) -> Vec<u8> {
        // We do not have any other information than the green channel, thus we simply
        // treat the buffer as a grayscale image.
        current
    }

    fn from_rgb(current: &Vec<u8>) -> Vec<u8> {
        let mut ret = Vec::with_capacity(current.len() / 3);
        for (idx, el) in current.iter().enumerate() {
            if idx % 3 == 0 {
                ret.push(*el)
            }
        }
        ret
    }
}

#[derive(Clone)]
pub struct Green {}
impl ColorCode for Green {
    type Marker = u8;

    fn into_grayscale(current: Vec<u8>) -> Vec<u8> {
        // We do not have any other information than the green channel, thus we simply
        // treat the buffer as a grayscale image.
        current
    }

    fn from_rgb(current: &Vec<u8>) -> Vec<u8> {
        let mut ret = Vec::with_capacity(current.len() / 3);
        for (idx, el) in current.iter().enumerate() {
            if ((idx) % 3) == 1 {
                ret.push(*el)
            }
        }
        ret
    }
}

#[derive(Clone)]
pub struct GrayScale {}
impl ColorCode for GrayScale {
    type Marker = u8;

    fn into_grayscale(current: Vec<u8>) -> Vec<u8> {
        current
    }

    fn from_rgb(current: &Vec<u8>) -> Vec<u8> {
        let mut ret = Vec::new();
        let mut cnt = 1;
        let mut sum: u32 = 0;
        for el in current {
            sum += *el as u32;
            if cnt % 3 == 0 {
                let avg = sum / 3;
                ret.push(avg as u8);
                sum = 0;
            }
            cnt += 1;
        }
        ret
    }
}
