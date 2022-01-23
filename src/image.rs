use crate::config::ListenerConfig;
use byteorder::{ByteOrder, LittleEndian};
use image::{DynamicImage, ImageBuffer, RgbaImage};
use rosrust;
use rosrust_msg;
use std::sync::{Arc, RwLock};

// remap a value from range min_val - max_val to 0 - 255
fn remap_u8(val: f64, min_val: f64, max_val: f64) -> u8 {
    ((val - min_val) * (u8::MAX as f64 / (max_val - min_val))) as u8
}

fn read_img_msg(img_msg: rosrust_msg::sensor_msgs::Image) -> DynamicImage {
    match img_msg.encoding.as_ref() {
        "8UC1" | "mono8" => DynamicImage::ImageLuma8(
            ImageBuffer::from_raw(img_msg.width, img_msg.height, img_msg.data).unwrap(),
        ),
        "8UC3" | "rgb8" => DynamicImage::ImageRgb8(
            ImageBuffer::from_raw(img_msg.width, img_msg.height, img_msg.data).unwrap(),
        ),
        "16UC1" | "mono16" => DynamicImage::ImageLuma8(
            ImageBuffer::from_raw(img_msg.width, img_msg.height, read_u16(&img_msg.data)).unwrap(),
        ),
        "32FC1" => DynamicImage::ImageLuma8(
            ImageBuffer::from_raw(img_msg.width, img_msg.height, read_f32(&img_msg.data)).unwrap(),
        ),
        _ => panic!("Image encoding {:?} not supported", img_msg.encoding),
    }
}

fn read_f32(vec: &Vec<u8>) -> Vec<u8> {
    let mut vals: Vec<f32> = Vec::with_capacity(vec.len() / 4);
    let mut max_val = f32::MIN;
    let mut min_val = f32::MAX;
    for elem in vec.chunks(4) {
        let val = LittleEndian::read_f32(&elem);
        vals.push(val);
        if val > max_val {
            max_val = val;
        }
        if val < min_val {
            min_val = val;
        }
    }
    let mut bytes: Vec<u8> = Vec::with_capacity(vals.len());
    for val in vals {
        bytes.push(remap_u8(val as f64, min_val as f64, max_val as f64));
    }
    bytes
}

fn read_u16(vec: &Vec<u8>) -> Vec<u8> {
    let mut vals: Vec<u16> = Vec::with_capacity(vec.len() / 2);
    let mut max_val = u16::MIN;
    let mut min_val = u16::MAX;
    for elem in vec.chunks(2) {
        let val = LittleEndian::read_u16(&elem);
        vals.push(val);
        if val > max_val {
            max_val = val;
        }
        if val < min_val {
            min_val = val;
        }
    }
    let mut bytes: Vec<u8> = Vec::with_capacity(vals.len());
    for val in vals {
        bytes.push(remap_u8(val as f64, min_val as f64, max_val as f64));
    }
    bytes
}

pub struct ImageListener {
    pub config: ListenerConfig,
    pub img: Arc<RwLock<RgbaImage>>,
    _subscriber: Option<rosrust::Subscriber>,
}

impl ImageListener {
    pub fn new(config: ListenerConfig) -> ImageListener {
        let img = Arc::new(RwLock::new(RgbaImage::new(0, 0)));

        ImageListener {
            config,
            img,
            _subscriber: None,
        }
    }

    pub fn setup_sub(&mut self) {
        let cb_img = self.img.clone();
        let sub = rosrust::subscribe(
            &self.config.topic,
            1,
            move |img_msg: rosrust_msg::sensor_msgs::Image| {
                let img = read_img_msg(img_msg);
                let mut cb_img = cb_img.write().unwrap();
                *cb_img = img.to_rgba8();
            },
        )
        .unwrap();
        self._subscriber = Some(sub)
    }

    pub fn is_active(&self) -> bool {
        self._subscriber.is_some()
    }

    pub fn activate(&mut self) {
        self.setup_sub();
    }

    pub fn deactivate(&mut self) {
        self._subscriber = None;
    }
}
