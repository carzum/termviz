use crate::config::ImageListenerConfig;
use byteorder::{ByteOrder, LittleEndian};
use image::{imageops, DynamicImage, ImageBuffer, RgbaImage};
use rosrust;
use rosrust_msg;
use std::sync::{Arc, RwLock};
use tui::buffer::Buffer;
use tui::layout::Rect;
use tui::widgets::Widget;
use viuer::{print, Config};

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
    pub config: ImageListenerConfig,
    pub img: Arc<RwLock<RgbaImage>>,
    _subscriber: Option<rosrust::Subscriber>,
    _rotation: Arc<RwLock<i64>>,
}

impl ImageListener {
    pub fn new(config: ImageListenerConfig) -> ImageListener {
        let img = Arc::new(RwLock::new(RgbaImage::new(0, 0)));
        let default_rotation = config.rotation.clone();
        ImageListener {
            config,
            img,
            _subscriber: None,
            _rotation: Arc::new(RwLock::new(default_rotation)),
        }
    }

    pub fn setup_sub(&mut self) {
        let cb_img = self.img.clone();
        let cb_rotation = self._rotation.clone();
        let sub = rosrust::subscribe(
            &self.config.topic,
            1,
            move |img_msg: rosrust_msg::sensor_msgs::Image| {
                let mut img = read_img_msg(img_msg).to_rgba8();
                // img = imageops::resize(&img, 320, 240, imageops::FilterType::Nearest);
                let rot = cb_rotation.read().unwrap();
                match *rot {
                    90 => img = imageops::rotate90(&img),
                    180 => img = imageops::rotate180(&img),
                    270 => img = imageops::rotate270(&img),
                    _ => (),
                }
                let mut cb_img = cb_img.write().unwrap();
                *cb_img = img;
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

    pub fn rotate(&mut self, angle: i64) {
        let mut rot = *self._rotation.read().unwrap();
        rot += angle;
        if rot < 0 {
            rot = 270;
        }
        if rot > 270 {
            rot = 0;
        }
        let mut rotation = self._rotation.write().unwrap();
        *rotation = rot;
    }
}

impl Widget for &ImageListener {
    fn render(self, area: Rect, buf: &mut Buffer) {
        let conf = Config {
            x: area.x,
            y: area.y as i16,
            width: Some((area.width) as u32),
            height: Some((area.height) as u32),
            ..Default::default()
        };
        let img = image::DynamicImage::ImageRgba8(self.img.read().unwrap().clone());
        print(&img, &conf).expect("image printing failed'");
    }
}
