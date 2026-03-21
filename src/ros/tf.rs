use crate::ros::types::{Time, TransformStamped};

#[derive(Debug, Clone)]
#[allow(dead_code)]
pub struct TfError(pub String);

pub trait TfClient: Send + Sync {
    fn lookup_transform(
        &self,
        target_frame: &str,
        source_frame: &str,
        time: Time,
    ) -> Result<TransformStamped, TfError>;
}
