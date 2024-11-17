use std::{
    error::Error,
    fmt::{Display, Formatter},
};

use anyhow::{Context, Result};
use pyo3::{prelude::*, types::PyDict};

use crate::utils::{SMatrix, SVector};

#[derive(Debug, Default)]
pub struct Constants {
    pub imu_hz: usize,
    pub gravity: SVector<3>,

    pub std_acc_noise: f64,
    pub std_acc_bias_noise: f64,
    pub std_gyro_noise: f64,
    pub std_gyro_bias_noise: f64,

    pub cov_acc_noise: SMatrix<3, 3>,
    pub cov_acc_bias_noise: SMatrix<3, 3>,
    pub cov_gyro_noise: SMatrix<3, 3>,
    pub cov_gyro_bias_noise: SMatrix<3, 3>,

    pub state_covariance: SMatrix<15, 15>,

    pub std_dvl_noise: f64,
    pub cov_dvl_noise: SMatrix<3, 3>,
    pub dvl_translation: SVector<3>,
    pub dvl_rotation: SMatrix<3, 3>,

    pub std_depth_noise: f64,
    pub cov_depth_noise: SMatrix<3, 3>,
    pub depth_translation: SVector<3>,

    pub measurement_covariance: SMatrix<4, 4>,
}

impl Constants {
    pub fn update_from_py(&mut self, dict: &Bound<PyDict>) -> Result<()> {
        macro_rules! get {
            ($fn:ident, $key:ident) => {
                self.$key = $fn(dict, stringify!($key)).with_context(|| stringify!($key))?;
            };
        }

        get!(get_val, imu_hz);
        get!(get_mat, gravity);

        get!(get_val, std_acc_noise);
        get!(get_val, std_acc_bias_noise);
        get!(get_val, std_gyro_noise);
        get!(get_val, std_gyro_bias_noise);

        get!(get_mat, cov_acc_noise);
        get!(get_mat, cov_acc_bias_noise);
        get!(get_mat, cov_gyro_noise);
        get!(get_mat, cov_gyro_bias_noise);

        get!(get_mat, state_covariance);

        get!(get_val, std_dvl_noise);
        get!(get_mat, cov_dvl_noise);
        get!(get_mat, dvl_translation);
        get!(get_mat, dvl_rotation);

        get!(get_val, std_depth_noise);
        get!(get_mat, cov_depth_noise);
        get!(get_mat, depth_translation);

        get!(get_mat, measurement_covariance);
        Ok(())
    }
}

fn get_val<'py, T: FromPyObject<'py>>(dict: &Bound<'py, PyDict>, key: &str) -> Result<T> {
    dict.get_item(key)?
        .ok_or(MissingEntryError::from(key))?
        .extract()
        .map_err(Into::into)
}

fn get_mat<const R: usize, const C: usize>(
    dict: &Bound<PyDict>,
    key: &str,
) -> Result<SMatrix<R, C>> {
    let py_matrix: numpy::PyReadonlyArrayDyn<'_, f64> = get_val(dict, key)?;
    SMatrix::<R, C>::try_from(&py_matrix).map_err(Into::into)
}

#[derive(Debug)]
pub struct MissingEntryError {
    key: String,
}

impl From<&str> for MissingEntryError {
    fn from(value: &str) -> Self {
        Self {
            key: value.to_string(),
        }
    }
}

impl Display for MissingEntryError {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        write!(f, "Unable to find key {} in dictionary", self.key)
    }
}

impl Error for MissingEntryError {}
