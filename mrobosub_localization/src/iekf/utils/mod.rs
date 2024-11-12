pub mod random;
pub mod static_matrix;

use std::{
    error::Error,
    fmt::{Display, Formatter},
};

use numpy::{ndarray::Dimension, PyReadonlyArray};

#[derive(Debug)]
pub struct InvalidMatrixLengthError {
    expected: usize,
    found: usize,
}

impl InvalidMatrixLengthError {
    pub fn new(expected: usize, found: usize) -> Self {
        Self { expected, found }
    }
}

impl Display for InvalidMatrixLengthError {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "Invalid matrix length: expected {}, found {}",
            self.expected, self.found
        )
    }
}

impl Error for InvalidMatrixLengthError {}

pub fn assert_mat_len<D: Dimension>(
    mat: &PyReadonlyArray<'_, f64, D>,
    expected_len: usize,
) -> Result<(), InvalidMatrixLengthError> {
    let len = mat.as_array().len();
    if len == expected_len {
        Ok(())
    } else {
        Err(InvalidMatrixLengthError::new(expected_len, len))
    }
}

pub use static_matrix::{SMatrix, SQMatrix, SVector, SetFrom};
