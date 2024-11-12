mod constants;
mod iekf;
mod state;
mod utils;

use iekf::IEKF;
use pyo3::prelude::*;
use state::State;

/// Robosub IEKF
#[pymodule]
fn umriekf(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<State>()?;
    m.add_class::<IEKF>()?;
    Ok(())
}
