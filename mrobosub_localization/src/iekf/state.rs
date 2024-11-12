use nalgebra::{Matrix, ViewStorageMut, U1, U3, U5};
use numpy::{PyArray2, PyReadonlyArrayDyn, ToPyArray};
use pyo3::{pyclass, pymethods, Bound, PyResult, Python};

use crate::utils::{static_matrix::SQMatrix, SMatrix, SVector, SetFrom};

pub type ViewMut3<'a> = Matrix<f64, U3, U1, ViewStorageMut<'a, f64, U3, U1, U1, U5>>;
pub type ViewMut3x3<'a> = Matrix<f64, U3, U3, ViewStorageMut<'a, f64, U3, U3, U1, U5>>;

#[pyclass(module = "umriekf")]
#[derive(Debug, Clone)]
pub struct State {
    pub matrix: SQMatrix<5>,
}

impl Default for State {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl State {
    pub const IDENTITY: Self = Self::new(SQMatrix::from_array_transposed([
        [1., 0., 0., 0., 0.],
        [0., 1., 0., 0., 0.],
        [0., 0., 1., 0., 0.],
        [0., 0., 0., 1., 0.],
        [0., 0., 0., 0., 1.],
    ]));

    pub const fn new(matrix: SQMatrix<5>) -> Self {
        Self { matrix }
    }

    pub fn from_components(
        rotation: SQMatrix<3>,
        velocity: SVector<3>,
        position: SVector<3>,
    ) -> Self {
        let mut state = Self::IDENTITY;
        state.rotation_mut().set_from(rotation);
        state.velocity_mut().set_from(velocity);
        state.position_mut().set_from(position);
        state
    }

    pub fn rotation(&self) -> SQMatrix<3> {
        self.matrix.view::<3, 3>(0, 0)
    }

    pub fn rotation_mut(&mut self) -> ViewMut3x3 {
        self.matrix.view_mut(0, 0)
    }

    pub fn velocity(&self) -> SVector<3> {
        self.matrix.view::<3, 1>(0, 3)
    }

    pub fn velocity_mut(&mut self) -> ViewMut3 {
        self.matrix.view_mut(0, 3)
    }

    pub fn position(&self) -> SVector<3> {
        self.matrix.view::<3, 1>(0, 4)
    }

    pub fn position_mut(&mut self) -> ViewMut3 {
        self.matrix.view_mut(0, 4)
    }

    pub fn inverse(&self) -> Self {
        Self::new(self.matrix.try_inverse().expect("state is invertible"))
    }
}

#[pymethods]
impl State {
    #[staticmethod]
    fn identity() -> Self {
        Self::IDENTITY
    }

    #[getter]
    fn get_matrix<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray2<f64>> {
        self.matrix.0.to_pyarray_bound(py)
    }

    #[setter]
    fn set_matrix(&mut self, matrix: PyReadonlyArrayDyn<'_, f64>) -> PyResult<()> {
        self.matrix
            .as_view_mut()
            .set_from(SMatrix::try_from(&matrix).map_err(anyhow::Error::from)?);
        Ok(())
    }

    #[getter]
    fn get_rotation<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray2<f64>> {
        self.rotation().0.to_pyarray_bound(py)
    }

    #[setter]
    fn set_rotation(&mut self, rotation: PyReadonlyArrayDyn<'_, f64>) -> PyResult<()> {
        self.rotation_mut()
            .set_from(SMatrix::try_from(&rotation).map_err(anyhow::Error::from)?);
        Ok(())
    }

    #[getter]
    fn get_velocity<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray2<f64>> {
        self.velocity().0.to_pyarray_bound(py)
    }

    #[setter]
    fn set_velocity(&mut self, velocity: PyReadonlyArrayDyn<'_, f64>) -> PyResult<()> {
        self.velocity_mut()
            .set_from(SMatrix::try_from(&velocity).map_err(anyhow::Error::from)?);
        Ok(())
    }

    #[getter]
    fn get_position<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray2<f64>> {
        self.position().0.to_pyarray_bound(py)
    }

    #[setter]
    fn set_position(&mut self, position: PyReadonlyArrayDyn<'_, f64>) -> PyResult<()> {
        self.position_mut()
            .set_from(SMatrix::try_from(&position).map_err(anyhow::Error::from)?);
        Ok(())
    }

    #[new]
    fn py_new(matrix: PyReadonlyArrayDyn<'_, f64>) -> PyResult<Self> {
        Ok(Self::new(
            SMatrix::try_from(&matrix).map_err(anyhow::Error::from)?,
        ))
    }

    #[staticmethod]
    #[pyo3(name = "from_components")]
    fn py_from_components(
        rotation: PyReadonlyArrayDyn<'_, f64>,
        velocity: PyReadonlyArrayDyn<'_, f64>,
        position: PyReadonlyArrayDyn<'_, f64>,
    ) -> PyResult<Self> {
        let mut state = Self::IDENTITY;
        state.set_rotation(rotation)?;
        state.set_velocity(velocity)?;
        state.set_position(position)?;
        Ok(state)
    }

    #[pyo3(name = "inverse")]
    fn py_inverse(&self) -> Self {
        self.inverse()
    }
}
