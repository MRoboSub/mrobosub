#![allow(dead_code)]
use std::ops::{Add, Mul, Neg, Sub};

use nalgebra::{
    ArrayStorage, Const, DimMin, Dyn, Matrix, Storage, ViewStorage, ViewStorageMut, U1,
};
use numpy::{ndarray::Dimension, PyReadonlyArray};

use super::{assert_mat_len, InvalidMatrixLengthError};

pub(super) type ViewMat<'a, const R_LEN: usize, const C_LEN: usize, const R: usize> = Matrix<
    f64,
    Const<R_LEN>,
    Const<C_LEN>,
    ViewStorage<'a, f64, Const<R_LEN>, Const<C_LEN>, U1, Const<R>>,
>;

pub(super) type ViewMatMut<'a, const R_LEN: usize, const C_LEN: usize, const R: usize> = Matrix<
    f64,
    Const<R_LEN>,
    Const<C_LEN>,
    ViewStorageMut<'a, f64, Const<R_LEN>, Const<C_LEN>, U1, Const<R>>,
>;

#[derive(Debug, Clone, Copy)]
pub struct SMatrix<const R: usize, const C: usize>(pub nalgebra::SMatrix<f64, R, C>);

impl<const R: usize, const C: usize> Default for SMatrix<R, C> {
    fn default() -> Self {
        Self::from_array_transposed([[0.; R]; C])
    }
}

pub type SQMatrix<const N: usize> = SMatrix<N, N>;
pub type SVector<const N: usize> = SMatrix<N, 1>;

impl<const R: usize, const C: usize> SMatrix<R, C> {
    pub fn zeros() -> Self {
        Self(nalgebra::SMatrix::zeros())
    }

    pub const fn from_array_transposed(value: [[f64; R]; C]) -> Self {
        Self(Matrix::from_array_storage(ArrayStorage(value)))
    }

    pub fn from_array(value: [[f64; C]; R]) -> Self {
        SMatrix::from_array_transposed(value).transpose()
    }

    pub const fn from_element(value: f64) -> Self {
        Self::from_array_transposed([[value; R]; C])
    }

    pub fn as_view(&self) -> ViewMat<'_, R, C, R> {
        self.0.as_view()
    }

    pub fn as_view_mut(&mut self) -> ViewMatMut<'_, R, C, R> {
        self.0.as_view_mut()
    }

    pub fn view<const R_LEN: usize, const C_LEN: usize>(
        &self,
        r_start: usize,
        c_start: usize,
    ) -> SMatrix<R_LEN, C_LEN> {
        self.0.fixed_view::<R_LEN, C_LEN>(r_start, c_start).owned()
    }

    pub fn view_mut<const R_LEN: usize, const C_LEN: usize>(
        &mut self,
        r_start: usize,
        c_start: usize,
    ) -> ViewMatMut<'_, R_LEN, C_LEN, R> {
        self.0.fixed_view_mut(r_start, c_start)
    }

    pub fn rows<const R_LEN: usize>(&self, start: usize) -> SMatrix<R_LEN, C> {
        self.0.fixed_rows::<R_LEN>(start).owned()
    }

    pub fn row(&self, row: usize) -> SMatrix<1, C> {
        self.rows(row)
    }

    pub fn rows_mut<const R_LEN: usize>(&mut self, start: usize) -> ViewMatMut<'_, R_LEN, C, R> {
        self.0.fixed_rows_mut(start)
    }

    pub fn row_mut(&mut self, row: usize) -> ViewMatMut<'_, 1, C, R> {
        self.rows_mut(row)
    }

    pub fn cols<const C_LEN: usize>(&self, start: usize) -> SMatrix<R, C_LEN> {
        self.0.fixed_columns::<C_LEN>(start).owned()
    }

    pub fn col(&self, col: usize) -> SMatrix<R, 1> {
        self.cols(col)
    }

    pub fn cols_mut<const C_LEN: usize>(&mut self, start: usize) -> ViewMatMut<'_, R, C_LEN, R> {
        self.0.fixed_columns_mut(start)
    }

    pub fn col_mut(&mut self, col: usize) -> ViewMatMut<'_, R, 1, R> {
        self.cols_mut(col)
    }

    pub fn transpose(&self) -> SMatrix<C, R> {
        let mut res = SMatrix::zeros();
        self.0.transpose_to(&mut res.0);
        res
    }
}

impl<const N: usize> SQMatrix<N> {
    pub fn identity() -> Self {
        Self(nalgebra::SMatrix::identity())
    }

    pub fn try_inverse(&self) -> Option<Self> {
        self.0.try_inverse().map(Self)
    }

    pub fn inverse(&self) -> Self {
        self.try_inverse().expect("matrix should be invertible")
    }

    pub fn exp(&self) -> Self
    where
        Const<N>: DimMin<Const<N>, Output = Const<N>>,
    {
        Self(self.0.exp())
    }
}

impl<const N: usize> SVector<N> {
    pub const fn from_array_1d(arr: [f64; N]) -> Self {
        Self::from_array_transposed([arr])
    }
}

impl<const R: usize, const C: usize> From<[[f64; C]; R]> for SMatrix<R, C> {
    fn from(value: [[f64; C]; R]) -> Self {
        Self::from_array(value)
    }
}

impl<const R: usize, const C: usize, const C_STRIDE: usize> From<ViewMat<'_, R, C, C_STRIDE>>
    for SMatrix<R, C>
{
    fn from(value: ViewMat<'_, R, C, C_STRIDE>) -> Self {
        Self(value.into())
    }
}

impl<const R: usize, const C: usize, const C_STRIDE: usize> From<ViewMatMut<'_, R, C, C_STRIDE>>
    for SMatrix<R, C>
{
    fn from(value: ViewMatMut<'_, R, C, C_STRIDE>) -> Self {
        Self(value.into())
    }
}

impl<const R: usize, const C: usize, D: Dimension> TryFrom<&PyReadonlyArray<'_, f64, D>>
    for SMatrix<R, C>
{
    type Error = InvalidMatrixLengthError;

    fn try_from(value: &PyReadonlyArray<'_, f64, D>) -> Result<Self, Self::Error> {
        assert_mat_len(value, R * C)?;
        Ok(Self(
            value
                .try_as_matrix::<Const<R>, Const<C>, Dyn, Dyn>()
                .expect("Matrix is expected length")
                .into(),
        ))
    }
}

impl<const N: usize, const I: usize, const M: usize, S: Storage<f64, Const<I>, Const<M>>>
    Mul<Matrix<f64, Const<I>, Const<M>, S>> for SMatrix<N, I>
{
    type Output = SMatrix<N, M>;

    fn mul(self, rhs: Matrix<f64, Const<I>, Const<M>, S>) -> Self::Output {
        let mut res = SMatrix::zeros();
        self.0.mul_to(&rhs, &mut res.0);
        res
    }
}

impl<const N: usize, const I: usize, const M: usize> Mul<SMatrix<I, M>> for SMatrix<N, I> {
    type Output = SMatrix<N, M>;

    fn mul(self, rhs: SMatrix<I, M>) -> Self::Output {
        self * rhs.0
    }
}

impl<const R: usize, const C: usize> Mul<f64> for SMatrix<R, C> {
    type Output = Self;

    fn mul(mut self, rhs: f64) -> Self::Output {
        self.0.component_mul_assign(&SMatrix::from_element(rhs).0);
        self
    }
}

impl<const R: usize, const C: usize, S: Storage<f64, Const<R>, Const<C>>>
    Add<Matrix<f64, Const<R>, Const<C>, S>> for SMatrix<R, C>
{
    type Output = Self;

    fn add(self, rhs: Matrix<f64, Const<R>, Const<C>, S>) -> Self::Output {
        let mut res = SMatrix::zeros();
        self.0.add_to(&rhs, &mut res.0);
        res
    }
}

impl<const R: usize, const C: usize> Add for SMatrix<R, C> {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        self + rhs.0
    }
}

impl<const R: usize, const C: usize, S: Storage<f64, Const<R>, Const<C>>>
    Sub<Matrix<f64, Const<R>, Const<C>, S>> for SMatrix<R, C>
{
    type Output = Self;

    fn sub(self, rhs: Matrix<f64, Const<R>, Const<C>, S>) -> Self::Output {
        let mut res = SMatrix::zeros();
        self.0.sub_to(&rhs, &mut res.0);
        res
    }
}

impl<const R: usize, const C: usize> Sub for SMatrix<R, C> {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        self - rhs.0
    }
}

impl<const R: usize, const C: usize> Neg for SMatrix<R, C> {
    type Output = Self;

    fn neg(self) -> Self::Output {
        self * -1.
    }
}

pub trait SetFrom<Value> {
    fn set_from(&mut self, value: Value);
}

impl<const R: usize, const C: usize, const C_STRIDE: usize> SetFrom<SMatrix<R, C>>
    for ViewMatMut<'_, R, C, C_STRIDE>
{
    fn set_from(&mut self, value: SMatrix<R, C>) {
        self.copy_from(&value.0);
    }
}

pub trait Owned {
    type Output;

    fn owned(&self) -> Self::Output;
}

impl<const R: usize, const C: usize, S: Storage<f64, Const<R>, Const<C>>> Owned
    for Matrix<f64, Const<R>, Const<C>, S>
{
    type Output = SMatrix<R, C>;

    fn owned(&self) -> Self::Output {
        SMatrix(self.as_view().into())
    }
}
