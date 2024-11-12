use nalgebra::SimdComplexField;
use rand::prelude::Distribution;
use statrs::distribution::Normal;

use super::{SMatrix, SVector};

fn cholesky<const N: usize>(mut matrix: SMatrix<N, N>) -> SMatrix<N, N> {
    let n = matrix.0.nrows();

    for j in 0..n {
        for k in 0..j {
            let factor = -matrix.0.get((j, k)).expect("(j, k) is in bounds");

            let (mut col_j, col_k) = matrix.0.columns_range_pair_mut(j, k);
            let mut col_j = col_j.rows_range_mut(j..);
            let col_k = col_k.rows_range(j..);
            col_j.axpy(factor.simd_conjugate(), &col_k, 1.);
        }

        let diag = matrix.0.get((j, j)).expect("(j, j) is in bounds");
        let denom = diag.simd_sqrt();

        *matrix.0.get_mut((j, j)).expect("(j, j) is in bounds") = denom;

        let mut col = matrix.0.view_range_mut(j + 1.., j);
        col /= denom;
    }

    matrix.0.fill_upper_triangle(0., 1);
    matrix
}

pub fn multivariate_normal<const N: usize>(mean: SVector<N>, cov: SMatrix<N, N>) -> SVector<N> {
    let choelsky = cholesky(cov);
    choelsky * normal_matrix(0., 1.) + mean
}

pub fn normal_matrix<const R: usize, const C: usize>(mean: f64, variance: f64) -> SMatrix<R, C> {
    let dist = Normal::new(mean, variance.sqrt()).expect("Stdev is finite");
    SMatrix(nalgebra::SMatrix::<f64, R, C>::from_iterator(
        dist.sample_iter(rand::thread_rng()),
    ))
}
