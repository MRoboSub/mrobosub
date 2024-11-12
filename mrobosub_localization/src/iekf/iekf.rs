use std::time::SystemTime;

use numpy::{ndarray::Array1, IntoPyArray, PyArray1, PyReadonlyArray1, PyReadonlyArray2};
use pyo3::{pyclass, pymethods, types::PyDict, Bound, PyResult, Python};

use crate::{
    constants::Constants,
    state::State,
    utils::{random::multivariate_normal, SMatrix, SQMatrix, SVector, SetFrom},
};

type AdjointType = SQMatrix<9>;

fn make_skew_sym(vec: SVector<3>) -> SQMatrix<3> {
    let vec = vec.0;
    SMatrix::from_array([
        [0., -vec[2], vec[1]],
        [vec[2], 0., -vec[0]],
        [-vec[1], vec[0], 0.],
    ])
}

fn calc_adjoint(state: &State) -> AdjointType {
    let rotation = state.rotation();
    let vel_cross = make_skew_sym(state.velocity());
    let pos_cross = make_skew_sym(state.position());
    let mut adjoint = AdjointType::zeros();
    adjoint.view_mut::<3, 3>(0, 0).set_from(rotation);
    adjoint
        .view_mut::<3, 3>(3, 0)
        .set_from(vel_cross * rotation);
    adjoint.view_mut::<3, 3>(3, 3).set_from(rotation);
    adjoint
        .view_mut::<3, 3>(6, 0)
        .set_from(pos_cross * rotation);
    adjoint.view_mut::<3, 3>(6, 6).set_from(rotation);
    adjoint
}

fn carat(xi: SVector<9>) -> SQMatrix<5> {
    let w_cross = make_skew_sym(xi.view(0, 0));
    let v = xi.view(3, 0);
    let p = xi.view(6, 0);
    let mut carat = SMatrix::zeros();
    carat.view_mut::<3, 3>(0, 0).set_from(w_cross);
    carat.view_mut::<3, 1>(0, 3).set_from(v);
    carat.view_mut::<3, 1>(0, 4).set_from(p);
    carat
}

#[allow(clippy::identity_op)]
fn calc_right_invariant_error(state: &State, constants: &Constants) -> SQMatrix<15> {
    let gskew = make_skew_sym(constants.gravity);
    let rot = state.rotation();
    let vxr = make_skew_sym(state.velocity()) * rot;
    let pxr = make_skew_sym(state.position()) * rot;
    let mut right_invariant_error = SMatrix::zeros();
    right_invariant_error.view_mut::<3, 3>(0, 9).set_from(-rot);
    right_invariant_error.view_mut::<3, 3>(3, 0).set_from(gskew);
    right_invariant_error.view_mut::<3, 3>(3, 9).set_from(-vxr);
    right_invariant_error.view_mut::<3, 3>(3, 9).set_from(-rot);
    right_invariant_error
        .view_mut::<3, 3>(6, 3)
        .set_from(SMatrix::identity());
    right_invariant_error.view_mut::<3, 3>(6, 9).set_from(-pxr);
    right_invariant_error.view_mut::<3, 3>(6, 12).set_from(-rot);
    right_invariant_error
}

fn change_of_basis<const N: usize, const M: usize>(
    basis: SMatrix<N, M>,
    value: SQMatrix<M>,
) -> SQMatrix<N> {
    basis * value * basis.transpose()
}

#[pyclass(module = "umriekf")]
#[derive(Debug)]
#[allow(clippy::upper_case_acronyms)]
pub struct IEKF {
    constants: Constants,
    pred_state: State,
    pred_cov: SQMatrix<15>,
    pred_acc_bias: SVector<3>,
    pred_gyro_bias: SVector<3>,
    pred_biased_acc: SVector<3>,
    pred_biased_gyro: SVector<3>,
    last_imu_time: f64,
}

impl IEKF {
    pub fn new(constants: Constants, init_state: State, init_cov: SQMatrix<15>) -> Self {
        Self {
            constants,
            pred_state: init_state,
            pred_cov: init_cov,
            pred_acc_bias: SMatrix::zeros(),
            pred_gyro_bias: SMatrix::zeros(),
            pred_biased_acc: SMatrix::zeros(),
            pred_biased_gyro: SMatrix::zeros(),
            last_imu_time: 0.,
        }
    }

    fn adj_xb(&self) -> SQMatrix<15> {
        let mut adj = SMatrix::zeros();
        adj.view_mut::<9, 9>(0, 0)
            .set_from(calc_adjoint(&self.pred_state));
        adj
    }

    fn adj_xb_inv(&self) -> SQMatrix<15> {
        let mut adj = SMatrix::zeros();
        adj.view_mut::<9, 9>(0, 0)
            .set_from(calc_adjoint(&self.pred_state.inverse()));
        adj
    }

    fn add_imu_measurement(&mut self, measured_acc: SVector<3>, measured_gyro: SVector<3>) {
        let curr_time = SystemTime::now()
            .duration_since(SystemTime::UNIX_EPOCH)
            .expect("Now is not before unix epoch")
            .as_secs_f64();
        let _dt = curr_time - self.last_imu_time;
        let dt = 1. / self.constants.imu_hz as f64; // Used for testing

        // Add noise to IMU acceleration measurement
        let pred_acc_noise = multivariate_normal(SVector::zeros(), self.constants.cov_acc_noise);
        let pred_acc_bias_noise =
            multivariate_normal(SVector::zeros(), self.constants.cov_acc_bias_noise);
        let pred_acc = measured_acc + pred_acc_noise + pred_acc_bias_noise;

        // Add noise to IMU gyro measurement
        let pred_gyro_noise = multivariate_normal(SVector::zeros(), self.constants.cov_gyro_noise);
        let pred_gyro_bias_noise =
            multivariate_normal(SMatrix::zeros(), self.constants.cov_gyro_bias_noise);
        let pred_gyro = measured_gyro + pred_gyro_noise + pred_gyro_bias_noise;

        self.pred_acc_bias = SMatrix::zeros();
        self.pred_gyro_bias = SMatrix::zeros();
        self.pred_biased_acc = pred_acc - self.pred_acc_bias;
        let pred_global_acc =
            self.pred_state.rotation() * self.pred_biased_acc + self.constants.gravity;
        self.pred_biased_gyro = pred_gyro - self.pred_gyro_bias;

        // Calculate updated state
        let pred_rotation =
            self.pred_state.rotation() * make_skew_sym(self.pred_biased_gyro * dt).exp();
        let pred_vel = self.pred_state.velocity() + pred_global_acc * dt;
        let pred_pos = self.pred_state.position()
            + self.pred_state.velocity() * dt
            + pred_global_acc * dt.powi(2) * 0.5;

        self.pred_state = State::from_components(pred_rotation, pred_vel, pred_pos);

        let phi = (calc_right_invariant_error(&self.pred_state, &self.constants) * dt).exp();
        self.pred_cov = change_of_basis(
            phi,
            self.pred_cov + change_of_basis(self.adj_xb(), self.constants.state_covariance) * dt,
        );

        self.last_imu_time = curr_time;
    }

    fn add_dvl_measurement(&mut self, dvl_velocity: SVector<3>) {
        // Add noise to measurement
        let pred_dvl_noise = multivariate_normal(SVector::zeros(), self.constants.cov_dvl_noise);
        let pred_dvl_vel = dvl_velocity + pred_dvl_noise;

        // Convert predicted velocity into IMU reference frame
        let vel = self.constants.dvl_rotation * pred_dvl_vel
            + make_skew_sym(self.constants.dvl_translation) * self.pred_biased_gyro;
        let vel = vel.0;
        let pred_vel_in_imu_frame = SVector::from_array_1d([vel[0], vel[1], vel[2], -1., 0.]);

        // Note: Currently just using the latest ang velocity reading from the IMU
        // TODO: Convert to using a queue to align IMU & DVL measurements if performance is bad
        let cov_pred_vel_in_imu_frame =
            change_of_basis(self.constants.dvl_rotation, self.constants.cov_dvl_noise)
                + change_of_basis(
                    make_skew_sym(self.constants.dvl_translation),
                    self.constants.cov_gyro_noise + self.constants.cov_gyro_bias_noise,
                );

        // Calculate the measurement covariance
        let measurement_covariance = (self.pred_cov.view::<3, 3>(3, 3)
            + change_of_basis(self.pred_state.rotation(), cov_pred_vel_in_imu_frame))
        .inverse();

        // Calculate the Kalman gains
        let kalman_gains = self.pred_cov.cols::<3>(3) * measurement_covariance;
        let state_kalman_gain = kalman_gains.rows::<9>(0);
        let bias_kalman_gain = kalman_gains.rows::<6>(9);

        // Update the current state estimate to include the DVL measurement
        self.pred_state.matrix =
            carat(state_kalman_gain * self.pred_state.matrix.rows::<3>(0) * pred_vel_in_imu_frame)
                .exp()
                * self.pred_state.matrix;

        let bias_update =
            bias_kalman_gain * (self.pred_state.matrix * pred_vel_in_imu_frame).rows::<3>(0);
        let gyro_bias_update = bias_update.rows::<3>(0);
        let acc_bias_update = bias_update.rows::<3>(3);
        self.pred_gyro_bias = self.pred_gyro_bias + gyro_bias_update;
        self.pred_acc_bias = self.pred_acc_bias + acc_bias_update;

        let mut select = SMatrix::<3, 15>::zeros();
        select.view_mut::<3, 3>(0, 3).set_from(SMatrix::identity());
        self.pred_cov = (SQMatrix::<15>::identity() - kalman_gains * select) * self.pred_cov;
    }

    fn add_depth_measurement(&mut self, depth_measurement: f64) {
        let mut select = SMatrix::zeros();
        select.view_mut::<3, 3>(0, 6).set_from(SMatrix::identity());
        let depth_measurement_from_imu = depth_measurement
            + (self.pred_state.rotation().row(2) * self.constants.depth_translation).0[0];
        let cov_tilde = change_of_basis(select * self.adj_xb_inv(), self.pred_cov)
            .try_inverse()
            .expect("cov squiggle is invertible");

        let depth_cov = SMatrix::from_array([
            [0., 0., 0.],
            [0., 0., 0.],
            [0., 0., 1. / self.constants.std_depth_noise.powi(2)],
        ]);
        let measurement_covariance = cov_tilde
            - (cov_tilde
                * ((change_of_basis(self.pred_state.rotation().transpose(), depth_cov)
                    + cov_tilde)
                    .inverse())
                * cov_tilde);
        let pseudo_measurement = SVector::from_array_1d([
            self.pred_state.matrix.0[(0, 4)],
            self.pred_state.matrix.0[(1, 4)],
            depth_measurement_from_imu,
            0.,
            1.,
        ]);

        let kalman_gains = self.pred_cov
            * self.adj_xb_inv().transpose()
            * select.transpose()
            * measurement_covariance;
        let state_kalman_gain = kalman_gains.rows::<9>(0);
        let bias_kalman_gain = kalman_gains.rows::<6>(9);

        // Update the current state estimate to include the DVL measurement
        let innovation = self.pred_state.inverse().matrix * pseudo_measurement;
        let innovation = innovation.rows::<3>(0);
        self.pred_state.matrix =
            carat(state_kalman_gain * innovation).exp() * self.pred_state.matrix;

        let bias_update = bias_kalman_gain * innovation;
        let gyro_bias_update = bias_update.rows::<3>(0);
        let acc_bias_update = bias_update.rows::<3>(3);
        self.pred_gyro_bias = self.pred_gyro_bias + gyro_bias_update;
        self.pred_acc_bias = self.pred_acc_bias + acc_bias_update;

        self.pred_cov =
            (SMatrix::identity() - kalman_gains * select * self.adj_xb_inv()) * self.pred_cov;
    }
}

#[pymethods]
#[allow(clippy::unit_arg)]
impl IEKF {
    #[new]
    fn py_new<'py>(
        constants_dict: Bound<'py, PyDict>,
        init_state: State,
        init_cov: PyReadonlyArray2<'py, f64>,
    ) -> PyResult<Self> {
        let mut constants = Constants::default();
        constants.update_from_py(&constants_dict)?;
        Ok(Self::new(
            constants,
            init_state,
            SMatrix::try_from(&init_cov).map_err(anyhow::Error::from)?,
        ))
    }

    fn predict(&self) -> State {
        self.pred_state.clone()
    }

    #[pyo3(name = "add_imu_measurement")]
    fn py_add_imu_measurement(
        &mut self,
        measured_acc: PyReadonlyArray1<'_, f64>,
        measured_gyro: PyReadonlyArray1<'_, f64>,
    ) -> PyResult<()> {
        Ok(self.add_imu_measurement(
            SMatrix::try_from(&measured_acc).map_err(anyhow::Error::from)?,
            SMatrix::try_from(&measured_gyro).map_err(anyhow::Error::from)?,
        ))
    }

    #[pyo3(name = "add_dvl_measurement")]
    fn py_add_dvl_measurement(&mut self, dvl_velocity: PyReadonlyArray1<'_, f64>) -> PyResult<()> {
        Ok(
            self.add_dvl_measurement(
                SMatrix::try_from(&dvl_velocity).map_err(anyhow::Error::from)?,
            ),
        )
    }

    #[pyo3(name = "add_depth_measurement")]
    fn py_add_depth_measurement(&mut self, depth_measurement: f64) -> PyResult<()> {
        Ok(self.add_depth_measurement(depth_measurement))
    }

    fn reload_constants(&mut self, constants: Bound<'_, PyDict>) -> PyResult<()> {
        self.constants
            .update_from_py(&constants)
            .map_err(Into::into)
    }

    #[getter]
    fn pred_acc_bias<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<f64>> {
        Array1::from_shape_fn(3, |i| self.pred_acc_bias.0[i]).into_pyarray_bound(py)
    }

    #[getter]
    fn pred_gyro_bias<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<f64>> {
        Array1::from_shape_fn(3, |i| self.pred_gyro_bias.0[i]).into_pyarray_bound(py)
    }
}
