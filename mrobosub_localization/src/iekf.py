import numpy as np
from numpy.linalg import inv
from scipy.linalg import expm
from tqdm import tqdm
import warnings 
   
class HoveringAUV:
    def __init__(self, Q, R, filename, dvl_p=np.zeros(3), dvl_r=np.eye(3)):
        """Our system for a quadcopter, with IMU measurements as controls. 

        Args:
            filename      (str) : Where the recorded data has been stored
            Q   (15,15 ndarray) : Covariance of noise on state
            R     (6,6 ndarray) : Covariance of noise on measurements
            dvl_p   (3 ndarray) : Translation of DVL from IMU
            dvl_r (3,3 ndarray) : Rotation matrix of DVL from IMU"""
        self.Q = Q
        self.R = R
        if filename is not None:
            self.data = np.load(filename)
        else:
            self.data = None
        self.Hz = self.data['ticks']
        self.T = self.data['x'].shape[0]
        # local velocity
        self.b1 = np.array([0, 0, 0, -1, 0])
        # global depth
        self.b2 = np.array([0, 0, 0, 0, 1])

        self.dvl_p = HoveringAUV.cross(dvl_p)
        self.dvl_r = dvl_r

        # convert z1 noise into the correct frame
        self.R[:3,:3] = self.dvl_r@self.R[:3,:3]@self.dvl_r.T + self.dvl_p@(self.Q[0:3,0:3] + self.Q[9:12,9:12])@self.dvl_p.T

    def gen_data(self, t, imuHz, dvlHz, depthHz, noise=True, sim_bias=True):
        """Generates model data using Lie Group model.

        Args:
            t         (int) : How many timesteps to run
            noise    (bool) : Whether or not to include noise. Defaults to True.
            sim_bias (bool) : Whether or not to include bias. Defaults to True.

        Returns:
            x    (t,5,5 ndarray) : x steps after x0 
            u    (t,2,3 ndarray) : controls that were applied
            bias (t,2,3 ndarray) : bias of IMU (0s if bias is turned off)
            z      (t,3 ndarray) : measurements taken.
        """
        #Get closest sample rate we can
        imu_every = self.Hz // imuHz
        dvl_every = self.Hz // dvlHz
        depth_every = self.Hz // depthHz

        if t > self.T:
            warnings.warn (f"You requested {t} steps, there's only {self.T} available.")
            t = self.T
        t_range = np.linspace(0, self.T/self.Hz, self.T)
        x  = self.data['x'][:t:imu_every]
        u  = self.data['u'][:t:imu_every]
        u[:,[0,1]] = u[:,[1,0]]
        z1 = self.data['z1'][:t:dvl_every]
        z2 = self.data['z2'][:t:depth_every]
        bias = np.zeros_like(u)

        if noise:
            temp = np.random.multivariate_normal(mean=np.zeros(3), cov=self.R[:3,:3], size=z1.shape[0])
            z1 += temp

            temp = np.random.normal(loc=np.zeros(1), scale=np.sqrt(self.R[-1,-1]), size=z2.shape[0])
            z2 += temp

            temp = np.random.multivariate_normal(mean=np.zeros(15), cov=self.Q, size=u.shape[0])
            u[:,0] += temp[:,0:3]
            u[:,1] += temp[:,3:6]
            if sim_bias:
                bias[:,0] += np.cumsum(temp[:,9:12], axis=0)
                bias[:,1] += np.cumsum(temp[:,12:15], axis=0)
                u += bias


        return (x, t_range[:t:imu_every]), (u, t_range[:t:imu_every]), (bias, t_range[:t:imu_every]), (z1, t_range[:t:dvl_every]), (z2, t_range[:t:depth_every])

    def f_lie(self, state, u, dt, bias):
        """Propagates state forward in Lie Group. Used for IEKF.

        Args:
            state (5,5 ndarray) : X_n of model in Lie Group
            u     (2,3 ndarray) : U_n of model (IMU measurements)
            bias  (2,3 ndarray) : Estimated bias

        Returns:
            X_{n+1} (5,5 ndarray)"""
        # transform u into the right representation
        #get stuff we need
        g = np.array([0, 0, -9.8])
        R = state[:3,:3]
        v = state[:3,3]
        p = state[:3,4]

        omega = u[0] - bias[0]
        a = u[1] - bias[1]

        ## put it together
        Rnew = R @ expm( self.cross( omega*dt ))
        vnew = v + (R@a + g)*dt
        pnew = p + v*dt + (R@a + g)*dt**2/2

        return np.block([[Rnew, vnew.reshape(-1,1), pnew.reshape(-1,1)],
                        [np.zeros((2,3)), np.eye(2)]])

    def h(self, state):
        """Calculates measurement given a state. Note that the result is
            the same if it's in standard or Lie Group form, so we simplify into
            one function.
            
        Args:
            state (3,3 ndarray) : Current state in either standard or Lie Group form
            noise        (bool) : Whether or not to add noise. Defaults to False.

        Returns:
            z1 (3 ndarray) : DVL Velocity measurement
            z2   (1 float) : Depth measurement"""
        # local velocity
        z1 = ( inv(state) @ self.b1 )[:3]
        # global depth
        z2 = ( (state) @ self.b2 )[2]

        return z1, z2

    @staticmethod
    def cross(x):
        """Moves a 3 vector into so(3)

        Args:
            x (3 ndarray) : Parametrization of Lie Algebra

        Returns:
            x (3,3 ndarray) : Element of so(3)"""
        return np.array([[   0, -x[2],  x[1]],
                        [ x[2],     0, -x[0]],
                        [-x[1],  x[0],     0]])

    @staticmethod
    def carat(xi):
        """Moves a 9 vector to the Lie Algebra se_2(3).

        Args:
            xi (9 ndarray) : Parametrization of Lie algebra

        Returns:
            xi^ (5,5 ndarray) : Element in Lie Algebra se_2(3)"""
        w_cross = HoveringAUV.cross(xi[0:3])
        v       = xi[3:6].reshape(-1,1)
        p       = xi[6:9].reshape(-1,1)
        return np.block([[w_cross, v, p],
                         [np.zeros((2,5))]])

    @staticmethod
    def adjoint(xi):
        """Takes adjoint of element in SE_2(3)

        Args:
            xi (5,5 ndarray) : Element in Lie Group

        Returns:
            Ad_xi (9,9 ndarray) : Adjoint in SE_2(3)"""
        R = xi[:3,:3]
        v_cross = HoveringAUV.cross(xi[:3,3])
        p_cross = HoveringAUV.cross(xi[:3,4])
        zero    = np.zeros((3,3))
        return np.block([[        R, zero, zero],
                         [v_cross@R,    R, zero],
                         [p_cross@R, zero,   R]])

# we do our testing down here
if __name__ == "__main__":
    import sys
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    from scipy.spatial.transform import Rotation
    np.set_printoptions(suppress=True, formatter={'float_kind':f'{{:0.7f}}'.format}) 

    ############################     SETUP SYSTEM    ############################
    # Setup rates of sensors
    imuHz = 200
    dvlHz = 20
    depthHz = 100

    # Set up various noises 
    # std         - spec *    units    *  sqrt rate
    std_a         =  20  * 10**-6*9.8 * np.sqrt(imuHz)
    std_gyro      = .005 *  np.pi/180  * np.sqrt(imuHz) 
    std_dvl       = .0101*     2.6
    std_depth     =  51  *    1/100    *    1/2
    std_a_bias    = .0001              * np.sqrt(imuHz) # See https://arxiv.org/pdf/1402.5450.pdf
    std_gyro_bias = .000618*   8/18    * np.sqrt(imuHz)
    Q = np.diag([std_gyro, std_gyro, std_gyro, std_a, std_a, std_a, 0, 0, 0, 
                std_gyro_bias, std_gyro_bias, std_gyro_bias, std_a_bias, std_a_bias, std_a_bias])**2
    R = np.diag([std_dvl, std_dvl, std_dvl, 
                std_depth])**2

    # DVL offsets
    dvl_p = np.array([-0.17137, 0.00922, -0.33989])
    dvl_r = Rotation.from_euler('xyz', [6, 3, 90], degrees=True).as_matrix()

    #make system
    sys = HoveringAUV(Q, R, 'data/test_move_forward.npz', dvl_p, dvl_r)
    t = sys.T

    # load data, and run it through our system to check they match (ish)
    (x, tx), (u, tu), (bias, tb), (z1, tdvl), (z2, tdepth) = sys.gen_data(t, imuHz, dvlHz, depthHz, noise=False, sim_bias=False)
    x_result = np.zeros((t, 5, 5))
    x_result[0] = x[0]
    dt = 0
    ti_last = 0
    for i, (ti, ui, bi) in enumerate(zip(tu, u, bias)):
        dt = ti - ti_last
        ti_last = ti
        if i+1 == t:
            break
        print(ti, end=' ')
        x_result[i+1] = sys.f_lie(x_result[i], ui, dt, bi)

    # Plot everything as requested
    plot = 'avp'
    if plot != '':
        t = tx
        # set up our figure
        rows = len(plot) if 'b' not in plot else len(plot)+1
        fig, ax = plt.subplots(rows, 3, figsize=(8, rows*2+2))
        if rows == 1:
            ax = ax.reshape((1,3))

        i = 0
        # iterate through plotting everything
        for p in plot:
            # plot angles
            if p == "a":
                ax[i,0].set_ylabel("Angles")
                ax[i,0].set_title("Pitch")
                ax[i,0].plot(t, -np.arcsin(x[:,2,0]), label="Actual")
                ax[i,0].plot(t, -np.arcsin(x_result[:,2,0]), label="Predicted")
                ax[i,1].set_title("Roll")
                ax[i,1].plot(t, np.arctan2(x[:,2,1], x[:,2,2]), label="Actual")
                ax[i,1].plot(t, np.arctan2(x_result[:,2,1], x_result[:,2,2]), label="Predicted")
                ax[i,2].set_title("Yaw")
                ax[i,2].plot(t, np.arctan2(x[:,1,0], x[:,0,0]), label="Actual")
                ax[i,2].plot(t, np.arctan2(x_result[:,1,0], x_result[:,0,0]), label="Predicted")
                i += 1

            # plot velocity
            if p == 'v':
                ax[i,0].set_ylabel("Velocity")
                ax[i,0].set_title("X (global)")
                ax[i,1].set_title("Y (global)")
                ax[i,2].set_title("Z (global)")
                for j in range(3):
                    ax[i,j].plot(t, x[:,j,3], label="Actual")
                    ax[i,j].plot(t, x_result[:,j,3], label="Predicted")
                i += 1
                
            # plot position
            if p == 'p':
                ax[i,0].set_ylabel("Position")
                ax[i,0].set_title("X (global)")
                ax[i,0].plot(t, x[:,0,4], label="Actual")
                ax[i,0].plot(t, x_result[:,0,4], label="Predicted")
                ax[i,1].set_title("Y (global)")
                ax[i,1].plot(t, x[:,1,4], label="Actual")
                ax[i,1].plot(t, x_result[:,1,4], label="Predicted")
                ax[i,2].set_title("Z (global)")
                ax[i,2].plot(t, x[:,2,4], label="Actual")
                ax[i,2].plot(t, x_result[:,2,4], label="Predicted")
                i += 1

            # plot bias
            if p == 'b':
                for j in range(3):
                    ax[i,j].set_ylabel("Gyro Bias")
                    ax[i,j].plot(t, bias[:,0,j], label="Actual")
                    ax[i,j].plot(t, bias_result[:,0,j], label="Result")
                    ax[i+1,j].set_ylabel("Accel Bias")
                    ax[i+1,j].plot(t, bias[:,1,j], label="Actual")
                    ax[i+1,j].plot(t, bias_result[:,1,j], label="Result")
                i += 2

        ax[-1,2].legend(loc='best')
        fig.tight_layout()
        plt.show()


class IEKF:
    def __init__(self, system, mu_0, sigma_0, left=False):
        """The newfangled Invariant Extended Kalman Filter

        Args:
            system      (class) : The system to run the iEKF on. It will pull Q, R, f, h from this.
            mu0     (nxn array) : Initial starting point of system
            sigma0  (mxm array) : Initial covariance of system"""
        if mu_0.shape == (9,):
            mu_0 = expm( system.carat(mu_0) )

        self.sys = system
        self.mus = [mu_0]
        self.sigmas = [sigma_0]
        self.biass = [np.zeros((2,3))]

        self.invR = np.zeros((3,3))
        self.invR[-1,-1] = 1 / self.sys.R[3,3]

        self.left = left

    # These 3 properties are helpers. Since our mus and sigmas are stored in a list
    # it can be a pain to access the last one. These provide "syntactic sugar"
    @property
    def mu(self):
        return self.mus[-1]

    @property
    def sigma(self):
        return self.sigmas[-1]

    @property
    def bias(self):
        return self.biass[-1]

    def predict(self, u, dt):
        """Runs prediction step of iEKF.

        Args:
            u       (k ndarray) : control taken at this step

        Returns:
            mu    (nxn ndarray) : Propagated state
            sigma (nxn ndarray) : Propagated covariances"""

        #get mubar and sigmabar
        mu_bar = self.sys.f_lie(self.mu, u, dt, self.bias)

        # make adjoint
        I = np.eye(6)
        zero = np.zeros((9,6))
        adj_X = np.block([[self.sys.adjoint(mu_bar), zero],
                          [                  zero.T,    I]])

        # make propagation matrix
        zero    = np.zeros((3,3))
        I       = np.eye(3)
        if self.left:
            w_cross = self.sys.cross( u[0] - self.bias[0] )
            a_cross = self.sys.cross( u[1] - self.bias[1] )
            self.expA = expm( np.block([[-w_cross,     zero,     zero,   -I, zero],
                                        [-a_cross, -w_cross,     zero, zero,   -I],
                                        [    zero,        I, -w_cross, zero, zero],
                                        [    zero,     zero,     zero, zero, zero],
                                        [    zero,     zero,     zero, zero, zero]])*dt ) 
            sigma_bar = self.expA @ self.sigma @ self.expA.T + self.expA@ self.sys.Q @self.expA.T * dt

        else:
            R       = mu_bar[:3,:3]
            g_cross = self.sys.cross( np.array([0, 0, -9.8]) )
            # v_cross = self.sys.cross( mu_bar[:3,3] )
            # p_cross = self.sys.cross( mu_bar[:3,4] )
            self.expA = expm( np.block([[   zero, zero, zero,         -R, zero],
                                        [g_cross, zero, zero, -adj_X[3:6,0:3],   -R],
                                        [   zero,    I, zero, -adj_X[6:9,0:3], zero],
                                        [   zero, zero, zero,       zero, zero],
                                        [   zero, zero, zero,       zero, zero]])*dt ) 
            sigma_bar = self.expA @ (self.sigma + adj_X@self.sys.Q@adj_X.T*dt) @self.expA.T

        #save for use later
        self.last_u = u
        self.mus.append( mu_bar )
        self.biass.append( self.bias )
        self.sigmas.append( sigma_bar )

        return mu_bar, sigma_bar

    def update_depth(self, z, sim_bias=True):
        """Runs correction step of iEKF.

        Args:
            z (m ndarray): measurement at this step

        Returns:
            mu    (nxn ndarray) : Corrected state
            sigma (nxn ndarray) : Corrected covariances"""
        # make H matrices
        zero    = np.zeros((3,3))
        I       = np.eye(3)    
        H = np.block([zero, zero, I, zero, zero])

        I = np.eye(6)
        zero = np.zeros((9,6))
        # convert H to Right measurement
        if not self.left:
            H = H@np.block([[self.sys.adjoint(inv(self.mu)), zero],
                            [                          zero.T,    I]])

        # put our measurements into full form
        z = np.array([self.mu[0,4], self.mu[1,4], z, 0, 1])

        # make innovation
        V = (inv(self.mu) @ z)[:3]

        R = self.mu[:3,:3]

        # make our special measurement covariance
        sig_til = inv(H@self.sigma@H.T)
        meas_cov   = sig_til - sig_til@inv(R.T@self.invR@R + sig_til)@sig_til

        K = self.sigma @ H.T @ meas_cov
        K_state = K[:9]
        K_bias = K[9:]
        if self.left:
            self.mus[-1] = self.mu @ expm( self.sys.carat(K @ V) )
        else:
            self.mus[-1] = expm( self.sys.carat(K @ V) ) @ self.mu
        if sim_bias:
            self.biass[-1] = self.bias + ( K_bias@V ).reshape((2,3))
        self.sigmas[-1] = (np.eye(15) - K @ H) @ self.sigma

        return self.mu, self.sigma

    def update_dvl(self, z, sim_bias=True):
        """Runs correction step of iEKF.

        Args:
            z (m ndarray): measurement at this step

        Returns:
            mu    (nxn ndarray) : Corrected state
            sigma (nxn ndarray) : Corrected covariances"""
        # convert dvl into correct frame
        z = self.sys.dvl_r@z + self.sys.dvl_p@(self.last_u[0] - self.bias[0])

        # make H matrices
        zero    = np.zeros((3,3))
        I       = np.eye(3)    
        H = np.block([zero, I, zero, zero, zero])

        I = np.eye(6)
        zero = np.zeros((9,6))
        # convert H1 to Left measurement
        if self.left:
            H = H@np.block([[self.sys.adjoint(self.mu), zero],
                            [                     zero.T,    I]])

        # put our measurements into full form
        z = np.array([z[0], z[1], z[2], -1, 0])

        # make innovation
        V = (self.mu @ z)[:3]

        R = self.mu[:3,:3]

        # make our special measurement covariance
        meas_cov   = inv(H@self.sigma@H.T + R@self.sys.R[:3,:3]@R.T)

        K = self.sigma @ H.T @ meas_cov
        K_state = K[:9]
        K_bias = K[9:]
        if self.left:
            self.mus[-1] = self.mu @ expm( self.sys.carat(K_state @ V) )
        else:
            self.mus[-1] = expm( self.sys.carat(K_state @ V) ) @ self.mu
        if sim_bias:
            self.biass[-1] = self.bias + ( K_bias@V ).reshape((2,3))
        self.sigmas[-1] = (np.eye(15) - K @ H) @ self.sigma

        return self.mu, self.sigma

    def iterate(self, u, tu, z1, tz1, z2, tz2, sim_bias=True, use_tqdm=True):
        """Given a sequence of observation, iterate through EKF

        Args:
            us (txk ndarray) : controls for each step, each of size k
            zs (txm ndarray) : measurements for each step, each of size m

        Returns:
            mus    (txnxn ndarray) : resulting states
            sigmas (txnxn ndarray) : resulting sigmas"""
        ui = 0
        z1i = 0
        z2i = 0
        t_range = np.linspace(0, self.sys.T/self.sys.Hz, self.sys.T)

        # append one at the end, so after they're done they won't keep running
        tu = np.append(tu, t_range[-1]+1)
        tz1 = np.append(tz1, t_range[-1]+1)
        tz2 = np.append(tz2, t_range[-1]+1)

        dt = 0
        for t in tqdm(t_range, disable=(not use_tqdm)):
            if tu[ui] <= t:
                if ui != 0:
                    dt = tu[ui] - tu[ui-1]
                self.predict(u[ui], dt)
                if ui < len(tu)-1:
                    ui += 1
            if tz1[z1i] <= t:
                self.update_dvl(z1[z1i], sim_bias)
                if z1i < len(tz1)-1:
                    z1i += 1
            if tz2[z2i] <= t:
                self.update_depth(z2[z2i], sim_bias)
                if z2i < len(tz2)-1:
                    z2i += 1

        return np.array(self.mus)[1:], np.array(self.biass)[1:], np.array(self.sigmas)[1:]
