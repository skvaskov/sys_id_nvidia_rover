import numpy as np
import scipy.linalg
from copy import deepcopy
from threading import Lock


class UKFException(Exception):
    """Raise for errors in the UKF, usually due to bad inputs"""


class UKF:
    def __init__(self, num_states, num_observations, process_noise, initial_state, initial_covar, alpha, k, beta, iterate_function,observation_function):
        """
        Initializes the unscented kalman filter
        :param num_states: int, the size of the state
        :param process_noise: the process noise covariance per unit time, should be num_states x num_states
        :param initial_state: initial values for the states, should be num_states x 1
        :param initial_covar: initial covariance matrix, should be num_states x num_states, typically large and diagonal
        :param alpha: UKF tuning parameter, determines spread of sigma points, typically a small positive value
        :param k: UKF tuning parameter, typically 0 or 3 - num_states
        :param beta: UKF tuning parameter, beta = 2 is ideal for gaussian distributions
        :param iterate_function: function that predicts the next state
                    takes in a num_states x 1 state and a float timestep and inputs
                    returns a num_states x 1 state
        :param observation_function: function the translates states to measurements
                    takes in a num_states x 1 state and inputs
                    returns a num_observations x 1 measurement
        """
        """
            The following code corresponds to block (15)

            Following the UKF paper by Wan, van der Merwe
            self.covar_weights[0]=W_{0}^{c}
            self.mean_weights[0]=W_{0}^{m}
            self.mean_weights[i]=W_{i}^{m}
            self.covar_weights[i]=W_{i}^{c}
            self.p=P_{x}
            self.sigmas=X matrix n_dim x n_sig matrix. column 0 is the inital state
        """
        self.n_dim = int(num_states)
        self.n_obs = int(num_observations)
        self.n_sig = 1 + num_states * 2
        self.q = process_noise
        self.x = initial_state
        self.p = initial_covar
        self.beta = beta
        self.alpha = alpha
        self.k = k
        self.iterate = iterate_function
        self.observe = observation_function

        #scaling parameter
        self.lambd = pow(self.alpha, 2) * (self.n_dim + self.k) - self.n_dim

        self.covar_weights = np.zeros(self.n_sig)
        self.mean_weights = np.zeros(self.n_sig)

        self.covar_weights[0] = (self.lambd / (self.n_dim + self.lambd)) + (1 - pow(self.alpha, 2) + self.beta)
        self.mean_weights[0] = (self.lambd / (self.n_dim + self.lambd))

        for i in range(1, self.n_sig):
            self.covar_weights[i] = 1 / (2*(self.n_dim + self.lambd))
            self.mean_weights[i] = 1 / (2*(self.n_dim + self.lambd))

        self.sigmas = self.__get_sigmas()

        self.lock = Lock()



    def __get_sigmas(self):
        """generates sigma points"""
        ret = np.zeros([self.n_sig, self.n_dim])
        gamma=np.sqrt(self.n_dim + self.lambd)
        tmp_mat = self.p

        # print spr_mat
        spr_mat = scipy.linalg.sqrtm(tmp_mat)

        ret[0] = self.x
        for i in range(self.n_dim):
            ret[i+1] = self.x + gamma*spr_mat[i]
            ret[i+1+self.n_dim] = self.x - gamma*spr_mat[i]

        return ret.T

    def update(self, observations, data,r_matrix,inputs=[]):
        """
        performs a measurement update
        :param observations: list of indices (zero-indexed) of which states were measured, that is, which are being updated
        :param data: list of the data corresponding to the values in states
        :param r_matrix: error matrix for the data, again corresponding to the values in states

        Modifed to follow Algorithm 2.1 in  Surat Kwanmuang's PhD thesis
        self.x=x_{k}^{^-}
        self.sigma=X_{k}^{-}  n_dimx n_sig matrix. column 0 is the inital state
        self.p=P_{k}^{-}
        """

        self.lock.acquire()

        n_data=len(data)
        #Line (2.16)
        Z_k=np.array([self.observe(x,inputs) for x in self.sigmas.T]).T
        #Z_k is n_obs x n_sig
        #reduce Z_k to only the data we have measurements for
        Z_k=Z_k[observations,:]
        #Z_k is now n_data x n_sig

        #Line (2.17)
        z_kmean=np.zeros(n_data)
        for i in range(n_data):
            # the mean of that variable the sum of
            # the weighted values of that variable for each iterated sigma point
            z_kmean[i] = sum((self.mean_weights[j] * Z_k[i][j] for j in range(self.n_sig)))

        #Line (2.18)
        # differences in y from y mean
        Z_diff = deepcopy(Z_k)
        x_diff = deepcopy(self.sigmas)
        for i in range(self.n_sig):
            for j in range(n_data):
                Z_diff[j][i] -= z_kmean[j]
            for j in range(self.n_dim):
                x_diff[j][i] -= self.x[j]

        # variance of measurement
        P_zz = np.zeros([n_data,n_data])
        for i, val in enumerate(np.array_split(Z_diff, self.n_sig, 1)):
            P_zz += self.covar_weights[i] * val.dot(val.T)

        # add measurement noise
        P_zz += r_matrix

        #Line (2.19)
        # covariance of measurement with states
        P_xz = np.zeros([self.n_dim, n_data])
        for i, val in enumerate(zip(np.array_split(Z_diff, self.n_sig, 1), np.array_split(x_diff, self.n_sig, 1))):
            P_xz += self.covar_weights[i] * val[1].dot(val[0].T)

        #Line (2.20)
        K = np.dot(P_xz, np.linalg.inv(P_zz))
        
        z_actual = data

        #Line (2.21)
        self.x += np.dot(K, (z_actual - z_kmean))
        #Line (2.22)
        self.p -= np.dot(K, np.dot(P_zz, K.T))
        #Line (2.12)
        self.sigmas = self.__get_sigmas()

        self.lock.release()

    def predict(self, timestep, inputs=[]):
        """
        performs a prediction step
        :param timestep: float, amount of time since last prediction
        comments with line numbers correspond to line in Algorithm 2.1 in Surat Kwanmuang's PhD thesis

        self.x=x_out=x_{k}^{^-}
        self.sigma=sigmas_out=X_{k}^{-} n_dimxn_sig
        self.p=p_out=P_{k}^{-}
        """

        self.lock.acquire()


        #line (2.13)
        sigmas_out = np.array([self.iterate(x, timestep, inputs) for x in self.sigmas.T]).T

        #line (2.14)
        x_out = np.zeros(self.n_dim)
        # for each variable in X
        for i in range(self.n_dim):
            # the mean of that variable is the sum of
            # the weighted values of that variable for each iterated sigma point
            x_out[i] = sum((self.mean_weights[j] * sigmas_out[i][j] for j in range(self.n_sig)))

        #Line (2.15)
        p_out = np.zeros((self.n_dim, self.n_dim))
        # for each sigma point
        for i in range(self.n_sig):
            # take the distance from the mean
            # make it a covariance by multiplying by the transpose
            # weight it using the calculated weighting factor
            # and sum
            diff = sigmas_out.T[i] - x_out
            diff = np.atleast_2d(diff)
            p_out += self.covar_weights[i] * np.dot(diff.T, diff)

        # add process noise
        p_out += timestep * self.q

        self.x = x_out
        self.p = p_out
        #Line (2.12)
        self.sigmas = self.__get_sigmas()

        self.lock.release()

    def get_state(self, index=-1):
        """
        returns the current state (n_dim x 1), or a particular state variable (float)
        :param index: optional, if provided, the index of the returned variable
        :return:
        """
        if index >= 0:
            return self.x[index]
        else:
            return self.x

    def get_covar(self):
        """
        :return: current state covariance (n_dim x n_dim)
        """
        return self.p

    def set_state(self, value, index=-1):
        """
        Overrides the filter by setting one variable of the state or the whole state
        :param value: the value to put into the state (1 x 1 or n_dim x 1)
        :param index: the index at which to override the state (-1 for whole state)
        """
        with self.lock:
            if index != -1:
                self.x[index] = value
            else:
                self.x = value
            self.sigmas=self.__get_sigmas()

    def reset(self, state, covar):
        """
        Restarts the UKF at the given state and covariance
        :param state: n_dim x 1
        :param covar: n_dim x n_dim
        """

        self.lock.acquire()
        self.x = state
        self.p = covar
        self.sigmas = self.__get_sigmas()
        self.lock.release()
