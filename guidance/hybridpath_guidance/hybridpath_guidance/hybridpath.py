import numpy as np
import matplotlib.pyplot as plt

class HybridPathGenerator:
    '''
    Generates the coefficients for subpaths between given waypoints.
    
    Input data:
    WP: Waypoints

    r: Differentiability order

    lambda_val: Curvature constant

    plot_handle: Handle for the plot

    Output data:
    Path: A path structure with the following fields:
        NumSubpaths: Number of subpaths
        Order: Order of the subpaths
        WP: Waypoints
        LinSys: Linear system to solve the subpaths
            A: Coefficients matrix
            bx: x-vector for each subpath
            by: y-vector for each subpath
        coeff: Coefficients
            a: x-coefficients
            b: y-coefficients
            a_der: x-coefficients for derivatives
            b_der: y-coefficients for derivatives

        Based on MATLAB code by prof. Roger Skjetne, NTNU
    '''

    def __init__(self, WP, r, lambda_val, plot_handle = None):
        if len(WP) == 2:
            self.WP = np.array([WP[0], [(WP[0][0] + WP[1][0])/2, (WP[0][1] + WP[1][1])/2], WP[1]])
        else:
            self.WP = WP
        self.r = r
        self.lambda_val = lambda_val
        self.plot_handle = plot_handle
        self.ord = 2*r + 1
        self.Path = {
            'NumSubpaths': len(self.WP) - 1,
            'Order': self.ord,
            'WP': {'x': self.WP[:,0], 'y': self.WP[:,1]},
            'LinSys': {
                'A': None,
                'bx': [],
                'by': []
            },
            'coeff': {
                'a': [],
                'b': [],
                'a_der': [],
                'b_der': []
            }
        }
        self._calculate_subpaths()

    def _calculate_subpaths(self):
        ord_plus_one = self.ord + 1
        A = np.zeros((ord_plus_one, ord_plus_one))
        coefs = np.ones(ord_plus_one)
        A[0,0] = coefs[0]
        A[1,:] = coefs
        c_der = coefs.copy() #
        for k in range(2, (self.ord + 1) // 2 + 1):
            c_der = np.polyder(c_der[::-1])[::-1]
            coefs = np.concatenate([np.zeros(k-1), c_der])
            #coefs = np.pad(c_der, (k-1, 0), 'constant')
            A[2*k-2, k-1] = c_der[0]
            A[2*k-1, :] = coefs
        self.Path['LinSys']['A'] = A

        N = self.Path['NumSubpaths'] #
        for j in range(N):
            ax, bx = np.zeros(ord_plus_one), np.zeros(ord_plus_one)
            ax[:2] = self.WP[j:j+2, 0]
            bx[:2] = self.WP[j:j+2, 1]
            if self.ord > 2: # More than two waypoints
                if j == 0:
                    ax[2:4] = [self.WP[j+1, 0] - self.WP[j, 0], self.lambda_val * (self.WP[j+2, 0] - self.WP[j, 0])] 
                    bx[2:4] = [self.WP[j+1, 1] - self.WP[j, 1], self.lambda_val * (self.WP[j+2, 1] - self.WP[j, 1])] 
                elif j == N - 1:
                    ax[2:4] = [self.lambda_val * (self.WP[j+1, 0] - self.WP[j-1, 0]), self.WP[j+1, 0] - self.WP[j, 0]]
                    bx[2:4] = [self.lambda_val * (self.WP[j+1, 1] - self.WP[j-1, 1]), self.WP[j+1, 1] - self.WP[j, 1]]

                else:
                    ax[2:4] = [self.lambda_val * (self.WP[j+1, 0] - self.WP[j-1, 0]), self.lambda_val * (self.WP[j+2, 0] - self.WP[j, 0])]
                    bx[2:4] = [self.lambda_val * (self.WP[j+1, 1] - self.WP[j-1, 1]), self.lambda_val * (self.WP[j+2, 1] - self.WP[j, 1])]
            
            a_vec = np.linalg.solve(A, ax)
            b_vec = np.linalg.solve(A, bx)
            self.Path['LinSys']['bx'].append(ax)
            self.Path['LinSys']['by'].append(bx)
            self.Path['coeff']['a'].append(a_vec)
            self.Path['coeff']['b'].append(b_vec)
            for k in range(1, self.ord + 1):
                a_vec = np.polyder(a_vec[::-1])[::-1]
                b_vec = np.polyder(b_vec[::-1])[::-1]
                if k - 1 < len(self.Path['coeff']['a_der']): 
                    self.Path['coeff']['a_der'][k - 1].append(a_vec)
                    self.Path['coeff']['b_der'][k - 1].append(b_vec)
                else:
                    self.Path['coeff']['a_der'].append([a_vec])
                    self.Path['coeff']['b_der'].append([b_vec])
                # a_poly = Polynomial(a_vec).deriv(k).coef
                # b_poly = Polynomial(b_vec).deriv(k).coef
                # self.Path['coeff']['a_der'].append(a_poly)
                # self.Path['coeff']['b_der'].append(b_poly)
                    
    @staticmethod
    def update_s(path, dt, u_d, s):
        signals = HybridPathSignals(path, s)
        v_ref, _ = signals.calc_vs(u_d)
        s = v_ref * dt
        return s

    def plot_path(self):
        if self.plot_handle is not None:
            e = np.arange(0, 1, 0.01)
            plt.figure(self.plot_handle)
            plt.clf()
            for j in range(self.Path['NumSubpaths']):
                X,Y = np.zeros_like(e), np.zeros_like(e)
                for k in range(self.ord + 1):
                    X += self.Path['coeff']['a'][j][k] * e**k
                    Y += self.Path['coeff']['b'][j][k] * e**k
                if j == 0:
                    plt.plot(Y,X,'b', linewidth = 1.25, label = 'ASV Reference Path')
                else:
                    plt.plot(Y,X,'b', linewidth = 1.25)
            
            plt.plot(self.WP[:, 1], self.WP[:, 0], 'r*', linewidth = 1.5, label = 'Waypoints')
            plt.xlabel('y')
            plt.ylabel('x')
            plt.axis('equal')
            minX, maxX = np.min(self.WP[:, 1]), np.max(self.WP[:, 1])
            minY, maxY = np.min(self.WP[:, 0]), np.max(self.WP[:, 0])
            plt.axis([1.1*minX - 0.1*maxX, 1.1*maxX - 0.1*minX, 1.1*minY - 0.1*maxY, 1.1*maxY - 0.1*minY])
            plt.grid()
            plt.legend()
            plt.title('Path')
            plt.show()

            
class HybridPathSignals:
    def __init__(self, path, s):
        self.path = path
        self.s = self._clamp_s(s, path['NumSubpaths'])
        self.ord = path['Order']
        self.pd = None
        self.pd_der = []
        self.pd, self.pd_der = self.get_signals()
        self.psi, self.psi_der, self.psi_dder = self.get_heading()
        
    
    def _clamp_s(self, s, num_subpaths):
        """
        Ensure s is within the valid range.
        """
        if s < 0:
            return 0.0
        elif s >= num_subpaths:
            return num_subpaths - 1e-3 # A small epsilon
        return s
    
    def get_signals(self):
        """
        Compute the location and derivative at s.
        """
        idx = int(self.s) + 1
        t = self.s - (idx - 1)
        xd, yd = 0, 0
        # Compute position
        for k in range(self.ord + 1):
            xd += self.path['coeff']['a'][idx - 1][k] * t**k
            yd += self.path['coeff']['b'][idx - 1][k] * t**k
        self.pd = [xd, yd]

        # Compute derivatives
        for k in range(1, self.ord + 1): ##
            xd_der, yd_der = 0, 0
            # print('Index number: ', idx, ' was OK! Max is: ', self.path['NumSubpaths'])
            # print('k: ', k, ' was OK! Max is: ', len(self.path['coeff']['a_der']))
            a_vec = self.path['coeff']['a_der'][k - 1][idx - 1]
            b_vec = self.path['coeff']['b_der'][k - 1][idx - 1]
            for j in range(len(a_vec)):
                xd_der += a_vec[j] * t**j
                yd_der += b_vec[j] * t**j
            self.pd_der.append([xd_der, yd_der])

        return self.pd, self.pd_der
    
    def get_heading(self):
        psi = np.arctan2(self.pd_der[0][1], self.pd_der[0][0])
        psi_der = (self.pd_der[0][0] * self.pd_der[1][1] - self.pd_der[0][1] * self.pd_der[1][0]) / (self.pd_der[0][0]**2 + self.pd_der[0][1]**2)
        psi_dder = (self.pd_der[0][0] * self.pd_der[2][1] - self.pd_der[0][1] * self.pd_der[2][0]) / (self.pd_der[0][0]**2 + self.pd_der[0][1]**2) - 2 * (self.pd_der[0][0] * self.pd_der[1][1] - self.pd_der[1][0] * self.pd_der[0][1]) * (self.pd_der[0][0] * self.pd_der[1][0] - self.pd_der[1][1] * self.pd_der[0][1]) / ((self.pd_der[0][0]**2 + self.pd_der[0][1]**2)**2)
        return psi, psi_der, psi_dder
    
    def calc_vs(self, u_d):
        #vs = u_d/np.sqrt(self.pd_der[0][0]**2 + self.pd_der[0][1]**2)
        vs = u_d / np.linalg.norm(self.pd_der[0])
        vs_s = -u_d * (np.array(self.pd_der[0]) @ np.array(self.pd_der[1])) / (np.sqrt(self.pd_der[0][0]**2 + self.pd_der[0][1]**2)**3)
        return vs, vs_s


if __name__ == '__main__':
    # Example Parameters
    pos = np.array([[10, 0],
        [10, 10],
        [0, 20],
        [30, 30],
        [40,0],
        [0,-10],
        [0, 0],
        [10, 0]])
    
    pos2 = np.array([[15,-3],
                     [33, 5],
                     [30, 20],
                     [15, 19],
                     [15, -3]])
    
    s = 0.5  # Path parameter
    lambda_val = 0.4  # Curvature constant
    r = 1  # Differentiability order
    PlotHandle = 1  # 1: Plot path, 2: Plot path and derivatives
    generator = HybridPathGenerator(pos, r, lambda_val, PlotHandle)
    signals = HybridPathSignals(generator.Path, s)
    
    if PlotHandle >= 1:
        plt.figure()
        generator.plot_path()
        if PlotHandle == 1:
            plt.show()


    # Extra plotting
    if PlotHandle > 1:
        s = np.arange(0, generator.Path['NumSubpaths'], 0.01)
        for k in range(signals.ord):
            pd_der = np.zeros((len(s), 2))
            for i in range(len(s)):
                PS = HybridPathSignals(generator.Path, s[i])
                pd_der[i, :] = PS.pd_der[k]
            plt.figure('Derivative number: ' + str(k+1))
            plt.plot(s, pd_der[:, 1], s, pd_der[:, 0])
            plt.legend(['y', 'x'])
            plt.grid()
        plt.show()