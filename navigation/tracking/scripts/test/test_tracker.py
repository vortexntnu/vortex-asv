import numpy as np

def dummy(number):
    return number**2

def test_dummy():
    num = 2

    print("running dummy test")

    assert (num**2 == dummy(num))

#-------------------------------------

class Tracker:

    def __init__(self):
        #x = [r, thetha, r', theta']

        self.time_step = 0.002
        self.x_pri = np.ndarray((4,), buffer = np.array(
            [0.0, 0.0, 0.0, 0.0]), 
            dtype=float)
        self.P_pri = np.ndarray((4,4), buffer=np.array([
            [0.0  ,0.0,  0.0,  0.0], 
            [0.0,  0.0,  0.0,  0.0], 
            [0.0,  0.0,  0.0,  0.0], 
            [0.0,  0.0,  0.0,  0.0]]), 
            dtype = float)

        self.x_post = np.ndarray((4,), dtype=float)
        self.P_post = np.ndarray((4,4), dtype=float)

        self.L = np.ndarray((2,2), dtype=float)

        self.A = np.ndarray((4,4), buffer=np.array([
            [1.002, 0, 0.002004, 0], 
            [0, 1.002, 0, 0.002004],
            [0, 0, 1.002, 0], #assuming constnat velocity
            [0, 0, 0, 1.002]]),#assuming constnat velocity
            dtype = float)

        self.A_cont = np.ndarray((4,4), buffer=np.array([
            [1, 0, 1, 0], 
            [0, 1, 0, 1],
            [0, 0, 1, 0], #assuming constnat velocity
            [0, 0, 0, 1]]),#assuming constnat velocity
            dtype = float)

        self.C = np.ndarray((2,4), buffer=np.array([
            [1.0, 0.0, 0.0, 0.0], 
            [0.0, 1.0, 0.0, 0.0]]),
            dtype = float)

        self.Q = np.ndarray((4,4), buffer=np.array([
            [0.001, 0, 0, 0], 
            [0, 0.001, 0, 0],
            [0, 0, 0.1, 0], 
            [0, 0, 0, 0.1]]),
            dtype = float)

        self.R = np.ndarray((2,2), buffer=np.array([
            [0.1, 0], 
            [0, 0.1]]),
            dtype = float)

    def prediction_step(self):
        self.x_pri = np.matmul(self.A,self.x_post)
        self.P_pri = np.matmul(self.A, np.matmul(self.P_post,np.transpose(self.A)))+ self.Q

    def correction_step(self, y):

        temp1 = np.matmul(self.P_pri, np.transpose(self.C))
        temp2 = np.matmul(self.C, temp1)
        self.L = np.matmul(temp1,np.linalg.inv(temp2 + self.R))

        temp3 = np.matmul(self.C, self.x_pri)
        self.x_post = self.x_pri + np.matmul(self.L,(y - temp3))

        temp4 = (np.identity(len(self.x_pri))-np.matmul(self.L,self.C))
        temp5 = np.matmul(self.L, np.matmul(self.R, np.transpose(self.L)))
        self.P_post = np.matmul(temp4, np.matmul(self.P_pri,np.transpose(temp4))) + temp5


#-----------------------------------

def test_init():
    try: 
        tracker = Tracker()
    except:
        assert False

def test_prediction():

    try: 
        tracker = Tracker()
        tracker.prediction_step()
    except:
        assert False

def test_correction():
    y = np.ndarray((2,), buffer = np.array([1,2]), dtype = float)

    try: 
        tracker = Tracker()
        tracker.correction_step(y)
    except:
        assert False


def test_KF_highQ():
    """
    We simulate a baot standing still. 
    High variance in the distrubance. 
    Thus we expect estimates to be close to the measurments. 
    """

    r = 5
    theta = 0.3
    tollerance = 0.1
    n_mes = 5

    tracker = Tracker()

    for i in range (len(tracker.x_post)):
        tracker.Q[i,i] = 10

    for i in range(len(tracker.C)):
        tracker.R[i, i] = 0.1

    measurments = np.ndarray((n_mes,2), dtype = float)
    for i in range(n_mes):
        measurments[i, 0] = r
        measurments[i, 1] = theta

    for y in measurments:
        
        tracker.correction_step(y)

        tracker.prediction_step()

    print(tracker.x_post)

    assert abs(tracker.x_post[0] - r)     < tollerance  
    assert abs(tracker.x_post[1] - theta) < tollerance  
    assert abs(tracker.x_post[2])         < tollerance  
    assert abs(tracker.x_post[3])         < tollerance  

def test_KF_highR():
    """
    We simulate a baot standing still. 
    High variance in the measurments. 
    Thus we expect estimates to be close to initial values (when initial velocity is 0). 
    """

    r = 5
    theta = 0.3
    tollerance = 0.1
    n_mes = 5

    tracker = Tracker()

    
    tracker.x_pri[0] = r
    tracker.x_pri[1] = theta
    tracker.x_pri[2] = 0
    tracker.x_pri[3] = 0

    for i in range (len(tracker.x_post)):
        tracker.Q[i,i] = 0.001

    for i in range(len(tracker.C)):
        tracker.R[i, i] = 1

    measurments = np.ndarray((n_mes,2), dtype = float)
    for i in range(n_mes):
        measurments[i, 0] = r
        measurments[i, 1] = theta

    for y in measurments:
        
        tracker.correction_step(y)

        tracker.prediction_step()

    print(tracker.x_post)

    assert abs(tracker.x_post[0] - r)     < tollerance  
    assert abs(tracker.x_post[1] - theta) < tollerance  
    assert abs(tracker.x_post[2])         < tollerance  
    assert abs(tracker.x_post[3])         < tollerance  












