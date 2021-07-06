import time

class pid:
    def __init__(self, P, I, D):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.sample_time = 0.00
        self.current_time = time.time()
        self.last_time = self.current_time
        self.parameterInit()

    def setoutputMax(self,outputMax):
        self.setoutputMax(outputMax)

    def setP(self, P):
        self.Kp = P;

    def setSampleTime(self, sample_time):
        self.sample_time = sample_time

    def parameterInit(self):
        self.target = 0.0
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0
        self.output = 0.0
        self.outputMax = 0.08
        
    def update(self, feedback):
        error = self.target - feedback
        self.current_time = time.time()
        dt = self.current_time - self.last_time
        derror = error - self.last_error
        if (dt >= self.sample_time):
            self.PTerm = self.Kp * error
            self.ITerm += error * dt
            self.DTerm = 0.0
            if dt > 0:
                self.DTerm = derror / dt
            self.last_time = self.current_time
            self.last_error = error
            output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)
            if(output > self.outputMax):
                self.output = self.setoutputMax()
            else:
                self.output = output

