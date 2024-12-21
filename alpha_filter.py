from system import System
class alphaFilter:

    #variables used for calculation
    def __init__(self, initial_estimate, state ):
        self.initial_estimate = initial_estimate  
        self.predicted_values = []
        self.measurements = []
        self.estimates = []
        #if the system is static then the initial prediction is the initial estimate of the state
        if state == System.STATIC:
            self.predicted_values.append(self.initial_estimate)  

    #display the class data on the console
    def showValue(self):
        print(self.initial_estimate)
        print(self.predicted_values)
        print(self.measurements)
        print(self.estimates)   

    def applyFilter(self, n):
        for i in range(n):
            measurement = int(input("Enter measurement: "))
            self.measurements.append(measurement)
            #formula for estimate of current state
            estimate = self.predicted_values[i] + ((1/(i+1)) * (measurement - self.predicted_values[i]))
            self.estimates.append(estimate)
            self.predicted_values.append(estimate)
            self.showValue()




     



