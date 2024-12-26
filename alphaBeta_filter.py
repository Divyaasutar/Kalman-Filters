from system import System
class alphaBeta:

    #step 1: initialize the variables used for calculation
    def __init__(self, initial_position, initial_velocity, state):
        self.initial_position = initial_position
        self.initial_velocity = initial_velocity
        
        #initial guess should be extrapolated to the first cycle using the State Extrapolation Equations:
        self.predictedPosition = self.initial_position + 5 * self.initial_velocity
        self.predictedVelocity = initial_velocity
        
        #outputs for the plot
        self.measurements =[]
        self.prior_estimate_position = [self.predictedPosition]
        self.prior_estimate_velocity = [self.predictedVelocity]
        self.current_estValue_position = []
        self.current_estValue_velocity = []
        
        if state == System.DYNAMIC:
            #the alpha beta constants 
            self.alpha = 0.2
            self.beta = 0.1
        
    def showvalue(self):
            print(self.initial_position)
            print(self.initial_velocity)
            print(self.measurements)
            print(self.current_estValue_position)
            print(self.current_estValue_velocity)
            print(self.prior_estimate_position)
            print(self.prior_estimate_velocity)
        
    def applyfilter(self, n):
        for i in range(n):
            
            #the initial guess is the prior estimate:
            prior_estimate_position = self.prior_estimate_position[i]
            prior_estimate_velocity = self.prior_estimate_velocity[i]
            
            #measurement input from the radar
            measurement = int(input("Enter measurement: "))
            self.measurements.append(measurement)
            
            #Calculating the current estimate using the State Update Equation: 
            current_estValue_position = (prior_estimate_position + (self.alpha) * (measurement - prior_estimate_position))
            current_estValue_velocity = (prior_estimate_velocity + (self.beta) * ((measurement - prior_estimate_position)/5))
            self.current_estValue_position.append(current_estValue_position)
            self.current_estValue_velocity.append(current_estValue_velocity)
            
            #next state estimate using state extrapolation equation
            next_estimate_position = current_estValue_position + 5 * current_estValue_velocity
            next_estimate_velocity = current_estValue_velocity
            
            self.prior_estimate_position.append(next_estimate_position)
            self.prior_estimate_velocity.append(next_estimate_velocity)
            
            self.showvalue()
            


        
