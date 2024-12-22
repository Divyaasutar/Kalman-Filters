from system import System
class alphaBeta:

    #step 1: initialize the variables used for calculation
    def __init__(self, initial_position, initial_velocity, state):
        self.initial_position = initial_position
        self.initial_guess_velocity = initial_velocity
        self.initial_guess_position = initial_position + 5 * initial_velocity
        self.predictedValue_position = []
        self.predictedValue_velocity = []
        self.measurements = []
        self.current_estValue_position = []
        self.current_estValue_velocity = []

        
        #if system is dynamic then the initial guess is the prior estimate
        if state == System.DYNAMIC:
           self.predictedValue_position.append(self.initial_guess_position) 
           self.predictedValue_velocity.append(self.initial_guess_velocity)
           #alpha and beta values are kalman constants. For high precision-measurements, we should choose high α and β
           #assuming low precision equipment
           self.alpha = 0.2
           self.beta = 0.1

    #show the values 
    def showvalue(self):
        print(self.initial_position)
        print(self.initial_guess_velocity)
        print(self.initial_guess_position)
        #print(self.time)
        print(self.measurements)
        print(self.predictedValue_position)       
        print(self.predictedValue_velocity)
        print(self.current_estValue_position)
        print(self.current_estValue_velocity)
        

    #mathematical model of the system 
    def applyFilter2(self,n):
        for i in range(n):
            #step 1: The radar measures the aircraft range
            measurement = int(input("Enter measurement: ")) 
            self.measurements.append(measurement)  
            #step 2: Calculating the current estimate using the State Update Equation
            current_estValue_position = (self.predictedValue_position[i] + (self.alpha) * (measurement - self.predictedValue_position[i]))
            current_estValue_velocity = (self.predictedValue_velocity[i] + (self.beta) * ((measurement - self.predictedValue_position[i])/5))
            self.current_estValue_position.append(current_estValue_position)
            self.current_estValue_velocity.append(current_estValue_velocity)
            self.predictedValue_position.append(current_estValue_position)
            self.predictedValue_velocity.append(current_estValue_velocity)
            self.showvalue()

            


        


        
aircraft = alphaBeta(30000,40,System.DYNAMIC)
aircraft.applyFilter2(10)

        
