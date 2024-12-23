import random
from matrix import Matrix
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class MultiVariateFilter:
    
    def __init__(self, accX, accY, accZ, velX, velY, velZ, disX, disY, disZ):
        self.state = [[disX], [disY], [disZ], [velX], [velY], [velZ], [accX], [accY], [accZ]]

        # Initial covariance matrix (9x9 identity matrix)
        self.P = [[1 if i == j else 0 for j in range(9)] for i in range(9)]
        
        # Process noise covariance matrix (9x9 scaled identity)
        self.Q = [[0.1 if i == j else 0 for j in range(9)] for i in range(9)]

        self.F = Matrix.get_zeros_matrix(9, 9)

    def updateState(self, delta_t):
        self.F = [
            [1, 0, 0, delta_t, 0, 0, 0.5 * delta_t**2, 0, 0],
            [0, 1, 0, 0, delta_t, 0, 0, 0.5 * delta_t**2, 0],
            [0, 0, 1, 0, 0, delta_t, 0, 0, 0.5 * delta_t**2],
            [0, 0, 0, 1, 0, 0, delta_t, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, delta_t, 0],
            [0, 0, 0, 0, 0, 1, 0, 0, delta_t],
            [0, 0, 0, 0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 1],
        ]

    def predict(self, delta_t):
        # Update F based on delta_t
        self.updateState(delta_t)

        # Predict the next state: X' = F * X
        self.state = Matrix.multiply_matrices(self.F, self.state)

        # Predict the next covariance: P' = F * P * F^T + Q
        F_transpose = Matrix.transpose_matrix(self.F)
        FP = Matrix.multiply_matrices(self.F, self.P)
        self.P = Matrix.add_matrix(Matrix.multiply_matrices(FP, F_transpose), self.Q)


    def update(self, measurement, H, R):
        # Calculate Kalman gain: K = P * H^T * (H * P * H^T + R)^-1
        H_transpose = Matrix.transpose_matrix(H)
        HP = Matrix.multiply_matrices(H, self.P)
        HPHt = Matrix.multiply_matrices(HP, H_transpose)
        S = Matrix.add_matrix(HPHt, R)
        S_inv = [[1 / S[0][0]]]  # Inverse of scalar (assume R is 1x1 for simplicity)
        K = Matrix.multiply_matrices(Matrix.multiply_matrices(self.P, H_transpose), S_inv)

        # Update state: X = X + K * (measurement - H * X)
        HX = Matrix.multiply_matrices(H, self.state)
        y = [[measurement[i][0] - HX[i][0]] for i in range(len(measurement))]
        self.state = Matrix.add_matrix(self.state, Matrix.multiply_matrices(K, y))

        # Update covariance: P = (I - K * H) * P
        I = [[1 if i == j else 0 for j in range(len(self.P))] for i in range(len(self.P))]
        KH = Matrix.multiply_matrices(K, H)
        I_KH = Matrix.add_matrix(I, [[-KH[i][j] for j in range(len(KH[0]))] for i in range(len(KH))])
        self.P = Matrix.multiply_matrices(I_KH, self.P)

    def simulate(displacement_inputs, delta_t=1):
        """
        Simulates the motion using displacement inputs for each time step.
        
        :param displacement_inputs: List of displacement vectors for each time step. Example:
                                    [[40, 1000, 20], [45, 1010, 25], ...]
        :param delta_t: Time step (default is 1 second)
        """
        total_time = len(displacement_inputs)
        
        # Lists to store results
        actual_states = []
        measurements = []
        estimated_states = []

        # Initialize with the first displacement
        initial_displacement = displacement_inputs[0]
        aircraft = MultiVariateFilter(
            accX=0, accY=0, accZ=0,
            velX=0, velY=0, velZ=0,
            disX=initial_displacement[0],
            disY=initial_displacement[1],
            disZ=initial_displacement[2],
        )

        for t in range(total_time):
            current_displacement = displacement_inputs[t]
            
            # Calculate velocity
            if t == 0:
                velocity = [0, 0, 0]  # Initial velocity is assumed to be 0
            else:
                previous_displacement = displacement_inputs[t - 1]
                velocity = [
                    (current_displacement[i] - previous_displacement[i]) / delta_t
                    for i in range(3)
                ]
            
            # Calculate acceleration
            if t <= 1:
                acceleration = [0, 0, 0]  # Assume no acceleration initially
            else:
                previous_velocity = [
                    (displacement_inputs[t - 1][i] - displacement_inputs[t - 2][i]) / delta_t
                    for i in range(3)
                ]
                acceleration = [
                    (velocity[i] - previous_velocity[i]) / delta_t for i in range(3)
                ]

            # Simulate the actual state
            aircraft.state[0][0] = current_displacement[0]  # Update displacement X
            aircraft.state[1][0] = current_displacement[1]  # Update displacement Y
            aircraft.state[2][0] = current_displacement[2]  # Update displacement Z
            aircraft.state[3][0] = velocity[0]  # Update velocity X
            aircraft.state[4][0] = velocity[1]  # Update velocity Y
            aircraft.state[5][0] = velocity[2]  # Update velocity Z
            aircraft.state[6][0] = acceleration[0]  # Update acceleration X
            aircraft.state[7][0] = acceleration[1]  # Update acceleration Y
            aircraft.state[8][0] = acceleration[2]  # Update acceleration Z
            actual_state = [row[0] for row in aircraft.state]
            actual_states.append(actual_state)

            # Predict the next state using the Kalman filter
            aircraft.predict(delta_t)

            # Simulate radar measurement with noise
            measurement = [
                [current_displacement[0] + random.gauss(0, 2)],
                [current_displacement[1] + random.gauss(0, 2)],
                [current_displacement[2] + random.gauss(0, 2)],
            ]
            measurements.append([m[0] for m in measurement])

            # Apply Kalman filter update
            H = [
                [1, 0, 0, 0, 0, 0, 0, 0, 0],  # Observing displacement X
                [0, 1, 0, 0, 0, 0, 0, 0, 0],  # Observing displacement Y
                [0, 0, 1, 0, 0, 0, 0, 0, 0],  # Observing displacement Z
            ]
            R = [[100, 0, 0], [0, 100, 0], [0, 0, 100]]  # Measurement noise covariance (diagonal)
            aircraft.update(measurement, H, R)

            # Store estimated state
            estimated_states.append([aircraft.state[i][0] for i in range(3)])  # Only X, Y, Z

        return actual_states, measurements, estimated_states



    def plot_results(actual_states, measurements, estimated_states):

        

        actual_x = [state[0] for state in actual_states]
        actual_y = [state[1] for state in actual_states]
        actual_z = [state[2] for state in actual_states]

        measured_x = [m[0] for m in measurements]
        measured_y = [m[1] for m in measurements]
        measured_z = [m[2] for m in measurements]

        estimated_x = [state[0] for state in estimated_states]
        estimated_y = [state[1] for state in estimated_states]
        estimated_z = [state[2] for state in estimated_states]

        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')

        # Plot actual trajectory
        ax.plot(actual_x, actual_y, actual_z, label="Actual Trajectory", color="blue", linewidth=2)
        ax.scatter(actual_x, actual_y, actual_z, color="blue", marker="o", label="Actual Points")

        # Plot measured trajectory
        ax.plot(measured_x, measured_y, measured_z, label="Measured Trajectory", color="orange", linestyle="dotted", linewidth=2)
        ax.scatter(measured_x, measured_y, measured_z, color="orange", marker="x", label="Measured Points")

        # Plot estimated trajectory
        ax.plot(estimated_x, estimated_y, estimated_z, label="Estimated Trajectory", color="green", linestyle="dashed", linewidth=2)
        ax.scatter(estimated_x, estimated_y, estimated_z, color="green", marker="^", label="Estimated Points")

        # Labels and legend
        ax.set_title("3D Displacement: Actual vs Measured vs Estimated", fontsize=14)
        ax.set_xlabel("Displacement X", fontsize=12)
        ax.set_ylabel("Displacement Y", fontsize=12)
        ax.set_zlabel("Displacement Z", fontsize=12)
        ax.legend()
        ax.grid(True)

        plt.show()


        # print("------------------------------------------------------------------------------------------------------")
        # print("Measured Displacement \t\t Estimated Displacement \t Actual Displacement")
        # print("------------------------------------------------------------------------------------------------------")
        # sums = [0,0]
        # for i in range(30):
        #     print(measurements[i][0],"\t\t",estimated_states[i][0],"\t\t",actual_states[i][0])
        #     sums[0] += actual_states[i][0] - measurements[i][0]
        #     sums[1] += actual_states[i][0] - estimated_states[i][0]
        # print("------------------------------------------------------------------------------------------------------")
        # print(sums[0]/30,"\t\t",sums[1]/30)
        # print("------------------------------------------------------------------------------------------------------")
