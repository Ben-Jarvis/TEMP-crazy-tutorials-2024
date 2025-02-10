from a_star_3D import AStar3D
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

class MotionPlanner3D():
    
    #Question: SIMON PID, what is vel_max set for PID? Check should be same here
    def __init__(self, path, times):
        self.path = path
        self.times = times
        self.seg_times = np.diff(times)
        self.m = len(self.path)

        self.v_0 = np.zeros(3)
        self.a_0 = np.zeros(3)
        self.v_f = np.zeros(3)
        self.a_f = np.zeros(3)

    def compute_poly_matrix(self, t):
        # Inputs:
        # - t: The total time in which the current path segment should be completed
        # Outputs: 
        # - The polynomial coefficient matrix "A"
        # The "A" matrix is used to solve the constraint_vector = A * coefficient_vector system of equations
        poly_deriv_matrix = np.array([
            [t**5, t**4, t**3, t**2, t, 1], #pos
            [5*(t**4), 4*(t**3), 3*(t**2), 2*t, 1, 0], #vel
            [20*(t**3), 12*(t**2), 6*t, 2, 0, 0], #acc  
            [60*(t**2), 24*t, 6, 0, 0, 0], #snap
            [120*t, 24, 0, 0, 0, 0] #jerk
        ])

        return poly_deriv_matrix

    def compute_poly_coefficients(self):
        
        poly_coeffs = []

        # """
        # Compute a minimum jerk trajectory given time and position waypoints.

        for dim in range(3):  # Compute for x, y, and z separately
            A = np.zeros((6*(self.m-1), 6*(self.m-1)))
            b = np.zeros(6*(self.m-1))
            pos = np.array([p[dim] for p in self.path])
            A_0 = self.compute_poly_matrix(0)

            row = 0
            for i in range(self.m-1):
                pos_0 = pos[i]
                pos_f = pos[i+1]
                A_f = self.compute_poly_matrix(self.seg_times[i])
                if i == 0:
                    A[row, i*6:(i+1)*6] = A_0[0] #Initial position constraint
                    b[row] = pos_0
                    row += 1
                    A[row, i*6:(i+1)*6] = A_f[0] #Final position constraint
                    b[row] = pos_f
                    row += 1
                    A[row, i*6:(i+1)*6] = A_0[1] #Initial velocity constraint
                    b[row] = self.v_0[dim]
                    row += 1
                    A[row, i*6:(i+1)*6] = A_0[2] #Initial acceleration constraint
                    b[row] = self.a_0[dim]
                    row += 1
                    #Continuity of velocity, acceleration, snap and jerk
                    A[row:row+4, i*6:(i+1)*6] = A_f[1:]
                    A[row:row+4, (i+1)*6:(i+2)*6] = -A_0[1:]
                    b[row:row+4] = np.zeros(4)
                    row += 4
                elif i == self.m - 2:
                    A[row, i*6:(i+1)*6] = A_0[0] #Initial position constraint
                    b[row] = pos_0
                    row += 1
                    A[row, i*6:(i+1)*6] = A_f[0] #Final position constraint
                    b[row] = pos_f
                    row += 1
                    A[row, i*6:(i+1)*6] = A_f[1] #Final velocity constraint
                    b[row] = self.v_f[dim]
                    row += 1
                    A[row, i*6:(i+1)*6] = A_f[2] #Final acceleration constraint
                    b[row] = self.a_f[dim]
                    row += 1
                elif i < self.m - 2:
                    A[row, i*6:(i+1)*6] = A_0[0] #Initial position constraint
                    b[row] = pos_0
                    row += 1
                    A[row, i*6:(i+1)*6] = A_f[0] #Final position constraint
                    b[row] = pos_f
                    row += 1
                    #Continuity of velocity, acceleration, snap and jerk
                    A[row:row+4, i*6:(i+1)*6] = A_f[1:]
                    A[row:row+4, (i+1)*6:(i+2)*6] = -A_0[1:]
                    b[row:row+4] = np.zeros(4)
                    row += 4
                    
            poly_coeffs.append(np.linalg.solve(A, b))
        
        poly_coeffs = np.array(poly_coeffs).T
    
        return poly_coeffs
    
    def vel_check(self, poly_coeffs):
        # Find candidates where acceleration is zero and where the velocity is maximum for each segment
        

        # Perform check if velocity is greater than allowed limit
        # Return error
        pass
    
    def plot_obstacle(self, ax, x, y, z, dx, dy, dz, color='gray', alpha=0.3):
        """Plot a rectangular cuboid (obstacle) in 3D space."""
        vertices = np.array([[x, y, z], [x+dx, y, z], [x+dx, y+dy, z], [x, y+dy, z],
                            [x, y, z+dz], [x+dx, y, z+dz], [x+dx, y+dy, z+dz], [x, y+dy, z+dz]])
        
        faces = [[vertices[j] for j in [0, 1, 2, 3]], [vertices[j] for j in [4, 5, 6, 7]], 
                [vertices[j] for j in [0, 1, 5, 4]], [vertices[j] for j in [2, 3, 7, 6]], 
                [vertices[j] for j in [0, 3, 7, 4]], [vertices[j] for j in [1, 2, 6, 5]]]
        
        ax.add_collection3d(Poly3DCollection(faces, color=color, alpha=alpha))
    
    def evaluate_and_plot(self, poly_coeffs, obs, num_points = 100):

        t_fine = np.linspace(self.times[0], self.times[-1], num_points)  # Fine time intervals
        x_vals, y_vals, z_vals = [], [], []
        v_x_vals, v_y_vals, v_z_vals = [], [], []
        coeff_x = poly_coeffs[:,0]
        coeff_y = poly_coeffs[:,1]
        coeff_z = poly_coeffs[:,2]
        # print(coeff_x)

        for t in t_fine:
            # Find which segment this time belongs to
            segment = min(max(np.searchsorted(self.times, t)-1,0), len(coeff_x) - 1)
            x_vals.append(np.dot(self.compute_poly_matrix(t-self.times[segment])[0],coeff_x[segment*6:(segment+1)*6]))
            y_vals.append(np.dot(self.compute_poly_matrix(t-self.times[segment])[0],coeff_y[segment*6:(segment+1)*6]))
            z_vals.append(np.dot(self.compute_poly_matrix(t-self.times[segment])[0],coeff_z[segment*6:(segment+1)*6]))
            v_x_vals.append(np.dot(self.compute_poly_matrix(t-self.times[segment])[1],coeff_x[segment*6:(segment+1)*6]))
            v_y_vals.append(np.dot(self.compute_poly_matrix(t-self.times[segment])[1],coeff_y[segment*6:(segment+1)*6]))
            v_z_vals.append(np.dot(self.compute_poly_matrix(t-self.times[segment])[1],coeff_z[segment*6:(segment+1)*6]))

        # print(x_vals)
        # print(y_vals)
        # print(z_vals)

        # Plot 3D trajectory
        fig = plt.figure(figsize=(8, 6))
        ax = fig.add_subplot(111, projection='3d', )

        for ob in obs:
            self.plot_obstacle(ax, ob[0], ob[1], ob[2], ob[3], ob[4], ob[5])

        ax.plot(x_vals, y_vals, z_vals, label="Minimum-Jerk Trajectory", linewidth=2)
        ax.set_xlim(0, 5)
        ax.set_ylim(0, 3)
        ax.set_zlim(0, 1.5)

        # Plot waypoints
        waypoints_x = [p[0] for p in self.path]
        waypoints_y = [p[1] for p in self.path]
        waypoints_z = [p[2] for p in self.path]
        ax.scatter(waypoints_x, waypoints_y, waypoints_z, color='red', marker='o', label="Waypoints")

        # Labels and legend
        ax.set_xlabel("X Position")
        ax.set_ylabel("Y Position")
        ax.set_zlabel("Z Position")
        ax.set_title("3D Minimum-Jerk Trajectory")
        ax.legend()
        plt.show()

        return x_vals, y_vals, z_vals, v_x_vals, v_y_vals, v_z_vals
    
if __name__ == '__main__':
    start = (0.0, 0.0, 0.0)
    goal = (5, 1, 1)
    grid_size = 0.5
    obstacles = [(1.25, 1.25, 0.0, 0.5, 0.5, 2.0),
                 (1.5, 2.25, 0.0, 0.5, 0.5, 2.0),
                 (3.5, 1.35, 0.0, 0.25, 0.25, 1.5),
                 (4.25, 2.25, 0.0, 0.5, 0.5, 1.5),
                 (0.8, 0.3, 0.0, 0.5, 0.5, 1.5),
                 (3.75, 0.4, 0.0, 0.5, 0.5, 1.5),
                 (2.5, 0.0, 0.0, 0.5, 2.0, 1.5),
                 (2.5, 2.0, 0.0, 0.5, 1.0, 0.25),
                 (2.5, 2.0, 0.75, 0.5, 1.0, 0.75)
                 ]  # (x, y, z, width_x, width_y, width_z)
    bounds = (0, 5, 0, 5, 0, 1.5)  # (x_min, x_max, y_min, y_max, z_min, z_max)

    diagonal_flag = True
    astar = AStar3D(start, goal, grid_size, obstacles, bounds, diagonal_flag)
    path = astar.find_path()
    print("Path:", path)

    path_np = np.array([np.asarray(p) for p in path])
    dists = np.linalg.norm(np.diff(path_np, axis=0), axis=1)
    dist_ratios = (dists/np.sum(dists))

    t_f = 5.0

    # IMPROVEMENT to allow for more even speed distribution (less speed exceeding peaks)
    #times = [0]
    #[times.append(np.sum(dist_ratios[:i+1])*t_f) for i in range(len(dist_ratios))]# (dists/np.sum(dists))*t_f 
    times = np.linspace(0, t_f, len(path))
    print(times)

    mp = MotionPlanner3D(path,times)

    poly_coeffs = mp.compute_poly_coefficients()

    x,y,z,vx,vy,vz = np.array(mp.evaluate_and_plot(poly_coeffs, obstacles))

    print("Maximum speed: " + str(np.max(np.sqrt(vx**2 + vy**2 + vz**2))))
    print("Average speed: " + str(np.mean(np.sqrt(vx**2 + vy**2 + vz**2))))