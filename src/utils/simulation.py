import numpy as np
import matplotlib.pyplot as plt


class sim_mobile_robot: # Visualizer on 2D plot

    # INITIALIZATION, run only once in the beginning
    #-----------------------------------------------------------------
    def __init__(self, mode = 'unicycle'):
        # Generate the simulation window and plot initial objects
        self.fig = plt.figure(1)
        self.ax = plt.gca()
        self.ax.set(xlabel="x [m]", ylabel="y [m]")
        self.ax.set_aspect('equal', adjustable='box', anchor='C')
        plt.tight_layout()
        # Plot initial value for trajectory and time stamp
        self.traj_pl, = self.ax.plot(0,0, 'b--')
        # Store the plot option (use icon or not)
        self.draw_with_mobile_robot_icon = True
        if mode == 'omnidirectional': self.icon_id = 3
        elif mode == 'unicycle': self.icon_id = 2
        else: self.draw_with_mobile_robot_icon = False
        # Draw current robot position    
        if self.draw_with_mobile_robot_icon: # use mobile robot icon
            self.moro_patch = None
            self.draw_icon( np.zeros(3) )
        else: # use simple x marker
            self.pos_pl, = self.ax.plot(0,0, 'b', marker='X', markersize=10)

    def set_field( self, x_axis_range_tuple, y_axis_range_tuple ):
        # set the plot limit with the given range
        self.ax.set( xlim=x_axis_range_tuple, ylim = y_axis_range_tuple)

    def show_goal( self, goal_state):
        # Draw the goal state as an arrow 
        # the arrow tail denotes the goal position 
        # the arrow direction denotes the goal direction / angle
        arrow_size = 0.2
        ar_d = [arrow_size*np.cos(goal_state[2]), arrow_size*np.sin(goal_state[2])]
        self.pl_goal = plt.quiver( goal_state[0], goal_state[1], ar_d[0], ar_d[1], 
            scale_units='xy', scale=1, color='r', width=0.1*arrow_size)

    def plot_obs(self, obstacles, radious):
        for obs in obstacles:
            circle = plt.Circle((obs[0], obs[1]), radious)
            self.ax.add_artist(circle)

    # PLOT UPDATES, run in every iterations
    #-----------------------------------------------------------------

    def update_trajectory(self, robot_state): # update robot status
        # Update the simulation with the new data
        if self.draw_with_mobile_robot_icon: # use wheeled robot icon
            self.draw_icon( robot_state )
        else: # update the x marker
            self.pos_pl.set_data(robot_state[0], robot_state[1]) # plot only last position
        # Pause to show the movement  
 

    def update_point(self, x, y, prev_coords):
        self.ax.scatter(x, y)
        self.ax.plot([prev_coords[0], x], [prev_coords[1], y])
        plt.pause(0.001)

    # OPTIONAL PLOT, not necessary but provide nice view in simulation
    #-----------------------------------------------------------------
    def draw_icon(self, robot_state): # draw mobile robot as an icon
        # Extract data for plotting
        px = robot_state[0]
        py = robot_state[1]
        th = robot_state[2]
        # Basic size parameter
        scale = 2
        body_rad = 0.08 * scale # m
        wheel_size = [0.1*scale, 0.02*scale] 
        arrow_size = body_rad
        # left and right wheels anchor position (bottom-left of rectangle)
        if self.icon_id == 2: thWh = [th+0., th+np.pi] # unicycle
        else: thWh = [ (th + i*(2*np.pi/3) - np.pi/2) for i in range(3)] # default to omnidirectional
        thWh_deg = [np.rad2deg(i) for i in thWh]
        wh_x = [ px - body_rad*np.sin(i) - (wheel_size[0]/2)*np.cos(i) + (wheel_size[1]/2)*np.sin(i) for i in thWh ]
        wh_y = [ py + body_rad*np.cos(i) - (wheel_size[0]/2)*np.sin(i) - (wheel_size[1]/2)*np.cos(i) for i in thWh ]
        # Arrow orientation anchor position
        ar_st= [px, py] #[ px - (arrow_size/2)*np.cos(th), py - (arrow_size/2)*np.sin(th) ]
        ar_d = (arrow_size*np.cos(th), arrow_size*np.sin(th))
        # initialized unicycle icon at the center with theta = 0
        if self.moro_patch is None: # first time drawing
            self.moro_patch = [None]*(2+len(thWh))
            self.moro_patch[0] = self.ax.add_patch( plt.Circle( (px, py), body_rad, color='#AAAAAAAA') )
            self.moro_patch[1] = plt.quiver( ar_st[0], ar_st[1], ar_d[0], ar_d[1], 
                scale_units='xy', scale=1, color='b', width=0.1*arrow_size)
            for i in range( len(thWh) ):
                self.moro_patch[2+i] = self.ax.add_patch( plt.Rectangle( (wh_x[i], wh_y[i]), 
                    wheel_size[0], wheel_size[1], angle=thWh_deg[i], color='k') )

        else: # update existing patch
            self.moro_patch[0].set( center=(px, py) )
            self.moro_patch[1].set_offsets( ar_st )
            self.moro_patch[1].set_UVC( ar_d[0], ar_d[1] )
            for i in range( len(thWh) ):
                self.moro_patch[2+i].set( xy=(wh_x[i], wh_y[i]) )
                self.moro_patch[2+i].angle = thWh_deg[i]

    