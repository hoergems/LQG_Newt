import os
import glob
import argparse
import numpy as np
import time

from libopenrave_interface import Environment, v_double, v2_double

class Play:
    def __init__(self, algorithm, dir, numb, play_failed):
        config_file = glob.glob(os.path.join(dir + "/config_*"))
        robot_filename = ""
        environment_filename = ""
        with open(config_file[0], "r") as f:
            for line in f.readlines():
                if "robot_file:" in line:
                    robot_filename = line.split(":")[1].strip()
                elif "environment_file:" in line:
                    environment_filename = line.split(":")[1].strip()
                    
        
                    
        
        robot_files = glob.glob(os.path.join(os.path.join(dir + "/model/", robot_filename)))
        print robot_files
        environment_files = glob.glob(os.path.join(os.path.join(dir + "/environment/", environment_filename)))
        
        
        
        if len(robot_files) > 1:
            print "Error: Multiple robot files found"
            return False
        if len(environment_files) > 1:
            print "Error: Multiple environment files found" 
            return False
        self.robot_file = robot_files[0]
        self.environment_file = environment_files[0]        
        
        if algorithm == None:
            print "No algorithm provided. Use the -a flag"
            return
        if dir == None:
            print "No directory provided. Use the -d flag"
            return
        elif not os.path.isdir(dir):
            print "Provided directory doesn't exist"
            return       
        if numb == None:
            numb = 0        
        if not self.init_environment():
            return
        print "PLAY"
        self.play_runs(dir, algorithm, numb, play_failed)
        
    def is_terminal(self, state):
        ja = v_double()
        ja[:] = [state[j] for j in xrange(len(state) / 2)]
        ee_position_arr = v_double()
        self.robot.getEndEffectorPosition(ja, ee_position_arr)
        ee_position = np.array([ee_position_arr[j] for j in xrange(len(ee_position_arr))])
        norm = np.linalg.norm(ee_position - self.goal_position)               
        if norm - 0.01 <= self.goal_radius:                       
            return True
        return False        
        
    def play_runs(self, dir, algorithm, numb, play_failed):      
        files = glob.glob(os.path.join(os.path.join(dir, "*.log")))
        print files
        log_files = []
        for file in files:
            file_str = file.split("/")[-1]
            if algorithm in file_str:
                log_files.append(file) 
        
        with open(sorted(log_files)[numb]) as f:
            states = []
            all_particles = []
            particles = []
            nominal_states = []
            col = []
            terminal = False
            for line in f:
                if "S: " in line:
                    line_arr = line.rstrip("\n ").split(" ")
                    state = np.array([float(line_arr[i]) for i in xrange(1, len(line_arr))])
                    states.append(state)                    
                elif "S_NOMINAL: " in line:
                    line_arr = line.rstrip("\n ").split(" ")                    
                    state = np.array([float(line_arr[i]) for i in xrange(1, len(line_arr))])
                    nominal_states.append(state)
                elif "Terminal:" in line:
                    term_line = line.rstrip("\n ").split(": ")[1]
                    if term_line == 'true':
                        terminal = True
                elif "collided: " in line or "Collision detected" in line:                    
                    if not "estimate" in line:
                        if "collided: " in line:
                            if line.rstrip("\n ").split(": ")[1] == "true":
                                col.append(True)
                            else:
                                col.append(False)
                        elif "Collision detected" in line:
                            if line.rstrip("\n").split(": ")[2] == "True":                                
                                col.append(True)
                            else:
                                col.append(False)
                elif "PARTICLES BEGIN" in line:
                    particles = []
                elif "PARTICLES END" in line:
                    all_particles.append(particles)
                elif "p: " in line:
                    line_arr = line.rstrip("\n").split(": ")[1].split(" ")
                    particle = np.array([float(line_arr[i]) for i in xrange(len(line_arr))])
                    particles.append(particle)                   
                elif "Final State:" in line:
                    line_arr = line.rstrip("\n ").split(": ")[1].split(" ")
                    state = np.array([float(line_arr[i]) for i in xrange(len(line_arr))])
                    states.append(state)
                    terminal = self.is_terminal(state)
                elif ("RUN #" in line or 
                      "Run #" in line or
                      "#####" in line) and len(states) != 0:
                    
                    if play_failed:
                        if terminal == False:
                            self.show_nominal_path(nominal_states)                            
                            self.play_states(states, col, all_particles)
                    else:
                        
                        self.show_nominal_path(nominal_states)                        
                        self.play_states(states, col, all_particles)
                    states = []
                    nominal_states = []
                    col = []
                    all_particles = []
                    
    def show_nominal_path(self, path):
        """ Shows the nominal path in the viewer """       
        self.environment.removePermanentParticles()
        particle_joint_values = v2_double()
        particle_joint_colors = v2_double()
        pjvs = []
        for p in path:
            particle = v_double()
            particle[:] = [p[i] for i in xrange(len(p) / 2)]
            pjvs.append(particle)
                
            color = v_double()
            color[:] = [0.0, 0.0, 0.0, 0.7]
            particle_joint_colors.append(color)            
        particle_joint_values[:] = pjvs
        self.environment.plotPermanentParticles(particle_joint_values,
                                                particle_joint_colors)
                    
    def play_states(self, states, col, particles):
        for i in xrange(len(states)):
            cjvals = v_double()
            cjvels = v_double()
            print "states[i] " + str(states[i])
            cjvals_arr = [states[i][j] for j in xrange(len(states[i]) / 2)]
            cjvels_arr = [states[i][j] for j in xrange(len(states[i]) / 2, len(states[i]))]
            cjvals[:] = cjvals_arr
            cjvels[:] = cjvels_arr            
            particle_joint_values = v2_double()
            particle_joint_colors = v2_double()
            
            if i > 1 and len(particles) > 0:                
                for p in particles[i-1]:
                    particle = v_double()
                    particle_color = v_double()
                    particle[:] = [p[k] for k in xrange(len(p) / 2)]
                    particle_color[:] = [0.5, 0.5, 0.5, 0.5]
                    particle_joint_values.append(particle)
                    particle_joint_colors.append(particle_color)
            print "updating robot values"
            self.environment.updateRobotValues(cjvals, 
                                               cjvels,
                                               particle_joint_values,
                                               particle_joint_colors)
            print "updated robot values"
            '''for o in self.obstacles:
                self.robot.setObstacleColor(o.getName(), 
                                            o.getStandardDiffuseColor(),
                                            o.getStandardAmbientColor()) '''
            self.environment.setKinBodiesDefaultColor()           
            try:
                if col[i] == True:                    
                    diffuse_col = v_double()
                    ambient_col = v_double()
                    diffuse_col[:] = [0.5, 0.0, 0.0, 0.0]
                    ambient_col[:] = [0.8, 0.0, 0.0, 0.0]
                    for o in self.obstacles:
                        self.robot.setObstacleColor(o.getName(), 
                                                    diffuse_col, 
                                                    ambient_col) 
            except:
                pass           
            time.sleep(0.3)
            
    def is_in_collision(self, previous_state, state):
        """
        Is the given end effector position in collision with an obstacle?
        """
        collidable_obstacles = [o for o in self.obstacles if not o.isTraversable()]
        joint_angles_goal = v_double()
        joint_angles_goal[:] = [state[i] for i in xrange(self.robot_dof)]
        print previous_state
        print state
        collision_objects_goal = self.robot.createRobotCollisionObjects(joint_angles_goal)        
        if len(previous_state) > 0:
            """
            Perform continuous collision checking if previous state is provided
            """            
            joint_angles_start = v_double()        
            joint_angles_start[:] = [previous_state[i] for i in xrange(self.robot_dof)]
            collision_objects_start = self.robot.createRobotCollisionObjects(joint_angles_start)
            
            for obstacle in collidable_obstacles:                
                for i in xrange(len(collision_objects_start)):                    
                    if obstacle.inCollisionContinuous([collision_objects_start[i], collision_objects_goal[i]]):
                        print "return true"
                        return True, obstacle
        
            return False, None
        else:            
            for obstacle in collidable_obstacles:
                if obstacle.inCollisionDiscrete(collision_objects_goal):                               
                    return True, None
            return False, None 
    
    def init_environment(self):
        self.environment = Environment()
        self.environment.setupEnvironment(self.environment_file)
        self.environment.loadRobotFromURDF(self.robot_file)
        self.robot = self.environment.getRobot()
        self.robot_dof = self.robot.getDOF()
        goal_area = v_double()        
        self.environment.getGoalArea(goal_area)
        self.goal_position = [goal_area[i] for i in xrange(0, 3)]
        self.goal_radius = goal_area[3]   
        self.environment.showViewer()     
        return True    
        
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='LQG-MP.')
    parser.add_argument("-a", "--algorithm", help="The algorithm to play")
    parser.add_argument("-d", "--directory", help="The directory of the logfiles")
    parser.add_argument("-n", "--numb", nargs='?', help="The number of covariance value to play", type=int, const=0)
    parser.add_argument("-f", "--play_failed", 
                        help="Play only the failed runs", 
                        action="store_true")
    args = parser.parse_args()
    Play(args.algorithm, args.directory, args.numb, args.play_failed)