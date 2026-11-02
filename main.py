"""
main.py: runner for the robot path tracking simulator.

Runs three PID gain sets (under-damped, over-damped, tuned) on the same
planned path, shows each run as a pygame animation, and prints the result
of each run. Playback speed is set by fps.

Instructions:
cd robot_pid_project_Georgios_Mavropoulos_bi89qc/
pip -r install requirements.txt
Then ->  Run from the project root: python main.py

    
"""


from robot_pid.simulator import Simulator
from robot_pid.pid import PID


#helper method
def _run_sim_and_plot_results(heading_gains_list:tuple[list])->tuple[list]:

    """
    Executes simulation runs for multiple PID gain configurations and evaluates performance.

    This function iterates through a list of PID gain profiles. For each profile, it 
    initializes dedicated heading and speed PID controllers, executes a closed-loop 
    simulation, displays a live animation, prints a cumulative performance metrics table 
    to the terminal, and exports comparison plots and CSV logs.

    Args:
        heading_gains_list (list of tuples): A list where each element is a tuple 
            containing:
            - label (str): Unique identifier for the simulation run (e.g., 'tuned').
            - heading_gains (tuple): Proportional, Integral, and Derivative gains 
            (Kp, Ki, Kd) for the heading PID controller.
            - speed_gains (tuple): Proportional, Integral, and Derivative gains 
            (Kp, Ki, Kd) for the speed PID controller.

    Returns:
        list of tuples: A list of recorded simulation runs, where each tuple contains 
            `(label, sim_instance, simulation_data)`.
    """
    runs = []
    for label, heading_gains, speed_gains in heading_gains_list:
    
            #create heading and speed pid objects inside the loop
            heading_pid = PID(*heading_gains, output_limits=(-2.0, 2.0))
            speed_pid = PID(*speed_gains, output_limits=(0.0, 1.0))
            sim = Simulator(heading_pid, speed_pid, dt=0.1, threshold=0.1, t_max=60.0)
            data = sim.run_simulation()
            
            sim.animate(data, fps=60)  #close the window (or ESC) to go to the next one
            runs.append((label, sim, data))
    
    
            #print the results table
    header = (f"{'Run':<8}{'Reached':<9}{'Time [s]':<10}"
                        f"{'Path [m]':<10}{'CTE mean':<10}{'CTE max':<10}")
    print(header)
    print("-" * len(header))
    for label, sim, data in runs:
        m = sim.metrics(data)
        time_str = f"{m['time']:.1f}" if isinstance(m["time"], float) else "-"
        print(f"{label:<8}{str(data['reached_goal']):<9}{time_str:<10}"
        f"{m['path_length']:<10.2f}{m['cte_mean']:<10.3f}{m['cte_max']:<10.3f}")

    #export CSV and comparison plot
    sim.export_metrics_to_csv(runs)
    sim.plot_comparison(runs)

    return runs

def main():

    """
    Runs and animates the three gain sets, one after the other.

    For each gain set, the function builds the PID controllers, creates a
    Simulator (which builds the map and plans the A* path), runs the closed-loop
    simulation, prints reached/metrics, and opens the pygame animation. Close the
    window (or press ESC) to start the next gain set.
    Inside root you will find the plot pictures along with a csv file which contain the metrics. 
    You can also watch the metrics print on terminal after each run. 
    Be aware that upon saving each csv file and plot the previous one are being overwritten.
    """
   
    #run the three gain sets 
                                    #heading             #speed
    heading_gains_list = [("under", (12.0, 0.01, 0.02), (0.4, 0.1, 0.1)),
                          ("over", (4.0, 0.01, 1.5), (0.03, 0.01, 2.1)),
      
      
                          ("tuned", (7.0, 0.01, 0.4), (0.03, 0.01, 2.1))]

    
    #delegate the simulation execution, metrics tracking, 
    #plotting, and CSV exporting to the run_sim_and_plot_results function
    runs = _run_sim_and_plot_results(heading_gains_list)
    return runs

  

   



if __name__ == "__main__":
    main()