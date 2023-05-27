"""
| **@created on:** 03/15/22
| **@author:** Krutarth Trivedi | ktrivedi@wpi.edu
| **@version:** v1.0 | Initial Release
|
| **Description:**
|   RBE550 Motion Planning Coursework - HW3 - Valet
|   Differential Drive Robot Navigation
"""

from global_route_planner_delivery_robot import *

# dislpaying the animation
def environment(x,y,theta):
    plt.figure("Path Animation - The Delivery Robot")
    plt.title("Path Animation - The Delivery Robot")
    ax = plt.subplot(1,1,1)

    obstacle, car1, car2, agent, goal = environment_position() #Getting coordinates of all the static obstacles, start and goal position.

    #Putting everything in the simulation
    ax.add_patch(obstacle)
    ax.add_patch(car1)
    ax.add_patch(car2)
    ax.add_patch(agent)
    ax.add_patch(goal)

    plt.scatter(agent_goal[0],agent_goal[1])        #Plotting Goal position
    plt.plot(x,y, "sk")
    boundary = get_boundary(x,y,theta) # getting the vertices of the robot, for real-time animation

    X = []
    Y = []
    for x,y in boundary:
        X.append(x)
        Y.append(y)

    plt.plot(X,Y)
    plt.xlim([0, 200])
    plt.ylim([-20, 200])


#Main function
def main():
    #Find a global route from the start position to the goal position while considerigng the non-holonomic constraints and,
    #avoiding the static obstacles at the same time.

    path = A_star()
    print("Path found!")
    print("Start : ", agent_start)
    print("Goal : ", agent_goal)

    # animation of the final path
    for points in path:
        plt.cla()
        environment(points[0],points[1],points[2])
        plt.pause(0.00001)

    # ploting the path taken by the robot
    plt.figure("Path Depiction - The Delivery Robot")
    plt.title("Path Depiction - The Delivery Robot")
    plt.xlim([0, 200])
    plt.ylim([-20, 200])
    for points in path:
        plt.scatter(points[0],points[1],color = 'black',s=1) 
    plt.show()

if __name__ == "__main__":
    main()