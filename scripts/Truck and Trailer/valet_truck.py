"""
| **@created on:** 03/15/22
| **@author:** Krutarth Trivedi | ktrivedi@wpi.edu
| **@version:** v1.0 | Initial Release
|
| **Description:**
|   RBE550 Motion Planning Coursework - HW3 - Valet
|   Truck with a trailer - Navigation using Ackerman Geometry for truck and trailer(s) geometry
"""

from global_route_planner_truck import *

def environment(x,y,theta,xt,yt,theta_t):
    plt.figure("Path Animation - Truck with a trailer")
    plt.title("Path Animation - Truck with a trailer")
    ax = plt.subplot(1,1,1)

    obstacle, car1, agent, agent_trailer, goal  = environment_position()  #Getting coordinates of all the static obstacles, start and goal position.

    #Putting everything in the simulation
    ax.add_patch(obstacle)
    ax.add_patch(car1)
    ax.add_patch(agent)
    ax.add_patch(agent_trailer)
    ax.add_patch(goal)
    plt.scatter(agent_goal[0],agent_goal[1])        #Plotting Goal position
    plt.plot(x,y,"sk")
    boundary = get_boundary(x,y,theta)  # get the boundary of truck, for real-time simulation
    boundary_t = get_boundary(xt,yt,theta_t,"trailer") # get the boundary of trailer, for real-time simulation

    X = []
    Y = []
    for x,y in boundary:
        X.append(x)
        Y.append(y)
    plt.plot(X,Y,color = 'blue')

    X = []
    Y = []
    plt.plot(X,Y)
    for x,y in boundary_t:
        X.append(x)
        Y.append(y)

    plt.plot(X,Y,color = 'blue')
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
        environment(points[0],points[1],points[2],points[3],points[4],points[5])
        plt.pause(0.00001)

    # ploting the axle center path
    plt.figure("Path Depiction - Truck with a trailer")
    plt.title("Path Depiction - Truck with a trailer")
    plt.xlim([0, 200])
    plt.ylim([-20, 200])
    for points in path:
        plt.scatter(points[0],points[1],color = 'black',s=1) 
    plt.show()

if __name__ == "__main__":
    main()