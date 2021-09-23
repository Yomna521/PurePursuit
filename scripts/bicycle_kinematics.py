import numpy as np
import rospy 
from pure_pursuit.msg import input 
from pure_pursuit.msg import output
from time import sleep
    
class bicycle_model:
    def __init__(self,params,publisher):
        self.params= params
        self.pub  = publisher
        self.state = np.array([0,0,0], dtype='f')
        self.x,self.y =[],[]
        
    def updated_state(self,input):
        x_dot = input.vel*np.cos(self.state[2])
        y_dot = input.vel*np.sin(self.state[2])
        theta_dot = input.vel*np.tan(input.steer)/self.params['l']
        return np.array([x_dot,y_dot,theta_dot])
    
    def current_state(self,input):
        state = self.state + self.updated_state(input)*self.params['dt']
        state[2] = np.arctan2(np.sin(state[2] ), np.cos(state[2] ))
        self.state = state
	self.pub.publish(output(self.state[0],self.state[1],self.state[2],input.vel))
        rospy.loginfo(self.state.tolist())

    
def main():
    
    
if __name__ == '__main__':
    try:
        variables = {"dt":0.5, "l" : 2, "lr":1} 
    	rospy.init_node("simulation")
    	pub = rospy.Publisher("states",output,queue_size=10)
    	a = None
    	a = bicycle_model(variables,pub)

    	sleep(variables['dt'])
    	a.current_state(input(0,0))
    	rospy.Subscriber("simulation_inputs",input,a.current_state)
    	rospy.spin()

    except rospy.ROSInterruptException:
        pass