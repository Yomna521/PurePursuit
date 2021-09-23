import numpy as np
import rospy 
from pure_pursuit.msg import states
from pure_pursuit.msg import output

path = []
k = 1
lfc = 3.0
num_points = 100
dt = 0.05
def generate_path():
    for i in np.linspace(0,num_points,1000):
        y = 2*np.sin(0.1*i)*np.exp(0.01*i)
        path.append(np.array([i,y]))

def getDistance(p1,p2):
    return np.hypot(p1[0]-p2[0],p1[1]-p2[1])

def getNearestIdx(p):
    temp = path - p
    dis = np.hypot(temp[:,0],temp[:,1])
    return np.argmin(dis)    
        
def get_lookahead(cur,ld):
    idx = getNearestIdx(cur)
    
    while ld > getDistance(path[idx],cur):
        
        if idx +1 >=len(path):
            break
        idx = idx+1
    return path[idx]

curr_target = states(0,0,0,0,0,0)

def lookahead_callback(msg):
    global curr_target
    state = np.array([msg.x,msg.y])
    ld = msg.vel * k + lfc
    lookahead = get_lookahead(state,ld)
    curr_target = states(msg.x,msg.y,msg.theta,lookahead[0],lookahead[1],5)
      
if __name__ == '__main__':
    try:
    	global curr_target
    	generate_path()
    	rospy.init_node("generator")
    	pub = rospy.Publisher("targets",states,queue_size=10)
    	rospy.Subscriber("states",output,lookahead_callback)
    	rate = rospy.Rate(0.5)
    	while not rospy.is_shutdown():
        	pub.publish(curr_target)
        	rate.sleep()

    except rospy.ROSInterruptException:
        pass