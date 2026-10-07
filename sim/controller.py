"""Map-based pure pursuit and speed control, independent of the simulator."""
import math
import numpy as np


def district_route():
    pieces=[]
    def line(a,b):
        pieces.extend(np.linspace(a,b,max(2,int(np.linalg.norm(np.subtract(b,a))/.25)),endpoint=False))
    def arc(center,radius,a,b):
        for t in np.linspace(a,b,max(2,int(abs(b-a)*radius/.25)),endpoint=False):
            pieces.append(np.array(center)+radius*np.array([math.cos(t),math.sin(t)]))
    # With Y up and +Z forward, the driver's right is -X.
    # +Z traffic uses the west half of each road; +X traffic uses its north half.
    line((-23.5,-30),(-23.5,-10))
    arc((-11,-10),12.5,math.pi,math.pi/2)
    line((-11,2.5),(5,2.5))
    # Ease toward the lane's inner edge before the tighter right turn.
    # The 6.5 m radius and approach offset leave room for the whole body.
    for t in np.linspace(0,1,30,endpoint=False):
        pieces.append(np.array([5+7*t,2.5-(3*t*t-2*t*t*t)]))
    arc((12,8),6.5,-math.pi/2,0)
    line((18.5,8),(18.5,30))
    pieces.append(np.array([18.5,30]))
    return np.asarray(pieces)


class RouteController:
    def __init__(self,target_speed=3.5,route=None):
        self.route=district_route() if route is None else np.asarray(route,dtype=float)
        self.distance=np.r_[0,np.cumsum(np.linalg.norm(np.diff(self.route,axis=0),axis=1))]
        self.index=0
        self.target_speed=target_speed

    def update(self,position,forward,speed,obstacle_distance=float('inf')):
        forward=forward/max(1e-6,float(np.linalg.norm(forward)))
        # Pure pursuit is defined at the rear axle, 1.6 m behind the body origin.
        # Using the body origin here cuts inside tight bends.
        rear_axle=position-1.6*forward
        # A bounded search prevents jumping to another street at an intersection.
        start=max(0,self.index-6);end=min(len(self.route),self.index+70)
        self.index=start+int(np.argmin(np.linalg.norm(self.route[start:end]-rear_axle,axis=1)))
        body_index=start+int(np.argmin(np.linalg.norm(self.route[start:end]-position,axis=1)))
        cte=float(np.linalg.norm(self.route[body_index]-position))
        remaining=float(self.distance[-1]-self.distance[body_index])
        lookahead=2.3+0.2*speed
        target_index=min(len(self.route)-1,int(np.searchsorted(self.distance,self.distance[self.index]+lookahead)))
        delta=self.route[target_index]-rear_axle
        actual_lookahead=max(.5,float(np.linalg.norm(delta)))
        # PhysX's positive steering axis is +X when the vehicle faces +Z.
        steering_axis=np.array([forward[1],-forward[0]])
        lateral=float(np.dot(delta,steering_axis))
        steering=math.atan2(2*3.2*lateral,actual_lookahead**2)/.554264
        desired=min(self.target_speed,math.sqrt(max(0,2*1.2*(remaining-.5))))
        if abs(steering)>.55:desired=min(desired,2.7)
        desired=min(desired,math.sqrt(max(0,2*1.0*(obstacle_distance-3.0))))
        error=desired-speed
        throttle=float(np.clip(.07+.65*error,0,1))
        brake=float(np.clip(-.35*error,0,1))
        if desired<.12:throttle=0.;brake=.65
        return dict(throttle=throttle,brake=brake,steer=float(np.clip(steering,-1,1)),
                    desired_speed=desired,cte=cte,remaining=remaining,target_index=target_index,
                    progress=float(self.distance[body_index]/self.distance[-1]))
