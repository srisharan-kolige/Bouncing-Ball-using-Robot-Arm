import pybullet as p
import os
import math

class Ball:
    def __init__(self,client):
        self.client = client
        f_name = os.path.join(os.path.dirname(__file__), 'bouncy_ball.urdf')
        self.ball = p.loadURDF(fileName=f_name,
                        basePosition=[0.2, 0.2, 2], # adjust for starting position of ball
                        physicsClientId=client)
        # SET UP SO BOUNCING IS ENABLED --> restitution of 1 might be too much
        p.changeDynamics(self.ball, -1, restitution=1.)

    def get_observation(self):
        position = p.getBasePositionAndOrientation(self.ball, self.client)[0]
        velocity = p.getBaseVelocity(self.ball, self.client)[0][0:3]
        observation = (position + velocity) # concatenate position and velocity
        
        return observation
    
    def get_ids(self):
        return self.ball, self.client