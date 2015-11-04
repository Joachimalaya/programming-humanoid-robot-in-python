'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''


from pid import PIDAgent
from keyframes import hello, wipe_forehead
from spark_agent import JOINT_CMD_NAMES



class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])
        
        # track elapsed time
        self.et = 0.0

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes)
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes):
        target_joints = {}
        # YOUR CODE HERE
        
        self.et += self.joint_controller.dt
        
        names, times, totalKeys = keyframes
        
        for i in range(0, len(names)):
            if names[i] in JOINT_CMD_NAMES:
                name = names[i]
                time = times[i]
                keys = totalKeys[i]
                # find relevant keyframe
                for j in range(0, len(time)-2):
                    if time[j] < self.et < time[j+1]:
                        # target_joints[name] = keys[j][0]
                        # cubic spline interpolation to get started
                        normalizedTimes = normalizeTimes(time)
                        a = keys[j-1][0]
                        b = keys[j][0]
                        c = keys[j+1][0]
                        d = keys[j+2][0]
                        
                        t0 = time[j-1]
                        tEnd = time[j+2]
                        
                        tRel = (self.et - t0)/(tEnd - t0)
                        
                        target_joints[name] = a*tRel**3 + b*tRel**2 + c*tRel + d
                            
        return target_joints

def normalizeTimes(times):
    largest = times[-1]
    normalizedTimes = []
    for t in times:
        normalizedTimes.append(t / largest)
        
    return normalizedTimes

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
