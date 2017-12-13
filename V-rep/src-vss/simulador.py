# -*- coding: utf-8 -*-
import vrep
import time
import numpy as np
import math

class Enviroment:
	def __init__(self, ip, port, num_robots, max_velocity, num_campos):
		self.clientID = vrep.simxStart(ip, port, True, True, 2000, 5)
		if self.clientID == -1:
			exit()
		self.num_robots = num_robots
		self.robotObjectHandle = [0]*self.num_robots*num_campos*2
		self.robotOrientationtHandle = [0]*self.num_robots*num_campos*2
		self.leftMotorHandle = [0]*self.num_robots*num_campos*2
		self.rightMotorHandle = [0]*self.num_robots*num_campos*2
		self.leftRodaHandle = [0]*self.num_robots*num_campos*2
		self.rightRodaHandle = [0]*self.num_robots*num_campos*2
		self.golDistHandle = [0]*num_campos*2
		for i in range(0,num_campos*2, 2):
			resGolTime0, self.golDistHandle[i] =  vrep.simxGetDistanceHandle(self.clientID,"Dist_bola_gol"+str(i*2),vrep.simx_opmode_oneshot_wait)
			resGolTime1, self.golDistHandle[i+1] = vrep.simxGetDistanceHandle(self.clientID,"Dist_bola_gol"+str(i*2+1),vrep.simx_opmode_oneshot_wait)
			if resGolTime0 or resGolTime1:
				exit()
		resBall, self.ballObjectHandle = vrep.simxGetObjectHandle(self.clientID, "bola", vrep.simx_opmode_oneshot_wait)
		if resBall:
			exit()
		for i in range(0, self.num_robots*2*num_campos*2, 2):
			resBase, self.robotObjectHandle[int(i/2)] = vrep.simxGetObjectHandle(self.clientID, "robo"+str(int(i/2)), vrep.simx_opmode_oneshot_wait)
			resOrien, self.robotOrientationtHandle[int(i/2)] = vrep.simxGetObjectHandle(self.clientID, "orientacao"+str(int(i/2)), vrep.simx_opmode_oneshot_wait)
			resLeft, self.leftMotorHandle[int(i/2)] = vrep.simxGetObjectHandle(self.clientID, "motor"+str(i+1), vrep.simx_opmode_oneshot_wait)
			resRight, self.rightMotorHandle[int(i/2)] = vrep.simxGetObjectHandle(self.clientID, "motor"+str(i), vrep.simx_opmode_oneshot_wait)
			resLeftR, self.leftRodaHandle[int(i/2)] = vrep.simxGetObjectHandle(self.clientID, "Cylinder"+str(i+1), vrep.simx_opmode_oneshot_wait)
			resRightR, self.rightRodaHandle[int(i/2)] = vrep.simxGetObjectHandle(self.clientID, "Cylinder"+str(i), vrep.simx_opmode_oneshot_wait)
			if resLeft != vrep.simx_return_ok or resRight != vrep.simx_return_ok or resBase != vrep.simx_return_ok or resLeftR != vrep.simx_return_ok or resRightR != vrep.simx_return_ok:
				exit()
		self.state_size = (self.num_robots*2 + self.num_robots*2*2 + 2)*2
		self.action_size = 2*self.num_robots
		self.max_velocity = max_velocity
		self.max_width = 0.9
		self.max_height = 0.64
		self.raio_robo = 0.1562
		self.z = 0.0382
		self.min_dist_gol = 0.13
		self.num_campos = num_campos
		self.campo = [False] * self.num_campos
		self.team = [0] * (self.num_campos * 2)
		self.time_step_action = 0.1
		self.weight_rewards = [0.17, 0.17, 0.33, 0.33]

	def close(self):
		#não sei
		vrep.simxFinish(self.clientID)

	def free_team(self, team_id):
		self.team[team_id] = 0
		enemy_id
		if team_id%2 == 0:
			enemy_id = team_id+1
		else:
			enemy_id = team_id-1
		if self.team[enemy_id] == 0:
			self.campo[int(team_id/2)] = False
		

	def get_state(self, team_id, last_state = []):
		#get motor handles from team team_id
		my_robots_leftMotor = [self.leftMotorHandle[team_id*self.num_robots + i] for i in range(self.num_robots)]
		my_robots_rightMotor = [self.rightMotorHandle[team_id*self.num_robots + i] for i in range(self.num_robots)]
		my_robots_handle = [self.robotOrientationtHandle[team_id*self.num_robots + i] for i in range(self.num_robots)]
		enemy_robot_handle = []
		if team_id%2==0:
			enemy_robot_handle = [self.robotOrientationtHandle[(team_id+1)*self.num_robots + i] for i in range(self.num_robots)]
		else :
			enemy_robot_handle = [self.robotOrientationtHandle[(team_id-1)*self.num_robots + i] for i in range(self.num_robots)]

			
		state = [0.] * (int(self.state_size/2))
		init = 0
		#geting my robots positions
		for i in range(self.num_robots):
			ret, pos = vrep.simxGetObjectPosition(self.clientID,my_robots_handle[i],-1,operationMode=vrep.simx_opmode_oneshot_wait)
			while ret != vrep.simx_return_ok:
				ret, pos = vrep.simxGetObjectPosition(self.clientID,my_robots_handle[i],-1,operationMode=vrep.simx_opmode_oneshot_wait)
			state[i*2] = pos[1]/self.max_width
			state[i*2+1] = pos[0]/self.max_height
		#geting my robots orientations
		init += 2*self.num_robots
		for i in range(self.num_robots):
			ret, ori = vrep.simxGetObjectOrientation(self.clientID,my_robots_handle[i],-1,operationMode=vrep.simx_opmode_oneshot_wait)
			while ret != vrep.simx_return_ok:
				ret, ori = vrep.simxGetObjectOrientation(self.clientID,my_robots_handle[i],-1,operationMode=vrep.simx_opmode_oneshot_wait)
			state[init+i] = ori[2]/180.
		#geting enemy robots positions
		init += self.num_robots
		for i in range(self.num_robots):
			ret,pos = vrep.simxGetObjectPosition(self.clientID,enemy_robot_handle[i],-1,operationMode=vrep.simx_opmode_oneshot_wait)
			while ret != vrep.simx_return_ok:
				ret,pos = vrep.simxGetObjectPosition(self.clientID,enemy_robot_handle[i],-1,operationMode=vrep.simx_opmode_oneshot_wait)
			state[init+i*2] = pos[1]/self.max_width
			state[init+i*2+1] = pos[0]/self.max_height
		#geting enemy robots orientations
		init += 2*self.num_robots
		for i in range(self.num_robots):
			ret, ori = vrep.simxGetObjectOrientation(self.clientID,enemy_robot_handle[i],-1,operationMode=vrep.simx_opmode_oneshot_wait)
			while ret != vrep.simx_return_ok:
				ret, ori = vrep.simxGetObjectOrientation(self.clientID,enemy_robot_handle[i],-1,operationMode=vrep.simx_opmode_oneshot_wait)
			state[init+i] = ori[2]/180.
		#geting ball position
		init += self.num_robots
		ret,pos = vrep.simxGetObjectPosition(self.clientID,self.ballObjectHandle,-1,operationMode=vrep.simx_opmode_oneshot_wait)
		while ret != vrep.simx_return_ok:
			ret,pos = vrep.simxGetObjectPosition(self.clientID,self.ballObjectHandle,-1,operationMode=vrep.simx_opmode_oneshot_wait)
		state[init] = pos[1]/self.max_width
		state[init+1] = pos[0]/self.max_height


		if len(last_state) != 0:
			state = last_state[int(self.state_size/2):]+state
		else:
			state = state+state
		
		return state
		

	def get_reward(self,team_id, last_state):
		#função bonitinha
		'''
		Virar para a bola -> cos(atan2(Ybola-Yrobô, Xbola-Xrobô)-GAMMArobô)
		Ir até a bola     -> 1 - DISTÂNCIA_rb/sqrt((2*width)**2+(2*height)**2)
		Mira bola p/ gol  -> 1 - abs(atan2(Ybola-Yrobô, Xbola-Xrobô)-atan2(Ygol-Ybola, Xgol-Xbola))/pi
		Leva bola p/ gol  -> 1 - DISTÂNCIA_bg/sqrt((2*width)**2+(2*height)**2)
		'''
		x_gol = 0
		y_gol = 0.9
		enemy_id = 0
		if team_id%2 != 0:
			y_gol = -y_gol
			enemy_id = team_id+1
		else:
			enemy_id = team_id-1
		result_state = self.get_state(team_id,last_state)
		reward = self.weight_rewards[0]*math.cos(math.atan2((result_state[int(self.state_size/2)+6*self.num_robots]-result_state[int(self.state_size/2)+0])*self.max_width,(result_state[int(self.state_size/2)+6*self.num_robots+1]-result_state[int(self.state_size/2)+1])*self.max_height)-result_state[2*self.num_robots]*math.pi)
		reward += self.weight_rewards[1]*(1 - math.sqrt(((result_state[int(self.state_size/2)+6*self.num_robots]-result_state[int(self.state_size/2)+0])*self.max_width)**2+((result_state[int(self.state_size/2)+6*self.num_robots+1]-result_state[int(self.state_size/2)+1])*self.max_height)**2)/math.sqrt((2*self.max_width)**2+(2*self.max_height)**2))
		reward += self.weight_rewards[2]*(1 - math.fabs(math.atan2((result_state[int(self.state_size/2)+6*self.num_robots]-result_state[int(self.state_size/2)+0])*self.max_width,(result_state[int(self.state_size/2)+6*self.num_robots+1]-result_state[int(self.state_size/2)+1])*self.max_height)-math.atan2(y_gol-result_state[int(self.state_size/2)+6*self.num_robots]*self.max_width, x_gol-result_state[int(self.state_size/2)+6*self.num_robots+1]*self.max_height))/math.pi)
		reward += self.weight_rewards[3]*(1 - math.sqrt((result_state[int(self.state_size/2)+6*self.num_robots]*self.max_width-y_gol)**2+(result_state[int(self.state_size/2)+6*self.num_robots+1]*self.max_height-x_gol)**2) / math.sqrt((2*self.max_width)**2+(2*self.max_height)**2))

		acabou = False
		tomei_gol = False
		
		# verifica se alguem fez gol
		ret0 ,dist_gol_other = vrep.simxReadDistance(self.clientID,self.golDistHandle[enemy_id],vrep.simx_opmode_oneshot_wait)
		ret1 ,dist_gol_my = vrep.simxReadDistance(self.clientID,self.golDistHandle[team_id],vrep.simx_opmode_oneshot_wait)
		if ret0 != vrep.simx_return_ok:
			dist_gol_other = self.min_dist_gol
		if ret1 != vrep.simx_return_ok:
			dist_gol_my = self.min_dist_gol

		if dist_gol_other < self.min_dist_gol:
			acabou = True
		elif dist_gol_my < self.min_dist_gol:
			acabou = True
			tomei_gol = True

		return tomei_gol, acabou, reward, result_state
			

	def get_team(self):
		indice = -1
		for i in range(len(self.team)):
			if self.team[i] == 0 and not self.campo[int(i/2)]:
				self.team[i] = 1
				indice = i
				break
		return indice

	def reset(self,team_id):
		if self.campo[int(team_id/2)]:
			return True
		enemy_id = 0
		if team_id%2 == 0:
			enemy_id = team_id+1
		else:
			enemy_id = team_id-1
		if self.team[enemy_id] == 1:
			#RESETAR POSIÇÕES E AÇÃO -----------------------------------------------------------------------------------------------------------
			self.campo[int(enemy_id/2)] = True
			alocados = []
			for i in range(self.num_robots*2+1):
				flag = False
				while not flag:
					x = 2*(self.max_height-self.raio_robo)*np.random.random() - (self.max_height-self.raio_robo)
					y = 2*(self.max_width-self.raio_robo)*np.random.random() - (self.max_width-self.raio_robo)
					flag = True
					for k in alocados:
						dist = math.sqrt((k[0]-x)**2+(k[1]-y)**2)
						if dist < self.raio_robo:
							flag = False
							break
					if flag:
						alocados.append((x,y,self.z))
			
			self.apply_action(team_id, [0.]*self.action_size)
			self.apply_action(enemy_id, [0.]*self.action_size)
			for i in range(self.num_robots):
				ret = vrep.simxSetObjectPosition(self.clientID, self.robotObjectHandle[team_id*self.num_robots+i],-1, position=alocados[i], operationMode=vrep.simx_opmode_oneshot_wait)
				ret1 = vrep.simxSetObjectPosition(self.clientID, self.robotObjectHandle[enemy_id*self.num_robots+i],-1, position=alocados[self.num_robots+i], operationMode=vrep.simx_opmode_oneshot_wait)
				
				#ret6 = vrep.simxSetObjectOrientation(self.clientID, self.robotObjectHandle[team_id*self.num_robots+i],-1, [-89.,0.,-89.], operationMode=vrep.simx_opmode_oneshot_wait)
				#ret7 = vrep.simxSetObjectOrientation(self.clientID, self.robotObjectHandle[enemy_id*self.num_robots+i],-1, [-89.,0.,-89.], operationMode=vrep.simx_opmode_oneshot_wait)
				
				ret2 = vrep.simxSetObjectPosition(self.clientID, self.leftRodaHandle[team_id*self.num_robots+i], vrep.sim_handle_parent, [0.,0.,0.], vrep.simx_opmode_oneshot_wait)
				ret3 = vrep.simxSetObjectPosition(self.clientID, self.rightRodaHandle[team_id*self.num_robots+i], vrep.sim_handle_parent, [0.,0.,0.], vrep.simx_opmode_oneshot_wait)
				ret4 = vrep.simxSetObjectPosition(self.clientID, self.leftRodaHandle[enemy_id*self.num_robots+i], vrep.sim_handle_parent, [0.,0.,0.], vrep.simx_opmode_oneshot_wait)
				ret5 = vrep.simxSetObjectPosition(self.clientID, self.rightRodaHandle[enemy_id*self.num_robots+i], vrep.sim_handle_parent, [0.,0.,0.], vrep.simx_opmode_oneshot_wait)
				
				if ret != vrep.simx_return_ok or ret1 != vrep.simx_return_ok or ret2 != vrep.simx_return_ok or ret3 != vrep.simx_return_ok or ret4 != vrep.simx_return_ok or ret5 != vrep.simx_return_ok:
					exit()
			ret = vrep.simxSetObjectPosition(self.clientID, self.ballObjectHandle,-1, position=alocados[-1], operationMode=vrep.simx_opmode_oneshot_wait)
			if ret != vrep.simx_return_ok:
				exit()			
			return True
		else:
			return False

	def apply_action(self,team_id, action):
		#aplica ação nos robôs
		action = np.array(action)*self.max_velocity
		print (action)
		for i in range(self.num_robots):
			ret = vrep.simxSetJointTargetVelocity(self.clientID, self.leftMotorHandle[team_id*self.num_robots+i], action[i*2], vrep.simx_opmode_oneshot_wait)
			ret = vrep.simxSetJointTargetVelocity(self.clientID, self.rightMotorHandle[team_id*self.num_robots+i], action[i*2+1], vrep.simx_opmode_oneshot_wait)



if __name__ == "__main__":
	env = Enviroment("127.0.0.1", 19999, 1, 9, 1)

	state_size = env.state_size
	action_size = env.action_size
	team = env.get_team()
	team2 = env.get_team()
	env.reset(team)
	state = env.get_state(team)
	env.apply_action(team, [1.,1.])
	tomei_gol, acabou, reward, result_state = env.get_reward(team, state)

	env.close()


