# -*- coding: utf-8 -*-
import vrep
import time

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
			if resLeft != vrep.simx_return_ok or resRight != vrep.simx_return_ok or resBase != vrep.simx_return_ok:
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

	def close():
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
			ret,x,y,z = vrep.simxGetObjectPosition(self.clientID,my_robots_handle[i],operationMode=vrep.simx_opmode_oneshot_wait)
			while ret != vrep.simx_return_ok:
				ret,x,y,z = vrep.simxGetObjectPosition(self.clientID,my_robots_handle[i],operationMode=vrep.simx_opmode_oneshot_wait)
			state[i*2] = y/self.max_width
			state[i*2+1] = x/self.max_height
		#geting my robots orientations
		init += 2*self.num_robots
		for i in range(self.num_robots):
			ret,a,b,g = vrep.simxGetObjectOrientation(self.clientID,my_robots_handle[i],operationMode=vrep.simx_opmode_oneshot_wait)
			while ret != vrep.simx_return_ok:
				ret,a,b,g = vrep.simxGetObjectOrientation(self.clientID,my_robots_handle[i],operationMode=vrep.simx_opmode_oneshot_wait)
			state[init+i] = g/180.
		#geting enemy robots positions
		init += self.num_robots
		for i in range(self.num_robots):
			ret,x,y,z = vrep.simxGetObjectPosition(self.clientID,enemy_robot_handle[i],operationMode=vrep.simx_opmode_oneshot_wait)
			while ret != vrep.simx_return_ok:
				ret,x,y,z = vrep.simxGetObjectPosition(self.clientID,enemy_robot_handle[i],operationMode=vrep.simx_opmode_oneshot_wait)
			state[init+i*2] = y/self.max_width
			state[init+i*2+1] = x/self.max_height
		#geting enemy robots orientations
		init += 2*self.num_robots
		for i in range(self.num_robots):
			ret,a,b,g = vrep.simxGetObjectOrientation(self.clientID,enemy_robot_handle[i],operationMode=vrep.simx_opmode_oneshot_wait)
			while ret != vrep.simx_return_ok:
				ret,a,b,g = vrep.simxGetObjectOrientation(self.clientID,enemy_robot_handle[i],operationMode=vrep.simx_opmode_oneshot_wait)
			state[init+i] = g/180.
		#geting ball position
		init += self.num_robots
		ret,x,y,z = vrep.simxGetObjectPosition(self.clientID,ballObjectHandle,operationMode=vrep.simx_opmode_oneshot_wait)
		while ret != vrep.simx_return_ok:
			ret,x,y,z = vrep.simxGetObjectPosition(self.clientID,ballObjectHandle,operationMode=vrep.simx_opmode_oneshot_wait)
		state[init] = y/self.max_width
		state[init+1] = x/self.max_height


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
		result_state = self.get_state(last_state)
		reward = weight_rewards[0]*math.cos(math.atan2((result_state[int(self.state_size/2)+6*self.num_robots]-result_state[int(self.state_size/2)+0])*self.width,(result_state[int(self.state_size/2)+6*self.num_robots+1]-result_state[int(self.state_size/2)+1])*self.height)-result_state[2*self.num_robots]*math.pi)
		reward += weight_rewards[1]*(1 - math.sqrt(((result_state[int(self.state_size/2)+6*self.num_robots]-result_state[int(self.state_size/2)+0])*self.width)**2+((result_state[int(self.state_size/2)+6*self.num_robots+1]-result_state[int(self.state_size/2)+1])*self.height)**2)/math.sqrt((2*self.width)**2+(2*self.height)**2))
		reward += weight_rewards[2]*(1 - math.fabs(math.atan2((result_state[int(self.state_size/2)+6*self.num_robots]-result_state[int(self.state_size/2)+0])*self.width,(result_state[int(self.state_size/2)+6*self.num_robots+1]-result_state[int(self.state_size/2)+1])*self.height)-math.atan2(y_gol-result_state[int(self.state_size/2)+6*self.num_robots]*self.width, x_gol-result_state[int(self.state_size/2)+6*self.num_robots+1]*self.height))/math.pi)
		reward += weight_rewards[3]*(1 - math.sqrt((result_state[int(self.state_size/2)+6*self.num_robots]*self.width-y_gol)**2+(result_state[int(self.state_size/2)+6*self.num_robots+1]*self.height-x_gol)**2) / math.sqrt((2*self.width)**2+(2*self.height)**2))

		acabou = False
		tomei_gol = False
		
		# verifica se alguem fez gol
		ret0 ,dist_gol_other = vrep.simxReadDistance(self.clientID,golDistHandle[enemy_id],vrep.simx_opmode_oneshot_wait)
		ret1 ,dist_gol_my = vrep.simxReadDistance(self.clientID,golDistHandle[team_id],vrep.simx_opmode_oneshot_wait)
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
			for i in range(self.num_robots*2):
				flag = False
				while not flag:
					x = 2*(self.max_height-self.raio_robo)*np.random.random() - (self.max_height-self.raio_robo)
					y = 2*(self.max_width-self.raio_robo)*np.random.random() - (self.max_widthi-self.raio_robo)
					flag = True
					for k in alocados:
						dist = math.sqrt((k[0]-x)**2+(k[1]-y)**2)
						if dist < self.raio_robo:
							flag = False
							break
					if flag:
						alocados.add((x,y,self.z))
			
			for i in range(self.num_robots):
				ret = vrep.simxSetObjectPosition(self.clientID, robotObjectHandle[team_id*self.num_robots+i], position=alocados[i], operationMode=vrep.simx_opmode_oneshot_wait)
				ret1 = vrep.simxSetObjectPosition(self.clientID, robotObjectHandle[enemy_id*self.num_robots+i], position=alocados[self.num_robots+i], operationMode=vrep.simx_opmode_oneshot_wait)
				if ret != vrep.simx_return_ok or ret1 != vrep.simx_return_ok:
					exit()
			return True
		else:
			return False

	def apply_action(self,team_id, action):
		#aplica ação nos robôs
		action = action*self.max_velocity
		for i in range(self.num_robots):
			ret = vrep.simxSetJointTargetVelocity(self.clientID, leftMotorHandle[team_id*self.num_robots+i], action[i*2], vrep.simx_opmode_oneshot_wait)
			ret = vrep.simxSetJointTargetVelocity(self.clientID, rightMotorHandle[team_id*self.num_robots+i], action[i*2+1], vrep.simx_opmode_oneshot_wait)




