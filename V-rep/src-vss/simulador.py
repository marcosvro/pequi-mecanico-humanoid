# -*- coding: utf-8 -*-
import vrep
import time
import numpy as np
import math

class Enviroment:
	def __init__(self, ip, port, num_robots, max_velocity, num_campos, tempo_por_partida):
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
		self.golObjectHandle = [0]*num_campos*2
		for i in range(0,num_campos*2, 2):
			resGolT0, self.golObjectHandle[i] = vrep.simxGetObjectHandle(self.clientID, "gol"+str(i), vrep.simx_opmode_oneshot_wait)
			resGolT1, self.golObjectHandle[i+1] = vrep.simxGetObjectHandle(self.clientID, "gol"+str(i+1), vrep.simx_opmode_oneshot_wait)
			resGolTime0, self.golDistHandle[i] =  vrep.simxGetDistanceHandle(self.clientID,"Dist_bola_gol"+str(i),vrep.simx_opmode_oneshot_wait)
			resGolTime1, self.golDistHandle[i+1] = vrep.simxGetDistanceHandle(self.clientID,"Dist_bola_gol"+str(i+1),vrep.simx_opmode_oneshot_wait)
			if resGolTime0 != vrep.simx_return_ok or resGolTime1 != vrep.simx_return_ok or resGolT0 != vrep.simx_return_ok or resGolT0 != vrep.simx_return_ok:
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
		self.state_size = self.num_robots*2*2
		self.action_size = self.num_robots*9
		self.max_velocity = max_velocity
		self.max_width = 0.9
		self.max_height = 0.64
		self.raio_robo = 0.1562
		self.z = 0.0382
		self.min_dist_gol = 0.13
		self.num_campos = num_campos
		self.timers = [0.]*num_campos
		self.max_time = tempo_por_partida
		self.campo = [False] * self.num_campos
		self.semafaro = [False] * self.num_campos
		self.team = [0] * (self.num_campos * 2)
		self.time_step_action = 0.05
		self.weight_rewards = [0.17, 0.17, 0.33, 0.33]
		self.actions = [(-1,-1),(-1,0),(-1,1),(0,-1),(0,0),(0,1),(1,-1),(1,0),(1,1)]*self.num_robots

	def close(self):
		#não sei
		vrep.simxFinish(self.clientID)

	def free_team(self, team_id):
		self.team[team_id] = 0
		enemy_id = 0
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
		enemy_id = 0
		if team_id%2==0:
			enemy_id = team_id+1
			enemy_robot_handle = [self.robotOrientationtHandle[(team_id+1)*self.num_robots + i] for i in range(self.num_robots)]
		else :
			enemy_id = team_id-1
			enemy_robot_handle = [self.robotOrientationtHandle[(team_id-1)*self.num_robots + i] for i in range(self.num_robots)]

			
		state = [0.] * 7
		init = 0
		#geting my robots positions
		for i in range(self.num_robots):
			ret, pos = vrep.simxGetObjectPosition(self.clientID,my_robots_handle[i],-1,operationMode=vrep.simx_opmode_oneshot_wait)
			while ret != vrep.simx_return_ok:
				ret, pos = vrep.simxGetObjectPosition(self.clientID,my_robots_handle[i],-1,operationMode=vrep.simx_opmode_oneshot_wait)
			state[i*2] = pos[1]/self.max_width
			if math.isnan(state[i*2]):
				state[i*2] = 0.
			state[i*2+1] = pos[0]/self.max_height
			if math.isnan(state[i*2+1]):
				state[i*2+1] = 0.
		#geting my robots orientations
		init += 2*self.num_robots
		for i in range(self.num_robots):
			ret, ori = vrep.simxGetObjectOrientation(self.clientID,my_robots_handle[i],-1,operationMode=vrep.simx_opmode_oneshot_wait)
			while ret != vrep.simx_return_ok:
				ret, ori = vrep.simxGetObjectOrientation(self.clientID,my_robots_handle[i],-1,operationMode=vrep.simx_opmode_oneshot_wait)
			state[init+i] = ori[2]/math.pi
			if math.isnan(state[init+i]):
				state[init+i] = 0.
		#geting enemy robots positions
		init += self.num_robots
		for i in range(self.num_robots):
			ret,pos = vrep.simxGetObjectPosition(self.clientID,enemy_robot_handle[i],-1,operationMode=vrep.simx_opmode_oneshot_wait)
			while ret != vrep.simx_return_ok:
				ret,pos = vrep.simxGetObjectPosition(self.clientID,enemy_robot_handle[i],-1,operationMode=vrep.simx_opmode_oneshot_wait)
			state[init+i*2] = pos[1]/self.max_width
			if math.isnan(state[init+i*2]):
				state[init+i*2] = 0.
			state[init+i*2+1] = pos[0]/self.max_height
			if math.isnan(state[init+i*2+1]):
				state[init+i*2+1] = 0.
		#geting ball position
		init += 2*self.num_robots
		ret,pos = vrep.simxGetObjectPosition(self.clientID,self.ballObjectHandle,-1,operationMode=vrep.simx_opmode_oneshot_wait)
		while ret != vrep.simx_return_ok:
			ret,pos = vrep.simxGetObjectPosition(self.clientID,self.ballObjectHandle,-1,operationMode=vrep.simx_opmode_oneshot_wait)
		state[init] = pos[1]/self.max_width
		if math.isnan(state[init]):
				state[init] = 0.
		state[init+1] = pos[0]/self.max_height
		if math.isnan(state[init+1]):
				state[init+1] = 0.

		ret,pos_gol = vrep.simxGetObjectPosition(self.clientID,self.golObjectHandle[enemy_id],-1,operationMode=vrep.simx_opmode_oneshot_wait)
		while ret != vrep.simx_return_ok:
			ret,pos_gol = vrep.simxGetObjectPosition(self.clientID,self.golObjectHandle[enemy_id],-1,operationMode=vrep.simx_opmode_oneshot_wait)

		#policy's
		policy = []
		policy.append(math.atan2((state[5*self.num_robots]-state[0])*self.max_width,(state[5*self.num_robots+1]-state[1])*self.max_height)-state[2*self.num_robots]*math.pi)
		policy.append(math.atan2(pos_gol[1]-state[0]*self.max_width,pos_gol[0]-state[1]*self.max_height)-state[2*self.num_robots]*math.pi)


		if math.isnan(policy[0]):
			if len(last_state)!=0:
				policy[0] = last_state[int(self.state_size/2)]
			else:
				policy[0] = 0.
		if math.isnan(policy[1]):
			if len(last_state)!=0:
				policy[1] = last_state[int(self.state_size/2)+1]
			else:
				policy[1] = 0.


		if len(last_state) != 0:
			return state, last_state[int(self.state_size/2):]+policy
		else:
			return policy+policy
		

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
			enemy_id = team_id-1
		else:
			enemy_id = team_id+1
		result_state, final_state = self.get_state(team_id,last_state)
		reward = self.weight_rewards[0]*math.cos(math.atan2((result_state[5*self.num_robots]-result_state[0])*self.max_width,(result_state[5*self.num_robots+1]-result_state[1])*self.max_height)-result_state[2*self.num_robots]*math.pi)
		reward += self.weight_rewards[1]*(1 - math.sqrt(((result_state[5*self.num_robots]-result_state[0])*self.max_width)**2+((result_state[5*self.num_robots+1]-result_state[1])*self.max_height)**2)/math.sqrt((2*self.max_width)**2+(2*self.max_height)**2))
		reward += self.weight_rewards[2]*(1 - math.fabs(math.atan2((result_state[5*self.num_robots]-result_state[0])*self.max_width,(result_state[5*self.num_robots+1]-result_state[1])*self.max_height)-math.atan2(y_gol-result_state[5*self.num_robots]*self.max_width, x_gol-result_state[5*self.num_robots+1]*self.max_height))/math.pi)
		reward += self.weight_rewards[3]*(1 - math.sqrt((result_state[5*self.num_robots]*self.max_width-y_gol)**2+(result_state[5*self.num_robots+1]*self.max_height-x_gol)**2) / math.sqrt((2*self.max_width)**2+(2*self.max_height)**2))

		acabou = False
		fiz_gol = False
		
		# verifica se alguem fez gol
		ret0 ,dist_gol_other = vrep.simxReadDistance(self.clientID,self.golDistHandle[enemy_id],vrep.simx_opmode_oneshot_wait)
		ret1 ,dist_gol_my = vrep.simxReadDistance(self.clientID,self.golDistHandle[team_id],vrep.simx_opmode_oneshot_wait)
		if ret0 != vrep.simx_return_ok:
			dist_gol_other = self.min_dist_gol
		if ret1 != vrep.simx_return_ok:
			dist_gol_my = self.min_dist_gol

		if dist_gol_other < self.min_dist_gol:
			acabou = True
			fiz_gol = True
			self.timers[int(team_id/2)] += self.max_time
		elif dist_gol_my < self.min_dist_gol:
			acabou = True
			self.timers[int(team_id/2)] += self.max_time
		elif time.time() > self.timers[int(team_id/2)] or self.team[enemy_id] == 0:
			acabou = True

		return fiz_gol, acabou, reward, final_state
			

	def get_team(self):
		indice = -1
		for i in range(len(self.team)):
			if self.team[i] == 0 and not self.campo[int(i/2)]:
				self.team[i] = 1
				indice = i
				print("Time ", i, " selecionado!!")
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
			if (self.semafaro[int(team_id/2)] and not self.campo[int(team_id/2)]) or self.campo[int(team_id/2)]:
				return False
			else:
				self.semafaro[int(team_id/2)] = True

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
			
			self.apply_action(team_id, 4)
			self.apply_action(enemy_id, 4)
			for i in range(self.num_robots):

				#ret = vrep.simxSetObjectPosition(self.clientID, self.robotObjectHandle[team_id*self.num_robots+i],-1, position=alocados[i], operationMode=vrep.simx_opmode_oneshot_wait)
				#ret1 = vrep.simxSetObjectPosition(self.clientID, self.robotObjectHandle[enemy_id*self.num_robots+i],-1, position=alocados[self.num_robots+i], operationMode=vrep.simx_opmode_oneshot_wait)
				ret = vrep.simxSetObjectPosition(self.clientID, self.robotObjectHandle[team_id*self.num_robots+i],-1, position=[0.,-0.3,self.z], operationMode=vrep.simx_opmode_oneshot_wait)
				ret1 = vrep.simxSetObjectPosition(self.clientID, self.robotObjectHandle[enemy_id*self.num_robots+i],-1, position=[0., 0.3,self.z], operationMode=vrep.simx_opmode_oneshot_wait)

				ret2 = vrep.simxSetObjectPosition(self.clientID, self.leftRodaHandle[team_id*self.num_robots+i], vrep.sim_handle_parent, [0.,0.,0.], vrep.simx_opmode_oneshot_wait)
				ret3 = vrep.simxSetObjectPosition(self.clientID, self.rightRodaHandle[team_id*self.num_robots+i], vrep.sim_handle_parent, [0.,0.,0.], vrep.simx_opmode_oneshot_wait)
				ret4 = vrep.simxSetObjectPosition(self.clientID, self.leftRodaHandle[enemy_id*self.num_robots+i], vrep.sim_handle_parent, [0.,0.,0.], vrep.simx_opmode_oneshot_wait)
				ret5 = vrep.simxSetObjectPosition(self.clientID, self.rightRodaHandle[enemy_id*self.num_robots+i], vrep.sim_handle_parent, [0.,0.,0.], vrep.simx_opmode_oneshot_wait)			

				ret2 = vrep.simxSetObjectOrientation(self.clientID, self.leftRodaHandle[team_id*self.num_robots+i], vrep.sim_handle_parent, [0.,0.,0.], vrep.simx_opmode_oneshot_wait)
				ret3 = vrep.simxSetObjectOrientation(self.clientID, self.rightRodaHandle[team_id*self.num_robots+i], vrep.sim_handle_parent, [0.,0.,0.], vrep.simx_opmode_oneshot_wait)
				ret4 = vrep.simxSetObjectOrientation(self.clientID, self.leftRodaHandle[enemy_id*self.num_robots+i], vrep.sim_handle_parent, [0.,0.,0.], vrep.simx_opmode_oneshot_wait)
				ret5 = vrep.simxSetObjectOrientation(self.clientID, self.rightRodaHandle[enemy_id*self.num_robots+i], vrep.sim_handle_parent, [0.,0.,0.], vrep.simx_opmode_oneshot_wait)

				#ret6 = vrep.simxSetObjectOrientation(self.clientID, self.robotObjectHandle[team_id*self.num_robots+i],-1, [0.,90.,0.], operationMode=vrep.simx_opmode_oneshot_wait)
				#ret7 = vrep.simxSetObjectOrientation(self.clientID, self.robotObjectHandle[enemy_id*self.num_robots+i],-1, [-89.,0.,-89.], operationMode=vrep.simx_opmode_oneshot_wait)
				
				if ret != vrep.simx_return_ok or ret1 != vrep.simx_return_ok or ret2 != vrep.simx_return_ok or ret3 != vrep.simx_return_ok or ret4 != vrep.simx_return_ok or ret5 != vrep.simx_return_ok:
					exit()
			ret = vrep.simxSetObjectPosition(self.clientID, self.ballObjectHandle,-1, position=[0.,0.,self.z], operationMode=vrep.simx_opmode_oneshot_wait)
			if ret != vrep.simx_return_ok:
				exit()	
			#seta timer final
			self.timers[int(team_id/2)] = time.time()+self.max_time
			print ("Jogo do campo ", int(team_id/2)," iniciado!!\n")
			self.campo[int(enemy_id/2)] = True
			self.semafaro[int(team_id/2)] = False
			return True
		else:
			return False

	def apply_action(self,team_id, action_id):
		#aplica ação nos robôs
		action = np.array(self.actions[action_id])*self.max_velocity
		for i in range(self.num_robots):
			ret = vrep.simxSetJointTargetVelocity(self.clientID, self.leftMotorHandle[team_id*self.num_robots+i], action[i*2], vrep.simx_opmode_oneshot_wait)
			ret = vrep.simxSetJointTargetVelocity(self.clientID, self.rightMotorHandle[team_id*self.num_robots+i], action[i*2+1], vrep.simx_opmode_oneshot_wait)



if __name__ == "__main__":
	env = Enviroment("127.0.0.1", 19999, 1, 10, 1, 20)

	for i in range(10):
		state_size = env.state_size
		action_size = env.action_size
		team = env.get_team()
		team2 = env.get_team()
		env.reset(team)
		while True:
			state = env.get_state(team)
			env.apply_action(team, [1.,1.])
			env.apply_action(team2, [-1.,-1.])
			time.sleep(0.1)
			tomei_gol, acabou, reward, result_state = env.get_reward(team, state)
			if acabou:
				break
		env.free_team(team)
		env.free_team(team2)

	env.close()


