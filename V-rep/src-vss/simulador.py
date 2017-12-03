# -*- coding: utf-8 -*-
import vrep
import time

class Enviroment:
	def __init__(self, ip, port, num_robots, max_velocity, num_campos):
		self.clientID = vrep.simxStart(ip, port, True, True, 2000, 5)
		if self.clientID == -1:
			exit()
		self.num_robots = num_robots
		self.leftMotorHandle = [0]*self.num_robots
		self.rightMotorHandle = [0]*self.num_robots
		for i in range(1, self.num_robots*2, 2):
			resLeft, leftMotorHandle[(i-1)/2] = vrep.simxGetObjectHandle(self.clientID, "motor"+str(i+1), vrep.simx_opmode_oneshot_wait)
			resRight, rightMotorHandle[(i-1)/2] = vrep.simxGetObjectHandle(self.clientID, "motor"+str(i), vrep.simx_opmode_oneshot_wait)
			if resLeft != vrep.simx_return_ok or resRight != vrep.simx_return_ok:
				exit()
		self.state_size = self.num_robots*5 + self.num_robots*2*2 + 3
		self.action_size = 2*self.num_robots
		self.max_velocity = max_velocity
		self.max_width = 1.5/2.
		self.max_height = 1.3/2.
		self.num_campos = num_campos
		self.campo = [False] * self.num_campos
		self.team = [0] * (self.num_campos * 2)
		self.time_step_action = 0.1

	def close():
		#não sei

	def get_state():
		#retorna vetor com entrada para rede neural

	def get_reward():
		#função bonitinha
		'''
			Virar para a bola -> cos(atan2(Ybola-Yrobô, Xbola-Xrobô)-GAMMArobô)
			Ir até a bola     -> 1 - DISTÂNCIA_rb/sqrt((2*width)**2+(2*height)**2)
			Mira bola p/ gol  -> 1 - abs(atan2(Ybola-Yrobô, Xbola-Xrobô)-atan2(Ygol-Ybola, Xgol-Xbola))/pi
			Leva bola p/ gol  -> 1 - DISTÂNCIA_bg/sqrt((2*width)**2+(2*height)**2)
		'''

	def get_team():
		indice = -1
		for i in range(len(self.team)):
			if not self.team[i]:
				team[i] = 1
				indice = i
				break
		return indice

	def reset(team_id):
		if campo[int(team_id/2)]:
			return True
		ver = 0
		if team_id%2 == 0:
			ver = team_id+1
		else:
			ver = team_id-1
		if team[ver] == 1:
			#RESETAR POSIÇÕES E AÇÃO -----------------------------------------------------------------------------------------------------------
			campo[int(ver/2)] = True
			return True
		else:
			return False

	def apply_action(team_id, action):
		#aplica ação nos robôs







serverIP = "127.0.0.1"
serverPort = 19999
#---------------------Conecta no servidor---------------------------------
clientID = vrep.simxStart(serverIP, serverPort, True, True, 2000, 5)
leftMotorHandle = []
rightMotorHandle = []

global v_Left, v_Right, tacoDir, tacoEsq, path_lenght, colisao, atingiu, clearance

v_Left = 0
v_Right = 0

raio = 0.195/2
colisao = False
atingiu = False
esperando = False
clearance = 0

if (clientID!=-1):
	print ("Servidor Conectado!")

#------------------------------Inicializa Sensores ----------------------------
	for i in range(0,8):
		nomeSensor.append("sensor" + str(i+1))

		res, handle = vrep.simxGetObjectHandle(clientID, nomeSensor[i], vrep.simx_opmode_oneshot_wait)

		if(res != vrep.simx_return_ok):
			print (nomeSensor[i] + " nao conectado")
		else:
			print (nomeSensor[i] + " conectado")
			sensorHandle.append(handle)

#------------------------------Inicializa Motores ----------------------------
	resLeft, leftMotorHandle = vrep.simxGetObjectHandle(clientID, "Pioneer_p3dx_leftMotor", vrep.simx_opmode_oneshot_wait)
	if(resLeft != vrep.simx_return_ok):
		print("Motor Esquerdo : Handle nao encontrado!")
	else:
		print("Motor Esquerdo: Conectado")

	resRight, rightMotorHandle = vrep.simxGetObjectHandle(clientID, "Pioneer_p3dx_rightMotor", vrep.simx_opmode_oneshot_wait)
	if(resRight != vrep.simx_return_ok):
		print("Motor Direito: Handle nao encontrado!")
	else:
		print("Motor Direito: Conectado")
else:
	print ("Servidor nao conectado!")

global padrao, virando, posInicial, salvar
padrao = raw_input('Qual Padrao de ambiente sera treinado? ')
posInicial = raw_input('Qual a posicao inicial do robo? ')
#numTreinamento = raw_input('Qual o numero do treinamento? ')
virando = False
salvar = False
#-----------------Inicializa localizacao------------------
localizacao = localization.localizacao()
localization.iniciar(clientID)

#----------------Inicializa o blending -------------------
blending = blending.blending()

#---------------------Seta velocidades nos motores-----------------------
vrep.simxSetJointTargetVelocity(clientID, rightMotorHandle, v_Right, vrep.simx_opmode_streaming)
vrep.simxSetJointTargetVelocity(clientID, leftMotorHandle, v_Left, vrep.simx_opmode_streaming)

thetaDir = vrep.simxGetJointPosition(clientID, rightMotorHandle, vrep.simx_opmode_streaming)[1]
thetaEsq = vrep.simxGetJointPosition(clientID, leftMotorHandle, vrep.simx_opmode_streaming)[1]
localizacao.setAngulos(thetaDir, thetaEsq)


#----------------------Thread do teclado---------------------------------------------
def listen_keyboard():
	# Collect events until released
	with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
		listener.join()

def on_press(key):
	global virando, salvar
	if key == keyboard.Key.left:
		virando = True
		salvar = False
		velEsq = -0.5
		velDir = 0.5
	elif key == keyboard.Key.right:
		virando = True
		salvar = False
		velEsq = 0.5
		velDir = -0.5
	elif key == keyboard.Key.esc:
		# Stop listener
		sys.exit(0)
		return False
	elif key == keyboard.Key.up:
		virando = False
		salvar = True
		velEsq = 1
		velDir = 1
	else:
		virando = False
		velEsq = 0
		velDir = 0

	vrep.simxSetJointTargetVelocity(clientID, rightMotorHandle, velDir, vrep.simx_opmode_streaming)
	vrep.simxSetJointTargetVelocity(clientID, leftMotorHandle, velEsq, vrep.simx_opmode_streaming)

	thetaDir = vrep.simxGetJointPosition(clientID, rightMotorHandle, vrep.simx_opmode_streaming)[1]
	thetaEsq = vrep.simxGetJointPosition(clientID, leftMotorHandle, vrep.simx_opmode_streaming)[1]
	localizacao.setAngulos(thetaDir, thetaEsq)



def on_release(key):
	global virando, salvar
	if virando:
		salvar = True
		virando = False
	else:
		salvar = False

	vrep.simxSetJointTargetVelocity(clientID, rightMotorHandle, v_Right, vrep.simx_opmode_streaming)
	vrep.simxSetJointTargetVelocity(clientID, leftMotorHandle, v_Left, vrep.simx_opmode_streaming)

	thetaDir = vrep.simxGetJointPosition(clientID, rightMotorHandle, vrep.simx_opmode_streaming)[1]
	thetaEsq = vrep.simxGetJointPosition(clientID, leftMotorHandle, vrep.simx_opmode_streaming)[1]

	localizacao.setAngulos(thetaDir, thetaEsq)

def getThetaAlvo(thetaRobo, xRobo, yRobo):
	xAlvo = 0
	yAlvo = 0
	tolerancia = 0.5

	if padrao == 'A':
	# ------------- Posicao1 --------------
		if posInicial == '1':
			xAlvo = 7.3
			yAlvo = 0.8

		# ------------- Posicao2 --------------
		elif posInicial == '2':
			xAlvo = 5.4
			yAlvo = -0.5
		# ------------- Posicao3 --------------
		elif posInicial == '3':
			xAlvo = 6.8
			yAlvo = 2.5
		# ------------- Posicao4 --------------
		elif posInicial == '4':
			xAlvo = 2.8
			yAlvo = 2.2

		# ------------- Posicao5 --------------
		elif posInicial == '5':
			xAlvo = 7.2
			yAlvo = -1.6

		# ------------- Posicao6 --------------
		elif posInicial == '6':
			xAlvo = 2.0
			yAlvo = -2.3

	elif padrao == 'B':
		if posInicial == '1':
			xAlvo = 7.1
			yAlvo = 0.0
	elif padrao == 'C':
		if posInicial == '1':
			xAlvo = 7.6
			yAlvo = 0.78
	elif padrao == 'D':
		if posInicial == '1':
			xAlvo = 1.8
			yAlvo = 2
	elif padrao == 'E':
		if posInicial == '1':
			xAlvo = 0.5
			yAlvo = -3.2
	elif padrao == 'F':
		if posInicial == '1':
			xAlvo = 1.9
			yAlvo = -0.5
	elif padrao == 'G':
		if posInicial == '1':
			xAlvo = 6.0
			yAlvo = -2.3
	elif padrao == 'H':
		if posInicial == '1':
			xAlvo = -3.5
			yAlvo = 0.0
	elif padrao == 'I':
		if posInicial == '1':
			xAlvo = 4.9
			yAlvo = 0.0
		elif posInicial == '2':
			xAlvo = 2.4
			yAlvo = -0.9
		elif posInicial == '3':
			xAlvo = 1.4
			yAlvo = -0.45

		
	if(xAlvo > xRobo):
		thetaAlvo =  - thetaRobo + math.atan((yAlvo - yRobo)/(xAlvo - xRobo))
	else:
		if(yAlvo > yRobo):
			thetaAlvo = -thetaRobo + math.pi + math.atan((yAlvo - yRobo)/(xAlvo - xRobo))
		else:
			thetaAlvo = -thetaRobo - math.pi + math.atan((yAlvo - yRobo)/(xAlvo - xRobo))
	
	#thetaAlvo = math.atan((yAlvo - yRobo)/(xAlvo - xRobo))
	if (abs(xRobo - xAlvo) < tolerancia) and (abs(yRobo - yAlvo) < tolerancia):
		thetaAlvo = 0
	
	return thetaAlvo


thread.start_new_thread(listen_keyboard,())
thetaRoboAnt = 0
lista_entradas = []
lista_saidas = []
leituras = []
clearance = 0
#---------------------------Loop principal ---------------------------------------
while vrep.simxGetConnectionId(clientID) != -1:
	thetaDir = vrep.simxGetJointPosition(clientID, rightMotorHandle, vrep.simx_opmode_streaming)[1]
	thetaEsq = vrep.simxGetJointPosition(clientID, leftMotorHandle, vrep.simx_opmode_streaming)[1]
	localizacao.setAngulos(thetaDir, thetaEsq)
	path_lenght = localizacao.getPathLenght() #atributo Lng

	#----------------------------lê os sensores---------------------------------
	for i in range(0,8):
		returnCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(clientID, sensorHandle[i], vrep.simx_opmode_streaming)
		if (returnCode == vrep.simx_return_ok):
			if(detectionState != 0):
				dist.append(detectedPoint[2])
			else:
				dist.append(5.0)
		#print math.degrees(thetaRobo-getThetaAlvo(thetaRobo, xRobo, yRobo))
		time.sleep(0.01)

	thetaRobo = localizacao.getOrientacao()
	xRobo, yRobo = localizacao.getPosicao()
	thetaAlvo = getThetaAlvo(thetaRobo, xRobo, yRobo)

	print "x: ", (xRobo), " y: ",(yRobo)," ThetaRobo: ",math.degrees(thetaRobo)
	print "ThetaAlvo: ", (math.degrees(thetaAlvo))
	if len(dist)==8:
		#for da PARAMETRIZACAO
		while not salvar and vrep.simxGetConnectionId(clientID) != -1:
			thetaDir = vrep.simxGetJointPosition(clientID, rightMotorHandle, vrep.simx_opmode_streaming)[1]
			thetaEsq = vrep.simxGetJointPosition(clientID, leftMotorHandle, vrep.simx_opmode_streaming)[1]
			localizacao.setAngulos(thetaDir, thetaEsq)
			thetaRobo = localizacao.getOrientacao()
			xRobo, yRobo = localizacao.getPosicao()
			thetaAlvo = getThetaAlvo(thetaRobo, xRobo, yRobo)
			esperando = True
		esperando = False

		thetaDir = vrep.simxGetJointPosition(clientID, rightMotorHandle, vrep.simx_opmode_streaming)[1]
		thetaEsq = vrep.simxGetJointPosition(clientID, leftMotorHandle, vrep.simx_opmode_streaming)[1]
		localizacao.setAngulos(thetaDir, thetaEsq)
		thetaRobo = localizacao.getOrientacao()
		xRobo, yRobo = localizacao.getPosicao()
		thetaAlvo = getThetaAlvo(thetaRobo, xRobo, yRobo)
		print "x: ", (xRobo), " y: ",(yRobo)," ThetaRobo: ",math.degrees(thetaRobo)
		print "ThetaAlvo: ", (math.degrees(thetaAlvo))

		for n in range(len(dist)):
			dist[n] = dist[n]/5.0
			#leituras.append(dist[n])
			if(dist[n] <= 0.01):
				colisao = True
		if min(dist) < 0.05:
			clearance = clearance + 1.0 - min(dist)/(sum(dist)/len(dist))

		blending.setLeituras(dist)
		padrao_blending = blending.definePadrao()
		#print blending.calculaPesos(padrao)

		if thetaAlvo == 0:
			atingiu = True
			print "ATINGIU"
			#print min(leituras)
			print clearance

		entradas = str(dist[0])+", "+str(dist[1])+", "+str(dist[2])+", "+str(dist[3])+", "+str(dist[4])+", "+str(dist[5])+", "+str(dist[6])+", "+str(dist[7])+", "+str(thetaAlvo/math.pi)


		saida = str((thetaRobo-thetaRoboAnt)/(math.pi))

		lista_entradas.append(entradas)
		lista_saidas.append(saida)

		print "Saida: ", math.degrees(thetaRobo-thetaRoboAnt)
		thetaRoboAnt = thetaRobo
		salvar = False
	dist=[]

raw_input("Aperte ENTER para salvar o treinamento ou CTRL+C para Cancelar")
nome_diretorio = 'Padrao'+padrao
nome_arquivo_entrada = 'Entrada'+padrao+'.txt'
nome_arquivo_saida = 'Saida'+padrao+'.txt'

for i in range(len(lista_entradas)):
	#verifica se ja existe o diretorio
	if os.path.isdir(nome_diretorio):
			#grava entradas no txt
			if os.path.isfile(nome_diretorio+'/'+nome_arquivo_entrada):
				arquivo = open(nome_diretorio+'/'+nome_arquivo_entrada, 'a+')
				arquivo.write(lista_entradas[i]+'\n')
				arquivo.close()
			else:
				arquivo = open(nome_diretorio+'/'+nome_arquivo_entrada, 'w+')
				arquivo.write(lista_entradas[i]+'\n')
				arquivo.close()


			#grava saidas no txt
			if os.path.isfile(nome_diretorio+'/'+nome_arquivo_saida):
				arquivo = open(nome_diretorio+'/'+nome_arquivo_saida, 'a+')
				arquivo.write(lista_saidas[i]+'\n')
				arquivo.close()
			else:
				arquivo = open(nome_diretorio+'/'+nome_arquivo_saida, 'w+')
				arquivo.write(lista_saidas[i]+'\n')
				arquivo.close()
	else:
		os.mkdir(nome_diretorio)

print "Treinamento salvo com sucesso!"
