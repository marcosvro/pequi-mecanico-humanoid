import threading
import numpy as np
import tensorflow as tf
import time
import math
import simulador as sim
from keras.layers import Dense, Input, LSTM
from keras.models import Model
from keras.optimizers import Adam
from keras import backend as K


# global variables for threading
episode = 0
scores = []

EPISODES = 2000

# This is A3C(Asynchronous Advantage Actor Critic) agent(global) for the Cartpole
# In this example, we use A3C algorithm
class A3CAgent:
	def __init__(self, state_size, action_size, env):
		# get size of state and action
		self.state_size = state_size
		self.action_size = action_size

		# these are hyper parameters for the A3C
		self.actor_lr = 0.005
		self.critic_lr = 0.005
		self.discount_factor = .95
		self.hidden0, self.hidden1, self.hidden2 = 50, 50, 25
		self.threads = 2
		self.reward_factor = 10

		# create model for actor and critic network
		self.actor, self.critic = self.build_model()

		# method for training actor and critic network
		self.optimizer = [self.actor_optimizer(), self.critic_optimizer(), False]

		self.sess = tf.InteractiveSession()
		K.set_session(self.sess)
		self.sess.run(tf.global_variables_initializer())
		self.env = env

	# approximate policy and value using Neural Network
	# actor -> state is input and probability of each action is output of network
	# critic -> state is input and value of state is output of network
	# actor and critic network share first hidden layer
	def build_model(self):
		state = Input(batch_shape=(None, 2,int(self.state_size/2)))
		lstm_layer = LSTM(self.hidden0, activation='tanh', kernel_initializer='glorot_uniform')(state)
		shared = Dense(self.hidden1, activation='relu', kernel_initializer='glorot_uniform')(lstm_layer)

		actor_hidden = Dense(self.hidden2, activation='relu', kernel_initializer='glorot_uniform')(shared)
		action_prob = Dense(self.action_size, activation='softmax', kernel_initializer='glorot_uniform')(actor_hidden)

		value_hidden = Dense(self.hidden2, activation='relu', kernel_initializer='he_uniform')(shared)
		state_value = Dense(1, activation='linear', kernel_initializer='he_uniform')(value_hidden)

		actor = Model(inputs=state, outputs=action_prob)
		critic = Model(inputs=state, outputs=state_value)

		actor._make_predict_function()
		critic._make_predict_function()

		actor.summary()
		critic.summary()

		return actor, critic

	# make loss function for Policy Gradient
	# [log(action probability) * advantages] will be input for the back prop
	# we add entropy of action probability to loss
	def actor_optimizer(self):
		action = K.placeholder(shape=(None, self.action_size))
		advantages = K.placeholder(shape=(None, ))

		policy = self.actor.output

		good_prob = K.sum(action * policy, axis=1)
		eligibility = K.log(good_prob + 1e-10) * K.stop_gradient(advantages)
		loss = -K.sum(eligibility)

		entropy = K.sum(policy * K.log(policy + 1e-10), axis=1)

		actor_loss = loss + 0.01*entropy

		optimizer = Adam(lr=self.actor_lr)
		updates = optimizer.get_updates(self.actor.trainable_weights, [], actor_loss)
		train = K.function([self.actor.input, action, advantages], [], updates=updates)
		return train

	# make loss function for Value approximation
	def critic_optimizer(self):
		discounted_reward = K.placeholder(shape=(None, ))

		value = self.critic.output

		loss = K.mean(K.square(discounted_reward - value))

		optimizer = Adam(lr=self.critic_lr)
		updates = optimizer.get_updates(self.critic.trainable_weights, [], loss)
		train = K.function([self.critic.input, discounted_reward], [], updates=updates)
		return train

	# make agents(local) and start training
	def train(self):
		self.load_model('./save_model/vss_a3c')
		agents = [Agent(i, self.actor, self.critic, self.optimizer, self.discount_factor, self.action_size, self.state_size, self.env, self.reward_factor) for i in range(self.threads)]

		for agent in agents:
			agent.start()

		while True:
			time.sleep(20)
			plot = scores[:]
			self.save_model('./save_model/vss_a3c')

	def save_model(self, name):
		self.actor.save_weights(name + "_actor.h5")
		self.critic.save_weights(name + "_critic.h5")

	def load_model(self, name):
		self.actor.load_weights(name + "_actor.h5")
		self.critic.load_weights(name + "_critic.h5")

# This is Agent(local) class for threading
class Agent(threading.Thread):
	def __init__(self, index, actor, critic, optimizer, discount_factor, action_size, state_size, env, reward_factor):
		threading.Thread.__init__(self)

		self.env = env

		self.states = []
		self.rewards = []
		self.actions = []

		self.index = index
		self.actor = actor
		self.critic = critic
		self.optimizer = optimizer
		self.discount_factor = discount_factor
		self.action_size = action_size
		self.state_size = state_size
		self.reward_factor = reward_factor

	# Thread interactive with environment
	def run(self):
		global episode
		#env = gym.make(self.env_name)

		while episode < EPISODES:
			team_id = self.env.get_team()
			while team_id == -1:
				time.sleep(2)
				team_id = self.env.get_team()

			camPlay = self.env.reset(team_id)
			while not camPlay:
				time.sleep(1)
				camPlay = self.env.reset(team_id)
			#print ("Time ", team_id, " iniciando partida!!")
			score = 0
			state = self.env.get_state(team_id)
			action = self.get_action(state)
			fiz_gol = False
			while True:
				#action = self.get_action(state)
				#next_state, reward, done, _ = env.step(action)
				#score += reward
				self.env.apply_action(team_id, action)
				#print ("Ação time ", team_id, " : ", action)
				time.sleep(self.env.time_step_action)

				fiz_gol, acabou, reward, next_state = self.env.get_reward(team_id, state)
				reward = reward*self.reward_factor
				if team_id==0:
					print (reward)
				score += reward
				self.memory(state, action, reward)

				if acabou:
					episode += 1
					self.env.free_team(team_id)
					print("episode: ", episode, "/ score : ", score)
					scores.append(score)
					break
				else:
					state = next_state
					action = self.get_action(state)
			time.sleep(1)
			if math.isnan(score):
				self.states, self.actions, self.rewards = [], [], []
			else:
				self.train_episode(fiz_gol)

	# In Policy Gradient, Q function is not available.
	# Instead agent uses sample returns for evaluating policy
	def discount_rewards(self, rewards, fiz_gol=False):
		discounted_rewards = np.zeros_like(rewards)
		running_add = 0
		if fiz_gol:
			#se não tomou gol ainda há recompensa para receber(valor predito por running_add)
			running_add = self.critic.predict(np.reshape(self.states[-1], (1,2, int(self.state_size/2))))[0]
		for t in reversed(range(0, len(rewards))):
			running_add = running_add * self.discount_factor + rewards[t]
			discounted_rewards[t] = running_add
		return discounted_rewards

	# save <s, a ,r> of each step
	# this is used for calculating discounted rewards
	def memory(self, state, action, reward):
		self.states.append(state)
		act = np.zeros(self.action_size)
		act[action] = 1
		self.actions.append(act)
		self.rewards.append(reward)

	# update policy network and value network every episode
	def train_episode(self, tomou_gol):
		discounted_rewards = self.discount_rewards(self.rewards, tomou_gol)

		values = self.critic.predict(np.reshape(self.states, (len(self.states),2, int(self.state_size/2))))
		values = np.reshape(values, len(values))

		advantages = discounted_rewards - values

		while self.optimizer[2]:
			time.sleep(0.5)
		self.optimizer[2] = True
		self.optimizer[0]([np.reshape(self.states, (len(self.states),2, int(self.state_size/2))), self.actions, advantages])
		self.optimizer[1]([np.reshape(self.states, (len(self.states),2, int(self.state_size/2))), discounted_rewards])
		self.optimizer[2] = False
		self.states, self.actions, self.rewards = [], [], []

	def get_action(self, state):
		try:
			while self.optimizer[2]:
				time.sleep(0.5)
			#self.optimizer[2] = True
			#print ("\n",state)
			policy = self.actor.predict(np.reshape(state, [1,2, int(self.state_size/2)]))[0]
			#self.optimizer[2] = False
		except RuntimeWarning:
			exit("Erro ao buscar ação!!")
		#return policy
		#print (policy)
		return np.random.choice(self.action_size, 1, p=policy)[0]


if __name__ == "__main__":
	env = sim.Enviroment("127.0.0.1", 19999, 1, 3, 1, 20)

	state_size = env.state_size
	action_size = env.action_size

	global_agent = A3CAgent(state_size, action_size, env)
	global_agent.train()
	env.close()

