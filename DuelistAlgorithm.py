'''Duelist Algorithm
###
###Code and Implementation by Sin Yong, Teng
###
### Duelist Algorithm is a metaheuristic (just like genetic algorithm) that performs optimization procedure by mimicing high-level procedure in our world.
### In the original paper, Duelist Algorithm is able to outperform various state-of-art metaheuristic algorithms for stochastic problems.
### I implemented this algorithm in Python to allow for integrative use of this algorithm in other software and works.
###
###Original Paper:
###	1. Biyanto, T.R., Fibrianto, H.Y., Nugroho, G., Hatta, A.M., Listijorini, E., Budiati, T. and Huda, H., 2016, June. Duelist algorithm: an algorithm inspired by how duelist improve their capabilities in a duel. In International Conference on Swarm Intelligence (pp. 39-47). Springer, Cham.
###	https://arxiv.org/abs/1512.00708
###
###Implemented on 25/04/2019
'''


import numpy as np
import random
import time
import matplotlib.pyplot as mp

class DuelistAlgorithm():
	def __init__(self,f,x,xmin,xmax,pop=1,luck=0.01,mut=0.1,learn=0.8,max_gen=500,nc=1,shuffle=False):
		#setup class variables
		self.f=f
		self.x=x
		self.xmin=xmin
		self.xmax=xmax
		self.pop=pop
		self.luck=luck
		self.mut=mut
		self.learn=learn
		self.max_gen=max_gen
		self.nc=nc
		self.champion=np.empty((x.__len__()+1,nc),dtype=np.float64)
		self.winloss=np.empty((pop,1),dtype=np.float64)
		self.battlescore=np.empty((pop,1),dtype=np.float64)
		self.gen=0
		self.shuffle=shuffle
		self.plotgen=[]
		self.plotfit=[]
		#check if multivariate optimization should be turned on.
		if type(x) is list:
			self.mult=1
			assert x.__len__()==xmin.__len__()==xmax.__len__() , "Constraint error, check xmin and xmax"
		else:
			self.mult=0
		
		#initialize calculation matrix
		if self.mult==1:
			shape=(x.__len__(),pop)
		else:
			shape=(1,pop)
		self.matrix=np.empty(shape,dtype=np.float64)
		self.train=np.empty(shape,dtype=np.float64)
		self.score=np.empty(pop,dtype=np.float64)
		self.tmat=np.empty((x.__len__()+1,pop),dtype=np.float64)
		self.bestlog=np.empty((0,x.__len__()+1),dtype=np.float64)
		
	def solve(self,plot=False):
		self.plot=plot
		#steps for duelist algorithm
		self.registration()
		self.qualification()
		while self.gen <self.max_gen+1:
			self.champion_select_train()
			self.duel()
			self.duelist_improvement()
			self.post_qualification()
			self.eliminate()
			self.gen=self.gen+1
		self.announce_answer()

	def registration(self):
		for i in range(0,self.x.__len__()):
		#pseudo-random generator for initializing population
			t = int( time.time() * 1000.0 )
			np.random.seed( ((t & 0xff000000) >> 24) +
             ((t & 0x00ff0000) >>  8) +
             ((t & 0x0000ff00) <<  8) +
             ((t & 0x000000ff) << 24)   )
			#randomize input such that it can be bounded by specified constraints
			self.matrix[i,:]=np.random.uniform(size=self.pop,low=self.xmin[i],high=self.xmax[i])


	def qualification(self):
		#this part only works for post-qualification when population is doubled
		if self.score.shape[0]<self.matrix.shape[1]:
			self.score=np.append(self.score,self.score)
		#compute score and sort evaluate them
		for i in range(0,self.matrix.shape[1]):
			self.score[i]=self.f(*self.matrix.T.tolist()[i])
		self.evaluatescore()

	def evaluatescore(self):
		#evaluate score by sorting solutions from lowest to highest
		#this is for minimization, for maximization please insert negative in objective function
		self.score=np.asarray([self.score])
		self.tmat=np.concatenate((self.score,self.matrix),axis=0).T
		
		self.tmat=self.tmat[self.tmat[:,0].argsort()].T
		self.score=self.tmat[0,:]
		self.matrix=self.tmat[1:,:]
	
	def post_qualification(self):
		#transpose matrix to sortable form
		self.matrix=self.matrix.T
		#run qualification again
		self.qualification()
		
	def champion_select_train(self):
		#log the best champion
		for i in range(0,self.nc):
			self.bestlog=np.concatenate((self.bestlog,np.asarray([self.tmat[:,i]])))
		#separate champions from population
		self.champion=self.tmat[:,0:self.nc]
		print("#Generation ", self.gen)
		print("Best Champion Fitness",self.tmat[:,0][0],"Solution",self.tmat[:,0][1::])
		self.plotgen.append(self.gen)
		if self.plotfit.__len__()==0:
			self.plotfit.append(self.tmat[:,0][0])
		elif self.tmat[:,0][0]<min(self.plotfit):
			self.plotfit.append(self.tmat[:,0][0])
		else:
			self.plotfit.append(min(self.plotfit))
		#let champion train duelists that are similar to themselves
		for j in range(0,self.nc):
			for i in range(0,self.x.__len__()):
				if (random.uniform(0,1)<self.mut):
					self.matrix[i,j]=random.uniform(self.xmin[i],self.xmax[i])
		
	def duel(self):
		#shuffle the population so that dueling is randomly matched
		self.matrix=self.matrix.T
		if(self.shuffle==True):
			np.random.shuffle(self.matrix)
		
		#dueling procedure
		i=0
		while i<self.matrix.shape[0]:
			#if population is odd, duelist that doesn't get matched automatically wins
			if(i==self.matrix.shape[0]-1):
				self.winloss[i]=1
			else:
			#compute battle score for each of the two duelist. Battle score is based on fitness and luck
				tempmatrix=self.matrix.tolist()
				self.battlescore[i]=self.f(*tempmatrix[i])*(1+(self.luck+(random.uniform(0,1)*self.luck)))
				self.battlescore[i+1]=self.f(*tempmatrix[i+1])*(1+(self.luck+(random.uniform(0,1)*self.luck)))
			#compare battle score and indicate winner and loser
				if(self.battlescore[i]>self.battlescore[i+1]):
					self.winloss[i]=1
					self.winloss[i+1]=0
				else:
					self.winloss[i]=0
					self.winloss[i+1]=1
			i=i+2
	
	def duelist_improvement(self):
		#duelist improvement splits for winner and loser
		self.train=np.copy(self.matrix)
		for j in range(0,self.x.__len__()):
			for i in range(0,self.pop):
				if self.winloss[i]==1:
				#winner improves itself by mutating
					if random.uniform(0,1)<self.mut:
						self.train[i,j]=random.uniform(self.xmin[j],self.xmax[j])
				else:
				#loser learns from winner
					if random.uniform(0,1)<self.learn:
						if (i%2==0):
							self.train[i,j]=self.matrix[i+1,j]
						else:
							self.train[i,j]=self.matrix[i-1,j]
		#add newly trained duelist into duelist pool
		self.matrix=np.concatenate((self.matrix,self.train),axis=0)
	
	def eliminate(self):
		self.matrix=self.matrix[:,:self.pop]
		
	def announce_answer(self):
		answerlog=self.bestlog[self.bestlog[:,0].argsort()]
		print("Optimized using Duelist Algorithm. Answer is:",answerlog[0][1::], "with fitness of", answerlog[0][0])
		if self.plot==True:
			mp.plot(self.plotgen,self.plotfit)
			mp.title("Duelist Algorithm Optimization")
			mp.xlabel("Number of Generation")
			mp.ylabel("Objective Function of Overall Best Solution")
			mp.autoscale()
			mp.show()
	
	def get_result():
		return self.plotgen, self.plotfit
		