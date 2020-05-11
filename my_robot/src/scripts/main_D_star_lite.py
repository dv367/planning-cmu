from myClasses import *
from basics import *
from copy import copy, deepcopy



#________________________________________Global Variables__________________________________#
expansions = init = 0
goalF = float('inf')
goalBackPointer = "null"
goalX = goalY = 0
startX = startY = 0
#__________________________________________________________________________________________#

   #0 1 2 3 4 5 6 7 8 9
#0 [_ _ _ # _ _ _ _ _ _] 
#1 [_ _ _ # _ _ _ # _ _]
#2 [_ _ # # _ _ _ # _ _]
#3 [_ _ # # _ _ G # _ _]
#4 [_ _ # # # _ _ _ _ _]
#5 [_ _ # # # # _ _ _ _]
#6 [_ _ _ _ # # _ _ _ _]
#7 [_ _ _ _ # # _ _ _ _]
#8 [_ _ _ _ _ # _ # _ _]
#9 [_ _ _ _ _ # _ _ _ _]
#10[S _ _ _ _ # _ _ _ _] Start = (10,0) Goal(3,6)
        #0 1 2 3 4 5 6 7 8 9 
Map = [				  
	[0,0,0,0,0,0,0,0,0,0],#0
	[0,0,0,0,0,0,0,0,0,0],#1
	[0,0,0,0,0,0,0,0,0,0],#2
	[0,0,1,0,1,0,0,0,0,0],#3
	[1,1,1,0,1,1,0,0,0,0],#4
	[0,0,0,0,0,0,0,0,0,0],#5
	[0,0,0,0,1,1,0,0,0,0],#6
	[0,0,0,0,1,0,0,0,0,0],#7
	[0,0,0,0,1,0,0,0,0,0],#8
	[1,1,1,0,1,0,0,0,0,0],#9
	[0,0,0,0,1,0,0,0,0,0]#10
]


OPEN = []
OPEN_ = []
CLOSED = []
CLOSED_ = []


Point_i = [1,-1,0,0,-1,-1,1,1]
Point_j = [0,0,1,-1,-1,1,1,-1]

def smallest_F(data,minimum):
	
	index = -1

	for i in range(len(data)):
		if data[i].f < minimum:
			minimum = data[i].f 
			index = i

	return index
		
def notIn(s,data):
	
	for i in range(len(data)):
		if data[i] == s:
			return i
	return -1

def extractIndicies(s):
	i = ""
	j = ""
	space = []
	for k in range(len(s)):
		if s[k] == ' ':
			space.append(k)
	i = i + s[space[0]:space[1]]
	j = j + s[space[1]:]

	return int(i),int(j)


def computePathReuse(xs,ys,xg,yg,e,Map):
	
	global OPEN, CLOSED	
	global OPEN_, CLOSED_
	global Point_i, Point_j
	global expansions, goalF, init, goalBackPointer

	if init == 0:

		OPEN = []
		OPEN_ = []
		CLOSED = []
		CLOSED_ = []

		print len(OPEN)
		
		startNode = node(xs,ys,Map[xs][ys],0,xg,yg,"null",e)#i,j,val,g,xg,yg,backP,e
		startNode.backpointer = startNode.name
		
		expansions = 0
		goalF = float('inf')
		goalBackPointer = "null"		
		init = 1

		OPEN.append(startNode)
		OPEN_.append(startNode.name)
	else:
		expansions = 0

		for i in range(len(OPEN)):
			OPEN[i].updateF(e)


	goalIndex = -1
	goalExpanded = 0 	
	solutionExist = 0
	

	temp = deepcopy(Map)
	temp[xs][ys] = 2
	temp[xg][yg] = 3

	while goalExpanded == 0:
		
		index = smallest_F(OPEN,1000)
		
		if goalF < OPEN[index].f:
			solutionExist = 1
			break
 
		print "PUSH IN CLOSED = ",OPEN[index].name,OPEN[index].f
		i = OPEN[index].i
		j = OPEN[index].j
		g = OPEN[index].g	
		name = OPEN[index].name
		OPEN[index].v = OPEN[index].g

		#if i!=xs and j!=ys:
			#temp[i][j] = 4			#show only expaneded nodes

		expansions = expansions + 1

		CLOSED.append(OPEN.pop(index))
		CLOSED_.append(OPEN_.pop(index))
		
		for k in range(len(Point_i)):
			m = Point_i[k] + i
			n = Point_j[k] + j
			
			if m < len(Map) and m >= 0 and n < len(Map[0]) and n >= 0:
				s = "s" + " " + str(m) + " " + str(n)
					
				openIndex = notIn(s,OPEN_)
				closedIndex = notIn(s,CLOSED_) 
				
				cost = 1	
				#print s		
				if m!=0 and n!=0:
					cost = 1.4
							
				if closedIndex == -1 and openIndex ==-1:			
					OPEN.append(node(m,n,Map[m][n],g+cost,xg,yg,name,e))
					OPEN_.append(s)

					if Map[m][n] == 0:
						temp[m][n] = 4	# show all nodes that are explored				
					
					if s == "s" + " " + str(xg) + " " + str(yg):
						goalExpanded = 1			
						solutionExist = 1
						goalF = OPEN[len(OPEN)-1].f
						goalBackPointer = name
						goalIndex = len(OPEN)-1
					
						
					
				elif openIndex != -1:
					OPEN[openIndex].checkG(Map[m][n],g+cost,name,e)
					if s == "s" + " " + str(xg) + " " + str(yg):
						goalExpanded = 1			
						solutionExist = 1
						goalF = OPEN[openIndex].f
						goalBackPointer = name
						goalIndex = openIndex			
						
				elif closedIndex != -1:
					CLOSED[closedIndex].checkG(Map[m][n],g+cost,name,e)
					if s == "s" + " " + str(xg) + " " + str(yg):
						goalExpanded = 1			
						solutionExist = 1
						goalF = OPEN[openIndex].f
						goalBackPointer = name
						goalIndex = openIndex			
					
			

				
						
		plotGrid(temp,"draw")
	
	if solutionExist:
		print "Goal Found in Expansions = ",expansions
		return goalBackPointer
	else:
		print "Expansions = ",expansions
		return "null" 
		
		
							
def getSolution(xs,ys,xg,yg,goalBackPointer,Map):
	
	global CLOSED, CLOSED_

	solved = deepcopy(Map)
	solution = []

	goal = "s"+" "+str(xg)+" "+str(yg)
	start = "s"+" "+str(xs)+" "+str(ys)

	solution.append(goal)	
	i, j = extractIndicies(goal)
	solved[i][j] = 3

	backP = goalBackPointer
	while(backP != "s"+" "+str(xs)+" "+str(ys)):
		
		solution.append(backP)
		i, j = extractIndicies(backP)
		solved[i][j] = 3	

		index = notIn(backP,CLOSED_)
		backP = CLOSED[index].backpointer
		
	solution.append(start)
	i, j = extractIndicies(start)
	solved[i][j] = 2

	return solved,solution		#solved is a Map and solution is a route
			
def followPath(xs,ys,xg,yg,solution,Map):

	temp = deepcopy(Map)
	m = n = -1
	prev_m = prev_n = -1


	temp[xs][ys] = 2
	temp[xg][yg] = 3
	for i in range(len(solution)):

		m,n = extractIndicies(solution[i])
	
		if Map[m][n] == 0:
			
			temp[m][n] = 4
			
			plotGrid(temp,"draw")
		else:
			return prev_m,prev_n,m,n

		prev_m = m
		prev_n = n

	return m,n,-1,-1
	
def updateEdges(blockX,blockY,Map):
	
					
					
	
				
def D_Star_Lite(xs,ys,xg,yg,e,Map):
	
	global startX, startY
	global goalX, goalY
	global init
	
	startX = xs
	startY = ys
	
	goalX = xg
	goalY = yg
	
	
	while(startX != goalX and startY != goalY):		

		goalBackPointer = computePathReuse(goalX,goalY,startX,startY,e,Map)
		solvedMap , route = getSolution(goalX,goalY,startX,startY,goalBackPointer,Map)	

		Map[5][4] = 1
		
		startX, startY, blockX, blockY = followPath(startX,startY,goalX,goalY,route,Map)

		if blockX == -1 and blockY == -1:
			break	
	
		updateEdges(blockX,blockY,Map)
		init = 0
		
		
		
	
		
		
			
		
		
		
			
	
			


if __name__ == '__main__':
	D_Star_Lite(10,0,4,8,1.0,Map)
	  




           	

	





