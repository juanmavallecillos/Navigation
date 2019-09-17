# This file contains all the required routines to make an A* search algorithm.
#
__authors__='Jose Cegarra, Sandra Perez, Juanma Vallecillos'
__group__='DM17_08'
# _________________________________________________________________________________________
# Intel.ligencia Artificial
# Grau en Enginyeria Informatica
# Curs 2016- 2017
# Universitat Autonoma de Barcelona
# _______________________________________________________________________________________

from SubwayMap import *
from math import sqrt



class Node:
    # __init__ Constructor of Node Class.
    def __init__(self, station, father):
        """
        __init__: 	Constructor of the Node class
        :param
                - station: STATION information of the Station of this Node
                - father: NODE (see Node definition) of his father
        """
        
        self.station = station      # STATION information of the Station of this Node
        self.g = 0                  # REAL cost - depending on the type of preference -
                                    # to get from the origin to this Node
        self.h = 0                  # REAL heuristic value to get from the origin to this Node
        self.f = 0                  # REAL evaluate function
        if father ==None:
			self.parentsID=[]
        else:
			self.parentsID = [father.station.id]
			self.parentsID.extend(father.parentsID)         # TUPLE OF NODES (from the origin to its father)
        self.father = father        # NODE pointer to his father
        self.time = 0               # REAL time required to get from the origin to this Node
                                    # [optional] Only useful for GUI
        self.num_stopStation = 0    # INTEGER number of stops stations made from the origin to this Node
                                    # [optional] Only useful for GUI
        self.walk = 0               # REAL distance made from the origin to this Node
                                    # [optional] Only useful for GUI
        self.transfers = 0          # INTEGER number of transfers made from the origin to this Node
                                    # [optional] Only useful for GUI


    def setEvaluation(self):
        """
        setEvaluation: 	Calculates the Evaluation Function. Actualizes .f value
       
        """
        self.f = self.g + self.h

    def setHeuristic(self, typePreference, node_destination,city):
        """"
        setHeuristic: 	Calculates the heuristic depending on the preference selected
        :params
                - typePreference: INTEGER Value to indicate the preference selected: 
                                0 - Null Heuristic
                                1 - minimum Time
                                2 - minimum Distance 
                                3 - minimum Transfers
                                4 - minimum Stops
                - node_destination: PATH of the destination station
                - city: CITYINFO with the information of the city (see CityInfo class definition)
        """
        # No Preference
        if typePreference == 0:
            self.h = 0
            
        # Minimum Time
        if typePreference == 1:
            # If stations name's are different, then calculate distance.
            # Else time is 0 cause it's the same station with different lines
            if self.station.name != node_destination.station.name:
                d = minimumDistance(self.station, node_destination.station)
                v = city.max_velocity
                self.h = d/v
            else:
                self.h = 0
                
        # Minimum Distance
        elif typePreference == 2:
            # If stations name's are different, then set distance.
            # Else distance is 0 cause it's the same station
            if self.station.name != node_destination.station.name:
                # Distance
                self.h = minimumDistance(self.station, node_destination.station)
            else:
                self.h = 0
            
        # Minimum Transfers
        elif typePreference == 3:
            # If lines are the same, maybe we have no transfers.
            # Else we could have 1 or more transfers.
            if self.station.line == node_destination.station.line:
                self.h = 0
            else:
                self.h = 1
                
        # Minimum Stops
        elif typePreference == 4:
            # If name's are different, we could have 1 or more stops.
            # Else we are in the same station so, 0 stops.
            if self.station.name != node_destination.station.name:
                self.h = 1
            else:
                self.h = 0
                
        # Error Control
        else:
            print "Invalid typePreference."



    def setRealCost(self,  costTable):
        """
        setRealCost: 	Calculates the real cost depending on the preference selected
        :params
                 - costTable: DICTIONARY. Relates each station with their adjacency an their real cost. NOTE that this
                             cost can be in terms of any preference.
        """
        if self.father != None:
            # Real time/distance/transfers/stops value + real value from origin to actual
            self.g = costTable[self.station.id][self.father.station.id] + self.father.g


def Expand(fatherNode, stationList, typePreference, node_destination, costTable,city):
    """
        Expand: It expands a node and returns the list of connected stations (childrenList)
        :params
                - fatherNode: NODE of the current node that should be expanded
                - stationList: LIST of the stations of a city. (- id, destinationDic, name, line, x, y -)
                - typePreference: INTEGER Value to indicate the preference selected:
                                0 - Null Heuristic
                                1 - minimum Time
                                2 - minimum Distance
                                3 - minimum Transfers
                                4 - minimum Stops
                - node_destination: NODE (see Node definition) of the destination
                - costTable: DICTIONARY. Relates each station with their adjacency an their real cost. NOTE that this
                             cost can be in terms of any preference.
                - city: CITYINFO with the information of the city (see CityInfo class definition)
        :returns
                - childrenList:  LIST of the set of child Nodes for this current node (fatherNode)

    """
    childrenList = []
    
    # For every station in the father's station destinationDic:
    for station in fatherNode.station.destinationDic:
        
        # Create newNode with that station with this father
        newNode = Node(stationList[station - 1], fatherNode)
        
        # Calculate the heuristics, costTable and evaluation with our typePreference and city
        newNode.setHeuristic(typePreference, node_destination, city)
        newNode.setRealCost(costTable[typePreference])
        newNode.setEvaluation()
        
        # Append the parent ID to the fatherNode.parentsID list and set it to newNode
        newNode.parentsID = list(fatherNode.parentsID)
        newNode.parentsID.append(fatherNode.station.id)
        
        # Update time, walk, transfers and num_stopStation from newNode
        newNode.time = costTable[1][fatherNode.station.id][station] + fatherNode.time
        newNode.walk = costTable[2][fatherNode.station.id][station] + fatherNode.walk
        newNode.transfers = costTable[3][fatherNode.station.id][station] + fatherNode.transfers
        newNode.num_stopStation = costTable[4][fatherNode.station.id][station] + fatherNode.num_stopStation
        
        # Append newNode to the childrenList.
        childrenList.append(newNode)
    
    return childrenList


def RemoveCycles(childrenList):
    """
        RemoveCycles: It removes from childrenList the set of childrens that include some cycles in their path.
        :params
                - childrenList: LIST of the set of child Nodes for a certain Node
        :returns
                - listWithoutCycles:  LIST of the set of child Nodes for a certain Node which not includes cycles
    """

    listWithoutCycles= [ ]
    
    # Por cada nodo en la lista de nodos (childrenList) que hacen referencia
    # a los hijos de un nodo
    for node in childrenList:
        # anadir a la lista sin ciclos el nodo que estamos mirando
        listWithoutCycles.append(node)
        # si este nodo hijo existe en la lista de los nodos padres
        if node.station.id in node.father.parentsID:
            listWithoutCycles.remove( node ) # lo eliminamos
    
    return listWithoutCycles



def RemoveRedundantPaths(childrenList, nodeList, partialCostTable):
    """
        RemoveRedundantPaths:   It removes the Redundant Paths. They are not optimal solution!
                                If a node is visited and have a lower g in this moment, TCP is updated.
                                In case of having a higher value, we should remove this child.
                                If a node is not yet visited, we should include to the TCP.
        :params
                - childrenList: LIST of NODES, set of childs that should be studied if they contain rendundant path
                                or not.
                - nodeList : LIST of NODES to be visited
                - partialCostTable: DICTIONARY of the minimum g to get each key (Node) from the origin Node
        :returns
                - childrenList: LIST of NODES, set of childs without rendundant path.
                - nodeList: LIST of NODES to be visited updated (without redundant paths)
                - partialCostTable: DICTIONARY of the minimum g to get each key (Node) from the origin Node (updated)
    """
    listWithoutRendundantPath = [ ]
    # Por cada nodo en la lista de nodos (childrenList) que hacen referencia
    # a los hijos de un nodo
    for node in childrenList:
        # anadimos a la lista sin redundandoncias el nodo que estamos mirando 
        listWithoutRendundantPath.append(node)
        # Si este nodo hijo esta en la TCP, 
        if node.station.id in partialCostTable.keys( ) :
            # Si g de la TCP es > que el de la lista de hijos
            if partialCostTable[node.station.id] > node.g:
                # Actualizamos G en la TCP
                partialCostTable[node.station.id] = node.g
                # Para cada node de la lista de nodos a visitar
                """
                for i in nodeList:
                    # el nodo que visitamos es igual al nodo hijo, entonces lo eliminamos de nodeList 
                    if i.station.id == node.station.id:
                        nodeList.remove(i)
                """
            else: 
                listWithoutRendundantPath.remove(node)
        else :
            partialCostTable[node.station.id] = node.g
    return listWithoutRendundantPath , nodeList , partialCostTable


def sorted_insertion(nodeList,childrenList):

    """
       Sorted_insertion: It inserts each of the elements of childrenList into the nodeList.
    	The insertion must be sorted depending on the evaluation function value.
							
		: params:
			- nodeList : LIST of NODES to be visited
			- childrenList: LIST of NODES, set of childs that should be studied if they contain rendundant path
            or not.
        :returns
        - nodeList: sorted LIST of NODES to be visited updated with the childrenList included 
    """

    nodeList = nodeList + childrenList
    nodeList.sort(key=lambda node: node.f)
    
    return nodeList


def setCostTable( typePreference, stationList,city):
    """
    setCostTable :      Real cost of a travel.
    :param
            - typePreference: INTEGER Value to indicate the preference selected:
                                0 - Adjacency
                                1 - minimum Time
                                2 - minimum Distance
                                3 - minimum Transfers
                                4 - minimum Stops
            - stationList: LIST of the stations of a city. (- id, destinationDic, name, line, x, y -)
            - city: CITYINFO with the information of the city (see CityInfo class definition)
    :return:
            - costTable: DICTIONARY. Relates each station with their adjacency an their g, depending on the
                                 type of Preference Selected.
    """
    costTable = {}
    
    # No TypePreference
    if typePreference == 0:
        for i in city.adjacency.keys():
            for j in city.adjacency[i].keys():
                # Si el origen no esta introducido en el diccionario costTable, entonces creamos la entrada.
                # Guardo en la posicion [i][j] de la tabla costTable un 1 si las estaciones son adyacentes
                # ej. stationList[0].destinationDic[2] = 1...
                if i not in costTable.keys():
                    costTable[i] = {}
                costTable[i][j] = 1
                
    # Minimum Time
    elif typePreference == 1:
        for i in city.adjacency.keys():
            for j in city.adjacency[i].keys():
                # Si el origen no esta introducido en el diccionario costTable, entonces creamos la entrada 
                # para poder introducir el tiempo o los tiempos correspondientes.
                # guardo en la posicion [i][j] de la tabla costTable el tiempo real que tarda en llegar
                # del origen al destino segun el diccionario destinationDic de la estacion en cuestion
                # ej. stationList[0].destinationDic[2] = 9,0537... (origen id 1 - destino id 2)
                if i not in costTable.keys():
                    costTable[i] = {}
                costTable[i][j] = stationList[i-1].destinationDic[j]
       
    # Minimum Distance         
    elif typePreference == 2:
        for i in city.adjacency.keys():
            for j in city.adjacency[i].keys():
                if stationList[i-1].name != stationList[j-1].name:
                    v = city.velocity_lines[stationList[i-1].line - 1]
                    t = stationList[i-1].destinationDic[j]
                else:
                    t = 0
                # Si el origen no esta introducido en el diccionario costTable, entonces creamos la entrada 
                # para poder introducir la distancia o distancias correspondientes.
                # guardo en la posicion [i][j] de la tabla costTable la distancia real que tarda en llegar
                # ej. stationList[0].destinationDic[2] = 9,0537... (origen id 1 - destino id 2)
                if i not in costTable.keys():
                    costTable[i] = {}
                costTable[i][j] = t*v
                
          
    # Minimum Transfers
    elif typePreference == 3:
        for i in city.adjacency.keys():
            for j in city.adjacency[i].keys():
                if stationList[i-1].line == stationList[j-1].line:
                    value = 0
                else:
                    value = 1
                # Si el origen no esta introducido en el diccionario costTable, entonces creamos la entrada 
                # para poder introducir los transbordos o paradas correspondientes segun typePreference.
                # guardo en la posicion [i][j] de la tabla costTable la distancia real que tarda en llegar
                # ej. stationList[0].destinationDic[2] = 1...
                if i not in costTable.keys():
                    costTable[i] = {}
                costTable[i][j] = value
    
    # Minimum Stops
    elif typePreference == 4:
        for i in city.adjacency.keys():
            for j in city.adjacency[i].keys():
                if stationList[i-1].name == stationList[j-1].name:
                    value = 0
                else:
                    value = 1
                # Si el origen no esta introducido en el diccionario costTable, entonces creamos la entrada 
                # para poder introducir las paradas correspondientes segun typePreference.
                # guardo en la posicion [i][j] de la tabla costTable la distancia real que tarda en llegar
                # ej. stationList[0].destinationDic[2] = 1...
                if i not in costTable.keys():
                    costTable[i] = {}
                costTable[i][j] = value
                
    return costTable


def coord2station(coord, stationList):
    """
    coord2station :      From coordinates, it searches the closest station.
    :param
            - coord:  LIST of two REAL values, which refer to the coordinates of a point in the city.
            - stationList: LIST of the stations of a city. (- id, destinationDic, name, line, x, y -)

    :return:
            - possible_origins: List of the Indexes of the stationList structure, which corresponds to the closest
            station
    """
    possible_origins = []
    
    x = coord[0]
    y = coord[1]
    actualPosition = Station(0,0,0,x,y)
    closest = minimumDistance(actualPosition, stationList[0])
    for i in stationList:
        distance = minimumDistance(actualPosition, i)
        if distance == closest:
            possible_origins.append(i.id - 1)
        elif distance < closest:
            closest = distance
            possible_origins = []
            possible_origins.append(i.id - 1)
            
    return possible_origins
        
	

def AstarAlgorithm(stationList, coord_origin, coord_destination, typePreference,city,flag_redundants):
    """
     AstarAlgorithm: main function. It is the connection between the GUI and the AStar search code.
     INPUTS:
            - stationList: LIST of the stations of a city. (- id, name, destinationDic, line, x, y -)
            - coord_origin: TUPLE of two values referring to the origin coordinates
            - coord_destination: TUPLE of two values referring to the destination coordinates
            - typePreference: INTEGER Value to indicate the preference selected:
                                0 - Adjacency
                                1 - minimum Time
                                2 - minimum Distance
                                3 - minimum Transfers
                                4 - minimum Stops
            - city: CITYINFO with the information of the city (see CityInfo class definition)
			- flag_redundants: [0/1]. Flag to indicate if the algorithm has to remove the redundant paths (1) or not (0)
			
    OUTPUTS:
            - time: REAL total required time to make the route
            - distance: REAL total distance made in the route
            - transfers: INTEGER total transfers made in the route
            - stopStations: INTEGER total stops made in the route
            - num_expanded_nodes: INTEGER total expanded nodes to get the optimal path
            - depth: INTEGER depth of the solution
            - visitedNodes: LIST of INTEGERS, IDs of the stations corresponding to the visited nodes
            - idsOptimalPath: LIST of INTEGERS, IDs of the stations corresponding to the optimal path
            (from origin to destination)
            - min_distance_origin: REAL the distance of the origin_coordinates to the closest station
            - min_distance_destination: REAL the distance of the destination_coordinates to the closest station
            


            EXAMPLE:
            return optimalPath.time, optimalPath.walk, optimalPath.transfers,optimalPath.num_stopStation,
            len(expandedList), len(idsOptimalPath), visitedNodes, idsOptimalPath, min_distance_origin,
            min_distance_destination
    """
    typePreference = int(typePreference)

    # Cost tables
    costAdjacencyTable = setCostTable(0, stationList, city)
    costTimeTable = setCostTable(1, stationList, city)
    costDistanceTable = setCostTable(2, stationList, city)
    costTransferTable = setCostTable(3, stationList, city)
    costStopsTable = setCostTable(4, stationList, city)
    
    costTable = [costAdjacencyTable, costTimeTable, costDistanceTable, costTransferTable, costStopsTable]
    #costTable = costTable1[typePreference]
    partialCostTable = {}
    
    possible_origins = coord2station(coord_origin, stationList)
    origin_station = stationList[possible_origins[0]]
    
    possible_destinations = coord2station(coord_destination, stationList)
    destination_station = stationList[possible_destinations[0]]
    
    nodeArrel = Node(origin_station, None)
    nodeObjectiu = Node(destination_station, None)
    
    llista = [[nodeArrel]]
    visited_nodes = [nodeArrel.station.name]
    
    while True:
        path = llista.pop(0)
        pathCap = path[-1]
        
        if pathCap.station == destination_station:
            break
        
        E = Expand(pathCap, stationList, typePreference, nodeObjectiu, costTable, city)
        E = RemoveCycles(E)
        
        for i in E:
            llista.append(path + [i])
            visited_nodes.append(i.station.name)
            
        if flag_redundants == 1:
            E = RemoveRedundantPaths(E, llista, partialCostTable)
            E = E[0]
             
        for i in llista:
            i = sorted_insertion(i, E)
        
    
    if path != None:
        optIDPath = []
        lastNode = path[-1]
        for i in path:
            optIDPath.append(i.station.id)

        return lastNode.time,lastNode.walk,lastNode.transfers,lastNode.num_stopStation,len(visited_nodes),len(optIDPath),visited_nodes,optIDPath,0,0
    else:
        return "No existeix cap solucio possible"
    
    
       
    

def minimumDistance(origin, destination):
    """
    minimumDistance :    From station nodes, it searches the minimum distance.
    :param
            - origin:  origin station
            - destination: destination station

    :return:
            - sqrt(x**2 + y**2): minimum distance between two stations
    """
    x = destination.x - origin.x
    y = destination.y - origin.y
    # Distance
    return sqrt(x**2 + y**2)
