import matplotlib.pyplot as plt
import csv

#Contains epsilon specific lists
class EpsilonDataContainter:
    def __init__(self, epsilon):
        #Lists for keeping track of time with and without variance
        self.timeListVariance = []
        self.timeListNoVariance = []

        #Lists for keeping track of expansions with and without variance
        self.expansionsListNoVariance = []
        self.expansionsListVariance = []

        #Lists for keeping track of cost with and without variance
        self.costListVariance = []
        self.costListNoVariance = []

         # Avg values for expansions, time, and cost
        self.timeAvgNoVariance = 0
        self.expAvgNoVariance = 0
        self.costAvgNoVariance = 0

        self.timeAvgVariance = 0
        self.expAvgVariance = 0
        self.costAvgVariance = 0

        self.epsilon = epsilon

# This class is a container for everything pertaining to a specific scheduler
class SchedulerCSVDataContainer:
    def __init__(self):

        # List to keep track of epsilon values
        self.epsilonDict = {}

        # number of instances each tracker has, so we can remove those 
        # that don't occur from the plot
        self.instances = 0
        self.epsilonConstant = False

    # Update all the lists.
    def updateData(self, expansions, time, variance, epsilon, cost):
        # Determine if variance was checked to put the data in the 
        # correct list
        if(epsilon not in self.epsilonDict.keys()):
            self.epsilonDict[epsilon] = EpsilonDataContainter(epsilon)
            if(len(self.epsilonDict.keys()) > 1):
                self.epsilonConstant = False

        if(variance == "True"):
            self.epsilonDict[epsilon].timeListVariance.append(time)
            self.epsilonDict[epsilon].expansionsListVariance.append(expansions)
            self.epsilonDict[epsilon].costListVariance.append(cost)
        else:
            self.epsilonDict[epsilon].timeListNoVariance.append(time)
            self.epsilonDict[epsilon].expansionsListNoVariance.append(expansions)
            self.epsilonDict[epsilon].costListNoVariance.append(cost)

        # Increment the number of instances for this scheduler
        self.instances += 1
    
    def avgLists(self):
            # Average all of the lists for each epsilon value
            for epsilon in self.epsilonDict.keys():
                lengthNoVariance = len(self.epsilonDict[epsilon].timeListNoVariance)
                lengthVariance = len(self.epsilonDict[epsilon].timeListVariance)
                if(lengthVariance > 0):
                    self.epsilonDict[epsilon].timeAvgVariance = sum(self.epsilonDict[epsilon].timeListVariance)/lengthVariance
                    self.epsilonDict[epsilon].expAvgVariance = sum(self.epsilonDict[epsilon].expansionsListVariance)/lengthVariance
                    self.epsilonDict[epsilon].costAvgVariance = sum(self.epsilonDict[epsilon].costListVariance)/lengthVariance
                if(lengthNoVariance > 0):
                    self.epsilonDict[epsilon].timeAvgNoVariance = sum(self.epsilonDict[epsilon].timeListNoVariance)/lengthNoVariance
                    self.epsilonDict[epsilon].expAvgNoVariance = sum(self.epsilonDict[epsilon].expansionsListNoVariance)/lengthNoVariance
                    self.epsilonDict[epsilon].costAvgNoVariance = sum(self.epsilonDict[epsilon].costListNoVariance)/lengthNoVariance
                
class Plotter:
    def __init__(self, fileName, barGraph = False):
        # A dictionary to store all the containers for each scheduler
        self.scheduler = {
            "Individual A* DTS": SchedulerCSVDataContainer(),
            "Individual Greedy DTS": SchedulerCSVDataContainer(),
            "Shared MultiHeuristic A*": SchedulerCSVDataContainer(),
            "EISMHA": SchedulerCSVDataContainer(),
            "Shared MultiHeuristic Greedy Best First Search": SchedulerCSVDataContainer()
        }

        # Filename we're parsing
        self.fileName = fileName
        self.barGraph = barGraph

    # If epsilon is constant, then we are plotting to compare EISMHA and other algorithms
    # if epsilon is NOT constant, then we are comparing the performance of EISMHA with itself across different 
    # epsilon values
    def plot(self):
        with open(self.fileName) as csvfile:
            spamreader = csv.reader(csvfile, delimiter=',')
            # Boolean cause we don't want the header
            firstRow = True 
            for row in spamreader:
                #first row is the header, so we avoid it
                if(not firstRow):
                    # If this is a row of data, extract the data from the indexes
                    expansions = int(row[0])
                    time = float(row[1])
                    schedulerName = row[4]
                    variance = row[5]
                    epsilon = row[6]
                    cost = float(row[7])
                    # Use this new data to update the information about the scheduler
                    self.scheduler[schedulerName].updateData(expansions, time, variance, epsilon, cost)
                firstRow = False


        # avg the values for each scheduler
        # must use sorted keys so that everything lines up for the plot, otherwise it is inconsistent (property of hashmap)
        sortedDict = list(sorted(self.scheduler.keys()))
        print(sortedDict)
        epsilons = self.scheduler[sortedDict[0]].epsilonDict.keys()
        print(epsilons)
        EISMHASpecificPerformancsList = []
        #Plot performance for each epsilon across all schedulers
        for epsilon in epsilons:
                # I put these in a specific order as dictated by sorted(keys)
            schedulerNames = ["EISMHA*","Individual A* DTS", "Individual Greedy DTS", "SMHA* RR", "SMHGBFS RR"]
            indexesToRemove = []
            # relevant lists for plotting averaged data without variance
            expPlotListNoVariance = []
            timePlotListNoVariance = []
            costPlotListNoVariance = []

            # relevant lists for plotting averaged data with variance
            expPlotListVariance = []
            timePlotListVariance = []
            costPlotListVariance = []
            for s in range(len(sortedDict)):
                #remove from the names list if it is not in the CSV
                print(sortedDict[s], self.scheduler[sortedDict[s]].instances)
                if(self.scheduler[sortedDict[s]].instances == 0):
                    indexesToRemove.append(s)
                    continue
                else:
                    # Average all the lists for that scheduler
                    self.scheduler[sortedDict[s]].avgLists()
                    print(self.scheduler[sortedDict[s]].epsilonDict.keys())

                    #append its averages to the appropriate list for plotting
                    # If we have EISMHA, we actially care about the epsilon
                    if(sortedDict[s] == "EISMHA"):
                        # Variance Lists
                        expPlotListVariance.append(self.scheduler[sortedDict[s]].epsilonDict[epsilon].expAvgVariance)
                        timePlotListVariance.append(self.scheduler[sortedDict[s]].epsilonDict[epsilon].timeAvgVariance)
                        costPlotListVariance.append(self.scheduler[sortedDict[s]].epsilonDict[epsilon].costAvgVariance)

                        # No Variance Lists
                        expPlotListNoVariance.append(self.scheduler[sortedDict[s]].epsilonDict[epsilon].expAvgNoVariance)
                        timePlotListNoVariance.append(self.scheduler[sortedDict[s]].epsilonDict[epsilon].timeAvgNoVariance)
                        costPlotListNoVariance.append(self.scheduler[sortedDict[s]].epsilonDict[epsilon].costAvgNoVariance)
                        EISMHASpecificPerformancsList.append([epsilon, self.scheduler[sortedDict[s]].epsilonDict[epsilon].expAvgVariance,
                                                                self.scheduler[sortedDict[s]].epsilonDict[epsilon].timeAvgVariance,
                                                                self.scheduler[sortedDict[s]].epsilonDict[epsilon].costAvgVariance,
                                                                self.scheduler[sortedDict[s]].epsilonDict[epsilon].expAvgNoVariance,
                                                                self.scheduler[sortedDict[s]].epsilonDict[epsilon].timeAvgNoVariance,
                                                                self.scheduler[sortedDict[s]].epsilonDict[epsilon].costAvgNoVariance])
                    else:
                        # For every other algorithm, we can choose a generic epsilon:
                        genericEpsilon = list(sorted(self.scheduler[sortedDict[s]].epsilonDict.keys()))[0]
                        print(genericEpsilon)
                         # Variance Lists
                        expPlotListVariance.append(self.scheduler[sortedDict[s]].epsilonDict[genericEpsilon].expAvgVariance)
                        timePlotListVariance.append(self.scheduler[sortedDict[s]].epsilonDict[genericEpsilon].timeAvgVariance)
                        costPlotListVariance.append(self.scheduler[sortedDict[s]].epsilonDict[genericEpsilon].costAvgVariance)

                        # No Variance Lists
                        expPlotListNoVariance.append(self.scheduler[sortedDict[s]].epsilonDict[genericEpsilon].expAvgNoVariance)
                        timePlotListNoVariance.append(self.scheduler[sortedDict[s]].epsilonDict[genericEpsilon].timeAvgNoVariance)
                        costPlotListNoVariance.append(self.scheduler[sortedDict[s]].epsilonDict[genericEpsilon].costAvgNoVariance)
            # Reverse the list so we remove backwards (otherwise out of bounds)
            indexesToRemove.reverse()
            # Delete the names that don't appear in the CSV from the list
            for index in indexesToRemove:
                print("Deleting:", schedulerNames[index])
                del schedulerNames[index]
            if (self.barGraph):
                barWidth = .10
                plt.figure()
                plt.grid(linestyle='--', linewidth=.5)
                # Plot bar graph of Avg Expansions with and without variance
                # These first two lists are the positions of the bar. I place two next to eachother using barWidth/2
                #If there is no data for variance, or vice versa, just center the label.
                if(sum(expPlotListNoVariance) == 0):
                    barsVariancePos = [i+barWidth for i, _ in enumerate(schedulerNames)]
                    plt.bar(barsVariancePos, expPlotListVariance,color = 'b', width = barWidth, align='center')
                    plt.legend(["Variance"], loc=0)
                elif(sum(expPlotListVariance) == 0):
                    barsNoVariancePos = [i+barWidth for i, _ in enumerate(schedulerNames)]
                    plt.bar(barsNoVariancePos, expPlotListNoVariance,color = 'r', width = barWidth, align='center')
                    plt.legend(["No Variance"], loc=0)

                #Otherwise we have data for both, so lets center it between them
                else:
                    barsVariancePos = [i for i, _ in enumerate(schedulerNames)]
                    barsNoVariancePos = [i+barWidth/2 for i, _ in enumerate(schedulerNames)]
                    plt.bar(barsVariancePos, expPlotListVariance,color = 'b', width = barWidth, align='center')
                    plt.bar(barsNoVariancePos, expPlotListNoVariance,color = 'r', width = barWidth, align='edge')
                    plt.legend(["Variance", "No Variance"], loc=0)
                plt.xticks(barsNoVariancePos, schedulerNames)
                plt.xlabel('Scheduler Used')
                plt.ylabel('Expansions Made')
                plt.title('Average Number of Expansions, Epsilon: ' + str(epsilon))
                plt.show(block = False)

                plt.figure()
                plt.grid(linestyle='--', linewidth=.5)
                # # Plot bar graph of avg time with and without variance
                if(sum(timePlotListNoVariance) == 0):
                    plt.bar(barsVariancePos, timePlotListVariance,color = 'b', width = barWidth, align='center')
                    plt.legend(["Variance"], loc=0)
                elif(sum(timePlotListVariance) == 0):
                    plt.bar(barsNoVariancePos, timePlotListNoVariance,color = 'r', width = barWidth, align='center')
                    plt.legend(["No Variance"], loc=0)
                else:
                    plt.bar(barsVariancePos, timePlotListVariance,color = 'b', width = barWidth, align='center')
                    plt.bar(barsNoVariancePos, timePlotListNoVariance,color = 'r', width = barWidth, align='edge')
                    plt.legend(["Variance", "No Variance"], loc=0)
                plt.xticks(barsNoVariancePos, schedulerNames)
                plt.xlabel('Scheduler Used')
                plt.ylabel('Execution Time (sec)')
                plt.title('Average Execution Time, Epsilon: ' + str(epsilon))
                plt.show(block = False)

                # # Plot bar graph of avg cost with and without variance
                plt.figure()
                plt.grid(linestyle='--', linewidth=.5)
                if(sum(costPlotListNoVariance) == 0):
                    plt.bar(barsVariancePos, costPlotListVariance,color = 'b', width = barWidth, align='center')
                    plt.legend(["Variance"], loc=0)
                elif(sum(costPlotListVariance) == 0):
                    plt.bar(barsNoVariancePos, costPlotListNoVariance,color = 'r', width = barWidth, align='center')
                    plt.legend(["No Variance"], loc=0)
                else:
                    plt.bar(barsVariancePos, costPlotListVariance,color = 'b', width = barWidth, align='center')
                    plt.bar(barsNoVariancePos, costPlotListNoVariance,color = 'r', width = barWidth, align='edge')
                    plt.legend(["Variance", "No Variance"], loc=0)
                plt.xticks(barsNoVariancePos, schedulerNames)
                plt.xlabel('Scheduler Used')
                plt.ylabel('Path Cost (Units)')
                plt.title('Average Path Cost, Epsilon: ' + str(epsilon))
                plt.show(block = False)

        # If we have more than 1 epsilon
        if(len(EISMHASpecificPerformancsList) > 1):
            #Plot performance of just EISMHA for each epsilon
            epsilonValues = []
            timesVariance = []
            costsVariance = []
            expansionsVariance = []
            timesNoVariance = []
            costsNoVariance = []
            expansionsNoVariance = []
            for dataPoint in EISMHASpecificPerformancsList:
                epsilonValues.append(dataPoint[0])
                expansionsVariance.append(dataPoint[1])
                timesVariance.append(dataPoint[2])
                costsVariance.append(dataPoint[3])
                expansionsNoVariance.append(dataPoint[4])
                timesNoVariance.append(dataPoint[5])
                costsNoVariance.append(dataPoint[6])

            # Sort the two lists together
            zipped_lists = zip(epsilonValues, expansionsNoVariance)
            sorted_pairs = sorted(zipped_lists)

            tuples = zip(*sorted_pairs)
            plt.figure()
            epsilonValues, expansionsNoVariance = [ list(tuple) for tuple in  tuples]

            plt.plot(epsilonValues, expansionsNoVariance, linewidth = 2)

            labels = ["EISMHA*"]
            for i in range(1, len(expPlotListNoVariance)):
                plt.plot([0.0, epsilonValues[-1]], [expPlotListNoVariance[i], expPlotListNoVariance[i]], linewidth=2)
                labels.append(sortedDict[i])
                plt.legend(labels, loc = 0)
            plt.xlabel('Epsilon Value')
            plt.ylabel('Number of Expansions')
            plt.title('Expansions vs Epsilon')
            plt.show(block = False)
        plt.show()

data = Plotter('WackyMaze_WackyMaze.csv', barGraph = True)
data.plot()

