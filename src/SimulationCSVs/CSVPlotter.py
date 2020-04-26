import matplotlib.pyplot as plt
import csv

# This class is a container for everything pertaining to a specific scheduler
class SchedulerCSVDataContainer:
    def __init__(self, epsilonConstant = True):
        #Lists for keeping track of time with and without variance
        self.timeListVariance = []
        self.timeListNoVariance = []

        #Lists for keeping track of expansions with and without variance
        self.expansionsListNoVariance = []
        self.expansionsListVariance = []

        #Lists for keeping track of cost with and without variance
        self.costListVariance = []
        self.costListNoVariance = []

        # List to keep track of epsilon values
        self.epsilonDict = {}

        # Avg values for expansions, time, and cost
        self.timeAvgNoVariance = 0
        self.expAvgNoVariance = 0
        self.costAvgNoVariance = 0

        self.timeAvgVariance = 0
        self.expAvgVariance = 0
        self.costAvgVariance = 0

        # number of instances each tracker has, so we can remove those 
        # that don't occur from the plot
        self.instances = 0

        self.epsilonConstant = epsilonConstant

    # Update all the lists.
    def updateData(self, expansions, time, variance, epsilon, cost):
        # Determine if variance was checked to put the data in the 
        # correct list
        if(variance == "True"):
            self.timeListVariance.append(time)
            self.expansionsListVariance.append(expansions)
            self.costListVariance.append(cost)
        else:
            self.timeListNoVariance.append(time)
            self.expansionsListNoVariance.append(expansions)
            self.costListNoVariance.append(cost)
        
        # if epsilon is not constant, associate all these lists with the specific epsilon value
        if(not epsilonConstant):
            self.epsilonDict[epsilon] = [self.timeListNoVariance, self.timeListVariance, 
                                    self.expansionsListNoVariance, self.expansionsListVariance,
                                    self.costListNoVariance, self.costListVariance]

        # Increment the number of instances for this scheduler
        self.instances += 1
    
    def avgLists(self):
        if(self.epsilonConstant):
            # Average the relevant time and expasion data
            lengthNoVariance = len(self.timeListNoVariance)
            lengthVariance = len(self.timeListVariance)
            if(lengthVariance > 0):
                self.timeAvgVariance = sum(self.timeListVariance)/lengthVariance
                self.expAvgVariance = sum(self.expansionsListVariance)/lengthVariance
                self.costAvgVariance = sum(self.costListVariance)/lengthVariance
            if(lengthNoVariance > 0):
                self.timeAvgNoVariance = sum(self.timeListNoVariance)/lengthNoVariance
                self.expAvgNoVariance = sum(self.expansionsListNoVariance)/lengthNoVariance
                self.costAvgNoVariance = sum(self.costListNoVariance)/lengthVariance

        else:
            # For each epsilon value, avg the value
            for epsilon in self.epsilonDict.keys():
                lengthNoVariance = len(self.epsilonDict[epsilon][0])
                lengthVariance = len(self.epsilonDict[epsilon][1])
                if(lengthVariance > 0):
                    self.timeAvgVariance = sum(self.timeListVariance)/lengthVariance
                    self.expAvgVariance = sum(self.expansionsListVariance)/lengthVariance
                    self.costAvgVariance = sum(self.costListVariance)/lengthVariance
                if(lengthNoVariance > 0):
                    self.timeAvgNoVariance = sum(self.timeListNoVariance)/lengthNoVariance
                    self.expAvgNoVariance = sum(self.expansionsListNoVariance)/lengthNoVariance
                    self.costAvgNoVariance = sum(self.costListNoVariance)/
                
                # This averages all the lists and stores them back in the dictionary instead of 
                # the lists
                self.epsilonDict[epsilon] = [self.timeAvgNoVariance, self.timeAvgVariance, 
                                    self.expAvgNoVariance, self.expAvgVariance,
                                    self.costAvgNoVariance, self.costAvgVariance]
class Plotter:
    def __init__(self, fileName, epsilonConstant = True):
        # A dictionary to store all the containers for each scheduler
        self.scheduler = {
            "Individual A* DTS": SchedulerCSVDataContainer(epsilonConstant),
            "Individual Greedy DTS": SchedulerCSVDataContainer(epsilonConstant),
            "Shared MultiHeuristic A*": SchedulerCSVDataContainer(epsilonConstant),
            "EISMHA": SchedulerCSVDataContainer(epsilonConstant),
            "Shared MultiHeuristic Greedy Best First Search": SchedulerCSVDataContainer(epsilonConstant)
        }

        # Filename we're parsing
        self.fileName = fileName
        self.epsilonConstant = epsilonConstant


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
                    cost = row[7]
                    # Use this new data to update the information about the scheduler
                    self.scheduler[schedulerName].updateData(expansions, time, variance, epsilon, cost)
                firstRow = False



        # relevant lists for plotting averaged data without variance
        expPlotListNoVariance = []
        timePlotListNoVariance = []
        costPlotListNoVariance = []

        # relevant lists for plotting averaged data with variance
        expPlotListVariance = []
        timePlotListVariance = []
        costPlotListVariance = []
        if(self.epsilonConstant):

            # avg the values for each scheduler
            # must use sorted keys so that everything lines up for the plot, otherwise it is inconsistent (property of hashmap)
            sortedDict = list(sorted(self.scheduler.keys()))

            # I put these in a specific order as dictated by sorted(keys)
            schedulerNames = ["EISMHA*","Individual A* DTS", "Individual Greedy DTS", "SMHA* RR", "SMHGBFS RR"]
            indexesToRemove = []

            for s in range(len(sortedDict)):
                #remove from the names list if it is not in the CSV
                if(self.scheduler[sortedDict[s]].instances == 0):
                    indexesToRemove.append(s)
                    continue
                else:
                    # Average all the lists for that scheduler
                    self.scheduler[sortedDict[s]].avgLists()
                    
                    #append its averages to the appropriate list for plotting
                    # Variance Lists
                    expPlotListVariance.append(self.scheduler[sortedDict[s]].expAvgVariance)
                    timePlotListVariance.append(self.scheduler[sortedDict[s]].timeAvgVariance)
                    costPlotListVariance.append(self.scheduler[sortedDict[s]].costAvgVariance)

                    # No Variance Lists
                    expPlotListNoVariance.append(self.scheduler[sortedDict[s]].expAvgNoVariance)
                    timePlotListNoVariance.append(self.scheduler[sortedDict[s]].timeAvgNoVariance)
                    costPlotListNoVariance.append(self.scheduler[sortedDict[s]].costAvgNoVariance)

            # Reverse the list so we remove backwards (otherwise out of bounds)
            indexesToRemove.reverse()
            # Delete the names that don't appear in the CSV from the list
            for index in indexesToRemove:
                del schedulerNames[index]

            barWidth = .10

            plt.grid(linestyle='--', linewidth=.5)
            
            # Plot bar graph of Avg Expansions with and without variance
            # These first two lists are the positions of the bar. I place two next to eachother using barWidth/2
            barsVariancePos = [i for i, _ in enumerate(schedulerNames)]
            barsNoVariancePos = [i+barWidth/2 for i, _ in enumerate(schedulerNames)]
            plt.bar(barsVariancePos, expPlotListVariance,color = 'b', width = barWidth, align='center')
            plt.bar(barsNoVariancePos, expPlotListNoVariance,color = 'r', width = barWidth, align='edge')
            plt.xticks(barsNoVariancePos, schedulerNames)
            plt.xlabel('Scheduler Used')
            plt.ylabel('Expansions Made')
            plt.title('Average Number of Expansions')
            plt.legend(["Variance", "No Variance"],loc = 0)
            plt.show()

            plt.grid(linestyle='--', linewidth=.5)
            # # Plot bar graph of avg time with and without variance
            plt.bar(barsVariancePos, timePlotListVariance,color = 'b', width = barWidth, align='center')
            plt.bar(barsNoVariancePos, timePlotListNoVariance,color = 'r', width = barWidth, align='edge')
            plt.xticks(barsNoVariancePos, schedulerNames)
            plt.xlabel('Scheduler Used')
            plt.ylabel('Execution Time (sec)')
            plt.title('Average Execution Time')
            plt.legend(["Variance", "No Variance"], loc = 0)
            plt.show()

            # # Plot bar graph of avg cost with and without variance
            plt.bar(barsVariancePos, costPlotListVariance,color = 'b', width = barWidth, align='center')
            plt.bar(barsNoVariancePos, costPlotListNoVariance,color = 'r', width = barWidth, align='edge')
            plt.xticks(barsNoVariancePos, schedulerNames)
            plt.xlabel('Scheduler Used')
            plt.ylabel('Path Cost (Units)')
            plt.title('Average Path Cost')
            plt.legend(["Variance", "No Variance"], loc = 0)
            plt.show()

        else:
        # Plot avg expansions as epsilon changes for EISMHA*
            EISMHAData = self.scheduler["EISMHA"]
            # Format of epsilon data structure: [timeAvgNoVariance, timeAvgVariance, expAvgNoVariance, expAvgVariance, costAvgNoVariance, costAvgVariance]
            epsilonValues = []
            # List containing all epsilon plotting data
            for epsilon in EISMHAData.epsilonDict.keys():
                epsilonValues.append[epsilon]
                timePlotListNoVariance.append(EISMHAData.epsilonDict[epsilon][0])
                timePlotListVariance.append(EISMHAData.epsilonDict[epsilon][1])
                expPlotListNoVariance.append(EISMHAData.epsilonDict[epsilon][2])
                expPlotListVariance.append(EISMHAData.epsilonDict[epsilon][3])
                costPlotListNoVariance.append(EISMHAData.epsilonDict[epsilon][4])
                costPlotListVariance.append(EISMHAData.epsilonDict[epsilon][5])
            
            plt.plot(epsilonValues, timePlotListNoVariance, marker = 'o', label = 'Execution Time')

data = Plotter('csvPlot_both2_aLittleBitOfEverything2.csv', epsilonConstant = True)
data.plot()

