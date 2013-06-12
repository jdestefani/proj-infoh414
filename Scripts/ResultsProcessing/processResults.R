relativeOccupationPercentile <- function(relativeOccupationMatrix,percentile){
  percentileOccupation <- relativeOccupationMatrix[relativeOccupationMatrix[,1]>=percentile &
                                                   relativeOccupationMatrix[,2]>=percentile &
                                                   relativeOccupationMatrix[,3]>=percentile &
                                                   relativeOccupationMatrix[,4]>=percentile,1]
  
  # If a certain percentile is never reached, 
  # return the time corresponding to the simulation duration + 1
  # to signal infeasibily, without occuring in NA.
  if(length(percentileOccupation) == 0){
    return(length(relativeOccupationMatrix[,1])+1)
  }
  else{
    return(percentileOccupation)
  }
}

boxplotToPdf <- function(file,data,stringTitle,stringXLabel,pdfHeight,pdfWidth){
  pdf(file)
  boxplot(as.data.frame(data),horizontal=TRUE,main=stringTitle,col=terrain.colors(length(colNames)),las=1)
  title(xlab=stringXLabel)
  dev.off()
}

extractMetrics <- function(inputFile){
  #Read data
  inputData <- read.table(inputFile, header = TRUE)
  
  #Extract the number of robots in the chain at the last time step
  robotsInChain <- inputData[dim(inputData)[1],2]
  
  #Extract the completion time
  completionTime <- inputData[dim(inputData)[1],1]
  
  returnValues <- list(robotsInChain,completionTime)
  return(returnValues)
}

createTexGraph <- function(xlabel,ylabel,height,width,xdataset,ydataset){
  #Output values to console -> Redirect to file to obtain a valid tex file
  sink("./results/Occupation.tex",append=TRUE)
  sprintf("\\begin{tikzpicture}\n")
  sprintf("  \\begin{axis}[\n")
  sprintf("	xlabel=%s,\n",xlabel)
  sprintf("	ylabel=%s$,\n",ylabel)
  sprintf("	height=%fcm,\n",height)
  sprintf("	width=%fcm,\n",width)
  sprintf("	grid=major,\n")
  sprintf("	legend style={at={(0,0)},\n")
  sprintf("	anchor=north,legend columns=-1},")
  sprintf("	]\n") 
  sprintf("\\addplot [color=green,mark=none,very thick] coordinates {\n")
  sprintf("(%d,%f)\n",xdataset,ydataset)
  sprintf("    };\n")
  #sprintf("\\addlegendentry{"+str(legend[index])+"}\n")
  #sprintf("\\legend{")
  #for element in legend:
  #  sprintf(str(element)+",")
  #sprintf("}\n") 
  sprintf("	\\end{axis}\n")
  sprintf("\\end{tikzpicture}\n")
}

#computeQuantileByRow <- function(matrix){
#  return t(apply(matrix,1,quantile))
#}
#############################################################################


#Set the directory containing the experiment results as current working directory
setwd("/home/deste/workspace/INFO-H-414/Results/")
#List all the files containing information about occupation
resultsFiles <- list.files(pattern = "\\.txt$")
#Process every single file separately
metrics <- lapply(resultsFiles,extractMetrics)
#Transform processing results into dataframe
metrics <- data.frame(matrix(unlist(metrics), nrow=length(metrics)))
colnames(metrics) <- c("Robots in Chain","Completion Time")
#Create data frame for reporting
report <- data.frame()
report["Robots in Chain","Mean"] <- mean(metrics[,"Robots in Chain"])
report["Completion Time","Mean"] <- mean(metrics[,"Completion Time"])
report["Robots in Chain","Standard Deviation"] <- sd(metrics[,"Robots in Chain"])
report["Completion Time","Standard Deviation"] <- sd(metrics[,"Completion Time"])
report["Robots in Chain","Median"] <- median(metrics[,"Robots in Chain"])
report["Completion Time","Median"] <- median(metrics[,"Completion Time"])
report["Robots in Chain","Min"] <- min(metrics[,"Robots in Chain"])
report["Completion Time","Min"] <- min(metrics[,"Completion Time"])
report["Robots in Chain","Max"] <- max(metrics[,"Robots in Chain"])
report["Completion Time","Max"] <- max(metrics[,"Completion Time"])

#Write reporting dataframe
write.table(report,"Results.stat",sep="\t",row.names=TRUE,col.names=TRUE)

#Plot box plots and empirical cumulative density functions
boxplotToPdf("BoxplotTime.pdf",metrics[,"Completion Time"],paste("Boxplot of completion times across",length(resultsFiles),"trials"),"Simulation steps")
boxplotToPdf("BoxplotRobots.pdf",metrics[,"Robots in chain"],paste("Boxplot of number of robots in chain across",length(resultsFiles),"trials"),"Number of robots")

#Plot empirical CDF of completion times
pdf("EcdfTime.pdf")
graphTitle <- paste("Empirical cdf of completion time across",length(resultsFiles),"trials")
plot(ecdf(metrics[,"Completion Time"]),col="orange",xlab="Simulation steps")
title(str(graphTitle))
dev.off()


#Output analysis result
#sprintf("Time required to get 25%% coverage: %.0f",firstQuartileOccupation[1])
#sprintf("Time required to get 50%% coverage: %.0f",secondQuartileOccupation[1])
#sprintf("Time required to get 75%% coverage: %.0f",thirdQuartileOccupation[1])

rm(metrics)
rm(report)
rm(resultsFiles)

#createTexGraph("t","$\\max_i(e_i(t))$",5,5,inputData[,1],maxErrorIntegrated[,1])
