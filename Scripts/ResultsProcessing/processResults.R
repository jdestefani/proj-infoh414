library(ggplot2)

boxplotToPdf <- function(file,data,stringTitle,stringXLabel,color,pdfHeight,pdfWidth){
  pdf(file)
  boxplot(as.data.frame(data),horizontal=TRUE,main=stringTitle,col=color,las=1)
  title(xlab=stringXLabel)
  dev.off()
}

extractMetrics <- function(inputFile){
  print(inputFile)
  #Read data
  inputData <- read.table(inputFile, header = TRUE)
  
  #Extract the number of robots in the chain at the last time step
  robotsInChain <- inputData[dim(inputData)[1],2]
  
  #Extract the completion time
  completionTime <- inputData[dim(inputData)[1],1]
  
  returnValues <- c(as.numeric(robotsInChain),as.numeric(completionTime))
  return(returnValues)
}

padVector <- function(vector,desiredLength){
  if(length(vector) < desiredLength){
    paddingLength = desiredLength - length(vector)
    return(append(vector,rep(vector[length(vector)],paddingLength)))
  }
  return(vector)
}

extractColumn <- function(inputFile,index,maxDuration){
  #Read data
  inputData <- read.table(inputFile, header = TRUE)
  return(padVector(inputData[,index],maxDuration))
}


plotRobotInChain <- function(inChainPlot){
  #Plot max error 
  pdf("RobotInChain.pdf")
  xRange <- range(0,dim(inChainPlot)[1])
  yRange <- range(0,50)
  nPlots <- dim(inChainPlot)[2]
  plotColors <- rainbow(nPlots) 
  lineType <- c(1,2,4,2,1,5) 
  plotChar <- seq(18,18+nPlots,1)
  
  plot(xRange,yRange, type="n", xlab="Simulation steps", ylab="Robots in chain",main=paste("Time evolution of number of robots in chain across",length(resultsFiles),"trials"))   
  
  #Add lines
  matlines(inChainPlot, type="l" , lwd=1.5, lty=lineType, col=plotColors, pch=plotChar)
  
  #Add a title
  #title(graphTitle)
  
  #Add a legend 
  legend(x="bottomleft", colnames(inChainPlot), cex=0.8, col=plotColors, lty=lineType, title="Percentiles")
  dev.off()
  
  write.table(inChainPlot,"RobotInChainPlots.plt",sep="\t",row.names=FALSE,col.names=TRUE)
}

plotRobotOnTarget <- function(onSpotPlot){
  #Plot max error 
  pdf("RobotOnTarget.pdf")
  xRange <- range(0,dim(onSpotPlot)[1])
  yRange <- range(0,7)
  nPlots <- dim(onSpotPlot)[2]
  plotColors <- rainbow(nPlots) 
  lineType <- c(1,2,4,2,1,5) 
  plotChar <- seq(18,18+nPlots,1)
  
  plot(xRange,yRange, type="n", xlab="Simulation steps", ylab="Robots on spot",main=paste("Time evolution of number of robots on target across",length(resultsFiles),"trials"))   

  #Add lines
  matlines(onSpotPlot, type="l" , lwd=1.5, lty=lineType, col=plotColors, pch=plotChar)

  #Add a title
  #title(graphTitle)

  #Add a legend 
  legend(x="bottomleft", colnames(onSpotPlot), cex=0.8, col=plotColors, lty=lineType, title="Percentiles")
  dev.off()

  write.table(onSpotPlot,"RobotOnSpotPlots.plt",sep="\t",row.names=FALSE,col.names=TRUE)
}


#computeQuantileByRow <- function(matrix){
#  return t(apply(matrix,1,quantile))
#}
#############################################################################


#Set the directory containing the experiment results as current working directory
setwd("Results/ResultsDS")
#List all the files containing information about occupation
resultsFiles <- list.files(pattern = "\\.txt$")
#Process every single file separately
print("[STATUS] - Processing results file")
metrics <- sapply(resultsFiles,extractMetrics)
#Transform processing results into dataframe
metrics <- as.matrix(metrics)
rownames(metrics) <- c("Robots in Chain","Completion Time")
duration <- max(metrics["Completion Time",])

inChain <- lapply(resultsFiles,extractColumn,index=2,maxDuration=duration)
onTarget <- lapply(resultsFiles,extractColumn,index=3,maxDuration=duration)
inChain <- data.frame(matrix(unlist(inChain), ncol=length(inChain)))
onTarget <- data.frame(matrix(unlist(onTarget), ncol=length(onTarget)))

#print("[STATUS] - Computing quantiles & mean")
#Given the matrix containing the integrated errors for each trial as column, compute the quantiles per row.
#That corresponds to obtain the quantiles at each time step
inChainPlot <- t(apply(inChain,1,quantile))
onTargetPlot <- t(apply(onTarget,1,quantile))
#Compute row by row mean of the integrated error of every single trial
meanInChain <- apply(inChain,1,mean)
meanOnTarget <-apply(onTarget,1,mean)
#Bind toghether the elements
inChainPlot <- cbind(inChainPlot,meanInChain)
colnames(inChainPlot) <- c("0%","25%", "50%","75%","100%","Mean")
onTargetPlot <- cbind(onTargetPlot,meanOnTarget)
colnames(onTargetPlot) <- c("0%","25%", "50%","75%","100%","Mean")

print("[STATUS] - Create reporting data frame")
#Create data frame for reporting
report <- data.frame()
report["Mean","Robots in Chain"] <- mean(metrics["Robots in Chain",])
report["Mean","Completion Time"] <- mean(metrics["Completion Time",])
report["Standard Deviation","Robots in Chain"] <- sd(metrics["Robots in Chain",])
report["Standard Deviation","Completion Time"] <- sd(metrics["Completion Time",])
report["Median","Robots in Chain"] <- median(metrics["Robots in Chain",])
report["Median","Completion Time"] <- median(metrics["Completion Time",])
report["Min","Robots in Chain"] <- min(metrics["Robots in Chain",])
report["Min","Completion Time"] <- min(metrics["Completion Time",])
report["Max","Robots in Chain"] <- max(metrics["Robots in Chain",])
report["Max","Completion Time"] <- max(metrics["Completion Time",])
#Write reporting dataframe
write.table(report,"Results.stat",sep="\t",row.names=TRUE,col.names=TRUE)
#write(paste("Correlation (Pearson)",cor(x=metrics["Robots in Chain",],y=metrics["Completion Time",], use="all.obs", method="pearson"),sep="\t"),"Result.stat",append = TRUE)
#write(paste("Correlation (Spearman)",cor(x=metrics["Robots in Chain",],y=metrics["Completion Time",], use="all.obs", method="spearman"),sep="\t"),"Result.stat",append = TRUE)
#write(paste("Correlation (Kendall)",cor(x=metrics["Robots in Chain",],y=metrics["Completion Time",], use="all.obs", method="kendall"),sep="\t"),"Result.stat",append = TRUE)

#Plot box plots and empirical cumulative density functions
#boxplotToPdf("BoxplotTime.pdf",metrics["Completion Time",],paste("Boxplot of completion times across",length(resultsFiles),"trials"),"Simulation steps","blue")
#boxplotToPdf("BoxplotRobots.pdf",metrics["Robots in Chain",],paste("Boxplot of number of robots in chain across",length(resultsFiles),"trials"),"Number of robots","orange")

# Add boxplots to a scatterplot
# Scatterplot
pdf("ResultsDistribution.pdf")
par(fig=c(0,0.8,0,0.8), new=TRUE)
plot(x=metrics["Robots in Chain",],y=metrics["Completion Time",],col=rgb(0,100,0,80,maxColorValue=255), pch=19 , xlab="Robots in Chain",
     ylab="Completion Time")
# Fit lines
abline(lm(metrics["Completion Time",]~metrics["Robots in Chain",]), col="red") # regression line (y~x) 
lines(lowess(metrics["Robots in Chain",],metrics["Completion Time",]), col="blue") # lowess line (x,y)
# Boxplot X-Axis
par(fig=c(0,0.8,0.55,1), new=TRUE)
boxplot(metrics["Robots in Chain",], horizontal=TRUE, axes=FALSE)
# Boxplot Y-Axis
par(fig=c(0.65,1,0,0.8), new=TRUE)
boxplot(metrics["Completion Time",], axes=FALSE)
# Title
mtext(expression(bold("Results Distribution")), side=3, outer=TRUE, line=-3,cex=1.5)
dev.off()

#Plot empirical CDF of completion times
pdf("EcdfTime.pdf")
graphTitle <- paste("Empirical cdf of completion time across",length(resultsFiles),"trials")
plot(ecdf(metrics["Completion Time",]),col="blue",xlab="Simulation steps",main=paste("Empirical cdf of completion times across",length(resultsFiles),"trials"))
title(str(graphTitle))
dev.off()

#Plot empirical CDF of completion times
pdf("EcdfRobots.pdf")
graphTitle <- paste("Empirical cdf of completion time across",length(resultsFiles),"trials")
plot(ecdf(metrics["Robots in Chain",]),col="blue",xlab="Simulation steps",main=paste("Empirical cdf of completion times across",length(resultsFiles),"trials"))
title(str(graphTitle))
dev.off()

plotRobotInChain(inChainPlot)
plotRobotOnTarget(onTargetPlot)

#Output analysis result
#sprintf("Time required to get 25%% coverage: %.0f",firstQuartileOccupation[1])
#sprintf("Time required to get 50%% coverage: %.0f",secondQuartileOccupation[1])
#sprintf("Time required to get 75%% coverage: %.0f",thirdQuartileOccupation[1])

rm(metrics)
rm(report)
rm(resultsFiles)

#createTexGraph("t","$\\max_i(e_i(t))$",5,5,inputData[,1],maxErrorIntegrated[,1])
