#!/bin/bash
#$ -N epuck_coverage
#$ -m as
#$ -M jacopo.de.stefani@ulb.ac.be
#$ -cwd

# Parameters
#1 : Configuration file
#2 : Experiment name
#3 : Output dir

# Needs to be exported at each execution
export ARGOSINSTALLDIR="/home/jdestefani/argos2/"

EXPERIMENT_NAME=$2
USERNAME=`whoami`
#USERNAME="jdestefani"
TMPDIR=/tmp/$USERNAME/experiments/$EXPERIMENT_NAME
OUTPUTDIR=$3
COMMAND=/home/$USERNAME/argos2/build/simulator/argos

#Create /tmp/$USERNAME/experiments/$EXPERIMENTNAME and copy the content of the current directory 
mkdir -p $TMPDIR
cp $1 $TMPDIR
cd $TMPDIR

#Execute command and store return value
COMPLETE_COMMAND=${COMMAND}" -c "$2".xml"
${COMPLETE_COMMAND} &> /dev/null
RET=$?

#Move all the data produced by the experiment back to home folder
mv *.occ *.wt $OUTPUTDIR
cd $OUTPUTDIR

#Cleanup temp directory
#rmdir -p $TMPDIR &> /dev/null
rm -rf $TMPDIR 


exit $RET
