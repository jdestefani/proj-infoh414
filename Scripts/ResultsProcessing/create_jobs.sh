#!/bin/bash

# cleans the data about the old runs
function clean_old_runs() {
    echo "*****************************************"
    echo -n "Cleaning old xml and runtime files... "
    echo "*****************************************"
    rm -rf $1 || exit 1
    rm -rf $2 || exit 1
    echo "done"
    echo
}

####################
####################

# creates the needed directories
function setup_jobs() {
    echo "******************************"
    echo "Creating results directory: $1"
    echo "******************************"
    mkdir -p $1 || exit 1
    echo
    echo "********************************"
    echo "Creating configuration files: $2"
    echo "********************************"
    echo 
    #If XMLDIR does not exists, create it.
	if [ ! -d "$2" ];
	then
		mkdir -p $2
	fi
}

####################
####################

echo ""
echo "################################################################"
echo "			JOB CREATION SCRIPT"
echo "################################################################"
echo ""

# Random is has to be given as the only parameter
if [ $# -ne 5 ]; then
            echo "[ERROR] Usage - $0 <experiment_name> <template_file> <swarm_size> <nruns> <seed>"
            exit 1
else
EXPERIMENT_NAME=${1}
TEMPLATE_FILE=${2}
SWARM_SIZE=${3}
RUNS=${4}
fi


#Case cluster
#USERNAME=`whoami`
#RESULTSDIR="/home/$USERNAME/argos2/user/$USERNAME/results/"
#XMLDIR=${RESULTSDIR}${EXPERIMENT_NAME}"/xml/"

#Case local and cluster
USERNAME=`whoami`
RESULTSDIR="/home/${USERNAME}/workspace/INFO-H-414/Results/"
XMLDIR=${RESULTSDIR}${EXPERIMENT_NAME}"/xml/"

#clean_old_runs $XMLDIR$EXPERIMENT_NAME $XMLDIR$EXPERIMENT_NAME 

#Create a folder named ${EXPERIMENT_NAME} in both the /xml and /experiments subdirectory of the user folder to contain respectively, the configuration file and the results of the experiment
setup_jobs ${RESULTSDIR}${EXPERIMENT_NAME}"/" ${XMLDIR}

for counter in `seq 1 $RUNS`
do
	SEED=${RANDOM}
    #Build the configuration file name by composing the directory, the experiment name and the parameters to the experiment 
    PARAMETERS=${SWARM_SIZE}"_seed-"${RANDOM}
    XML_FILENAME=${XMLDIR}${EXPERIMENT_NAME}${PARAMETERS}".xml"
    
	echo "Generating XML file: "${XML_FILENAME}
	echo ${TEMPLATE_FILE}
        #Reg-exp substitution to actually create the configuration file
	touch ${XML_FILENAME}
        sed -e "s/#seed#/${counter}/"                    				\
        -e "s/#swarm_size#/${SWARM_SIZE}/"         						\
        -e "s/#output_file#/${EXPERIMENT_NAME}${PARAMETERS}.txt/"   \
        < ${TEMPLATE_FILE} > ${XML_FILENAME}
	
	echo "Executing simulation:"
	echo ""
	#Launch of each job separately using the xml file generated in the previous step
	argos3 -c ${XML_FILENAME}
 	#echo "qsub ./run_job.sh ${XML_FILENAME} ${EXPERIMENT_NAME}${PARAMETERS} ${RESULTSDIR}${EXPERIMENT_NAME}"
 	echo ""
        
        
done

