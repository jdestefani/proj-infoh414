#!/bin/bash

for XMLFILE in `ls Results/project/xml`
do
	argos3 -c Results/project/xml/${XMLFILE}
done

exit 0
