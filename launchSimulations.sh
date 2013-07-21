#!/bin/bash

for XMLFILE in `ls Results/$1/xml`
do
	argos3 -c Results/$1/xml/${XMLFILE}
done

exit 0
