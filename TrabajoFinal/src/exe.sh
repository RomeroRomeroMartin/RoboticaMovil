#!/bin/bash

if [[ $2 = 1 ]]
then
make clean
rm -r datos
mkdir datos
fi



make

# args--> 1 mapa, 2

for i in $(seq 1 200)
do
    echo  "GENERACION" $i
    var=$RANDOM
    # echo $RANDOM

    ./OARobot_VFH datos/data_VFH_$i.dat $1 $var $3
    # python plot.py datos/data_VFH.dat

    ./OARobot_CVM datos/data_CVM_$i.dat $1 $var $3
    # python plot.py datos/data_CVM.dat

    # ./COMPARA
    ./OARobot_PFM datos/data_PFM_$i.dat $1 $var $3
done
python plot.py 