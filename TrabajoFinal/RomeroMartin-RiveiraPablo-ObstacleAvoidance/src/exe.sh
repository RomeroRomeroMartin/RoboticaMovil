#!/bin/bash

if [[ $2 = 1 ]]
then
make clean
rm -r datos
mkdir datos
fi



make 

# args binarios--> 1 mapa, 2 clean y rm, 3 mostrar simulacion o no

for i in $(seq 1 200)
do
    echo  "GENERACION" $i
    var=$RANDOM

    ./OARobot_VFH datos/data_VFH_$i.dat $1 $var $3

    ./OARobot_CVM datos/data_CVM_$i.dat $1 $var $3

    ./OARobot_PFM datos/data_PFM_$i.dat $1 $var $3
done
python plot.py