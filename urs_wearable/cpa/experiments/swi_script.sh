#!/bin/sh

#echo 
#echo Creating dp.pddl file from $1 and $2
#cat $1 $2 > dp.pddl

#../pddl2a/cpa.pddl2pl dp.pddl
../pddl2a/cpa.pddl2pl $1

echo Performing statistical analysis ...

cat mult5zswi.pl pddl2pl.pl > new.pl
swipl -L128m -G128m -f new.pl -g 'main,halt.' > trash 


echo Running CPA+ ...
../cpa/cpa+ theory_names > temp

#echo "Running DNFct  ..."
#./DNFct theory_0.al > temp

sed -e 's/cpa_//g' temp > plan-result

cat plan-result 

