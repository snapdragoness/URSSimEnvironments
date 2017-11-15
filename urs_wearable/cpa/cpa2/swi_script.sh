#!/bin/sh

../pddl2a/cpa.pddl2pl urs.pddl

cat ../experiments/mult5zswi.pl pddl2pl.pl > new.pl
swipl -L128m -G128m -f new.pl -g 'main,halt.' > trash

#../cpa/cpa+ theory_names > temp

#sed -e 's/cpa_//g' temp > plan-result

#cat plan-result
