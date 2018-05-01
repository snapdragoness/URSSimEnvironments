#!/bin/sh

# Running CPA(H) with sicstus
#/usr/bin/time -f "Total time: %e (seconds)" -o result-time ./sic_script.sh $1 $2

# Running CPA(H) with swi-prolog
/usr/bin/time "Total time: %e (seconds)" -o result-time ./swi_script.sh $1 $2

cat result-time
