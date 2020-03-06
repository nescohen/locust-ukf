#! /bin/bash

projRoot="$(git rev-parse --show-toplevel)"

pushd "${projRoot}/test" > /dev/null

bash build.sh

valgrind --error-exitcode=1 --tool=cachegrind --branch-sim=yes --cachegrind-out-file=cachegrind.out -- ./test.bin > /dev/null

# Make paths relative
sed -i -e "s|${projRoot}"'/test/src/||g' cachegrind.out

cg_annotate --sort=D1mr,DLmr,Bcm --show=D1mr,DLmr,Bcm --include="${projRoot}/test/src/" cachegrind.out math/matrix_util.c

popd > /dev/null
