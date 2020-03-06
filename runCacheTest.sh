#! /bin/bash

projRoot="$(git rev-parse --show-toplevel)"

pushd "${projRoot}/test" > /dev/null

bash build.sh

valgrind --error-exitcode=1 --tool=cachegrind --cachegrind-out-file=cachegrind.out -- ./test.bin > /dev/null

# Make paths relative
sed -i -e "s|${projRoot}"'/test/src/||g' cachegrind.out

cg_annotate --include="${projRoot}/test/src/" --show=D1mr,DLmr cachegrind.out math/matrix_util.c

popd > /dev/null
