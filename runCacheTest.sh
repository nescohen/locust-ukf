#! /bin/bash

projRoot="$(git rev-parse --show-toplevel)"

pushd "${projRoot}/test" > /dev/null

bash build.sh

valgrind --error-exitcode=1 --tool=cachegrind --cachegrind-out-file=cachegrind.out -- ./test.bin > /dev/null

cg_annotate --include="${projRoot}/src" --show=D1mr,DLmr cachegrind.out

popd > /dev/null
