FORMAT_DIR=$1

find ${FORMAT_DIR} -iname '*.h' -o -iname '*.c' | xargs clang-format-13 -i
