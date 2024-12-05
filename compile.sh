#!/bin/bash
idx_dir="index_data"
build_dir="auto_build"

mkdir -p ${idx_dir} ${build_dir}
cmake "-B${build_dir}" -DCMAKE_BUILD_TYPE=Release
cmake --build ${build_dir}
# build exec
cp "${build_dir}/libGPPCentry.so" lib
