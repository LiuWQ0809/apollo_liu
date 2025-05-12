#!/usr/bin/env bash

###############################################################################
# Copyright 2023 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################
TOP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"

declare -A dic
models_dir="${TOP_DIR}/modules/perception/data/models/"

dic=(
    [apollo_bevnet_onnx.zip]="http://apollo-perception.bj.bcebos.com/core_model/apollo_bevnet_onnx.zip?authorization=bce-auth-v1%2FALTAK6fleneBT08Sn61Gseah1T%2F2024-11-19T03%3A19%3A47Z%2F-1%2Fhost%2Fcff6ed21d22bb27395182790e07582b95edbed4059f611211db1224aad93acc7" \
    )

pushd "${TOP_DIR}"

for key in $(echo ${!dic[*]}); do
    download_link=${dic[$key]}
    wget -O ${key} ${download_link}
    unzip ${key}
    for i in `ls ${key::-4}`; do
        [[ -e ${models_dir}${key::-4}/${i} ]] && rm -f ${models_dir}${key::-4}/${i}
        sudo bash -c "mv -f ${key::-4}/${i} ${models_dir}${key::-4}/"
    done
    rm -rf ${key::-4} ${key}
  done

popd