# Copyright 2023 Ekumen, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import numpy as np
import pandas as pd

from lambkin.shepherd.data.pandas import inner_join


def test_inner_join():
    lvalue = pd.DataFrame({
        'sample_index': np.arange(100), 'device': 'thm0',
        'temperature': 25 + np.random.uniform(-2, 2, 100)
    })
    rvalue = pd.DataFrame({
        'sample_index': np.arange(100), 'device': 'bar0',
        'pressure': 101325 + np.random.uniform(-100, 100, 100)
    })
    mvalue = inner_join([lvalue, rvalue], on=['sample_index'])
    mvalue = mvalue.drop(columns=['device', 'device_1'])
    assert sorted(mvalue.columns) == [
        'pressure', 'sample_index', 'temperature']
    indexed_mvalue = mvalue.set_index('sample_index')

    indexed_lvalue = lvalue.set_index('sample_index')
    assert (indexed_mvalue['temperature'] == indexed_lvalue['temperature']).all()

    indexed_rvalue = rvalue.set_index('sample_index')
    assert (indexed_mvalue['pressure'] == indexed_rvalue['pressure']).all()
