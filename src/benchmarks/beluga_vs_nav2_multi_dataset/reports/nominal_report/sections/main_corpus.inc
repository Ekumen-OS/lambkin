
.. repl-quiet::

    def get_bagfile_info(bagpath):
        import subprocess
        process = subprocess.Popen(['ros2', 'bag', 'info', f'{bagpath}'], stdout=subprocess.PIPE)
        output, error = process.communicate()
        return output.decode('utf-8')

    data = pd.concat([
        lks.evo.series('/beluga_amcl/pose', 'ape', normalization='long'),
        lks.evo.series('/nav2_amcl/pose', 'ape', normalization='long')
    ])

    data = data[[
        'case.name',
        'variation.parameters.dataset',
        'variation.parameters.laser_model',
        'trajectory.name',
        'variation.parameters.basedir',
        'metric.series.value'
    ]]

    data = data.rename(columns = {
        'case.name': 'case_name',
        'variation.parameters.dataset': 'bagfile',
        'variation.parameters.laser_model': 'laser_model',
        'trajectory.name': 'implementation',
        'variation.parameters.basedir': 'basedir',
        'metric.series.value': 'ape',
    })

    ape = data.groupby([
        'case_name',
        'bagfile',
        'implementation',
        'laser_model',
    ])['ape'].agg(['median', 'mean', 'std', 'max']).round(3)

    bagfile_location_data = data.groupby([
        'case_name',
        'bagfile',
    ])['basedir'].agg(['first']).rename(columns={'first': 'basedir'})

    from ament_index_python import get_package_share_directory

    this_pkg_dir = get_package_share_directory('beluga_vs_nav2_multi_dataset')

    with open(f'{this_pkg_dir}/reports/nominal_report/templates/template_table_header.txt', 'r') as f:
        TABLE_HEADER_TEMPLATE = f.read()

    with open(f'{this_pkg_dir}/reports/nominal_report/templates/template_table_row.txt', 'r') as f:
        ROW_TEMPLATE = f.read()

    with open(f'{this_pkg_dir}/reports/nominal_report/templates/template_dataset_chapter.txt', 'r') as f:
        DATASET_CHAPTER_TEMPLATE = f.read()

    with open(f'{this_pkg_dir}/reports/nominal_report/templates/template_bagfile_section.txt', 'r') as f:
        BAGFILE_SECTION_TEMPLATE = f.read()

    with open(f'{this_pkg_dir}/reports/nominal_report/templates/template_resource_table_header.txt', 'r') as f:
        RESOURCE_TABLE_HEADER_TEMPLATE = f.read()

    with open(f'{this_pkg_dir}/reports/nominal_report/templates/template_resource_table_row.txt', 'r') as f:
        RESOURCE_TABLE_ROW_TEMPLATE = f.read()

    output = []

    def equalize_digits(value, digits=3):
        return f'{value:.{digits}f}' if value is not str else value

    desired_section_order = [
        'Magazino Datasets',
        'Openloris Office',
        'TorWIC Mapping',
        'TorWIC SLAM',
        'Willow Garage',
        'Diff Drive Sim 24hs',
        'Omni Drive Sim 24hs',
    ]
    datasets_in_data = ape.index.get_level_values(0).unique()
    missing_datasets = [x for x in datasets_in_data if x not in desired_section_order]
    desired_section_order = desired_section_order + missing_datasets

    for case_name in desired_section_order:
        output.append(DATASET_CHAPTER_TEMPLATE.format(dataset_name=case_name))
        case_name_str = case_name.replace(' ', '_').lower()
        output.append(f'.. include:: sections/datasets/{case_name_str}_description.inc\n\n')
        data_for_case = ape.loc[case_name]
        for bagfile in data_for_case.index.get_level_values(0).unique():
            metadata = ""
            mapfile = ""
            try:
                bagpath = bagfile_location_data.loc[case_name].loc[bagfile].loc['basedir']
                metadata = get_bagfile_info(os.path.join('..', '..', bagpath, 'bagfiles', bagfile))
                metadata = [x for x in metadata.split('\n') if "Topic:" not in x]
                metadata = "    " + "\n    ".join(metadata)
                mapfile = os.path.join('..', '..', bagpath, 'artifacts', bagfile, 'map.pgm')
            except:
                pass
            output.append(BAGFILE_SECTION_TEMPLATE.format(bagfile_name=bagfile, metadata=metadata))
            data_for_bagfile = data_for_case.loc[bagfile]
            try:
                beluga_likelihood_data = data_for_bagfile.loc['/beluga_amcl/pose', 'likelihood_field']
                beluga_beam_data = data_for_bagfile.loc['/beluga_amcl/pose', 'beam']
                beluga_likelihood_median = beluga_likelihood_data['median']
                beluga_likelihood_mean = beluga_likelihood_data['mean']
                beluga_likelihood_std = beluga_likelihood_data['std']
                beluga_likelihood_max = beluga_likelihood_data['max']
                beluga_beam_median = beluga_beam_data['median']
                beluga_beam_mean = beluga_beam_data['mean']
                beluga_beam_std = beluga_beam_data['std']
                beluga_beam_max = beluga_beam_data['max']
            except:
                beluga_likelihood_rms = beluga_likelihood_mean = beluga_likelihood_std = beluga_likelihood_max = "???"
            try:
                nav2_likelihood_data = data_for_bagfile.loc['/nav2_amcl/pose', 'likelihood_field']
                nav2_beam_data = data_for_bagfile.loc['/nav2_amcl/pose', 'beam']
                nav2_likelihood_median = nav2_likelihood_data['median']
                nav2_likelihood_mean = nav2_likelihood_data['mean']
                nav2_likelihood_std = nav2_likelihood_data['std']
                nav2_likelihood_max = nav2_likelihood_data['max']
                nav2_beam_median = nav2_beam_data['median']
                nav2_beam_mean = nav2_beam_data['mean']
                nav2_beam_std = nav2_beam_data['std']
                nav2_beam_max = nav2_beam_data['max']
            except:
                nav2_likelihood_rms = nav2_likelihood_mean = nav2_likelihood_std = nav2_likelihood_max = "???"
            output.append(TABLE_HEADER_TEMPLATE.format(
                trajectory_name=bagfile,
                sensor_model='Likelihood field',
                nrows=len(data_for_bagfile)))
            output.append(ROW_TEMPLATE.format(
                implementation_name="Beluga AMCL",
                median=equalize_digits(beluga_likelihood_median),
                mean=equalize_digits(beluga_likelihood_mean),
                std=equalize_digits(beluga_likelihood_std),
                worst=equalize_digits(beluga_likelihood_max)))
            output.append(ROW_TEMPLATE.format(
                implementation_name="Nav2 AMCL",
                median=equalize_digits(nav2_likelihood_median),
                mean=equalize_digits(nav2_likelihood_mean),
                std=equalize_digits(nav2_likelihood_std),
                worst=equalize_digits(nav2_likelihood_max)))
            output.append(TABLE_HEADER_TEMPLATE.format(
                trajectory_name=bagfile,
                sensor_model='Beam',
                nrows=len(data_for_bagfile)))
            output.append(ROW_TEMPLATE.format(
                implementation_name="Beluga AMCL",
                median=equalize_digits(beluga_beam_median),
                mean=equalize_digits(beluga_beam_mean),
                std=equalize_digits(beluga_beam_std),
                worst=equalize_digits(beluga_beam_max)))
            output.append(ROW_TEMPLATE.format(
                implementation_name="Nav2 AMCL",
                median=equalize_digits(nav2_beam_median),
                mean=equalize_digits(nav2_beam_mean),
                std=equalize_digits(nav2_beam_std),
                worst=equalize_digits(nav2_beam_max)))

    output = '\n\n'.join(output)

    with open('_generated/generated_section.inc', 'w') as f:
        f.write(output)

.. include:: _generated/generated_section.inc
