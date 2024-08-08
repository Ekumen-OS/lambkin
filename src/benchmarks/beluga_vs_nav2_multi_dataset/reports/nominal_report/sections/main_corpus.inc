
.. repl::

    def rms(x):
        return np.sqrt(np.mean(np.power(x, 2.)))

    data = pd.concat([
        lks.evo.series('/beluga_amcl/pose', 'ape', normalization='long'),
        lks.evo.series('/nav2_amcl/pose', 'ape', normalization='long')
    ])

    print(data.columns)

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
    ])['ape'].agg([rms, 'mean', 'std', 'max']).round(3)

    ape.to_pickle('_generated/ape.pkl')

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

    output = []

    for case_name in ape.index.get_level_values(0).unique():
        output.append(DATASET_CHAPTER_TEMPLATE.format(dataset_name=case_name))
        case_name_str = case_name.replace(' ', '_').lower()
        output.append(f'.. include:: sections/datasets/{case_name_str}_description.inc\n\n')
        data_for_case = ape.loc[case_name]
        for bagfile in data_for_case.index.get_level_values(0).unique():
            output.append(BAGFILE_SECTION_TEMPLATE.format(bagfile_name=bagfile))
            data_for_bagfile = data_for_case.loc[bagfile]
            basedir = None
            try:
                beluga_likelihood_data = data_for_bagfile.loc['/beluga_amcl/pose', 'likelihood_field']
                beluga_beam_data = data_for_bagfile.loc['/beluga_amcl/pose', 'beam']
                beluga_likelihood_rms = beluga_likelihood_data['rms']
                beluga_likelihood_mean = beluga_likelihood_data['mean']
                beluga_likelihood_std = beluga_likelihood_data['std']
                beluga_likelihood_max = beluga_likelihood_data['max']
                beluga_beam_rms = beluga_beam_data['rms']
                beluga_beam_mean = beluga_beam_data['mean']
                beluga_beam_std = beluga_beam_data['std']
                beluga_beam_max = beluga_beam_data['max']
            except:
                beluga_likelihood_rms = beluga_likelihood_mean = beluga_likelihood_std = beluga_likelihood_max = "???"
            try:
                nav2_likelihood_data = data_for_bagfile.loc['/nav2_amcl/pose', 'likelihood_field']
                nav2_beam_data = data_for_bagfile.loc['/nav2_amcl/pose', 'beam']
                nav2_likelihood_rms = nav2_likelihood_data['rms']
                nav2_likelihood_mean = nav2_likelihood_data['mean']
                nav2_likelihood_std = nav2_likelihood_data['std']
                nav2_likelihood_max = nav2_likelihood_data['max']
                nav2_beam_rms = nav2_beam_data['rms']
                nav2_beam_mean = nav2_beam_data['mean']
                nav2_beam_std = nav2_beam_data['std']
                nav2_beam_max = nav2_beam_data['max']
            except:
                nav2_likelihood_rms = nav2_likelihood_mean = nav2_likelihood_std = nav2_likelihood_max = "???"
            output.append(TABLE_HEADER_TEMPLATE.format(
                trajectory_name=bagfile,
                sensor_model='Likelihood field',
                nrows=len(data_for_bagfile),
                table_name='hallway-localization-ape-comparison'))
            output.append(ROW_TEMPLATE.format(
                implementation_name="Beluga AMCL",
                rms=beluga_likelihood_rms,
                mean=beluga_likelihood_mean,
                std=beluga_likelihood_std,
                max=beluga_likelihood_max))
            output.append(ROW_TEMPLATE.format(
                implementation_name="Nav2 AMCL",
                rms=nav2_likelihood_rms,
                mean=nav2_likelihood_mean,
                std=nav2_likelihood_std,
                max=nav2_likelihood_max))
            output.append(TABLE_HEADER_TEMPLATE.format(
                trajectory_name=bagfile,
                sensor_model='Beam',
                nrows=len(data_for_bagfile),
                table_name='hallway-localization-ape-comparison'))
            output.append(ROW_TEMPLATE.format(
                implementation_name="Beluga AMCL",
                rms=beluga_beam_rms,
                mean=beluga_beam_mean,
                std=beluga_beam_std,
                max=beluga_beam_max))
            output.append(ROW_TEMPLATE.format(
                implementation_name="Nav2 AMCL",
                rms=nav2_beam_rms,
                mean=nav2_beam_mean,
                std=nav2_beam_std,
                max=nav2_beam_max))

    output = '\n\n'.join(output)

    with open('_generated/generated_section.inc', 'w') as f:
        f.write(output)

.. include:: _generated/generated_section.inc
