
.. repl-quiet::

    def get_bagfile_info(bagpath):
        """TODO: replace this with rosbag2_py.Info().read_metadata(bag_path)"""
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

    prof_data = pd.concat([
        lks.timem.summary('beluga_amcl', 'cpu_util'),
        lks.timem.summary('nav2_amcl', 'cpu_util'),
        lks.timem.summary('beluga_amcl', 'peak_rss'),
        lks.timem.summary('nav2_amcl', 'peak_rss'),
    ])

    prof_data = lks.pandas.rescale(prof_data, {
        'beluga_amcl.summary.cpu_util': 100.,
        'beluga_amcl.summary.peak_rss': 1 / 8e6,
        'nav2_amcl.summary.cpu_util': 100.,
        'nav2_amcl.summary.peak_rss': 1 / 8e6
    })

    prof_data = prof_data[[
        'case.name',
        'variation.parameters.dataset',
        'variation.parameters.laser_model',
        'beluga_amcl.summary.peak_rss',
        'beluga_amcl.summary.cpu_util',
        'nav2_amcl.summary.peak_rss',
        'nav2_amcl.summary.cpu_util',
    ]]

    prof_data = prof_data.rename(columns = {
        'case.name': 'case_name',
        'variation.parameters.dataset': 'bagfile',
        'variation.parameters.laser_model': 'laser_model',
        'beluga_amcl.summary.peak_rss': 'beluga_amcl_peak_rss',
        'beluga_amcl.summary.cpu_util': 'beluga_amcl_cpu_util',
        'nav2_amcl.summary.peak_rss': 'nav2_amcl_peak_rss',
        'nav2_amcl.summary.cpu_util': 'nav2_amcl_cpu_util',
    })

    prof_data = prof_data.groupby([
        'case_name',
        'bagfile',
        'laser_model',
    ])

    beluga_max_cpu = prof_data['beluga_amcl_cpu_util'].agg(['max'])
    beluga_max_rss = prof_data['beluga_amcl_peak_rss'].agg(['max'])
    nav2_max_cpu = prof_data['nav2_amcl_cpu_util'].agg(['max'])
    nav2_max_rss = prof_data['nav2_amcl_peak_rss'].agg(['max'])

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

    def round_digits(value, digits=3):
        # if the value is not a number, just pass through whatever it is
        try:
            output_str = f'{value:.{digits}f}'
        except:
            output_str = str(value)
        return output_str

    desired_section_order = [
        'Willow Garage',
        'TorWIC SLAM',
        'TorWIC Mapping',
        'Magazino Datasets',
        'Openloris Office',
        'Diff Drive Sim 24hs',
        'Omni Drive Sim 24hs',
    ]
    datasets_in_data = ape.index.get_level_values(0).unique()
    missing_datasets = [x for x in datasets_in_data if x not in desired_section_order]
    desired_section_order = desired_section_order + missing_datasets
    datasets_to_process = [x for x in desired_section_order if x in datasets_in_data]

    for case_name in datasets_to_process:
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
            try:
                beluga_likelihood_cpu = beluga_max_cpu.loc[case_name].loc[bagfile].loc['likelihood_field']['max']
                beluga_likelihood_peak_rss = beluga_max_rss.loc[case_name].loc[bagfile].loc['likelihood_field']['max']
                nav2_likelihood_cpu = nav2_max_cpu.loc[case_name].loc[bagfile].loc['likelihood_field']['max']
                nav2_likelihood_peak_rss = nav2_max_rss.loc[case_name].loc[bagfile].loc['likelihood_field']['max']
                beluga_beam_cpu = beluga_max_cpu.loc[case_name].loc[bagfile].loc['beam']['max']
                beluga_beam_peak_rss = beluga_max_rss.loc[case_name].loc[bagfile].loc['beam']['max']
                nav2_beam_cpu = nav2_max_cpu.loc[case_name].loc[bagfile].loc['beam']['max']
                nav2_beam_peak_rss = nav2_max_rss.loc[case_name].loc[bagfile].loc['beam']['max']
            except:
                beluga_likelihood_cpu = beluga_likelihood_peak_rss = nav2_likelihood_cpu = nav2_likelihood_peak_rss = "???"
                beluga_beam_cpu = beluga_beam_peak_rss = nav2_beam_cpu = nav2_beam_peak_rss = "???"
            output.append(TABLE_HEADER_TEMPLATE.format(
                trajectory_name=bagfile,
                sensor_model='Likelihood field',
                nrows=len(data_for_bagfile)))
            output.append(ROW_TEMPLATE.format(
                implementation_name="Beluga AMCL",
                median=round_digits(beluga_likelihood_median),
                mean=round_digits(beluga_likelihood_mean),
                std=round_digits(beluga_likelihood_std),
                worst=round_digits(beluga_likelihood_max)))
            output.append(ROW_TEMPLATE.format(
                implementation_name="Nav2 AMCL",
                median=round_digits(nav2_likelihood_median),
                mean=round_digits(nav2_likelihood_mean),
                std=round_digits(nav2_likelihood_std),
                worst=round_digits(nav2_likelihood_max)))
            output.append(TABLE_HEADER_TEMPLATE.format(
                trajectory_name=bagfile,
                sensor_model='Beam',
                nrows=len(data_for_bagfile)))
            output.append(ROW_TEMPLATE.format(
                implementation_name="Beluga AMCL",
                median=round_digits(beluga_beam_median),
                mean=round_digits(beluga_beam_mean),
                std=round_digits(beluga_beam_std),
                worst=round_digits(beluga_beam_max)))
            output.append(ROW_TEMPLATE.format(
                implementation_name="Nav2 AMCL",
                median=round_digits(nav2_beam_median),
                mean=round_digits(nav2_beam_mean),
                std=round_digits(nav2_beam_std),
                worst=round_digits(nav2_beam_max)))
            output.append(RESOURCE_TABLE_HEADER_TEMPLATE.format(
                trajectory_name=bagfile,
                nrows=2))
            output.append(RESOURCE_TABLE_ROW_TEMPLATE.format(
                implementation_name="Beluga AMCL",
                likelihood_field_peak_cpu=round_digits(beluga_likelihood_cpu, 1),
                likelihood_field_peak_rss=round_digits(beluga_likelihood_peak_rss, 0),
                beam_peak_cpu=round_digits(beluga_beam_cpu, 1),
                beam_peak_rss=round_digits(beluga_beam_peak_rss, 0)))
            output.append(RESOURCE_TABLE_ROW_TEMPLATE.format(
                implementation_name="Nav2 AMCL",
                likelihood_field_peak_cpu=round_digits(nav2_likelihood_cpu, 1),
                likelihood_field_peak_rss=round_digits(nav2_likelihood_peak_rss, 0),
                beam_peak_cpu=round_digits(nav2_beam_cpu, 1),
                beam_peak_rss=round_digits(nav2_beam_peak_rss, 0)))

    output = '\n\n'.join(output)

    with open('_generated/generated_section.inc', 'w') as f:
        f.write(output)

.. include:: _generated/generated_section.inc
