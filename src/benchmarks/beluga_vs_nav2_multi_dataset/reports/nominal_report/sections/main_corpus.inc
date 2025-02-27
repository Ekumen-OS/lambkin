
.. repl-quiet::

    def get_bagfile_info(bagpath):
        """TODO: replace this with rosbag2_py.Info().read_metadata(bag_path)"""
        import subprocess
        process = subprocess.Popen(['ros2', 'bag', 'info', f'{bagpath}'], stdout=subprocess.PIPE)
        output, error = process.communicate()
        return output.decode('utf-8')

    data = pd.concat([
        lks.evo.series('/nav2_amcl_likelihood/pose', 'ape', normalization='long'),
        lks.evo.series('/nav2_amcl_likelihood_prob/pose', 'ape', normalization='long'),
        lks.evo.series('/nav2_amcl_likelihood_beam_skip/pose', 'ape', normalization='long'),
        lks.evo.series('/beluga_amcl_likelihood/pose', 'ape', normalization='long'),
        lks.evo.series('/beluga_amcl_likelihood_prob/pose', 'ape', normalization='long'),
    ])

    data = data[[
        'case.name',
        'variation.parameters.dataset',
        'trajectory.name',
        'variation.parameters.basedir',
        'metric.series.value'
    ]]

    data = data.rename(columns = {
        'case.name': 'case_name',
        'variation.parameters.dataset': 'bagfile',
        'trajectory.name': 'implementation',
        'variation.parameters.basedir': 'basedir',
        'metric.series.value': 'ape',
    })

    def build_bootstrapped_estimator(name, func, p = 50, n_bootstraps=500):
        def bootstrap_confidence_interval(data, name, func, p, n_bootstraps):
            bootstrapped = np.array([func(data.sample(frac=1, replace=True)) for _ in range(n_bootstraps)])
            return np.percentile(bootstrapped, [p])[0]
        return lambda data: bootstrap_confidence_interval(data, name, func, p, n_bootstraps)

    bootstrapped_mean_05 = build_bootstrapped_estimator('mean', np.mean, 5)
    bootstrapped_mean_95 = build_bootstrapped_estimator('mean', np.mean, 95)
    bootstrapped_median_05 = build_bootstrapped_estimator('median', np.median, 5)
    bootstrapped_median_95 = build_bootstrapped_estimator('median', np.median, 95)

    ape = data.groupby([
        'case_name',
        'bagfile',
        'implementation',
    ])['ape'].agg([
        ('median', np.median),
        ('median_05', bootstrapped_median_05),
        ('median_95', bootstrapped_median_95),
        ('mean', np.mean),
        ('mean_05', bootstrapped_mean_05),
        ('mean_95', bootstrapped_mean_95),
        'std',
        'max',
    ])

    bagfile_location_data = data.groupby([
        'case_name',
        'bagfile',
    ])['basedir'].agg(['first']).rename(columns={'first': 'basedir'})

    prof_data = pd.concat([
        lks.timem.summary('nav2_amcl_likelihood', 'cpu_util'),
        lks.timem.summary('nav2_amcl_likelihood', 'peak_rss'),
        lks.timem.summary('nav2_amcl_likelihood_prob', 'cpu_util'),
        lks.timem.summary('nav2_amcl_likelihood_prob', 'peak_rss'),
        lks.timem.summary('nav2_amcl_likelihood_beam_skip', 'cpu_util'),
        lks.timem.summary('nav2_amcl_likelihood_beam_skip', 'peak_rss'),
        lks.timem.summary('beluga_amcl_likelihood', 'cpu_util'),
        lks.timem.summary('beluga_amcl_likelihood', 'peak_rss'),
        lks.timem.summary('beluga_amcl_likelihood_prob', 'cpu_util'),
        lks.timem.summary('beluga_amcl_likelihood_prob', 'peak_rss'),
    ])

    prof_data = lks.pandas.rescale(prof_data, {
        'nav2_amcl_likelihood.summary.cpu_util': 100.,
        'nav2_amcl_likelihood.summary.peak_rss': 1 / 8e6,
        'nav2_amcl_likelihood_prob.summary.cpu_util': 100.,
        'nav2_amcl_likelihood_prob.summary.peak_rss': 1 / 8e6,
        'nav2_amcl_likelihood_beam_skip.summary.cpu_util': 100.,
        'nav2_amcl_likelihood_beam_skip.summary.peak_rss': 1 / 8e6,
        'beluga_amcl_likelihood.summary.cpu_util': 100.,
        'beluga_amcl_likelihood.summary.peak_rss': 1 / 8e6,
        'beluga_amcl_likelihood_prob.summary.cpu_util': 100.,
        'beluga_amcl_likelihood_prob.summary.peak_rss': 1 / 8e6,
    })

    prof_data = prof_data[[
        'case.name',
        'variation.parameters.dataset',
        'nav2_amcl_likelihood.summary.peak_rss',
        'nav2_amcl_likelihood.summary.cpu_util',
        'nav2_amcl_likelihood_prob.summary.peak_rss',
        'nav2_amcl_likelihood_prob.summary.cpu_util',
        'nav2_amcl_likelihood_beam_skip.summary.peak_rss',
        'nav2_amcl_likelihood_beam_skip.summary.cpu_util',
        'beluga_amcl_likelihood.summary.peak_rss',
        'beluga_amcl_likelihood.summary.cpu_util',
        'beluga_amcl_likelihood_prob.summary.peak_rss',
        'beluga_amcl_likelihood_prob.summary.cpu_util',
    ]]

    prof_data = prof_data.rename(columns = {
        'case.name': 'case_name',
        'variation.parameters.dataset': 'bagfile',
        'nav2_amcl_likelihood.summary.cpu_util': 'nav2_amcl_likelihood_cpu_util',
        'nav2_amcl_likelihood.summary.peak_rss': 'nav2_amcl_likelihood_peak_rss',
        'nav2_amcl_likelihood_prob.summary.cpu_util': 'nav2_amcl_likelihood_prob_cpu_util',
        'nav2_amcl_likelihood_prob.summary.peak_rss': 'nav2_amcl_likelihood_prob_peak_rss',
        'nav2_amcl_likelihood_beam_skip.summary.cpu_util': 'nav2_amcl_likelihood_beam_skip_cpu_util',
        'nav2_amcl_likelihood_beam_skip.summary.peak_rss': 'nav2_amcl_likelihood_beam_skip_peak_rss',
        'beluga_amcl_likelihood.summary.cpu_util': 'beluga_amcl_likelihood_cpu_util',
        'beluga_amcl_likelihood.summary.peak_rss': 'beluga_amcl_likelihood_peak_rss',
        'beluga_amcl_likelihood_prob.summary.cpu_util': 'beluga_amcl_likelihood_prob_cpu_util',
        'beluga_amcl_likelihood_prob.summary.peak_rss': 'beluga_amcl_likelihood_prob_peak_rss',
    })

    prof_data = prof_data.groupby([
        'case_name',
        'bagfile',
    ])

    nav2_amcl_likelihood_cpu_util_data = prof_data['nav2_amcl_likelihood_cpu_util'].agg(['max'])
    nav2_amcl_likelihood_peak_rss_data = prof_data['nav2_amcl_likelihood_peak_rss'].agg(['max'])
    nav2_amcl_likelihood_prob_cpu_util_data = prof_data['nav2_amcl_likelihood_prob_cpu_util'].agg(['max'])
    nav2_amcl_likelihood_prob_peak_rss_data = prof_data['nav2_amcl_likelihood_prob_peak_rss'].agg(['max'])
    nav2_amcl_likelihood_beam_skip_cpu_util_data = prof_data['nav2_amcl_likelihood_beam_skip_cpu_util'].agg(['max'])
    nav2_amcl_likelihood_beam_skip_peak_rss_data = prof_data['nav2_amcl_likelihood_beam_skip_peak_rss'].agg(['max'])
    beluga_amcl_likelihood_cpu_util_data = prof_data['beluga_amcl_likelihood_cpu_util'].agg(['max'])
    beluga_amcl_likelihood_peak_rss_data = prof_data['beluga_amcl_likelihood_peak_rss'].agg(['max'])
    beluga_amcl_likelihood_prob_cpu_util_data = prof_data['beluga_amcl_likelihood_prob_cpu_util'].agg(['max'])
    beluga_amcl_likelihood_prob_peak_rss_data = prof_data['beluga_amcl_likelihood_prob_peak_rss'].agg(['max'])

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
                nav2_amcl_likelihood_data      = data_for_bagfile.loc['/nav2_amcl_likelihood/pose']
                nav2_amcl_likelihood_median    = nav2_amcl_likelihood_data['median']
                nav2_amcl_likelihood_median_05 = nav2_amcl_likelihood_data['median_05']
                nav2_amcl_likelihood_median_95 = nav2_amcl_likelihood_data['median_95']
                nav2_amcl_likelihood_mean      = nav2_amcl_likelihood_data['mean']
                nav2_amcl_likelihood_mean_05   = nav2_amcl_likelihood_data['mean_05']
                nav2_amcl_likelihood_mean_95   = nav2_amcl_likelihood_data['mean_95']
                nav2_amcl_likelihood_std       = nav2_amcl_likelihood_data['std']
                nav2_amcl_likelihood_max       = nav2_amcl_likelihood_data['max']
                """ """
                nav2_amcl_likelihood_prob_data      = data_for_bagfile.loc['/nav2_amcl_likelihood_prob/pose']
                nav2_amcl_likelihood_prob_median    = nav2_amcl_likelihood_prob_data['median']
                nav2_amcl_likelihood_prob_median_05 = nav2_amcl_likelihood_prob_data['median_05']
                nav2_amcl_likelihood_prob_median_95 = nav2_amcl_likelihood_prob_data['median_95']
                nav2_amcl_likelihood_prob_mean      = nav2_amcl_likelihood_prob_data['mean']
                nav2_amcl_likelihood_prob_mean_05   = nav2_amcl_likelihood_prob_data['mean_05']
                nav2_amcl_likelihood_prob_mean_95   = nav2_amcl_likelihood_prob_data['mean_95']
                nav2_amcl_likelihood_prob_std       = nav2_amcl_likelihood_prob_data['std']
                nav2_amcl_likelihood_prob_max       = nav2_amcl_likelihood_prob_data['max']
                """ """
                nav2_amcl_likelihood_beam_skip_data      = data_for_bagfile.loc['/nav2_amcl_likelihood_beam_skip/pose']
                nav2_amcl_likelihood_beam_skip_median    = nav2_amcl_likelihood_beam_skip_data['median']
                nav2_amcl_likelihood_beam_skip_median_05 = nav2_amcl_likelihood_beam_skip_data['median_05']
                nav2_amcl_likelihood_beam_skip_median_95 = nav2_amcl_likelihood_beam_skip_data['median_95']
                nav2_amcl_likelihood_beam_skip_mean      = nav2_amcl_likelihood_beam_skip_data['mean']
                nav2_amcl_likelihood_beam_skip_mean_05   = nav2_amcl_likelihood_beam_skip_data['mean_05']
                nav2_amcl_likelihood_beam_skip_mean_95   = nav2_amcl_likelihood_beam_skip_data['mean_95']
                nav2_amcl_likelihood_beam_skip_std       = nav2_amcl_likelihood_beam_skip_data['std']
                nav2_amcl_likelihood_beam_skip_max       = nav2_amcl_likelihood_beam_skip_data['max']
                """ """
                beluga_amcl_likelihood_data      = data_for_bagfile.loc['/beluga_amcl_likelihood/pose']
                beluga_amcl_likelihood_median    = beluga_amcl_likelihood_data['median']
                beluga_amcl_likelihood_median_05 = beluga_amcl_likelihood_data['median_05']
                beluga_amcl_likelihood_median_95 = beluga_amcl_likelihood_data['median_95']
                beluga_amcl_likelihood_mean      = beluga_amcl_likelihood_data['mean']
                beluga_amcl_likelihood_mean_05   = beluga_amcl_likelihood_data['mean_05']
                beluga_amcl_likelihood_mean_95   = beluga_amcl_likelihood_data['mean_95']
                beluga_amcl_likelihood_std       = beluga_amcl_likelihood_data['std']
                beluga_amcl_likelihood_max       = beluga_amcl_likelihood_data['max']
                """ """
                beluga_amcl_likelihood_prob_data      = data_for_bagfile.loc['/beluga_amcl_likelihood_prob/pose']
                beluga_amcl_likelihood_prob_median    = beluga_amcl_likelihood_prob_data['median']
                beluga_amcl_likelihood_prob_median_05 = beluga_amcl_likelihood_prob_data['median_05']
                beluga_amcl_likelihood_prob_median_95 = beluga_amcl_likelihood_prob_data['median_95']
                beluga_amcl_likelihood_prob_mean      = beluga_amcl_likelihood_prob_data['mean']
                beluga_amcl_likelihood_prob_mean_05   = beluga_amcl_likelihood_prob_data['mean_05']
                beluga_amcl_likelihood_prob_mean_95   = beluga_amcl_likelihood_prob_data['mean_95']
                beluga_amcl_likelihood_prob_std       = beluga_amcl_likelihood_prob_data['std']
                beluga_amcl_likelihood_prob_max       = beluga_amcl_likelihood_prob_data['max']
            except:
                nav2_amcl_likelihood_median    = '???'
                nav2_amcl_likelihood_median_05 = '???'
                nav2_amcl_likelihood_median_95 = '???'
                nav2_amcl_likelihood_mean      = '???'
                nav2_amcl_likelihood_mean_05   = '???'
                nav2_amcl_likelihood_mean_95   = '???'
                nav2_amcl_likelihood_std       = '???'
                nav2_amcl_likelihood_max       = '???'
                """ """
                nav2_amcl_likelihood_prob_median    = '???'
                nav2_amcl_likelihood_prob_median_05 = '???'
                nav2_amcl_likelihood_prob_median_95 = '???'
                nav2_amcl_likelihood_prob_mean      = '???'
                nav2_amcl_likelihood_prob_mean_05   = '???'
                nav2_amcl_likelihood_prob_mean_95   = '???'
                nav2_amcl_likelihood_prob_std       = '???'
                nav2_amcl_likelihood_prob_max       = '???'
                """ """
                nav2_amcl_likelihood_beam_skip_median    = '???'
                nav2_amcl_likelihood_beam_skip_median_05 = '???'
                nav2_amcl_likelihood_beam_skip_median_95 = '???'
                nav2_amcl_likelihood_beam_skip_mean      = '???'
                nav2_amcl_likelihood_beam_skip_mean_05   = '???'
                nav2_amcl_likelihood_beam_skip_mean_95   = '???'
                nav2_amcl_likelihood_beam_skip_std       = '???'
                nav2_amcl_likelihood_beam_skip_max       = '???'
                """ """
                beluga_amcl_likelihood_median    = '???'
                beluga_amcl_likelihood_median_05 = '???'
                beluga_amcl_likelihood_median_95 = '???'
                beluga_amcl_likelihood_mean      = '???'
                beluga_amcl_likelihood_mean_05   = '???'
                beluga_amcl_likelihood_mean_95   = '???'
                beluga_amcl_likelihood_std       = '???'
                beluga_amcl_likelihood_max       = '???'
                """ """
                beluga_amcl_likelihood_prob_median    = '???'
                beluga_amcl_likelihood_prob_median_05 = '???'
                beluga_amcl_likelihood_prob_median_95 = '???'
                beluga_amcl_likelihood_prob_mean      = '???'
                beluga_amcl_likelihood_prob_mean_05   = '???'
                beluga_amcl_likelihood_prob_mean_95   = '???'
                beluga_amcl_likelihood_prob_std       = '???'
                beluga_amcl_likelihood_prob_max       = '???'
            try:
                nav2_amcl_likelihood_cpu_util = nav2_amcl_likelihood_cpu_util_data.loc[case_name].loc[bagfile]['max']
                nav2_amcl_likelihood_peak_rss = nav2_amcl_likelihood_peak_rss_data.loc[case_name].loc[bagfile]['max']
                """ """
                nav2_amcl_likelihood_prob_cpu_util = nav2_amcl_likelihood_prob_cpu_util_data.loc[case_name].loc[bagfile]['max']
                nav2_amcl_likelihood_prob_peak_rss = nav2_amcl_likelihood_prob_peak_rss_data.loc[case_name].loc[bagfile]['max']
                """ """
                nav2_amcl_likelihood_beam_skip_cpu_util = nav2_amcl_likelihood_beam_skip_cpu_util_data.loc[case_name].loc[bagfile]['max']
                nav2_amcl_likelihood_beam_skip_peak_rss = nav2_amcl_likelihood_beam_skip_peak_rss_data.loc[case_name].loc[bagfile]['max']
                """ """
                beluga_amcl_likelihood_cpu_util = beluga_amcl_likelihood_cpu_util_data.loc[case_name].loc[bagfile]['max']
                beluga_amcl_likelihood_peak_rss = beluga_amcl_likelihood_peak_rss_data.loc[case_name].loc[bagfile]['max']
                """ """
                beluga_amcl_likelihood_prob_cpu_util = beluga_amcl_likelihood_prob_cpu_util_data.loc[case_name].loc[bagfile]['max']
                beluga_amcl_likelihood_prob_peak_rss = beluga_amcl_likelihood_prob_peak_rss_data.loc[case_name].loc[bagfile]['max']
            except:
                nav2_amcl_likelihood_cpu_util           = "???"
                nav2_amcl_likelihood_peak_rss           = "???"
                nav2_amcl_likelihood_prob_cpu_util      = "???"
                nav2_amcl_likelihood_prob_peak_rss      = "???"
                nav2_amcl_likelihood_beam_skip_cpu_util = "???"
                nav2_amcl_likelihood_beam_skip_peak_rss = "???"
                beluga_amcl_likelihood_cpu_util         = "???"
                beluga_amcl_likelihood_peak_rss         = "???"
                beluga_amcl_likelihood_prob_cpu_util    = "???"
                beluga_amcl_likelihood_prob_peak_rss    = "???"
            """ """
            output.append(TABLE_HEADER_TEMPLATE.format(
                trajectory_name=bagfile,
                sensor_model='Likelihood field',
                nrows=len(data_for_bagfile)))
            output.append(ROW_TEMPLATE.format(
                implementation_name="Nav2 LF",
                median=round_digits(nav2_amcl_likelihood_median),
                median_05=round_digits(nav2_amcl_likelihood_median_05),
                median_95=round_digits(nav2_amcl_likelihood_median_95),
                mean=round_digits(nav2_amcl_likelihood_mean),
                mean_05=round_digits(nav2_amcl_likelihood_mean_05),
                mean_95=round_digits(nav2_amcl_likelihood_mean_95),
                worst=round_digits(nav2_amcl_likelihood_max)))
            output.append(ROW_TEMPLATE.format(
                implementation_name="Nav2 LFP",
                median=round_digits(nav2_amcl_likelihood_prob_median),
                median_05=round_digits(nav2_amcl_likelihood_prob_median_05),
                median_95=round_digits(nav2_amcl_likelihood_prob_median_95),
                mean=round_digits(nav2_amcl_likelihood_prob_mean),
                mean_05=round_digits(nav2_amcl_likelihood_prob_mean_05),
                mean_95=round_digits(nav2_amcl_likelihood_prob_mean_95),
                worst=round_digits(nav2_amcl_likelihood_prob_max)))
            output.append(ROW_TEMPLATE.format(
                implementation_name="Nav2 LFP + BS",
                median=round_digits(nav2_amcl_likelihood_beam_skip_median),
                median_05=round_digits(nav2_amcl_likelihood_beam_skip_median_05),
                median_95=round_digits(nav2_amcl_likelihood_beam_skip_median_95),
                mean=round_digits(nav2_amcl_likelihood_beam_skip_mean),
                mean_05=round_digits(nav2_amcl_likelihood_beam_skip_mean_05),
                mean_95=round_digits(nav2_amcl_likelihood_beam_skip_mean_95),
                worst=round_digits(nav2_amcl_likelihood_beam_skip_max)))
            output.append(ROW_TEMPLATE.format(
                implementation_name="Beluga LF",
                median=round_digits(beluga_amcl_likelihood_median),
                median_05=round_digits(beluga_amcl_likelihood_median_05),
                median_95=round_digits(beluga_amcl_likelihood_median_95),
                mean=round_digits(beluga_amcl_likelihood_mean),
                mean_05=round_digits(beluga_amcl_likelihood_mean_05),
                mean_95=round_digits(beluga_amcl_likelihood_mean_95),
                worst=round_digits(beluga_amcl_likelihood_max)))
            output.append(ROW_TEMPLATE.format(
                implementation_name="Beluga LFP",
                median=round_digits(beluga_amcl_likelihood_prob_median),
                median_05=round_digits(beluga_amcl_likelihood_prob_median_05),
                median_95=round_digits(beluga_amcl_likelihood_prob_median_95),
                mean=round_digits(beluga_amcl_likelihood_prob_mean),
                mean_05=round_digits(beluga_amcl_likelihood_prob_mean_05),
                mean_95=round_digits(beluga_amcl_likelihood_prob_mean_95),
                worst=round_digits(beluga_amcl_likelihood_prob_max)))
            """ """
            output.append(RESOURCE_TABLE_HEADER_TEMPLATE.format(
                trajectory_name=bagfile,
                nrows=len(data_for_bagfile)))
            output.append(RESOURCE_TABLE_ROW_TEMPLATE.format(
                implementation_name="Nav2 LF",
                likelihood_field_peak_cpu=round_digits(nav2_amcl_likelihood_cpu_util, 1),
                likelihood_field_peak_rss=round_digits(nav2_amcl_likelihood_peak_rss, 0)))
            output.append(RESOURCE_TABLE_ROW_TEMPLATE.format(
                implementation_name="Nav2 LFP",
                likelihood_field_peak_cpu=round_digits(nav2_amcl_likelihood_prob_cpu_util, 1),
                likelihood_field_peak_rss=round_digits(nav2_amcl_likelihood_prob_peak_rss, 0)))
            output.append(RESOURCE_TABLE_ROW_TEMPLATE.format(
                implementation_name="Nav2 LFP + BS",
                likelihood_field_peak_cpu=round_digits(nav2_amcl_likelihood_beam_skip_cpu_util, 1),
                likelihood_field_peak_rss=round_digits(nav2_amcl_likelihood_beam_skip_peak_rss, 0)))
            output.append(RESOURCE_TABLE_ROW_TEMPLATE.format(
                implementation_name="Beluga LF",
                likelihood_field_peak_cpu=round_digits(beluga_amcl_likelihood_cpu_util, 1),
                likelihood_field_peak_rss=round_digits(beluga_amcl_likelihood_peak_rss, 0)))
            output.append(RESOURCE_TABLE_ROW_TEMPLATE.format(
                implementation_name="Beluga LFP",
                likelihood_field_peak_cpu=round_digits(beluga_amcl_likelihood_prob_cpu_util, 1),
                likelihood_field_peak_rss=round_digits(beluga_amcl_likelihood_prob_peak_rss, 0)))

    output = '\n\n'.join(output)

    with open('_generated/generated_section.inc', 'w') as f:
        f.write(output)

.. include:: _generated/generated_section.inc
