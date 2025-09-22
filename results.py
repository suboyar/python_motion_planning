import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np

pd.set_option('display.max_columns', None)
pd.set_option('display.max_rows', None)
pd.set_option('display.width', None)

files = {
    10: "./examples/results_elevation_weight_1_1.csv",
    25: "./examples/results_elevation_weight_1_25.csv",
    50: "./examples/results_elevation_weight_1_5.csv"
}

desired_algorithms = ["lazy_theta_star", "s_theta_star", "theta_star",
                      "a_star", "dijkstra", "rrt_star", "informed_rrt",
                      "rrt", "rrt_connect"]


sns.set_style("darkgrid")

title_size = 22
label_size = 16

# Create a fixed color mapping for each algorithm
colors = plt.cm.tab10(range(len(desired_algorithms)))
algorithm_colors = dict(zip(desired_algorithms, colors))

def create_algorithm_boxplot(csv_file, plot_title):
    df = pd.read_csv(csv_file)
    df_filtered = df[desired_algorithms]

    # Calculate median for each algorithm and sort from best (lowest) to worst (highest)
    medians = df_filtered.median().sort_values()
    sorted_algorithms = medians.index.tolist()

    df_melted = df_filtered.melt(var_name='Algorithm', value_name='Performance')

    plt.figure(figsize=(15, 8))
    ax = sns.boxplot(data=df_melted, x='Algorithm', y='Performance',
                 order=sorted_algorithms, palette=algorithm_colors,
                 hue='Algorithm', legend=False,
                 whis=[0, 100])
    plt.title(f'{plot_title} (Sorted Best to Worst)', fontsize=title_size)
    plt.xlabel('Algorithms', fontsize=label_size)
    plt.ylabel('Path Length (m)', fontsize=label_size)
    plt.xticks(rotation=45, ha='right')
    plt.grid(True, alpha=0.6)
    plt.tight_layout()

    plt.savefig(f'boxplot_{plot_title.replace(" ", "_").lower()}.png', dpi=300, bbox_inches='tight')

    print(f"\nAlgorithm Ranking for {plot_title} (Best to Worst by Median):")
    for i, alg in enumerate(sorted_algorithms, 1):
        print(f"{i:2d}. {alg}: {medians[alg]:.2f}")

    print(f"\nSummary Statistics for {plot_title}:")
    print(df_filtered[sorted_algorithms].describe(include="all").T)
    print("\n" + "="*80 + "\n")


def create_algorithm_cdf(csv_file, plot_title):
    df = pd.read_csv(csv_file)
    df_filtered = df[desired_algorithms]

    # Calculate median for each algorithm and sort from best (lowest) to worst (highest)
    medians = df_filtered.median().sort_values()
    sorted_algorithms = medians.index.tolist()

    plt.figure(figsize=(15, 8))

    cdf_records = []

    # Plot CDF for each algorithm showing relative gap to best
    best_per_row = df_filtered.min(axis=1)
    for algorithm in sorted_algorithms:
        # Calculate relative gap as percentage: (algorithm - best) / best * 100
        relative_gaps = ((df_filtered[algorithm] - best_per_row) / best_per_row * 100).dropna()

        # Sort the gaps for CDF
        data = relative_gaps.sort_values()
        n = len(data)

        # Calculate empirical CDF
        y_values = np.arange(1, n + 1) / n

        for gap, prob in zip(data.values, y_values):
            cdf_records.append({
                'Algorithm': algorithm,
                'Gap_Percent': round(gap, 2),
                'CDF_Probability': round(prob, 3)
            })

        plt.step(data, y_values, where="post", label=algorithm,
        # plt.plot(data, y_values, label=algorithm,
                color=algorithm_colors[algorithm], linewidth=2.5)

    plt.xlim(-2, 65)
    plt.title(f'{plot_title} - Relative Gap to Best', fontsize=title_size)
    plt.xlabel('Relative Gap to Best Algorithm (%)', fontsize=label_size)
    plt.ylabel('CDF', fontsize=label_size)
    plt.grid(True, alpha=0.8)
    plt.legend(title='Algorithms (Best to Worst)', loc='lower right')
    plt.gca().yaxis.set_major_formatter(plt.FuncFormatter(lambda y, _: f'{y:.0%}'))

    plt.tight_layout()
    plt.savefig(f'cdf_{plot_title.replace(" ", "_").lower()}.png', dpi=300, bbox_inches='tight')

    # competitive_levels = {
    # 'Optimal (0%)': 0,
    # 'Excellent (<1%)': 1,
    # 'Good (<5%)': 5,
    # 'Acceptable (<10%)': 10,
    # 'Poor (>=10%)': float('inf')
    # }

    # print(f"\nCompetitive Performance Analysis for {plot_title}:")
    # for algorithm in sorted_algorithms:
    #     relative_gaps = ((df_filtered[algorithm] - best_per_row) / best_per_row * 100).dropna()
    #     print(f"\n{algorithm}:")

    #     for label, threshold in competitive_levels.items():
    #         if threshold == float('inf'):
    #             count = (relative_gaps >= 10).sum()
    #         elif  threshold < float('inf') && threshold >= 10:
    #             count =
    #         elif  threshold < 10 && threshold >= 5:
    #             count =
    #         else:
    #             count = (relative_gaps <= threshold).sum()
    #         print(f"  {label}: {count}/20 tests ({count/20:.1%})")






create_algorithm_boxplot(files[10], '10% Elevation Penalty')
create_algorithm_boxplot(files[25], '25% Elevation Penalty')
create_algorithm_boxplot(files[50], '50% Elevation Penalty')

create_algorithm_cdf(files[10], '10% Elevation Penalty')
create_algorithm_cdf(files[25], '25% Elevation Penalty')
create_algorithm_cdf(files[50], '50% Elevation Penalty')
