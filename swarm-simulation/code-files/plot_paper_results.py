import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
from matplotlib import rcParams
import os

# --- CONFIGURATION ---
# Mimic LaTeX font rendering without requiring a local TeX installation
# MASSIVELY INCREASED FONT SIZES (Base 24pt) as requested
plt.rcParams.update({
    "font.family": "serif",
    "font.serif": ["Times New Roman", "Computer Modern Roman", "serif"],
    "mathtext.fontset": "cm",
    "axes.labelsize": 24,
    "axes.titlesize": 26,
    "xtick.labelsize": 22,
    "ytick.labelsize": 22,
    "legend.fontsize": 22,
    # Increased figure size significantly to accommodate large text
    "figure.figsize": (24, 7) 
})

def load_and_prep_data():
    if not os.path.exists("experiment_results_paper.csv"):
        print("Error: experiment_results_paper.csv not found.")
        return None

    df = pd.read_csv("experiment_results_paper.csv")
    
    # Create a unified "Condition" column for the X-axis
    def get_condition(row):
        if row['scenario'] == 'Scalability':
            return f"N={int(row['num_agents'])}"
        elif row['scenario_tag'] == 'Fault_1':
            return "Fault\n(1 Agent)"
        elif row['scenario_tag'] == 'Fault_2':
            return "Fault\n(2 Agents)"
        return "Other"

    df['Condition'] = df.apply(get_condition, axis=1)
    
    # Define an ordering for the plots
    order = ['N=2', 'N=4', 'N=8', 'Fault\n(1 Agent)', 'Fault\n(2 Agents)']
    df['Condition'] = pd.Categorical(df['Condition'], categories=order, ordered=True)
    return df

def generate_combined_plot(df):
    # Set the visual theme
    sns.set_theme(style="whitegrid", font="serif")
    
    # Create a figure with 3 subplots
    fig, axes = plt.subplots(1, 3, figsize=(30, 8), constrained_layout=True)
    
    # Palette
    palette = sns.color_palette("muted")
    colors = {
        'N=2': palette[0],
        'N=4': palette[1],
        'N=8': palette[2],
        'Fault\n(1 Agent)': palette[8], # Yellowish/Orange
        'Fault\n(2 Agents)': palette[3]  # Redish
    }

    # --- PLOT 1: DURATION (Efficiency) ---
    sns.barplot(
        data=df, x='Condition', y='duration', ax=axes[0],
        palette=colors, capsize=0.1, errorbar=('ci', 95)
    )
    axes[0].set_title("Mission Efficiency (Duration)", fontweight='bold', pad=20, fontsize=28)
    axes[0].set_ylabel("Time to Completion (s)", fontsize=24)
    axes[0].set_xlabel("")
    axes[0].tick_params(axis='x', labelsize=22)
    axes[0].tick_params(axis='y', labelsize=22)
    
    # Annotate mean values
    means = df.groupby('Condition')['duration'].mean()
    for i, cat in enumerate(df['Condition'].cat.categories):
        val = means[cat]
        # Larger annotation text
        axes[0].text(i, val + 20, f"{val:.0f}s", ha='center', color='black', weight='bold', fontsize=22)

    # --- PLOT 2: GINI INDEX (Equality) ---
    sns.boxplot(
        data=df, x='Condition', y='gini_index', ax=axes[1],
        palette=colors, showfliers=False
    )
    sns.stripplot(
        data=df, x='Condition', y='gini_index', ax=axes[1],
        color=".2", alpha=0.5, jitter=True, size=8
    )
    axes[1].set_title("Workload Distribution", fontweight='bold', pad=20, fontsize=28) # Shortened title
    axes[1].set_ylabel("Gini Coefficient (0 = Equality)", fontsize=24)
    axes[1].set_xlabel("")
    axes[1].tick_params(axis='x', labelsize=22)
    axes[1].tick_params(axis='y', labelsize=22)
    axes[1].axhline(0, color='gray', linestyle='--', linewidth=2)

    # --- PLOT 3: MARKET LIQUIDITY (Bids per Auction) ---
    sns.pointplot(
        data=df, x='Condition', y='bids_per_auction', ax=axes[2],
        color=palette[4], markers='o', errorbar='sd', linestyle='none', capsize=0.1, scale=1.5
    )
    sns.barplot(
        data=df, x='Condition', y='bids_per_auction', ax=axes[2],
        palette=colors, alpha=0.6, errorbar=None
    )
    
    axes[2].set_title("Market Liquidity (Avg Bids)", fontweight='bold', pad=20, fontsize=28)
    axes[2].set_ylabel("Avg Bids per Auction", fontsize=24)
    axes[2].set_xlabel("")
    axes[2].tick_params(axis='x', labelsize=22)
    axes[2].tick_params(axis='y', labelsize=22)
    
    # Save
    output_dir = "documentation/plots"
    if not os.path.exists(output_dir): os.makedirs(output_dir)
    
    # Save as PDF 
    save_path = os.path.join(output_dir, "combined_results_paper.pdf")
    fig.savefig(save_path, bbox_inches='tight', format='pdf')
    print(f"Plot saved to {save_path}")

def main():
    print("Generating publication-quality plots (24pt Font)...")
    df = load_and_prep_data()
    if df is not None:
        generate_combined_plot(df)

if __name__ == "__main__":
    main()
