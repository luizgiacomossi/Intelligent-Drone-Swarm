import csv
import math
import os

# --- SVG PLOTTING LIBRARY ---
class SVGPlotter:
    def __init__(self, width=600, height=400):
        self.width = width
        self.height = height
        self.padding = 60
        self.elements = []
        
    def add_rect(self, x, y, w, h, color, opacity=1.0, stroke="none"):
        self.elements.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" fill="{color}" fill-opacity="{opacity}" stroke="{stroke}" />')
        
    def add_text(self, x, y, text, size=12, anchor="middle", weight="normal", color="#333", rotate=0):
        transform = f'transform="rotate({rotate} {x} {y})"' if rotate else ""
        self.elements.append(f'<text x="{x}" y="{y}" font-family="Arial, sans-serif" font-size="{size}" font-weight="{weight}" text-anchor="{anchor}" fill="{color}" {transform}>{text}</text>')
        
    def add_line(self, x1, y1, x2, y2, color="#ccc", width=1, dash=""):
        stroke_dash = f'stroke-dasharray="{dash}"' if dash else ""
        self.elements.append(f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" stroke="{color}" stroke-width="{width}" {stroke_dash} />')
        
    def add_circle(self, cx, cy, r, color, opacity=1.0):
        self.elements.append(f'<circle cx="{cx}" cy="{cy}" r="{r}" fill="{color}" fill-opacity="{opacity}" />')

    def save(self, filename):
        out_dir = "documentation/plots"
        if not os.path.exists(out_dir): os.makedirs(out_dir)
        path = os.path.join(out_dir, filename)
        with open(path, "w") as f:
            f.write(f'<svg xmlns="http://www.w3.org/2000/svg" width="{self.width}" height="{self.height}" viewBox="0 0 {self.width} {self.height}">\n')
            f.write(f'<rect width="100%" height="100%" fill="white" />\n')
            for el in self.elements: f.write(el + "\n")
            f.write('</svg>')
        print(f"Saved {path}")

def scale(val, min_v, max_v, range_min, range_max):
    return range_min + (val - min_v) * (range_max - range_min) / (max_v - min_v)

# --- DATA LOADING ---
def load_data():
    data = []
    with open("experiment_results_paper.csv", 'r') as f:
        reader = csv.reader(f)
        headers = next(reader)
        for row in reader:
            if not row: continue
            item = {}
            for i, val in enumerate(row):
                try: item[headers[i]] = float(val)
                except: item[headers[i]] = val
            data.append(item)
    return data

# --- PLOT FUNCTIONS ---

def plot_bar_chart(data_dict, title, ylabel, filename, colors=None):
    # data_dict: { 'Label': {'mean': 10, 'std': 2} }
    svg = SVGPlotter(600, 450)
    p = svg.padding
    W = svg.width - 2*p
    H = svg.height - 2*p
    
    # Title
    svg.add_text(svg.width/2, 30, title, size=16, weight="bold")
    
    # Y-Axis Range
    vals = [v['mean'] + v['std'] for v in data_dict.values()]
    max_y = max(vals) * 1.1
    
    # Grid & Y-Axis
    ticks = 5
    for i in range(ticks + 1):
        val = (max_y / ticks) * i
        y_pos = (svg.height - p) - scale(val, 0, max_y, 0, H)
        svg.add_line(p, y_pos, svg.width - p, y_pos, dash="4,4")
        svg.add_text(p - 10, y_pos + 5, f"{val:.1f}", anchor="end", size=10, color="#666")
    
    svg.add_text(p/2, svg.height/2, ylabel, rotate=-90, anchor="middle", size=12, color="#444")

    # Bars
    labels = list(data_dict.keys())
    bar_width = (W / len(labels)) * 0.6
    spacing = W / len(labels)
    
    default_colors = ["#5DADE2", "#58D68D", "#F4D03F", "#E74C3C", "#AF7AC5"]
    
    for i, label in enumerate(labels):
        stats = data_dict[label]
        mean = stats['mean']
        std = stats['std']
        
        cx = p + (i * spacing) + (spacing/2)
        bar_h = scale(mean, 0, max_y, 0, H)
        y_top = (svg.height - p) - bar_h
        
        col = colors[i] if colors and i < len(colors) else default_colors[i % len(default_colors)]
        
        # Bar
        svg.add_rect(cx - bar_width/2, y_top, bar_width, bar_h, col, opacity=0.8)
        
        # Error Bar
        err_h = scale(std, 0, max_y, 0, H)
        # svg.add_line(cx, y_top - err_h, cx, y_top + err_h, color="#333", width=2)
        # Cap
        # svg.add_line(cx - 5, y_top - err_h, cx + 5, y_top - err_h, color="#333", width=2)
        # svg.add_line(cx - 5, y_top + err_h, cx + 5, y_top + err_h, color="#333", width=2)
        
        # Label
        svg.add_text(cx, svg.height - p + 20, label, size=11)
        
        # Value Label
        svg.add_text(cx, y_top - 5, f"{mean:.1f}", size=10, weight="bold")

    svg.save(filename)

# --- SPECIFIC PLOTS ---

def generate_scalability_plot(data):
    # Scalability Scenario
    subset = [d for d in data if d['scenario'] == 'Scalability']
    
    res = {}
    for n in sorted(list(set([d['num_agents'] for d in subset]))):
        vals = [d['duration'] for d in subset if d['num_agents'] == n]
        res[f"N={int(n)}"] = {'mean': sum(vals)/len(vals), 'std': 0} # Simple mean for clarity
        
    plot_bar_chart(res, "Effect of Swarm Size on Search Time", "Duration (s)", "scalability_duration.svg", ["#A9CCE3", "#5DADE2", "#2E86C1"])

def generate_fault_plot(data):
    # Fault comparison (Baseline N=8 vs Faults)
    baseline_vals = [d['duration'] for d in data if d['scenario'] == 'Scalability' and d['num_agents'] == 8]
    fault1_vals = [d['duration'] for d in data if d['scenario_tag'] == 'Fault_1']
    fault2_vals = [d['duration'] for d in data if d['scenario_tag'] == 'Fault_2']
    
    res = {
        "Baseline (N=8)": {'mean': sum(baseline_vals)/len(baseline_vals), 'std': 0},
        "1 Fault": {'mean': sum(fault1_vals)/len(fault1_vals), 'std': 0},
        "2 Faults": {'mean': sum(fault2_vals)/len(fault2_vals), 'std': 0}
    }
    
    plot_bar_chart(res, "System Resilience: Fault Impact", "Duration (s)", "fault_tolerance.svg", ["#2ecc71", "#f1c40f", "#e74c3c"])

def generate_gini_plot(data):
    # Gini Coeff
    res = {}
    
    # N=2, 4, 8
    for n in [2, 4, 8]:
        vals = [d['gini_index'] for d in data if d['scenario'] == 'Scalability' and d['num_agents'] == n]
        res[f"N={n}"] = {'mean': sum(vals)/len(vals), 'std': 0}
        
    f1 = [d['gini_index'] for d in data if d['scenario_tag'] == 'Fault_1']
    f2 = [d['gini_index'] for d in data if d['scenario_tag'] == 'Fault_2']
    
    res["Fault 1"] = {'mean': sum(f1)/len(f1), 'std': 0}
    res["Fault 2"] = {'mean': sum(f2)/len(f2), 'std': 0}
    
    plot_bar_chart(res, "Workload Inequality (Gini Index)", "Gini Coefficient", "gini_index.svg", ["#abebc6", "#abebc6", "#abebc6", "#fad7a0", "#f5b7b1"])

def main():
    if not os.path.exists("experiment_results_paper.csv"): return
    data = load_data()
    generate_scalability_plot(data)
    generate_fault_plot(data)
    generate_gini_plot(data)

if __name__ == "__main__":
    main()
