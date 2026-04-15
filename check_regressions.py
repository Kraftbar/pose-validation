import subprocess
import json
import sys
from pathlib import Path

# Baseline ATE for 5-second runs of pure_c_plus (approximate)
BASELINES = {
    "test_freiburgxyz525": 0.1465,
    "test_freiburgrpy525": 0.0880,
    "test_freiburgroom525": 1.2500,
    "test_freiburgdesk525": 0.6500,
}

def main():
    print("Running fast regression check (5s sweep)...")
    cmd = [
        sys.executable, "benchmark_native.py",
        "--all_gt",
        "--seconds", "5",
        "--force"
    ]
    
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        print("Benchmark failed!")
        print(result.stderr)
        sys.exit(1)
        
    summary_path = Path("runs/benchmark/summary_all_5s.json")
    if not summary_path.exists():
        print(f"Error: {summary_path} not found.")
        sys.exit(1)
        
    with open(summary_path, "r") as f:
        data = json.load(f)
        
    regressions = 0
    print(f"\n{'Sequence':<25} {'Current ATE':>12} {'Baseline':>12} {'Delta':>10}")
    print("-" * 65)
    
    # We only care about pure_c_plus in this check
    for row in data:
        if row["impl"] != "pure_c_plus":
            continue
            
        video = row["video"]
        ate = row.get("ate_rmse")
        if ate is None:
            print(f"{video:<25} {'FAILED':>12}")
            regressions += 1
            continue
            
        baseline = BASELINES.get(video, 999.0)
        delta = ate - baseline
        
        status = ""
        if delta > 0.02:
            status = "⚠️ REGRESSION"
            regressions += 1
        elif delta < -0.02:
            status = "🚀 IMPROVEMENT"
            
        print(f"{video:<25} {ate:12.4f} {baseline:12.4f} {delta:+10.4f} {status}")

    if regressions > 0:
        print(f"\nFound {regressions} regressions. Use tools/plot_frame_errors.py to diagnose.")
        sys.exit(1)
    else:
        print("\nNo regressions found. implementations are stable.")

if __name__ == "__main__":
    main()
