import csv
import matplotlib.pyplot as plt
import numpy as np

# Read CSV file with three columns (no header) in order: Captură, Procesare, Servo
capt = []
proc = []
servo = []
with open('results/response_detailed.csv', 'r') as f:
    reader = csv.reader(f)
    for row in reader:
        if not row:
            continue
        # expect at least 3 columns; skip rows that don't match
        try:
            c = float(row[0])
            p = float(row[1])
            s = float(row[2])
        except (ValueError, IndexError):
            continue
        capt.append(c)
        proc.append(p)
        servo.append(s)

capt = np.array(capt)
proc = np.array(proc)
servo = np.array(servo)

def stats(arr):
    return np.mean(arr), np.median(arr), np.std(arr)

# Compute and print statistics for each column
capt_mean, capt_med, capt_std = stats(capt)
proc_mean, proc_med, proc_std = stats(proc)
servo_mean, servo_med, servo_std = stats(servo)

print('Statistics (ms):')
print(f"Captură - Mean: {capt_mean:.3f}, Median: {capt_med:.3f}, Std: {capt_std:.3f}")
print(f"Procesare - Mean: {proc_mean:.3f}, Median: {proc_med:.3f}, Std: {proc_std:.3f}")
print(f"Servo    - Mean: {servo_mean:.3f}, Median: {servo_med:.3f}, Std: {servo_std:.3f}")

# Also compute stats for the sum of the three columns
sum_arr = capt + proc + servo
sum_mean, sum_med, sum_std = stats(sum_arr)
print(f"Sum      - Mean: {sum_mean:.3f}, Median: {sum_med:.3f}, Std: {sum_std:.3f}")

# Plot 1: three columns on same graph (no mean/median lines)
plt.figure(figsize=(10, 6))
events = np.arange(len(capt))
plt.plot(events, capt, marker='o', linestyle='-', linewidth=1, markersize=4, label='Captură')
plt.plot(events, proc, marker='o', linestyle='-', linewidth=1, markersize=4, label='Procesare')
plt.plot(events, servo, marker='o', linestyle='-', linewidth=1, markersize=4, label='Servo')
plt.xlabel('Eveniment de mișcare')
plt.ylabel('Latență (ms)')
plt.title('Componente latență')
plt.legend()
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig('results/response_times_components.png')
plt.show()

# Plot 2: sum of the three columns, with mean and median lines (like original)
plt.figure(figsize=(10, 6))
plt.plot(sum_arr, marker='o', linestyle='-', linewidth=1, markersize=4)
plt.axhline(y=sum_mean, color='r', linestyle='--', label=f'Medie: {sum_mean:.2f}')
plt.axhline(y=sum_med, color='g', linestyle='--', label=f'Mediană: {sum_med:.2f}')
plt.xlabel('Eveniment de mișcare')
plt.ylabel('Latență (ms)')
plt.title('Latența totală')
plt.legend()
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig('results/response_times_sum.png')
plt.show()
