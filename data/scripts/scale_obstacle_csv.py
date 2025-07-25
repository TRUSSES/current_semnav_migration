# Generate new CSV with x, y points of polygons scaled up by kx and ky
import csv
import pandas as pd

input_filename = '../mass_10.0_1.5x1m.csv'
output_filename = '../mass_10.0_1.5x1m_rounded.csv'
rotate_clockwise = False

kx = 0.25    # x-axis scale factor
ky = 0.25   # y-axis scale factor

original_width = 600
original_height = 400
new_width = original_width * kx
new_height = original_height * ky

df = pd.read_csv(input_filename, header=None, dtype=float)

x = df.iloc[0] * kx
y = df.iloc[1] * ky

# Rotate coordinates clockwise 90 degrees, skips NaN
if rotate_clockwise:
    new_x = y.copy()
    new_y = new_width - x
    
    new_df = pd.DataFrame([new_x, new_y])
else:
    new_df = pd.DataFrame([x, y])

new_df.to_csv(output_filename, index=False, header=False)