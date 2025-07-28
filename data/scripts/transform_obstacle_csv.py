# Generate new CSV with x, y points of polygons scaled up by kx and ky
import csv
import pandas as pd

input_filename = '../mass_5.0_6x4m.csv'
output_filename = '../mass_5.0_4x6m.csv'

df = pd.read_csv(input_filename, header=None, dtype=float)

width = df.iloc[0].max()
height = df.iloc[1].max()

x = df.iloc[0]
y = df.iloc[1]

def rotate_ccw():
    # Assumes that x and y are coordinate rows.
    global x, y, width, height
    new_y = x.copy()
    new_x = height - y

    tmp = height
    height = width
    width = tmp

    x = new_x
    y = new_y

def scale(kx, ky):
    global x, y, width, height
    x = x * kx
    y = y * ky
    width = width * kx
    height = height * ky

def center():
    global x, y
    center_x = (x.max() - x.min()) / 2
    center_y = (y.max() - y.min()) / 2
    x = x - center_x
    y = y - center_y

rotate_ccw()
center()
new_df = pd.DataFrame([x, y])

new_df.to_csv(output_filename, index=False, header=False)