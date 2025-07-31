import tkinter as tk
from tkinter import simpledialog, filedialog 
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.transforms as transforms
import pandas as pd
from scipy.stats import multivariate_normal

class GaussianEllipseGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Define 2D Gaussians via Clicks")

        self.fig, self.ax = plt.subplots()
        self.ax.set_title("Click center, major axis point, minor axis point")
        

        self.canvas = FigureCanvasTkAgg(self.fig, master=root)
        self.canvas_widget = self.canvas.get_tk_widget()
        self.canvas_widget.pack()

        self.canvas.mpl_connect("button_press_event", self.on_click)

        self.click_stage = 0
        self.center = None
        self.major_point = None
        self.minor_point = None

        self.gaussians = []
        
        axis_frame = tk.Frame(root)
        axis_frame.pack(pady=5)

        tk.Label(axis_frame, text="X min").grid(row=0, column=0)
        self.x_min = tk.DoubleVar(value=-10)
        tk.Entry(axis_frame, textvariable=self.x_min, width=6).grid(row=0, column=1)

        tk.Label(axis_frame, text="X max").grid(row=0, column=2)
        self.x_max = tk.DoubleVar(value=10)
        tk.Entry(axis_frame, textvariable=self.x_max, width=6).grid(row=0, column=3)

        tk.Label(axis_frame, text="Y min").grid(row=0, column=4)
        self.y_min = tk.DoubleVar(value=-10)
        tk.Entry(axis_frame, textvariable=self.y_min, width=6).grid(row=0, column=5)

        tk.Label(axis_frame, text="Y max").grid(row=0, column=6)
        self.y_max = tk.DoubleVar(value=10)
        tk.Entry(axis_frame, textvariable=self.y_max, width=6).grid(row=0, column=7)

        tk.Button(axis_frame, text="Update Axes", command=self.update_axes).grid(row=0, column=8, padx=10)
        self.update_axes()

        grid_frame = tk.Frame(root)
        grid_frame.pack()

        tk.Label(grid_frame, text="x step").grid(row=0, column=0)
        self.x_step = tk.DoubleVar(value=0.2)
        tk.Entry(grid_frame, textvariable=self.x_step, width=6).grid(row=0, column=1)

        tk.Label(grid_frame, text="y step").grid(row=0, column=2)
        self.y_step = tk.DoubleVar(value=0.2)
        tk.Entry(grid_frame, textvariable=self.y_step, width=6).grid(row=0, column=3)

        tk.Label(grid_frame, text="map base val").grid(row=0, column=4)
        self.base_val = tk.DoubleVar(value=1e7)
        tk.Entry(grid_frame, textvariable=self.base_val, width=6).grid(row=0, column=5)

        button_frame = tk.Frame(root)
        button_frame.pack(pady=5)

        tk.Button(button_frame, text="Undo Last Gaussian", command=self.undo_last_gaussian).pack(side="left", padx=5)
        tk.Button(button_frame, text="Save Samples to CSV", command=self.save_samples).pack(side="left", padx=5)


    def update_axes(self):
        self.ax.set_xlim(self.x_min.get(), self.x_max.get())
        self.ax.set_ylim(self.y_min.get(), self.y_max.get())
        self.ax.grid(True)
        self.canvas.draw()

    def on_click(self, event):
        if event.inaxes != self.ax:
            return

        x, y = event.xdata, event.ydata

        if self.click_stage == 0:
            self.center = np.array([x, y])
            self.ax.plot(x, y, 'ro')
            self.ax.set_title("Click to define major axis")
            self.click_stage = 1

        elif self.click_stage == 1:
            self.major_point = np.array([x, y])
            self.ax.plot(x, y, 'go')
            self.ax.set_title("Click to define minor axis")
            self.click_stage = 2

        elif self.click_stage == 2:
            self.minor_point = np.array([x, y])
            self.ax.plot(x, y, 'bo')
            self.click_stage = 0
            self.ax.set_title("Click center, major axis point, minor axis point")

            self.prompt_and_draw_gaussian()

        self.canvas.draw()

    def prompt_and_draw_gaussian(self):
        mean_value = simpledialog.askfloat("Mean Value", "Enter mean/intensity value for this Gaussian:")
        
        if mean_value is None:
            return

        # Major axis vector and length
        major_vec = self.major_point - self.center
        major_len = np.linalg.norm(major_vec)
        angle = np.arctan2(major_vec[1], major_vec[0])  # in radians

        # Minor axis vector and length
        minor_vec = self.minor_point - self.center
        minor_len = np.linalg.norm(minor_vec)

        # Covariance matrix from ellipse parameters
        R = np.array([[np.cos(angle), -np.sin(angle)],
                      [np.sin(angle),  np.cos(angle)]])
        D = np.diag([major_len**2, minor_len**2])
        cov = R @ D @ R.T

        # Draw ellipse
        ellipse = Ellipse(xy=self.center, width=2*major_len, height=2*minor_len,
                          angle=np.degrees(angle), edgecolor='blue', facecolor='none', lw=2)
        self.ax.add_patch(ellipse)

        self.gaussians.append({
            "mean": self.center,
            "cov": cov,
            "value": mean_value
        })

        self.canvas.draw()

    def undo_last_gaussian(self):
        if not self.gaussians:
            print("No Gaussians to remove.")
            return

        self.gaussians.pop()

        # Redraw everything from scratch (recommended for simplicity)
        self.ax.clear()
        self.ax.set_title("Click center, major axis point, minor axis point")
        self.ax.set_xlim(self.x_min.get(), self.x_max.get())
        self.ax.set_ylim(self.y_min.get(), self.y_max.get())
        self.ax.grid(True)

        for g in self.gaussians:
            mean = g["mean"]
            cov = g["cov"]

            # Extract ellipse properties from covariance
            eigvals, eigvecs = np.linalg.eigh(cov)
            order = eigvals.argsort()[::-1]
            eigvals, eigvecs = eigvals[order], eigvecs[:, order]

            angle = np.degrees(np.arctan2(eigvecs[1, 0], eigvecs[0, 0]))
            width, height = 2 * np.sqrt(eigvals)

            ellipse = Ellipse(xy=mean, width=width, height=height,
                            angle=angle, edgecolor='blue', facecolor='none', lw=2)
            self.ax.add_patch(ellipse)
            self.ax.plot(mean[0], mean[1], 'ro')  # plot mean

        self.canvas.draw()



    def save_samples(self):
        if not self.gaussians:
            print("No Gaussians defined.")
            return

        filepath = filedialog.asksaveasfilename(defaultextension=".csv",
                                                filetypes=[("CSV files", "*.csv")],
                                                title="Save grid samples to CSV")
        if not filepath:
            return

        # Grid range from plot limits
        x_min, x_max = self.ax.get_xlim()
        y_min, y_max = self.ax.get_ylim()
        x_step = self.x_step.get()
        y_step = self.y_step.get()
        base_val = self.base_val.get()

        x_vals = np.arange(x_min, x_max, x_step)
        y_vals = np.arange(y_min, y_max, y_step)
        xx, yy = np.meshgrid(x_vals, y_vals)
        grid_points = np.column_stack([xx.ravel(), yy.ravel()])

        total_value = base_val*np.ones(len(grid_points))

        for g in self.gaussians:
            mvn = multivariate_normal(mean=g["mean"], cov=g["cov"])
            density = mvn.pdf(grid_points)
            weighted = density * g["value"]
            total_value += weighted

        # Save to CSV
        df = pd.DataFrame({
            "x": grid_points[:, 0],
            "y": grid_points[:, 1],
            "stiffness": total_value
        })
        df.to_csv(filepath, index=False)
        print(f"Saved grid of shape ({len(y_vals)} x {len(x_vals)}) = {len(df)} points to {filepath}")

if __name__ == "__main__":
    root = tk.Tk()
    app = GaussianEllipseGUI(root)
    root.mainloop()
