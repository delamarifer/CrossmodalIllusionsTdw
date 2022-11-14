import numpy as np

import plotly.graph_objects as go
from sympy import plot

path_len =60
zstart = 0
list_pos = 0.01* np.exp(np.linspace(0.9,0.2,100))
print(len(list_pos[:-1]))

for p in list_pos:
    print(p)

def plot_poly(x,y):
    fig = go.Figure(data=go.Scatter(x=x, y=y))
    fig.show()
plot_poly([x for x in range(len(list_pos))],list_pos)
