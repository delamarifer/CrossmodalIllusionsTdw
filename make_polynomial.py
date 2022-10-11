from dis import dis
import numpy as np
import plotly.graph_objects as go
from sympy import plot
a = -0.0142635
b = 0.35218
c = -2.7433
d = 7.03816
f = -0.737243

def make_poly(x):
    y = a*(x**4) + b*(x**3) + c*(x**2) + d*(x**1) + f
    return y



def plot_poly(x,y):
    fig = go.Figure(data=go.Scatter(x=x, y=y))
    fig.show()

def get_poly_velocity(path_len, discont_len):
    a = -0.0142635
    b = 0.35218
    c = -2.7433
    d = 7.03816
    f = -0.737243
    x = np.linspace(0,12.35,path_len)
    y = a*(x**4) + b*(x**3) + c*(x**2) + d*(x**1) + f
    initpos = int(path_len/2-5)
    endpos = int(path_len/2+5)
    mid_arr = np.repeat([0],discont_len)
    y = 0.35*y
    new_y = np.hstack(( y,mid_arr,y)).ravel()    
    return [x for x in range(len(new_y))], new_y


def get_poly_velocity2(path_len,discont_len):
    a = 3.4936*(10**-7)
    b = -0.0000258902
    c = 0.000796324
    d = -0.0131828
    f = 0.12722
    g = -0.72374
    h = 2.33763
    i = -3.97475
    j = 5.5485 
    k = -0.0490286
    if discont_len == 0:
        x = np.linspace(0.1,17.4,path_len)
        y = a*(x**9) + b*(x**8) + c*(x**7) + d*(x**6) + f*(x**5) + g*(x**4) + h*(x**3) + i*(x**2) + j*(x**1) + k
        y = 0.07*y
        new_y = -0.7*y+1.5
    else:
        x = np.linspace(0.1,17.4,path_len)
        y = a*(x**9) + b*(x**8) + c*(x**7) + d*(x**6) + f*(x**5) + g*(x**4) + h*(x**3) + i*(x**2) + j*(x**1) + k
        mid_arr = np.repeat([0.0001],discont_len)
        y = 0.07*y
        new_y = np.hstack(( y,mid_arr,y)).ravel()    
    return  new_y
   


# y = get_poly_velocity2(60,0)
# x = [x for x in range(len(y))]
# plot_poly(x,-0.7*y+1.5)

# print(np.size(y))

# [print(yx) for yx in y]