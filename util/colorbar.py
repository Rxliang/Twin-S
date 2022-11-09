from matplotlib.widgets import Slider
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors

class SqueezedNorm(matplotlib.colors.Normalize):
    def __init__(self, vmin=None, vmax=None, mid=0, s1=2, s2=2, clip=False):
        self.vmin = vmin # minimum value
        self.mid  = mid  # middle value
        self.vmax = vmax # maximum value
        self.s1=s1; self.s2=s2
        f = lambda x, zero,vmax,s: np.abs((x-zero)/(vmax-zero))**(1./s)*0.5
        self.g = lambda x, zero,vmin,vmax, s1,s2: f(x,zero,vmax,s1)*(x>=zero) - \
                                             f(x,zero,vmin,s2)*(x<zero)+0.5
        matplotlib.colors.Normalize.__init__(self, vmin, vmax, clip)

    def __call__(self, value, clip=None):
        r = self.g(value, self.mid,self.vmin,self.vmax, self.s1,self.s2)
        return np.ma.masked_array(r)


fig, (ax, ax2, ax3) = plt.subplots(nrows=3, 
                                   gridspec_kw={"height_ratios":[3,2,1], "hspace":0.25})

x = np.linspace(0,0.26, 110)
norm=SqueezedNorm(vmin=0, vmax=0.26, mid=0, s1=1.7, s2=4)

line, = ax.plot(x, norm(x))
ax.margins(0)
ax.set_ylim(0,1)

im = ax2.imshow(np.atleast_2d(x).T, cmap="Spectral_r", norm=norm, aspect="auto")
cbar = fig.colorbar(im ,cax=ax3,ax=ax2, orientation="horizontal")
midax = plt.axes([0.1, 0.04, 0.2, 0.03], facecolor="lightblue")
s1ax = plt.axes([0.4, 0.04, 0.2, 0.03], facecolor="lightblue")
s2ax = plt.axes([0.7, 0.04, 0.2, 0.03], facecolor="lightblue")

mid = Slider(midax, 'Midpoint', x[0], x[-1], valinit=0)
s1 = Slider(s1ax, 'S1', 0.5, 6, valinit=1.7)
s2 = Slider(s2ax, 'S2', 0.5, 6, valinit=4)


def update(val):
    norm=SqueezedNorm(vmin=0, vmax=0.26, mid=mid.val, s1=s1.val, s2=s2.val)
    im.set_norm(norm)
    cbar.update_bruteforce(im) 
    line.set_ydata(norm(x))
    fig.canvas.draw_idle()

mid.on_changed(update)
s1.on_changed(update)
s2.on_changed(update)

fig.subplots_adjust(bottom=0.15)
plt.show()