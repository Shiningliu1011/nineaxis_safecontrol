import matplotlib.pyplot as plt

from .pipe_painter import pipe_painter


def run_pipe_painter(model=None, options=None):
    app = pipe_painter(model, options)
    if app.options["Visible"] == "on":
        plt.show()
    return app
