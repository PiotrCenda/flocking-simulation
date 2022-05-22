using InteractiveChaos
using DynamicalSystems, GLMakie
using OrdinaryDiffEq

ds = Systems.henonheiles()  # 4D chaotic/regular continuous system

u0s = [[0.0, -0.25, 0.42081, 0.0],
[0.0, 0.1, 0.5, 0.0],
[0.0, -0.31596, 0.354461, 0.0591255]]

diffeq = (alg = Vern9(), dtmax = 0.01)
idxs = (1, 2, 4)
colors = ["#233B43", "#499cbf", "#E84646"]

figure, obs = interactive_evolution(
    ds, u0s; idxs, tail = 10000, diffeq, colors
)