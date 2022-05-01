using Plots
plotlyjs()

include("boids.jl")

element = Element()




# function step!(elem::Element)
#     dx = rand(1:5)
#     dy = rand(1:5)
#     dz = rand(1:5)
#     elem.x += dx * 0.01
#     elem.y += dy * 0.01
#     elem.z += dz * 0.01
#     Point3f(elem.x, elem.y, elem.z)
# end

# element = Element()

# points = Observable(Point3f[])
# colors = Observable(Int[])

# for i in 1:100
#     meshscatter(step!(element), markersize = 0.1)
#     display()
# end