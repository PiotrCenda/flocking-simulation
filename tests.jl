using GLMakie

init = randn(3) 
points = Observable(Point3f(init))

fig, ax, scat = scatter(points, axis = (;type=Axis3), markersize = 3000)
limits!(ax, -4, 4, -4, 4, -4, 4)

display(fig)

fps = 60
nframes = 1000

function point_step(point)
    d = randn(3) .* 0.1
    point = point .+ d
    return point[1], point[2], point[3]
end

function animation_step(init, point)
    x, y, z = point_step(init)
    point = Point3f(x, y, z)
end

for i = 1:nframes
    # points[] = push!(points[], new_point)
    animation_step(init, points)
    sleep(1/fps) # refreshes the display!
end
