using GLMakie, LinearAlgebra

# Include file with movement algorithm
include("boids.jl")

# Make boids
axis_limit = 1500;
generation_area = 300
numBoids = 400
flock = [Boid(i, rand(-generation_area:generation_area, 3), randn(3).*3, randn(3).*2) for i=1:numBoids]
points = Observable([Point3f(boid.position) for boid in flock])

# Create figure
fig = Figure()

# Create 3d axis
ax = Axis3(fig[1, 1], aspect = (1, 1, 1), title = "Flocking simulation")
ax.perspectiveness = 0.3

# Set axis limits
limits!(ax, -axis_limit, axis_limit, -axis_limit, axis_limit, -axis_limit, axis_limit)

# Create sliders layout
lsgrid = labelslidergrid!(fig,
    ["align", "separation", "cohesion", "align perception", "separation perception", "cohesion perception"],
    [LinRange(0.1:0.01:2), LinRange(0.1:0.01:2), LinRange(0.1:0.01:2), LinRange(30:1:300), LinRange(30:1:300), LinRange(30:1:300)];
    formats = [x -> "$(round(x, digits = 2))"],
    labelkw = Dict([(:textsize, 20)]),
    sliderkw = Dict([(:linewidth, 10)]),
    valuekw = Dict([(:textsize, 20)])
)

# Create sliders grid
sl_sublayout = GridLayout(height=30)
fig[2, 1] = sl_sublayout
fig[2, 1] = lsgrid.layout

# Set initial parameters
set_close_to!(lsgrid.sliders[1], 0.7)
set_close_to!(lsgrid.sliders[2], 1.1)
set_close_to!(lsgrid.sliders[3], 0.65)
set_close_to!(lsgrid.sliders[4], 150)
set_close_to!(lsgrid.sliders[5], 100)
set_close_to!(lsgrid.sliders[6], 230)

# Update parameters values
alignValue = lsgrid.sliders[1].value
separationValue = lsgrid.sliders[2].value
cohesionValue = lsgrid.sliders[3].value
align_perception_radius = lsgrid.sliders[4].value
separate_perception_radius = lsgrid.sliders[5].value
cohese_perception_radius = lsgrid.sliders[6].value

# Create buttons grid
fig[3, 1] = buttongrid = GridLayout(height=100)

labels = ["Run / Pause", "Step", "Restart"]
button_count = 3

# Create buttons
buttons = buttongrid[3, 1:button_count] = [
    Button(fig, label=l, height=35, width=150, textsize=20)
    for l in labels
]

# Run button
isrunning = Observable(false)
on(buttons[1].clicks) do clicks; isrunning[] = !isrunning[]; end

on(buttons[1].clicks) do clicks
    @async while isrunning[]
        for boid in flock
            fly!(boid, flock, alignValue[], separationValue[], cohesionValue[], axis_limit[], align_perception_radius[], separate_perception_radius[], cohese_perception_radius[])
        end
        
        points[] = [Point3f(boid.position) for boid in flock]
        sleep(0.01)
    end
end

# Step button
on(buttons[2].clicks) do click
    for boid in flock
        fly!(boid, flock, alignValue[], separationValue[], cohesionValue[], axis_limit[], align_perception_radius[], separate_perception_radius[], cohese_perception_radius[])
    end
    
    points[] = [Point3f(boid.position) for boid in flock]
    sleep(0.01)
end

# Reset button
on(buttons[3].clicks) do click
    for boid in flock
        boid.position = rand(-generation_area:generation_area, 3)
    end
    points[] = [Point3f(boid.position) for boid in flock]
    sleep(0.01)
end

scatter!(ax, points, markersize=20, color=:black)

window = display(fig)
wait(window)
